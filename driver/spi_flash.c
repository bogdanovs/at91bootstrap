// Copyright (C) 2015 Microchip Technology Inc. and its subsidiaries
//
// SPDX-License-Identifier: MIT

#include "common.h"
#include "hardware.h"
#include "board.h"
#include "spi.h"
#include "arch/at91_pio.h"
#include "gpio.h"
#include "string.h"
#include "timer.h"
#include "div.h"
#include "fdt.h"
#include "debug.h"

/* Manufacturer Device ID Read */
#define CMD_READ_DEV_ID			0x9f
/* Continuous Array Read */
#define CMD_READ_ARRAY_FAST		0x0b
#define CMD_READ_ARRAY		0x03

/* JEDEC Code */
#define MANUFACTURER_ID_ATMEL		0x1f
#define MANUFACTURER_ID_MICRON		0x20
#define MANUFACTURER_ID_WINBOND		0xef

/* Family Code */
#define DF_FAMILY_AT26F			0x00
#define DF_FAMILY_AT45			0x20
#define DF_FAMILY_AT26DF		0x40	/* AT25DF and AT26DF */

#define DF_FAMILY_N25Q			0xA0
#define DF_FAMILY_M25P			0x20

/* AT45 Density Code */
#define DENSITY_AT45DB011D		0x0C
#define DENSITY_AT45DB021D		0x14
#define DENSITY_AT45DB041D		0x1C
#define DENSITY_AT45DB081D		0x24
#define DENSITY_AT45DB161D		0x2C
#define DENSITY_AT45DB321D		0x34
#define DENSITY_AT45DB642D		0x3C
#define DENSITY_AT45DB1282D		0x10
#define DENSITY_AT45DB2562D		0x18
#define DENSITY_AT45DB5122D		0x20

/* Winbond W25 */
#define WINBOND_W25Q128JV		0x40

/* AT45 Status Register Read */
#define CMD_READ_STATUS_AT45		0xd7

/* AT45 status register bits */
#define STATUS_PAGE_SIZE_AT45		(1 << 0)
#define STATUS_READY_AT45		(1 << 7)

struct dataflash_descriptor {
	unsigned char	family;

	unsigned int	pages;		/* page number */
	unsigned int	page_size;	/* page size */
	unsigned int	page_offset;	/* page offset in command */
	unsigned char	is_power_2;	/* = 1: power of 2, = 0: not*/
	unsigned char	is_spinor;	/* = 1: nor flash, = 0: dataflash */
};

static int df_send_command(unsigned char *cmd,
				unsigned char cmd_len,
				unsigned char *data,
				unsigned int data_len)
{
	int i;

	if (!cmd)
		return -1;

	if (!cmd_len)
		return -1;

	if (data_len)
		if (!data)
			return -1;

	at91_spi_cs_activate();

	/* read spi status to clear events */
	at91_spi_read_sr();

	for (i = 0; i < cmd_len; i++) {
		at91_spi_write_data(*cmd++);
		at91_spi_read_spi();
	}

	for (i = 0; i < data_len; i++) {
		at91_spi_write_data(0);
		*data++ = at91_spi_read_spi();
	}
	at91_spi_cs_deactivate();

	return 0;
}

static int dataflash_read_array(struct dataflash_descriptor *df_desc,
				unsigned int offset,
				unsigned int len,
				void *buf)
{
	unsigned char cmd[5];
	unsigned char cmd_len;
	unsigned int address;
	unsigned int page_addr = 0;
	unsigned int byte_addr = 0;
	unsigned int page_shift;
	unsigned int page_size;
	int ret;

	if (!df_desc->is_power_2) {
		page_shift = df_desc->page_offset;
		page_size = df_desc->page_size;

		division(offset, page_size, &page_addr, &byte_addr);

		address = (page_addr << page_shift) + byte_addr;
	} else
		address = offset;

	cmd[0] = CMD_READ_ARRAY_FAST;
	if (df_desc->pages > 16384) {
		cmd[1] = (unsigned char)(address >> 24);
		cmd[2] = (unsigned char)(address >> 16);
		cmd[3] = (unsigned char)(address >> 8);
		cmd[4] = (unsigned char)address;

	} else {
		cmd[1] = (unsigned char)(address >> 16);
		cmd[2] = (unsigned char)(address >> 8);
		cmd[3] = (unsigned char)address;
		cmd[4] = 0x00;
	}

	cmd_len = 5;

	ret = df_send_command(cmd, cmd_len, buf, len);
	if (ret)
		return -1;

	return 0;
}

static int spinor_read_array(struct dataflash_descriptor *df_desc,
				unsigned int offset,
				unsigned int len,
				void *buf)
{
	unsigned char cmd[5];
	unsigned char cmd_len;
	unsigned int address;
	int ret;

	address = offset;

	cmd[0] = CMD_READ_ARRAY_FAST;
	cmd[1] = (unsigned char)(address >> 16);
	cmd[2] = (unsigned char)(address >> 8);
	cmd[3] = (unsigned char)address;
	cmd_len = 5; /* 5th command is for dummy cycle*/

	ret = df_send_command(cmd, cmd_len, buf, len);
	if (ret)
		return -1;

	return 0;
}

static int read_array(struct dataflash_descriptor *df_desc,
				unsigned int offset,
				unsigned int len,
				void *buf)
{
	if (!df_desc->is_spinor)
		return dataflash_read_array(df_desc, offset, len, buf);
	else
		return spinor_read_array(df_desc, offset, len, buf);
}

#if defined(CONFIG_LOAD_LINUX) || defined(CONFIG_LOAD_ANDROID)
static int update_image_length(struct dataflash_descriptor *df_desc,
				unsigned int offset,
				unsigned char *dest,
				unsigned char flag)
{
	unsigned int length = df_desc->page_size;
	int ret;

	ret = read_array(df_desc, offset, length, dest);
	if (ret)
		return -1;

	if (flag == KERNEL_IMAGE)
		return kernel_size(dest);
#ifdef CONFIG_OF_LIBFDT
	else {
		ret = check_dt_blob_valid((void *)dest);
		if (!ret)
			return of_get_dt_total_size((void *)dest);
	}
#endif
	return -1;
}
#endif

static unsigned char df_read_status_at45(unsigned char *status)
{
	unsigned char cmd = CMD_READ_STATUS_AT45;
	int ret;

	ret = df_send_command(&cmd, 1, status, 1);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_DATAFLASH_RECOVERY

/* AT25 Block Erase(4-KBytes) Command*/
#define CMD_ERASE_BLOCK4K_AT25		0x20
/* Write Enable Command */
#define CMD_WRITE_ENABLE_AT25		0x06
/* Status Register Commands */
#define CMD_READ_STATUS_AT25		0x05
#define CMD_WRITE_STATUS_AT25           0x01
/* Page Erase AT45 */
#define CMD_ERASE_PAGE_AT45		0x81

/* AT25 status register bits */
#define STATUS_READY_AT25		(1 << 0)
#define STATUS_WEL_AT25			(1 << 1)
#define STATUS_SWP_AT25			(3 << 2)
#define STATUS_EPE_AT25			(1 << 5)
#define STATUS_SPRL_AT25		(1 << 7)

static unsigned char df_read_status_at25(unsigned char *status)
{
	unsigned char cmd = CMD_READ_STATUS_AT25;
	int ret;

	ret = df_send_command(&cmd, 1, status, 1);
	if (ret)
		return ret;

	return 0;
}

static int at25_cmd_write_enbale(void)
{
	unsigned char cmd;
	int ret;

	cmd = CMD_WRITE_ENABLE_AT25;
	ret = df_send_command(&cmd, 1, NULL, 0);
	if (ret)
		return ret;

	return 0;
}

static int at25_cmd_write_status_register(unsigned char status)
{
	unsigned char cmd[2];
	int ret;

	cmd[0] = CMD_WRITE_STATUS_AT25;
	cmd[1] = status;

	ret = df_send_command(cmd, 2, NULL, 0);
	if (ret)
		return ret;

	return 0;
}

static int at25_unprotect(void)
{
	unsigned char status;
	int ret;

	/* read status register */
	ret = df_read_status_at25(&status);
	if (ret)
		return ret;

	/* check if All sectors are software unprotected
	 * (all Sector Protection Register are 0)
	 */
	if (!(status & STATUS_SWP_AT25))
		return 0;

	/* check if Sector Protection Registers are locked */
	if (status & STATUS_SPRL_AT25) {
		/* Unprotect Sector Potection Registers. */
		ret = at25_cmd_write_enbale();
		if (ret)
			return ret;

		ret = at25_cmd_write_status_register(0);
		if (ret)
			return ret;
	}

	/* a global unprotect command */
	ret = at25_cmd_write_enbale();
	if (ret)
		return ret;

	ret = at25_cmd_write_status_register(0);
	if (ret)
		return ret;

	/* check Status Register SPRL & SWP bits */
	ret = df_read_status_at25(&status);
	if (ret)
		return ret;

	if (status & (STATUS_SPRL_AT25 | STATUS_SWP_AT25)) {
		dbg_info("SF: Unprotect AT25 failed\n");
		return -1;
	}

	return 0;
}

static int dataflash_page0_erase_at25(void)
{
	unsigned char status;
	unsigned char cmd[5];
	unsigned int timeout = 1000;
	int ret;

	ret = at25_unprotect();
	if (ret)
		return ret;

	ret = at25_cmd_write_enbale();
	if (ret)
		return ret;

	/* Erase page0 */
	cmd[0] = CMD_ERASE_BLOCK4K_AT25;
	cmd[1] = 0;
	cmd[2] = 0;
	cmd[3] = 0;

	ret = df_send_command(cmd, 4, NULL, 0);
	if (ret) {
		dbg_info("SF: AT25 page 0 erase failed\n");
		return ret;
	}

	udelay(33000); /* 33 ms: the maximum delay of udelay() */

	do {
		ret = df_read_status_at25(&status);
		if (ret)
			return ret;

		if (!(status & STATUS_READY_AT25))
			break;
	} while (--timeout);

	if (!timeout) {
		dbg_info("SF: AT25 page0 erase timed out\n");
		return -1;
	}

	return 0;
}

static int dataflash_page0_erase_at45(void)
{
	unsigned char status;
	unsigned char cmd[4];
	unsigned int timeout = 1000;
	int ret;

	cmd[0] = CMD_ERASE_PAGE_AT45;
	cmd[1] = 0;
	cmd[2] = 0;
	cmd[3] = 0;

	ret = df_send_command(cmd, 4, NULL, 0);
	if (ret) {
		dbg_info("SF: AT45 page 0 erase failed\n");
		return ret;
	}

	udelay(33000); /* 33 ms: the maximum delay of udelay() */

	do {
		ret = df_read_status_at45(&status);
		if (ret)
			return ret;

		if (status & STATUS_READY_AT45)
			break;
	} while (--timeout);

	if (!(status & STATUS_READY_AT45)) {
		dbg_info("SF: AT45 page0 erase timed out\n");
		return -1;
	}

	return 0;
}

static int dataflash_recovery(struct dataflash_descriptor *df_desc)
{
	int ret;

	/*
	 * If Recovery Button is pressed during boot sequence,
	 * erase dataflash page0
	*/
	dbg_info("SF: Press the recovery button (%s) to recovery\n",
			RECOVERY_BUTTON_NAME);

	if ((pio_get_value(CONFIG_SYS_RECOVERY_BUTTON_PIN)) == 0) {
		dbg_info("SF: The recovery button (%s) has been pressed,\n",
				RECOVERY_BUTTON_NAME);
		dbg_info("SF: The page 0 is erasing...\n");

		if ((df_desc->family == DF_FAMILY_AT26F)
			|| (df_desc->family == DF_FAMILY_AT26DF))
			ret = dataflash_page0_erase_at25();
		 else
			ret = dataflash_page0_erase_at45();

		if (ret) {
			dbg_info("SF: The erasing failed\n");
			return ret;
		}
		dbg_info("SF: The erasing is done\n");

		return 0;
	}

	return -1;
}
#endif /* #ifdef CONFIG_DATAFLASH_RECOVERY */

static int w_25q128_desc_init(struct dataflash_descriptor *df_desc)
{
	df_desc->pages = 346;
	df_desc->page_size = 256;
	df_desc->page_offset = 0;
	df_desc->is_spinor = 1;
	return 0;
}

static int df_n25q_desc_init(struct dataflash_descriptor *df_desc)
{
	df_desc->pages = 16384;
	df_desc->page_size = 256;
	df_desc->page_offset = 0;
	df_desc->is_spinor = 1;
	return 0;
}

static int df_at45_desc_init(struct dataflash_descriptor *df_desc)
{
	unsigned char status;
	unsigned char density;
	int ret;

	ret = df_read_status_at45(&status);
	if (ret)
		return ret;

	if (status & STATUS_PAGE_SIZE_AT45)
		df_desc->is_power_2 = 1;
	else
		df_desc->is_power_2 = 0;

	density = status & 0x3c;
	switch (density) {
	case DENSITY_AT45DB011D:
		df_desc->pages = 512;
		df_desc->page_size = 264;
		df_desc->page_offset = 9;
	break;

	case DENSITY_AT45DB021D:
		df_desc->pages = 1024;
		df_desc->page_size = 264;
		df_desc->page_offset = 9;
		break;

	case DENSITY_AT45DB041D:
		df_desc->pages = 2048;
		df_desc->page_size = 264;
		df_desc->page_offset = 9;
		break;

	case DENSITY_AT45DB081D:
		df_desc->pages = 4096;
		df_desc->page_size = 264;
		df_desc->page_offset = 9;
		break;

	case DENSITY_AT45DB161D:
		df_desc->pages = 4096;
		df_desc->page_size = 528;
		df_desc->page_offset = 10;
		break;

	case DENSITY_AT45DB321D:
		df_desc->pages = 8192;
		df_desc->page_size = 528;
		df_desc->page_offset = 10;
		break;

	case DENSITY_AT45DB642D:
		df_desc->pages = 8192;
		df_desc->page_size = 1056;
		df_desc->page_offset = 11;
		break;
/*
	case DENSITY_AT45DB1282D:
		df_desc->pages = 16384;
		df_desc->pages_size = 1056;
		df_desc->page_offset = 11;
		break;

	case DENSITY_AT45DB2562D:
		df_desc->pages = 16384;
		df_desc->page_size = 2112;
		df_desc->page_offset = 12;
		break;

	case DENSITY_AT45DB5122D:
		df_desc->pages = 32768;
		df_desc->page_size = 2112;
		df_desc->page_offset = 12;
		break;
*/
	default:
		return -1;
	}

	return 0;
}

static int df_at25_desc_init(struct dataflash_descriptor *df_desc)
{
	/* AT25DF321 */
	df_desc->is_power_2 = 1;

	df_desc->pages = 16384;
	df_desc->page_size = 256;
	df_desc->page_offset = 0;

	return 0;
}


/* Exclude chip names for SPL to save space */
//#if !CONFIG_IS_ENABLED(SPI_FLASH_TINY)
#define INFO_NAME(_name) .name = _name,
//#else
//#define INFO_NAME(_name)
//#endif

/* Used when the "_ext_id" is two bytes at most */
#define INFO(_name, _jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		INFO_NAME(_name)					\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = (!(_jedec_id) ? 0 : (3 + ((_ext_id) ? 2 : 0))),	\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),

#define INFO6(_name, _jedec_id, _ext_id, _sector_size, _n_sectors, _flags)	\
		INFO_NAME(_name)					\
		.id = {							\
			((_jedec_id) >> 16) & 0xff,			\
			((_jedec_id) >> 8) & 0xff,			\
			(_jedec_id) & 0xff,				\
			((_ext_id) >> 16) & 0xff,			\
			((_ext_id) >> 8) & 0xff,			\
			(_ext_id) & 0xff,				\
			},						\
		.id_len = 6,						\
		.sector_size = (_sector_size),				\
		.n_sectors = (_n_sectors),				\
		.page_size = 256,					\
		.flags = (_flags),


#define SPI_NOR_MAX_ID_LEN	6
#define SPI_NOR_MAX_ADDR_WIDTH	4
#define BIT(b)			(1UL << (b))

struct flash_info {
	char		*name;

	/*
	 * This array stores the ID bytes.
	 * The first three bytes are the JEDIC ID.
	 * JEDEC ID zero means "no ID" (mostly older chips).
	 */
	unsigned char	id[SPI_NOR_MAX_ID_LEN];
	unsigned char		id_len;

	/* The size listed here is what works with SPINOR_OP_SE, which isn't
	 * necessarily called a "sector" by the vendor.
	 */
	unsigned int	sector_size;
	unsigned short		n_sectors;

	unsigned short		page_size;
	unsigned short		addr_width;

	unsigned long		flags;
#define SECT_4K			BIT(0)	/* SPINOR_OP_BE_4K works uniformly */
#define SPI_NOR_NO_ERASE	BIT(1)	/* No erase command needed */
#define SST_WRITE		BIT(2)	/* use SST byte programming */
#define SPI_NOR_NO_FR		BIT(3)	/* Can't do fastread */
#define SECT_4K_PMC		BIT(4)	/* SPINOR_OP_BE_4K_PMC works uniformly */
#define SPI_NOR_DUAL_READ	BIT(5)	/* Flash supports Dual Read */
#define SPI_NOR_QUAD_READ	BIT(6)	/* Flash supports Quad Read */
#define USE_FSR			BIT(7)	/* use flag status register */
#define SPI_NOR_HAS_LOCK	BIT(8)	/* Flash supports lock/unlock via SR */
#define SPI_NOR_HAS_TB		BIT(9)	/*
					 * Flash SR has Top/Bottom (TB) protect
					 * bit. Must be used with
					 * SPI_NOR_HAS_LOCK.
					 */
#define	SPI_S3AN		BIT(10)	/*
					 * Xilinx Spartan 3AN In-System Flash
					 * (MFR cannot be used for probing
					 * because it has the same value as
					 * ATMEL flashes)
					 */
#define SPI_NOR_4B_OPCODES	BIT(11)	/*
					 * Use dedicated 4byte address op codes
					 * to support memory size above 128Mib.
					 */
#define NO_CHIP_ERASE		BIT(12) /* Chip does not support chip erase */
#define SPI_NOR_SKIP_SFDP	BIT(13)	/* Skip parsing of SFDP tables */
#define USE_CLSR		BIT(14)	/* use CLSR command */
#define SPI_NOR_HAS_SST26LOCK	BIT(15)	/* Flash supports lock/unlock via BPR */
#define SPI_NOR_OCTAL_READ	BIT(16)	/* Flash supports Octal Read */
#define UNLOCK_GLOBAL_BLOCK	BIT(17)	/* Unlock global block protection */
#define SECT_4K_ONLY		BIT(18)	/* Use only CMD_ERASE_4K */


};


/* NOTE: double check command sets and memory organization when you add
 * more nor chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 *
 * All newly added entries should describe *hardware* and should use SECT_4K
 * (or SECT_4K_PMC) if hardware supports erasing 4 KiB sectors. For usage
 * scenarios excluding small sectors there is config option that can be
 * disabled: CONFIG_SPI_FLASH_USE_4K_SECTORS.
 * For historical (and compatibility) reasons (before we got above config) some
 * old entries may be missing 4K flag.
 */
// #define CONFIG_SPI_FLASH_ATMEL
// #define CONFIG_SPI_FLASH_EON
// #define CONFIG_SPI_FLASH_GIGADEVICE
// #define CONFIG_SPI_FLASH_ISSI
// #define CONFIG_SPI_FLASH_SST
// #define CONFIG_SPI_FLASH_MACRONIX
// #define CONFIG_SPI_FLASH_STMICRO
// #define CONFIG_SPI_FLASH_SPANSION
// #define CONFIG_SPI_FLASH_WINBOND
// #define CONFIG_SPI_FLASH_XMC

const struct flash_info spi_nor_ids[] = {
#ifdef CONFIG_SPI_FLASH_ATMEL		/* ATMEL */
	/* Atmel -- some are (confusingly) marketed as "DataFlash" */
	{ INFO("at26df321",	0x1f4700, 0, 64 * 1024, 64, SECT_4K) },
	{ INFO("at25df321a",	0x1f4701, 0, 64 * 1024, 64, SECT_4K) },

	{ INFO("at45db011d",	0x1f2200, 0, 64 * 1024,   4, SECT_4K) },
	{ INFO("at45db021d",	0x1f2300, 0, 64 * 1024,   8, SECT_4K) },
	{ INFO("at45db041d",	0x1f2400, 0, 64 * 1024,   8, SECT_4K) },
	{ INFO("at45db081d",	0x1f2500, 0, 64 * 1024,  16, SECT_4K) },
	{ INFO("at45db161d",	0x1f2600, 0, 64 * 1024,  32, SECT_4K) },
	{ INFO("at45db321d",	0x1f2700, 0, 64 * 1024,  64, SECT_4K) },
	{ INFO("at45db641d",	0x1f2800, 0, 64 * 1024, 128, SECT_4K) },
	{ INFO("at25sl321",	0x1f4216, 0, 64 * 1024,  64, SECT_4K) },
	{ INFO("at26df081a", 	0x1f4501, 0, 64 * 1024,  16, SECT_4K) },
#endif
#ifdef CONFIG_SPI_FLASH_EON		/* EON */
	/* EON -- en25xxx */
	{ INFO("en25q32b",   0x1c3016, 0, 64 * 1024,   64, 0) },
	{ INFO("en25q64",    0x1c3017, 0, 64 * 1024,  128, SECT_4K) },
	{ INFO("en25qh128",  0x1c7018, 0, 64 * 1024,  256, 0) },
	{ INFO("en25s64",    0x1c3817, 0, 64 * 1024,  128, SECT_4K) },
#endif
#ifdef CONFIG_SPI_FLASH_GIGADEVICE	/* GIGADEVICE */
	/* GigaDevice */
	{
		INFO("gd25q16", 0xc84015, 0, 64 * 1024,  32,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25q32", 0xc84016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25lq32", 0xc86016, 0, 64 * 1024, 64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25q64", 0xc84017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25lq64c", 0xc86017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25q128", 0xc84018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("gd25lq128", 0xc86018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
#endif
#ifdef CONFIG_SPI_FLASH_ISSI		/* ISSI */
	/* ISSI */
	{ INFO("is25lq040b", 0x9d4013, 0, 64 * 1024,   8,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("is25lp032",	0x9d6016, 0, 64 * 1024,  64, 0) },
	{ INFO("is25lp064",	0x9d6017, 0, 64 * 1024, 128, 0) },
	{ INFO("is25lp128",  0x9d6018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ) },
	{ INFO("is25lp256",  0x9d6019, 0, 64 * 1024, 512,
			SECT_4K | SPI_NOR_DUAL_READ) },
	{ INFO("is25wp032",  0x9d7016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("is25wp064",  0x9d7017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("is25wp128",  0x9d7018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("is25wp256",  0x9d7019, 0, 64 * 1024, 512,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_4B_OPCODES) },
#endif
#ifdef CONFIG_SPI_FLASH_MACRONIX	/* MACRONIX */
	/* Macronix */
	{ INFO("mx25l2005a",  0xc22012, 0, 64 * 1024,   4, SECT_4K) },
	{ INFO("mx25l4005a",  0xc22013, 0, 64 * 1024,   8, SECT_4K) },
	{ INFO("mx25l8005",   0xc22014, 0, 64 * 1024,  16, 0) },
	{ INFO("mx25l1606e",  0xc22015, 0, 64 * 1024,  32, SECT_4K) },
	{ INFO("mx25l3205d",  0xc22016, 0, 64 * 1024,  64, SECT_4K) },
	{ INFO("mx25l6405d",  0xc22017, 0, 64 * 1024, 128, SECT_4K) },
	{ INFO("mx25u2033e",  0xc22532, 0, 64 * 1024,   4, SECT_4K) },
	{ INFO("mx25u1635e",  0xc22535, 0, 64 * 1024,  32, SECT_4K) },
	{ INFO("mx25u3235f",  0xc22536, 0, 4 * 1024,  1024, SECT_4K) },
	{ INFO("mx25u6435f",  0xc22537, 0, 64 * 1024, 128, SECT_4K) },
	{ INFO("mx25l12805d", 0xc22018, 0, 64 * 1024, 256, SECT_4K) },
	{ INFO("mx25u12835f", 0xc22538, 0, 64 * 1024, 256, SECT_4K) },
	{ INFO("mx25l12855e", 0xc22618, 0, 64 * 1024, 256, 0) },
	{ INFO("mx25l25635e", 0xc22019, 0, 64 * 1024, 512, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("mx25u25635f", 0xc22539, 0, 64 * 1024, 512, SECT_4K | SPI_NOR_4B_OPCODES) },
	{ INFO("mx25l25655e", 0xc22619, 0, 64 * 1024, 512, 0) },
	{ INFO("mx66l51235l", 0xc2201a, 0, 64 * 1024, 1024, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("mx66u51235f", 0xc2253a, 0, 64 * 1024, 1024, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("mx66u2g45g",  0xc2253c, 0, 64 * 1024, 4096, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("mx66l1g45g",  0xc2201b, 0, 64 * 1024, 2048, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("mx25l1633e", 0xc22415, 0, 64 * 1024,   32, SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES | SECT_4K) },
	{ INFO("mx25r6435f", 0xc22817, 0, 64 * 1024,   128,  SECT_4K) },
#endif

#ifdef CONFIG_SPI_FLASH_STMICRO		/* STMICRO */
	/* Micron */
	{ INFO("n25q016a",	 0x20bb15, 0, 64 * 1024,   32, SECT_4K | SPI_NOR_QUAD_READ) },
	{ INFO("n25q032",	 0x20ba16, 0, 64 * 1024,   64, SPI_NOR_QUAD_READ) },
	{ INFO("n25q032a",	0x20bb16, 0, 64 * 1024,   64, SPI_NOR_QUAD_READ) },
	{ INFO("n25q064",     0x20ba17, 0, 64 * 1024,  128, SECT_4K | SPI_NOR_QUAD_READ) },
	{ INFO("n25q064a",    0x20bb17, 0, 64 * 1024,  128, SECT_4K | SPI_NOR_QUAD_READ) },
	{ INFO("n25q128a11",  0x20bb18, 0, 64 * 1024,  256, SECT_4K | SPI_NOR_QUAD_READ) },
	{ INFO("n25q128a13",  0x20ba18, 0, 64 * 1024,  256, SECT_4K | SPI_NOR_QUAD_READ) },
	{ INFO6("mt25ql256a",    0x20ba19, 0x104400, 64 * 1024,  512, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES | USE_FSR) },
	{ INFO("n25q256a",    0x20ba19, 0, 64 * 1024,  512, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_FSR) },
	{ INFO6("mt25qu256a",  0x20bb19, 0x104400, 64 * 1024,  512, SECT_4K | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES | USE_FSR) },
	{ INFO("n25q256ax1",  0x20bb19, 0, 64 * 1024,  512, SECT_4K | SPI_NOR_QUAD_READ | USE_FSR) },
	{ INFO6("mt25qu512a",  0x20bb20, 0x104400, 64 * 1024, 1024,
		 SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES |
		 USE_FSR) },
	{ INFO("n25q512a",    0x20bb20, 0, 64 * 1024, 1024, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ) },
	{ INFO6("mt25ql512a",  0x20ba20, 0x104400, 64 * 1024, 1024, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("n25q512ax3",  0x20ba20, 0, 64 * 1024, 1024, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ) },
	{ INFO("n25q00",      0x20ba21, 0, 64 * 1024, 2048, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ | NO_CHIP_ERASE) },
	{ INFO("n25q00a",     0x20bb21, 0, 64 * 1024, 2048, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ | NO_CHIP_ERASE) },
	{ INFO("mt25ql01g",   0x21ba20, 0, 64 * 1024, 2048, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ | NO_CHIP_ERASE) },
	{ INFO("mt25qu02g",   0x20bb22, 0, 64 * 1024, 4096, SECT_4K | USE_FSR | SPI_NOR_QUAD_READ | NO_CHIP_ERASE) },
	{ INFO("mt35xu512aba", 0x2c5b1a, 0,  128 * 1024,  512, USE_FSR | SPI_NOR_OCTAL_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("mt35xu02g",  0x2c5b1c, 0, 128 * 1024,  2048, USE_FSR | SPI_NOR_OCTAL_READ | SPI_NOR_4B_OPCODES) },
#endif
#ifdef CONFIG_SPI_FLASH_SPANSION	/* SPANSION */
	/* Spansion/Cypress -- single (large) sector size only, at least
	 * for the chips listed here (without boot sectors).
	 */
	{ INFO("s25sl032p",  0x010215, 0x4d00,  64 * 1024,  64, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("s25sl064p",  0x010216, 0x4d00,  64 * 1024, 128, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("s25fl256s0", 0x010219, 0x4d00, 256 * 1024, 128, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl256s1", 0x010219, 0x4d01,  64 * 1024, 512, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO6("s25fl512s",  0x010220, 0x4d0080, 256 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO6("s25fs512s",  0x010220, 0x4d0081, 256 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl512s_256k",  0x010220, 0x4d00, 256 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl512s_64k",  0x010220, 0x4d01, 64 * 1024, 1024, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl512s_512k", 0x010220, 0x4f00, 256 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25sl12800", 0x012018, 0x0300, 256 * 1024,  64, 0) },
	{ INFO("s25sl12801", 0x012018, 0x0301,  64 * 1024, 256, 0) },
	{ INFO6("s25fl128s",  0x012018, 0x4d0180, 64 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl129p0", 0x012018, 0x4d00, 256 * 1024,  64, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25fl129p1", 0x012018, 0x4d01,  64 * 1024, 256, SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | USE_CLSR) },
	{ INFO("s25sl008a",  0x010213,      0,  64 * 1024,  16, 0) },
	{ INFO("s25sl016a",  0x010214,      0,  64 * 1024,  32, 0) },
	{ INFO("s25sl032a",  0x010215,      0,  64 * 1024,  64, 0) },
	{ INFO("s25sl064a",  0x010216,      0,  64 * 1024, 128, 0) },
	{ INFO("s25fl116k",  0x014015,      0,  64 * 1024,  32, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("s25fl164k",  0x014017,      0,  64 * 1024, 128, SECT_4K) },
	{ INFO("s25fl208k",  0x014014,      0,  64 * 1024,  16, SECT_4K | SPI_NOR_DUAL_READ) },
	{ INFO("s25fl064l",  0x016017,      0,  64 * 1024, 128, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
	{ INFO("s25fl128l",  0x016018,      0,  64 * 1024, 256, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_4B_OPCODES) },
#endif
#ifdef CONFIG_SPI_FLASH_SST		/* SST */
	/* SST -- large erase sizes are "overlays", "sectors" are 4K */
	{ INFO("sst25vf040b", 0xbf258d, 0, 64 * 1024,  8, SECT_4K | SST_WRITE) },
	{ INFO("sst25vf080b", 0xbf258e, 0, 64 * 1024, 16, SECT_4K | SST_WRITE) },
	{ INFO("sst25vf016b", 0xbf2541, 0, 64 * 1024, 32, SECT_4K | SST_WRITE) },
	{ INFO("sst25vf032b", 0xbf254a, 0, 64 * 1024, 64, SECT_4K | SST_WRITE) },
	{ INFO("sst25vf064c", 0xbf254b, 0, 64 * 1024, 128, SECT_4K) },
	{ INFO("sst25wf512",  0xbf2501, 0, 64 * 1024,  1, SECT_4K | SST_WRITE) },
	{ INFO("sst25wf010",  0xbf2502, 0, 64 * 1024,  2, SECT_4K | SST_WRITE) },
	{ INFO("sst25wf020",  0xbf2503, 0, 64 * 1024,  4, SECT_4K | SST_WRITE) },
	{ INFO("sst25wf020a", 0x621612, 0, 64 * 1024,  4, SECT_4K) },
	{ INFO("sst25wf040b", 0x621613, 0, 64 * 1024,  8, SECT_4K) },
	{ INFO("sst25wf040",  0xbf2504, 0, 64 * 1024,  8, SECT_4K | SST_WRITE) },
	{ INFO("sst25wf080",  0xbf2505, 0, 64 * 1024, 16, SECT_4K | SST_WRITE) },
	{ INFO("sst26vf064b", 0xbf2643, 0, 64 * 1024, 128,
	       SECT_4K_ONLY | UNLOCK_GLOBAL_BLOCK | SPI_NOR_HAS_SST26LOCK |
	       SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("sst26wf016",  0xbf2651, 0, 64 * 1024,  32, SECT_4K | SPI_NOR_HAS_SST26LOCK) },
	{ INFO("sst26wf032",  0xbf2622, 0, 64 * 1024,  64, SECT_4K | SPI_NOR_HAS_SST26LOCK) },
	{ INFO("sst26wf064",  0xbf2643, 0, 64 * 1024, 128, SECT_4K | SPI_NOR_HAS_SST26LOCK) },
#endif
#ifdef CONFIG_SPI_FLASH_STMICRO		/* STMICRO */
	/* ST Microelectronics -- newer production may have feature updates */
	{ INFO("m25p10",  0x202011,  0,  32 * 1024,   4, 0) },
	{ INFO("m25p20",  0x202012,  0,  64 * 1024,   4, 0) },
	{ INFO("m25p40",  0x202013,  0,  64 * 1024,   8, 0) },
	{ INFO("m25p80",  0x202014,  0,  64 * 1024,  16, 0) },
	{ INFO("m25p16",  0x202015,  0,  64 * 1024,  32, 0) },
	{ INFO("m25p32",  0x202016,  0,  64 * 1024,  64, 0) },
	{ INFO("m25p64",  0x202017,  0,  64 * 1024, 128, 0) },
	{ INFO("m25p128", 0x202018,  0, 256 * 1024,  64, 0) },
	{ INFO("m25pe16", 0x208015,  0, 64 * 1024, 32, SECT_4K) },
	{ INFO("m25px16",    0x207115,  0, 64 * 1024, 32, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("m25px64",    0x207117,  0, 64 * 1024, 128, 0) },
#endif
#ifdef CONFIG_SPI_FLASH_WINBOND		/* WINBOND */
	/* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
	{ INFO("w25p80", 0xef2014, 0x0,	64 * 1024,    16, 0) },
	{ INFO("w25p16", 0xef2015, 0x0,	64 * 1024,    32, 0) },
	{ INFO("w25p32", 0xef2016, 0x0,	64 * 1024,    64, 0) },
	{ INFO("w25x05", 0xef3010, 0, 64 * 1024,  1,  SECT_4K) },
	{ INFO("w25x40", 0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
	{ INFO("w25x16", 0xef3015, 0, 64 * 1024,  32, SECT_4K) },
	{
		INFO("w25q16dw", 0xef6015, 0, 64 * 1024,  32,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{ INFO("w25x32", 0xef3016, 0, 64 * 1024,  64, SECT_4K) },
	{ INFO("w25q20cl", 0xef4012, 0, 64 * 1024,  4, SECT_4K) },
	{ INFO("w25q20bw", 0xef5012, 0, 64 * 1024,  4, SECT_4K) },
	{ INFO("w25q20ew", 0xef6012, 0, 64 * 1024,  4, SECT_4K) },
	{ INFO("w25q32", 0xef4016, 0, 64 * 1024,  64, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{
		INFO("w25q32dw", 0xef6016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q32jv", 0xef7016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q32jwm", 0xef8016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{ INFO("w25x64", 0xef3017, 0, 64 * 1024, 128, SECT_4K) },
	{
		INFO("w25q64dw", 0xef6017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q64jv", 0xef7017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q128fw", 0xef6018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q128jv", 0xef7018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q256fw", 0xef6019, 0, 64 * 1024, 512,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{
		INFO("w25q256jw", 0xef7019, 0, 64 * 1024, 512,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{ INFO("w25q80", 0xef5014, 0, 64 * 1024,  16, SECT_4K) },
	{ INFO("w25q80bl", 0xef4014, 0, 64 * 1024,  16, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("w25q16cl", 0xef4015, 0, 64 * 1024,  32, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("w25q64cv", 0xef4017, 0, 64 * 1024,  128, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("w25q128", 0xef4018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB)
	},
	{ INFO("w25q256", 0xef4019, 0, 64 * 1024, 512, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("w25m512jw", 0xef6119, 0, 64 * 1024, 1024, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("w25m512jv", 0xef7119, 0, 64 * 1024, 1024, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
#endif
#ifdef CONFIG_SPI_FLASH_XMC
	/* XMC (Wuhan Xinxin Semiconductor Manufacturing Corp.) */
	{ INFO("XM25QH64A", 0x207017, 0, 64 * 1024, 128, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ INFO("XM25QH128A", 0x207018, 0, 64 * 1024, 256, SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM
	{ INFO(CONFIG_SPI_FLASH_CUSTOM_NAME, CONFIG_SPI_FLASH_CUSTOM_ID,
		CONFIG_SPI_FLASH_CUSTOM_EXT_ID,
		CONFIG_SPI_FLASH_CUSTOM_SECTOR_SIZE,
		CONFIG_SPI_FLASH_CUSTOM_N_SECTORS,
#ifdef CONFIG_SPI_FLASH_CUSTOM_SECT_4K
	SECT_4K 
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_NO_ERASE
    | SPI_NOR_NO_ERASE
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SST_WRITE
	| SST_WRITE
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_NO_FR
	| SPI_NOR_NO_FR
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SECT_4K_PMC
	| SECT_4K_PMC
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_DUAL_READ
	| SPI_NOR_DUAL_READ
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_QUAD_READ
	| SPI_NOR_QUAD_READ
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_USE_FSR
	| USE_FSR
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_HAS_LOCK
	| SPI_NOR_HAS_LOCK
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_HAS_TB
	| SPI_NOR_HAS_TB
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_S3AN
	| SPI_S3AN
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_4B_OPCODES
	| SPI_NOR_4B_OPCODES
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_NO_CHIP_ERASE
	| NO_CHIP_ERASE
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_SKIP_SFDP
	| SPI_NOR_SKIP_SFDP
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_USE_CLSR
	| USE_CLSR
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_HAS_SST26LOCK
	| SPI_NOR_HAS_SST26LOCK
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SPI_NOR_OCTAL_READ
	| SPI_NOR_OCTAL_READ
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_UNLOCK_GLOBAL_BLOCK
	| UNLOCK_GLOBAL_BLOCK
#endif
#ifdef CONFIG_SPI_FLASH_CUSTOM_SECT_4K_ONLY
	| SECT_4K_ONLY
#endif
		) },
#endif
	{ },
};


static int df_desc_init(struct dataflash_descriptor *df_desc, unsigned char vendor,
			unsigned char family)
{
	int ret;

	df_desc->family = family;

	switch ( vendor ) {
		case MANUFACTURER_ID_ATMEL: {

			if ((df_desc->family == DF_FAMILY_AT26F)
				|| (df_desc->family == DF_FAMILY_AT26DF)) {
				ret = df_at25_desc_init(df_desc);
				if (ret)
					return ret;
			} else if (df_desc->family == DF_FAMILY_AT45) {
				ret = df_at45_desc_init(df_desc);
				if (ret)
					return ret;
			} else {
				dbg_info("SF: Unsupported SerialFlash family %x\n", family);
				return -1;
			}
		}
		break;

		case MANUFACTURER_ID_MICRON:{

			if (df_desc->family == DF_FAMILY_M25P) {
				ret = df_n25q_desc_init(df_desc);
				if (ret)
					return ret;
			}else if (df_desc->family == DF_FAMILY_N25Q) {
				ret = df_n25q_desc_init(df_desc);
				if (ret)
					return ret;
			} else {
				dbg_info("SF: Unsupported SerialFlash family %x\n", family);
				return -1;
			}
		}
		break;

		case MANUFACTURER_ID_WINBOND:{

			if (df_desc->family == WINBOND_W25Q128JV) {
				ret = w_25q128_desc_init(df_desc);
				if (ret)
					return ret;
			} else {
				dbg_info("SF: Unsupported SerialFlash family %x\n", family);
				return -1;
			}
		}
		break;

		default:
			dbg_info("SF: Unsupported Manufactorer ID %x\n", vendor);
			return -1;
	}

	return 0;
}


static int dataflash_probe_atmel(struct dataflash_descriptor *df_desc)
{
	unsigned char dev_id[5];
	unsigned char cmd = CMD_READ_DEV_ID;
	int ret;

	/* Read device ID */
	ret = df_send_command(&cmd, 1, dev_id, 5);
	if (ret)
		return ret;

//#ifdef CONFIG_DEBUG
	unsigned int i;
	unsigned char *p = dev_id;

	dbg_info("SF: Got Manufacturer and Device ID:");
	for (i = 0; i < 5; i++)
		dbg_info(" %x", *p++);
	dbg_info("\n");
//#endif

	const struct flash_info	*info;

	info = spi_nor_ids;
	for (; info->name; info++) {
		if (info->id_len) {
			if (!memcmp(info->id, dev_id, info->id_len)) {
					df_desc->pages = info->n_sectors;
					df_desc->page_size = info->page_size;
					df_desc->page_offset = info;
					//  Loading Environment from SPIFlash... SF: Detected w25q128 with page size 256 Bytes, erase size 4 KiB, total 16 MiB
					dbg_info("SF: Detected %s and Device ID: ", info->name);
							for (i = 0; i < 5; i++) dbg_info(" %x", *p++);
					dbg_info(", Sector size:%d, Sectors: %d, Page size:%d\n", info->sector_size, info->n_sectors, info->page_size);
					return 0;
			}
		}
	}
	// df_desc->family


/*	if (dev_id[0] != MANUFACTURER_ID_ATMEL &&
	    dev_id[0] != MANUFACTURER_ID_WINBOND &&
	    dev_id[0] != MANUFACTURER_ID_MICRON) {
		dbg_info("Not supported spi flash Manufactorer ID: %x\n",
			 dev_id[0]);
		return -1;
	}

	ret = df_desc_init(df_desc, dev_id[0], (dev_id[1] & 0xe0) );
	if (ret)
		return ret;
*/
	return 1;
}

int spi_flash_loadimage(struct image_info *image)
{
	struct dataflash_descriptor	df_descriptor;
	struct dataflash_descriptor	*df_desc = &df_descriptor;
	int ret = 0;

	memset(df_desc, 0, sizeof(*df_desc));

	at91_spi0_hw_init();

	ret = at91_spi_init(AT91C_SPI_PCS_DATAFLASH,
				CONFIG_SYS_SPI_CLOCK, CONFIG_SYS_SPI_MODE);
	if (ret) {
		dbg_info("SF: Fail to initialize spi\n");
		return -1;
	}

	at91_spi_enable();

	ret = dataflash_probe_atmel(df_desc);
	if (ret) {
		dbg_info("SF: Fail to probe atmel spi flash\n");
		ret = -1;
		goto err_exit;
	}

#ifdef CONFIG_DATAFLASH_RECOVERY
	if (!dataflash_recovery(df_desc)) {
		ret = -2;
		goto err_exit;
	}
#endif

#if defined(CONFIG_LOAD_LINUX) || defined(CONFIG_LOAD_ANDROID)
	int length = update_image_length(df_desc,
				image->offset, image->dest, KERNEL_IMAGE);
	if (length == -1)
		return -1;

	image->length = length;
#endif

	dbg_info("SF: Copy %x bytes from %x to %x\n",
			image->length, image->offset, image->dest);

	ret = read_array(df_desc, image->offset, image->length, image->dest);
	if (ret) {
		dbg_info("** SF: Serial flash read error**\n");
		ret = -1;
		goto err_exit;
	}

#ifdef CONFIG_OF_LIBFDT
	length = update_image_length(df_desc,
			image->of_offset, image->of_dest, DT_BLOB);
	if (length == -1)
		return -1;

	image->of_length = length;

	dbg_info("SF: dt blob: Copy %x bytes from %x to %x\n",
		image->of_length, image->of_offset, image->of_dest);

	ret = read_array(df_desc,
		image->of_offset, image->of_length, image->of_dest);
	if (ret) {
		dbg_info("** SF: DT: Serial flash read error**\n");
		ret = -1;
		goto err_exit;
	}
#endif

err_exit:
	at91_spi_disable();
	return ret;
}
