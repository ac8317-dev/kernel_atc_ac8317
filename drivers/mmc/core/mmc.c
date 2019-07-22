/*
 *  linux/drivers/mmc/core/mmc.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd_ops.h"

/* specific segments of MMC write protect from uboot */
#define SEG_NUM 10	/* this value must be equal or greater than uboot wp segments */
static u64 Start_addr[SEG_NUM] = {0};
static u64 End_addr[SEG_NUM] = {0};
static u64 Start_addr_backup[SEG_NUM] = {0};
static u64 End_addr_backup[SEG_NUM] = {0};
static u8 wp_seg_backup = 0;
static int wp_array_index = 0;

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

/*
 * Given the decoded CSD structure, decode the raw CID to our CID structure.
 */
static int mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	/*
	 * The selection of the format here is based upon published
	 * specs from sandisk and from what people have reported.
	 */
	switch (card->csd.mmca_vsn) {
	case 0: /* MMC v1.0 - v1.2 */
	case 1: /* MMC v1.4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: /* MMC v2.0 - v2.2 */
	case 3: /* MMC v3.1 - v3.3 */
	case 4: /* MMC v4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.oemid		= UNSTUFF_BITS(resp, 104, 16);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	default:
		pr_err("%s: card has unknown MMCA version %d\n",
			mmc_hostname(card->host), card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

static void mmc_set_erase_size(struct mmc_card *card)
{
	if (card->ext_csd.erase_group_def & 1)
		card->erase_size = card->ext_csd.hc_erase_size;
	else
		card->erase_size = card->csd.erase_size;

	mmc_init_erase(card);
}

/*
 * Given a 128-bit response, decode to our card CSD structure.
 */
static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	/*
	 * We only understand CSD structure v1.1 and v1.2.
	 * v1.2 has extra information in bits 15, 11 and 10.
	 * We also support eMMC v4.4 & v4.41.
	 */
	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
	csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

/*
 * Read extended CSD.
 */
static int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	BUG_ON(!card);
	BUG_ON(!new_ext_csd);

	*new_ext_csd = NULL;

	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate a buffer to "
			"receive the ext_csd.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		kfree(ext_csd);
		*new_ext_csd = NULL;

		/* If the host or the card can't do the switch,
		 * fail more gracefully. */
		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			return err;

		/*
		 * High capacity cards should have this "magic" size
		 * stored in their CSD.
		 */
		if (card->csd.capacity == (4096 * 512)) {
			pr_err("%s: unable to read EXT_CSD "
				"on a possible high capacity card. "
				"Card will be ignored.\n",
				mmc_hostname(card->host));
		} else {
			pr_warning("%s: unable to read "
				"EXT_CSD, performance might "
				"suffer.\n",
				mmc_hostname(card->host));
			err = 0;
		}
	} else
		*new_ext_csd = ext_csd;

	return err;
}


void dump_ext_csd(u8 *ext_csd)
{
    u32 tmp;
    char *rev[] = { "4.0", "4.1", "4.2", "4.3", "Obsolete", "4.41", "4.5", "5.0", "5.1"};

    printk("===========================================================\n");
    printk("[ECSD 192] EXT_CSD rev.              : v1.%d (MMCv%s)\n", 
        ext_csd[EXT_CSD_REV], rev[ext_csd[EXT_CSD_REV]]);
    printk("[ECSD 194] CSD struct rev.           : v1.%d\n", ext_csd[EXT_CSD_STRUCT]);
    printk("[ECSD 504] Supported command sets    : %xh\n", ext_csd[EXT_CSD_S_CMD_SET]);
    printk("[ECSD 503] HPI features              : %xh\n", ext_csd[EXT_CSD_HPI_FEATURE]);
    printk("[ECSD 502] BG operations support     : %xh\n", ext_csd[EXT_CSD_BKOPS_SUPP]);
    printk("[ECSD 246] BG operations status      : %xh\n", ext_csd[EXT_CSD_BKOPS_STATUS]);
    memcpy(&tmp, &ext_csd[EXT_CSD_CORRECT_PRG_SECTS_NUM], 4);
    printk("[ECSD 242] Correct prg. sectors      : %xh\n", tmp);
    printk("[ECSD 241] 1st init time after part. : %d ms\n", ext_csd[EXT_CSD_INI_TIMEOUT_AP] * 100);
    printk("[ECSD 235] Min. write perf.(DDR,52MH,8b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_DDR_W_8_52]);
    printk("[ECSD 234] Min. read perf. (DDR,52MH,8b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_DDR_R_8_52]);
    printk("[ECSD 232] TRIM timeout: %d ms\n", ext_csd[EXT_CSD_TRIM_MULT] & 0xFF * 300);
    printk("[ECSD 231] Secure feature support: %xh\n", ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT]);
    printk("[ECSD 230] Secure erase timeout  : %d ms\n", 300 *
        ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT] * ext_csd[EXT_CSD_SEC_ERASE_MULT]);
    printk("[ECSD 229] Secure trim timeout   : %d ms\n", 300 *
        ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT] * ext_csd[EXT_CSD_SEC_TRIM_MULT]);
    printk("[ECSD 225] Access size           : %d bytes\n", ext_csd[EXT_CSD_ACC_SIZE] * 512);
    printk("[ECSD 224] HC erase unit size    : %d kbytes\n", ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] * 512);
    printk("[ECSD 223] HC erase timeout      : %d ms\n", ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT] * 300);
    printk("[ECSD 221] HC write prot grp size: %d kbytes\n", 512 *
        ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] * ext_csd[EXT_CSD_HC_WP_GPR_SIZE]);
    printk("[ECSD 175] HC erase grp def.     : %xh\n", ext_csd[EXT_CSD_ERASE_GRP_DEF]);
    printk("[ECSD 222] Reliable write sect count: %xh\n", ext_csd[EXT_CSD_REL_WR_SEC_C]);
    printk("[ECSD 220] Sleep current (VCC) : %xh\n", ext_csd[EXT_CSD_S_C_VCC]);
    printk("[ECSD 219] Sleep current (VCCQ): %xh\n", ext_csd[EXT_CSD_S_C_VCCQ]);
    printk("[ECSD 217] Sleep/awake timeout : %d ns\n", 
        100 * (2 << ext_csd[EXT_CSD_S_A_TIMEOUT]));
    memcpy(&tmp, &ext_csd[EXT_CSD_SEC_CNT], 4);
    printk("[ECSD 212] Sector count : %xh\n", tmp);
    printk("[ECSD 210] Min. WR Perf.  (52MH,8b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_W_8_52]);
    printk("[ECSD 209] Min. Read Perf.(52MH,8b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_R_8_52]);
    printk("[ECSD 208] Min. WR Perf.  (26MH,8b,52MH,4b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_W_8_26_4_25]);
    printk("[ECSD 207] Min. Read Perf.(26MH,8b,52MH,4b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_R_8_26_4_25]);
    printk("[ECSD 206] Min. WR Perf.  (26MH,4b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_W_4_26]);
    printk("[ECSD 205] Min. Read Perf.(26MH,4b): %xh\n", ext_csd[EXT_CSD_MIN_PERF_R_4_26]);
    printk("[ECSD 187] Power class: %x\n", ext_csd[EXT_CSD_PWR_CLASS]);
    printk("[ECSD 239] Power class(DDR,52MH,3.6V): %xh\n", ext_csd[EXT_CSD_PWR_CL_DDR_52_360]);
    printk("[ECSD 238] Power class(DDR,52MH,1.9V): %xh\n", ext_csd[EXT_CSD_PWR_CL_DDR_52_195]);
    printk("[ECSD 230] Power class(26MH,3.6V)    : %xh\n", ext_csd[EXT_CSD_PWR_CL_26_360]);
    printk("[ECSD 202] Power class(52MH,3.6V)    : %xh\n", ext_csd[EXT_CSD_PWR_CL_52_360]);    
    printk("[ECSD 201] Power class(26MH,1.9V)    : %xh\n", ext_csd[EXT_CSD_PWR_CL_26_195]);
    printk("[ECSD 200] Power class(52MH,1.9V)    : %xh\n", ext_csd[EXT_CSD_PWR_CL_52_195]);
    printk("[ECSD 199] Part. switch timing    : %xh\n", ext_csd[EXT_CSD_PART_SWITCH_TIME]);
    printk("[ECSD 198] Out-of-INTR busy timing: %xh\n", ext_csd[EXT_CSD_OUT_OF_INTR_TIME]);  
    printk("[ECSD 196] Card type       : %xh\n", ext_csd[EXT_CSD_CARD_TYPE]);
    printk("[ECSD 191] Command set     : %xh\n", ext_csd[EXT_CSD_CMD_SET]);
    printk("[ECSD 189] Command set rev.: %xh\n", ext_csd[EXT_CSD_CMD_SET_REV]);
    printk("[ECSD 185] HS timing       : %xh\n", ext_csd[EXT_CSD_HS_TIMING]);
    printk("[ECSD 183] Bus width       : %xh\n", ext_csd[EXT_CSD_BUS_WIDTH]);
    printk("[ECSD 181] Erase memory content : %xh\n", ext_csd[EXT_CSD_ERASED_MEM_CONT]);
    printk("[ECSD 179] Partition config      : %xh\n", ext_csd[EXT_CSD_PART_CFG]);
    printk("[ECSD 226] Boot partition size   : %d kbytes\n", ext_csd[EXT_CSD_BOOT_SIZE_MULT] * 128);    
    printk("[ECSD 228] Boot information      : %xh\n", ext_csd[EXT_CSD_BOOT_INFO]);    
    printk("[ECSD 178] Boot config protection: %xh\n", ext_csd[EXT_CSD_BOOT_CONFIG_PROT]);
    printk("[ECSD 177] Boot bus width        : %xh\n", ext_csd[EXT_CSD_BOOT_BUS_WIDTH]);
    printk("[ECSD 173] Boot area write prot  : %xh\n", ext_csd[EXT_CSD_BOOT_WP]);
    printk("[ECSD 171] User area write prot  : %xh\n", ext_csd[EXT_CSD_USR_WP]);
    printk("[ECSD 169] FW configuration      : %xh\n", ext_csd[EXT_CSD_FW_CONFIG]);
    printk("[ECSD 168] RPMB size : %d kbytes\n", ext_csd[EXT_CSD_RPMB_SIZE_MULT] * 128);
    printk("[ECSD 167] Write rel. setting  : %xh\n", ext_csd[EXT_CSD_WR_REL_SET]);
    printk("[ECSD 166] Write rel. parameter: %xh\n", ext_csd[EXT_CSD_WR_REL_PARAM]);
    printk("[ECSD 164] Start background ops : %xh\n", ext_csd[EXT_CSD_BKOPS_START]);
    printk("[ECSD 163] Enable background ops: %xh\n", ext_csd[EXT_CSD_BKOPS_EN]);
    printk("[ECSD 162] H/W reset function   : %xh\n", ext_csd[EXT_CSD_RST_N_FUNC]);
    printk("[ECSD 161] HPI management       : %xh\n", ext_csd[EXT_CSD_HPI_MGMT]);
    memcpy(&tmp, &ext_csd[EXT_CSD_MAX_ENH_SIZE_MULT], 4);
    printk("[ECSD 157] Max. enhanced area size : %xh (%d kbytes)\n", 
        tmp & 0x00FFFFFF, (tmp & 0x00FFFFFF) * 512 *
        ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 160] Part. support  : %xh\n", ext_csd[EXT_CSD_PART_SUPPORT]);
    printk("[ECSD 156] Part. attribute: %xh\n", ext_csd[EXT_CSD_PART_ATTR]);
    printk("[ECSD 155] Part. setting  : %xh\n", ext_csd[EXT_CSD_PART_SET_COMPL]);
    printk("[ECSD 143] General purpose 1 size : %xh (%d kbytes)\n", 
        (ext_csd[EXT_CSD_GP1_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP1_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP1_SIZE_MULT + 2] << 16),
        (ext_csd[EXT_CSD_GP1_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP1_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP1_SIZE_MULT + 2] << 16) * 512 *
         ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * 
         ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 146] General purpose 2 size : %xh (%d kbytes)\n", 
        (ext_csd[EXT_CSD_GP2_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP2_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP2_SIZE_MULT + 2] << 16),
        (ext_csd[EXT_CSD_GP2_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP2_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP2_SIZE_MULT + 2] << 16) * 512 *
         ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * 
         ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 149] General purpose 3 size : %xh (%d kbytes)\n", 
        (ext_csd[EXT_CSD_GP3_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP3_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP3_SIZE_MULT + 2] << 16),
        (ext_csd[EXT_CSD_GP3_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP3_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP3_SIZE_MULT + 2] << 16) * 512 *
         ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * 
         ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 152] General purpose 4 size : %xh (%d kbytes)\n", 
        (ext_csd[EXT_CSD_GP4_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP4_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP4_SIZE_MULT + 2] << 16),
        (ext_csd[EXT_CSD_GP4_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_GP4_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_GP4_SIZE_MULT + 2] << 16) * 512 *
         ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * 
         ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 140] Enh. user area size : %xh (%d kbytes)\n", 
        (ext_csd[EXT_CSD_ENH_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_ENH_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_ENH_SIZE_MULT + 2] << 16),
        (ext_csd[EXT_CSD_ENH_SIZE_MULT + 0] | 
         ext_csd[EXT_CSD_ENH_SIZE_MULT + 1] << 8 | 
         ext_csd[EXT_CSD_ENH_SIZE_MULT + 2] << 16) * 512 *
         ext_csd[EXT_CSD_HC_WP_GPR_SIZE] * 
         ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]);
    printk("[ECSD 136] Enh. user area start: %xh\n", 
        (ext_csd[EXT_CSD_ENH_START_ADDR + 0] |
         ext_csd[EXT_CSD_ENH_START_ADDR + 1] << 8 |
         ext_csd[EXT_CSD_ENH_START_ADDR + 2] << 16 |
         ext_csd[EXT_CSD_ENH_START_ADDR + 3]) << 24);
    printk("[ECSD 134] Bad block mgmt mode: %xh\n", ext_csd[EXT_CSD_BADBLK_MGMT]);
    printk("===========================================================\n");
}

/*
 * Decode extended CSD.
 */
#define VENDOR_SAMSUNG  (0x15)
static int mmc_read_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0, idx;
	unsigned int part_size;
	u8 hc_erase_grp_sz = 0, hc_wp_grp_sz = 0;

	BUG_ON(!card);

	if (!ext_csd)
		return 0;

	/* Version is coded in the CSD_STRUCTURE byte in the EXT_CSD register */
	card->ext_csd.raw_ext_csd_structure = ext_csd[EXT_CSD_STRUCTURE];
	if (card->csd.structure == 3) {
		if (card->ext_csd.raw_ext_csd_structure > 2) {
			pr_err("%s: unrecognised EXT_CSD structure "
				"version %d\n", mmc_hostname(card->host),
					card->ext_csd.raw_ext_csd_structure);
			err = -EINVAL;
			goto out;
		}
	}

	card->ext_csd.rev = ext_csd[EXT_CSD_REV];
	if (card->ext_csd.rev > 8) {
		pr_err("%s: unrecognised EXT_CSD revision %d\n",
			mmc_hostname(card->host), card->ext_csd.rev);
		err = -EINVAL;
		goto out;
	}

	card->ext_csd.raw_sectors[0] = ext_csd[EXT_CSD_SEC_CNT + 0];
	card->ext_csd.raw_sectors[1] = ext_csd[EXT_CSD_SEC_CNT + 1];
	card->ext_csd.raw_sectors[2] = ext_csd[EXT_CSD_SEC_CNT + 2];
	card->ext_csd.raw_sectors[3] = ext_csd[EXT_CSD_SEC_CNT + 3];
	if (card->ext_csd.rev >= 2) {
		card->ext_csd.sectors =
			ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			ext_csd[EXT_CSD_SEC_CNT + 3] << 24;

		/* Cards with density > 2GiB are sector addressed */
		if (card->ext_csd.sectors > (2u * 1024 * 1024 * 1024) / 512)
			mmc_card_set_blockaddr(card);
	}
	card->ext_csd.raw_card_type = ext_csd[EXT_CSD_CARD_TYPE];
	switch (ext_csd[EXT_CSD_CARD_TYPE] & EXT_CSD_CARD_TYPE_MASK) {
	case EXT_CSD_CARD_TYPE_SDR_ALL:
	case EXT_CSD_CARD_TYPE_SDR_ALL_DDR_1_8V:
	case EXT_CSD_CARD_TYPE_SDR_ALL_DDR_1_2V:
	case EXT_CSD_CARD_TYPE_SDR_ALL_DDR_52:
		card->ext_csd.hs_max_dtr = 200000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_SDR_200;
		break;
	case EXT_CSD_CARD_TYPE_SDR_1_2V_ALL:
	case EXT_CSD_CARD_TYPE_SDR_1_2V_DDR_1_8V:
	case EXT_CSD_CARD_TYPE_SDR_1_2V_DDR_1_2V:
	case EXT_CSD_CARD_TYPE_SDR_1_2V_DDR_52:
		card->ext_csd.hs_max_dtr = 200000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_SDR_1_2V;
		break;
	case EXT_CSD_CARD_TYPE_SDR_1_8V_ALL:
	case EXT_CSD_CARD_TYPE_SDR_1_8V_DDR_1_8V:
	case EXT_CSD_CARD_TYPE_SDR_1_8V_DDR_1_2V:
	case EXT_CSD_CARD_TYPE_SDR_1_8V_DDR_52:
		card->ext_csd.hs_max_dtr = 200000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_SDR_1_8V;
		break;
	case EXT_CSD_CARD_TYPE_DDR_52 | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_52;
		break;
	case EXT_CSD_CARD_TYPE_DDR_1_2V | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_2V;
		break;
	case EXT_CSD_CARD_TYPE_DDR_1_8V | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_8V;
		break;
	case EXT_CSD_CARD_TYPE_52 | EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		break;
	case EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 26000000;
		break;
	default:
		/* MMC v4 spec says this cannot happen */
		pr_warning("%s: card is mmc v4 but doesn't "
			"support any high-speed modes.\n",
			mmc_hostname(card->host));
	}

	card->ext_csd.raw_s_a_timeout = ext_csd[EXT_CSD_S_A_TIMEOUT];
	card->ext_csd.raw_erase_timeout_mult =
		ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
	card->ext_csd.raw_hc_erase_grp_size =
		ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
	if (card->ext_csd.rev >= 3) {
		u8 sa_shift = ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.part_config = ext_csd[EXT_CSD_PART_CONFIG];

		/* EXT_CSD value is in units of 10ms, but we store in ms */
		card->ext_csd.part_time = 10 * ext_csd[EXT_CSD_PART_SWITCH_TIME];

		/* Sleep / awake timeout in 100ns units */
		if (sa_shift > 0 && sa_shift <= 0x17)
			card->ext_csd.sa_timeout =
					1 << ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.erase_group_def =
			ext_csd[EXT_CSD_ERASE_GROUP_DEF];
		card->ext_csd.hc_erase_timeout = 300 *
			ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
		card->ext_csd.hc_erase_size =
			ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] << 10;

		card->ext_csd.rel_sectors = ext_csd[EXT_CSD_REL_WR_SEC_C];

		/*
		 * There are two boot regions of equal size, defined in
		 * multiples of 128K.
		 */
		// Disable boot partition be mount as sub partition. Max Xia
#ifdef EMMC_BOOT_DEV_ENABLE
		if (ext_csd[EXT_CSD_BOOT_MULT] && 
				mmc_boot_partition_access(card->host)) {
			for (idx = 0; idx < MMC_NUM_BOOT_PARTITION; idx++) {
				part_size = ext_csd[EXT_CSD_BOOT_MULT] << 17;
				mmc_part_add(card, part_size,
						EXT_CSD_PART_CONFIG_ACC_BOOT0 + idx,
						"boot%d", idx, false,
						MMC_BLK_DATA_AREA_BOOT);
			}
		}
#endif
	}

	card->ext_csd.raw_hc_erase_gap_size =
		ext_csd[EXT_CSD_HC_WP_GRP_SIZE];
	card->ext_csd.raw_sec_trim_mult =
		ext_csd[EXT_CSD_SEC_TRIM_MULT];
	card->ext_csd.raw_sec_erase_mult =
		ext_csd[EXT_CSD_SEC_ERASE_MULT];
	card->ext_csd.raw_sec_feature_support =
		ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
	card->ext_csd.raw_trim_mult =
		ext_csd[EXT_CSD_TRIM_MULT];
	card->ext_csd.raw_partition_support = ext_csd[EXT_CSD_PARTITION_SUPPORT];
	if (card->ext_csd.rev >= 4) {
		/*
		 * Enhanced area feature support -- check whether the eMMC
		 * card has the Enhanced area enabled.  If so, export enhanced
		 * area offset and size to user by adding sysfs interface.
		 */
		if ((ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x2) &&
		    (ext_csd[EXT_CSD_PARTITION_ATTRIBUTE] & 0x1)) {
			hc_erase_grp_sz =
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
			hc_wp_grp_sz =
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

			card->ext_csd.enhanced_area_en = 1;
			/*
			 * calculate the enhanced data area offset, in bytes
			 */
			card->ext_csd.enhanced_area_offset =
				(ext_csd[139] << 24) + (ext_csd[138] << 16) +
				(ext_csd[137] << 8) + ext_csd[136];
			if (mmc_card_blockaddr(card))
				card->ext_csd.enhanced_area_offset <<= 9;
			/*
			 * calculate the enhanced data area size, in kilobytes
			 */
			card->ext_csd.enhanced_area_size =
				(ext_csd[142] << 16) + (ext_csd[141] << 8) +
				ext_csd[140];
			card->ext_csd.enhanced_area_size *=
				(size_t)(hc_erase_grp_sz * hc_wp_grp_sz);
			card->ext_csd.enhanced_area_size <<= 9;
		} else {
			/*
			 * If the enhanced area is not enabled, disable these
			 * device attributes.
			 */
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		}

		/*
		 * General purpose partition feature support --
		 * If ext_csd has the size of general purpose partitions,
		 * set size, part_cfg, partition name in mmc_part.
		 */
		if (ext_csd[EXT_CSD_PARTITION_SUPPORT] &
			EXT_CSD_PART_SUPPORT_PART_EN) {
			if (card->ext_csd.enhanced_area_en != 1) {
				hc_erase_grp_sz =
					ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
				hc_wp_grp_sz =
					ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

				card->ext_csd.enhanced_area_en = 1;
			}

			for (idx = 0; idx < MMC_NUM_GP_PARTITION; idx++) {
				if (!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3] &&
				!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1] &&
				!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2])
					continue;
				part_size =
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2]
					<< 16) +
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1]
					<< 8) +
				ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3];
				part_size *= (size_t)(hc_erase_grp_sz *
					hc_wp_grp_sz);
				mmc_part_add(card, part_size << 19,
					EXT_CSD_PART_CONFIG_ACC_GP0 + idx,
					"gp%d", idx, false,
					MMC_BLK_DATA_AREA_GP);
			}
		}
		card->ext_csd.sec_trim_mult =
			ext_csd[EXT_CSD_SEC_TRIM_MULT];
		card->ext_csd.sec_erase_mult =
			ext_csd[EXT_CSD_SEC_ERASE_MULT];
		card->ext_csd.sec_feature_support =
			ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
		card->ext_csd.trim_timeout = 300 *
			ext_csd[EXT_CSD_TRIM_MULT];

		/*
		 * Note that the call to mmc_part_add above defaults to read
		 * only. If this default assumption is changed, the call must
		 * take into account the value of boot_locked below.
		 */
		card->ext_csd.boot_ro_lock = ext_csd[EXT_CSD_BOOT_WP];
		card->ext_csd.boot_ro_lockable = true;
	}

	if (card->ext_csd.rev >= 5) {
		/* check whether the eMMC card supports HPI */
		if (ext_csd[EXT_CSD_HPI_FEATURES] & 0x1) {
			card->ext_csd.hpi = 1;
			if (ext_csd[EXT_CSD_HPI_FEATURES] & 0x2)
				card->ext_csd.hpi_cmd =	MMC_STOP_TRANSMISSION;
			else
				card->ext_csd.hpi_cmd = MMC_SEND_STATUS;
			/*
			 * Indicate the maximum timeout to close
			 * a command interrupted by HPI
			 */
			card->ext_csd.out_of_int_time =
				ext_csd[EXT_CSD_OUT_OF_INTERRUPT_TIME] * 10;
		}

		card->ext_csd.rel_param = ext_csd[EXT_CSD_WR_REL_PARAM];
		card->ext_csd.rst_n_function = ext_csd[EXT_CSD_RST_N_FUNCTION];
	}

	card->ext_csd.raw_erased_mem_count = ext_csd[EXT_CSD_ERASED_MEM_CONT];
	if (ext_csd[EXT_CSD_ERASED_MEM_CONT])
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

        /* for samsung emmc4.41 plus spec */
        if ((card->cid.manfid == VENDOR_SAMSUNG) && 
            (card->ext_csd.rev == 5)             && 
            (1 == (0x1 & ext_csd[EXT_CSD_SAMSUNG_FEATURE]))){
            printk("set to support discard\n");
            card->ext_csd.feature_support |= MMC_DISCARD_FEATURE;
        }

	/* eMMC v4.5 or later */
	if (card->ext_csd.rev >= 6) {
		card->ext_csd.feature_support |= MMC_DISCARD_FEATURE;

		card->ext_csd.generic_cmd6_time = 10 *
			ext_csd[EXT_CSD_GENERIC_CMD6_TIME];
		card->ext_csd.power_off_longtime = 10 *
			ext_csd[EXT_CSD_POWER_OFF_LONG_TIME];

		card->ext_csd.cache_size =
			ext_csd[EXT_CSD_CACHE_SIZE + 0] << 0 |
			ext_csd[EXT_CSD_CACHE_SIZE + 1] << 8 |
			ext_csd[EXT_CSD_CACHE_SIZE + 2] << 16 |
			ext_csd[EXT_CSD_CACHE_SIZE + 3] << 24;

		if (ext_csd[EXT_CSD_DATA_SECTOR_SIZE] == 1)
			card->ext_csd.data_sector_size = 4096;
		else
			card->ext_csd.data_sector_size = 512;

		if ((ext_csd[EXT_CSD_DATA_TAG_SUPPORT] & 1) &&
		    (ext_csd[EXT_CSD_TAG_UNIT_SIZE] <= 8)) {
			card->ext_csd.data_tag_unit_size =
			((unsigned int) 1 << ext_csd[EXT_CSD_TAG_UNIT_SIZE]) *
			(card->ext_csd.data_sector_size);
		} else {
			card->ext_csd.data_tag_unit_size = 0;
		}
	}

out:
	return err;
}

static inline void mmc_free_ext_csd(u8 *ext_csd)
{
	kfree(ext_csd);
}


static int mmc_compare_ext_csds(struct mmc_card *card, unsigned bus_width)
{
	u8 *bw_ext_csd;
	int err;

	if (bus_width == MMC_BUS_WIDTH_1)
		return 0;

	err = mmc_get_ext_csd(card, &bw_ext_csd);

	if (err || bw_ext_csd == NULL) {
		if (bus_width != MMC_BUS_WIDTH_1)
			err = -EINVAL;
		goto out;
	}

	if (bus_width == MMC_BUS_WIDTH_1)
		goto out;

	/* only compare read only fields */
	err = !((card->ext_csd.raw_partition_support ==
			bw_ext_csd[EXT_CSD_PARTITION_SUPPORT]) &&
		(card->ext_csd.raw_erased_mem_count ==
			bw_ext_csd[EXT_CSD_ERASED_MEM_CONT]) &&
		(card->ext_csd.rev ==
			bw_ext_csd[EXT_CSD_REV]) &&
		(card->ext_csd.raw_ext_csd_structure ==
			bw_ext_csd[EXT_CSD_STRUCTURE]) &&
		(card->ext_csd.raw_card_type ==
			bw_ext_csd[EXT_CSD_CARD_TYPE]) &&
		(card->ext_csd.raw_s_a_timeout ==
			bw_ext_csd[EXT_CSD_S_A_TIMEOUT]) &&
		(card->ext_csd.raw_hc_erase_gap_size ==
			bw_ext_csd[EXT_CSD_HC_WP_GRP_SIZE]) &&
		(card->ext_csd.raw_erase_timeout_mult ==
			bw_ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT]) &&
		(card->ext_csd.raw_hc_erase_grp_size ==
			bw_ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]) &&
		(card->ext_csd.raw_sec_trim_mult ==
			bw_ext_csd[EXT_CSD_SEC_TRIM_MULT]) &&
		(card->ext_csd.raw_sec_erase_mult ==
			bw_ext_csd[EXT_CSD_SEC_ERASE_MULT]) &&
		(card->ext_csd.raw_sec_feature_support ==
			bw_ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT]) &&
		(card->ext_csd.raw_trim_mult ==
			bw_ext_csd[EXT_CSD_TRIM_MULT]) &&
		(card->ext_csd.raw_sectors[0] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 0]) &&
		(card->ext_csd.raw_sectors[1] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 1]) &&
		(card->ext_csd.raw_sectors[2] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 2]) &&
		(card->ext_csd.raw_sectors[3] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 3]));
	if (err)
		err = -EINVAL;

out:
	mmc_free_ext_csd(bw_ext_csd);
	return err;
}

/* 
 * "cat wp" will call this function.
 * the content of wp file can be only 0 or 1.
 * 0 represent current status of eMMC write protect is diable (clear);
 * 1 represent current status of eMMC write protect is enable (set);
 */
static ssize_t mmc_wp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 wp_status = 0;
	ssize_t count = 0;

	
	/* mmc_wp_is_set_in_kernel() must be called after mmc_wp_check() */
	if(mmc_wp_is_set_in_kernel())
	{
		wp_status = 1;
	}
	else
	{
		wp_status = 0;
	}

	count = sprintf(buf, "%d", wp_status);	
	return count;
}


/*
 * "echo number > wp" operation will call this function.
 * "number" can be only 0(clear WP) or 1(set WP).
 * if WP was none or clear in UBOOT, "echo 1 > wp" operation will be illegal;
 * if WP was none or clear in UBOOT, "echo 0 > wp" operation will do nothing (but giving a message);
 */
static ssize_t mmc_wp_store(struct device *dev, struct device_attribute *attr, const char *buf, ssize_t count)
{
	u32 wp_status = 0;
	struct mmc_card* card = NULL;
	struct mmc_host* host = NULL;


	card = mmc_dev_to_card(dev);
	BUG_ON(card == NULL);

	host = card->host;
	BUG_ON(host == NULL);


	sscanf(buf, "%d", &wp_status);	

#if 1
	if((wp_status != 0) && (wp_status != 1))
	{
		pr_err("echo number in wp invalid: number can be only 0 or 1.\n");
		return -EINVAL;
	}

	mmc_claim_host(host);

	if(mmc_wp_is_set_in_uboot())
	{
		if(wp_status == 1)
		{
			if(mmc_wp_is_set_in_kernel())
			{
				printk(KERN_ALERT"\n>>>>>>>>>>> eMMC WP has been set <<<<<<<<<<<<<\n");
			}
			else
			{
				/* set WP at specific address range */	
				mmc_wp_reset_segments();
				mmc_wp_enable(host, 1);
			}
		}

		if(wp_status == 0)
		{
			if(mmc_wp_is_set_in_kernel())
			{
				/* clear WP at specific address range */
				mmc_wp_reset_segments();
				mmc_wp_enable(host, 0);
			}
			else
			{
				printk(KERN_ALERT"\n>>>>>>>>>>> eMMC WP has been clear <<<<<<<<<<<<<\n");
			}
		}
	}
	else
	{
		if(wp_status == 1)
		{
			/* set WP at specific address range */	
			printk(KERN_ALERT"\n>>>>>>>>>>> PLEASE SET EMMC WP IN UBOOT !!! <<<<<<<<<<<<<\n");
		}

		if(wp_status == 0)
		{
			/* clear WP at specific address range */
			printk(KERN_ALERT"\n>>>>>>>>>>> EMMC WP HAS BEEN CLEAR IN UBOOT ... <<<<<<<<<<<<<\n");
		}
	}

	mmc_wp_check(host);
	mmc_release_host(host);
#endif

	return count;
}

static DEVICE_ATTR(wp, 0777, mmc_wp_show, mmc_wp_store);

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(erase_size, "%u\n", card->erase_size << 9);
MMC_DEV_ATTR(preferred_erase_size, "%u\n", card->pref_erase << 9);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);
MMC_DEV_ATTR(enhanced_area_offset, "%llu\n",
		card->ext_csd.enhanced_area_offset);
MMC_DEV_ATTR(enhanced_area_size, "%u\n", card->ext_csd.enhanced_area_size);

static struct attribute *mmc_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_date.attr,
	&dev_attr_erase_size.attr,
	&dev_attr_preferred_erase_size.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_wp.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	&dev_attr_enhanced_area_offset.attr,
	&dev_attr_enhanced_area_size.attr,
	NULL,
};

static struct attribute_group mmc_std_attr_group = {
	.attrs = mmc_std_attrs,
};

static const struct attribute_group *mmc_attr_groups[] = {
	&mmc_std_attr_group,
	NULL,
};


//  struct device_type mmc_type��Ϊmmc_card�����˺ܶ����ԣ�������sysfs�н��в鿴
static struct device_type mmc_type = {
	.groups = mmc_attr_groups,
};
   /**********
/sys/class/mmc_host/mmc0/mmc0:0001
����/sys/bus/mmc/devices/mmc0:0001�¿��Բ鿴��������

����˵�������Է�����Щ��Ϣ���Ǵ�mmc_card��cid�Ĵ�����ext_csd�Ĵ����ж�ȡ��

    ***************/

/*
 * Select the PowerClass for the current bus width
 * If power class is defined for 4/8 bit bus in the
 * extended CSD register, select it by executing the
 * mmc_switch command.
 */
static int mmc_select_powerclass(struct mmc_card *card,
		unsigned int bus_width, u8 *ext_csd)
{
	int err = 0;
	unsigned int pwrclass_val;
	unsigned int index = 0;
	struct mmc_host *host;

	BUG_ON(!card);

	host = card->host;
	BUG_ON(!host);

	if (ext_csd == NULL)
		return 0;

	/* Power class selection is supported for versions >= 4.0 */
	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	/* Power class values are defined only for 4/8 bit bus */
	if (bus_width == EXT_CSD_BUS_WIDTH_1)
		return 0;

	switch (1 << host->ios.vdd) {
	case MMC_VDD_165_195:
		if (host->ios.clock <= 26000000)
			index = EXT_CSD_PWR_CL_26_195;
		else if	(host->ios.clock <= 52000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_52_195 :
				EXT_CSD_PWR_CL_DDR_52_195;
		else if (host->ios.clock <= 200000000)
			index = EXT_CSD_PWR_CL_200_195;
		break;
	case MMC_VDD_27_28:
	case MMC_VDD_28_29:
	case MMC_VDD_29_30:
	case MMC_VDD_30_31:
	case MMC_VDD_31_32:
	case MMC_VDD_32_33:
	case MMC_VDD_33_34:
	case MMC_VDD_34_35:
	case MMC_VDD_35_36:
		if (host->ios.clock <= 26000000)
			index = EXT_CSD_PWR_CL_26_360;
		else if	(host->ios.clock <= 52000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_52_360 :
				EXT_CSD_PWR_CL_DDR_52_360;
		else if (host->ios.clock <= 200000000)
			index = EXT_CSD_PWR_CL_200_360;
		break;
	default:
		pr_warning("%s: Voltage range not supported "
			   "for power class.\n", mmc_hostname(host));
		return -EINVAL;
	}

	pwrclass_val = ext_csd[index];

	if (bus_width & (EXT_CSD_BUS_WIDTH_8 | EXT_CSD_DDR_BUS_WIDTH_8))
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_8BIT_MASK) >>
				EXT_CSD_PWR_CL_8BIT_SHIFT;
	else
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_4BIT_MASK) >>
				EXT_CSD_PWR_CL_4BIT_SHIFT;

	/* If the power class is different from the default value */
	if (pwrclass_val > 0) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_CLASS,
				 pwrclass_val,
				 card->ext_csd.generic_cmd6_time);
	}

	return err;
}

/*
 * Selects the desired buswidth and switch to the HS200 mode
 * if bus width set without error
 */
static int mmc_select_hs200(struct mmc_card *card)
{
	int idx, err = 0;
	struct mmc_host *host;
	static unsigned ext_csd_bits[] = {
		EXT_CSD_BUS_WIDTH_4,
		EXT_CSD_BUS_WIDTH_8,
	};
	static unsigned bus_widths[] = {
		MMC_BUS_WIDTH_4,
		MMC_BUS_WIDTH_8,
	};

	BUG_ON(!card);

	host = card->host;

	if (card->ext_csd.card_type & EXT_CSD_CARD_TYPE_SDR_1_2V &&
	    host->caps2 & MMC_CAP2_HS200_1_2V_SDR)
		if (mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120, 0))
			err = mmc_set_signal_voltage(host,
						     MMC_SIGNAL_VOLTAGE_180, 0);

	/* If fails try again during next card power cycle */
	if (err)
		goto err;

	idx = (host->caps & MMC_CAP_8_BIT_DATA) ? 1 : 0;

	/*
	 * Unlike SD, MMC cards dont have a configuration register to notify
	 * supported bus width. So bus test command should be run to identify
	 * the supported bus width or compare the ext csd values of current
	 * bus width and ext csd values of 1 bit mode read earlier.
	 */
	for (; idx >= 0; idx--) {

		/*
		 * Host is capable of 8bit transfer, then switch
		 * the device to work in 8bit transfer mode. If the
		 * mmc switch command returns error then switch to
		 * 4bit transfer mode. On success set the corresponding
		 * bus width on the host.
		 */
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_BUS_WIDTH,
				 ext_csd_bits[idx],
				 card->ext_csd.generic_cmd6_time);
		if (err)
			continue;

		mmc_set_bus_width(card->host, bus_widths[idx]);

		if (!(host->caps & MMC_CAP_BUS_WIDTH_TEST))
			err = mmc_compare_ext_csds(card, bus_widths[idx]);
		else
			err = mmc_bus_test(card, bus_widths[idx]);
		if (!err)
			break;
	}

	/* switch to HS200 mode if bus width set successfully */
	if (!err)
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_HS_TIMING, 2, 0);
err:
	return err;
}

/*
 * Handle the detection and initialisation of a card.
 *
 * In the case of a resume, "oldcard" will contain the card
 * we're trying to reinitialise.
 */

/********
���Կ���mmc_attach_mmc�е�һ�����ĺ�������mmc_init_card�����ڶ�mmc type card����ʵ���Եĳ�ʼ������Ϊ�����ͳ�ʼ��һ����Ӧ��mmc_card��
�ⲿ�ֺ�Э����أ���Ҫ��ѧϰһ��mmcЭ�顣

��Ҫ����

����Э���ʼ��mmc type card��ʹ�������Ӧ״̬��standby state��
Ϊmmc type card�����Ӧmmc_card����������
��card��csd�Ĵ����Լ�ext_csd�Ĵ�����ȡcard��Ϣ�����õ�mmc_card����Ӧ��Ա��
����host�����Լ�һЩ�����޸�ext_csd�Ĵ�����ֵ
����mmc����ʱ��Ƶ���Լ�λ��

**********/

 //  card ������ȷ���Ժ�Ϳ��Խ��п��ĳ�ʼ�� 
int mmc_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)   
{

	// struct mmc_host *host����mmc cardʹ�õ�host
    // ocr����ʾ��hostҪʹ�õĵ�ѹ����mmc_attach_mmc�У��Ѿ��õ���һ��HOST��card��֧�ֵ���͵�ѹ  struct mmc_card *card

	struct mmc_card *card;
	int err, ddr = 0;
	u32 cid[4];
	unsigned int max_dtr;
	u32 rocr;
	u8 *ext_csd = NULL;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	/* Set correct bus mode for MMC before attempting init */
	if (!mmc_host_is_spi(host))
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);   //  ��������ģʽΪ��©ģʽ  

	/* Initialization should be done at 3.3 V I/O voltage. */
	mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330, 0);

	/*
	 * Since we're changing the OCR value, we seem to
	 * need to tell some cards to go back to the idle
	 * state.  We wait 1ms to give cards time to
	 * respond.
	 * mmc_go_idle is needed for eMMC that are asleep
	 */

	//  ���� mmc Э�� ��mmc ������ѡȡһ�� card (Э��ĳ�ʼ������)
	mmc_go_idle(host);   // ���� cmd0 ָ��  GO IDLE STATE   ʹmmc ���� idle state 
	          // ��Ȼ������ idle state �����ϵ縴λ�Ĺ��̲���һ���������  ����Ҫ����ȡ OCR ��busy λ���ж� �����̹��Ϊ��һ��  

	/* The extra bit indicates that we support high capacity */
	err = mmc_send_op_cond(host, ocr | (1 << 30), &rocr);  // ���� cmd1  send op cond  ��ͻ����� card �Ĺ�����ѹ�Ĵ��� OCR ��ͨ���ж� busy λ bit31 ���ж�card  ��
	if (err)               // �ϵ縴λ�Ĺ����Ƿ����   ���û����� ���ظ�����      ������ ��mmc card ���� ready state  
		goto err;

	/*
	 * For SPI, enable CRC as appropriate.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto err;
	}

	/*
	 * Fetch CID from card.
	 */
	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);   // ����ᷢ�� cmd2 ָ��  �㲥ָ�� ʹcard �ظ���Ӧ�� CID �Ĵ�����ֵ ������ͻ���� cid ��ֵ��  �洢�� cid �� 
	if (err)                                 //  ���֮�� card �ͽ����� identification state 
		goto err;

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}

		card = oldcard;
	} else {
		/*
		 * Allocate card structure.
		 */
		card = mmc_alloc_card(host, &mmc_type);    //  ���� mmc alloca card ����һ�� mmc card �����в�������  
		if (IS_ERR(card)) {                        // Ϊcard���һ��struct mmc_card�ṹ�岢���г�ʼ������mmc_type��Ϊmmc�����˴���������
			err = PTR_ERR(card);
			goto err;
		}

		card->type = MMC_TYPE_MMC;   // ����card��typeΪMMC_TYPE_MMC   
		card->rca = 1;               // ����card��RCA��ַΪ1
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));  // ��������CID�洢��card->raw_cid��Ҳ����ԭʼCIDֵ��
	}

	/*
	 * For native busses:  set card RCA and quit open drain mode.
	 */
	 /* ����card RCA��ַ */
	if (!mmc_host_is_spi(host)) {
		err = mmc_set_relative_addr(card);
                // ����CMD3ָ�SET_RELATIVE_ADDR
                // ���ø�mmc card�Ĺ�����ַΪcard->rca��Ҳ����0x0001
                // ���֮�󣬸�MMC card����standbyģʽ
		if (err)
			goto free_card;

		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);  // ��������ģʽΪMMC_BUSMODE_PUSHPULL
	}
	
	/* ��card��csd�Ĵ����Լ�ext_csd�Ĵ�����ȡ��Ϣ�����õ�mmc_card����Ӧ��Ա�� */
	if (!oldcard) {
		/*
		 * Fetch CSD from card.
		 */
		err = mmc_send_csd(card, card->raw_csd);
		                // ����CMD9ָ�MMC_SEND_CSD
                        // Ҫ��mmc card����csd�Ĵ������洢��card->raw_csd�У�Ҳ����ԭʼ��csd�Ĵ�����ֵ��
                        // ��ʱmmc card���Ǵ���standby state
		if (err)
			goto free_card;

		err = mmc_decode_csd(card);  // ����raw_csd����ȡ������bit��ֵ�����õ�card->csd�е���Ӧ��Ա��
		if (err)
			goto free_card;
		err = mmc_decode_cid(card);  // ����raw_cid����ȡ������bit��ֵ�����õ�card->cid�е���Ӧ��Ա��
		if (err)
			goto free_card;
	}

	/*
	 * Select card, as all following commands rely on that.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);    // ����CMD7ָ�SELECT/DESELECT CARD
						                // ѡ����߶Ͽ�ָ����card
						                // ��ʱ������transfer state����������ͨ������ָ����뵽receive-data state����sending-data
		if (err)
			goto free_card;
	}

	if (!oldcard) {
		/*
		 * Fetch and process extended CSD.
		 */
			               // ����CMD8ָ�SEND_EXT_CSD
						  // ����Ҫ����transfer state��card����ext_csd�Ĵ����������ȡ֮������ext_csd�Ĵ�����
						  // �����ʹcard����sending-data state�����֮�����˳���transfer state��

		err = mmc_get_ext_csd(card, &ext_csd);
		if (err)
			goto free_card;
		err = mmc_read_ext_csd(card, ext_csd);  // ����ext_csd��ֵ����ȡ������bit��ֵ�����õ�card->ext_csd�е���Ӧ��Ա�� 
		if (err)
			goto free_card;

		/* If doing byte addressing, check if required to do sector
		 * addressing.  Handle the case of <2GB cards needing sector
		 * addressing.  See section 8.1 JEDEC Standard JED84-A441;
		 * ocr register has bit 30 set for sector addressing.
		 */
		if (!(mmc_card_blockaddr(card)) && (rocr & (1<<30)))
			mmc_card_set_blockaddr(card);

		/* Erase size depends on CSD and Extended CSD */
		mmc_set_erase_size(card);            // ����card��erase_size����������Ĳ����ֽ�������������512K
	}

	/*
	 * If enhanced_area_en is TRUE, host needs to enable ERASE_GRP_DEF
	 * bit.  This bit will be lost every time after a reset or power off.
	 */
	 /* ����host�����Լ�һЩ�����޸�ext_csd�Ĵ�����ֵ */
	if (card->ext_csd.enhanced_area_en ||
	    (card->ext_csd.rev >= 3 && (host->caps2 & MMC_CAP2_HC_ERASE_SZ))) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_ERASE_GROUP_DEF, 1,
				 card->ext_csd.generic_cmd6_time);
		               // ����CMD6���MMC_SWITCH
					   // ��������ext_csd�Ĵ�����ĳЩbit
					   // ��enhanced_area_en �����õ�ʱ��host��Ҫȥ����ext_csd�Ĵ����е�EXT_CSD_ERASE_GROUP_DEFλΪ1
		   }

		if (err && err != -EBADMSG)
			goto free_card;

		if (err) {
			err = 0;
			/*
			 * Just disable enhanced area off & sz
			 * will try to enable ERASE_GROUP_DEF
			 * during next time reinit
			 */
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		} else {
			card->ext_csd.erase_group_def = 1;
			/*
			 * enable ERASE_GRP_DEF successfully.
			 * This will affect the erase size, so
			 * here need to reset erase size
			 */
			mmc_set_erase_size(card);
		}
	}

	/*
	 * Ensure eMMC user default partition is enabled
	 */
	if (card->ext_csd.part_config & EXT_CSD_PART_CONFIG_ACC_MASK) {
		card->ext_csd.part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_PART_CONFIG,
				 card->ext_csd.part_config,
				 card->ext_csd.part_time);
	                  	// ����CMD6���MMC_SWITCH
						// ��������ext_csd�Ĵ�����ĳЩbit
						// ����ext_csd�Ĵ����е�EXT_CSD_CMD_SET_NORMALλΪEXT_CSD_PART_CONFIG

		
		if (err && err != -EBADMSG)
			goto free_card;
	}

	/*
	 * If the host supports the power_off_notify capability then
	 * set the notification byte in the ext_csd register of device
	 */
	if ((host->caps2 & MMC_CAP2_POWEROFF_NOTIFY) &&
	    (card->ext_csd.rev >= 6)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_OFF_NOTIFICATION,
				 EXT_CSD_POWER_ON,
				 card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;

		/*
		 * The err can be -EBADMSG or 0,
		 * so check for success and update the flag
		 */
		if (!err)
			card->poweroff_notify_state = MMC_POWERED_ON;
	}

	/*
	 * Activate high speed (if supported)
	 */
	if (card->ext_csd.hs_max_dtr != 0) {
		err = 0;
		if (card->ext_csd.hs_max_dtr > 52000000 &&
		    host->caps2 & MMC_CAP2_HS200)
			err = mmc_select_hs200(card);
		else if	(host->caps & MMC_CAP_MMC_HIGHSPEED)
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					 EXT_CSD_HS_TIMING, 1,
					 card->ext_csd.generic_cmd6_time);

		if (err && err != -EBADMSG)
			goto free_card;

		if (err) {
			pr_warning("%s: switch to highspeed failed\n",
			       mmc_hostname(card->host));
			err = 0;
		} else {
			if (card->ext_csd.hs_max_dtr > 52000000 &&
			    host->caps2 & MMC_CAP2_HS200) {
				mmc_card_set_hs200(card);
				mmc_set_timing(card->host,
					       MMC_TIMING_MMC_HS200);
			} else {
				mmc_card_set_highspeed(card);
				mmc_set_timing(card->host, MMC_TIMING_MMC_HS);
			}
		}
	}

	/*
	 * Compute bus speed.
	 */
	max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card) || mmc_card_hs200(card)) {
		if (max_dtr > card->ext_csd.hs_max_dtr)
			max_dtr = card->ext_csd.hs_max_dtr;
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
	}


	mmc_set_clock(host, max_dtr);

	/*
	 * Indicate DDR mode (if supported).
	 */
	if (mmc_card_highspeed(card)) {
		if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_8V)
			&& ((host->caps & (MMC_CAP_1_8V_DDR |
			     MMC_CAP_UHS_DDR50))
				== (MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50)))
				ddr = MMC_1_8V_DDR_MODE;
		else if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_2V)
			&& ((host->caps & (MMC_CAP_1_2V_DDR |
			     MMC_CAP_UHS_DDR50))
				== (MMC_CAP_1_2V_DDR | MMC_CAP_UHS_DDR50)))
				ddr = MMC_1_2V_DDR_MODE;
	}

	/*
	 * Indicate HS200 SDR mode (if supported).
	 */
	if (mmc_card_hs200(card)) {
		u32 ext_csd_bits;
		u32 bus_width = card->host->ios.bus_width;

		/*
		 * For devices supporting HS200 mode, the bus width has
		 * to be set before executing the tuning function. If
		 * set before tuning, then device will respond with CRC
		 * errors for responses on CMD line. So for HS200 the
		 * sequence will be
		 * 1. set bus width 4bit / 8 bit (1 bit not supported)
		 * 2. switch to HS200 mode
		 * 3. set the clock to > 52Mhz <=200MHz and
		 * 4. execute tuning for HS200
		 */
		if ((host->caps2 & MMC_CAP2_HS200) &&
		    card->host->ops->execute_tuning) {
			mmc_host_clk_hold(card->host);
			err = card->host->ops->execute_tuning(card->host,
				MMC_SEND_TUNING_BLOCK_HS200);
			mmc_host_clk_release(card->host);
		}
		if (err) {
			pr_warning("%s: tuning execution failed\n",
				   mmc_hostname(card->host));
			goto err;
		}

		ext_csd_bits = (bus_width == MMC_BUS_WIDTH_8) ?
				EXT_CSD_BUS_WIDTH_8 : EXT_CSD_BUS_WIDTH_4;
		err = mmc_select_powerclass(card, ext_csd_bits, ext_csd);
		if (err)
			pr_warning("%s: power class selection to bus width %d"
				   " failed\n", mmc_hostname(card->host),
				   1 << bus_width);
	}

	/*
	 * Activate wide bus and DDR (if supported).
	 */
	if (!mmc_card_hs200(card) &&
	    (card->csd.mmca_vsn >= CSD_SPEC_VER_4) &&
	    (host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA))) {
		static unsigned ext_csd_bits[][2] = {
			{ EXT_CSD_BUS_WIDTH_8, EXT_CSD_DDR_BUS_WIDTH_8 },
			{ EXT_CSD_BUS_WIDTH_4, EXT_CSD_DDR_BUS_WIDTH_4 },
			{ EXT_CSD_BUS_WIDTH_1, EXT_CSD_BUS_WIDTH_1 },
		};
		static unsigned bus_widths[] = {
			MMC_BUS_WIDTH_8,
			MMC_BUS_WIDTH_4,
			MMC_BUS_WIDTH_1
		};
		unsigned idx, bus_width = 0;

		if (host->caps & MMC_CAP_8_BIT_DATA)
			idx = 0;
		else
			idx = 1;
		for (; idx < ARRAY_SIZE(bus_widths); idx++) {
			bus_width = bus_widths[idx];
			if (bus_width == MMC_BUS_WIDTH_1)
				ddr = 0; /* no DDR for 1-bit width */
			err = mmc_select_powerclass(card, ext_csd_bits[idx][0],
						    ext_csd);
			if (err)
				pr_warning("%s: power class selection to "
					   "bus width %d failed\n",
					   mmc_hostname(card->host),
					   1 << bus_width);

			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					 EXT_CSD_BUS_WIDTH,
					 ext_csd_bits[idx][0],
					 card->ext_csd.generic_cmd6_time);
			if (!err) {
				mmc_set_bus_width(card->host, bus_width);

				/*
				 * If controller can't handle bus width test,
				 * compare ext_csd previously read in 1 bit mode
				 * against ext_csd at new bus width
				 */
				if (!(host->caps & MMC_CAP_BUS_WIDTH_TEST))
					err = mmc_compare_ext_csds(card,
						bus_width);
				else
					err = mmc_bus_test(card, bus_width);
				if (!err)
					break;
			}
		}

		if (!err && ddr) {
			err = mmc_select_powerclass(card, ext_csd_bits[idx][1],
						    ext_csd);
			if (err)
				pr_warning("%s: power class selection to "
					   "bus width %d ddr %d failed\n",
					   mmc_hostname(card->host),
					   1 << bus_width, ddr);

			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					 EXT_CSD_BUS_WIDTH,
					 ext_csd_bits[idx][1],
					 card->ext_csd.generic_cmd6_time);
		}
		if (err) {
			pr_warning("%s: switch to bus width %d ddr %d "
				"failed\n", mmc_hostname(card->host),
				1 << bus_width, ddr);
			goto free_card;
		} else if (ddr) {
			/*
			 * eMMC cards can support 3.3V to 1.2V i/o (vccq)
			 * signaling.
			 *
			 * EXT_CSD_CARD_TYPE_DDR_1_8V means 3.3V or 1.8V vccq.
			 *
			 * 1.8V vccq at 3.3V core voltage (vcc) is not required
			 * in the JEDEC spec for DDR.
			 *
			 * Do not force change in vccq since we are obviously
			 * working and no change to vccq is needed.
			 *
			 * WARNING: eMMC rules are NOT the same as SD DDR
			 */
			if (ddr == MMC_1_2V_DDR_MODE) {
				err = mmc_set_signal_voltage(host,
					MMC_SIGNAL_VOLTAGE_120, 0);
				if (err)
					goto err;
			}
			mmc_card_set_ddr_mode(card);
			mmc_set_timing(card->host, MMC_TIMING_UHS_DDR50);
			mmc_set_bus_width(card->host, bus_width);
		}
	}

	/*
	 * Enable HPI feature (if supported)
	 */
	if (card->ext_csd.hpi) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HPI_MGMT, 1,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;
		if (err) {
			pr_warning("%s: Enabling HPI failed\n",
				   mmc_hostname(card->host));
			err = 0;
		} else
			card->ext_csd.hpi_en = 1;
	}

	/*
	 * If cache size is higher than 0, this indicates
	 * the existence of cache and it can be turned on.
	 */
	if ((host->caps2 & MMC_CAP2_CACHE_CTRL) &&
			card->ext_csd.cache_size > 0) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_CACHE_CTRL, 1,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;

		/*
		 * Only if no error, cache is turned on successfully.
		 */
		if (err) {
			pr_warning("%s: Cache is supported, "
					"but failed to turn on (%d)\n",
					mmc_hostname(card->host), err);
			card->ext_csd.cache_ctrl = 0;
			err = 0;
		} else {
			card->ext_csd.cache_ctrl = 1;
		}
	}

	if (!oldcard)
		host->card = card;

#ifdef MTK_EMMC_SUPPORT_OTP 
    /* enable hc erase grp size */
    printk("switch to hc erase grp size\n");
    err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
            EXT_CSD_ERASE_GROUP_DEF, 1, 0);
    card->ext_csd.erase_group_def = 1;
#endif

	mmc_free_ext_csd(ext_csd);
	return 0;

free_card:
	if (!oldcard)
		mmc_remove_card(card);
err:
	mmc_free_ext_csd(ext_csd);

	return err;
}

/*
 * Host is being removed. Free up the current card.
 */
static void mmc_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_remove_card(host->card);
	host->card = NULL;
}

/*
 * Card detection - card is alive.
 */
static int mmc_alive(struct mmc_host *host)
{
	return mmc_send_status(host->card, NULL);
}

/*
 * Card detection callback from host.
 */
 //  ���mmc���ߵ�mmc type card�Ƿ�γ�  
static void mmc_detect(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);

	/*
	 * Just check if our card has been removed.
	 */ 
	 /* ���card�Ƿ񱻰γ� */  
	err = _mmc_detect_card_removed(host);

	mmc_release_host(host);  
	
 /* card��û�б��γ���˵�������쳣�ˣ����card��rpm״̬Ϊsuspend */
	if (err) {   /* cardȷʵ���γ��������ͷ�card */
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
	}
}

/*
 * Suspend callback from host.
 */
static int mmc_suspend(struct mmc_host *host)
{
	int err = 0;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	if (mmc_card_can_sleep(host)) {
		err = mmc_card_sleep(host);
		if (!err)
			mmc_card_set_sleep(host->card);
	} else if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~(MMC_STATE_HIGHSPEED | MMC_STATE_HIGHSPEED_200);
	mmc_release_host(host);

	return err;
}

/*
 * Resume callback from host.
 *
 * This function tries to determine if the same card is still present
 * and, if so, restore all state to it.
 */
static int mmc_resume(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	if (mmc_card_is_sleep(host->card)) {
		err = mmc_card_awake(host);
		mmc_card_clr_sleep(host->card);
	} else
		err = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	return err;
}

static int mmc_power_restore(struct mmc_host *host)
{
	int ret;

	host->card->state &= ~(MMC_STATE_HIGHSPEED | MMC_STATE_HIGHSPEED_200);
	mmc_card_clr_sleep(host->card);
	mmc_claim_host(host);
	ret = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	return ret;
}

static int mmc_sleep(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {
		err = mmc_card_sleepawake(host, 1);
		if (err < 0)
			pr_debug("%s: Error %d while putting card into sleep",
				 mmc_hostname(host), err);
	}

	return err;
}

// ʹmmc�����ϵ�mmc type card�˳�sleep state 
static int mmc_awake(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {      // �жϰ汾�Ƿ����3  
		err = mmc_card_sleepawake(host, 0);
		         // ����CMD5ָ�MMC_SLEEP_AWAKE������Ϊ0����ʾ�˳�sleep state.���������Ϊ1���ǽ���sleep state��
                 // ���֮�󣬸�MMC card��sleep state����standbyģʽ
		if (err < 0)
			pr_debug("%s: Error %d while awaking sleeping card",
				 mmc_hostname(host), err);
	}

	return err;
}

//  struct mmc_bus_ops��ʾmmc host�������ϵĲ������ϣ���host��card �豸��������mmc type card��sd type card��Ӧ�Ĳ��������ǲ�һ���ġ�
//  mmc_ops��mmc_ops_unsafe���ʾmmc type card������host�������ߵĲ�������
//  mmc_ops�ṩ�˲��ֺ�mmc type cardЭ����ز�������Щ��������mmc.c��mmc�ĳ�ʼ�������б�ʹ�õ�
//  ��Щ�������ᷢ��mmc������˻����mmc core��ģ���mmc����API������mmc core�н���˵��
static const struct mmc_bus_ops mmc_ops = {
	.awake = mmc_awake,
	.sleep = mmc_sleep,
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = NULL,
	.resume = NULL,
	.power_restore = mmc_power_restore,
	.alive = mmc_alive,
};

static const struct mmc_bus_ops mmc_ops_unsafe = {
	.awake = mmc_awake,        // ʹmmc�����ϵ�mmc type card�˳�sleep state 
	.sleep = mmc_sleep,        // ʹmmc���ߵ�mmc type card����sleep state
	.remove = mmc_remove,      // �ͷ�mmc type card
	.detect = mmc_detect,      // ���mmc���ߵ�mmc type card�Ƿ�γ�
	.suspend = mmc_suspend,     // suspend��mmc�����ϵ�mmc type card��ע�ⲻ������ʹcard����sleep state�������clock�Լ�mmc cache���в���
	.resume = mmc_resume,        // resume��mmc�����ϵ�mmc type card
	.power_restore = mmc_power_restore, // �ָ�mmc�����ϵ�mmc type card�ĵ�Դ״̬ 
	.alive = mmc_alive,  // ���mmc�����ϵ�mmc type card״̬�Ƿ�����
};   //  .change_bus_speed = mmc_change_bus_speed,   // �޸�mmc����ʱ��Ƶ��

/*******

mmc_ops_unsafe��mmc_ops�����������Ƿ�ʵ��suspend��resume������ 
����card�����Ƴ���host��˵����Ҫʹ��mmc_ops_unsafe���mmc_bus_ops��֧��suspend��resume�� 
֮����������ע���в���˵��mmc���ߣ���Ϊ��ǿ��Ӧ�ú�mmc_bus�����������ֿ����������mmc�������������Ǻ�host controllerֱ��������ġ�

*********/

static void mmc_attach_bus_ops(struct mmc_host *host)
{
	const struct mmc_bus_ops *bus_ops;

	if (!mmc_card_is_removable(host))
		bus_ops = &mmc_ops_unsafe;
	else
		bus_ops = &mmc_ops;
	mmc_attach_bus(host, bus_ops);
}

/*
 * Starting point for MMC card init.
 */

// �ṩ��mmc core��ģ��ʹ�ã����ڰ�card��host bus�ϣ�Ҳ����card��host�İ󶨣�
// ͨ��mmc_host��ȡmmc type card��Ϣ����ʼ��mmc_card�������в��������������ע�ᵽmmc_bus��
/******
��Ҫ������
��������ģʽ
ѡ��һ��card��host��֧�ֵ���͹�����ѹ
���ڲ�ͬtype��card����Ӧmmc�����ϵĲ���Э��Ҳ����������ͬ��������Ҫ������Ӧ�����߲������ϣ�mmc_host->bus_ops��
��ʼ��cardʹ����빤��״̬��mmc_init_card��
Ϊcard�����Ӧ��mmc_card����ע�ᵽmmc_bus�У�mmc_add_card������ο���mmc core����busģ��˵������*******/

int mmc_attach_mmc(struct mmc_host *host)
{
	int err;
	u32 ocr;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	/* Set correct bus mode for MMC before attempting attach */
	if (!mmc_host_is_spi(host))  /* �ڳ���ƥ��֮ǰ����������ȷ������ģʽ */
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);

	err = mmc_send_op_cond(host, 0, &ocr); /* ��ȡcard��ocr�Ĵ��� */
	
	                                  // ����CMD1���MMC_SEND_OP_COND�������Ҳ���Ϊ0
			   // �����ȡOCR��Operation condition register��32λ��OCR�������豸֧�ֵĹ�����ѹ���洢��ocr������
// ���Host��IO��ѹ�ɵ������ǵ���ǰ��Ҫ��ȡOCR��Ϊ�˲�ʹ�������Inactive State�����Ը�MMC�����Ͳ���������CMD1���������Խ���ȡOCR�Ĵ�����������ı俨��״̬��
	if (err)
		return err;

	
	/* ���ڲ�ͬtype��card����Ӧmmc�����ϵĲ���Э��Ҳ����������ͬ */
	/* ������������mmc_host�����߲������ϣ�Ϊmmc_ops_unsafe����mmc_ops�������Ѿ�˵�� */
	mmc_attach_bus_ops(host); // ����host->bus_ops��Ҳ���ǻ�Ϊhost��busѡ��һ��������������non-removable��host��˵�������ӦӦ��Ϊmmc_ops_unsafe
	if (host->ocr_avail_mmc)  /* Ϊcardѡ��һ��HOST��card��֧�ֵ���͵�ѹ */
		host->ocr_avail = host->ocr_avail_mmc;   // ѡ��mmc�Ŀ���ocrֵ��Ϊhost��ocr_availֵ

	/*
	 * We need to get OCR a different way for SPI.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_read_ocr(host, 1, &ocr);
		if (err)
			goto err;
	}

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		pr_warning("%s: MMC card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));  // �ڱ�׼MMCЭ���У�OCR�Ĵ�����bit6-0λ�����ڱ���λ��������ʹ�ã����������Ӧ��������
		ocr &= ~0x7F;
	}

	host->ocr = mmc_select_voltage(host, ocr);  // ͨ��OCR�Ĵ���ѡ��һ��HOST��card��֧�ֵ���͵�ѹ

	/*
	 * Can we support the voltage of the card?
	 */
	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}

	/*
	 * Detect and init the card.
	 */

	/* ����mmc_init_card��ʼ����mmc type card�������Ǻ��ĺ��������������˵�� */
	err = mmc_init_card(host, host->ocr, NULL);  // ��ʼ����mmc type card����Ϊ�����ͳ�ʼ��һ����Ӧ��mmc_card
	if (err)
		goto err;
	
	/* �����䵽��mmc_cardע�ᵽmmc_bus�� */
	mmc_release_host(host);  // ���ͷŵ�host����������mmc_add_card�л��ȡ���host  
	err = mmc_add_card(host->card);// ���õ�mmc_add_card����cardע�ᵽ�豸����ģ���С�
                       // ��ʱ���mmc_card�͹�����mmc_bus�ϣ����mmc_bus�ϵ�block����mmc driverƥ��������������ѧϰmmc card driver��ʱ����˵����
	
#ifdef MTK_EMMC_SUPPORT
	host->card_init_complete(host);
#endif
	mmc_claim_host(host);  // �ٴ�����host  
	if (err)
		goto remove_card;

	/* eMMC write protect operation */
	if(host->card->type == MMC_TYPE_MMC)
	{
		err = mmc_wp_check(host);
		if(err)	
		{
			pr_err("call mmc_wp_check() fail ... \n");
		}
	}

	return 0;

remove_card:
	mmc_release_host(host);
	mmc_remove_card(host->card);
	mmc_claim_host(host);
	host->card = NULL;
err:
	mmc_detach_bus(host);

	pr_err("%s: error %d whilst initialising MMC card\n",
		mmc_hostname(host), err);

	return err;
}

/*****************
������mmc_add_card֮��mmc_card�͹�����mmc_bus�ϣ����mmc_bus�ϵ�block��mmc_driver��ƥ��������
��Ӧblock��mmc_driver���ͻ����probe������card��ʵ��card��ʵ�ʹ��ܣ�Ҳ���Ǵ洢�豸�Ĺ��ܣ�����Խӵ����豸��ϵͳ�С�
������ѧϰmmc card driver��ʱ����˵��
**********************/





/* eMMC write protection function */
void reverse_buf(u8* buf, u32 len)
{
	u32 i = 0, j = 0;
	u8 temp = 0;


	for(i = 0, j = len - 1; i < j; i++, j--)
	{
		temp = buf[i];
		buf[i] = buf[j];
		buf[j] = temp;
	}
}


unsigned int u64_to_u32(unsigned long long val, unsigned int* val_high, unsigned int* val_low)
{
	*val_low = (unsigned int)val;
	*val_high = (unsigned int)(val >> 32);

	return 0;
}


u32 mmc_grp_size(struct mmc_host* host)
{
	struct mmc_card* card = NULL;
	u32* resp = NULL;
	u32 wp_grp_size = 0;
	static u8 log_flag = 1;


	BUG_ON(host == NULL);

	card = host->card;
	resp = card->raw_csd;

	if(card->ext_csd.erase_group_def == 1)
	{
		if(log_flag == 1)
		{
			printk(KERN_ALERT"write protect group size is decide by CSD register:\n");
			printk(KERN_ALERT"WP_GRP_SIZE: %d, ERASE_GRP_SIZE = %d, ERASE_GRP_MULT = %d.\n", UNSTUFF_BITS(resp, 32, 5), UNSTUFF_BITS(resp, 42, 5), UNSTUFF_BITS(resp, 37, 5));
		}

		wp_grp_size = 512 * (UNSTUFF_BITS(resp, 32, 5) + 1) * (UNSTUFF_BITS(resp, 42, 5) + 1) * (UNSTUFF_BITS(resp,      37, 5) + 1);
	}
	else
	{
		if(log_flag == 1)
		{
			printk(KERN_ALERT"write protect group size is decide by EXT_CSD register:\n");
			printk(KERN_ALERT"WP_GRP_SIZE: %d, ERASE_GRP_SIZE = %d.\n", card->ext_csd.raw_hc_erase_gap_size, card->ext_csd.raw_hc_erase_grp_size);
		}

		wp_grp_size = 512 * card->ext_csd.raw_hc_erase_gap_size * card->ext_csd.raw_hc_erase_grp_size * 1024;
	}

	if(log_flag == 1)
	{
		printk(KERN_ALERT"write protect group size = 0x%x\n", wp_grp_size);
	}

	if(log_flag == 1)
	{
		log_flag = 0;
	}

	return wp_grp_size;
}


/* parse wp_status to get wp segments */
int mmc_get_wp_segment(struct mmc_host* host, u64 wp_addr, u32 wp_status)
{
	u32 mask = 0x01; /* get single bit of wp_status */
	u32 wp_grp_size = 0;
	u32 i = 0;
	u32 bit_len = 0;
	u32 bit_cur = 0, bit_next = 0, bit_start = 0, bit_end = 0;
	u8 flag = 0;

	wp_grp_size = mmc_grp_size(host);

	bit_len = sizeof(wp_status) * 8;
	bit_start = bit_end = 0;
	for(i = 0; i < bit_len; i++)
	{
		bit_cur = (wp_status & mask) >> i;
		bit_next = (wp_status & (mask << 1)) >> (i + 1);
		if(bit_cur ^ bit_next)
		{
			if(bit_cur == 0)
			{
				/* decide wp start address */
				bit_start = i + 1;
				//flag_start = 1;
			}
			else 
			{
				/* decide wp end address */
				bit_end = i;
				//flag_end = 1;
				flag = 1;
			}

			/* decide wp segment */
			if(flag == 1)
			{
				Start_addr[wp_array_index] = wp_addr + bit_start * wp_grp_size;
				End_addr[wp_array_index] = wp_addr + (bit_end + 1) * wp_grp_size;

				wp_array_index++;

				flag = 0;
			}
		}

		mask = mask << 1;
	}

	return 0;
}


/* back up WP segments */
int mmc_wp_seg_backup()
{
	int i = 0;

	for(i = 0; i < SEG_NUM; i++)
	{
		Start_addr_backup[i] = Start_addr[i];
		End_addr_backup[i] = End_addr[i];
	}
}


/* check wp is existing in specific segments or not from uboot */
int mmc_wp_is_set_in_uboot()
{
	int i = 0;

	for(i = 0; i < SEG_NUM; i++)	
	{
		if(End_addr_backup[i] > Start_addr_backup[i])
		{
			return 1;
		}
	}

	return 0;
}


/* check wp is existing in specific segments or not from kernel */
int mmc_wp_is_set_in_kernel()
{
	int i = 0;

	for(i = 0; i < SEG_NUM; i++)	
	{
		if(End_addr[i] > Start_addr[i])
		{
			return 1;
		}
	}

	return 0;
}


/* clear wp at specific segments from kernel */
int mmc_wp_clear_segments()
{
	int i = 0;

	for(i = 0; i < SEG_NUM; i++)	
	{
		End_addr[i] = Start_addr[i] = 0;
	}

	return 0;
}


/* reset wp at specific segments from backup */
int mmc_wp_reset_segments()
{
	int i = 0;

	for(i = 0; i < SEG_NUM; i++)	
	{
		Start_addr[i] = Start_addr_backup[i];
		End_addr[i] = End_addr_backup[i];
	}

	return 0;
}


/* set or clear eMMC WP group */
int mmc_wp_grp(struct mmc_host* host, u64 addr_grp, u32 flag)
{
	int err = 0;
	struct mmc_command cmd = {0};
	u32 wp_grp_size = 0;


	BUG_ON(host == NULL);

	wp_grp_size = mmc_grp_size(host);
	if(addr_grp % wp_grp_size)
	{
		pr_err("addr for mmc_wp_grp not aligned.\n");
		return -EINVAL;
	}

	if(flag == 1)
	{
		cmd.opcode = MMC_SET_WRITE_PROT;
		cmd.arg = (host->caps & MMC_CAP_MMC_HIGHSPEED) ? (addr_grp >> 9) : addr_grp;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	if(flag == 0)
	{
		cmd.opcode = MMC_CLR_WRITE_PROT;
		cmd.arg = (host->caps & MMC_CAP_MMC_HIGHSPEED) ? (addr_grp >> 9) : addr_grp;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if(err)
	{
		return err;
	}

	return 0;
}


/* enable or disable eMMC WP at specific segments */
int mmc_wp_enable(struct mmc_host* host, u32 flag)
{
	u32 seg_num = 0;
	u64 addr = 0, addr_start = 0, addr_end = 0;
	u32 wp_grp_size = 0;
	int err = 0;
	u32 i = 0;


	wp_grp_size = mmc_grp_size(host);

	if(flag == 1)
	{
		for(seg_num = 0; seg_num < SEG_NUM; seg_num++)	
		{
			addr_start = Start_addr[seg_num];
			addr_end = End_addr[seg_num];

			for(addr = addr_start; addr < addr_end; addr += wp_grp_size)
			{
				err = mmc_wp_grp(host, addr, 1);
				if(err)
				{
					pr_err("call mmc_wp_grp() for setting WP fail ...\n");
					return err;
				}
			}
		}
	}

	if(flag == 0)
	{
		for(seg_num = 0; seg_num < SEG_NUM; seg_num++)	
		{
			addr_start = Start_addr[seg_num];
			addr_end = End_addr[seg_num];

			for(addr = addr_start; addr < addr_end; addr += wp_grp_size)
			{
				err = mmc_wp_grp(host, addr, 0);
				if(err)
				{
					pr_err("call mmc_wp_grp() for clearing WP fail ...\n");
					return err;
				}
			}
		}

		/* once WP is clear, global array: Start_addr and End_addr should be reset to 0 */
		for(i = 0; i < SEG_NUM; i++)		
		{
			Start_addr[i] = 0;
			End_addr[i] = 0;
		}
	}

	return 0;
}


/* send command for eMMC WP status and type */
int mmc_get_wp_status_type(struct mmc_host *host, u32 opcode, u64 wp_addr, u8* buf, unsigned int len)
{
	struct mmc_card *card = NULL;
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	void *data_buf = NULL;
	u32 val_high = 0, val_low = 0;
	u64 temp = 0;


	BUG_ON(host == NULL);
	
	card = host->card;
	
	data_buf = kmalloc(len, GFP_KERNEL);
	if (data_buf == NULL)
		return -ENOMEM;

	memset(data_buf, 0, len);

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	
	cmd.arg = (host->caps & MMC_CAP_MMC_HIGHSPEED) ? (wp_addr >> 9) : wp_addr;

	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, data_buf, len);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(host, &mrq);

	memcpy(buf, data_buf, len);

	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}


/* get eMMC WP information (status and type), this function will call mmc_get_wp_status_type*/
int mmc_get_wp_info(struct mmc_host* host, u64 wp_addr, u32* wp_status, u64* wp_type)
{
	u32 wp_grp_size = 0;
	u32 len_status = 4, len_type = 8;
	int err = 0;
	void* buf_status = NULL;
	void* buf_type = NULL;
	
	
	buf_status = kmalloc(len_status, GFP_KERNEL);
	if(buf_status == NULL)
	{
		return -ENOMEM;
	}
	memset(buf_status, 0, len_status);

	buf_type = kmalloc(len_type, GFP_KERNEL);
	if(buf_type == NULL)
	{
		return -ENOMEM;
	}
	memset(buf_type, 0, len_type);

	BUG_ON(host == NULL);


	if(wp_status)
	{
		err = mmc_get_wp_status_type(host, MMC_SEND_WRITE_PROT, wp_addr, buf_status, len_status);
		if(err)
		{
			pr_err("send CMD30 failed.\n");
			kfree(buf_status);
			return -ENOMEM;
		}

		memcpy(wp_status, buf_status, len_status);
		reverse_buf((u8*)wp_status, len_status);
	}

	if(wp_type)
	{
		err = mmc_get_wp_status_type(host, MMC_SEND_WRITE_PROT_TYPE, wp_addr, buf_type, len_type);
		if(err)
		{
			pr_err("send CMD31 failed.\n");
			kfree(buf_type);
			return -ENOMEM;
		}

		memcpy(wp_type, buf_type, len_type);
		reverse_buf((u8*)wp_type, len_type);
	}


	kfree(buf_status);
	kfree(buf_type);


	return 0;
}



/* check eMMC WP status and type */
int mmc_wp_check(struct mmc_host* host)
{
	u64 wp_addr = 0, start_addr = 0, end_addr = 0;
	u32 addr_high = 0, addr_low = 0;
	u32 wp_status = 0;
	u64 wp_type = 0;
	u32 type_high = 0, type_low = 0;
	u32 wp_grp_size = 0;
	u32 wp_grp_num = 0;	/* value: 0~31 */
	int err = 0;


	BUG_ON(host == NULL);

	end_addr = host->card->ext_csd.sectors * 512;

	wp_grp_size = mmc_grp_size(host);

	if(start_addr % wp_grp_size)
	{
		start_addr -= start_addr % wp_grp_size; /* grp_size aligned */
	}
	if(end_addr % wp_grp_size)
	{
		end_addr -= end_addr % wp_grp_size; /* grp_size aligned */
	}
	
	wp_grp_num = 0;
	for(wp_addr = start_addr; wp_addr < end_addr; wp_addr += wp_grp_size)
	{
		addr_high = addr_low = 0;
		u64_to_u32(wp_addr, &addr_high, &addr_low);

		wp_status = 0;
		wp_type = 0;
		err = mmc_get_wp_info(host, wp_addr, &wp_status, &wp_type);
		if(err)
		{
			pr_err("mmc_get_wp_info() fail.\n");
			return err;
		}

		if(wp_status != 0)
		{
			type_high = type_low = 0;
			u64_to_u32(wp_type, &type_high, &type_low);

			printk(KERN_ALERT"eMMC_wp: mmc_get_wp_status addr:0x%x%08x wp_status:0x%08x wp_type:0x%x%08x\n", addr_high, addr_low, wp_status, type_high, type_low);	

			/* get wp segments */
			if(wp_grp_num == 0)
			{
				if(wp_array_index < SEG_NUM)
				{
					mmc_get_wp_segment(host, wp_addr, wp_status);
				}
			}
		}

		wp_grp_num++;
		if(wp_grp_num == 32)
		{
			wp_grp_num = 0;	/* next address segnemts specified by wp_status (32 wp group size) */
		}
	}


	if(wp_seg_backup == 0)	/* need to back up */
	{
		mmc_wp_seg_backup();
		wp_seg_backup = 1;
	}


	return 0;
}
