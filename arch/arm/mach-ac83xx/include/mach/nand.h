/*
 *  linux/arch/arm/mach-ac83xx/include/mach/nand.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */

/* system header files */


#ifndef __ARCH_ARM_MTK_NAND_H
#define __ARCH_ARM_MTK_NAND_H

#include <linux/mtd/nand.h>

struct mtk_nand_pdata {
	/* NAND chip information */
	unsigned short		page_size;
	unsigned short		data_width;

	/* RD/WR strobe delay timing information, all times in SCLK cycles */
	unsigned short		rd_dly;
	unsigned short		wr_dly;

	/* NAND MTD partition information */
	int                     nr_partitions;
	struct mtd_partition    *partitions;
};

#endif	/* __ARCH_ARM_DAVINCI_NAND_H */
