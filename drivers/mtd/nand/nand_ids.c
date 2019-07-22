/*
 *  drivers/mtd/nandids.c
 *
 *  Copyright (C) 2002 Thomas Gleixner (tglx@linutronix.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/mtd/nand.h>
/*
*	Chip ID list
*
*	Name. ID code, pagesize, chipsize in MegaByte, eraseblock size,
*	options
*
*	Pagesize; 0, 256, 512
*	0	get this information from the extended chip ID
+	256	256 Byte page size
*	512	512 Byte page size
*/
struct nand_flash_dev nand_flash_ids[] = {

#define LP_OPTIONS (NAND_SAMSUNG_LP_OPTIONS | NAND_NO_READRDY | NAND_NO_AUTOINCR)
#define LP_OPTIONS16 (LP_OPTIONS | NAND_BUSWIDTH_16)
	/****Name    		      maf_id dev_id       page  oob  chip   block****/
	{"H27U8G8T2B 1GiB 3,3V 8-bit",0xAD,0xD3,0x14,0xB6,4096, 128,1024, 0x80000, LP_OPTIONS},
	{"H27UBG8T2A 4GiB 3,3V 8-bit",0xAD,0xD7,0x94,0x9A,8192, 448,4096, 0x200000, LP_OPTIONS},
	{"H27UBG8T2B 4GiB 3,3V 8-bit",0xAD,0xD7,0x94,0xDA,8192, 640,4096, 0x200000, LP_OPTIONS},
	{"K9GBG08U0A 4GiB 3,3V 8-bit",0xEC,0xD7,0x94,0x7A,8192, 640,4096, 0x100000, LP_OPTIONS},
	{"MT29F32G08CBACA 4GiB 3,3V 8-bit", 0x2C,0x68,0x04,0x4A,4096, 224,4096, 0x100000, LP_OPTIONS},
	{NULL,}
};

/*
*	Manufacturer ID list
*/
struct nand_manufacturers nand_manuf_ids[] = {
	{NAND_MFR_TOSHIBA, "Toshiba"},
	{NAND_MFR_SAMSUNG, "Samsung"},
	{NAND_MFR_FUJITSU, "Fujitsu"},
	{NAND_MFR_NATIONAL, "National"},
	{NAND_MFR_RENESAS, "Renesas"},
	{NAND_MFR_STMICRO, "ST Micro"},
	{NAND_MFR_HYNIX, "Hynix"},
	{NAND_MFR_MICRON, "Micron"},
	{NAND_MFR_AMD, "AMD"},
	{NAND_MFR_MACRONIX, "Macronix"},
	{0x0, "Unknown"}
};

EXPORT_SYMBOL(nand_manuf_ids);
EXPORT_SYMBOL(nand_flash_ids);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thomas Gleixner <tglx@linutronix.de>");
MODULE_DESCRIPTION("Nand device & manufacturer IDs");
