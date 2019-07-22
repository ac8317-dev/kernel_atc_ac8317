/*
 * drivers/mtd/nand/nand_util.c
 *
 * Copyright (C) 2006 by Weiss-Electronic GmbH.
 * All rights reserved.
 *
 * @author:	Guido Classen <clagix@gmail.com>
 * @descr:	NAND Flash support
 * @references: borrowed heavily from Linux mtd-utils code:
 *		flash_eraseall.c by Arcom Control System Ltd
 *		nandwrite.c by Steven J. Hill (sjhill@realitydiluted.com)
 *			       and Thomas Gleixner (tglx@linutronix.de)
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/nand_bch.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>

typedef struct mtd_info nand_info_t;
struct mtd_info *g_mtd = NULL;

/**
 * get_len_incl_bad
 *
 * Check if length including bad blocks fits into device.
 *
 * @param nand NAND device
 * @param offset offset in flash
 * @param length image length
 * @return image length including bad blocks
 */
static size_t get_len_incl_bad (nand_info_t *nand, loff_t offset,
				const size_t length)
{
	size_t len_incl_bad = 0;
	size_t len_excl_bad = 0;
	size_t block_len;

	while (len_excl_bad < length) {
		block_len = nand->erasesize - (offset & (nand->erasesize - 1));

		if (!nand->_block_isbad (nand, offset & ~(nand->erasesize - 1)))
			len_excl_bad += block_len;

		len_incl_bad += block_len;
		offset       += block_len;

		if ((offset + len_incl_bad) >= nand->size)
			break;
	}

	return len_incl_bad;
}



/*************************************************************************/

/**
 * nand_erase:
 *
 * erase NAND flash.
 *
 * @param nand  	NAND device
 * @param offset	offset in flash
 * @param length	erase length
 * @return		0 in case of success
 */
static int nand_erase(nand_info_t *nand, loff_t addr, size_t length)
{
	struct erase_info instr;
	memset(&instr, 0, sizeof(instr));
	instr.mtd = nand;
	instr.len = length;
	instr.addr = addr;
	return nand->_erase(nand, &instr);
}

/*************************************************************************/

/**
 * nand_write_skip_bad:
 *
 * Write image to NAND flash.
 * Blocks that are marked bad are skipped and the is written to the next
 * block instead as long as the image is short enough to fit even after
 * skipping the bad blocks.
 *
 * @param nand  	NAND device
 * @param offset	offset in flash
 * @param length	buffer length
 * @param buf           buffer to read from
 * @return		0 in case of success
 */
static int nand_write_skip_bad(nand_info_t *nand, loff_t offset, size_t *length,
			u_char *buffer)
{
	int rval;
	size_t left_to_write = *length;
	size_t len_incl_bad;
	u_char *p_buffer = buffer;

	/* Reject writes, which are not page aligned */
	if ((offset & (nand->writesize - 1)) != 0 ||
	    (*length & (nand->writesize - 1)) != 0) {
		printk(KERN_ALERT "Attempt to write non page aligned data\n");
		return -EINVAL;
	}

	len_incl_bad = get_len_incl_bad (nand, offset, *length);

	if ((offset + len_incl_bad) >= nand->size) {
		printk(KERN_ALERT "Attempt to write outside the flash area\n");
		return -EINVAL;
	}

	if (len_incl_bad == *length) {
		rval = nand->_write (nand, offset, left_to_write, length, buffer);
		if (rval != 0)
			printk(KERN_ALERT "NAND write to offset %llx failed %d\n",
				offset, rval);

		return rval;
	}

	while (left_to_write > 0) {
		size_t block_offset = offset & (nand->erasesize - 1);
		size_t write_size;

		if (nand->_block_isbad (nand, offset & ~(nand->erasesize - 1))) {
			printk(KERN_ALERT "Skip bad block 0x%08llx\n",
				offset & ~(nand->erasesize - 1));
			offset += nand->erasesize - block_offset;
			left_to_write -= nand->erasesize - block_offset;
			continue;
		}

		if (left_to_write < (nand->erasesize - block_offset))
			write_size = left_to_write;
		else
			write_size = nand->erasesize - block_offset;

		rval = nand->_write (nand, offset, write_size, &write_size, p_buffer);
		if (rval != 0) {
			printk(KERN_ALERT "NAND write to offset %llx failed %d\n",
				offset, rval);
			*length -= left_to_write;
			return rval;
		}

		left_to_write -= write_size;
		offset        += write_size;
		p_buffer      += write_size;
	}

	return 0;
}

/**
 * nand_read_skip_bad:
 *
 * Read image from NAND flash.
 * Blocks that are marked bad are skipped and the next block is readen
 * instead as long as the image is short enough to fit even after skipping the
 * bad blocks.
 *
 * @param nand NAND device
 * @param offset offset in flash
 * @param length buffer length, on return holds remaining bytes to read
 * @param buffer buffer to write to
 * @return 0 in case of success
 */
static int nand_read_skip_bad(nand_info_t *nand, loff_t offset, size_t *length,
		       u_char *buffer)
{
	int rval;
	size_t left_to_read = *length;
	size_t len_incl_bad;
	u_char *p_buffer = buffer;

	len_incl_bad = get_len_incl_bad (nand, offset, *length);

	if ((offset + len_incl_bad) >= nand->size) {
		printk(KERN_ALERT "Attempt to read outside the flash area\n");
		return -EINVAL;
	}

	if (len_incl_bad == *length) {
		rval = nand->_read (nand, offset, left_to_read, length, buffer);
		if (!rval || rval == -EUCLEAN)
			return 0;
		printk(KERN_ALERT "NAND read from offset %llx failed %d\n",
			offset, rval);
		return rval;
	}

	while (left_to_read > 0) {
		size_t block_offset = offset & (nand->erasesize - 1);
		size_t read_length;


		if (nand->_block_isbad (nand, offset & ~(nand->erasesize - 1))) {
			printk(KERN_ALERT "Skipping bad block 0x%08llx\n",
				offset & ~(nand->erasesize - 1));
			offset += nand->erasesize - block_offset;
			left_to_read -= nand->erasesize - block_offset;
			continue;
		}

		if (left_to_read < (nand->erasesize - block_offset))
			read_length = left_to_read;
		else
			read_length = nand->erasesize - block_offset;

		rval = nand->_read (nand, offset, read_length, &read_length, p_buffer);
		if (rval && rval != -EUCLEAN) {
			printk(KERN_ALERT "NAND read from offset %llx failed %d\n",
				offset, rval);
			*length -= left_to_read;
			return rval;
		}

		left_to_read -= read_length;
		offset       += read_length;
		p_buffer     += read_length;
	}

	return 0;
}


int mtd_write_metazone(u8 * pbBuffer, u32 u4Size)
{
    int ret;
	int ret2 = 0;
    size_t size = u4Size;
	if (0 == nand_erase(g_mtd, 0x43200000, 0x200000))
	{
	    ret = nand_write_skip_bad(g_mtd, 0x43200000, &size, pbBuffer);
		if ( 0 != ret)
		{
			printk(KERN_ALERT "[MTZ] mtd_write_metazone faile to write first block\r\n");
	  	}
		else
		{
		    ret2 = 1;
		}
	}
	else
	{
		printk(KERN_ALERT "[MTZ] mtd_write_metazone faile to erase first block\r\n");
	}
	size = u4Size;
	if (0 == nand_erase(g_mtd, 0x43400000, 0x200000))
	{
		ret = nand_write_skip_bad(g_mtd, 0x43400000, &size, pbBuffer);
		if ( 0 != ret)
		{
			printk(KERN_ALERT "[MTZ] mtd_write_metazone faile to write second block\r\n");
		}
		else
		{
			ret2 = 1;
		}
	}
	else
	{
		printk(KERN_ALERT "[MTZ] mtd_write_metazone faile to erase first block\r\n");
	}

	return (ret2);
}

u32 mtd_read_metazone(u8 *pbBuffer, u32 u4Size)
{
    return (0);
}

int mtd_write_logo(u8 * pbBuffer, u32 u4Size)
{
    int ret;
	int ret2 = 0;
    size_t size = u4Size;
	if (0 == nand_erase(g_mtd, 0x800000, 0x400000))
	{
	    ret = nand_write_skip_bad(g_mtd, 0x800000, &size, pbBuffer);
		if ( 0 != ret)
		{
			printk(KERN_ALERT "[MTZ] mtd_write_logo faile to write data\r\n");
	  	}
		else
		{
		    ret2 = 1;
		}
	}
	else
	{
		printk(KERN_ALERT "[MTZ] mtd_write_logo faile to erase data\r\n");
	}
    return (ret2);
}

u32 mtd_read_logo(u8 *pbBuffer, u32 u4Size)
{
    return (0);
}

EXPORT_SYMBOL(mtd_write_metazone);
EXPORT_SYMBOL(mtd_read_metazone);
EXPORT_SYMBOL(mtd_read_logo);
EXPORT_SYMBOL(mtd_write_logo);


