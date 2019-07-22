/*-----------------------------------------------------------------------------
 * arch/arm/mach-mt85xx/pmem.c
 * Copyright (c) 2006, MediaTek Inc.
 * All rights reserved.
 *
 * Unauthorized use, practice, perform, copy, distribution, reproduction,
 * or disclosure of this information in whole or in part is prohibited.
 *---------------------------------------------------------------------------
 *
 * $Author: daowen.deng $
 * $Date: 2014/07/10 $
 * $Revision: #1 $
 *
 *---------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <asm/mach-types.h>

#define MT_RESMEM_BASE  (128*1024*1024)
#define MT_PMEM_SIZE    (8*1024*1024)
#define MT_ADSP_SIZE    (3*1024*1024)
#define MT_DEMUX_SIZE   (25*1024*1024)
#define MT_VDEC_SIZE    (38*1024*1024)
#define MT_JPEG_SIZE    (7*1024*1024)
#define MT_RLE_SIZE     (2*1024*1024)

struct mt_pmem_setting {
    resource_size_t pmem_start;
    resource_size_t pmem_size;
    resource_size_t pmem_adsp_start;
    resource_size_t pmem_adsp_size;
    resource_size_t pmem_demux_start;
    resource_size_t pmem_demux_size;
    resource_size_t pmem_vdec_start;
    resource_size_t pmem_vdec_size;
    resource_size_t pmem_jpeg_start;
    resource_size_t pmem_jpeg_size;
    resource_size_t pmem_rle_start;
    resource_size_t pmem_rle_size;
    /* TODO: */
    resource_size_t fb_start;
    resource_size_t fb_size;
};

static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_demux_pdata = {
	.name = "pmem_demux",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_vdec_pdata = {
	.name = "pmem_vdec",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_jpeg_pdata = {
	.name = "pmem_jpeg",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_rle_pdata = {
	.name = "pmem_rle",
	.no_allocator = 0,
	.cached = 0,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static struct platform_device pmem_demux_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_demux_pdata },
};

static struct platform_device pmem_vdec_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &pmem_vdec_pdata },
};

static struct platform_device pmem_jpeg_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &pmem_jpeg_pdata },
};

static struct platform_device pmem_rle_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &pmem_rle_pdata },
};

void __init mt_add_mem_devices(struct mt_pmem_setting *setting)
{
	if (setting->pmem_size) {
		pmem_pdata.start = setting->pmem_start;
		pmem_pdata.size = setting->pmem_size;
		platform_device_register(&pmem_device);
	}

	if (setting->pmem_adsp_size) {
		pmem_adsp_pdata.start = setting->pmem_adsp_start;
		pmem_adsp_pdata.size = setting->pmem_adsp_size;
		platform_device_register(&pmem_adsp_device);
	}

	if (setting->pmem_demux_size) {
		pmem_demux_pdata.start = setting->pmem_demux_start;
		pmem_demux_pdata.size = setting->pmem_demux_size;
		platform_device_register(&pmem_demux_device);
	}
 
	if (setting->pmem_vdec_size) {
		pmem_vdec_pdata.start = setting->pmem_vdec_start;
		pmem_vdec_pdata.size = setting->pmem_vdec_size;
		platform_device_register(&pmem_vdec_device);
	}

	if (setting->pmem_jpeg_size) {
		pmem_jpeg_pdata.start = setting->pmem_jpeg_start;
		pmem_jpeg_pdata.size = setting->pmem_jpeg_size;
		platform_device_register(&pmem_jpeg_device);
	}

    if (setting->pmem_rle_size) {
		pmem_rle_pdata.start = setting->pmem_rle_start;
		pmem_rle_pdata.size = setting->pmem_rle_size;
		platform_device_register(&pmem_rle_device);
	}

}

static struct mt_pmem_setting pmem_setting;

#define CALC_PMEM(name, prev, size) \
	pmem_setting.name## _start = pmem_setting.prev## _start+\
		pmem_setting.prev## _size;\
	pmem_setting.name## _size = size;

static int __init mt_pmem_init(void) {

    pmem_setting.pmem_start=MT_RESMEM_BASE;
    pmem_setting.pmem_size=MT_PMEM_SIZE;
    CALC_PMEM(pmem_adsp, pmem, MT_ADSP_SIZE);
    CALC_PMEM(pmem_demux, pmem_adsp, MT_DEMUX_SIZE);
    CALC_PMEM(pmem_vdec, pmem_demux, MT_VDEC_SIZE);
    CALC_PMEM(pmem_jpeg, pmem_vdec, MT_JPEG_SIZE);
    CALC_PMEM(pmem_rle, pmem_jpeg, MT_RLE_SIZE);
	mt_add_mem_devices(&pmem_setting);

    printk(KERN_INFO "PMEM init\n");
	return 0;
}
arch_initcall(mt_pmem_init);

