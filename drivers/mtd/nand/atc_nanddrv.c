#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <mach/cache_operation.h>

#define atc_nand_suspend NULL
#define atc_nand_resume NULL
extern int atc_nand_remove(struct platform_device *pdev);
extern int atc_nand_probe(struct platform_device *pdev);

static struct platform_driver atc_nand_driver = {
    .probe          = atc_nand_probe,
    .remove       = atc_nand_remove,
    .suspend     = atc_nand_suspend,
    .resume       = atc_nand_resume,
    .driver          = 
    {
        .name      = "atc_nand",
        .owner     = THIS_MODULE,
    },
};

module_platform_driver(atc_nand_driver);

#define IC_VERIFY_NAND_LINUX_DRIVER    1

#if (IC_VERIFY_NAND_LINUX_DRIVER ==1) // mtk40184 add, for use Nand Linux Native driver emulation & verification IC.
extern bool _fgUsingDMA;
extern bool _fgECCSWCorrect;

EXPORT_SYMBOL(_fgUsingDMA);
EXPORT_SYMBOL(_fgECCSWCorrect);
#endif

MODULE_LICENSE ("GPL");
