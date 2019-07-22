/*
 *  atc_mmcdrv.c - Autochips SD/MMC driver
 *
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mmc/core.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <mach/dma.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <mach/ac83xx_memory.h>
#include <linux/kthread.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/atc_storage_partition.h>


#define DRV_NAME            "atc-msdc"
extern struct attribute_group msdc_attr_group;
extern int msdc_drv_probe(struct platform_device *pdev);
extern int msdc_drv_remove(struct platform_device *pdev);
extern int msdc_debug_proc_init(void);
extern void msdc_init_dma_latest_address(void);
extern int msdc_drv_suspend(struct platform_device *pdev, pm_message_t state);
extern int msdc_drv_resume(struct platform_device *pdev);
extern int msdc_drv_shutdown(struct platform_device *pdev);
extern int  autok_init(void);
extern void autok_exit(void);
extern int  emmc_dump_init(void);


static struct kobject *msdc_attr_kobj;

static struct platform_driver mt_msdc_driver = {
    .probe   = msdc_drv_probe,
    .remove  = msdc_drv_remove,
#ifdef CONFIG_PM
    .suspend = msdc_drv_suspend,
    .resume  = msdc_drv_resume,
    .shutdown = msdc_drv_shutdown,
#endif
    .driver  = {
        .name  = DRV_NAME,
        .owner = THIS_MODULE,
    },
};

u64 msdc_get_expdb_offset(unsigned int len, unsigned int offset)
{
	unsigned int i;
	u64 l_start_offset = 0;
	unsigned int l_dest_num = 0;
	
	/* find the offset in emmc */
    for (i = 0; i < PART_NUM; i++) {
        if ('e' == *(PartInfo[i].part_name) && 'x' == *(PartInfo[i].part_name + 1) &&
                'p' == *(PartInfo[i].part_name + 2) && 'd' == *(PartInfo[i].part_name + 3) &&
                'b' == *(PartInfo[i].part_name + 4)){
            l_dest_num = i;
        }
    }

	if (l_dest_num >= PART_NUM) {
        printk("not find in scatter file error!\n");
        return l_start_offset;
    }

	if (PartInfo[l_dest_num].part_size < (len + offset)) {
        printk("read operation oversize!\n");
        return l_start_offset;
    }

	l_start_offset = offset + PartInfo[l_dest_num].part_offset;

	return l_start_offset;
}
EXPORT_SYMBOL(msdc_get_expdb_offset);


/*--------------------------------------------------------------------------*/
/* module init/exit                                                      */
/*--------------------------------------------------------------------------*/
static int __init atc_msdc_init(void)
{
    int ret;

	msdc_attr_kobj = kobject_create_and_add("sddetect", NULL); // "atc_msdc"
	if (!msdc_attr_kobj)
		return -ENOMEM;
	/* Create the files associated with this kobject */
	ret = sysfs_create_group(msdc_attr_kobj, &msdc_attr_group);
	if (ret)
		kobject_put(msdc_attr_kobj);
    ret = platform_driver_register(&mt_msdc_driver);
    if (ret) {
        printk(KERN_ERR DRV_NAME ": Can't register driver");
        return ret;
    }

    printk(KERN_INFO DRV_NAME ": Autochips MSDC Driver\n");

    msdc_debug_proc_init();
    msdc_init_dma_latest_address();
	autok_init();
	emmc_dump_init();
    return 0;
}

static void __exit atc_msdc_exit(void)
{
	autok_exit();
    platform_driver_unregister(&mt_msdc_driver);
}
module_init(atc_msdc_init);
module_exit(atc_msdc_exit);
MODULE_DESCRIPTION("Autochips MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jian Wang <jian.wang@autochips.com>, Qingqi Xia <qingqi.xia@autochips.com>");
