/*
 *  linux/drivers/mmc/core/bus.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC card bus driver model
 */

#include <linux/export.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/pm_runtime.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include "core.h"
#include "sdio_cis.h"
#include "bus.h"

#define to_mmc_driver(d)	container_of(d, struct mmc_driver, drv)

static ssize_t mmc_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mmc_card *card = mmc_dev_to_card(dev);

	switch (card->type) {
	case MMC_TYPE_MMC:
		return sprintf(buf, "MMC\n");
	case MMC_TYPE_SD:
		return sprintf(buf, "SD\n");
	case MMC_TYPE_SDIO:
		return sprintf(buf, "SDIO\n");
	case MMC_TYPE_SD_COMBO:
		return sprintf(buf, "SDcombo\n");
	default:
		return -EFAULT;
	}
}

static struct device_attribute mmc_dev_attrs[] = {
	__ATTR(type, S_IRUGO, mmc_type_show, NULL),
	__ATTR_NULL,
};

/*
 * This currently matches any MMC driver to any MMC card - drivers
 * themselves make the decision whether to drive this card in their
 * probe method.
 */
static int mmc_bus_match(struct device *dev, struct device_driver *drv)
{
	return 1;   //  直接返回 1   表示任意驱动都能和 mmc 卡设备  成功匹配 
}                // 无条件返回1，说明挂载mmc bus上的device（mmc_card）和driver（mmc_driver）是无条件匹配的

static int
mmc_bus_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct mmc_card *card = mmc_dev_to_card(dev);
	const char *type;
	int retval = 0;

	switch (card->type) {
	case MMC_TYPE_MMC:
		type = "MMC";
		break;
	case MMC_TYPE_SD:
		type = "SD";
		break;
	case MMC_TYPE_SDIO:
		type = "SDIO";
		break;
	case MMC_TYPE_SD_COMBO:
		type = "SDcombo";
		break;
	default:
		type = NULL;
	}

	if (type) {
		retval = add_uevent_var(env, "MMC_TYPE=%s", type);
		if (retval)
			return retval;
	}

	retval = add_uevent_var(env, "MMC_NAME=%s", mmc_card_name(card));
	if (retval)
		return retval;

	/*
	 * Request the mmc_block device.  Note: that this is a direct request
	 * for the module it carries no information as to what is inserted.
	 */
	retval = add_uevent_var(env, "MODALIAS=mmc:block");

	return retval;
}

static int mmc_bus_probe(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);   //  调用了 驱动中 的  probe 方法 
	struct mmc_card *card = mmc_dev_to_card(dev);

	return drv->probe(card);                       // 直接调用mmc_driver中的probe操作，对于block.c来说就是mmc_blk_probe
}

static int mmc_bus_remove(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = mmc_dev_to_card(dev);

	drv->remove(card);

	return 0;
}

static int mmc_bus_suspend(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = mmc_dev_to_card(dev);
	int ret = 0;

	if (dev->driver && drv->suspend)
		ret = drv->suspend(card);
	return ret;
}

static int mmc_bus_resume(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = mmc_dev_to_card(dev);
	int ret = 0;

	if (dev->driver && drv->resume)
		ret = drv->resume(card);
	return ret;
}

#ifdef CONFIG_PM_RUNTIME

static int mmc_runtime_suspend(struct device *dev)
{
	struct mmc_card *card = mmc_dev_to_card(dev);

	return mmc_power_save_host(card->host);
}

static int mmc_runtime_resume(struct device *dev)
{
	struct mmc_card *card = mmc_dev_to_card(dev);

	return mmc_power_restore_host(card->host);
}

static int mmc_runtime_idle(struct device *dev)
{
	return pm_runtime_suspend(dev);
}

#endif /* !CONFIG_PM_RUNTIME */

static const struct dev_pm_ops mmc_bus_pm_ops = {
	SET_RUNTIME_PM_OPS(mmc_runtime_suspend, mmc_runtime_resume,
			mmc_runtime_idle)
	SET_SYSTEM_SLEEP_PM_OPS(mmc_bus_suspend, mmc_bus_resume)
};

//  mmc_bus_type代表了mmc虚拟总线  
static struct bus_type mmc_bus_type = {
	.name		= "mmc",                     // 相应会在/sys/bus下生成mmc目录   
	.dev_attrs	= mmc_dev_attrs,             // bus下的device下继承的属性，可以看到/sys/bus/mmc/devices/mmc0:0001/type属性就是这里来的
	.match		= mmc_bus_match,             // 用于mmc bus上device和driver的匹配
	.uevent		= mmc_bus_uevent,            // 当match成功的时候，执行的probe操作
	.probe		= mmc_bus_probe,
	.remove		= mmc_bus_remove,
	.pm		= &mmc_bus_pm_ops,               // 挂在mmc bus上的device的电源管理操作集合    
};

//  用于注册mmc bus（虚拟mmc总线）到设备驱动模型中     相关节点：/sys/bus/mmc   
int mmc_register_bus(void)
{
	return bus_register(&mmc_bus_type);    // 以mmc_bus_type为bus_type注册一条虚拟bus，关于mmc_bus_type上面已经说明过了   
}

void mmc_unregister_bus(void)
{
	bus_unregister(&mmc_bus_type);
}

/**
 *	mmc_register_driver - register a media driver
 *	@drv: MMC media driver
 */
 //  用于注册struct mmc_driver *drv到mmc_bus上 mc_driver就是mmc core抽象出来的card设备driver
int mmc_register_driver(struct mmc_driver *drv)
{
	drv->drv.bus = &mmc_bus_type;      // 通过设置mmc_driver——》device_driver——》bus_type来设置mmc_driver所属bus为mmc_bus    注: mmc_bus_type 代表了 mmc 虚拟总线 
	return driver_register(&drv->drv);  // 这样就将mmc_driver挂在了mmc_bus上了
}                                       //  相关节点：/sys/bus/mmc/drivers

EXPORT_SYMBOL(mmc_register_driver);

/**
 *	mmc_unregister_driver - unregister a media driver
 *	@drv: MMC media driver
 */
void mmc_unregister_driver(struct mmc_driver *drv)
{
	drv->drv.bus = &mmc_bus_type;
	driver_unregister(&drv->drv);
}

EXPORT_SYMBOL(mmc_unregister_driver);

static void mmc_release_card(struct device *dev)
{
	struct mmc_card *card = mmc_dev_to_card(dev);

	sdio_free_common_cis(card);

	if (card->info)
		kfree(card->info);

	kfree(card);
}

/*
 * Allocate and initialise a new MMC card structure.
 */
// 用于分配一个struct mmc_card结构体，创建其于mmc host以及mmc bus之间的关联
struct mmc_card *mmc_alloc_card(struct mmc_host *host, struct device_type *type)
{
	struct mmc_card *card;

	card = kzalloc(sizeof(struct mmc_card), GFP_KERNEL);    // 分配一个mmc_card  
	if (!card)
		return ERR_PTR(-ENOMEM);

	card->host = host;                                      // 关联mmc_card与mmc_host   

	device_initialize(&card->dev);

	card->dev.parent = mmc_classdev(host);     // 设置card的device的parent device为mmc_host的classdev
	                                           // 注册到设备驱动模型中之后，会在/sys/class/mmc_host/mmc0目录下生成相应card的节点，如mmc0:0001
	card->dev.bus = &mmc_bus_type;             // 设置card的bus为mmc_bus_type，这样，mmc_card注册到设备驱动模型中之后就会挂在mmc_bus下      
	                                           // 会在/sys/bus/mmc/devices/目录下生成相应card的节点，如mmc0:0001
	card->dev.release = mmc_release_card;
	card->dev.type = type;                      // 设置device type  

	return card;
}

/*
 * Register a new MMC card with the driver model.
 */
 //  用于注册struct mmc_card到mmc_bus上  
int mmc_add_card(struct mmc_card *card)
{
	int ret;

	
	/* 以下用于打印card的注册信息 */
	
	const char *type;                          
	const char *uhs_bus_speed_mode = "";         // 设置速度模式的字符串，为了后面打印出card信息  
	static const char *const uhs_speeds[] = {
		[UHS_SDR12_BUS_SPEED] = "SDR12 ",
		[UHS_SDR25_BUS_SPEED] = "SDR25 ",
		[UHS_SDR50_BUS_SPEED] = "SDR50 ",
		[UHS_SDR104_BUS_SPEED] = "SDR104 ",
		[UHS_DDR50_BUS_SPEED] = "DDR50 ",
	};


	dev_set_name(&card->dev, "%s:%04x", mmc_hostname(card->host), card->rca);

	switch (card->type) {
	case MMC_TYPE_MMC:
		type = "MMC";
		break;
	case MMC_TYPE_SD:
		type = "SD";
		if (mmc_card_blockaddr(card)) {
			if (mmc_card_ext_capacity(card))
				type = "SDXC";
			else
				type = "SDHC";
		}
		break;
	case MMC_TYPE_SDIO:
		type = "SDIO";
		break;
	case MMC_TYPE_SD_COMBO:
		type = "SD-combo";
		if (mmc_card_blockaddr(card))
			type = "SDHC-combo";
		break;
	default:
		type = "?";
		break;
	}

	if (mmc_sd_card_uhs(card) &&
		(card->sd_bus_speed < ARRAY_SIZE(uhs_speeds)))
		uhs_bus_speed_mode = uhs_speeds[card->sd_bus_speed];

	if (mmc_host_is_spi(card->host)) {
		pr_info("%s: new %s%s%s card on SPI\n",
			mmc_hostname(card->host),
			mmc_card_highspeed(card) ? "high speed " : "",
			mmc_card_ddr_mode(card) ? "DDR " : "",
			type);
	} else {
		pr_info("%s: new %s%s%s%s%s card at address %04x\n",
			mmc_hostname(card->host),
			mmc_card_uhs(card) ? "ultra high speed " :
			(mmc_card_highspeed(card) ? "high speed " : ""),
			(mmc_card_hs200(card) ? "HS200 " : ""),
			mmc_card_ddr_mode(card) ? "DDR " : "",
			uhs_bus_speed_mode, type, card->rca);
	}

		// 在这里会打印出card信息的字符串
		// eg:mmc0: new HS200 MMC card at address 0001

		
/* 设置card的debug节点 */
#ifdef CONFIG_DEBUG_FS
	mmc_add_card_debugfs(card);    // 创建card对应的debug节点，对应路径例如：/sys/kernel/debug/mmc0/mmc0:0001
#endif

	/* 添加到设备驱动模型中 */
	ret = device_add(&card->dev);    // 会创建/sys/bus/mmc/devices/mmc0:0001节点和/sys/class/mmc_host/mmc0/mmc0:0001节点
	if (ret)
		return ret;
	
	/* 设置mmc card的state标识 */
	mmc_card_set_present(card);
	
        // 设置card的MMC_STATE_PRESENT状态
        // #define MMC_STATE_PRESENT    (1<<0)      /* present in sysfs */
        // 表示card已经合入到sysfs中了

		 return 0;
		 
}

/*
 * Unregister a new MMC card with the driver model, and
 * (eventually) free it.
 */
void mmc_remove_card(struct mmc_card *card)
{
#ifdef CONFIG_DEBUG_FS
	mmc_remove_card_debugfs(card);
#endif

	if (mmc_card_present(card)) {
		if (mmc_host_is_spi(card->host)) {
			pr_info("%s: SPI card removed\n",
				mmc_hostname(card->host));
		} else {
			pr_info("%s: card %04x removed\n",
				mmc_hostname(card->host), card->rca);
		}
		device_del(&card->dev);
	}

	put_device(&card->dev);
}

