/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *  Copyright (C) 2010 Linus Walleij
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include "core.h"
#include "host.h"
#define CARD_INIT_TIMEOUT (HZ * 5) //5s
#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	kfree(host);
}

//  mmc_host_class代表了mmc_host这个类   
static struct class mmc_host_class = {
	.name		= "mmc_host",                     // 添加到sys文件系统之后，会生成/sys/class/mmc_host这个目录   
	.dev_release	= mmc_host_classdev_release,   // 从mmc_host这个class下release掉某个设备之后要做的对应操作  
};    //  .pm     = &mmc_host_pm_ops,        // 该class下的host的pm电源管理操作

//  注册mmc_host class
int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);   // 以mmc_host_class为class创建一个class，关于mmc_host_class在上述数据结构已经说明过了  
}                                             //相关节点：/sys/class/mmc_host

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

static DEFINE_IDR(mmc_host_idr);
static DEFINE_SPINLOCK(mmc_host_lock);

#ifdef CONFIG_MMC_CLKGATE
static ssize_t clkgate_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	return snprintf(buf, PAGE_SIZE, "%lu\n", host->clkgate_delay);
}

static ssize_t clkgate_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long flags, value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clkgate_delay = value;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return count;
}

/*
 * Enabling clock gating will make the core call out to the host
 * once up and once down when it performs a request or card operation
 * intermingled in any fashion. The driver will see this through
 * set_ios() operations with ios.clock field set to 0 to gate (disable)
 * the block clock, and to the old frequency to enable it again.
 */
static void mmc_host_clk_gate_delayed(struct mmc_host *host)
{
	unsigned long tick_ns;
	unsigned long freq = host->ios.clock;
	unsigned long flags;

	if (!freq) {
		pr_debug("%s: frequency set to 0 in disable function, "
			 "this means the clock is already disabled.\n",
			 mmc_hostname(host));
		return;
	}
	/*
	 * New requests may have appeared while we were scheduling,
	 * then there is no reason to delay the check before
	 * clk_disable().
	 */
	spin_lock_irqsave(&host->clk_lock, flags);

	/*
	 * Delay n bus cycles (at least 8 from MMC spec) before attempting
	 * to disable the MCI block clock. The reference count may have
	 * gone up again after this delay due to rescheduling!
	 */
	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		tick_ns = DIV_ROUND_UP(1000000000, freq);
		ndelay(host->clk_delay * tick_ns);
	} else {
		/* New users appeared while waiting for this work */
		spin_unlock_irqrestore(&host->clk_lock, flags);
		return;
	}
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (!host->clk_requests) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		/* This will set host->ios.clock to 0 */
		mmc_gate_clock(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: gated MCI clock\n", mmc_hostname(host));
	}
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

/*
 * Internal work. Work to disable the clock at some later point.
 */
static void mmc_host_clk_gate_work(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host,
					      clk_gate_work.work);

	mmc_host_clk_gate_delayed(host);
}

/**
 *	mmc_host_clk_hold - ungate hardware MCI clocks
 *	@host: host to ungate.
 *
 *	Makes sure the host ios.clock is restored to a non-zero value
 *	past this call.	Increase clock reference count and ungate clock
 *	if we're the first user.
 */
void mmc_host_clk_hold(struct mmc_host *host)
{
	unsigned long flags;

	/* cancel any clock gating work scheduled by mmc_host_clk_release() */
	cancel_delayed_work_sync(&host->clk_gate_work);
	mutex_lock(&host->clk_gate_mutex);
	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated) {
		spin_unlock_irqrestore(&host->clk_lock, flags);
		mmc_ungate_clock(host);
		spin_lock_irqsave(&host->clk_lock, flags);
		pr_debug("%s: ungated MCI clock\n", mmc_hostname(host));
	}
	host->clk_requests++;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mutex_unlock(&host->clk_gate_mutex);
}

/**
 *	mmc_host_may_gate_card - check if this card may be gated
 *	@card: card to check.
 */
static bool mmc_host_may_gate_card(struct mmc_card *card)
{
	/* If there is no card we may gate it */
	if (!card)
		return true;
	/*
	 * Don't gate SDIO cards! These need to be clocked at all times
	 * since they may be independent systems generating interrupts
	 * and other events. The clock requests counter from the core will
	 * go down to zero since the core does not need it, but we will not
	 * gate the clock, because there is somebody out there that may still
	 * be using it.
	 */
	return !(card->quirks & MMC_QUIRK_BROKEN_CLK_GATING);
}

/**
 *	mmc_host_clk_release - gate off hardware MCI clocks
 *	@host: host to gate.
 *
 *	Calls the host driver with ios.clock set to zero as often as possible
 *	in order to gate off hardware MCI clocks. Decrease clock reference
 *	count and schedule disabling of clock.
 */
void mmc_host_clk_release(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_requests--;
	if (mmc_host_may_gate_card(host->card) &&
	    !host->clk_requests)
		queue_delayed_work(system_nrt_wq, &host->clk_gate_work,
				msecs_to_jiffies(host->clkgate_delay));
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

/**
 *	mmc_host_clk_rate - get current clock frequency setting
 *	@host: host to get the clock frequency for.
 *
 *	Returns current clock frequency regardless of gating.
 */
unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	unsigned long freq;
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	if (host->clk_gated)
		freq = host->clk_old;
	else
		freq = host->ios.clock;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	return freq;
}

/**
 *	mmc_host_clk_init - set up clock gating code
 *	@host: host with potential clock to control
 */
static inline void mmc_host_clk_init(struct mmc_host *host)
{
	host->clk_requests = 0;
	/* Hold MCI clock for 8 cycles by default */
	host->clk_delay = 8;
	/*
	 * Default clock gating delay is 0ms to avoid wasting power.
	 * This value can be tuned by writing into sysfs entry.
	 */
	host->clkgate_delay = 0;
	host->clk_gated = false;
	INIT_DELAYED_WORK(&host->clk_gate_work, mmc_host_clk_gate_work);
	spin_lock_init(&host->clk_lock);
	mutex_init(&host->clk_gate_mutex);
}

/**
 *	mmc_host_clk_exit - shut down clock gating code
 *	@host: host with potential clock to control
 */
static inline void mmc_host_clk_exit(struct mmc_host *host)
{
	/*
	 * Wait for any outstanding gate and then make sure we're
	 * ungated before exiting.
	 */
	if (cancel_delayed_work_sync(&host->clk_gate_work))
		mmc_host_clk_gate_delayed(host);
	if (host->clk_gated)
		mmc_host_clk_hold(host);
	/* There should be only one user now */
	WARN_ON(host->clk_requests > 1);
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{
	host->clkgate_delay_attr.show = clkgate_delay_show;
	host->clkgate_delay_attr.store = clkgate_delay_store;
	sysfs_attr_init(&host->clkgate_delay_attr.attr);
	host->clkgate_delay_attr.attr.name = "clkgate_delay";
	host->clkgate_delay_attr.attr.mode = S_IRUGO | S_IWUSR;
	if (device_create_file(&host->class_dev, &host->clkgate_delay_attr))
		pr_err("%s: Failed to create clkgate_delay sysfs entry\n",
				mmc_hostname(host));
}
#else

static inline void mmc_host_clk_init(struct mmc_host *host)
{
}

static inline void mmc_host_clk_exit(struct mmc_host *host)
{
}

static inline void mmc_host_clk_sysfs_init(struct mmc_host *host)
{}
#endif
static void mmc_card_init_wait(struct mmc_host *mmc)
{	 
    //if(mmc->card)
    //    return;
    if(!wait_for_completion_timeout(&mmc->card_init_done, CARD_INIT_TIMEOUT))
    {    	
        kasprintf(GFP_KERNEL, "[%s]:card initiation is timeout\n", __func__);
    }
    return;
}

static void mmc_card_init_complete(struct mmc_host* mmc)
{  
    complete(&mmc->card_init_done);
    return;
}

/**
 *	mmc_alloc_host - initialise the per-host structure.
 *	@extra: sizeof private data structure
 *	@dev: pointer to host device model structure
 *
 *	Initialise the per-host structure.
 */
 
 // 底层host controller驱动调用，用来分配一个struct mmc_host结构体，将其于mmc_host_class关联，并且做部分初始化操作

//主要工作：

//分配内存空间
//初始化其class device（对应/sys/class/mmc0节点）
//clock gate、锁、工作队列、wakelock、detect工作的初始化
//初始化detect成员（也就是检测工作）为mmc_rescan
struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{

	//	  参数说明：extra——》mmc_host的私有数据的长度，会和mmc_host结构体一起分配，
	//					   dev——》底层host controller的device结构体，用于作为mmc_host的device的父设备

	int err;
	struct mmc_host *host;

	
	/* 因为只是分配了一个mmc_host，host还没有准备好，所以这里禁用rescan，也就是设置mmc_host->rescan_disable 
		host->rescan_disable = 1*/	// 在mmc_start_host中会去使能

	if (!idr_pre_get(&mmc_host_idr, GFP_KERNEL))
		return NULL;
	
	/* 分配内存空间，其中多分配了extra字节作为私有数据 */
	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);    //  把 mmc_host 与 msmmmc_host 联系起来了  
	if (!host)
		return NULL;

	spin_lock(&mmc_host_lock);
	err = idr_get_new(&mmc_host_idr, host, &host->index);   /* 为该mmc_host分配一个唯一的id号，设置到host->index */
	spin_unlock(&mmc_host_lock);
	if (err)
		goto free;
	
	/* 设置mmc_host name */
	dev_set_name(&host->class_dev, "mmc%d", host->index);  // 以mmc_host的id号构成mmc_host的name，例如mmc0、mmc1


	/* 关联mmc_host class_dev并进行初始化 */
	/* class_dev就代表了mmc_host 的device结构体，是其在设备驱动模型中的体现 */
	host->parent = dev;                                // 将mmc_host的parent设置成对应host controller节点转化出来的device
	host->class_dev.parent = dev;                      // 将mmc_host的device(class_dev)的parent设置成对应host controller节点转化出来的device
	                                                   // 注册到sysfs之后，会相应生成/sys/bus/platform/devices/7824900.sdhci/mmc_host/mmc0
                                                       // 其中7824900.sdhci表示qcom的host controller节点转化出来的device
                                                       //  mtk 对应 /sys/bus/platform/devices/atc-msdc.0/mmc_host/mmc0
													   
	host->class_dev.class = &mmc_host_class;        // 将mmc_device(class_dev)的类设置为mmc_host_class   //  这会在 sys/class/mmc_host 下建立一个  mmc%d 的文件  
                                                    // 注册到sysfs之后，会相应生成/sys/class/mmc_host/mmc0
	 device_initialize(&host->class_dev);            // 初始化mmc_host->class_dev


	/* clock gate、锁、工作队列、wakelock、detect工作的初始化 */
	mmc_host_clk_init(host);
	init_completion(&host->card_init_done);
	host->card_init_wait = mmc_card_init_wait;
	host->card_init_complete = mmc_card_init_complete;

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	
	// 可以通过/sys/kernel/debug/wakeup_sources，相应生成了mmc0_detect和mmc1_detect两个wakelock
	wake_lock_init(&host->detect_wake_lock, WAKE_LOCK_SUSPEND,   // 初始化detect_wake_lock
		kasprintf(GFP_KERNEL, "%s_detect", mmc_hostname(host)));        // 设置detect_wake_lock的名称为mmc0_detect，在card检测的时候会使用

	 // ！！！！这个很重要！！！！初始化detect工作为mmc_rescan，后续调度host->detect来检测是否有card插入时，就会调用到mmc_rescan	
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);                        //  初始化一个工作队列  延时函数为  msmsdcc_host()  
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	/*
	 * By default, hosts do not support SGIO or large requests.
	 * They have to set these according to their abilities.
	 */
	 /* 一些size的初始化 */
	host->max_segs = 1; // 初始化最大支持段（由host自己根据硬件进行修改），可以通过/sys/block/mmcblk0/queue/max_segments进行修改
	host->max_seg_size = PAGE_CACHE_SIZE;  // 初始化段大小，（由host自己根据硬件进行修改）   // 这些都是对  host 进行一个默认的设置   有些值可能会被上文的 probe  替换掉  

	host->max_req_size = PAGE_CACHE_SIZE;  // 一次MMC请求的最大字节数
	host->max_blk_size = 512;               // 一个块的最大字节数
	host->max_blk_count = PAGE_CACHE_SIZE / 512;  // 一次MMC请求的最大块数量 

	return host;

free:
	kfree(host);
	return NULL;
}

EXPORT_SYMBOL(mmc_alloc_host);

/**
 *	mmc_add_host - initialise host hardware
 *	@host: mmc host
 *
 *	Register the host with the driver model. The host must be
 *	prepared to start servicing requests before this function
 *	completes.
 */

/********
底层host controller驱动调用，注册mmc_host到设备驱动中，添加到sys类下面，并设置相应的debug目录。然后启动mmc_host。

主要工作： 
使能pm runtime功能
将mmc_host的class_dev添加到设备驱动模型中，在sysfs中生成相应的节点
初始化mmc_host相关的debug目录
设置mmc_host的class_dev的属性
调用mmc_start_host启动host（进入mmc core主模块的部分）
**********/
int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);
	
	/* 通过device_add将mmc_host->class_dev添加到设备驱动模型中，在sys下生成相应节点 */
	err = device_add(&host->class_dev);    //  这里添加一个  mmc_host 的device 会与 block.c 中的 mmc_driver 相匹配   
										   // 通过mmc_alloc_host中关于mmc_host的class_dev的关联，可以生成如下两个节点
									       // /sys/bus/platform/devices/7824900.sdhci/mmc_host/mmc0
									       // /sys/class/mmc_host/mmc0
	if (err)
		return err;

	
	/* 使能mmc host的class_dev的异步suspend的功能 */
	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

/* 设置mmc_host的debug节点 */
#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	// 对应sys节点为/sys/kernel/debug/mmc0
	
	/* 以下设置mmc host的class_dev的属性 */
	mmc_host_clk_sysfs_init(host);
    // 对应/sys/class/mmc_host/mmc0/clkgate_delay属性
    
    /* 调用mmc_start_host，也就调用到了mmc core主模块的启动host部分，在mmc core主模块的时候说明 */
	mmc_start_host(host);   // 也就是说，关于host的初始化工作，需要在调用mmc_add_host之前就要完成了
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);

	return 0;
}
/*******   对应的相关节点   
/sys/bus/platform/devices/atc-msdc0/mmc_host/mmc0 
/sys/class/mmc_host/mmc0 
/sys/kernel/debug/mmc0
********/



EXPORT_SYMBOL(mmc_add_host);

/**
 *	mmc_remove_host - remove host hardware
 *	@host: mmc host
 *
 *	Unregister and remove all cards associated with this host,
 *	and power down the MMC bus. No new requests will be issued
 *	after this function has returned.
 */
void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		unregister_pm_notifier(&host->pm_notify);

	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif

	device_del(&host->class_dev);

	led_trigger_unregister_simple(host->led);

	mmc_host_clk_exit(host);
}

EXPORT_SYMBOL(mmc_remove_host);

/**
 *	mmc_free_host - free the host structure
 *	@host: mmc host
 *
 *	Free the host once all references to it have been dropped.
 */
void mmc_free_host(struct mmc_host *host)
{
	spin_lock(&mmc_host_lock);
	idr_remove(&mmc_host_idr, host->index);
	spin_unlock(&mmc_host_lock);
	wake_lock_destroy(&host->detect_wake_lock);

	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);
