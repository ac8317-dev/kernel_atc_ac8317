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

//  mmc_host_class������mmc_host�����   
static struct class mmc_host_class = {
	.name		= "mmc_host",                     // ���ӵ�sys�ļ�ϵͳ֮�󣬻�����/sys/class/mmc_host���Ŀ¼   
	.dev_release	= mmc_host_classdev_release,   // ��mmc_host���class��release��ĳ���豸֮��Ҫ���Ķ�Ӧ����  
};    //  .pm     = &mmc_host_pm_ops,        // ��class�µ�host��pm��Դ��������

//  ע��mmc_host class
int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);   // ��mmc_host_classΪclass����һ��class������mmc_host_class���������ݽṹ�Ѿ�˵������  
}                                             //��ؽڵ㣺/sys/class/mmc_host

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
 
 // �ײ�host controller�������ã���������һ��struct mmc_host�ṹ�壬������mmc_host_class���������������ֳ�ʼ������

//��Ҫ������

//�����ڴ�ռ�
//��ʼ����class device����Ӧ/sys/class/mmc0�ڵ㣩
//clock gate�������������С�wakelock��detect�����ĳ�ʼ��
//��ʼ��detect��Ա��Ҳ���Ǽ�⹤����Ϊmmc_rescan
struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{

	//	  ����˵����extra������mmc_host��˽�����ݵĳ��ȣ����mmc_host�ṹ��һ����䣬
	//					   dev�������ײ�host controller��device�ṹ�壬������Ϊmmc_host��device�ĸ��豸

	int err;
	struct mmc_host *host;

	
	/* ��Ϊֻ�Ƿ�����һ��mmc_host��host��û��׼���ã������������rescan��Ҳ��������mmc_host->rescan_disable 
		host->rescan_disable = 1*/	// ��mmc_start_host�л�ȥʹ��

	if (!idr_pre_get(&mmc_host_idr, GFP_KERNEL))
		return NULL;
	
	/* �����ڴ�ռ䣬���ж������extra�ֽ���Ϊ˽������ */
	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);    //  �� mmc_host �� msmmmc_host ��ϵ������  
	if (!host)
		return NULL;

	spin_lock(&mmc_host_lock);
	err = idr_get_new(&mmc_host_idr, host, &host->index);   /* Ϊ��mmc_host����һ��Ψһ��id�ţ����õ�host->index */
	spin_unlock(&mmc_host_lock);
	if (err)
		goto free;
	
	/* ����mmc_host name */
	dev_set_name(&host->class_dev, "mmc%d", host->index);  // ��mmc_host��id�Ź���mmc_host��name������mmc0��mmc1


	/* ����mmc_host class_dev�����г�ʼ�� */
	/* class_dev�ʹ�����mmc_host ��device�ṹ�壬�������豸����ģ���е����� */
	host->parent = dev;                                // ��mmc_host��parent���óɶ�Ӧhost controller�ڵ�ת��������device
	host->class_dev.parent = dev;                      // ��mmc_host��device(class_dev)��parent���óɶ�Ӧhost controller�ڵ�ת��������device
	                                                   // ע�ᵽsysfs֮�󣬻���Ӧ����/sys/bus/platform/devices/7824900.sdhci/mmc_host/mmc0
                                                       // ����7824900.sdhci��ʾqcom��host controller�ڵ�ת��������device
                                                       //  mtk ��Ӧ /sys/bus/platform/devices/atc-msdc.0/mmc_host/mmc0
													   
	host->class_dev.class = &mmc_host_class;        // ��mmc_device(class_dev)��������Ϊmmc_host_class   //  ����� sys/class/mmc_host �½���һ��  mmc%d ���ļ�  
                                                    // ע�ᵽsysfs֮�󣬻���Ӧ����/sys/class/mmc_host/mmc0
	 device_initialize(&host->class_dev);            // ��ʼ��mmc_host->class_dev


	/* clock gate�������������С�wakelock��detect�����ĳ�ʼ�� */
	mmc_host_clk_init(host);
	init_completion(&host->card_init_done);
	host->card_init_wait = mmc_card_init_wait;
	host->card_init_complete = mmc_card_init_complete;

	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	
	// ����ͨ��/sys/kernel/debug/wakeup_sources����Ӧ������mmc0_detect��mmc1_detect����wakelock
	wake_lock_init(&host->detect_wake_lock, WAKE_LOCK_SUSPEND,   // ��ʼ��detect_wake_lock
		kasprintf(GFP_KERNEL, "%s_detect", mmc_hostname(host)));        // ����detect_wake_lock������Ϊmmc0_detect����card����ʱ���ʹ��

	 // ���������������Ҫ����������ʼ��detect����Ϊmmc_rescan����������host->detect������Ƿ���card����ʱ���ͻ���õ�mmc_rescan	
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);                        //  ��ʼ��һ����������  ��ʱ����Ϊ  msmsdcc_host()  
#ifdef CONFIG_PM
	host->pm_notify.notifier_call = mmc_pm_notify;
#endif

	/*
	 * By default, hosts do not support SGIO or large requests.
	 * They have to set these according to their abilities.
	 */
	 /* һЩsize�ĳ�ʼ�� */
	host->max_segs = 1; // ��ʼ�����֧�ֶΣ���host�Լ�����Ӳ�������޸ģ�������ͨ��/sys/block/mmcblk0/queue/max_segments�����޸�
	host->max_seg_size = PAGE_CACHE_SIZE;  // ��ʼ���δ�С������host�Լ�����Ӳ�������޸ģ�   // ��Щ���Ƕ�  host ����һ��Ĭ�ϵ�����   ��Щֵ���ܻᱻ���ĵ� probe  �滻��  

	host->max_req_size = PAGE_CACHE_SIZE;  // һ��MMC���������ֽ���
	host->max_blk_size = 512;               // һ���������ֽ���
	host->max_blk_count = PAGE_CACHE_SIZE / 512;  // һ��MMC������������� 

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
�ײ�host controller�������ã�ע��mmc_host���豸�����У����ӵ�sys�����棬��������Ӧ��debugĿ¼��Ȼ������mmc_host��

��Ҫ������ 
ʹ��pm runtime����
��mmc_host��class_dev���ӵ��豸����ģ���У���sysfs��������Ӧ�Ľڵ�
��ʼ��mmc_host��ص�debugĿ¼
����mmc_host��class_dev������
����mmc_start_host����host������mmc core��ģ��Ĳ��֣�
**********/
int mmc_add_host(struct mmc_host *host)
{
	int err;

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);
	
	/* ͨ��device_add��mmc_host->class_dev���ӵ��豸����ģ���У���sys��������Ӧ�ڵ� */
	err = device_add(&host->class_dev);    //  ��������һ��  mmc_host ��device ���� block.c �е� mmc_driver ��ƥ��   
										   // ͨ��mmc_alloc_host�й���mmc_host��class_dev�Ĺ����������������������ڵ�
									       // /sys/bus/platform/devices/7824900.sdhci/mmc_host/mmc0
									       // /sys/class/mmc_host/mmc0
	if (err)
		return err;

	
	/* ʹ��mmc host��class_dev���첽suspend�Ĺ��� */
	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

/* ����mmc_host��debug�ڵ� */
#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif
	// ��Ӧsys�ڵ�Ϊ/sys/kernel/debug/mmc0
	
	/* ��������mmc host��class_dev������ */
	mmc_host_clk_sysfs_init(host);
    // ��Ӧ/sys/class/mmc_host/mmc0/clkgate_delay����
    
    /* ����mmc_start_host��Ҳ�͵��õ���mmc core��ģ�������host���֣���mmc core��ģ���ʱ��˵�� */
	mmc_start_host(host);   // Ҳ����˵������host�ĳ�ʼ����������Ҫ�ڵ���mmc_add_host֮ǰ��Ҫ�����
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		register_pm_notifier(&host->pm_notify);

	return 0;
}
/*******   ��Ӧ����ؽڵ�   
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