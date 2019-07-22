/*
 *  linux/include/linux/mmc/host.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Host driver specific definitions.
 */
#ifndef LINUX_MMC_HOST_H
#define LINUX_MMC_HOST_H

#include <linux/leds.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/fault-inject.h>
#include <linux/wakelock.h>

#include <linux/mmc/core.h>
#include <linux/mmc/pm.h>

//  struct mmc_ios 由mmc core定义的规范的结构，用来维护mmc总线相关的一些io setting 
struct mmc_ios {
	unsigned int	clock;			/* clock rate */    // 当前工作频率   
	unsigned short	vdd;                                // 支持的电压表

/* vdd stores the bit number of the selected voltage range from below. */

	unsigned char	bus_mode;		/* command output mode */   // 总线输出模式，包括开漏模式和上拉模式

#define MMC_BUSMODE_OPENDRAIN	1
#define MMC_BUSMODE_PUSHPULL	2

	unsigned char	chip_select;		/* SPI chip select */       // spi片选   

#define MMC_CS_DONTCARE		0
#define MMC_CS_HIGH		1
#define MMC_CS_LOW		2

	unsigned char	power_mode;		/* power supply mode */   // 电源状态模式  

#define MMC_POWER_OFF		0
#define MMC_POWER_UP		1
#define MMC_POWER_ON		2

	unsigned char	bus_width;		/* data bus width */     // 总线宽度  

#define MMC_BUS_WIDTH_1		0
#define MMC_BUS_WIDTH_4		2
#define MMC_BUS_WIDTH_8		3

	unsigned char	timing;			/* timing specification used */    // 时序类型  

#define MMC_TIMING_LEGACY	0
#define MMC_TIMING_MMC_HS	1
#define MMC_TIMING_SD_HS	2
#define MMC_TIMING_UHS_SDR12	MMC_TIMING_LEGACY
#define MMC_TIMING_UHS_SDR25	MMC_TIMING_SD_HS
#define MMC_TIMING_UHS_SDR50	3
#define MMC_TIMING_UHS_SDR104	4
#define MMC_TIMING_UHS_DDR50	5
#define MMC_TIMING_MMC_HS200	6

#define MMC_SDR_MODE		0
#define MMC_1_2V_DDR_MODE	1
#define MMC_1_8V_DDR_MODE	2
#define MMC_1_2V_SDR_MODE	3
#define MMC_1_8V_SDR_MODE	4

	unsigned char	signal_voltage;		/* signalling voltage (1.8V or 3.3V) */    // 信号的工作电压

#define MMC_SIGNAL_VOLTAGE_330	0
#define MMC_SIGNAL_VOLTAGE_180	1
#define MMC_SIGNAL_VOLTAGE_120	2

	unsigned char	drv_type;		/* driver type (A, B, C, D) */       // 驱动类型   

#define MMC_SET_DRIVER_TYPE_B	0
#define MMC_SET_DRIVER_TYPE_A	1
#define MMC_SET_DRIVER_TYPE_C	2
#define MMC_SET_DRIVER_TYPE_D	3
};



// mmc core将host需要提供的一些操作方法封装成struct mmc_host_ops
// mmc core主模块的很多接口都是基于这里面的操作方法来实现的，通过这些方法来操作host硬件达到对应的目的
//  所以struct mmc_host_ops也是host controller driver需要实现的核心部分
struct mmc_host_ops {
	/*
	 * 'enable' is called when the host is claimed and 'disable' is called
	 * when the host is released. 'enable' and 'disable' are deprecated.
	 */
	int (*enable)(struct mmc_host *host);     // 使能host，当host被占用时（第一次调用mmc_claim_host）调用
	int (*disable)(struct mmc_host *host);    // 禁用host，当host被释放时（第一次调用mmc_release_host）调用
	/*
	 * It is optional for the host to implement pre_req and post_req in
	 * order to support double buffering of requests (prepare one
	 * request while another request is active).
	 * pre_req() must always be followed by a post_req().
	 * To undo a call made to pre_req(), call post_req() with
	 * a nonzero err condition.
	 */

	// post_req和pre_req是为了实现异步请求处理而设置的
    // 异步请求处理就是指，当另外一个异步请求还没有处理完成的时候，可以先准备另外一个异步请求而不必等待
	
	void	(*post_req)(struct mmc_host *host, struct mmc_request *req,
			    int err);
	void	(*pre_req)(struct mmc_host *host, struct mmc_request *req,
			   bool is_first_req);
	void	(*request)(struct mmc_host *host, struct mmc_request *req);  // host处理mmc请求的方法，在mmc_start_request中会调用
	void	(*req_tuning)(struct mmc_host *host, struct mmc_request *req);  
	void	(*send_stop)(struct mmc_host *host, struct mmc_request *req);
	void	(*dma_error_reset)(struct mmc_host *host);
	bool	(*check_written_data)(struct mmc_host *host, struct mmc_request *req);

	#define MMC_INIT_CARD_STATUS_NO_START		0
	#define MMC_INIT_CARD_STATUS_DOING			1
	#define MMC_INIT_CARD_STATUS_FAILED			2
	#define MMC_INIT_CARD_STATUS_SUCCESS		3
	void	(*init_card_status)(struct mmc_host *host, int status);


	/*
	 * Avoid calling these three functions too often or in a "fast path",
	 * since underlaying controller might implement them in an expensive
	 * and/or slow way.
	 *
	 * Also note that these functions might sleep, so don't call them
	 * in the atomic contexts!
	 *
	 * Return values for the get_ro callback should be:
	 *   0 for a read/write card
	 *   1 for a read-only card
	 *   -ENOSYS when not supported (equal to NULL callback)
	 *   or a negative errno value when something bad happened
	 *
	 * Return values for the get_cd callback should be:
	 *   0 for a absent card
	 *   1 for a present card
	 *   -ENOSYS when not supported (equal to NULL callback)
	 *   or a negative errno value when something bad happened
	 */
	void	(*set_ios)(struct mmc_host *host, struct mmc_ios *ios);    // 设置host的总线的io setting
	int	(*get_ro)(struct mmc_host *host);      // 获取host上的card的读写属性
	int	(*get_cd)(struct mmc_host *host);      // 检测host的卡槽中card的插入状态
	int	(*get_rescan)(struct mmc_host *host);   

	void	(*enable_sdio_irq)(struct mmc_host *host, int enable);

	/* optional callback for HC quirks */
	void	(*init_card)(struct mmc_host *host, struct mmc_card *card);     // 初始化card的方法

	int	(*start_signal_voltage_switch)(struct mmc_host *host, struct mmc_ios *ios);     // 切换信号电压的方法 

	/* The tuning command opcode value is different for SD and eMMC cards */
	int	(*execute_tuning)(struct mmc_host *host, u32 opcode);    // 执行tuning操作，为card选择一个合适的采样点
	void	(*enable_preset_value)(struct mmc_host *host, bool enable);  
	int	(*select_drive_strength)(unsigned int max_dtr, int host_drv, int card_drv);  // 选择信号的驱动强度
	void	(*hw_reset)(struct mmc_host *host);   // 硬件复位
};

struct mmc_card;
struct device;

//  异步请求的结构体。封装了struct mmc_request请求结构体  
struct mmc_async_req {
	/* active mmc request */
	struct mmc_request	*mrq;      // mmc请求    
	/*
	 * Check error status of completed mmc request.
	 * Returns 0 if success otherwise non zero.
	 */
	int (*err_check) (struct mmc_card *, struct mmc_async_req *);
};

struct mmc_hotplug {
	unsigned int irq;
	void *handler_priv;
};


//  struct mmc_host   是mmc core 根据 mmc controller 抽象出来的结构体  用于代表一个 mmc 控制器  CPU 端   

struct mmc_host {
	struct device		*parent;      //  对应的 host controller  的 devices  
	struct device		class_dev;    // mmc_host的device结构体，会挂在class/mmc_host下
	int			index;                // 该 host 的索引号  
	const struct mmc_host_ops *ops;   //  该 host的操作集  由 host controller  设置   
	unsigned int		f_min;     // 该host支持的最低频率
	unsigned int		f_max;     // 该host支持的最大频率
	unsigned int		f_init;    // 该host使用的初始化频率
	u32			ocr_avail;         //  该 host 可用的 OCR  电压相关   
	u32			ocr_avail_sdio;	/* SDIO-specific OCR */
	u32			ocr_avail_sd;	/* SD-specific OCR */
	u32			ocr_avail_mmc;	/* MMC-specific OCR */
	struct notifier_block	pm_notify;

//  OCR  代表的电压值如下 

#define MMC_VDD_165_195		0x00000080	/* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21		0x00000100	/* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22		0x00000200	/* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23		0x00000400	/* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24		0x00000800	/* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25		0x00001000	/* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26		0x00002000	/* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27		0x00004000	/* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28		0x00008000	/* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29		0x00010000	/* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30		0x00020000	/* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31		0x00040000	/* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32		0x00080000	/* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33		0x00100000	/* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34		0x00200000	/* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35		0x00400000	/* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36		0x00800000	/* VDD voltage 3.5 ~ 3.6 */

	unsigned long		caps;		/* Host capabilities */      //   host 属性   

#define MMC_CAP_4_BIT_DATA		(1 << 0)	/* Can the host do 4 bit transfers */
#define MMC_CAP_MMC_HIGHSPEED	(1 << 1)	/* Can do MMC high-speed timing */
#define MMC_CAP_SD_HIGHSPEED	(1 << 2)	/* Can do SD high-speed timing */
#define MMC_CAP_SDIO_IRQ		(1 << 3)	/* Can signal pending SDIO IRQs */
#define MMC_CAP_SPI				(1 << 4)	/* Talks only SPI protocols */
#define MMC_CAP_NEEDS_POLL		(1 << 5)	/* Needs polling for card-detection */
#define MMC_CAP_8_BIT_DATA		(1 << 6)	/* Can the host do 8 bit transfers */

#define MMC_CAP_NONREMOVABLE	(1 << 8)	/* Nonremovable e.g. eMMC */
#define MMC_CAP_WAIT_WHILE_BUSY	(1 << 9)	/* Waits while card is busy */
#define MMC_CAP_ERASE			(1 << 10)	/* Allow erase/trim commands */
#define MMC_CAP_1_8V_DDR		(1 << 11)	/* can support DDR mode at 1.8V */
#define MMC_CAP_1_2V_DDR		(1 << 12)	/* can support DDR mode at 1.2V */
#define MMC_CAP_POWER_OFF_CARD	(1 << 13)	/* Can power off after boot */
#define MMC_CAP_BUS_WIDTH_TEST	(1 << 14)	/* CMD14/CMD19 bus width ok */
#define MMC_CAP_UHS_SDR12		(1 << 15)	/* Host supports UHS SDR12 mode */
#define MMC_CAP_UHS_SDR25		(1 << 16)	/* Host supports UHS SDR25 mode */
#define MMC_CAP_UHS_SDR50		(1 << 17)	/* Host supports UHS SDR50 mode */
#define MMC_CAP_UHS_SDR104		(1 << 18)	/* Host supports UHS SDR104 mode */
#define MMC_CAP_UHS_DDR50		(1 << 19)	/* Host supports UHS DDR50 mode */
#define MMC_CAP_SET_XPC_330		(1 << 20)	/* Host supports >150mA current at 3.3V */
#define MMC_CAP_SET_XPC_300		(1 << 21)	/* Host supports >150mA current at 3.0V */
#define MMC_CAP_SET_XPC_180		(1 << 22)	/* Host supports >150mA current at 1.8V */
#define MMC_CAP_DRIVER_TYPE_A	(1 << 23)	/* Host supports Driver Type A */
#define MMC_CAP_DRIVER_TYPE_C	(1 << 24)	/* Host supports Driver Type C */
#define MMC_CAP_DRIVER_TYPE_D	(1 << 25)	/* Host supports Driver Type D */
#define MMC_CAP_MAX_CURRENT_200	(1 << 26)	/* Host max current limit is 200mA */
#define MMC_CAP_MAX_CURRENT_400	(1 << 27)	/* Host max current limit is 400mA */
#define MMC_CAP_MAX_CURRENT_600	(1 << 28)	/* Host max current limit is 600mA */
#define MMC_CAP_MAX_CURRENT_800	(1 << 29)	/* Host max current limit is 800mA */
#define MMC_CAP_CMD23			(1 << 30)	/* CMD23 supported. */
#define MMC_CAP_HW_RESET		(1 << 31)	/* Hardware reset */

	unsigned int		caps2;		/* More host capabilities */   //  host 属性2  

#define MMC_CAP2_BOOTPART_NOACC	(1 << 0)	/* Boot partition no access */
#define MMC_CAP2_CACHE_CTRL		(1 << 1)	/* Allow cache control */
#define MMC_CAP2_POWEROFF_NOTIFY (1 << 2)	/* Notify poweroff supported */
#define MMC_CAP2_NO_MULTI_READ	(1 << 3)	/* Multiblock reads don't work */
#define MMC_CAP2_NO_SLEEP_CMD	(1 << 4)	/* Don't allow sleep command */
#define MMC_CAP2_HS200_1_8V_SDR	(1 << 5)        /* can support */
#define MMC_CAP2_HS200_1_2V_SDR	(1 << 6)        /* can support */
#define MMC_CAP2_HS200			(MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_HS200_1_2V_SDR)
#define MMC_CAP2_BROKEN_VOLTAGE	(1 << 7)	/* Use the broken voltage */
#define MMC_CAP2_DETECT_ON_ERR	(1 << 8)	/* On I/O err check card removal */
#define MMC_CAP2_HC_ERASE_SZ	(1 << 9)	/* High-capacity erase size */

// Max Xia Add
#define MMC_CAP2_FUNC_EMMC		(1 << 30)	/* Host is configed for eMMC  */


	mmc_pm_flag_t		pm_caps;	/* supported pm features */   //  电源管理属性  
	unsigned int        power_notify_type;
#define MMC_HOST_PW_NOTIFY_NONE		0
#define MMC_HOST_PW_NOTIFY_SHORT	1
#define MMC_HOST_PW_NOTIFY_LONG		2

#ifdef CONFIG_MMC_CLKGATE      //   clock  相关成员  
	int			clk_requests;	/* internal reference counter */
	unsigned int		clk_delay;	/* number of MCI clk hold cycles */
	bool			clk_gated;	/* clock gated */
	struct delayed_work	clk_gate_work; /* delayed clock gate */
	unsigned int		clk_old;	/* old clock value cache */
	spinlock_t		clk_lock;	/* lock for clk fields */
	struct mutex		clk_gate_mutex;	/* mutex for clock gating */
	struct device_attribute clkgate_delay_attr;
	unsigned long           clkgate_delay;
#endif

	/* host specific block data */   //  块相关成员  
	unsigned int		max_seg_size;	/* see blk_queue_max_segment_size */
	unsigned short		max_segs;	/* see blk_queue_max_segments */
	unsigned short		unused;
	unsigned int		max_req_size;	/* maximum number of bytes in one req */
	unsigned int		max_blk_size;	/* maximum size of one mmc block */
	unsigned int		max_blk_count;	/* maximum number of blocks in one req */
	unsigned int		max_discard_to;	/* max. discard timeout in ms */

	/* private data */    //  host 的 bus 使用的锁 
	spinlock_t		lock;		/* lock for claim and bus ops */

	struct mmc_ios		ios;		/* current io bus settings */   // 后续说明  
	u32			ocr;		/* the current OCR setting */   //  当前使用的OCR值       如  MMC_VDD_20_21       

	/* group bitfields together to minimize padding */
	unsigned int		use_spi_crc:1;
	unsigned int		claimed:1;	/* host exclusively claimed */  //  host 是否已经被占用  
	unsigned int		bus_dead:1;	/* bus has been released */     // host 的 bus 是否 处于激活状态   
#ifdef CONFIG_MMC_DEBUG
	unsigned int		removed:1;	/* host is being removed */
#endif

	int			rescan_disable;	/* disable card detection */      //  禁止 rescan的标识   即 禁止扫描 card 的功能  

	struct mmc_card		*card;		/* device attached to this host */  // 和该 host 绑在一起的 card 

	wait_queue_head_t	wq;
	struct task_struct	*claimer;	/* task that has host claimed */  //  该 host 的占有者的进程
	int			claim_cnt;	/* "claim" nesting count */    //  占有者进程对该host 的 占有计数  

	struct delayed_work	detect;                 //  检查卡槽变化的工作 
	struct wake_lock	detect_wake_lock;       //  检查卡槽变化的工作的锁 
	int			detect_change;	/* card detect flag */   // 需要检查卡槽变化的标识  
	int			enabledelay;
	struct mmc_hotplug	hotplug;

	const struct mmc_bus_ops *bus_ops;	/* current bus driver */   // host的mmc总线的操作集，后面说明   
	unsigned int		bus_refs;	/* reference counter */        // host的mmc总线的使用计数

	unsigned int		bus_resume_flags;       // host的mmc总线的resume标识  
#define MMC_BUSRESUME_MANUAL_RESUME	(1 << 0)
#define MMC_BUSRESUME_NEEDS_RESUME	(1 << 1)

	unsigned int		sdio_irqs;
	struct task_struct	*sdio_irq_thread;
	bool			sdio_irq_pending;
	atomic_t		sdio_irq_thread_abort;

	mmc_pm_flag_t		pm_flags;	/* requested pm features */

#ifdef CONFIG_LEDS_TRIGGERS
	struct led_trigger	*led;		/* activity led */
#endif

#ifdef CONFIG_REGULATOR
	bool			regulator_enabled; /* regulator state */     // 代表regulator（LDO）的状态  
#endif

	struct dentry		*debugfs_root;                             // 对应的debug目录结构体  

	struct mmc_async_req	*areq;		/* active async req */      // 当前正在处理的异步请求  

#ifdef CONFIG_FAIL_MMC_REQUEST
	struct fault_attr	fail_mmc_request;
#endif

	unsigned int		actual_clock;	/* Actual HC clock rate */     // 实际的时钟频率

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	struct {
		struct sdio_cis			*cis;
		struct sdio_cccr		*cccr;
		struct sdio_embedded_func	*funcs;
		int				num_funcs;
	} embedded_sdio_data;
#endif
	struct completion           card_init_done;
	void (*card_init_complete)(struct mmc_host*);
	void (*card_init_wait)(struct mmc_host*);
	unsigned long		private[0] ____cacheline_aligned;          //  没有什么特殊的意义 仅代表一个地址   也正是他将 mmc_host 与 相关的结构体联系起来的  
};

extern int emmcpart_expdb_exist;

extern struct mmc_host *mmc_alloc_host(int extra, struct device *);
extern int mmc_add_host(struct mmc_host *);
extern void mmc_remove_host(struct mmc_host *);
extern void mmc_free_host(struct mmc_host *);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
extern void mmc_set_embedded_sdio_data(struct mmc_host *host,
				       struct sdio_cis *cis,
				       struct sdio_cccr *cccr,
				       struct sdio_embedded_func *funcs,
				       int num_funcs);
#endif

static inline void *mmc_priv(struct mmc_host *host)
{
	return (void *)host->private;
}

#define mmc_host_is_spi(host)	((host)->caps & MMC_CAP_SPI)

#define mmc_dev(x)	((x)->parent)
#define mmc_classdev(x)	(&(x)->class_dev)
#define mmc_hostname(x)	(dev_name(&(x)->class_dev))
#define mmc_bus_needs_resume(host) ((host)->bus_resume_flags & MMC_BUSRESUME_NEEDS_RESUME)
#define mmc_bus_manual_resume(host) ((host)->bus_resume_flags & MMC_BUSRESUME_MANUAL_RESUME)

static inline void mmc_set_bus_resume_policy(struct mmc_host *host, int manual)
{
	if (manual)
		host->bus_resume_flags |= MMC_BUSRESUME_MANUAL_RESUME;
	else
		host->bus_resume_flags &= ~MMC_BUSRESUME_MANUAL_RESUME;
}

extern int mmc_resume_bus(struct mmc_host *host);

extern int mmc_suspend_host(struct mmc_host *);
extern int mmc_resume_host(struct mmc_host *);

extern int mmc_power_save_host(struct mmc_host *host);
extern int mmc_power_restore_host(struct mmc_host *host);

extern void mmc_detect_change(struct mmc_host *, unsigned long delay);
extern void mmc_request_done(struct mmc_host *, struct mmc_request *);

extern int mmc_cache_ctrl(struct mmc_host *, u8);

static inline void mmc_signal_sdio_irq(struct mmc_host *host)
{
	host->ops->enable_sdio_irq(host, 0);
	host->sdio_irq_pending = true;
	wake_up_process(host->sdio_irq_thread);
}

struct regulator;

#ifdef CONFIG_REGULATOR
int mmc_regulator_get_ocrmask(struct regulator *supply);
int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit);
#else
static inline int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	return 0;
}

static inline int mmc_regulator_set_ocr(struct mmc_host *mmc,
				 struct regulator *supply,
				 unsigned short vdd_bit)
{
	return 0;
}
#endif

int mmc_card_awake(struct mmc_host *host);
int mmc_card_sleep(struct mmc_host *host);
int mmc_card_can_sleep(struct mmc_host *host);

int mmc_pm_notify(struct notifier_block *notify_block, unsigned long, void *);

/* Module parameter */
extern bool mmc_assume_removable;

static inline int mmc_card_is_removable(struct mmc_host *host)
{
	return !(host->caps & MMC_CAP_NONREMOVABLE) && mmc_assume_removable;
}

static inline int mmc_card_keep_power(struct mmc_host *host)
{
	return host->pm_flags & MMC_PM_KEEP_POWER;
}

static inline int mmc_card_wake_sdio_irq(struct mmc_host *host)
{
	return host->pm_flags & MMC_PM_WAKE_SDIO_IRQ;
}

static inline int mmc_host_cmd23(struct mmc_host *host)
{
	return host->caps & MMC_CAP_CMD23;
}

static inline int mmc_boot_partition_access(struct mmc_host *host)
{
	return !(host->caps2 & MMC_CAP2_BOOTPART_NOACC);
}

#ifdef CONFIG_MMC_CLKGATE
void mmc_host_clk_hold(struct mmc_host *host);
void mmc_host_clk_release(struct mmc_host *host);
unsigned int mmc_host_clk_rate(struct mmc_host *host);

#else
static inline void mmc_host_clk_hold(struct mmc_host *host)
{
}

static inline void mmc_host_clk_release(struct mmc_host *host)
{
}

static inline unsigned int mmc_host_clk_rate(struct mmc_host *host)
{
	return host->ios.clock;
}
#endif
#endif /* LINUX_MMC_HOST_H */
