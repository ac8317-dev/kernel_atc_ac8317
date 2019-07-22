#include <mach/chip_ver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <mach/ac83xx_basic.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <mach/ac83xx_system.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <mach/quickboot.h>
#include <x_ver.h>




#define SPM_READ32(REG)            __raw_readl(__io(SPM_BASE_VA + REG))
#define SPM_WRITE32(VAL, REG)      __raw_writel(VAL, __io(SPM_BASE_VA + REG))

static struct quickboot_param qb_param; 

extern void ac83xx_power_off(void);
extern void quickboot_resume(void);
extern void enter_sleep_mode_coreoff(uint32_t *buf);
extern void enter_sleep_mode_coreon(void);
extern void ac83xx_mask_ack_bim_irq(uint32_t irq);
static uint32_t   clk_reg[0x80];
uint32_t   quickboot_suspend_state= 0 ; //0:core off, 1:core on
#define REGDUMP(offset)  printk("0x%x : 0x%x\n",base + offset,HAL_READ32(base+offset));
static void check_topregister()
{

	uint32_t base = 0xFD000000;
	uint32_t offset = 0;
	for(offset = 0x9C;offset <= 0xD0;offset+=4)
		REGDUMP(offset);
	return;

}
void save_clkgate()
{

   	uint32_t base = 0xFD000000;
	uint32_t offset = 0;
	for(offset = 0x0;offset < 0x200;offset+=4)
		clk_reg[offset/4] = HAL_READ32(base+offset);




}
void restore_clkgate()
{

   	uint32_t base = 0xFD000000;
	uint32_t offset = 0;
	for(offset = 0x0;offset < 0x200;offset+=4)
               HAL_WRITE32(base+offset,clk_reg[offset/4]);
    

}
 void (*ac83xx_sram_suspend)( );
extern void ac83xx_smp_resume(void); 
static int ac8317_suspend()
{
	
	PDWNC_WRITE32(0x160,0x6D617273);
	check_topregister();
	flush_cache_all();
	save_clkgate();
#if     1
       if(quickboot_suspend_state == 0)
	  enter_sleep_mode_coreoff(virt_to_phys((uint32_t *) &qb_param));
       else
         enter_sleep_mode_coreon();
//	restore_clkgate();
//
	HAL_WRITE32(0xFD008164,0x0);
	HAL_WRITE32(0xFD008148,0x6978);
	HAL_WRITE32(0xFD008164,0x3);
	
        SPM_WRITE32(0x02860001, 0);

        ac83xx_smp_resume();
#else


         ((ac83xx_sram_suspend)(0xFC000020))(0xFC005000,0x0,0x0,0x0);
         
#endif



}
static int ac83xx_pm_valid(suspend_state_t state)
{
	int fgResult;
	fgResult = ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
	printk(KERN_INFO "ac83xx_pm_valid %s (%u) %s\n", \
			(!state)   ?"PM_SUSPEND_ON":\
			(1==state) ?"PM_SUSPEND_STANDBY":\
			(3==state) ?"PM_SUSPEND_MEM":"UNKNOW",
			state,
			fgResult?"support":"unsupport");

	return fgResult;
}


static int ac83xx_pm_begin(suspend_state_t state)
{
	printk(KERN_INFO "ac83xx_pm_begin: Nothing to do!\n");

	return 0;
}

static int ac83xx_pm_prepare( void)
{
	printk(KERN_INFO "ac83xx_pm_prepare\n");

	return 0;
}

static int ac83xx_pm_prepare_late(void)
{
	u32 u4Clock;

	printk(KERN_INFO "ac83xx_pm_prepare_late");

	return 0;
}


static int ac83xx_pm_enter(suspend_state_t state)
{
	printk(KERN_INFO "ac83xx_pm_enter %s (%u)\n",\
			(!state)   ?"PM_SUSPEND_ON":\
			(1==state) ?"PM_SUSPEND_STANDBY":\
			(3==state) ?"PM_SUSPEND_MEM":"UNKNOW", state);

	switch(state)
	{
		case PM_SUSPEND_MEM:
			ac8317_suspend();
			break;
		case PM_SUSPEND_STANDBY:
			break;
		case PM_SUSPEND_ON:
			break;
		default:
			break;
	}
	return 0;
}

static void ac83xx_pm_wake(void)
{
	int i=0;
	u32 u4Clock;

	printk(KERN_INFO "ac83xx_pm_wake\n");
}

static void ac83xx_pm_finish(void)
{
	printk(KERN_INFO "ac83xx_pm_finish");

}

static void ac83xx_pm_end(void)
{
	printk(KERN_INFO "ac83xx_pm_end: Nothing to do!\n");
	HAL_WRITE32(0xFD008164,0x0);
        ac83xx_mask_ack_bim_irq(VECTOR_T0);
	HAL_WRITE32(0xFD008148,0x6978);
	HAL_WRITE32(0xFD008164,0x3);

}

static void ac83xx_pm_recover(void)
{
	printk(KERN_INFO "ac83xx_pm_recover: Nothing to do!\n");
}

// begin->*prepare->*prepare_late->*enter->*wake->*finish->end
static struct platform_suspend_ops ac83xx_pm_ops =
{
	.valid        = ac83xx_pm_valid,
	.begin        = ac83xx_pm_begin,
	.prepare      = ac83xx_pm_prepare,
	.prepare_late = ac83xx_pm_prepare_late,
	.enter        = ac83xx_pm_enter,
	.wake         = ac83xx_pm_wake,
	.finish       = ac83xx_pm_finish,
	.end          = ac83xx_pm_end,
	.recover      = ac83xx_pm_recover,
};

static int __init ac83xx_pm_init(void)
{
        unsigned char *buf;
#ifdef CONFIG_PM
	printk(KERN_INFO "ac83xx_pm_init\n");
	suspend_set_ops(&ac83xx_pm_ops);
#endif
        MOD_VERSION_INFO("POWER", 1, 0, 0);        
        buf =(uint32_t *)kmalloc(16*1024,GFP_ATOMIC);
         
        printk("calibration virtual address:%p phsical address:%p\n", buf,virt_to_phys(buf));
        qb_param.ddr_cal_addr  = virt_to_phys(buf);
	qb_param.version = 0x1;
	qb_param.resume_entry = virt_to_phys(quickboot_resume);
	qb_param.wakeup_src_gpio = GPIO_WAKEUP_SRC;
	qb_param.wakeup_src_polarity = GPIO_POLARITY_HIGH;
	qb_param.wakeup_sts_gpio = GPIO_WAKEUP_STS;
	qb_param.wakeup_sts_polarity = GPIO_POLARITY_HIGH; 
        pm_power_off = ac83xx_power_off;
        return 0;
}


late_initcall(ac83xx_pm_init);
