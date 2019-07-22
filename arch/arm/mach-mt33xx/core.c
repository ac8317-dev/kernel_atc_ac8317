/*
 *  linux/arch/arm/mach-MT85xx/core.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * CPU core init - irq, time, baseio
 */

/* system header files */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <mach/chip_ver.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/system.h>
   
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>

#include <asm/mach/time.h>
#include <mach/hardware.h>

#include <mach/system.h>
#include <mach/nand.h>

#include "pll.h"
#include <mach/ic_version.h>
#include <mach/mt33xx_64b_timer.h>
#include <mach/irq_priority.h>
#include <mach/mt33xx_isr_watch.h>
#include <mach/mt33xx_CS_watch.h>
#include <mach/mt33xx_system.h>
#include <mach/mt33xx_gpio_pinmux.h>
#include <mach/mt33xx_memory.h>
#if CONFIG_FAST_POWER_DOWN
#include <config/arch/faststandby_config.h>
#endif

//#include <assert.h>

#if defined(CONFIG_CACHE_L2X0)
#include <linux/spinlock.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
//#define L2Reg_WRITE32(offset, value)   (*((volatile unsigned int*)(L2_IO_VIRT + offset)) = (value))
//#define L2Reg_READ32(offset)   (*((volatile unsigned int*)(L2_IO_VIRT + offset)))
//extern void __init BSP_L2Cache_Operation_Init(void);
#endif
//sd control address
#define IO_BASE                          0x70000000

#define MT3360_MSDC0_BASE                (0x0B000 + IO_BASE)
#define MT3360_MSDC1_BASE                (0x021000 + IO_BASE)
#define MT3360_MSDC2_BASE                (0X0A000 + IO_BASE)

#define ECO_VER 0X104

extern void arch_pwr_Up(void);

/*======================================================================
 * irq
 */

static inline u32 __bim_readl(u32 regaddr32)
{
    return __raw_readl(BIM_VIRT + regaddr32);
}

static inline void __bim_writel(u32 regval32, u32 regaddr32)
{
    __raw_writel(regval32, BIM_VIRT + regaddr32);
}

//disable IRQ
void mt85xx_mask_irq_num(struct irq_data *data)
{
    u32 regval32;
    unsigned long flags;
	int irq = data->irq;
	
    if (!_bim_is_vector_valid(irq))
    {
	printk("irq not valid %d\n", irq);
        return;
    }

    //assert(irq < NR_IRQS);

    local_irq_save(flags);

    if(irq < 32)
    {
        regval32 = __bim_readl(REG_IRQEN);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;
        regval32 = __bim_readl(REG_IRQEN3);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN3);
    }
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    else if ((irq >= 96) && (irq < 128))
    {
        irq = irq - 96;
        regval32 = __bim_readl(REG_IRQEN4);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN4);
    }
    else
    {
        /* pdwn extend */
        irq = irq - 128; 
#else
    else
    {
        /* pdwn extend */
        irq = irq - 96;
#endif
        regval32 = PDWNC_READ32(REG_RW_INTEN);
        regval32 &= ~(1 << irq);
        PDWNC_WRITE32(REG_RW_INTEN, regval32);

        /* check pdwn */
        //if(regval32==0)
        {
            irq = VECTOR_PWDNC - 32;
            regval32 = __bim_readl(REG_IRQEN2);
            regval32 &= ~(1 << irq);
            __bim_writel(regval32, REG_IRQEN2);
        }
    }

    local_irq_restore(flags);
}

//extern void mt85xx_mask_irq_num(struct irq_data *data);
EXPORT_SYMBOL(mt85xx_mask_irq_num);

//disable IRQ
//void mt85xx_mask_irq(struct irq_data *data)
void mt85xx_mask_irq(int irq)
	
{
    u32 regval32;
    unsigned long flags;
	
    if (!_bim_is_vector_valid(irq))
    {
	printk("irq not valid %d\n", irq);
        return;
    }

    //assert(irq < NR_IRQS);

    local_irq_save(flags);

    if(irq < 32)
    {
        regval32 = __bim_readl(REG_IRQEN);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;
        regval32 = __bim_readl(REG_IRQEN3);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN3);
    }
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    else if ((irq >= 96) && (irq < 128))
    {
        irq = irq - 96;
        regval32 = __bim_readl(REG_IRQEN4);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN4);
    }
    else
    {
        /* pdwn extend */
        irq = irq - 128; 
#else
    else
    {
        /* pdwn extend */
        irq = irq - 96;
#endif
        regval32 = PDWNC_READ32(REG_RW_INTEN);
        regval32 &= ~(1 << irq);
        PDWNC_WRITE32(REG_RW_INTEN, regval32);

        /* check pdwn */
        //if(regval32==0)
        {
            irq = VECTOR_PWDNC - 32;
            regval32 = __bim_readl(REG_IRQEN2);
            regval32 &= ~(1 << irq);
            __bim_writel(regval32, REG_IRQEN2);
        }
    }

    local_irq_restore(flags);
}
EXPORT_SYMBOL(mt85xx_mask_irq);

//disable IRQ and clear IRQ status
//void mt85xx_mask_ack_irq(struct irq_data *data)
void mt85xx_mask_ack_irq(int irq)

{
    u32 regval32;
    unsigned long flags;
	
    if (!_bim_is_vector_valid(irq))
    {
	      printk("irq not valid %d\n", irq);
        return;
    }

    //assert(irq < NR_IRQS);

    local_irq_save(flags);

    if(irq < 32)
    {
        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL);
        __bim_writel(regval32, REG_IRQST);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL2);
        __bim_writel(regval32, REG_IRQST2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL3);
        __bim_writel(regval32, REG_IRQST3);
    }
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    else if((irq >= 96) && (irq < 128))
    {
     	irq = irq - 96;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL4);
        __bim_writel(regval32, REG_IRQST4);       
    }
    else
    {
        irq = irq - 128;
#else
    else
    {
        /* clear pdwn extend */
        irq = irq - 96;
#endif
        //set 1 to clear
        regval32 = (1 << irq);
        PDWNC_WRITE32(REG_RW_INTCLR, regval32);
        PDWNC_WRITE32(REG_RW_INTSTA, regval32);

        /* clear pwdn */
        irq = VECTOR_PWDNC - 32;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL2);
        __bim_writel(regval32, REG_IRQST2);
    }

    local_irq_restore(flags);  
}
EXPORT_SYMBOL(mt85xx_mask_ack_irq);

void mt85xx_mask_ack_irq_num(struct irq_data *data)
{
    u32 regval32;
    unsigned long flags;
	  int irq = 27;

	  irq = data->irq;
	
    if (!_bim_is_vector_valid(irq))
    {
	printk("irq not valid %d\n", irq);
        return;
    }

    //assert(irq < NR_IRQS);

    local_irq_save(flags);

    if(irq < 32)
    {
        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL);
        __bim_writel(regval32, REG_IRQST);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL2);
        __bim_writel(regval32, REG_IRQST2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL3);
        __bim_writel(regval32, REG_IRQST3);
    }
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    else if((irq >= 96) && (irq < 128))
    {
     	irq = irq - 96;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL4);
        __bim_writel(regval32, REG_IRQST4);       
    }
    else
    {
        irq = irq - 128;
#else
    else
    {
        /* clear pdwn extend */
        irq = irq - 96;
#endif
        //set 1 to clear
        regval32 = (1 << irq);
        PDWNC_WRITE32(REG_RW_INTCLR, regval32);
        PDWNC_WRITE32(REG_RW_INTSTA, regval32);

        /* clear pwdn */
        irq = VECTOR_PWDNC - 32;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL2);
        __bim_writel(regval32, REG_IRQST2);
    }

    local_irq_restore(flags);
}
EXPORT_SYMBOL(mt85xx_mask_ack_irq_num);

void mt85xx_unmask_irq_num(struct irq_data *data) //enable
{
		u32 regval32;
		unsigned long flags;
	  int irq;

	  irq = data->irq;
		//assert(irq < NR_IRQS);

		local_irq_save(flags);
		//__bim_writel((1 << irq), REG_RW_IRQCLR);

		if(irq < 32)
		{
				regval32 = __bim_readl(REG_IRQEN);
				regval32 |= (1 << irq);
				__bim_writel(regval32, REG_IRQEN);
		}
		else if((irq >= 32) && (irq < 64))
		{
			irq = irq - 32;
				regval32 = __bim_readl(REG_IRQEN2);
				regval32 |= (1 << irq);
				__bim_writel(regval32, REG_IRQEN2);
		}
		else if((irq >= 64) && (irq < 96))
		{
			irq = irq - 64;
				regval32 = __bim_readl(REG_IRQEN3);
				regval32 |= (1 << irq);
				__bim_writel(regval32, REG_IRQEN3);
		}
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
		else if((irq >= 96) && (irq < 128))
		{
			irq = irq - 96;
				regval32 = __bim_readl(REG_IRQEN4);
				regval32 |= (1 << irq);
				__bim_writel(regval32, REG_IRQEN4);
		}
		else
		{
				/* enable pdwn extend */
				irq = irq - 128;
#else
		else
		{
				/* enable pdwn extend */
				irq = irq - 96;
#endif
				regval32 = PDWNC_READ32(REG_RW_INTEN);
				regval32 |= (1 << irq);
				PDWNC_WRITE32(REG_RW_INTEN, regval32);

				/* enable pdwn */
				irq = VECTOR_PWDNC - 32;
				regval32 = __bim_readl(REG_IRQEN2);
				regval32 |= (1 << irq);
				__bim_writel(regval32, REG_IRQEN2);
		}
		local_irq_restore(flags);

}

EXPORT_SYMBOL(mt85xx_unmask_irq_num);

//void mt85xx_unmask_irq(struct irq_data *data) //enable
void mt85xx_unmask_irq(int irq) //enable
{
		u32 regval32;
    unsigned long flags;

    //assert(irq < NR_IRQS);

    local_irq_save(flags);
    //__bim_writel((1 << irq), REG_RW_IRQCLR);

    if(irq < 32)
    {
        regval32 = __bim_readl(REG_IRQEN);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;
        regval32 = __bim_readl(REG_IRQEN3);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN3);
    }
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    else if((irq >= 96) && (irq < 128))
    {
    	irq = irq - 96;
        regval32 = __bim_readl(REG_IRQEN4);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN4);
    }
    else
    {
        /* enable pdwn extend */
        irq = irq - 128;
#else
    else
    {
        /* enable pdwn extend */
        irq = irq - 96;
#endif
        regval32 = PDWNC_READ32(REG_RW_INTEN);
        regval32 |= (1 << irq);
        PDWNC_WRITE32(REG_RW_INTEN, regval32);

        /* enable pdwn */
        irq = VECTOR_PWDNC - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    local_irq_restore(flags);
}
EXPORT_SYMBOL(mt85xx_unmask_irq);

static struct irq_chip mt85xx_irqchip =
{
    .irq_ack    = mt85xx_mask_ack_irq_num,
    .irq_mask   = mt85xx_mask_irq_num,
    .irq_unmask = mt85xx_unmask_irq_num,
};

static void __init mt85xx_init_irq(void)
{
#if 1
    unsigned long flags;
    unsigned int irq;
    local_irq_save(flags);
    
	printk("mt85xx_init_irq : turn off all the irq\r\n");
	
    /* turn all irq off */
    __bim_writel(0, REG_IRQEN);
    __bim_writel(0, REG_IRQEN2);
    __bim_writel(0, REG_IRQEN3);

    /* clear all pending irq */
    __bim_writel(0xFFFFFFFF, REG_IRQCL);
    __bim_writel(0xFFFFFFFF, REG_IRQST);

    __bim_writel(0xFFFFFFFF, REG_IRQCL2);
    __bim_writel(0xFFFFFFFF, REG_IRQST2);

    __bim_writel(0xFFFFFFFF, REG_IRQCL3);
    __bim_writel(0xFFFFFFFF, REG_IRQST3);

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
	printk("mt85xx_init_irq : Mt3360 clear all pending irq\r\n");

    __bim_writel(0xFFFFFFFF, REG_IRQCL4);
    __bim_writel(0xFFFFFFFF, REG_IRQST4);
#endif

    /* Enable ARM interrupt, particularly for clearing PDWNC IRQ level 2 */
    arch_pwr_Up();

    //configure IRQ handlers
    for (irq = 0; irq < NR_IRQS; irq++)
    {
        dynamic_irq_init(irq);
        irq_set_chip(irq, &mt85xx_irqchip);
        irq_set_handler(irq, handle_level_irq);
        set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
    }

    /* for some reason, people believe we have to do above */
    local_irq_restore(flags);

#endif
}

#if CONFIG_FAST_POWER_DOWN
void  mt85xx_close_irq(void)
{
	unsigned long flags;

	local_irq_save(flags);
	 /* turn all irq off */
     __bim_writel(0, REG_IRQEN);
     __bim_writel(0, REG_IRQEN2);
     __bim_writel(0, REG_IRQEN3);
     local_irq_restore(flags);

}
#endif

/*======================================================================
 * timer interrupt
 */
static unsigned long _timer_tick_count;
/*
static u32 _CKGEN_GetXtalClock(void)
{
    ///  FIXME: temporary
    return 27000000;
}
*/
/*
#define TICKS_PER_SECOND 100
#define BIM_BASE					(IO_VIRT + 0x08000)
#define BIM_REG32(offset)           IO_REG32(BIM_BASE, offset)
#define BIM_READ32(off)             IO_READ32(BIM_BASE, (off))
#define BIM_WRITE32(off, val)       IO_WRITE32(BIM_BASE, (off), (val))

#define REG_IRQCL                   0x0038      // RISC IRQ Clear Register
#define VECTOR_T0                   3           // Timer 0
#define TIMER_ID                    2
#define TIMER_VECTOR                (VECTOR_T0 + TIMER_ID)
#define TIMER_IRQ                   (1 << TIMER_VECTOR)
#define TIMER_LOW_REG               (REG_RW_TIMER0_LOW + (TIMER_ID * 8))
#define TIMER_HIGH_REG              (REG_RW_TIMER0_HIGH + (TIMER_ID * 8))
#define TIMER_LIMIT_LOW_REG         (REG_RW_TIMER0_LLMT + (TIMER_ID * 8))
#define TIMER_LIMIT_HIGH_REG        (REG_RW_TIMER0_HLMT + (TIMER_ID * 8))
#define TCTL_TIMER_EN               (TMR_CNTDWN_EN(TIMER_ID))
#define TCTL_TIMER_AL               (TMR_AUTOLD_EN(TIMER_ID))
#define MAX_TIMER_INTERVAL          (0xffffffff)
*/

/*
 * IRQ handler for the timer
 */
static irqreturn_t
mt85xx_timer_interrupt(int irq, void *dev_id)
{
    u32 regval32;

    /* tick system */
    timer_tick();
    _timer_tick_count++;


    //set 1 to clear
    regval32 = (1 << irq);
    __bim_writel(regval32, REG_IRQCL);

    regval32 = (1 << irq);
    __bim_writel(regval32, REG_IRQST);

    return IRQ_HANDLED;
}

static struct irqaction mt85xx_timer_irq =
{
    .name       = "mt85xx Timer Tick",
    .flags      = IRQF_DISABLED | IRQF_TIMER,
    .handler    = mt85xx_timer_interrupt,
};

/* Valid TIMER_ID: 0, 1, 2 */
#define SYS_TIMER_ID        0
#define HP_TIMER_ID         2

#define SYS_TOFST           (SYS_TIMER_ID * 8)
#define HP_TOFST            (HP_TIMER_ID * 8)

#define SYS_TIMER_IRQ       (VECTOR_T0 +  SYS_TIMER_ID)
#define SYS_TIMER_REG_L     (REG_RW_TIMER0_LOW + SYS_TOFST)
#define SYS_TIMER_REG_H     (REG_RW_TIMER0_HIGH + SYS_TOFST)
#define SYS_TIMER_LIMIT_REG_L (REG_RW_TIMER0_LLMT + SYS_TOFST)
#define SYS_TIMER_LIMIT_REG_H (REG_RW_TIMER0_HLMT + SYS_TOFST)
#define SYS_TIMER_EN        (TCTL_T0EN << SYS_TOFST)
#define SYS_TIMER_AL        (TCTL_T0AL << SYS_TOFST)

#define HP_TIMER_IRQ       (IRQ_TIMER0 +  HP_TIMER_ID)
#define HP_TIMER_REG_L     (REG_RW_TIMER0_LOW + HP_TOFST)
#define HP_TIMER_REG_H     (REG_RW_TIMER0_HIGH + HP_TOFST)
#define HP_TIMER_LIMIT_REG_L (REG_RW_TIMER0_LLMT + HP_TOFST)
#define HP_TIMER_LIMIT_REG_H (REG_RW_TIMER0_HLMT + HP_TOFST)
#define HP_TIMER_EN        (TCTL_T0EN << HP_TOFST)
#define HP_TIMER_AL        (TCTL_T0AL << HP_TOFST)

/* bus ticks at 135MHz, actually 1/2 CPU freq */
#define SYSTEM_BUS_CLOCK            135000000UL

/* we tick kernel at 100Hz */
#define SYS_TIMER_INTERVAL          (SYSTEM_BUS_CLOCK / HZ)

static void __init mt85xx_timer_init(void)
{
    uint32_t u4_cfg_clock_per_ticks;
	
    /* disable timer interrupt */
    //mt85xx_mask_irq_num(VECTOR_T0);
		mt85xx_mask_irq(VECTOR_T0);
    /*set priority*/
    //BSP_SetIrqPriority(VECTOR_T0, IRQ_PRIORITY_HIGH);

    /* Set Timer Limit */
    u4_cfg_clock_per_ticks = u4_get_bus_clock() / CONFIG_HZ;
    __bim_writel(u4_cfg_clock_per_ticks, REG_RW_TIMER0_LIM_OFFSET);


    /* Start timer irq */
    __bim_writel(VAL_T0_AUTOLOAD | VAL_T0_ENABLE, REG_RW_TIMER_CTRL_OFFSET);

    _timer_tick_count = 0;
    

    /* set system irq */
    setup_irq(SYS_TIMER_IRQ, &mt85xx_timer_irq);
	

    /* enable timer interrupt */
    //mt85xx_unmask_irq_num(VECTOR_T0);
    mt85xx_unmask_irq(VECTOR_T0);
}

static unsigned long mt85xx_gettimeoffset(void)
{
    return 0;
}

static struct sys_timer mt85xx_timer =
{
    .init       = mt85xx_timer_init,
//    .offset     = mt85xx_gettimeoffset,
};

unsigned long mt85xx_timer_read(int nr)
{
    return __bim_readl((REG_RW_TIMER0_VALUE + (nr * 8)));
}
EXPORT_SYMBOL(mt85xx_timer_read);
/*
unsigned long long mt85xx_timer_read_ll(int nr)
{
    unsigned long long utime;
    unsigned long time_lo1, time_lo2;
    unsigned long time_hi1, time_hi2;

    do
    {
        time_lo1 = __bim_readl(REG_RW_TIMER0_LOW + (nr * 8));
        time_hi1 = __bim_readl(REG_RW_TIMER0_HIGH + (nr * 8));
        time_lo2 = __bim_readl(REG_RW_TIMER0_LOW + (nr * 8));
        time_hi2 = __bim_readl(REG_RW_TIMER0_HIGH + (nr * 8));
    } while (time_hi1 == time_hi2);

    utime = ((unsigned long long)time_hi1 << 32) + (unsigned long long)time_lo1;

    return utime;
}
EXPORT_SYMBOL(mt85xx_timer_read_ll);
*/
/* io map */

static struct map_desc mt85xx_io_desc[] __initdata =
{
    {IO_VIRT, __phys_to_pfn(IO_PHYS), IO_SIZE, MT_DEVICE},
    {NOR_IO_VIRT,   __phys_to_pfn(NOR_IO_PHYS),   NOR_IO_SIZE,    MT_DEVICE},
    #if defined(CONFIG_CACHE_L2X0)
    {L2_IO_VIRT, __phys_to_pfn(L2_IO_PHYS), L2_IO_SIZE, MT_DEVICE},
    #endif
    {MULTIMEDIA_IO_VIRT,   __phys_to_pfn(MULTIMEDIA_IO_PHYS),  MULTIMEDIA_IO_SIZE ,    MT_DEVICE},
	{MTK33XX_SRAM_VIRT,   __phys_to_pfn(MTK33XX_SRAM_PHYS),  MTK33XX_SRAM_SIZE ,    MT_DEVICE},
};

static void __init mt85xx_map_io(void)
{
    iotable_init(mt85xx_io_desc, ARRAY_SIZE(mt85xx_io_desc));
}

#if 1
struct platform_device mt85xx_device_ts = {
    .name = "mt33xx_tsadcc",
    .id   = -1,
};

struct platform_device mt85xx_device_keypad = {
    .name = "mt33xx_keypad",
    .id   = -1,
};

static struct resource mt3360_sd_rsrc[] = {
    [0] = {
                .start = MT3360_MSDC0_BASE,
                .end   = MT3360_MSDC0_BASE + ECO_VER,
                .flags = IORESOURCE_MEM
          },
	[1] = {
				.start = VECTOR_MSDC0,  //DMA IRQ
				.end   = 70,    //GPIO used to detect card 
				.flags = IORESOURCE_IRQ 
		  },
	[2] = {
				.start = MT3360_MSDC1_BASE,
				.end   = MT3360_MSDC1_BASE + ECO_VER,
				.flags = IORESOURCE_MEM
		  },
	[3] = {
				.start = VECTOR_MSDC1,
				.end   = 71,
				.flags = IORESOURCE_IRQ
		  },
	[4] = {
				.start = MT3360_MSDC2_BASE,
				.end   = MT3360_MSDC2_BASE + ECO_VER,
				.flags = IORESOURCE_MEM
		  },
	[5] = {
				.start = VECTOR_MSDC2,
				.end   = 72,
				.flags = IORESOURCE_IRQ
		  },		  
};

struct platform_device mt85xx_device_sd = {
    .name = "mt3360_sd",
    .id   = 0,
    .num_resources  = ARRAY_SIZE(mt3360_sd_rsrc),
    .resource       = mt3360_sd_rsrc
};


struct platform_device mt85xx_device_rtc = {
    .name = "mt85xx_rtc",
    .id   = 0,
};
#endif

#define NAND_BLOCK_SIZE		SZ_2M

static struct mtd_partition mt33xx_nand_partitions[] = {
	{
		/* UBL (a few copies) plus U-Boot */
		.name		= "boot",
		.offset		= 0,
		.size		= SZ_4M,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	}, {
		/* U-Boot environment */
		.name		= "env",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0,
	}, {
		.name		= "arm2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0,
	}, {
		.name		= "logo",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0,
	}, {
		.name		= "system",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_256M+SZ_32M+SZ_8M+SZ_4M),
		.mask_flags	= 0,
	}, {
		.name		= "data",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_256M+SZ_32M+SZ_8M+SZ_4M),
		.mask_flags	= 0,
	}, {
		.name		= "cache",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_64M+SZ_32M+SZ_4M),
		.mask_flags	= 0,
	}, {
		.name		= "swap",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_256M+SZ_32M+SZ_8M+SZ_4M),
		.mask_flags	= 0,
	}, {
		.name		= "recovery",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "misc",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0,
	}, {
		.name		= "backup",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_32M+SZ_16M+SZ_2M),
		.mask_flags	= 0,
	}, {
		.name		= "metazone",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "dvp",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "data4write",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_32M+SZ_16M+SZ_2M),
		.mask_flags	= 0,
	}, {
		.name		= "usrdata",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct mtk_nand_pdata mt33xx_nand_data = {
	.partitions			= mt33xx_nand_partitions,
	.nr_partitions		= ARRAY_SIZE(mt33xx_nand_partitions),
};



struct platform_device mt33xx_device_nand = {
    .name = "mt33xx_nand",
    .id   = 0,
	.dev			= {
		.platform_data	= &mt33xx_nand_data,
	},

};

struct platform_device mt33xx_device_battery = {
    .name = "mt33xx_battery",
    .id   = 0,
};

struct platform_device mt33xx_device_i2c = {
    .name = "mt33xx_i2c",
    .id   = 0,
};

static struct platform_device *mt85xx_devices[] __initdata = {

    &mt85xx_device_rtc,
    &mt85xx_device_ts,
    &mt85xx_device_keypad,

    &mt85xx_device_sd,
    &mt33xx_device_nand,
    &mt33xx_device_battery,
    &mt33xx_device_i2c,

};

typedef struct {
    uint32_t u4AfifoRPtr;
    uint32_t u4AfifoWPtr;
    uint32_t u4AfifoSA;
    uint32_t u4AfifoEA;
} POSINFO_T;

typedef struct {
    POSINFO_T posInfo[4];
} AFIFO_POSINFO_T;

AFIFO_POSINFO_T rAFifo_PosInfo;
AFIFO_POSINFO_T *m_afifo_info;

EXPORT_SYMBOL(m_afifo_info);
int gpio_inout_sel(unsigned gpio, int dir);
#define IO_BASE_VA   (0xFD000000)
void mt33xx_power_off(void)
{	

   unsigned  tmp = 0;
	/*Config GPIO149 as gpio function*/
	
	tmp = IO_READ32(IO_BASE_VA, 0x68) & (~(7U << 22));
	IO_WRITE32(IO_BASE_VA, 0x68, tmp);
	tmp = IO_READ32(IO_BASE_VA, 0x240f8) & (~(1U << 7));
	IO_WRITE32(IO_BASE_VA, 0x240f8, tmp);
	tmp = IO_READ32(IO_BASE_VA, 0x60) & (~(3U << 28));
	IO_WRITE32(IO_BASE_VA, 0x60, tmp);
	tmp = IO_READ32(IO_BASE_VA, 0x240f4)  | (1U << 7);
	IO_WRITE32(IO_BASE_VA, 0x240f4, tmp);

	/* config the out mode*/
	gpio_inout_sel(149,1);
	/*enable the out pull  down the exception for mt330_ap_pdwnc_register*/
	tmp = IO_READ32(IO_BASE_VA, 0x240f0)  | (1U << 4);
	IO_WRITE32(IO_BASE_VA, 0x240f0, tmp);
	tmp = IO_READ32(IO_BASE_VA, 0x240f0)  | (1U << 6);
	IO_WRITE32(IO_BASE_VA, 0x240f0, tmp);
	
	/*output to low*/
	
	tmp = IO_READ32(IO_BASE_VA, 0x240d4) | (1 << 7 );
	IO_WRITE32(IO_BASE_VA, 0x240d4, tmp);
	tmp = IO_READ32(IO_BASE_VA, 0x240d8) & ~ (1 << 7 );
	IO_WRITE32(IO_BASE_VA, 0x240d8, tmp);
	while(1);


   return;
}

static void  __init mt85xx_init(void)
{
	
	printk("MACHINE_START: mt85xx_init\r\n");
	
    IC_VERSION_T eVer;
    eVer = BSP_GetIcVersion();

	memset(&rAFifo_PosInfo, 0, sizeof(rAFifo_PosInfo));
	m_afifo_info = &rAFifo_PosInfo;
#ifdef MT3353_YECON_BOARD
	pm_power_off = mt33xx_power_off; 
#endif

  #if defined(CONFIG_CACHE_L2X0)
    {
      #if CONFIG_MT85XX_FPGA
        {
            printk("FPGA enable L2X0\n");
            l2x0_init(L2_IO_VIRT, 0x30020000, 0xffffffff);
           // BSP_L2Cache_Operation_Init();
        }
      #else
        //if(L2Reg_READ32(L2X0_CTRL) & 0x01)
        {
            //printk("enable L2X0\n");
            /* 128Kb (16Kb/way) 8-way associativity. evmon/parity/share enabled. */
            l2x0_init(L2_IO_VIRT, 0x30020000, 0xffffffff);//(0x50000000, 0x00730000, 0xfe000fff);
           // BSP_L2Cache_Operation_Init();
        }
      #endif
    }
  #endif
#if CONFIG_MT33XX_ISR_WATCH
    mt85xx_isr_watch_init();
#endif
#if CONFIG_MT33XX_CS_WATCH
    mt85xx_CS_watch_init();
#endif

    platform_add_devices(mt85xx_devices,
                 ARRAY_SIZE(mt85xx_devices));
}


static void __init mt85xx_fixup(struct machine_desc *desc, struct tag *tag,
                char **cmdline, struct meminfo *mi)
{
	printk("MACHINE_START: mt85xx_fixup\r\n");
#if 0
        tag->hdr.tag = ATAG_CORE;
        tag->hdr.size = tag_size(tag_core);
        tag->u.core.flags = 0;
        tag->u.core.pagesize = PAGE_SIZE;
        tag->u.core.rootdev = 0x0100;

        tag = tag_next(tag);
        tag->hdr.tag = ATAG_MEM;
        tag->hdr.size = tag_size(tag_mem32);
        tag->u.mem.size = 0x00000000;
        tag->u.mem.start = 0x18000000;
/*
        tag = tag_next(tag);
        tag->hdr.tag = ATAG_INITRD;
        tag->hdr.size = tag_size(tag_initrd);
        tag->u.initrd.start = 0x06800000;
        tag->u.initrd.size = 0xf00000;
*/
        tag = tag_next(tag);
        tag->hdr.tag = ATAG_NONE;
        tag->hdr.size = 0;
#endif
}

#if 1
/* added by mtk40505 begin */

static struct platform_device demuxer_device = {
    .name = "android_demuxer",
    .id = 0
};


static int __init mt_demuxer_init(void) 
{
	platform_device_register(&demuxer_device);
}

arch_initcall(mt_demuxer_init);

/* added by mtk40505 end  */
#endif

MACHINE_START(MT85XX, "MT33xx")
//    .phys_io        = MT85XX_PA_UART,
//    .io_pg_offst    = (MT85XX_VA_UART >> 18) & 0xfffc,
//    .boot_params    = MT85XX_SDRAM_PA + 0x100,
    .atag_offset    = MT85XX_SDRAM_PA + 0x100,// 0x100,
    .map_io         = mt85xx_map_io,
    .init_irq       = mt85xx_init_irq,
    .timer          = &mt85xx_timer,
    .init_machine   = mt85xx_init,
    .fixup          = mt85xx_fixup,
MACHINE_END

