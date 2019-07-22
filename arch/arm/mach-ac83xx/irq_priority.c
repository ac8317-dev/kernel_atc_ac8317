/*
 *  linux/arch/arm/mach-ac83xx/irq_priority.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * pll settings.
 */

//==================================================
// header files
//==================================================
#include <linux/module.h>
#include <mach/hardware.h>
//#include <x_typedef.h>
#include <mach/irqs.h>
#include <mach/irq_priority.h>
#include <asm/io.h>


//==================================================
// Define
//==================================================
#define _CHK_IRQ_PRIORITY(mask_array, irq_num) \
	((mask_array[irq_num>>NR_IRQS_ALIGN]) & (1<<(irq_num&((1<<NR_IRQS_ALIGN)-1))))

#define _SET_IRQ_PRIORITY(mask_array, irq_num, on) \
	(on) ? (mask_array[irq_num>>NR_IRQS_ALIGN] |=  (1<<(irq_num&((1<<NR_IRQS_ALIGN)-1)))) \
	     : (mask_array[irq_num>>NR_IRQS_ALIGN] &= ~(1<<(irq_num&((1<<NR_IRQS_ALIGN)-1))))

//==================================================
// Global variable
//==================================================
/* irq priority mask */
uint32_t u4_pm_high[NR_IRQS >> NR_IRQS_ALIGN] = {0x00000000, 0x00020000, 0x00000000, 0x00000000};
uint32_t u4_pm_mid[NR_IRQS >> NR_IRQS_ALIGN] = {0x00000008, 0x00000000, 0x00000000, 0x00000000};
uint32_t u4_pm_low[NR_IRQS >> NR_IRQS_ALIGN] = {0xFFFFFFF7, 0xFFFDFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

#if CONFIG_AC83XX_IRQ_PRIORITY_ENABLE
static u32 u4_pm_priority[NR_IRQS][(NR_IRQS >> NR_IRQS_ALIGN)] = {{0}};
#endif

//==================================================
// Private functions
//==================================================


//==================================================
// public functions
//==================================================
IRQ_PRIORITY BSP_GetIrqPriority(uint32_t irq_num)
{
    /* check irq_num */
    if (!_bim_is_vector_valid(irq_num))
    {
        while (1)
            ;
        return IRQ_PRIORITY_NONE;
    }

    /* get irq priority */
    if(_CHK_IRQ_PRIORITY(u4_pm_high, irq_num)!=0)
        return IRQ_PRIORITY_HIGH;

    if(_CHK_IRQ_PRIORITY(u4_pm_mid, irq_num)!=0)
        return IRQ_PRIORITY_MID;

    if(_CHK_IRQ_PRIORITY(u4_pm_low, irq_num)!=0)
        return IRQ_PRIORITY_LOW;

	/* not found - ERROR*/
	while(1);
	
	return IRQ_PRIORITY_NONE;
}
EXPORT_SYMBOL(BSP_GetIrqPriority);

uint32_t BSP_SetIrqPriority(uint32_t irq_num, IRQ_PRIORITY ePri)
{
	uint32_t ret = 0;
    /* check irq_num */
    if (!_bim_is_vector_valid(irq_num))
    {
        while (1)
            ;
        return ret;
    }

    /* set irq priority */
    switch (ePri)
    {
        case IRQ_PRIORITY_LOW:
            _SET_IRQ_PRIORITY(u4_pm_low, irq_num, 1);
            _SET_IRQ_PRIORITY(u4_pm_mid, irq_num, 0);
            _SET_IRQ_PRIORITY(u4_pm_high, irq_num, 0);
            break;
        case IRQ_PRIORITY_MID:
            _SET_IRQ_PRIORITY(u4_pm_mid, irq_num, 1);
            _SET_IRQ_PRIORITY(u4_pm_low, irq_num, 0);
            _SET_IRQ_PRIORITY(u4_pm_high, irq_num, 0);
            break;
        case IRQ_PRIORITY_HIGH:
            _SET_IRQ_PRIORITY(u4_pm_high, irq_num, 1);
            _SET_IRQ_PRIORITY(u4_pm_mid, irq_num, 0);
            _SET_IRQ_PRIORITY(u4_pm_low, irq_num, 0);
            break;
        case IRQ_PRIORITY_NONE:
        default:
            while(1);
            return ret;
            break;
    }
	ret = 1;
	
	return ret;
}
EXPORT_SYMBOL(BSP_SetIrqPriority);

#if CONFIG_AC83XX_IRQ_PRIORITY_ENABLE
static inline void BitwiseOR(u32 *u4Des, u32 *u4Src, u32 u4Len)
{
    while(u4Len)
    {
       u4Len--;
       u4Des[u4Len] |= u4Src[u4Len];
    }
}

static inline void BitwiseAND(u32 *u4Des, u32 *u4Src, u32 u4Len)
{
    while(u4Len)
    {
       u4Len--;
       u4Des[u4Len] &= u4Src[u4Len];
    }
}


static inline void BitwiseInvertAND(u32 *u4Des, u32 *u4Src, u32 u4Len)
{
	while(u4Len)
	{
	   u4Len--;
	   u4Des[u4Len] &= (~(u4Src[u4Len]));
	}
}

static inline u32 __bim_readl(u32 regaddr32)
{
    return __raw_readl(BIM_VIRT + regaddr32);
}

static inline void __bim_writel(u32 regval32, u32 regaddr32)
{
    __raw_writel(regval32, BIM_VIRT + regaddr32);
}

void ac83xx_IrqEnter(unsigned int irq_num)
{
    u32 regval32[NR_IRQS >> NR_IRQS_ALIGN];
    unsigned long flags;
    int i;
	
	if (!_bim_is_vector_valid(irq_num))
    {
        while (1)
            ;
        return;
    }
	
	local_irq_save(flags);
	
    for(i = 0; i < (NR_IRQS >> NR_IRQS_ALIGN); i++)
    {
       u4_pm_priority[irq_num][i] = 0; 
    }
	
    switch(BSP_GetIrqPriority(irq_num))
    {
       case IRQ_PRIORITY_HIGH:
	   	  BitwiseOR(u4_pm_priority[irq_num], (u32 *)u4_pm_high, (NR_IRQS >> NR_IRQS_ALIGN));
	   case IRQ_PRIORITY_MID:
	   	  BitwiseOR(u4_pm_priority[irq_num], (u32 *)u4_pm_mid, (NR_IRQS >> NR_IRQS_ALIGN));
	   case IRQ_PRIORITY_LOW:
	   	  BitwiseOR(u4_pm_priority[irq_num], (u32 *)u4_pm_low, (NR_IRQS >> NR_IRQS_ALIGN));
	   	  break;
		  
	   default:
	   	  /*ERROR*/
	   	  while(1);
	   	  break;
    }
	
	regval32[0] = __bim_readl(REG_IRQEN);
	regval32[1] = __bim_readl(REG_IRQEN2);
	regval32[2] = __bim_readl(REG_IRQEN3);
	regval32[3] = (u32)PDWNC_READ32(REG_RW_INTEN);

    BitwiseAND(u4_pm_priority[irq_num], regval32, (NR_IRQS >> NR_IRQS_ALIGN));
	BitwiseInvertAND(regval32, u4_pm_priority[irq_num], (NR_IRQS >> NR_IRQS_ALIGN));
	
    //regval32[irq_num/(1<<NR_IRQS_ALIGN)] &= (~(1<<(irq_num%(1<<NR_IRQS_ALIGN))));

    /* check pdwn */
	//if(regval32[3] != 0)
	//{
    //    regval32[1] |= (1 << (VECTOR_PWDNC - 32));
	//}
		
    __bim_writel(regval32[0], REG_IRQEN);
    __bim_writel(regval32[1], REG_IRQEN2);
    __bim_writel(regval32[2], REG_IRQEN3);
    //PDWNC_WRITE32(REG_RW_INTEN, (uint32_t)regval32[3]);

    local_irq_restore(flags);
}

void ac83xx_IrqExit(unsigned int irq_num)
{
    u32 regval32[NR_IRQS >> NR_IRQS_ALIGN];
    unsigned long flags;

	if (!_bim_is_vector_valid(irq_num))
    {
        while (1)
            ;
        return;
    }
	
	local_irq_save(flags);
	
    regval32[0] = __bim_readl(REG_IRQEN);
	regval32[1] = __bim_readl(REG_IRQEN2);
	regval32[2] = __bim_readl(REG_IRQEN3);
	regval32[3] = (u32)PDWNC_READ32(REG_RW_INTEN);

    //regval32[irq_num/(1<<NR_IRQS_ALIGN)] |= (1<<(irq_num%(1<<NR_IRQS_ALIGN))); 

	BitwiseOR(regval32, u4_pm_priority[irq_num], (NR_IRQS >> NR_IRQS_ALIGN));
	
    /* check pdwn */
	//if(regval32[3] != 0)
	//{
    //    regval32[1] |= (1 << (VECTOR_PWDNC - 32));
	//}
	
	__bim_writel(regval32[0], REG_IRQEN);
    __bim_writel(regval32[1], REG_IRQEN2);
    __bim_writel(regval32[2], REG_IRQEN3);
    //PDWNC_WRITE32(REG_RW_INTEN, (uint32_t)regval32[3]);

	local_irq_restore(flags);
}
#endif

