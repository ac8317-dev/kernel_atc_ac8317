/*
 *  linux/include/asm-arm/arch-mt85xx/irqs.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */

#ifndef IRQ_PRIORITY_H
#define IRQ_PRIORITY_H

//#include <x_typedef.h>

#define CONFIG_MT85XX_IRQ_PRIORITY_ENABLE  1

//----------------------------------------------------------------------------
// IRQ Priority
typedef enum {
    IRQ_PRIORITY_NONE = 0,
    IRQ_PRIORITY_LOW,
    IRQ_PRIORITY_MID,
    IRQ_PRIORITY_HIGH
} IRQ_PRIORITY;

extern IRQ_PRIORITY BSP_GetIrqPriority(uint32_t irq_num);
extern uint32_t BSP_SetIrqPriority(uint32_t irq_num, IRQ_PRIORITY ePri);

extern void MT85xx_IrqEnter(unsigned int irq_num);
extern void MT85xx_IrqExit(unsigned int irq_num);

#endif /*IRQ_PRIORITY_H*/

