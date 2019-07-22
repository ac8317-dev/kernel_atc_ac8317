/*
 *  linux/include/asm-arm/arch-mt85xx/irqs.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */

#if !defined(IRQS_H)
#define IRQS_H
#include "chip_ver.h"
#include "mt33xx_irqs_vector.h"

//----------------------------------------------------------------------------
// IRQ related define
#define IRQ_TIMER0		(0x1 << VECTOR_T0)
#define MASK_IREQEN_RS232       (0x1 << VECTOR_RS232_1)  //RS232 enable bit

//#define IRQ_TIMER0              3

// Interrupt vectors
//#define VECTOR_TIMER0				3
//#define VECTOR_RS232				17


//#define _IRQ(v)         (1U << (v)) // IRQ bit by vector
//#define _MISCIRQ(v)     (1U << (v - 32))    // MISC IRQ bit by vector                                

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
#define NR_IRQS                 160
#else
#define NR_IRQS                 128
#endif
#define NR_IRQS_ALIGN           5    /* 1NR_IRQS must align to 32 (2^5)bit */

#if (NR_IRQS & ((1<<NR_IRQS_ALIGN)-1))
#error "NR_IRQS must be 32*n."
#endif

#ifndef __ASSEMBLY__

//#include <mach/inc/x_typedef.h>
//#include <x_typedef.h>

//----------------------------------------------------------------------------
// inline functions
static inline int _bim_is_vector_valid(unsigned int irq_num)
{
    return (irq_num < NR_IRQS);
}

#endif

#endif /*IRQS_H*/

