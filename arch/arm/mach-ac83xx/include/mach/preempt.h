/*
 * arch/arm/mach-ac83xx/include/mach/preempt.h
 *
 * Copyright (C) 2009 MediaTek Inc.
 *
 */

#ifndef _ASM_ARCH_PREEMT_H
#define _ASM_ARCH_PREEMT_H

static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
	return (stop - start);
}

extern unsigned long ac83xx_timer_read(int nr);

/*
 * timer 1 runs @ 100Mhz, 100 tick = 1 microsecond
 * and is configured as a count down timer.
 */

#define TICKS_PER_USEC                  100
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define readclock()                     (~ac83xx_timer_read(1))
#define clock_to_usecs(x)               ((x) / TICKS_PER_USEC)
#define INTERRUPTS_ENABLED(x)           (!(x & PSR_I_BIT))

#endif

