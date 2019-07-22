/*
 *  linux/arch/arm/mach-ac83xx/include/mach/system.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <mach/hardware.h>
#include <asm/io.h>

static inline void arch_idle(void)
{
	/*
	 * switch off the system and wait for interrupt
	 */
	cpu_do_idle();
}

extern void arch_reset(char mode, const char *cmd);


#endif
