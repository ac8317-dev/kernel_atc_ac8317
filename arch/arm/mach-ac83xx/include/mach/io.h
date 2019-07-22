/*
 *  kernel/arch/arm/mach-ac83xx/include/mach/io.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <mach/memory.h>

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a)         ((void __iomem *)(PCI0_BASE + (a)))
#define __mem_pci(a)    (a)
#define __mem_isa(a)    (a)

#define __bim_writel(val,add)  __raw_writel(val,__io(BIM_BASE_VA+add))
#define __bim_readl(add)       __raw_readl(__io(BIM_BASE_VA+add))

#endif
