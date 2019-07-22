/*
 *  linux/include/asm-arm/arch-mt85xx/memory.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
 
#define PHYS_OFFSET	(0x00000000)
#define BUS_OFFSET	(0x00000000)

#define CONSISTENT_DMA_SIZE SZ_8M

/* TASK_SIZE¡BTASK_UNMAPPED_BASE 
   move to arch/arm/include/asm/memory.h
*/

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */


#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt(x) __phys_to_virt(x)

#define __arch_dma_to_virt(dev, x)	(void*)__phys_to_virt(x)
#define __arch_virt_to_dma(dev, x)	(dma_addr_t)__virt_to_phys(x)
/*
 * Convert a physical address to a Page Frame Number and back
 */

#define __arch_pfn_to_dma(dev, pfn)					\
	({								\
		/* __is_lbus_virt() can never be true for RAM pages */	\
		(dma_addr_t)__pfn_to_phys(pfn);				\
	})

#define __arch_dma_to_pfn(dev, addr)	__phys_to_pfn(addr)


/*
phys = virt - PAGE_OFFSET + PHYS_OFFSET
*/

/* 
#define __virt_to_bus(x)	((x) - PAGE_OFFSET)
#define __bus_to_virt(x)	((x) + PAGE_OFFSET)
#define __virt_to_phys(x)	((x) - PAGE_OFFSET)
#define __phys_to_virt(x)	((x) + PAGE_OFFSET)
*/
#define PCI0_BASE           0

#endif

