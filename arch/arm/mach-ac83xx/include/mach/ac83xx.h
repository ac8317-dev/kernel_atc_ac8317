#ifndef __ASM_ARCH_AC83XX_H
#define __ASM_ARCH_AC83XX_H

#include <mach/chip_ver.h>
#include <mach/irqs.h>

/*
 * I/O mapping
 */
#define IO_PHYS             0xf0000000
#define IO_VIRT             0xFD000000  

#define AC83XX_IO_PA  0xF0000000
#define AC83XX_IO_VA  0xFD000000
#define AC83XX_IO_SIZE 0x01000000

#define AC83XX_GIC_PA 0xF1000000
#define AC83XX_GIC_VA 0xFE000000
#define AC83XX_GIC_SIZE 0x00004000

#define AC83XX_L2_IO_VA 0xFE000000
#define AC83XX_L2_IO_PA 0x50000000
#define AC83XX_L2_IO_SIZE 0x1000

#define AC83XX_SRAM_VA   0xFC000000
#define AC83XX_SRAM_PA   0xF4000000
#define AC83XX_SRAM_SIZE 0x6000


#define AC83XX_SDRAM_PA 0x0

#define AC83XX_SDRAM_PA      0x00008000
#define AC83XX_PA_UART      (IO_PHYS + 0x0000C000)     //0x7000c000
#define AC83XX_VA_UART      (IO_VIRT + 0x0000C000)     //0xe000c000
#define AC83XX_UART_SIZE    0x40

#define BIM_BASE_PA					   (AC83XX_IO_PA + 0x00008000)
#define BIM_BASE_VA                    (AC83XX_IO_VA + 0x00008000)
#define RS232_BASE_PA				   (AC83XX_IO_PA + 0x0000C000)
#define RS232_BASE_VA                  (AC83XX_IO_VA + 0x0000C000)
#define CKGEN_BASE_VA                  (AC83XX_IO_VA + 0x00000)
#define NFI24_BASE_VA				   (AC83XX_IO_VA + 0x1E400)
#define NFIECC24_BASE_VA			   (AC83XX_IO_VA + 0x1EC00)
#define SYSTEM_BUS_CLOCK                324000000UL
#define SPM_BASE_VA   (AC83XX_IO_VA + 0x48000)

/* IO register definitions */
#define BIM_VIRT                    (IO_VIRT + 0x00008000)

/* BIM2 define */
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_AC83XX)
#define BIM2_VIRT                   (IO_VIRT + 0x00045000)
#endif

#define RS232_VIRT                  (IO_VIRT + 0x0000C000)
#define AUD_VIRT                              (IO_VIRT + 0x05000)
#define CKGEN_VIRT                              (IO_VIRT + 0x00000)
#define RS232_PHYS                  0x0000C000
#define PDWNC_VIRT                  (IO_VIRT + 0x00024000)
//#define BIM_PHYS 

// IRQ/FIQ
#define REG_IRQST           0x0030        //RISC L1 IRQ Status Register
#define REG_IRQEN           0x0034        //RISC L1 IRQ Enable Register
#define REG_IRQCL           0x0038        //RISC L1 IRQ Clear Register
#define REG_FIQST           0x003C        //RISC L1 FIQ Status Register
#define REG_FIQEN           0x0040        //RISC L1 FIQ Enable Register
#define REG_FIQCL           0x0044        //RISC L1 FIQ Clear Register

#define REG_IRQST2          0x0048        //RISC L2 IRQ Status Register
#define REG_IRQEN2          0x004C        //RISC L2 IRQ Enable Register
#define REG_IRQCL2          0x0050        //RISC L2 IRQ Clear Register
#define REG_FIQST2          0x0054        //RISC L2 FIQ Status Register
#define REG_FIQEN2          0x0058        //RISC L2 FIQ Enable Register
#define REG_FIQCL2          0x005C        //RISC L2 FIQ Clear Register

#define REG_IRQST3          0x0060        //RISC L3 IRQ Status Register
#define REG_IRQEN3          0x0064        //RISC L3 IRQ Enable Register
#define REG_IRQCL3          0x0068        //RISC L3 IRQ Clear Register
#define REG_FIQST3          0x006C        //RISC L3 FIQ Status Register
#define REG_FIQEN3          0x0070        //RISC L3 FIQ Enable Register
#define REG_FIQCL3          0x0074        //RISC L3 FIQ Clear Register

#define REG_IRQST4          0x0230        //RISC L4 IRQ Status Register
#define REG_IRQEN4          0x0234        //RISC L4 IRQ Enable Register
#define REG_IRQCL4          0x0238        //RISC L4 IRQ Clear Register
#define REG_FIQST4          0x023C        //RISC L4 FIQ Status Register
#define REG_FIQEN4          0x0240        //RISC L4 FIQ Enable Register
#define REG_FIQCL4          0x0244        //RISC L4 FIQ Clear Register

// Timer settings

/*
 * IO Macro
 */
#define HAL_WRITE32(_reg_, _val_)   		(*((volatile uint32_t*)(_reg_)) = (_val_))
#define HAL_READ32(_reg_)           		(*((volatile uint32_t*)(_reg_)))

#define IO_READ32(base, offset)                 HAL_READ32((base) + (offset))
#define IO_WRITE32(base, offset, value)         HAL_WRITE32((base) + (offset), (value))
#define	IO_REG32(base, offset)									HAL_READ32((base) + (offset))

//#define CKGEN_READ32(offset)           		IO_READ32(CKGEN_VIRT, (offset))
//#define CKGEN_WRITE32(offset, value)   		IO_WRITE32(CKGEN_VIRT, (offset), (value))

#define BIM_READ32(offset)           		IO_READ32(BIM_VIRT, (offset))
#define BIM_WRITE32(offset, value)   		IO_WRITE32(BIM_VIRT, (offset), (value))
#define BIM_REG32(offset)   						IO_REG32(BIM_VIRT, (offset))

#define BIM2_READ32(offset)           		IO_READ32(BIM2_VIRT, (offset))
#define BIM2_WRITE32(offset, value)   		IO_WRITE32(BIM2_VIRT, (offset), (value))
#define BIM2_REG32(offset)   						IO_REG32(BIM2_VIRT, (offset))

#define PDWNC_READ32(offset)           		IO_READ32(PDWNC_VIRT, (offset))
#define PDWNC_WRITE32(offset, value)   		IO_WRITE32(PDWNC_VIRT, (offset), (value))

#endif /* __ASM_ARCH_AC83XX_H */

