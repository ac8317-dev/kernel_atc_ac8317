#ifndef __ASM_ARCH_MT8530_H
#define __ASM_ARCH_MT8530_H

#include <mach/chip_ver.h>
#include <mach/irqs.h>

/*
 * I/O mapping
 */


#define IO_PHYS             0x70000000
#define IO_OFFSET           0x70000000

#ifndef IO_VIRT
//#define IO_VIRT             (IO_PHYS + IO_OFFSET)
#define IO_VIRT             0xFD000000    //temporarily marked because (IO_PHYS + IO_OFFSET) causes compiler warning - overflow, don't know why. Check this later.
#endif  // IO_VIRT

#define IO_SIZE             0x01000000  //assume temporarily

#define NOR_IO_PHYS         0x60000000
#define NOR_IO_VIRT         0xFC000000
#define NOR_IO_SIZE         0x01000000  // Max. 16 MB

#define L2_IO_PHYS         0x50000000
#define L2_IO_VIRT         0xFE000000
#define L2_IO_SIZE         0x1000
#define L2_CTRL			       0x100
#define L2_AUX_CTRL			   0x104
#define L2_CACHE_SYNC			 0x730
#define L2_INV_WAY			   0x77C
#define L2_CLEAN_INV_WAY	 0x7FC
#define L2_LOCKDOWN_WAY_D	 0x900
#define L2_LOCKDOWN_WAY_I	 0x904
#define MT85XX_CACHE_OPERATION_DEBUG 0

#define SVOIF_IO_VIRT      0xFE001000


#define MT85XX_SDRAM_PA   0x00008000
#define MT85XX_PA_UART      (IO_PHYS + 0x0000C000)     //0x7000c000
#define MT85XX_VA_UART      (IO_VIRT + 0x0000C000)     //0xe000c000
#define MT85XX_UART_SIZE    0x40

/*---------------------------------------------*/

/*
 *  MT8530 HAL
 */

/* Constant definitions */
#define MEMORY_ALIGNMENT            8

/* IO register definitions */
#define BIM_VIRT                    (IO_VIRT + 0x00008000)

/* BIM2 define */
#if ((CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT8550) || (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT8560)|| (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT8580))
#define BIM2_VIRT                   (IO_VIRT + 0x00008000)
#endif

#if ((CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT8555) || (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT8530 || CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360))
#define BIM2_VIRT                   (IO_VIRT + 0x00045000)
#endif

#define RS232_VIRT                  (IO_VIRT + 0x0000C000)
#define AUD_VIRT                              (IO_VIRT + 0x05000)
#define CKGEN_VIRT                              (IO_VIRT + 0x00000)
#define RS232_PHYS                  0x0000C000
#define PDWNC_VIRT                  (IO_VIRT + 0x00024000)
#define BIM_PHYS                    (IO_PHYS + 0x00008000)

// IRQ registers
//#define REG_RW_IRQEN_OFFSET     0x34         //IRQ enable register offset


//#define REG_WO_IRQCL_OFFSET     0x38         //IRQ clear register offset


//#define REG_RW_IRQST_OFFSET     0x3C         //IRQ status register
//----------------------------------------------------------------------------
// IRQ/FIQ
#define REG_IRQST           0x0030        //RISC L1 IRQ Status Register
#define REG_IRQEN           0x0034        //RISC L1 IRQ Enable Register
#define REG_IRQCL           0x0038        //RISC L1 IRQ Clear Register
#define REG_FIQST           0x003C        //RISC L1 FIQ Status Register
#define REG_FIQEN           0x0040        //RISC L1 FIQ Enable Register
#define REG_FIQCL           0x0044        //RISC L1 FIQ Clear Register

#define REG_IRQST2          0x0138        //RISC L2 IRQ Status Register
#define REG_IRQEN2          0x013C        //RISC L2 IRQ Enable Register
#define REG_IRQCL2          0x0140        //RISC L2 IRQ Clear Register
#define REG_FIQST2          0x0144        //RISC L2 FIQ Status Register
#define REG_FIQEN2          0x0148        //RISC L2 FIQ Enable Register
#define REG_FIQCL2          0x014C        //RISC L2 FIQ Clear Register

#define REG_IRQST3          0x0154        //RISC L3 IRQ Status Register
#define REG_IRQEN3          0x0158        //RISC L3 IRQ Enable Register
#define REG_IRQCL3          0x015C        //RISC L3 IRQ Clear Register
#define REG_FIQST3          0x0160        //RISC L3 FIQ Status Register
#define REG_FIQEN3          0x0164        //RISC L3 FIQ Enable Register
#define REG_FIQCL3          0x0168        //RISC L3 FIQ Clear Register

#define REG_IRQST4          0x0230        //RISC L4 IRQ Status Register
#define REG_IRQEN4          0x0234        //RISC L4 IRQ Enable Register
#define REG_IRQCL4          0x0238        //RISC L4 IRQ Clear Register
#define REG_FIQST4          0x023C        //RISC L4 FIQ Status Register
#define REG_FIQEN4          0x0240        //RISC L4 FIQ Enable Register
#define REG_FIQCL4          0x0244        //RISC L4 FIQ Clear Register


//----------------------------------------------------------------------------
//ckgen
#define REG_RW_PAD_CFG_S               0x0C0
#ifdef CONFIG_ARCH_MT3360
#define REG_RW_GPIO_EN_S               0x74
#define REG_RW_GPIO_OUT_S              0xE0
#define REG_RW_GPIO_IN_S               0x100
#else
#define REG_RW_GPIO_EN_S               0x180
#define REG_RW_GPIO_OUT_S              0x1A0
#define REG_RW_GPIO_IN_S               0x1C0
#endif

#define REG_RW_PAD_FECTL_S             0x280
#define REG_RW_PAD_PWMCTL_S            0x288

// pdwnc
#define REG_RW_GPIO_WAKEN             0x080
#define REG_RW_GPIO_PDSTAT            0x088
#define REG_RW_GPIO_PDSTCLR           0x08C
#define REG_RW_PDIO                   0x0C4
#define REG_RW_GPIOIN                 0x0D0
#define REG_RW_GPIOEN                 0x0D4
#define REG_RW_GPIOOUT                0x0D8
#define REG_RW_PAD_PINMUX1            0x0F4
#define REG_RW_PAD_PINMUX2            0x0F8
#define REG_RW_PAD_PINMUX3            0x0FC
#define REG_RW_INTSTA				  0x140                     //PDWNC INTERRUPT STATUS REGISTER
#define REG_RW_INTEN				  0x144                     //PDWNC INTERRUPT ENABLE REGISTER
#define REG_RW_INTCLR				  0x148                     //PDWNC INTERRUPT CLEAR REGISTER

// Timer settings
#define REG_RW_TIMER0_LIM_OFFSET    0x60
#define REG_RW_TIMER0_VALUE         0x64
#define REG_RW_TIMER_CTRL_OFFSET    0x78

#define VAL_T0_AUTOLOAD     0x2
#define VAL_T0_ENABLE       0x1

//RS232 settings
#define REG_RW_UART0_DATA_OFFSET        0x0
#define REG_RW_UART0_STATUS             0x4

#define VAL_UART0_WRITE_OFFSET          0x2


//RS232 settings
#define REG_RW_HSMPHE       0x01B4        //Hardware Semaphore Register
#define HSMPHE_UART1        (1U << 0)     //Hardware Semaphore 0
#define HSMPHE_DC_SMPHE     (1U << 1)     //Hardware Semaphore 1 (for Dual Communication Semaphore)
#define HSMPHE_NAND         (1U << 2)     // NAND semaphore
#define HSMPHE_LZHS         (1U << 3)     // LZHS semaphore
#define HSMPHE_SINFO4       (1U << 4)     // SINFO4_REG semaphore
#define HSMPHE_PNG          (1U << 5)     // PNG semaphore
#define HSMPHE_PWR_UP_CFG   (1U << 6)     // POWER UP REG semaphore

// General Purpose Register
#define REG_RW_GPRB0        0x00E0        //RISC Byte General Purpose Register 0
                                          // boot loader version
#define REG_RW_GPRB1        0x00E4        //RISC Byte General Purpose Register 1
                                          // ne upg version
#define REG_RW_GPRB2        0x00E8        //RISC Byte General Purpose Register 2
#define REG_RW_GPRB3        0x00EC        //RISC Byte General Purpose Register 3
#define REG_RW_GPRB4        0x00F0        //RISC Byte General Purpose Register 4
#define REG_RW_GPRB5        0x00F4        //RISC Byte General Purpose Register 5
#define REG_RW_GPRB6        0x00F8        //RISC Byte General Purpose Register 6
#define REG_RW_GPRB7        0x00FC        //RISC Byte General Purpose Register 7
#define REG_RW_GPRDW0       0x0100        //RISC Double Word General Purpose Register 0
#define REG_RW_GPRDW1       0x0104        //RISC Double Word General Purpose Register 1
#define REG_RW_GPRDW2       0x0120        //RISC Double Word General Purpose Register 2
#define REG_RW_GPRDW3       0x0124        //RISC Double Word General Purpose Register 3
#define REG_RW_GPRDW4       0x0110        //RISC Double Word General Purpose Register 4
#define REG_RW_GPRDW5       0x0114        //RISC Double Word General Purpose Register 5
#define REG_RW_GPRDW6       0x0118        //RISC Double Word General Purpose Register 6
#define REG_RW_GPRDW7       0x011C        //RISC Double Word General Purpose Register 7

// ARM2
#define REG_RW_SINFOE_REG   0x0238        //Dual Core Share Info Register E     : DUALCORE_MEM_BASE
#define REG_RW_SINFOF_REG   0x023C        //Dual Core Share Info Register F     ; DUALCORE_MEM_SIZE

#if (CONFIG_CHIP_VER_CURR >= CONFIG_CHIP_VER_MT8550)
#define REG_RW_SINFO4_REG   REG_RW_GPRDW7   //Dual Core Share Info Register 4
#else
#define REG_RW_SINFO4_REG   0x0210        //Dual Core Share Info Register 4
#endif
  #define FB_MEDIA_PRESENT    (1U << 0)     //[Fastboot]ARM2->ARM1 Detect Media Present
  #define FB_ARM2_ALIVE       (1U << 1)     //[Fastboot]ARM2->ARM1 Fastboot Function is Alive
  #define FB_ARM1_SATA_INIT   (1U << 2)     //[Fastboot]ARM1->ARM2 SATA Driver Init
  #define FB_ARM1_LIRC_INIT   (1U << 3)     //[Fastboot]ARM1->ARM2 LIRC Driver Init
  #define FB_ARM1_EJECT_KEY   (1U << 4)     //[Fastboot]ARM1->ARM2 Eject Key is Pressed

#endif /* __ASM_ARCH_MT8530_H */

