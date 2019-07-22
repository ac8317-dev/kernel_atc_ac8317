#ifndef __ASM_ARCH_AC83XX_BASIC_H
#define __ASM_ARCH_AC83XX_BASIC_H
#include <mach/chip_ver.h>
#include <mach/irqs.h>

/*
 * I/O mapping
 */


#define IO_PHYS             0xf0000000
#define IO_OFFSET           0xf0000000

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
#define AC83XX_CACHE_OPERATION_DEBUG 0

#define SVOIF_IO_VIRT      0xFE001000

#define AC83XX_SRAM_PHYS         0x40000000
#define AC83XX_SRAM_VIRT         0xFF000000
#define AC83XX_SRAM_SIZE         0x4000


#define AC83XX_SDRAM_PA      0x00008000
#define AC83XX_PA_UART      (IO_PHYS + 0x0000C000)     //0x7000c000
#define AC83XX_VA_UART      (IO_VIRT + 0x0000C000)     //0xe000c000
#define AC83XX_UART_SIZE    0x40

/*---------------------------------------------*/

/*
 *  AC83XX HAL
 */

/* Constant definitions */
#define MEMORY_ALIGNMENT            8

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
#define BIM_PHYS                    (IO_PHYS + 0x00008000)

// IRQ registers
//#define REG_RW_IRQEN_OFFSET     0x34         //IRQ enable register offset


//#define REG_WO_IRQCL_OFFSET     0x38         //IRQ clear register offset


//#define REG_RW_IRQST_OFFSET     0x3C         //IRQ status register
//----------------------------------------------------------------------------
#if 0
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
#endif

//----------------------------------------------------------------------------
//ckgen
#define REG_RW_PAD_CFG_S               0x0C0
#define REG_RW_GPIO_EN_S               0x180
#define REG_RW_GPIO_OUT_S              0x1A0
#define REG_RW_GPIO_IN_S               0x1C0

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

#define REG_RW_SINFO4_REG   REG_RW_GPRDW7   //Dual Core Share Info Register 4
  #define FB_MEDIA_PRESENT    (1U << 0)     //[Fastboot]ARM2->ARM1 Detect Media Present
  #define FB_ARM2_ALIVE       (1U << 1)     //[Fastboot]ARM2->ARM1 Fastboot Function is Alive
  #define FB_ARM1_SATA_INIT   (1U << 2)     //[Fastboot]ARM1->ARM2 SATA Driver Init
  #define FB_ARM1_LIRC_INIT   (1U << 3)     //[Fastboot]ARM1->ARM2 LIRC Driver Init
  #define FB_ARM1_EJECT_KEY   (1U << 4)     //[Fastboot]ARM1->ARM2 Eject Key is Pressed

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_AC83XX)
#define REG_RW_VERSION                  0x0000       // AC83XX chip version code
#define REG_RW_RISC_CFG0                0x0004       // define risc and axi clock frequency
#define REG_RW_DVD_REG1                 0x0008       // define DVD clock register
#define REG_RW_AP_REG0                  0x000c       // clock select of AP
  #define CLK_REG0_RFI_SEL_MASK              (3U << 27)  
  #define CLK_REG0_RFI_SEL_OFFSET            27          
  #define CLK_REG0_RFI_SEL_27M               0x00
  #define CLK_REG0_RFI_SEL_SYSPLL_D20        0x01    
  #define CLK_REG0_RFI_SEL_APLL_26M          0x02    
  #define CLK_REG0_RFI_SEL_USBPLL_D15        0x03 

  #define CLK_REG0_TP_F32K_SEL_MASK              (1U << 15)  
  #define CLK_REG0_TP_F32K_SEL_OFFSET            15          
  #define CLK_REG0_TP_F32K_SEL_SYSPLL_D1500      0x00
  #define CLK_REG0_TP_F32K_SEL_CLK_RTC           0x01  

  #define CLK_REG0_TP_SEL_MASK              (3U << 13)  
  #define CLK_REG0_TP_SEL_OFFSET            13          
  #define CLK_REG0_TP_SEL_SYSPLL_D12        0x00
  #define CLK_REG0_TP_SEL_CLK27M            0x01
  #define CLK_REG0_TP_SEL_MPHON_IN          0x02

  #define CLK_REG0_DEMUX_SEL_MASK              (7U << 6)  
  #define CLK_REG0_DEMUX_SEL_OFFSET            6          
  #define CLK_REG0_DEMUX_SEL_CLK27M        0x00
  #define CLK_REG0_DEMUX_SEL_APLL2_D2            0x01
  #define CLK_REG0_DEMUX_SEL_ARMPLL_D4          0x02
  #define CLK_REG0_DEMUX_SEL_SYSPLL_D6          0x03
  #define CLK_REG0_DEMUX_SEL_APLL1_D2          0x04
  #define CLK_REG0_DEMUX_SEL_SYSPLL_D3          0x05
  #define CLK_REG0_DEMUX_SEL_SYSPLL_D4          0x06
  #define CLK_REG0_DEMUX_SEL_DMPLL_D2          0x07
  
  #define CLK_REG0_DSP_SEL_MASK              (7U << 0)  
  #define CLK_REG0_DSP_SEL_OFFSET            0          
  #define CLK_REG0_DSP_SEL_CLK27M        0x00
  #define CLK_REG0_DSP_SEL_SYSPLL_D3            0x01
  #define CLK_REG0_DSP_SEL_ARMPLL_D2          0x02
  #define CLK_REG0_DSP_SEL_APLL2_D2          0x03
  #define CLK_REG0_DSP_SEL_USBPLL_D2          0x04
  #define CLK_REG0_DSP_SEL_CLK_APLL2          0x05
  #define CLK_REG0_DSP_SEL_CLK_DMPLL          0x06
  #define CLK_REG0_DSP_SEL_SYSPLL_D2          0x07

#define REG_RW_AP_REG1                  0x0010       //
  #define CLK_REG1_OSD_SEL_MASK              (7U << 28)  
  #define CLK_REG1_OSD_SEL_OFFSET            28          
  #define CLK_REG1_OSD_SEL_CLK27M        0x00
  #define CLK_REG1_OSD_SEL_SYSPLL_D10            0x01
  #define CLK_REG1_OSD_SEL_SYSPLL_D9          0x02
  #define CLK_REG1_OSD_SEL_SYSPLL_D8          0x03
  #define CLK_REG1_OSD_SEL_SYSPLL_D6          0x04
  #define CLK_REG1_OSD_SEL_APLL2_D2          0x05
  #define CLK_REG1_OSD_SEL_SYSPLL_D4          0x06
  #define CLK_REG1_OSD_SEL_SYSPLL_D3          0x07

  #define CLK_REG1_VDEC_MC_SEL_MASK              (7U << 24)  
  #define CLK_REG1_VDEC_MC_SEL_OFFSET            24          
  #define CLK_REG1_VDEC_MC_SEL_CLK27M        0x00
  #define CLK_REG1_VDEC_MC_SEL_CLK_APLL1            0x01
  #define CLK_REG1_VDEC_MC_SEL_SYSPLL_D2          0x02
  #define CLK_REG1_VDEC_MC_SEL_SYSPLL_D3          0x03
  #define CLK_REG1_VDEC_MC_SEL_DMPLL_D2          0x04
  #define CLK_REG1_VDEC_MC_SEL_USBPLL_D2          0x05
  #define CLK_REG1_VDEC_MC_SEL_CLK_APLL2          0x06
  #define CLK_REG1_VDEC_MC_SEL_APLL1_D2          0x07

  #define CLK_REG1_VDEC_SYS_SEL_MASK              (7U << 18)  
  #define CLK_REG1_VDEC_SYS_SEL_OFFSET            18          
  #define CLK_REG1_VDEC_SYS_SEL_CLK27M        0x00
  #define CLK_REG1_VDEC_SYS_SEL_SYSPLL_D3            0x01
  #define CLK_REG1_VDEC_SYS_SEL_DMPLL_D2          0x02
  #define CLK_REG1_VDEC_SYS_SEL_USBPLL_D2          0x03
  #define CLK_REG1_VDEC_SYS_SEL_CLK_APLL1          0x04
  #define CLK_REG1_VDEC_SYS_SEL_APLL2_D2          0x05
  #define CLK_REG1_VDEC_SYS_SEL_ARMPLL_D4          0x06
  #define CLK_REG1_VDEC_SYS_SEL_APLL1_D2          0x07

  #define CLK_REG1_JPEG_SEL_MASK              (7U << 15)  
  #define CLK_REG1_JPEG_SEL_OFFSET            15          
  #define CLK_REG1_JPEG_SEL_CLK27M        0x00
  #define CLK_REG1_JPEG_SEL_CLK_APLL1            0x01
  #define CLK_REG1_JPEG_SEL_APLL2_D2          0x02
  #define CLK_REG1_JPEG_SEL_SYSPLL_D6          0x03
  #define CLK_REG1_JPEG_SEL_SYSPLL_D3          0x04
  #define CLK_REG1_JPEG_SEL_SYSPLL_D2          0x05
  #define CLK_REG1_JPEG_SEL_USBPLL_D2          0x06
  #define CLK_REG1_JPEG_SEL_CLK_APLL2          0x07

  #define CLK_REG1_RSZ_SEL_MASK              (7U << 12)  
  #define CLK_REG1_RSZ_SEL_OFFSET            12          
  #define CLK_REG1_RSZ_SEL_CLK27M        0x00
  #define CLK_REG1_RSZ_SEL_CLK_APLL1            0x01
  #define CLK_REG1_RSZ_SEL_APLL2_D2          0x02
  #define CLK_REG1_RSZ_SEL_SYSPLL_D6          0x03
  #define CLK_REG1_RSZ_SEL_SYSPLL_D3          0x04
  #define CLK_REG1_RSZ_SEL_SYSPLL_D2          0x05
  #define CLK_REG1_RSZ_SEL_USBPLL_D2          0x06
  #define CLK_REG1_RSZ_SEL_CLK_APLL2          0x07

  #define CLK_REG1_FLASH_SEL_MASK              (7U << 6)  
  #define CLK_REG1_FLASH_SEL_OFFSET            6          
  #define CLK_REG1_FLASH_SEL_CLK27M        0x00
  #define CLK_REG1_FLASH_SEL_CLK27M_D2            0x01
  #define CLK_REG1_FLASH_SEL_SYSPLL_D9          0x02
  #define CLK_REG1_FLASH_SEL_SYSPLL_D12          0x03
  #define CLK_REG1_FLASH_SEL_SYSPLL_D16          0x04
  #define CLK_REG1_FLASH_SEL_SYSPLL_D18          0x05

  #define CLK_REG1_BCLK_SEL_MASK              (7U << 0)  
  #define CLK_REG1_BCLK_SEL_OFFSET            0          
  #define CLK_REG1_BCLK_SEL_CLK27M        0x00
  #define CLK_REG1_BCLK_SEL_SYSPLL_D2            0x01
  #define CLK_REG1_BCLK_SEL_ARMPLL2_D2          0x02
  #define CLK_REG1_BCLK_SEL_SYSPLL_D6          0x03
  #define CLK_REG1_BCLK_SEL_CLK_APLL1          0x04
  #define CLK_REG1_BCLK_SEL_ARMPLL_D4          0x05
  #define CLK_REG1_BCLK_SEL_CLK_APLL2          0x06
  #define CLK_REG1_BCLK_SEL_SYSPLL_D3          0x07
  
#define REG_RW_AP_REG2                  0x0014       //
  #define CLK_REG2_AUD_SEL_MASK              (7U << 28)  
  #define CLK_REG2_AUD_SEL_OFFSET            28          
  #define CLK_REG2_AUD_SEL_CLK27M        0x00
  #define CLK_REG2_AUD_SEL_ACLK_K4            0x01
  #define CLK_REG2_AUD_SEL_ACLK_IN          0x02
  #define CLK_REG2_AUD_SEL_I2S_OUT2_MCLK          0x03
  
  #define CLK_REG2_G3D_SEL_MASK              (7U << 24)  
  #define CLK_REG2_G3D_SEL_OFFSET            24          
  #define CLK_REG2_G3D_SEL_CLK27M        0x00
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_D3            0x01
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_D4          0x02
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_D6          0x03
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_D9          0x04
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_12          0x05
  #define CLK_REG2_G3D_SEL_AD_TTL_CK_18          0x06
  #define CLK_REG2_G3D_SEL_MHPON_IN          0x07

  #define CLK_REG2_FPD_SEL_MASK              (7U << 21)  
  #define CLK_REG2_FPD_SEL_OFFSET            21          
  #define CLK_REG2_FPD_SEL_CLK27M        0x00
  #define CLK_REG2_FPD_SEL_CLK_APLL2            0x01
  #define CLK_REG2_FPD_SEL_CLK_APLL1          0x02
  #define CLK_REG2_FPD_SEL_USBPLL_D2          0x03
  #define CLK_REG2_FPD_SEL_SYSPLL_D3          0x04
  #define CLK_REG2_FPD_SEL_ARMPLL_D4          0x05
  #define CLK_REG2_FPD_SEL_DMPLL_D2          0x06
  #define CLK_REG2_FPD_SEL_SYSPLL_D4          0x07

  #define CLK_REG2_SD21_SEL_MASK              (7U << 18)  
  #define CLK_REG2_SD21_SEL_OFFSET            18          
  #define CLK_REG2_SD21_SEL_CLK27M        0x00
  #define CLK_REG2_SD21_SEL_SYSPLL_D12            0x01
  #define CLK_REG2_SD21_SEL_SYSPLL_D16          0x02
  #define CLK_REG2_SD21_SEL_SYSPLL_D18          0x03
  #define CLK_REG2_SD21_SEL_DMPLL_D8          0x04
  #define CLK_REG2_SD21_SEL_USBPLL_D10          0x05
  #define CLK_REG2_SD21_SEL_SYSPLL_D36          0x06
  #define CLK_REG2_SD21_SEL_USBPLL_D20          0x07

  #define CLK_REG2_SD20_SEL_MASK              (7U << 15)  
  #define CLK_REG2_SD20_SEL_OFFSET            15          
  #define CLK_REG2_SD20_SEL_CLK27M        0x00
  #define CLK_REG2_SD20_SEL_DMPLL_D8            0x01
  #define CLK_REG2_SD20_SEL_USBPLL_D10          0x02
  #define CLK_REG2_SD20_SEL_SYSPLL_D12          0x03
  #define CLK_REG2_SD20_SEL_SYSPLL_D16          0x04
  #define CLK_REG2_SD20_SEL_ARMPLL_D16          0x05

  #define CLK_REG2_SD11_SEL_MASK              (7U << 12)  
  #define CLK_REG2_SD11_SEL_OFFSET            12          
  #define CLK_REG2_SD11_SEL_CLK27M        0x00
  #define CLK_REG2_SD11_SEL_SYSPLL_D12            0x01
  #define CLK_REG2_SD11_SEL_SYSPLL_D16          0x02
  #define CLK_REG2_SD11_SEL_SYSPLL_D18          0x03
  #define CLK_REG2_SD11_SEL_DMPLL_D8          0x04
  #define CLK_REG2_SD11_SEL_USBPLL_D10          0x05
  #define CLK_REG2_SD11_SEL_SYSPLL_D36          0x06
  #define CLK_REG2_SD11_SEL_USBPLL_D20          0x07

  #define CLK_REG2_SD10_SEL_MASK              (7U << 9)  
  #define CLK_REG2_SD10_SEL_OFFSET            9          
  #define CLK_REG2_SD10_SEL_CLK27M        0x00
  #define CLK_REG2_SD10_SEL_DMPLL_D8            0x01
  #define CLK_REG2_SD10_SEL_USBPLL_D10          0x02
  #define CLK_REG2_SD10_SEL_SYSPLL_D12          0x03
  #define CLK_REG2_SD10_SEL_SYSPLL_D16          0x04
  #define CLK_REG2_SD10_SEL_ARMPLL_D16          0x05

  #define CLK_REG2_SD01_SEL_MASK              (7U << 6)  
  #define CLK_REG2_SD01_SEL_OFFSET            6          
  #define CLK_REG2_SD01_SEL_CLK27M        0x00
  #define CLK_REG2_SD01_SEL_SYSPLL_D12            0x01
  #define CLK_REG2_SD01_SEL_SYSPLL_D16          0x02
  #define CLK_REG2_SD01_SEL_SYSPLL_D18          0x03
  #define CLK_REG2_SD01_SEL_DMPLL_D8          0x04
  #define CLK_REG2_SD01_SEL_USBPLL_D10          0x05
  #define CLK_REG2_SD01_SEL_SYSPLL_D36          0x06
  #define CLK_REG2_SD01_SEL_USBPLL_D20          0x07

  #define CLK_REG2_SD00_SEL_MASK              (7U << 3)  
  #define CLK_REG2_SD00_SEL_OFFSET            3          
  #define CLK_REG2_SD00_SEL_CLK27M        0x00
  #define CLK_REG2_SD00_SEL_DMPLL_D8            0x01
  #define CLK_REG2_SD00_SEL_USBPLL_D10          0x02
  #define CLK_REG2_SD00_SEL_SYSPLL_D12          0x03
  #define CLK_REG2_SD00_SEL_SYSPLL_D16          0x04
  #define CLK_REG2_SD00_SEL_ARMPLL_D16         0x05

  #define CLK_REG2_GRAPH_SEL_MASK              (7U << 0)  
  #define CLK_REG2_GRAPH_SEL_OFFSET            0          
  #define CLK_REG2_GRAPH_SEL_CLK27M        0x00
  #define CLK_REG2_GRAPH_SEL_CLK_APLL1            0x01
  #define CLK_REG2_GRAPH_SEL_SYSPLL_D2          0x02
  #define CLK_REG2_GRAPH_SEL_SYSPLL_D3          0x03
  #define CLK_REG2_GRAPH_SEL_USBPLL_D2          0x04
  #define CLK_REG2_GRAPH_SEL_CLK_APLL2          0x05
  
#define REG_RW_AP_REG3                  0x0018
  #define CLK_REG3_APLL1_2_SEL_MASK              (1U << 31)  
  #define CLK_REG3_APLL1_2_SEL_OFFSET            31          
  #define CLK_REG3_APLL1_2_SEL_CLK27M        0x00
  #define CLK_REG3_APLL1_2_SEL_SYSPLL_D12            0x01

  #define CLK_REG3_RLE_SEL_MASK              (7U << 28)  
  #define CLK_REG3_RLE_SEL_OFFSET            28          
  #define CLK_REG3_RLE_SEL_CLK27M        0x00
  #define CLK_REG3_RLE_SEL_CLK_APLL1            0x01
  #define CLK_REG3_RLE_SEL_SYSPLL_D4          0x02
  #define CLK_REG3_RLE_SEL_SYSPLL_D6          0x03
  #define CLK_REG3_RLE_SEL_SYSPLL_D3          0x04
  #define CLK_REG3_RLE_SEL_SYSPLL_D2          0x05
  #define CLK_REG3_RLE_SEL_USBPLL_D2          0x06
  #define CLK_REG3_RLE_SEL_CLK_APLL2          0x07

  #define CLK_REG3_NF_SEL_MASK              (7U << 24)  
  #define CLK_REG3_NF_SEL_OFFSET            24          
  #define CLK_REG3_NF_SEL_CLK27M        0x00
  #define CLK_REG3_NF_SEL_APLL2_D2            0x01
  #define CLK_REG3_NF_SEL_APLL1_D2          0x02
  #define CLK_REG3_NF_SEL_SYSPLL_D4          0x03
  #define CLK_REG3_NF_SEL_ARMPLL_D4          0x04
  #define CLK_REG3_NF_SEL_SYSPLL_D6          0x05

  #define CLK_REG3_BT_MIC_AUD_SEL_MASK              (3U << 18)  
  #define CLK_REG3_BT_MIC_AUD_SEL_OFFSET            18          
  #define CLK_REG3_BT_MIC_AUD_SEL_CLK27M        0x00
  #define CLK_REG3_BT_MIC_AUD_SEL_ACK_K9            0x01
  #define CLK_REG3_BT_MIC_AUD_SEL_BT_MCLK_IN          0x02
  #define CLK_REG3_BT_MIC_AUD_SEL_MPHON_IN          0x03

  #define CLK_REG3_ARM_AUD_SEL_MASK              (3U << 12)  
  #define CLK_REG3_ARM_AUD_SEL_OFFSET            12          
  #define CLK_REG3_ARM_AUD_SEL_CLK27M        0x00
  #define CLK_REG3_ARM_AUD_SEL_ACK_K8            0x01
  #define CLK_REG3_ARM_AUD_SEL_ACLK_IN          0x02
  #define CLK_REG3_ARM_AUD_SEL_MPHON_IN          0x03

  #define CLK_REG3_MPHON_SEL_MASK              (3U << 6)  
  #define CLK_REG3_MPHON_SEL_OFFSET            6          
  #define CLK_REG3_MPHON_SEL_CLK27M        0x00
  #define CLK_REG3_MPHON_SEL_ACK_K6            0x01
  #define CLK_REG3_MPHON_SEL_MPHONE_IN          0x02
  #define CLK_REG3_MPHON_SEL_SPMCLK_IN          0x03

  #define CLK_REG3_CPU2_SEL_MASK              (3U << 4)  
  #define CLK_REG3_CPU2_SEL_OFFSET            4          
  #define CLK_REG3_CPU2_SEL_CLK27M        0x00
  #define CLK_REG3_CPU2_SEL_CLK_ARMPLL2            0x01
  #define CLK_REG3_CPU2_SEL_SYSPLL_D2          0x02
  #define CLK_REG3_CPU2_SEL_ARMPLL_D2          0x03

  #define CLK_REG3_CPU1_SEL_MASK              (3U << 2)  
  #define CLK_REG3_CPU1_SEL_OFFSET            2          
  #define CLK_REG3_CPU1_SEL_CLK27M        0x00
  #define CLK_REG3_CPU1_SEL_CLK_ARMPLL            0x01
  #define CLK_REG3_CPU1_SEL_CLK_SYSPLL          0x02
  #define CLK_REG3_CPU1_SEL_CLK_DMPLL          0x03

  #define CLK_REG3_AUD2_SEL_MASK              (3U <<0)  
  #define CLK_REG3_AUD2_SEL_OFFSET            0          
  #define CLK_REG3_AUD2_SEL_CLK27M        0x00
  #define CLK_REG3_AUD2_SEL_ACLK_K4            0x01
  #define CLK_REG3_AUD2_SEL_ACK_IN          0x02
  #define CLK_REG3_AUD2_SEL_I2S_OUT2_MCLK          0x03
  
#define REG_RW_AP_REG4                  0x001c 
#define REG_RW_AP_REG5                  0x0020   
#define REG_RW_AP_REG6                  0x0024
  #define CLK_REG6_APLL2_K14_SEL_MASK              (1U << 23)  
  #define CLK_REG6_APLL2_K14_SEL_OFFSET            23          
  #define CLK_REG6_APLL2_K14_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K14_SEL_CLK_APLL2            0x01

  #define CLK_REG6_PLL_TEST_SEL_MASK              (1U << 22)  
  #define CLK_REG6_PLL_TEST_SEL_OFFSET            22          
  #define CLK_REG6_PLL_TEST_SEL_NORMAL_CLOCK        0x00
  #define CLK_REG6_PLL_TEST_SEL_TEST_CLOCK            0x01

  #define CLK_REG6_APLL2_K10_SEL_MASK              (1U << 21)  
  #define CLK_REG6_APLL2_K10_SEL_OFFSET            21          
  #define CLK_REG6_APLL2_K10_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K10_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K9_SEL_MASK              (1U << 20)  
  #define CLK_REG6_APLL2_K9_SEL_OFFSET            20          
  #define CLK_REG6_APLL2_K9_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K9_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K8_SEL_MASK              (1U << 19)  
  #define CLK_REG6_APLL2_K8_SEL_OFFSET            19          
  #define CLK_REG6_APLL2_K8_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K8_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K7_SEL_MASK              (1U << 18)  
  #define CLK_REG6_APLL2_K7_SEL_OFFSET            18          
  #define CLK_REG6_APLL2_K7_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K7_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K6_SEL_MASK              (1U << 17)  
  #define CLK_REG6_APLL2_K6_SEL_OFFSET            17          
  #define CLK_REG6_APLL2_K6_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K6_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K5_SEL_MASK              (1U << 16)  
  #define CLK_REG6_APLL2_K5_SEL_OFFSET            16          
  #define CLK_REG6_APLL2_K5_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K5_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K4_SEL_MASK              (1U << 15)  
  #define CLK_REG6_APLL2_K4_SEL_OFFSET            15          
  #define CLK_REG6_APLL2_K4_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K4_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K3_SEL_MASK              (1U << 14)  
  #define CLK_REG6_APLL2_K3_SEL_OFFSET            14          
  #define CLK_REG6_APLL2_K3_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K3_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K2_SEL_MASK              (1U << 13)  
  #define CLK_REG6_APLL2_K2_SEL_OFFSET            13          
  #define CLK_REG6_APLL2_K2_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K2_SEL_CLK_APLL2            0x01

  #define CLK_REG6_APLL2_K1_SEL_MASK              (1U << 12)  
  #define CLK_REG6_APLL2_K1_SEL_OFFSET            12          
  #define CLK_REG6_APLL2_K1_SEL_CLK_APLL1        0x00
  #define CLK_REG6_APLL2_K1_SEL_CLK_APLL2            0x01
  
#define REG_RW_AP_REG7                  0x0028

  #define CLK_REG7_APLL2_K12_SEL_MASK              (1U << 16)  
  #define CLK_REG7_APLL2_K12_SEL_OFFSET            16          
  #define CLK_REG7_APLL2_K12_SEL_CLK_APLL1        0x00
  #define CLK_REG7_APLL2_K12_SEL_CLK_APLL2            0x01

  #define CLK_REG7_APLL2_K11_SEL_MASK              (1U << 7)  
  #define CLK_REG7_APLL2_K11_SEL_OFFSET            7          
  #define CLK_REG7_APLL2_K11_SEL_CLK_APLL1        0x00
  #define CLK_REG7_APLL2_K11_SEL_CLK_APLL2            0x01

#define REG_RW_AP_REG8                  0x002c
  #define CLK_REG8_AUD_MPH_SEL_MASK              (3U << 27)  
  #define CLK_REG8_AUD_MPH_SEL_OFFSET            27          
  #define CLK_REG8_AUD_MPH_SEL_CLK27M        0x00
  #define CLK_REG8_AUD_MPH_SEL_ACLK_K13            0x01
  #define CLK_REG8_AUD_MPH_SEL_ACK_IN          0x02
  #define CLK_REG8_AUD_MPH_SEL_SPMCLK_IN          0x03

  #define CLK_REG8_AUD_GPS_SEL_MASK              (3U << 25)  
  #define CLK_REG8_AUD_GPS_SEL_OFFSET            25          
  #define CLK_REG8_AUD_GPS_SEL_CLK27M        0x00
  #define CLK_REG8_AUD_GPS_SEL_ACLK_K12            0x01
  #define CLK_REG8_AUD_GPS_SEL_ACLK_IN          0x02
  #define CLK_REG8_AUD_GPS_SEL_MPHON_IN          0x03

  #define CLK_REG8_AUD_PWM_SEL_MASK              (3U << 23)  
  #define CLK_REG8_AUD_PWM_SEL_OFFSET            23          
  #define CLK_REG8_AUD_PWM_SEL_CLK27M        0x00
  #define CLK_REG8_AUD_PWM_SEL_ACLK_K12            0x01
  #define CLK_REG8_AUD_PWM_SEL_ACLK_K5          0x02
  #define CLK_REG8_AUD_PWM_SEL_MPHON_IN          0x03

  #define CLK_REG8_AUD_ADC_SEL_MASK              (3U << 21)  
  #define CLK_REG8_AUD_ADC_SEL_OFFSET            21          
  #define CLK_REG8_AUD_ADC_SEL_CLK27M        0x00
  #define CLK_REG8_AUD_ADC_SEL_ACLK_K11            0x01
  #define CLK_REG8_AUD_ADC_SEL_ACLK_K5          0x02
  #define CLK_REG8_AUD_ADC_SEL_MPHONE_IN          0x03

  #define CLK_REG8_CFG_A3_SEL_MASK              (1U << 20)  
  #define CLK_REG8_CFG_A3_SEL_OFFSET            20          
  #define CLK_REG8_CFG_A3_SEL_CLK_D1        0x00
  #define CLK_REG8_CFG_A3_SEL_CLK_D2            0x01

  #define CLK_REG8_APLL2_A3_SEL_MASK              (1U << 19)  
  #define CLK_REG8_APLL2_A3_SEL_OFFSET            19          
  #define CLK_REG8_APLL2_A3_SEL_CLK_APLL1        0x00
  #define CLK_REG8_APLL2_A3_SEL_CLK_APLL2            0x01

  #define CLK_REG8_CFG_A2_SEL_MASK              (1U << 18)  
  #define CLK_REG8_CFG_A2_SEL_OFFSET            18          
  #define CLK_REG8_CFG_A2_SEL_CLK_D1        0x00
  #define CLK_REG8_CFG_A2_SEL_CLK_D2            0x01

  #define CLK_REG8_CFG_A1_SEL_MASK              (1U << 17)  
  #define CLK_REG8_CFG_A1_SEL_OFFSET            17          
  #define CLK_REG8_CFG_A1_SEL_CLK_D1        0x00
  #define CLK_REG8_CFG_A1_SEL_CLK_D2            0x01

  #define CLK_REG8_APLL2_A1_SEL_MASK              (1U << 16)  
  #define CLK_REG8_APLL2_A1_SEL_OFFSET            16          
  #define CLK_REG8_APLL2_A1_SEL_CLK_APLL1        0x00
  #define CLK_REG8_APLL2_A1_SEL_CLK_APLL2            0x01

  #define CLK_REG8_APLL2_K13_SEL_MASK              (1U << 0)  
  #define CLK_REG8_APLL2_K13_SEL_OFFSET            0          
  #define CLK_REG8_APLL2_K13_SEL_CLK_APLL1        0x00
  #define CLK_REG8_APLL2_K13_SEL_CLK_APLL2            0x01
  
#define REG_RW_AP_REG9                  0x0030
  #define CLK_REG9_AUD_K5_TST_SEL_MASK              (1U << 30)  
  #define CLK_REG9_AUD_K5_TST_SEL_OFFSET            30          
  #define CLK_REG9_AUD_K5_TST_SEL_ACLK_K5        0x00
  #define CLK_REG9_AUD_K5_TST_SEL_MPHON_IN            0x01

  #define CLK_REG9_AUD_A3_TST_SEL_MASK              (1U << 29)  
  #define CLK_REG9_AUD_A3_TST_SEL_OFFSET            29          
  #define CLK_REG9_AUD_A3_TST_SEL_ACLK_A3        0x00
  #define CLK_REG9_AUD_A3_TST_SEL_MPHON_IN            0x01

  #define CLK_REG9_AUD_A2_TST_SEL_MASK              (1U << 28)  
  #define CLK_REG9_AUD_A2_TST_SEL_OFFSET            28          
  #define CLK_REG9_AUD_A2_TST_SEL_ACLK_A2        0x00
  #define CLK_REG9_AUD_A2_TST_SEL_MPHON_IN            0x01

  #define CLK_REG9_AUD_A1_TST_SEL_MASK              (1U << 27)  
  #define CLK_REG9_AUD_A1_TST_SEL_OFFSET            27          
  #define CLK_REG9_AUD_A1_TST_SEL_ACLK_A1        0x00
  #define CLK_REG9_AUD_A1_TST_SEL_MPHON_IN            0x01

  #define CLK_REG9_MLIN_TST_SEL_MASK              (1U << 26)  
  #define CLK_REG9_MLIN_TST_SEL_OFFSET            26          
  #define CLK_REG9_MLIN_TST_SEL_ACLK_K3        0x00
  #define CLK_REG9_MLIN_TST_SEL_MPHON_IN            0x01

  #define CLK_REG9_DA_APLL1CK_SEL_MASK              (7U << 21)  
  #define CLK_REG9_DA_APLL1CK_SEL_OFFSET            21          
  #define CLK_REG9_DA_APLL1CK_SEL_CLK27M        0x00
  #define CLK_REG9_DA_APLL1CK_SEL_ACK_IN            0x01
  #define CLK_REG9_DA_APLL1CK_SEL_I2S_OUT2_MCLK          0x02
  #define CLK_REG9_DA_APLL1CK_SEL_MPHON_IN          0x03
  #define CLK_REG9_DA_APLL1CK_SEL_SPMCLK_IN          0x04

  #define CLK_REG9_DA_APLLCK_SEL_MASK              (7U << 18)  
  #define CLK_REG9_DA_APLLCK_SEL_OFFSET            18          
  #define CLK_REG9_DA_APLLCK_SEL_CLK27M        0x00
  #define CLK_REG9_DA_APLLCK_SEL_ACK_IN            0x01
  #define CLK_REG9_DA_APLLCK_SEL_I2S_OUT2_MCLK          0x02
  #define CLK_REG9_DA_APLLCK_SEL_MPHON_IN          0x03
  #define CLK_REG9_DA_APLLCK_SEL_SPMCLK_IN          0x04

  #define CLK_REG9_MCLK_D2_SEL_MASK              (1U << 16)  
  #define CLK_REG9_MCLK_D2_SEL_OFFSET            16          
  #define CLK_REG9_MCLK_D2_SEL_CLK_MCLK        0x00
  #define CLK_REG9_MCLK_D2_SEL_CLK_MCLK_D2            0x01

  #define CLK_REG9_SPI_MTK_SEL_MASK              (7U << 12)  
  #define CLK_REG9_SPI_MTK_SEL_OFFSET            12          
  #define CLK_REG9_SPI_MTK_SEL_CLK27M        0x00
  #define CLK_REG9_SPI_MTK_SEL_APLL2_D2            0x01
  #define CLK_REG9_SPI_MTK_SEL_SYSPLL_D6          0x02
  #define CLK_REG9_SPI_MTK_SEL_SYSPLL_D9          0x03
  #define CLK_REG9_SPI_MTK_SEL_SYSPLL_D12          0x04
  #define CLK_REG9_SPI_MTK_SEL_SYSPLL_D18          0x05
  #define CLK_REG9_SPI_MTK_SEL_CLK27M_D2          0x06
  #define CLK_REG9_SPI_MTK_SEL_CLK27M_D4          0x07

  #define CLK_REG9_TVD_MBIST_SEL_MASK              (1U << 10)  
  #define CLK_REG9_TVD_MBIST_SEL_OFFSET            10          
  #define CLK_REG9_TVD_MBIST_SEL_CVBS_ADC        0x00
  #define CLK_REG9_TVD_MBIST_SEL_SYSPLL_D12            0x01

  #define CLK_REG9_SPI_MOTO_SEL_MASK              (3U << 8)  
  #define CLK_REG9_SPI_MOTO_SEL_OFFSET            8          
  #define CLK_REG9_SPI_MOTO_SEL_CLK27M        0x00
  #define CLK_REG9_SPI_MOTO_SEL_SYSPLL_D3            0x01
  #define CLK_REG9_SPI_MOTO_SEL_SYSPLL_D6          0x02
  #define CLK_REG9_SPI_MOTO_SEL_APLL2_D2          0x03

  #define CLK_REG9_TS_OUT_SEL_MASK              (7U << 5)  
  #define CLK_REG9_TS_OUT_SEL_OFFSET            5          
  #define CLK_REG9_TS_OUT_SEL_CLK27M        0x00
  #define CLK_REG9_TS_OUT_SEL_APLL2_D3            0x01
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D6          0x02
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D9          0x03
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D12          0x04
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D16          0x05
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D18          0x06
  #define CLK_REG9_TS_OUT_SEL_SYSPLL_D8          0x07

  #define CLK_REG9_TS1_SEL_MASK              (3U <<2)  
  #define CLK_REG9_TS1_SEL_OFFSET            2          
  #define CLK_REG9_TS1_SEL_DEMUX        0x00
  #define CLK_REG9_TS1_SEL_EXT_TS1            0x01
  #define CLK_REG9_TS1_SEL_TS_OUT          0x03

  #define CLK_REG9_TS0_SEL_MASK              (3U <<0)  
  #define CLK_REG9_TS0_SEL_OFFSET            0          
  #define CLK_REG9_TS0_SEL_DEMUX        0x00
  #define CLK_REG9_TS0_SEL_EXT_TS0            0x01
  #define CLK_REG9_TS0_SEL_TS_OUT          0x03
  
#define REG_RW_AP_REG10                 0x0034
  #define CLK_REG10_PNG_SEL_MASK              (7U << 29)  
  #define CLK_REG10_PNG_SEL_OFFSET            29          
  #define CLK_REG10_PNG_SEL_CLK27M        0x00
  #define CLK_REG10_PNG_SEL_CLK_APLL1            0x01
  #define CLK_REG10_PNG_SEL_APLL2_D2          0x02
  #define CLK_REG10_PNG_SEL_SYSPLL_D6          0x03
  #define CLK_REG10_PNG_SEL_SYSPLL_D3          0x04
  #define CLK_REG10_PNG_SEL_CLK_ADPLL_324M          0x05
  #define CLK_REG10_PNG_SEL_USBPLL_D2          0x06
  #define CLK_REG10_PNG_SEL_CLK_APLL2          0x07

  #define CLK_REG10_PWM5_SEL_MASK              (7U << 24)  
  #define CLK_REG10_PWM5_SEL_OFFSET            24          
  #define CLK_REG10_PWM5_SEL_CLK27M        0x00
  #define CLK_REG10_PWM5_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM5_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM5_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM5_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM5_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM5_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM5_SEL_CLK_APLL1          0x07

  #define CLK_REG10_PWM4_SEL_MASK              (7U << 19)  
  #define CLK_REG10_PWM4_SEL_OFFSET            19          
  #define CLK_REG10_PWM4_SEL_CLK27M        0x00
  #define CLK_REG10_PWM4_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM4_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM4_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM4_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM4_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM4_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM4_SEL_CLK_APLL1          0x07

  #define CLK_REG10_PWM3_SEL_MASK              (7U << 16)  
  #define CLK_REG10_PWM3_SEL_OFFSET            16          
  #define CLK_REG10_PWM3_SEL_CLK27M        0x00
  #define CLK_REG10_PWM3_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM3_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM3_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM3_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM3_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM3_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM3_SEL_CLK_APLL1          0x07

  #define CLK_REG10_PWM2_SEL_MASK              (7U << 13)  
  #define CLK_REG10_PWM2_SEL_OFFSET            13          
  #define CLK_REG10_PWM2_SEL_CLK27M        0x00
  #define CLK_REG10_PWM2_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM2_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM2_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM2_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM2_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM2_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM2_SEL_CLK_APLL1          0x07

  #define CLK_REG10_PWM1_SEL_MASK              (7U << 9)  
  #define CLK_REG10_PWM1_SEL_OFFSET            9          
  #define CLK_REG10_PWM1_SEL_CLK27M        0x00
  #define CLK_REG10_PWM1_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM1_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM1_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM1_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM1_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM1_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM1_SEL_CLK_APLL1          0x07

  #define CLK_REG10_PWM0_SEL_MASK              (7U << 6)  
  #define CLK_REG10_PWM0_SEL_OFFSET            6          
  #define CLK_REG10_PWM0_SEL_CLK27M        0x00
  #define CLK_REG10_PWM0_SEL_SYSPLL_D3            0x01
  #define CLK_REG10_PWM0_SEL_SYSPLL_D6          0x02
  #define CLK_REG10_PWM0_SEL_CLK_APLL2          0x03
  #define CLK_REG10_PWM0_SEL_SYSPLL_D2          0x04
  #define CLK_REG10_PWM0_SEL_ARMPLL_D2          0x05
  #define CLK_REG10_PWM0_SEL_DMPLL_D2          0x06
  #define CLK_REG10_PWM0_SEL_CLK_APLL1          0x07

  #define CLK_REG10_APLL_26M_SEL_MASK              (1U << 5)  
  #define CLK_REG10_APLL_26M_SEL_OFFSET            5          
  #define CLK_REG10_APLL_26M_SEL_APLL_26M        0x00
  #define CLK_REG10_APLL_26M_SEL_MPHON_IN            0x01

  #define CLK_REG10_TWDS_SEL_MASK              (1U << 4)  
  #define CLK_REG10_TWDS_SEL_OFFSET            4          
  #define CLK_REG10_TWDS_SEL_TTL        0x00
  #define CLK_REG10_TWDS_SEL_TWDS_DPIX            0x01

  #define CLK_REG10_LVDS_SEL_MASK              (1U << 3)  
  #define CLK_REG10_LVDS_SEL_OFFSET            3          
  #define CLK_REG10_LVDS_SEL_TTL        0x00
  #define CLK_REG10_LVDS_SEL_LVDS_DPIX            0x01

  #define CLK_REG10_SM_CARD_SEL_MASK              (2U << 0)  
  #define CLK_REG10_SM_CARD_SEL_OFFSET            0          
  #define CLK_REG10_SM_CARD_SEL_CLK27M        0x00
  #define CLK_REG10_SM_CARD_SEL_USBPLL_D10            0x01
  #define CLK_REG10_SM_CARD_SEL_CLK27M_D2            0x02
  #define CLK_REG10_SM_CARD_SEL_SYSPLL_D12            0x03

#define REG_RW_GPIO_EN_CFG0             0x0074
#define REG_RW_GPIO_EN_CFG1             0x0078
#define REG_RW_GPIO_EN_CFG2             0x007c
#define REG_RW_GPIO_EN_CFG3             0x0080
#define REG_RW_GPIO_EN_CFG4             0x0084
#define REG_RW_GPIO_EN_CFG5             0x0088

#define REG_RW_MISC_CONTROL             0x0094       // top lever misc control 0

#define REG_RW_CLKGATE_CFG0             0x009c       // vdec clock gated
  #define CLK_PDN_VDEC_FULL                     (1U << 0)  //turn off VDEC FULL

#define REG_RW_CLKGATE_CFG1             0x00a0       // img_dramc gated
  #define CLK_PDN_GFX	                    (1U << 0)//
  #define CLK_PDN_RLE                     (1U << 1)//
  #define CLK_PDN_PNG	                    (1U << 2) //
  #define CLK_PDN_GIF	                    (1U << 3)         //
  #define CLK_PDN_IMG_RESZ                (1U << 4)//5  
  #define CLK_PDN_OSD_RESZ	              (1U << 5)//6  
  #define CLK_PDN_JPGDEC                  (1U << 6)//7  
  #define CLK_PDN_DEMUX	                  (1U << 7)//8  
  #define CLK_PDN_DEMUX_TS0	              (1U << 8)//9  
  #define CLK_PDN_DEMUX_TS1	              (1U << 9)//10 
  #define CLK_PDN_DEMUX_TSOUT	            (1U << 10)//11 
  #define CLK_PDN_DEMUX_27M	              (1U << 11)//12 
  #define CLK_PDN_GFX24BPP	              (1U << 12)//13 
  #define CLK_PDN_G3D	                    (1U << 16)//14

#define REG_RW_CLKGATE_CFG3             0x00a8       // audio peripher
  #define CLK_PDN_AUDIO_B00               (1U << 0)//15
  #define CLK_PDN_AUDIO_B01               (1U << 1)//16 
  #define CLK_PDN_AUDIO_B02	              (1U << 2)//17 
  #define CLK_PDN_AUDIO_B03	              (1U << 3)//18 
  #define CLK_PDN_AUDIO_B04	              (1U << 4)//19 
  #define CLK_PDN_AUDIO_B05	              (1U << 5)//20 
  #define CLK_PDN_AUDIO_B06               (1U << 6)//21 
  #define CLK_PDN_AUDIO_B07               (1U << 7)//22 
  #define CLK_PDN_AUDIO_B08               (1U << 8)//23 
  #define CLK_PDN_AUDIO_B09               (1U << 9)//24 
  #define CLK_PDN_AUDIO_B10               (1U << 10)//25 
  #define CLK_PDN_RFI_TOP                 (1U << 16)//26 
  #define CLK_PDN_MSDC_0                  (1U << 17)//27 
  #define CLK_PDN_MSDC_1                  (1U << 18)//28
  #define CLK_PDN_MSDC_2                  (1U << 19)//29
  #define CLK_PDN_SPI_MTK                 (1U << 20)//30
  #define CLK_PDN_SPI_MOTO1               (1U << 21)//31
  #define CLK_PDN_SPI_MOTO2               (1U << 22)//32
  #define CLK_PDN_PWM0                    (1U << 24)//33
  #define CLK_PDN_PWM1                    (1U << 25)//34
  #define CLK_PDN_PWM2                    (1U << 26)//35
  #define CLK_PDN_PWM3                    (1U << 27)//36
  #define CLK_PDN_PWM4                    (1U << 28)//37
  #define CLK_PDN_PWM5                    (1U << 29)//38
  #define CLK_PDN_TOUCHPANEL              (1U << 30)//39  
  #define CLK_PDN_SMARTCARD               (1U << 31)
  
#define REG_RW_CLKGATE_CFG4             0x00ac       // dvd clock gated
  #define CLK_PDN_MVDO                     (1U << 0)//41  CLKGATE_CFG4  DVD
  #define CLK_PDN_TVE                      (1U << 1)//42
  #define CLK_PDN_DGO                      (1U << 2)//43
  #define CLK_PDN_DACTST                   (1U << 3)
  #define CLK_PDN_DVD_OSD                  (1U << 4)
  #define CLK_PDN_GRA                      (1U << 5)//44
  #define CLK_PDN_BIM                      (1U << 6)//45
  #define CLK_PDN_TURBO32                  (1U << 7)//46
  #define CLK_PDN_VDEC                     (1U << 8)//47 
  #define CLK_PDN_PARSER                   (1U << 9)//48 
  #define CLK_PDN_RAMBUF                   (1U << 10)//49 
  #define CLK_PDN_PT110                    (1U << 11)//50
  #define CLK_PDN_RS232                    (1U << 12)//51
  #define CLK_PDN_CDDVD                    (1U << 13)//52
  #define CLK_PDN_AUDIO                    (1U << 14)//53
  #define CLK_PDN_SERVO_MISC               (1U << 15)//54
  
#define REG_RW_CLKGATE_CFG5             0x00b0       // lvds clock gated
  #define CLK_PDN_CLK_LVDS                 (1U << 0)
  
#define REG_RW_CLKGATE_CFG6             0x00b4       // vdout clock gated
  #define CLK_PDN_SCLER                   (1U << 0)//56 CLKGATE_CFG6 VDOUT
  #define CLK_PDN_TVD1                    (1U << 1)//57
  #define CLK_PDN_TVD2                    (1U << 2)//58
  #define CLK_PDN_OSD                     (1U << 3)//59
  #define CLK_PDN_OSD_R                   (1U << 4)//60
  #define CLK_PDN_FPD                     (1U << 5)//61
  #define CLK_PDN_FMT_VDO_F               (1U << 6)//62
  #define CLK_PDN_FMT_VDO_R               (1U << 7)//63
  #define CLK_PDN_WRITE_CHANEL            (1U << 8)//64
  #define CLK_PDN_FRAME_LOCK              (1U << 9)//65
    
#define REG_RW_SYNC_RESET_CFG0          0x00b8       // vdec full sync reset
  #define CLK_RESET_VDEC_FULL                     (1U << 0)  //turn off VDEC FULL
#define REG_RW_SYNC_RESET_CFG1          0x00bc       // image dramc sync reset
  #define CLK_RESET_GFX	                    (1U << 0)//
  #define CLK_RESET_RLE                     (1U << 1)//
  #define CLK_RESET_PNG	                    (1U << 2) //
  #define CLK_RESET_GIF	                    (1U << 3)         //
  #define CLK_RESET_IMG_RESZ                (1U << 4)//5  
  #define CLK_RESET_OSD_RESZ	              (1U << 5)//6  
  #define CLK_RESET_JPGDEC                  (1U << 6)//7  
  #define CLK_RESET_DEMUX	                  (1U << 7)//8  
  //#define CLK_RESET_DEMUX_TS0	              (1U << 8)//9  
  //#define CLK_RESET_DEMUX_TS1	              (1U << 9)//10 
  //#define CLK_RESET_DEMUX_TSOUT	            (1U << 10)//11 
  //#define CLK_RESET_DEMUX_27M	              (1U << 11)//12 
  #define CLK_RESET_GFX24BPP	              (1U << 12)//13 
  #define CLK_RESET_G3D	                    (1U << 16)//14
#define REG_RW_SYNC_RESET_CFG3          0x00c4       // audio prepher sync reset
  #define CLK_RESET_AUDIO_B00               (1U << 0)//15
  #define CLK_RESET_AUDIO_B01               (1U << 1)//16 
  #define CLK_RESET_AUDIO_B02	              (1U << 2)//17 
  #define CLK_RESET_AUDIO_B03	              (1U << 3)//18 
  #define CLK_RESET_AUDIO_B04	              (1U << 4)//19 
  #define CLK_RESET_AUDIO_B05	              (1U << 5)//20 
  #define CLK_RESET_AUDIO_B06               (1U << 6)//21 
  #define CLK_RESET_AUDIO_B07               (1U << 7)//22 
  #define CLK_RESET_AUDIO_B08               (1U << 8)//23 
  #define CLK_RESET_AUDIO_B09               (1U << 9)//24 
  #define CLK_RESET_AUDIO_B10               (1U << 10)//25 
  #define CLK_RESET_RFI_TOP                 (1U << 16)//26 
  #define CLK_RESET_MSDC_0                  (1U << 17)//27 
  #define CLK_RESET_MSDC_1                  (1U << 18)//28
  #define CLK_RESET_MSDC_2                  (1U << 19)//29
  #define CLK_RESET_SPI_MTK                 (1U << 20)//30
  #define CLK_RESET_SPI_MOTO1               (1U << 21)//31
  #define CLK_RESET_SPI_MOTO2               (1U << 22)//32
  #define CLK_RESET_PWM0                    (1U << 24)//33
  #define CLK_RESET_PWM1                    (1U << 25)//34
  #define CLK_RESET_PWM2                    (1U << 26)//35
  #define CLK_RESET_PWM3                    (1U << 27)//36
  #define CLK_RESET_PWM4                    (1U << 28)//37
  #define CLK_RESET_PWM5                    (1U << 29)//38
  #define CLK_RESET_TOUCHPANEL              (1U << 30)//39  
  #define CLK_RESET_SMARTCARD               (1U << 31)
#define REG_RW_SYNC_RESET_CFG4          0x00c8       // dvd sync reset
  #define CLK_RESET_MVDO                     (1U << 0)//41  CLKGATE_CFG4  DVD
  #define CLK_RESET_TVE                      (1U << 1)//42
  #define CLK_RESET_DGO                      (1U << 2)//43
  #define CLK_RESET_DACTST                   (1U << 3)
  #define CLK_RESET_DVD_OSD                  (1U << 4)
  #define CLK_RESET_GRA                      (1U << 5)//44
  #define CLK_RESET_BIM                      (1U << 6)//45
  #define CLK_RESET_TURBO32                  (1U << 7)//46
  #define CLK_RESET_VDEC                     (1U << 8)//47 
  #define CLK_RESET_PARSER                   (1U << 9)//48 
  #define CLK_RESET_RAMBUF                   (1U << 10)//49 
  #define CLK_RESET_PT110                    (1U << 11)//50
  #define CLK_RESET_RS232                    (1U << 12)//51
  #define CLK_RESET_CDDVD                    (1U << 13)//52
  #define CLK_RESET_AUDIO                    (1U << 14)//53
  #define CLK_RESET_SERVO_MISC               (1U << 15)//54
#define REG_RW_SYNC_RESET_CFG5          0x00cc       // lvds sync reset
  #define CLK_RESET_CLK_LVDS                 (1U << 0)
#define REG_RW_SYNC_RESET_CFG6          0x00d0       // vdout tvd sync reset
  #define CLK_RESET_SCLER                   (1U << 0)//56 CLKGATE_CFG6 VDOUT
  #define CLK_RESET_TVD1                    (1U << 1)//57
  //#define CLK_RESET_TVD2                    (1U << 2)//58
  #define CLK_RESET_OSD                     (1U << 3)//59
  #define CLK_RESET_OSD_R                   (1U << 4)//60
  #define CLK_RESET_FPD                     (1U << 5)//61
  #define CLK_RESET_FMT_VDO_F               (1U << 6)//62
  #define CLK_RESET_FMT_VDO_R               (1U << 7)//63
  #define CLK_RESET_WRITE_CHANEL            (1U << 8)//64
  #define CLK_RESET_FRAME_LOCK              (1U << 9)//65

#define REG_RW_GPIO_OUT_CFG0            0x00e0
#define REG_RW_GPIO_OUT_CFG1            0x00e4
#define REG_RW_GPIO_OUT_CFG2            0x00e8
#define REG_RW_GPIO_OUT_CFG3            0x00ec
#define REG_RW_GPIO_OUT_CFG4            0x00f0
#define REG_RW_GPIO_OUT_CFG5            0x00f4

#define REG_RW_GPIO_IN_CFG0             0x0100
#define REG_RW_GPIO_IN_CFG1             0x0104
#define REG_RW_GPIO_IN_CFG2             0x0108
#define REG_RW_GPIO_IN_CFG3             0x010c
#define REG_RW_GPIO_IN_CFG4             0x0110
#define REG_RW_GPIO_IN_CFG5             0x0114

#define REG_RW_TST_CLK_CFG              0x0188       // ap test clock select
#define REG_RW_TST_CLK_CFG1             0x018c       // 

#define REG_RW_TEST_BUS_CFG0            0x0194       //dvd test bus control
#define REG_RW_TEST_BUS_CFG1            0x0198       // audio vdout test bus control
#define REG_RW_TEST_BUS_CFG2            0x019c       // dramc test bus control
#define REG_RW_TEST_BUS_CFG3            0x01a0       //

#define REG_RW_ANA7_PLLGP_CFG1          0x0284

#define REG_RW_MISC_CONTROL2            0x0294       // top lever misc control2

#define REG_RW_ANA7_PLLGP_CFG1      0x0284
#define REG_RW_ANA7_PLLGP_CFG7      0x029C
  #define  ARMPLL_PWD_MASK         (1U << 0)
  #define  ARMPLL_PWD_DISABLE       0x1
#define REG_RW_ANA7_PLLGP_CFG8      0x02A0
  #define  ARM9PLL_PWD_MASK        (1U << 0)
  #define  ARM9PLL_PWD_DISABLE      0x1

#define REG_RW_ANA7_PLLGP_CFG9      0x02A4
#define REG_RW_ANA7_PLLGP_CFG14     0x02B8
#define REG_RW_ANA7_PLLGP_CFG25     0x02E8
  #define BIT_RG_APLL_PWD           (1 << 0)
  
#define REG_RW_ANA7_PLLGP_CFG10     0x02A8
#define REG_RW_ANA7_PLLGP_CFG15     0x02BC
  #define BIT_RG_APLL_RESERVE_MASK (0x3F << 18)
  #define BIT_RG_APLL_PCW_NCPO_CHG (1 << 26)
  #define BIT_RG_APLL_DDS_RSTB     (1 << 25)
  #define BIT_RG_APLL_DDS_PWDB     (1 << 24)
  #define BIT_RG_APLL_DDSEN        (1 << 17)
  #define BIT_RG_APLL_VODEN        (1 << 16)
  #define BIT_RG_APLL_AUTOK_LOAD   (1 << 13)
  #define BIT_RG_APLL_AUTOK_VCO    (1 << 12)
  
#define REG_RW_ANA7_PLLGP_CFG11     0x02AC
#define REG_RW_ANA7_PLLGP_CFG16     0x02C0
  
#define REG_RW_ANA7_PLLGP_CFG12     0x02B0
#define REG_RW_ANA7_PLLGP_CFG17     0x02C4
  #define BIT_RG_APLL_DDS_NCPO_EN     (1 << 19)
  #define BIT_RG_APLL_DDS_CLK_PH_INV  (1 << 17)
  #define BIT_RG_APLL_FIFO_START_MAN  (1 << 13)


  
#define REG_RW_ANA7_PLLGP_CFG13     0x02B4
#define REG_RW_ANA7_PLLGP_CFG18     0x02C8

#define REG_RW_ANA7_PLLGP_CFG19     0x02CC
  #define BIT_RG_PLL_RESERVE_MASK   (0xFF)

#define REG_RO_MONITOR_PLLGP_STATUS 0x02E4
  #define BIT_AD_RGS_APLL270_VCOCAL_FAIL   (1 << 15)
  #define BIT_AD_RGS_APLL270_VCOCAL_CPLT   (1 << 14)
  #define BIT_AD_RGS_APLL294_VCOCAL_FAIL   (1 << 7)
  #define BIT_AD_RGS_APLL294_VCOCAL_CPLT   (1 << 6)


#define REG_RW_MEMPLL0      (0x5AA90)

//============================================================================
// Constant definitions
//============================================================================
#define CKGEN_PLLGP_FREF  27000000
#define CKGEN_PLLGP_SYSPLL_FREQ  648000000

#define PLL_GET_FBSEL(v)  ((v >> 14)&0x3)
#define PLL_GET_CKCTRL(v)  ((v >> 12)&0x3)
#define PLL_GET_POSDIV(v)  ((v >> 10)&0x3)
#define PLL_GET_PREDIV(v) ((v >> 8)&0x3)
#define PLL_GET_FBDIV(v)  ((v >> 1)&0x7F)

#define PLL_FBSEL(v)    ((v & 0x3)<<14)
#define PLL_CKCTRL(v)   ((v & 0x3)<<12)
#define PLL_POSDIV(v)   ((v & 0x3)<<10)
#define PLL_PREDIV(v)   ((v & 0x3)<<8)
#define PLL_FBDIV(v)    ((v & 0x7F)<<1)
#define PLL_PWD         (1)

#define PLL_SETTING_MASK   (0xFFFF)
#define PLL_108MHZ    (PLL_FBSEL(0)| PLL_CKCTRL(0)|PLL_POSDIV(0)|PLL_PREDIV(1)|PLL_FBDIV(7)|PLL_PWD)
#define APLL_288MHz   (PLL_FBSEL(1)| PLL_CKCTRL(0)|PLL_POSDIV(2)|PLL_PREDIV(0)|PLL_FBDIV(7)|PLL_PWD)
#define APLL26_26MHz  (PLL_FBSEL(0)| PLL_CKCTRL(1)|PLL_POSDIV(0)|PLL_PREDIV(0)|PLL_FBDIV(0x19)|PLL_PWD)
#define APLL_CALIBRATION_TIMEOUT    0x10000

#endif


/*
 * IO Macro
 */
#define HAL_WRITE32(_reg_, _val_)   		(*((volatile uint32_t*)(_reg_)) = (_val_))
#define HAL_READ32(_reg_)           		(*((volatile uint32_t*)(_reg_)))

#define IO_READ32(base, offset)                 HAL_READ32((base) + (offset))
#define IO_WRITE32(base, offset, value)         HAL_WRITE32((base) + (offset), (value))
#define	IO_REG32(base, offset)									HAL_READ32((base) + (offset))

#define CKGEN_READ32(offset)           		IO_READ32(CKGEN_VIRT, (offset))
#define CKGEN_WRITE32(offset, value)   		IO_WRITE32(CKGEN_VIRT, (offset), (value))

#define BIM_READ32(offset)           		IO_READ32(BIM_VIRT, (offset))
#define BIM_WRITE32(offset, value)   		IO_WRITE32(BIM_VIRT, (offset), (value))
#define BIM_REG32(offset)   						IO_REG32(BIM_VIRT, (offset))

#define BIM2_READ32(offset)           		IO_READ32(BIM2_VIRT, (offset))
#define BIM2_WRITE32(offset, value)   		IO_WRITE32(BIM2_VIRT, (offset), (value))
#define BIM2_REG32(offset)   						IO_REG32(BIM2_VIRT, (offset))

#define PDWNC_READ32(offset)           		IO_READ32(PDWNC_VIRT, (offset))
#define PDWNC_WRITE32(offset, value)   		IO_WRITE32(PDWNC_VIRT, (offset), (value))

#endif /* __ASM_ARCH_AC83XX_BASIC_H */

