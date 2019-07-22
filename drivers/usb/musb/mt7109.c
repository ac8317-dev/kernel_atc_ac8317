#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include "musb.h"
#include "musb_core.h"

#include <mach/ic_version.h>
//#include "../host/mtk_hcd.h"
#include <mach/chip_ver.h>
//#include <linux/usb/android_composite.h>
#include <mach/ac83xx_basic.h>
#include <mach/irqs.h>

#if 10
//#if (MUSB_BASE0 < IO_VIRT)
//#error ......Wrong MUSB_BASE0 address setting
//#endif


    #define UNIFIED_USB 1 
//  #define USB_SINGLE_PORT           1


/* physical address */

#define USB_BASE         (0x7000E000) 

#define USB_SPACE_SIZE   0x1000
#define USB_IRQ          VECTOR_USB 

#define MUSB_VECTOR_USB0                VECTOR_USB 
#define MUSB_VECTOR_USB1                VECTOR_USB2 

#define MBIM_VIRT                       (IO_VIRT + 0x08000)
#ifndef MUSB_BASE
#define MUSB_BASE                       (IO_VIRT + 0xE000)
#define MUSB_BASE2                      (IO_VIRT + 0x3C000)
#endif

#ifdef UNIFIED_USB//dexiao
#define MUSB_COREBASE                                   (0x000)
#define MUSB_DMABASE                                    (0x200)
#define MUSB_PHYBASE                                    (0x1000)
#define MUSB_PORT0_PHYBASE                              (0x800)
#define MUSB_PORT1_PHYBASE                              (0x900)

#define M_REG_PERFORMANCE1 0x70
#define M_REG_PERFORMANCE2 0x72

#define M_REG_PERFORMANCE3 0x74
   // BUSPERF3
#define MGC_M_BUSPERF3_VBUSERR_MODE               (1<<11)
#define MGC_M_BUSPERF3_FLUSHFIFO_EN               (1<< 9)
#define MGC_M_BUSPERF3_NOISESTILL_SOF             (1<< 7)
#define MGC_M_BUSPERF3_BAB_CLR_EN                 (1<< 6)
#define MGC_M_BUSPERF3_UNDO_SRPFIX                (1<< 3)
#define MGC_M_BUSPERF3_OTG_DEGLITCH_DISABLE       (1<< 2)
#define MGC_M_BUSPERF3_EP_SWRST                   (1<< 1)
#define MGC_M_BUSPERF3_DISUSBREST                 (1<< 0)

#endif
#define MGC_PHY_Read32(_pBase, r) (\
    *((volatile uint32_t *)((MUSB_BASE)+(MUSB_PHYBASE)+ (MUSB_PORT0_PHYBASE) + (r)))\
)

extern int printk(const char *format, ...);
#define MGC_PHY_Write32(_pBase, r, v) { \
    *((volatile uint32_t *)((MUSB_BASE + MUSB_PHYBASE)+(MUSB_PORT0_PHYBASE) + (r))) = v;\
    printk("USB Port0 PHY@0x%08X = 0x%08X\n", ((uint32_t)((MUSB_BASE + MUSB_PHYBASE)+(MUSB_PORT0_PHYBASE) + (r))), (uint32_t)v);\
}

#define MGC_BIM_Read32(r)           *((volatile uint32_t *)((BIM_VIRT)+ (r)))

#define MGC_BIM_Write32(r, v) {\
    (*((volatile uint32_t *)((BIM_VIRT)+ (r))) = (v));\
    printk("USB MGC_BIM@0x%08X = 0x%08X\n", ((uint32_t)(BIM_VIRT+(r))), (uint32_t)(v));\
}

#define MGC_CKGEN_Read32(r)           *((volatile uint32_t *)((CKGEN_VIRT)+ (r)))

#define MGC_CKGEN_Write32(r, v) {\
    (*((volatile uint32_t *)((CKGEN_VIRT)+ (r))) = (v));\
    printk("USB MGC_CKGEN@0x%08X = 0x%08X\n", ((uint32_t)(CKGEN_VIRT+(r))), (uint32_t)(v));\
}

#endif

/* physical address */
#define MT7109_USB_BASE         (0x7000E000)
#define MT7109_USB_IRQ         VECTOR_USB 

#ifdef UNIFIED_USB
#define MT7109_USB_SPACE_SIZE   (0x1000)
#else
#define MT7109_USB_SPACE_SIZE   (0x1000 - 0x800)
#endif

static struct musb_hdrc_config musb_config_mt7109 = {
        .multipoint     = true, //false, //ben for test
        .dyn_fifo       = true,
        .soft_con       = true,
        .dma            = true, 

        .num_eps        = 5,
        .dma_channels   = 2,
        .ram_bits       = 10,
};

static struct musb_hdrc_platform_data usb_data_mt7109 = {
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
    .mode           = MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
    .mode           = MUSB_HOST,
#endif
    .config         = &musb_config_mt7109,
};

static struct resource usb_resources_mt7109[] = {
        {
                /* physical address */ 
                .start          = MT7109_USB_BASE,
                .end            = MT7109_USB_BASE + MT7109_USB_SPACE_SIZE - 1,
                .flags          = IORESOURCE_MEM,
        },
        {   //general IRQ
                .start          = MT7109_USB_IRQ,
                .flags          = IORESOURCE_IRQ,
        },
        {   //DMA IRQ
                .start          = MT7109_USB_IRQ,
                .flags          = IORESOURCE_IRQ,
        },
};

static void mt7109_release(struct device *dev)
{
     DBG(3, "dev = 0x%08X.\n", (uint32_t)dev);
}

static u64 usb_dmamask = DMA_BIT_MASK(32);

static struct platform_device usb_dev = {
        .name           = "musb_hdrc",
        .id             = -1,
        .dev = {
                .platform_data          = &usb_data_mt7109,
                .dma_mask               = &usb_dmamask,
                .coherent_dma_mask      = DMA_BIT_MASK(32),
                .release        = mt7109_release,
        },
        .resource       = usb_resources_mt7109,
        .num_resources  = ARRAY_SIZE(usb_resources_mt7109),
};

#define VENDOR_ID 0x18D1    //Google
#define PRODUCT_ID 0xD0DB
#define ADB_ONLY            0
#define MASS_STORAGE_ONLY   0


#if ADB_ONLY //-------------------------------------------------------------//
static char *usb_functions_adb[] = {
    "adb",
};

static char *usb_functions_all[] = {
    "adb",
};

static struct android_usb_product usb_products[] = {
    {
     .product_id = PRODUCT_ID,
     .num_functions = ARRAY_SIZE(usb_functions_adb),
     .functions = usb_functions_adb,
    },
};

static struct android_usb_platform_data android_usb_pdata = {
    .vendor_id  = VENDOR_ID,
    .product_id = PRODUCT_ID,
    .functions  = usb_functions_all,
    .products   = usb_products,
    .version    = 0x0100,
    .product_name   = "Mediatek Android USB Gadget",
    .manufacturer_name  = "Mediatek",
    .serial_number  = "MTK-B6CE",
    .num_functions  = ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
    .name   = "android_usb",
    .id = -1,
    .dev    = {
     .platform_data = &android_usb_pdata,
    },
};

void __init setup_mt7109_usb(void)
{
    platform_device_register(&usb_dev);
    platform_device_register(&androidusb_device);
}

void __exit destroy_mt7109_usb(void)
{
    platform_device_unregister(&usb_dev);
    platform_device_unregister(&androidusb_device);
}

#else //-------------------------------------------------------------//

#if MASS_STORAGE_ONLY

struct usb_mass_storage_platform_data mass_storage_data = {
    .vendor = "Mediatek",
    .product = "MTK Mass Storage",
    .release = 1,

    /* number of LUNS */
    .nluns = 1,
};

struct platform_device fsg_platform_device = {
     .name = "usb_mass_storage",  
     .id   = -1,
     .dev   = {
            .platform_data = &mass_storage_data,
        },
};


void __init setup_mt7109_usb(void)
{
    platform_device_register(&usb_dev);
    platform_device_register(&fsg_platform_device);
}

void __exit destroy_mt7109_usb(void)
{
    platform_device_unregister(&usb_dev);
    platform_device_unregister(&androidusb_device);
}

#else
/*
static char *usb_functions_adb[] = {
    "adb",
};

static char *usb_functions_mass_storage[] = {
    "usb_mass_storage",
};


static char *usb_functions_all[] = {
    "adb",
    "usb_mass_storage",
};

static struct android_usb_product usb_products[] = {
    {
     .product_id = PRODUCT_ID,
     .num_functions = ARRAY_SIZE(usb_functions_all),
     .functions = usb_functions_all,
    },
};


static struct android_usb_platform_data android_usb_pdata = {
    .vendor_id  = VENDOR_ID,
    .product_id = PRODUCT_ID,
    .functions  = usb_functions_all,
    .products   = usb_products,
    .version    = 0x0100,
    .product_name   = "Mediatek Android USB Gadget",
    .manufacturer_name  = "Mediatek",
    .serial_number  = "MTK-B6CE",
    .num_functions  = ARRAY_SIZE(usb_functions_all),
};

static struct platform_device android_usb_device = {
    .name   = "android_usb",
    .id     = -1,
    .dev    = {
     .platform_data = &android_usb_pdata,
    },
};

// Add for mass storage device
struct usb_mass_storage_platform_data mass_storage_data = {
    .vendor = "Mediatek",
    .product = "MTK Mass Storage",
    .release = 1,
    .nluns = 2,
};

struct platform_device fsg_platform_device = {
     .name = "usb_mass_storage",  
     .id   = -1,
     .dev   = {
            .platform_data = &mass_storage_data,
        },
};

*/
void __init setup_mt7109_usb(void)
{
    platform_device_register(&usb_dev);
     //platform_device_register(&android_usb_device);
     //platform_device_register(&fsg_platform_device);
}

void __exit destroy_mt7109_usb(void)
{
    platform_device_unregister(&usb_dev);
}

#endif

#endif //-------------------------------------------------------------//

//-------------------------------------------------------------------------
/** MUC_ResetPhy
 *  host controller register reset and initial.
 *  @param  void 
 *  @return  void
 */
//-------------------------------------------------------------------------
static int MUC_ResetPhy(void *pBase)
{
            uint32_t u4Reg = 0;
//          MUSB_ASSERT(pBase);
    //set usb clk
    u4Reg = MGC_CKGEN_Read32(0x284);
    u4Reg &= ~0x00000001; 
    MGC_CKGEN_Write32(0x284, u4Reg);    

    u4Reg = MGC_CKGEN_Read32(0xA0);
    u4Reg |= (0x1 << 13);
    MGC_CKGEN_Write32(0xA0, u4Reg);
    
    //Soft Reset, RG_RESET for Soft RESET
    u4Reg = MGC_PHY_Read32(pBase,(uint32_t)0x68);
    u4Reg |=   0x00004000; 
    MGC_PHY_Write32(pBase, (uint32_t)0x68, u4Reg);

    u4Reg = MGC_PHY_Read32(pBase,(uint32_t)0x68);
    u4Reg &=  ~0x00004000; 
    MGC_PHY_Write32(pBase, (uint32_t)0x68, u4Reg);
                
                    // set 0E410H, REL_SUSP, USBRST
                    //MGC_PHY_Write32(pBase, 0x10, 0x01010000);
                    //MGC_PHY_Write32(pBase, 0x10, 0x00010000);
            //otg bit setting
                u4Reg = MGC_PHY_Read32(pBase,(uint32_t)0x6C);
                u4Reg &= ~0x3f3f;
        #ifdef CONFIG_USB_GADGET_MUSB_HDRC
                u4Reg |=  0x3e2e;
        #else
                u4Reg |=  0x3e2c;
        #endif
                MGC_PHY_Write32(pBase, (uint32_t)0x6C, u4Reg);
                    
            //suspendom control
                u4Reg = MGC_PHY_Read32(pBase,(uint32_t)0x68);
                u4Reg &=  ~0x00040000; 
                MGC_PHY_Write32(pBase, (uint32_t)0x68, u4Reg);
                
                    // set 0E404H, PLL_EN (RG_PLL_STABLE0)
                    //MGC_PHY_Write32(pBase, 0x04, 0x48000406);
                //  u4Reg = MGC_PHY_Read32(pBase, 0x04);
                //  u4Reg |=  0x40000000; 
                //  MGC_PHY_Write32(pBase, 0x04, u4Reg);
                
                //  MGC_MISC_Write32(pBase, 0xE0, 0x1818);
                u4Reg = musb_readb((void *)MUSB_BASE, M_REG_PERFORMANCE3);
                u4Reg |=  0x80;
                u4Reg &= ~0x40;
                musb_writeb((void *)MUSB_BASE, M_REG_PERFORMANCE3, (uint8_t)u4Reg);
                mdelay(10);
                printk("[usb]pBase 0x%x phy init complete\n",(uint32_t)MUSB_BASE);

    return 0;
}

int mt7109_platform_init(struct musb *musb)
{
#if 0
    #define USBPHY_TESTMODE (volatile unsigned *)(HIF_USB+0x40)
    #define USBPHY_DPLL (volatile unsigned *)(HIF_USB+0x44)
    #define MISC_PINTRAP (volatile unsigned *)(MISC+0x0c)

    unsigned HIF_USB=(unsigned)ioremap_nocache(0x88005000, 0x1000);
    unsigned MISC;
    unsigned pintrap, clock, pll_dr, dpll;

    //disable TEST MODE of PHY and set USB PHY DPLL setting
    *USBPHY_TESTMODE=*USBPHY_TESTMODE & ~1;

    //DPLL setting of PHY
    MISC=(unsigned)ioremap_nocache(0x80070000, 0x1000);
    pintrap=*MISC_PINTRAP;
    switch((pintrap >> 21) & 3){
        case 0:
            clock=40;
            break;
        case 1:
            clock=30;
            break;
        case 2:
            clock=20;
            break;
        default:
            clock=0;
            break;
    }
    if(pintrap & 0x00800000){//CFG_XTAL_DIV2
        clock=clock >> 1;
    }
    DBG (2, "clock is %dMHz\n", clock);

    pll_dr=480/clock;
    dpll=*USBPHY_DPLL;
    dpll = (dpll & ~0x3f0000) | (pll_dr << 16);
    DBG (2, "DPLL = 0x%x\n", dpll);
    *USBPHY_DPLL=dpll;

    iounmap((void *)MISC);
    iounmap((void *)HIF_USB);

    #undef USBPHY_TESTMODE
    #undef USBPHY_DPLL
    #undef MISC_PINTRAP

#else
    MUC_ResetPhy((void *)MUSB_BASE);
#endif
    return 0;
}

int mt7109_platform_exit(struct musb *musb)
{
    return 0;
}

int mt7109_platform_set_mode(struct musb *musb, u8 musb_mode)
{
    return 0;
}

void mt7109_platform_enable(struct musb *musb)
{
}

void mt7109_platform_disable(struct musb *musb)
{
}

