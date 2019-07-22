#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include "musb.h"
#include "musb_core.h"
//#include "../host/mtk_hcd.h"

/* physical address */
#define MT7109_USB_BASE         (0x7000E000)
#define MT7109_USB_IRQ         VECTOR_USB


#define MT7109_USB_SPACE_SIZE   (0x1000)

//-------------------------------------------------------------------------

#if defined CONFIG_FPGA
#include "asm/arch/gpio/gpio.h"
#define SDA    0        /// GPIO #0: I2C data pin
#define SCL    1        /// GPIO #1: I2C clock pin

unsigned int  i2c_dummy_cnt;
#define I2C_DUMMY_DELAY(_delay) for (i2c_dummy_cnt = ((_delay)*10) ; i2c_dummy_cnt!=0; i2c_dummy_cnt--)

void SerialCommStart(void) /* Prepare the SDA and SCL for sending/receiving */
{
	GPIO_DIRSet(SDA, 1); //output
	GPIO_PollSet(SDA, 1, 0);
	I2C_DUMMY_DELAY(100);

	GPIO_Set(SDA, 1);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 1);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SDA, 0);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 0);
	I2C_DUMMY_DELAY(5);
}

void SerialCommStop(void)
{
	GPIO_DIRSet(SDA, 1); //output
	GPIO_PollSet(SDA, 1, 0);
	I2C_DUMMY_DELAY(100);

	GPIO_Set(SCL, 0);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SDA, 0);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 1);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SDA, 1);
	I2C_DUMMY_DELAY(5);
}

unsigned int SerialCommTxByte(unsigned char data) /* return 0 --> ack */
{
	int i, ack;

	GPIO_DIRSet(SDA, 1); //output
	GPIO_PollSet(SDA, 1, 0);
	I2C_DUMMY_DELAY(100);

	for(i=8; --i>0;){
		GPIO_Set(SDA, (data>>i)&0x01);
		I2C_DUMMY_DELAY(5);
		GPIO_Set(SCL, 1);
		I2C_DUMMY_DELAY(5);
		GPIO_Set(SCL, 0);
		I2C_DUMMY_DELAY(5);
	}

	GPIO_Set(SDA, (data>>i)&0x01);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 1);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 0);
	I2C_DUMMY_DELAY(5);

	GPIO_Set(SDA, 0);
	I2C_DUMMY_DELAY(5);

	GPIO_DIRSet(SDA, 0); //input
	GPIO_PollSet(SDA, 0, 0);
	I2C_DUMMY_DELAY(100);

   	GPIO_Set(SCL, 1);
	I2C_DUMMY_DELAY(5);
	ack = GPIO_Get(SDA);
	GPIO_Set(SCL, 0);
	I2C_DUMMY_DELAY(5);
	if(ack==1)
		return 0;
	else
		return 1;
}

void SerialCommRxByte(unsigned  char *data, unsigned char ack)
{
	int i;
	unsigned int dataCache;
	dataCache = 0;

	GPIO_DIRSet(SDA, 0); //input
	GPIO_PollSet(SDA, 0, 0);
	I2C_DUMMY_DELAY(100);

	for(i=8; --i>=0;){
		dataCache <<= 1;
		I2C_DUMMY_DELAY(5);
		GPIO_Set(SCL, 1);
		I2C_DUMMY_DELAY(5);
		dataCache |= GPIO_Get(SDA);
		GPIO_Set(SCL, 0);
		I2C_DUMMY_DELAY(5);
	}

	GPIO_DIRSet(SDA, 1); //output
	GPIO_PollSet(SDA, 1, 0);
	I2C_DUMMY_DELAY(100);

	GPIO_Set(SDA, ack);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 1);
	I2C_DUMMY_DELAY(5);
	GPIO_Set(SCL, 0);
	I2C_DUMMY_DELAY(5);
	*data = (unsigned char)dataCache;
}

int I2cWriteReg(unsigned char dev_id, unsigned char Addr, unsigned char Data)
{
	int acknowledge=0;
	SerialCommStart();
	acknowledge=SerialCommTxByte( (dev_id<<1) &0xff);
	if(acknowledge)
		acknowledge=SerialCommTxByte(Addr);
	else
		return 0;
	acknowledge=SerialCommTxByte(Data);
	if(acknowledge){
		SerialCommStop();
		return 1;
	}else{
	        return 0;
	}
}

int I2cReadReg(unsigned char dev_id, unsigned char Addr, unsigned char *Data)
{
	int acknowledge=0;
	SerialCommStart();
	acknowledge=SerialCommTxByte((dev_id<<1) &0xff);
	if(acknowledge)
		acknowledge=SerialCommTxByte(Addr);
	else
		return 0;
	SerialCommStart();
	acknowledge=SerialCommTxByte(((dev_id<<1) & 0xff) |0x01 );
	if(acknowledge)
		SerialCommRxByte(Data, 1);  // ack 0: ok , 1 error
	else
		return 0;
	SerialCommStop();
	return acknowledge;
}
#endif

//-------------------------------------------------------------------------

static struct musb_hdrc_config musb_config_mt7119 = {
        .multipoint     = true, //false, //ben for test
        .dyn_fifo       = true,
        .soft_con       = true,
        .dma            = true,

        .num_eps        = 5,
        .dma_channels   = 2,
        .ram_bits       = 10,
};

static struct musb_hdrc_platform_data usb_data_mt7119 = {
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#endif
	.config         = &musb_config_mt7119,
};

static struct resource usb_resources_mt7119[] = {
        {
                /* physical address */
                .start          = MT7109_USB_BASE,
                .end            = MT7109_USB_BASE + MT7109_USB_SPACE_SIZE - 1,
                .flags          = IORESOURCE_MEM,
        },
        {	//general IRQ
                .start          = MT7109_USB_IRQ,
                .flags          = IORESOURCE_IRQ,
        },
        {	//DMA IRQ
                .start          = MT7109_USB_IRQ,
                .flags          = IORESOURCE_IRQ,
        },
};

static u64 usb_dmamask = DMA_BIT_MASK(32);

static struct platform_device usb_dev = {
        .name           = "musb_hdrc",
        .id             = -1,
        .dev = {
                .platform_data          = &usb_data_mt7119,
                .dma_mask               = &usb_dmamask,
                .coherent_dma_mask      = DMA_BIT_MASK(32),
        },
        .resource       = usb_resources_mt7119,
        .num_resources  = ARRAY_SIZE(usb_resources_mt7119),
};

void __init setup_mt7119_usb(void)
{
	platform_device_register(&usb_dev);
}

void __exit destroy_mt7119_usb(void)
{
	platform_device_unregister(&usb_dev);
}

int workaround_counter=0;
spinlock_t workaround_lock;
unsigned AHBMON;

extern void ENABLE_USB_IRQ(void);

#if 0
static irqreturn_t gpt_interrupt(int irq, void *data)
{
        ENABLE_USB_IRQ();
        return IRQ_HANDLED;
}
#endif

int __init mt7119_platform_init(struct musb *musb)
{
#if 0
#define CLKCTL_B_USB_ON 1
#define CLKCTL_HIFCON_USB (volatile unsigned *)(CLKCTL_HIFCON)
#define USBPHY_TESTMODE (volatile unsigned *)(HIF_USB+0x40)
#define USBPHY_DPLL (volatile unsigned *)(HIF_USB+0x44)
#define MISC_PINTRAP (volatile unsigned *)(MISC+0x0c)

#define DPLL_AP_KEY_PASS 0x24542454;
#define DPLL_AP_USBPLL (volatile unsigned int *)(DPLL+0x70)
#define DPLL_AP_KEY_PRT (volatile unsigned int *)(DPLL+0x7c)
#define DPLL_M_AP_USBPLL_PLL_STABLE  0x00020000

	unsigned HIF_USB=(unsigned)ioremap_nocache(0x86000500, 0x100);
	unsigned CLKCTL_HIFCON=(unsigned)ioremap_nocache(0x8000000c, 0x100);
	unsigned DPLL=(unsigned)ioremap_nocache(0x800d0000, 0x100);
#if defined CONFIG_FPGA
	unsigned char printbuf;
#endif
	AHBMON=(unsigned)ioremap_nocache(0x800f0000, 0x1000);

	spin_lock_init(&workaround_lock);

#if 0
        *(volatile unsigned *)(GPT_BASE+0x18) = 0;//disable GPT1
        *(volatile unsigned *)(CIRQ_BASE+0xc) = (1<<1); //mask GPT1 interrupt
        request_irq(1, gpt_interrupt, 0, "GPT1", NULL);
        *(volatile unsigned *)(CIRQ_BASE+0x8) = (1<<1); //unmask GPT1 interrupt
#endif
	
	*CLKCTL_HIFCON_USB= *CLKCTL_HIFCON_USB | CLKCTL_B_USB_ON;
	msleep(1);
	*USBPHY_TESTMODE=*USBPHY_TESTMODE & ~1;
	*DPLL_AP_KEY_PRT = DPLL_AP_KEY_PASS;
	*DPLL_AP_USBPLL = *DPLL_AP_USBPLL & ~DPLL_M_AP_USBPLL_PLL_STABLE;
	msleep(1);
	*DPLL_AP_USBPLL = *DPLL_AP_USBPLL | DPLL_M_AP_USBPLL_PLL_STABLE;
	msleep(3);
	*DPLL_AP_KEY_PRT = 0;
	
#if 0
	unsigned pintrap, clock, pll_dr, dpll;
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
#endif

	iounmap((void *)DPLL);
	iounmap((void *)HIF_USB);
	iounmap((void *)CLKCTL_HIFCON);

	// FPGA - Reset USB PHY register in FPGA via APGPIO
#if defined CONFIG_FPGA
	GPIO_MODESet(SCL, 0);
	GPIO_MODESet(SDA, 0);

	GPIO_DIRSet(SCL, 1); //output
	GPIO_PollSet(SCL, 1, 0);
	I2C_DUMMY_DELAY(100);
	
        I2cWriteReg(0x60, 0x02, 0x5A);
        I2cReadReg(0x60, 0x02, (unsigned char *)&printbuf);
        printk ("I2C Reg=%02x\n", printbuf);
        I2cWriteReg(0x60, 0x02, 0xA5);
        I2cReadReg(0x60, 0x02, (unsigned char *)&printbuf);
        printk("I2C Reg=%02x\n", printbuf);

        I2cWriteReg(0x60, 0x02, 0x44);

	GPIO_Set(SCL, 1);
	GPIO_Set(SDA, 1);
#endif

#undef CLKCTL_B_USB_ON
#undef CLKCTL_HIFCON_USB
#undef USBPHY_TESTMODE
#undef USBPHY_DPLL
#undef MISC_PINTRAP
#else

#endif
    return 0;
}

int mt7119_platform_exit(struct musb *musb)
{
	return 0;
}

int mt7119_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	return 0;
}

void mt7119_platform_enable(struct musb *musb)
{
}

void mt7119_platform_disable(struct musb *musb)
{
}

