/*
 *  linux/arch/arm/arch-ac83xx/ac83xx_system.c
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */

#include <mach/hardware.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include <mach/ac83xx_system.h>

#define SPM_READ32(REG)            __raw_readl(__io(SPM_BASE_VA + REG))
#define SPM_WRITE32(VAL, REG)      __raw_writel(VAL, __io(SPM_BASE_VA + REG))



#define MAX_PDWNC_INTR_SOURCE   (4)
typedef struct _intr_source{

	unsigned  int id;
	irq_handler_t handler;
	const char *name;

}intr_source;

static intr_source g_pdwnc_intrsource[MAX_PDWNC_INTR_SOURCE];
spinlock_t  irq_lock;

int  request_pdwnc_irq(unsigned int sourceid, irq_handler_t handler, unsigned long flags,
		const char *name, void *dev)
{
	if(sourceid >= (MAX_PDWNC_INTR_SOURCE))
		return -1;


	printk("request_pdwnc_irq:  sourceid=%d\n", sourceid);
	g_pdwnc_intrsource[sourceid].id = sourceid;
	g_pdwnc_intrsource[sourceid].handler = handler;
	g_pdwnc_intrsource[sourceid].name = name;

	return 0;

}
EXPORT_SYMBOL(request_pdwnc_irq);

void  set_pdwnc_gpio_value(uint32_t u4Pin,uint32_t u4value)
{

	uint32_t u4Mode =0; 
	uint32_t u4Tmp;

	//set pin to GPIO
	u4Tmp = (1<<  u4Pin);
	PDWNC_WRITE32(REG_RW_PAD_PINMUX1, u4Tmp); 

	
	//GPIO direct
	u4Tmp = PDWNC_READ32(REG_RW_GPIOEN);

	if(u4Mode == 1)
		u4Tmp = u4Tmp & (~(1U << u4Pin));
	else
		u4Tmp = u4Tmp | ((1U << u4Pin));

	PDWNC_WRITE32(REG_RW_GPIOEN, u4Tmp);

	//set output value
	u4Tmp = PDWNC_READ32(REG_RW_GPIOOUT);
	u4Tmp = u4Tmp & ( ~(1 << u4Pin));
	u4Tmp = u4Tmp | (u4value << u4Pin);
	if(u4Mode == 0)
		PDWNC_WRITE32(REG_RW_GPIOOUT,u4Tmp);


}


void arch_reset(char mode, const char *cmd)
{
	/*
	 * use powerdown watch dog to reset system
	 */
	uint32_t u4Test;


	PDWNC_WRITE32(REG_RW_RESRV1, 0x33633363);

	PDWNC_WRITE32(REG_RW_WDT, 0xff000000);

	for(u4Test = 0; u4Test < 10000; u4Test++)
	{

	}
	PDWNC_WRITE32(REG_RW_WDTSET, 1);
	while(1);

}
void ac83xx_power_off(void)
{
#if 0
	// TODO: pull gpio to notify mcu shutting down is complete and do power off
	notify_mcu_power_off();
#else
	arch_reset(0,NULL);
#endif
	// wait for mcu to power off
	while(1);
}

static uint32_t  pdwnc_gpio_init(uint32_t u4Pin,uint32_t u4TrigMode,uint32_t u4Enable)
{

	uint32_t  u4Tmp;




	//set pin to GPIO
	u4Tmp = (1<<  u4Pin);
	PDWNC_WRITE32(REG_RW_PAD_PINMUX1, u4Tmp); 


	//GPIO input set
	u4Tmp = PDWNC_READ32(REG_RW_GPIOEN);
	u4Tmp = u4Tmp & (~(1U << u4Pin));
	PDWNC_WRITE32(REG_RW_GPIOEN, u4Tmp);
	//set Interrupt Trigger mode
	switch(u4TrigMode)
	{
		case 0://Level low mode
			u4Tmp = (0x100 << u4Pin);
			u4Tmp = u4Tmp | (0 << u4Pin);
			break;
		case 1://Level high mode
			u4Tmp = (0x100 << u4Pin);
			u4Tmp = u4Tmp | (1 << u4Pin);
			break;
		case 2://negative edge mode
			u4Tmp = (0x000 << u4Pin);
			u4Tmp = u4Tmp | (0 << u4Pin);
			break;
		case 3://positive edge mode
			u4Tmp = (0x000 << u4Pin);
			u4Tmp = u4Tmp | (1 << u4Pin);
			break;
		case 4://Double trigger mode
			u4Tmp = (0x10000 << u4Pin);
			break;
		default :
			printk("[Error]Trigger mode %d is error.\n",u4TrigMode);
			return -1;
	}
	printk("gpio trigger mode setting:0x%x\n",u4Tmp);
	PDWNC_WRITE32(REG_RW_EXINTCFG,u4Tmp);

	//Interropt enable or disable
	if(0 == u4Enable)
	{
		PDWNC_WRITE32(REG_RW_INTEN, PDWNC_READ32(REG_RW_INTEN)&(~(1<<u4Pin)));
		printk("Pin %d interrupt Disable.\n",u4Pin);
	}
	else
	{
		PDWNC_WRITE32(REG_RW_INTEN, PDWNC_READ32(REG_RW_INTEN)|(1<< u4Pin));
		printk("Pin %d interrupt Enable.\n",u4Pin);
	}

	return 0;

}

#if 0
static int32_t  pdwnc_config_wakeup(uint32_t u4Pin,uint32_t u4Prolarity)
{

	printk("set gpio %d wakeup\n",u4Pin);
	vPDWNC_SetGPIO_Wakeup(u4Pin,u4Prolarity);
	vPDWNC_SetDCStatus(1);
	vPDWNC_SetWakeup_DelayCount(0x30000);	
	return 0;

}

#endif
extern void ac83xx_mask_ack_bim_irq(uint32_t);
static irqreturn_t pdwnc_isr_handler(int irq, void *dev_id)
{
	uint32_t u4tmp = PDWNC_READ32(REG_RW_INTSTA);

	unsigned long irqflags;
	spin_lock_irqsave(&irq_lock, irqflags);
	disable_irq_nosync(VECTOR_PWDNC);
	spin_unlock_irqrestore(&irq_lock, irqflags);


	if(u4tmp & (1U << 0)){

		if(g_pdwnc_intrsource[PDWNC_INTR_GPIO_WAKEUP_STS].handler != NULL){
			g_pdwnc_intrsource[PDWNC_INTR_GPIO_WAKEUP_STS].handler(irq,dev_id);
		}

	}

	if(u4tmp & (1U << 1)){

		if(g_pdwnc_intrsource[PDWNC_INTR_GPIO_WAKEUP_SRC].handler != NULL){
			g_pdwnc_intrsource[PDWNC_INTR_GPIO_WAKEUP_SRC].handler(irq,dev_id);
		}
	}

	if(u4tmp & (1U << 2)){
		if(g_pdwnc_intrsource[PDWNC_INTR_GPIO_IR].handler != NULL){
			g_pdwnc_intrsource[PDWNC_INTR_GPIO_IR].handler(irq,dev_id);
		}
	}

	if(u4tmp & (1U << 13)){
		if(g_pdwnc_intrsource[PDWNC_INTR_IR].handler != NULL){
			g_pdwnc_intrsource[PDWNC_INTR_IR].handler(irq,dev_id);
		}
	}

	PDWNC_WRITE32(REG_RW_INTCLR,u4tmp);
	ac83xx_mask_ack_bim_irq(VECTOR_PWDNC);

	irqflags = 0;
	spin_lock_irqsave(&irq_lock, irqflags);
	enable_irq(VECTOR_PWDNC);
	spin_unlock_irqrestore(&irq_lock, irqflags);

	return IRQ_HANDLED;
}






static int __init  pdwnc_init()
{
	int ret = 0;

	/* enable spm clock  */
	SPM_WRITE32(0x02860001, 0);

	pdwnc_gpio_init(1,3,1);
	spin_lock_init(&irq_lock);

	memset((void *)g_pdwnc_intrsource,0x0,sizeof(g_pdwnc_intrsource));
	ret = request_irq(VECTOR_PWDNC, pdwnc_isr_handler,
			0,"PDWNC_ISR", NULL);
	if(0 == ret)
		printk("pdwnc_init success\r\n");
	else
		printk("pdwnc_init failed\r\n");

	return ret;

}
static void pdwnc_exit(void)
{

	free_irq(VECTOR_PWDNC,NULL);

}

module_init(pdwnc_init);
module_exit(pdwnc_exit);
