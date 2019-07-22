#include <linux/types.h>
#include <mach/chip_ver.h>
#include <mach/hardware.h>
#include <linux/module.h>
#include <asm/io.h>

#include <linux/irq.h>

void ac83xx_mask_bim_irq(uint32_t irq)
{
    uint32_t regval32;
    unsigned long flags;


	if(irq < IRQ_BIM_GIC_OFFSET)
	{
		return;
	}
	else
	{
		irq -= IRQ_BIM_GIC_OFFSET;
	}

    //assert(irq < NR_IRQS);
	
    //local_irq_save(flags);

    if(irq < 32)
    {
        regval32 = __bim_readl(REG_IRQEN);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;
        regval32 = __bim_readl(REG_IRQEN3);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN3);
    }
    else if((irq >= 96) && (irq < 128))
    {
    	irq = irq - 96;
        regval32 = __bim_readl(REG_IRQEN4);
        regval32 &= ~(1 << irq);
        __bim_writel(regval32, REG_IRQEN4);
    }
	else
	{

	}

    //local_irq_restore(flags);
}
EXPORT_SYMBOL(ac83xx_mask_bim_irq);

//disable IRQ and clear IRQ status



//void ac83xx_mask_ack_bim_irq(uint32_t irq)
//{
//}


//void ac83xx_mask_ack_bim_irq_gic(struct irq_data *data)
void ac83xx_mask_ack_bim_irq(uint32_t irq)
{
    uint32_t regval32;
    unsigned long flags;

    //uint32_t irq = data->hwirq;

	if(irq < IRQ_BIM_GIC_OFFSET)
	{
		return;
	}
	else
	{
		irq -= IRQ_BIM_GIC_OFFSET;
	}


    //assert(irq < NR_IRQS);

    //local_irq_save(flags);

    if(irq < 32)
    {
        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL);
        __bim_writel(regval32, REG_IRQST);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL2);
        __bim_writel(regval32, REG_IRQST2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL3);
        __bim_writel(regval32, REG_IRQST3);
    }
	else if((irq >= 96) && (irq < 128))
    {
    	irq = irq - 96;

        //set 1 to clear
        regval32 = (1 << irq);
        __bim_writel(regval32, REG_IRQCL4);
        __bim_writel(regval32, REG_IRQST4);
    }
    else
    {

    }

    //local_irq_restore(flags);
}
EXPORT_SYMBOL(ac83xx_mask_ack_bim_irq);



void ac83xx_unmask_bim_irq(uint32_t irq) //enable
{
    uint32_t regval32;
    unsigned long flags;



	if(irq < IRQ_BIM_GIC_OFFSET)
	{
		return;
	}
	else
	{
		irq -= IRQ_BIM_GIC_OFFSET;
	}


    //local_irq_save(flags);


    if(irq < 32)
    {
        regval32 = __bim_readl(REG_IRQEN);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN);
    }
    else if((irq >= 32) && (irq < 64))
    {
    	irq = irq - 32;
        regval32 = __bim_readl(REG_IRQEN2);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN2);
    }
    else if((irq >= 64) && (irq < 96))
    {
    	irq = irq - 64;
        regval32 = __bim_readl(REG_IRQEN3);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN3);
    }
	else if((irq >= 96) && (irq < 128))
    {
    	irq = irq - 96;
        regval32 = __bim_readl(REG_IRQEN4);
        regval32 |= (1 << irq);
        __bim_writel(regval32, REG_IRQEN4);
    }
    else
    {

    }
    //local_irq_restore(flags);
}
EXPORT_SYMBOL(ac83xx_unmask_bim_irq);


uint32_t ac83xx_pending_bim_irq(uint32_t irq)
{
	uint32_t regval32;
	if(irq < IRQ_BIM_GIC_OFFSET)
	{
		return 0;
	}
	else
	{
		irq -= IRQ_BIM_GIC_OFFSET;
	}
	
	if(irq < 32)
  {
    regval32 = __bim_readl(REG_IRQST);
    return (regval32 & (1 << irq));

  }
  else if((irq >= 32) && (irq < 64))
  {
	irq = irq - 32;
	regval32 = __bim_readl(REG_IRQST2);
	return (regval32 & (1 << irq));
  }
  else if((irq >= 64) && (irq < 96))
  {
    irq = irq - 64;
	regval32 = __bim_readl(REG_IRQST3);
    return (regval32 & (1 << irq));
  }
  else if((irq >= 96) && (irq < 128))
  {
    irq = irq - 96;
	regval32 = __bim_readl(REG_IRQST4);
    return (regval32 & (1 << irq));
  }

  return 0;
}

EXPORT_SYMBOL(ac83xx_pending_bim_irq);
