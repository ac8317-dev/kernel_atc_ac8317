/*
 * arch/arm/mach-mt85xx/mt85xx_gpio_pinmux.c
 *
 * Copyright (C) 2010 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/spinlock.h> 
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include <mach/mt33xx.h>

#include <asm/bug.h>
#include <asm/bitops.h>
#include <asm/delay.h>

#include <mach/chip_ver.h>

#ifndef __KERNEL_GPIO_PINMUX__
#define __KERNEL_GPIO_PINMUX__
#endif
#include <mach/mt33xx_gpio_pinmux.h>
#include "mt33xx_pinmux_table.h"

#define REG_RW_TST_CFG0                0x005C
#define REG_RW_PINMUX_OFFSET	       0x054

#define GPIO_EN_WRITE(idx,val)          CKGEN_WRITE32((REG_RW_GPIO_EN_S+(4*(idx))), (val))
#define GPIO_OUT_WRITE(idx,val)         CKGEN_WRITE32((REG_RW_GPIO_OUT_S+(4*(idx))), (val))
#define GPIO_IN_WRITE(idx,val)          CKGEN_WRITE32((REG_RW_GPIO_IN_S+(4*(idx))), (val))
#define GPIO_OUT_REG(idx)               CKGEN_READ32(REG_RW_GPIO_OUT_S+(4*(idx)))
#define GPIO_EN_REG(idx)                CKGEN_READ32(REG_RW_GPIO_EN_S+(4*(idx)))
#define GPIO_IN_REG(idx)                CKGEN_READ32(REG_RW_GPIO_IN_S+(4*(idx)))

#define PINMUX_WRITE(idx,val)    CKGEN_WRITE32((REG_RW_PINMUX_OFFSET+(4*(idx))), (val))
#define PINMUX_REG(idx)          CKGEN_READ32(REG_RW_PINMUX_OFFSET+(4*(idx)))

//#define PAD_CFG_WRITE(idx, value)   	CKGEN_WRITE32((REG_RW_PAD_CFG_S + ((idx)*4)), (value))    
//#define PAD_CFG_REG(idx)				CKGEN_READ32((REG_RW_PAD_CFG_S + ((idx)*4)))

/* protects */
static spinlock_t gpio_lock = __SPIN_LOCK_UNLOCKED(gpio_lock);

/* only access gpiores with atomic ops */
static DECLARE_BITMAP(gpiores, TOTAL_GPIO_NUM + 1);

//static int __gpio_setup(unsigned gpio);

static inline int valid_gpio(unsigned gpio)
{
   return (gpio <= TOTAL_GPIO_NUM);
}

static inline int valid_pinmux(unsigned pinmux_sel, unsigned function)
{
   return   ((function >= 0) &&
		     (function <= MAX_PINMUX_FUNCTION) &&
    		 (function <= _au1PinmuxFunctionMasks[pinmux_sel]) &&
    		 (pinmux_sel >= 0) &&
    		 (pinmux_sel <= MAX_PINMUX_SEL) &&
    		 (_au1PinmuxFunctionMasks[pinmux_sel] != 0));
}

static inline unsigned gpio_get_addridx(unsigned gpio_pinmux)
{
    return (gpio_pinmux / 32);
}

static inline unsigned gpio_get_addroffset(unsigned gpio_pinmux)
{
    return (gpio_pinmux % 32);
}

static int __bsp_pinset(unsigned pinmux_sel, unsigned function)
{
	unsigned mask, offset, idx, val;
	
	if (likely(valid_pinmux(pinmux_sel, function)))
	{
		offset = gpio_get_addroffset((pinmux_sel));
		idx = gpio_get_addridx((pinmux_sel));
		mask = ~((unsigned)_au1PinmuxFunctionMasks[pinmux_sel] << offset);
		val = PINMUX_REG(idx);
		val &= mask;
		val |= ((unsigned)function << offset);
		PINMUX_WRITE(idx, val);

		return 0;
        } else
		return -EINVAL;	
}

#if 0
static int __bsp_pinrelease_for(unsigned pinmux_sel, unsigned function)
{
    unsigned gpio = 0;
	unsigned unum;
	
	if (likely(valid_pinmux(pinmux_sel, function)))
    {
	    while(gpio <= PIN_TCK)
		{
		  for (unum = 1; ((unum < (2*_au1GPIOConfig[gpio][0])) && (unum < 2*MAX_PINMUX_FUNCTION)); unum = unum + 2)
		  {
			if((_au1GPIOConfig[gpio][unum] == pinmux_sel)
				   && (_au1GPIOConfig[gpio][unum+1] == function))
			{
			   __gpio_setup(gpio);
			   break;
			}
		  }

          gpio ++;
	      
        }

		return 0;
	}
	else
		return -EINVAL;	
}
#endif

int bsp_pinset(unsigned pinmux_sel, unsigned function)
{
	unsigned long flags;
	int ret;
	
	spin_lock_irqsave(&gpio_lock, flags);
	//__bsp_pinrelease_for(pinmux_sel, function);
	ret = __bsp_pinset(pinmux_sel, function);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return ret;	
}
EXPORT_SYMBOL(bsp_pinset);

int bsp_pinget(unsigned pinmux_sel)
{
	unsigned mask, offset, idx, val;

	if (likely(valid_pinmux(pinmux_sel, 0)))
	{
		offset = gpio_get_addroffset((pinmux_sel));
		idx = gpio_get_addridx((pinmux_sel));
		mask = _au1PinmuxFunctionMasks[pinmux_sel];

		val = PINMUX_REG(idx) >> offset;
		val &= mask;

		return (int)val;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(bsp_pinget);


static int __gpio_get_value(unsigned gpio)
{
	unsigned val, idx, offset;

	if (likely(valid_gpio(gpio))) 
	{		
		if(gpio <= PIN_VSYNC_IN)
		{
			idx = gpio_get_addridx(gpio);
			offset = gpio_get_addroffset(gpio);

			val = GPIO_IN_REG(idx);

			return ((val & (1U << offset)) ? 1 : 0);
		}
		else
		{
			val = PDWNC_READ32(REG_RW_GPIOIN);
			return ((val & (1U << (gpio - PIN_SCL0_IN))) ? 1 : 0);
		}
	}
	else
	    return -EINVAL;
}

static int __gpio_set_value(unsigned gpio, int value)
{
	unsigned val, idx, offset;

	if (likely(valid_gpio(gpio)))
	{
		if(gpio <= PIN_VSYNC_IN)
		{
			offset = gpio_get_addroffset(gpio);
			idx = gpio_get_addridx(gpio);
			val = GPIO_OUT_REG(idx);
			val = (value == 1) ? (val | (1U << offset)) : (val & ~(1U << offset));
			GPIO_OUT_WRITE(idx, val);
		}
		else
		{
			val = PDWNC_READ32(REG_RW_GPIOOUT);
			val = (value == 1) ? (val | (1U << (gpio - PIN_SCL0_IN))) : (val & ~(1U << (gpio - PIN_SCL0_IN)));
			PDWNC_WRITE32(REG_RW_GPIOOUT, val);
		}

		return 0;
	}
	else
		return -EINVAL;
}

static int __gpio_inout_sel(unsigned gpio, int dir)
{       
	unsigned val, idx, offset;
	int ret = -EINVAL;

	if(gpio <= PIN_VSYNC_IN)
	{      
		offset = gpio_get_addroffset(gpio);
		idx = gpio_get_addridx(gpio);

		val = GPIO_EN_REG(idx);    
		val = (dir == OUTPUT) ? (val | (1U << offset)) : (val & ~(1U << offset));
		GPIO_EN_WRITE(idx, val);

		ret = 0;
	}
	else if(gpio <= TOTAL_GPIO_NUM)
	{
		val = PDWNC_READ32(REG_RW_GPIOEN);    
		val = (dir == OUTPUT) ? (val | (1U << (gpio - PIN_SCL0_IN))) : (val & ~(1U << (gpio - PIN_SCL0_IN)));
		PDWNC_WRITE32(REG_RW_GPIOEN, val);

		ret = 0;
	}

	return ret;
} 

int gpio_inout_sel(unsigned gpio, int dir)
{       
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&gpio_lock, flags);
	ret = __gpio_inout_sel(gpio, dir);
	spin_unlock_irqrestore(&gpio_lock, flags);

	return ret;
}  
EXPORT_SYMBOL(gpio_inout_sel);

int gpio_request(unsigned gpio, const char *label)
{
	if (likely(valid_gpio(gpio)))
		return test_and_set_bit(gpio, gpiores) ? -EBUSY : 0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(gpio_request);

void gpio_free(unsigned gpio)
{
	clear_bit(gpio, gpiores);
	return;
}
EXPORT_SYMBOL(gpio_free);

int gpio_direction_input(unsigned gpio)
{
	if (likely(valid_gpio(gpio))) {
		int ret = -EINVAL;
		unsigned long flags;

		spin_lock_irqsave(&gpio_lock, flags);
		ret = __gpio_inout_sel(gpio, INPUT);
		//ret |= __gpio_setup(gpio);
		spin_unlock_irqrestore(&gpio_lock, flags);

		if(ret == 0)
		{
			ret = __gpio_get_value(gpio);
		}

		return ret;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(gpio_direction_input);

int gpio_direction_output(unsigned gpio, int value)
{
	if (likely(valid_gpio(gpio))) {
		int ret = -EINVAL;
		unsigned long flags;

		spin_lock_irqsave(&gpio_lock, flags);
		ret = __gpio_set_value(gpio, value);
		ret |= __gpio_inout_sel(gpio, OUTPUT);
		//ret |= __gpio_setup(gpio);
		spin_unlock_irqrestore(&gpio_lock, flags);

		return ret;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(gpio_direction_output);

int gpio_configure(unsigned gpio, int dir, int value)
{
	if (dir == INPUT)
	{
		return gpio_direction_input(gpio);
	}
	else if(dir == OUTPUT)
	{
		return gpio_direction_output(gpio, value);
	}
	else
		return -EINVAL;
}
EXPORT_SYMBOL(gpio_configure);

int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}
EXPORT_SYMBOL(gpio_get_value);

void gpio_set_value(unsigned gpio, int value)
{
	unsigned long flags;
	spin_lock_irqsave(&gpio_lock, flags);
	__gpio_set_value(gpio, value);
	spin_unlock_irqrestore(&gpio_lock, flags);
}
EXPORT_SYMBOL(gpio_set_value);

/*----------------------------------------------------------------------------
 * Function: gpio_verify
 *---------------------------------------------------------------------------*/
void gpio_verify(unsigned refergpio, unsigned gpio)
{
    //test input
    gpio_configure(gpio, INPUT, 0);
	
	gpio_configure(refergpio, OUTPUT, 1);
	__delay(100);
	if(__gpio_get_value(gpio) == 1)
	{
	   printk("GPIO %d Input 1 OK!\n", gpio);
	}
	else
	{
	   printk("GPIO %d Input 1 Fail!!!!!!!!\n", gpio);
	}

	gpio_configure(refergpio, OUTPUT, 0);
	__delay(100);
	if(__gpio_get_value(gpio) == 0)
	{
	   printk("GPIO %d Input 0 OK!\n", gpio);
	}
	else
	{
	   printk("GPIO %d Input 0 Fail!!!!!!!!\n", gpio);
	}

	//test output
    gpio_configure(refergpio, INPUT, 0);
	
	gpio_configure(gpio, OUTPUT, 1);
	__delay(100);
	if(__gpio_get_value(refergpio) == 1)
	{
	   printk("GPIO %d Output 1 OK!\n", gpio);
	}
	else
	{
	   printk("GPIO %d Output 1 Fail!!!!!!!!\n", gpio);
	}

	gpio_configure(gpio, OUTPUT, 0);
	__delay(100);
	if(__gpio_get_value(refergpio) == 0)
	{
	   printk("GPIO %d Output 0 OK!\n", gpio);
	}
	else
	{
	   printk("GPIO %d Output 0 Fail!!!!!!!!\n", gpio);
	}
}
EXPORT_SYMBOL(gpio_verify);

#define GPIO_MAJOR      248

 /* 
  * linux kernel module file operation functions implement 
  */
 static int k_gpio_open(struct inode *inode, struct file *filp) 
 {
   //Print a message
   //printk("GPIO module open success\n");
 
   return 0;
 }
 
 static int k_gpio_ioctl(struct inode *inode, struct file *filp, unsigned cmd, unsigned long arg)
 {
   int temp, gpio, gpiotype, gpiovalue;
      
   if(get_user(temp, (int __user *)arg))
   {
     goto fail;
   }
   
   gpio = GPIO_NR(temp);
   gpiotype = GPIO_TYPE(temp);
   gpiovalue = GPIO_VALUE(temp);

   if(gpio > TOTAL_GPIO_NUM)
   {
     goto fail;
   }
	   
   switch(cmd)
   {
	 case CMD_GPIO_CONFIG:
	   if(((gpiotype != OUTPUT) && (gpiotype != INPUT))
   	     || ((gpiovalue != 1) && (gpiovalue != 0)))
       {
         goto fail;
       }
	   
	   gpio_configure(gpio, gpiotype, gpiovalue);
	   break;
	   
	 case CMD_GPIO_INOUT_SEL:
	   if((gpiotype != OUTPUT) && (gpiotype != INPUT))
       {
         goto fail;
       }
	   
	   gpio_inout_sel(gpio, gpiotype);
	   break;
	   
	 case CMD_GPIO_SET_DATA:
	   if((gpiovalue != 1) && (gpiovalue != 0))
       {
         goto fail;
       }
	   
	   gpio_set_value(gpio, gpiovalue);
	   break;
	   
	 case CMD_GPIO_GET_DATA:
       gpiovalue = __gpio_get_value(gpio);

	   if(put_user((GPIO_PACK(gpio,gpiotype,gpiovalue)), (int __user *)arg))
	   {
	   	 goto fail; 
	   }
	   break;
	   
	 default:
	   goto fail;
   }
	    
   return 0;

 fail:
   return -EINVAL;
 }
 
 static int k_gpio_release(struct inode *inode, struct file *filp)
 {
   //Print a message
   //printk("gpio module close success\n");
 
   return 0;
 }
 
 static const struct file_operations k_gpio_fops = 
 {
   .owner	= THIS_MODULE,
   .open	= k_gpio_open,
   .unlocked_ioctl	= k_gpio_ioctl,	  
   .release = k_gpio_release,
 };
 
 static struct cdev gpio_cdev;
 
 /* 
  * linux kernel module init & uninit functions
  */
 void __exit mt85xx_gpio_exit(void)
 {
	 //unregister_chrdev(DEV_MJR_K_GPIO, "gpio");
	 cdev_del(&gpio_cdev);
	 
	 //Print a message
	 printk("k_gpio module un-init success\n");
 }
__exitcall(mt85xx_gpio_exit);

 /*----------------------------------------------------------------------------
 * Function: GPIO_Init
 * Description:
 * 	 Initialize all GPIO pin.
 * Inputs:  
 * Outputs: 
 * Returns: 
 *---------------------------------------------------------------------------*/
int __init mt85xx_gpio_Init(void)
{   
    int ret = 0;

    if((PDWNC_READ32(REG_RW_GPIO_PDSTAT) & 0xFF) != 0)
    {
        printk("[Wake up] GPIO wake up!!\n");
        PDWNC_WRITE32(REG_RW_GPIO_PDSTCLR, 0x01); 
    }

	PDWNC_WRITE32(REG_RW_PDIO, PDWNC_READ32(REG_RW_PDIO) & ((unsigned)0x00FFFFFF)); 
    PDWNC_WRITE32(REG_RW_GPIO_WAKEN, PDWNC_READ32(REG_RW_GPIO_WAKEN) & ((unsigned)0xFFFFFF00));
      
    cdev_init(&gpio_cdev, &k_gpio_fops);
    gpio_cdev.owner = THIS_MODULE;
    ret = cdev_add(&gpio_cdev, MKDEV(GPIO_MAJOR, 0), 1);
    
    if(ret)
    {
        printk("Unable to register \"gpio\" misc device\n"); 
    }
    else
    {
        printk("k_gpio module init success\n");
    }
	
    return ret;
}
device_initcall(mt85xx_gpio_Init);


