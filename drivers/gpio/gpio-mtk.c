/*
 * Copyright (c) 2009-2011 Skypine Co., Ltd.
 *
 * Skypine - GPIOlib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <mach/ac83xx_gpio_pinmux.h> 
#include <mach/ac83xx_pinmux_table.h>
#include <linux/gpio.h>


extern GPIO_MultiFun_Set(int gpioNum,int gpioFuncSel);
extern int gpio_cansleep(unsigned gpio);

#define MTK_GPIO_NAME "mtk_gpio"

#define MAX_GPIO_DEVS 1

#define MAX_GPIO_NUM (32*7-1)
//#define GPIO_NUM(bank, index)	(((bank)*(32)) + (index))

static int gpio_requested_array[32];
static int gpio_requested_num;

//extern  gpio_cansleep(unsigned gpio);

static int mtk_gpio_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int mtk_gpio_release(struct inode *inode, struct file *filp)
{
	return 0;
}


static long mtk_gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char szName[10];
	unsigned gpio = cmd;
	int value = arg;
	int i;

	printk("+++mtk_gpio_ioctl %d is called to %d, MAX_GPIO_NUM is %d\n", gpio, value, MAX_GPIO_NUM);
		
	if(value == -1)
	{
		int retval = __gpio_cansleep(gpio);
		if(retval == 0)
		{
			printk("get value %d, unsafe\n", gpio);
			return gpio_get_value_cansleep(gpio);
			
		}
		else
		{
			printk("get value %d, safe\n", gpio);
			return gpio_get_value(gpio);
		}
	}
	
	if (gpio > MAX_GPIO_NUM || (value != 0 && value != 1))
	{
		return -EINVAL;
	}

	sprintf(szName, "user gpio_%d", gpio);

	GPIO_MultiFun_Set(gpio,PINMUX_LEVEL_GPIO_END_FLAG);

	if (!gpio_request(gpio, szName))
	{
		if(gpio_requested_num >= 32)
		{
			printk("gpio array over number\n");
			return -EINVAL;
		}
		gpio_requested_array[gpio_requested_num] = gpio;
		gpio_requested_num++;
		gpio_direction_output(gpio, value);

             	printk("request gpio %d\n", gpio);

		return 0;
	}else
	{
		for (i = 0; i < gpio_requested_num; i++)
		{
			if (gpio_requested_array[i] == gpio)
			{
				printk("gpio %d already requested!\n", gpio);
				gpio_direction_output(gpio, value);
				return 0;
			}
		}
		
		gpio_free(gpio);
		if(gpio_requested_num >= 32)
		{
			printk("gpio array over number\n");
			return -EINVAL;
		}
		gpio_requested_array[gpio_requested_num] = gpio;
		gpio_requested_num++;
		gpio_direction_output(gpio, value);

             	printk("free gpio and request gpio %d\n", gpio);
		return 0;	
             //printk("gpio %d ioctrl failed!\n", gpio);
	     //return -EINVAL;
	}

	return 0;
}



static const struct file_operations mtk_gpio_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mtk_gpio_ioctl,
	.open = mtk_gpio_open,
	.release = mtk_gpio_release,
};

static dev_t mtk_gpio_devid;
static struct class *mtk_gpio_class;
static struct cdev mtk_gpio_cdev;

static int __init mtk_gpio_init_module(void)
{
	int retval = -ENOMEM;
	int gpio_major;

	mtk_gpio_class = class_create(THIS_MODULE, "mtk_gpio");
	if (IS_ERR(mtk_gpio_class)) {
		retval = PTR_ERR(mtk_gpio_class);
		printk(KERN_ERR "mtk_gpio: class_create failed\n");
		goto err_out1;
	}

	retval = alloc_chrdev_region(&mtk_gpio_devid, 0, MAX_GPIO_DEVS, MTK_GPIO_NAME);

	if (retval) {
		printk(KERN_ERR "mtk_gpio: alloc_chrdev_region failed\n");
		goto err_out2;
	}

	gpio_major = MAJOR(mtk_gpio_devid);

	cdev_init(&mtk_gpio_cdev, &mtk_gpio_fops);
	mtk_gpio_cdev.owner = THIS_MODULE;
	retval = kobject_set_name(&mtk_gpio_cdev.kobj, MTK_GPIO_NAME);
	if (retval)
		goto err_out3;

	retval = cdev_add(&mtk_gpio_cdev, MKDEV(gpio_major, 0), MAX_GPIO_DEVS);
	if (retval) {
		kobject_put(&mtk_gpio_cdev.kobj);
		goto err_out4;
	}

	device_create(mtk_gpio_class, NULL, MKDEV(gpio_major, 0), NULL, MTK_GPIO_NAME);

	gpio_requested_num = 0;

	printk("+++++++++++++++++mtk_gpio_init_module OK\n");
	
	return 0;

err_out4:
	cdev_del(&mtk_gpio_cdev);
err_out3:
	unregister_chrdev_region(MKDEV(gpio_major, 0), MAX_GPIO_DEVS);
err_out2:
	class_destroy(mtk_gpio_class);
err_out1:

	printk("---mtk_gpio_init_module OK\n");
	
	return retval;
}

static void __exit mtk_gpio_exit_module(void)
{
	cdev_del(&mtk_gpio_cdev);
	unregister_chrdev_region(MKDEV(mtk_gpio_devid, 0), MAX_GPIO_DEVS);
	class_destroy(mtk_gpio_class);
}

module_init(mtk_gpio_init_module);
module_exit(mtk_gpio_exit_module);

MODULE_AUTHOR("Lich<cli@skypine.cn>");
MODULE_DESCRIPTION("SiRF gpio driver for JNI");
MODULE_LICENSE("GPL");


