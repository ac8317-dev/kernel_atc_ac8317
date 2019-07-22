/*
 * arch/arm/mach-mt85xx/include/mach/mt85xx_gpio_pinmux.h
 *
 * Copyright (C) 2010 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */
#ifndef __MT85XX_GPIO_PINMUX_H
#define __MT85XX_GPIO_PINMUX_H

#ifdef __KERNEL_GPIO_PINMUX__

//#include <x_typedef.h>
#include <mach/chip_ver.h>

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include <mach/mt3360_gpio_pinmux.h>
  #endif
#endif  /* __KERNEL_GPIO_PINMUX__ */

#if (defined(__FASTLOGO_GPIO_PINMUX__) || defined(__GENERIC_GPIO_PINMUX__))

#include "x_typedef.h"
#include "chip_ver.h"

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include "mt3360_gpio_pinmux.h"
  #endif
#endif  /* (defined(__FASTLOGO_GPIO_PINMUX__) || defined(__GENERIC_GPIO_PINMUX__)) */


#ifdef __UBOOT_GPIO_PINMUX__

#include <chip_ver.h>

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include <asm/arch/mt3360_gpio_pinmux.h>
  #endif
#endif /* __UBOOT_GPIO_PINMUX__ */

#define OUTPUT        1
#define INPUT         0

#if (defined(__KERNEL_GPIO_PINMUX__) || defined(__GENERIC_GPIO_PINMUX__))
#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#define GPIO_IO_MAGIC   'g'

/* GPIO ioctl command*/
#define CMD_GPIO_CONFIG    _IOW(GPIO_IO_MAGIC, 1, int)
#define CMD_GPIO_INOUT_SEL _IOW(GPIO_IO_MAGIC, 2, int)
#define CMD_GPIO_SET_DATA  _IOW(GPIO_IO_MAGIC, 3, int)
#define CMD_GPIO_GET_DATA  _IOR(GPIO_IO_MAGIC, 4, int)

/* GPIO data format*/
#define GPIO_NR_BITS     16
#define GPIO_TYPE_BITS   8
#define GPIO_VALUE_BITS  8

#define GPIO_NR_MASK    ((1 << GPIO_NR_BITS)-1)
#define GPIO_TYPE_MASK  ((1 << GPIO_TYPE_BITS)-1)
#define GPIO_VALUE_MASK ((1 << GPIO_VALUE_BITS)-1)

#define GPIO_NR_SHIFT    0
#define GPIO_TYPE_SHIFT  (GPIO_NR_SHIFT+GPIO_NR_BITS)
#define GPIO_VALUE_SHIFT (GPIO_TYPE_SHIFT+GPIO_TYPE_BITS)

#define GPIO_PACK(nr,type,value) \
    (((nr)    << GPIO_NR_SHIFT) | \
     ((type)  << GPIO_TYPE_SHIFT) | \
     ((value) << GPIO_VALUE_SHIFT))

#define GPIO_TYPE(nr)       (((nr) >> GPIO_TYPE_SHIFT) & GPIO_TYPE_MASK)
#define GPIO_NR(nr)         (((nr) >> GPIO_NR_SHIFT) & GPIO_NR_MASK)
#define GPIO_VALUE(nr)      (((nr) >> GPIO_VALUE_SHIFT) & GPIO_VALUE_MASK)

#include <asm/errno.h>

int bsp_pinset(unsigned pinmux_sel, unsigned function);
int bsp_pinget(unsigned pinmux_sel);

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);

int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);

int gpio_configure(unsigned gpio, int dir, int value);
int gpio_inout_sel(unsigned gpio, int dir);

int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);

void gpio_table_check(void);
void gpio_verify(unsigned refergpio, unsigned gpio);
void gpio_related(unsigned gpio, int showall);

#ifdef __KERNEL_GPIO_PINMUX__
static inline int gpio_to_irq(int pin)
{
    return -EINVAL;
}

static inline int irq_to_gpio(int irq)
{
    return -EINVAL;
}

/* get the cansleep() stubs */
//#include <asm-generic/gpio.h>
#endif /* __KERNEL_GPIO_PINMUX__ */


#endif /* (defined(__KERNEL_GPIO_PINMUX__) || defined(__GENERIC_GPIO_PINMUX__)) */

#endif /* __MT85XX_GPIO_PINMUX_H */

