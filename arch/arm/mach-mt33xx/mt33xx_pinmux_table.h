/*
 * arch/arm/mach-mt85xx/include/mach/mt85xx_pinmux_table.h
 *
 * Copyright (C) 2010 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */
#ifndef __MT85XX_PINMUX_TABLE_H
#define __MT85XX_PINMUX_TABLE_H

#define PINMUX_FUNCTION0        0
#define PINMUX_FUNCTION1        1
#define PINMUX_FUNCTION2        2
#define PINMUX_FUNCTION3        3
#define PINMUX_FUNCTION4        4
#define PINMUX_FUNCTION5        5
#define PINMUX_FUNCTION6        6
#define PINMUX_FUNCTION7        7
#define PINMUX_FUNCTION8        8
#define PINMUX_FUNCTION9        9
#define PINMUX_FUNCTION10       10
#define PINMUX_FUNCTION11       11
#define PINMUX_FUNCTION12       12
#define PINMUX_FUNCTION13       13
#define PINMUX_FUNCTION14       14
#define PINMUX_FUNCTION15       15

#define MAX_PINMUX_FUNCTION     15

#ifdef __KERNEL_GPIO_PINMUX__

//#include <x_typedef.h>
#include <mach/chip_ver.h>

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include <mach/mt3360_gpio_pinmux.h>
    #include "mt3360_pinmux_table.h"
  #endif
#endif  /* __KERNEL_GPIO_PINMUX__ */

#ifdef __FASTLOGO_GPIO_PINMUX__

#include "x_typedef.h"
#include "chip_ver.h"

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include "mt3360_gpio_pinmux.h"
    #include "mt3360_pinmux_table.h"
  #endif
#endif  /* __FASTLOGO_GPIO_PINMUX__ */


#ifdef __UBOOT_GPIO_PINMUX__

#include <asm/arch/x_typedef.h>
#include <chip_ver.h>

  #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    #include <asm/arch/mt3360_gpio_pinmux.h>
    #include <asm/arch/mt3360_pinmux_table.h>
  #endif
#endif /* __UBOOT_GPIO_PINMUX__ */

#endif /* __MT85XX_PINMUX_TABLE_H */

