/*
 * arch/arm/mach-mt85xx/include/mach/mt3360_pinmux_table.h
 *
 * Copyright (C) 2011 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */
#ifndef __MT3360_PINMUX_TABLE_H
#define __MT3360_PINMUX_TABLE_H

#include <linux/types.h>

static const uint8_t _au1PinmuxFunctionMasks[MAX_PINMUX_SEL] =
{
  3,  0,  0,  3,  0,  0,  0,  0,  3,  0, 
  0,  0,  3,  0,  0,  3,  0,  0,  2,  0,
  2,  0,  1,  0,  1,  1,  1,  1,  3,  0,
  0,  0,  

  0,  0,  0,  0,  3,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  3,  0,  0,  3,  0,
  0,  3,  0,  0,  1,  3,  0,  0,  3,  0,
  0,  0,

  3,  0,  0,  0,  0,  0,  1,  1,  1,  1,
  2,  0,  2,  0,  0,  0,  1,  1,  2,  0,
  2,  0,  1,  1,  1,  1,  3,  0,  0,  1,
  1,  1,

  1,  1,  0,  0,  0,  0,  0,  0,  2,  0,
  1,  0,  2,  0,  0,  0,  2,  0,  2,  0,
  0,  0,  2,  0,  2,  0,  2,  0,  2,  0,
  2,  0,

  2,  0,  2,  0,  2,  0,  2,  0,  2,  0,
  1,  1,  1,  0,  0,  0,  1,  0,  0,  0,
  0,  0,  0,  0,  2,  0,  0,  0,  0,  0,
  0,  0,

  2,  0,  2,  0,  2,  0,  2,  0,  3,  0,
  0,  0,  3,  0,  0,  0,  3,  0,  0,  3,
  0,  0,  3,  0,  0,  3,  0,  0,  3,  0,
  0,  0,

  3,  0,  0,  3,  0,  0,  3,  0,  0,  0,
  0,  2,  0,  2,  0,  0,  2,  0,  2,  0,
  2,  0,  0,  2,  0,  0,  3
};

#endif /* __MT3360_PINMUX_TABLE_H */

