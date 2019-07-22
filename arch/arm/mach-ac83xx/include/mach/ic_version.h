/*
 *  linux/arch/arm/mach-ac83xx/include/mach/ic_version.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * pll settings.
 */

#ifndef _IC_VERSIO_H_
#define _IC_VERSIO_H_
#include <mach/chip_ver.h>

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_AC83XX)

typedef enum
{
  IC_83XX_VER_FPGA    = 0x0000,           // FPGA
  IC_83XX             = 0x3360,           //
  IC_83XX_VER_UNKNOWN = 0x7fffffff,       // Unknown version
} IC_VERSION_T;
#endif

extern IC_VERSION_T BSP_GetIcVersion(void);

#endif // _IC_VERSIO_H_
