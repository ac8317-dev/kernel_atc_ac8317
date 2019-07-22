/*
 *  linux/arch/arm/mach-MT85XX/pll.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * pll settings.
 */

//==================================================
// header files
//==================================================
#include <mach/chip_ver.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <mach/ic_version.h>
//#include <x_typedef.h>


//==================================================
// Define
//==================================================


//==================================================
// Private functions
//==================================================

//==================================================
// public functions
//==================================================
IC_VERSION_T BSP_GetIcVersion(void)
{
  uint32_t u4Tmp;
  IC_VERSION_T eVer;

  u4Tmp = IO_READ32(IO_VIRT, 0x0);
  return eVer;

}
EXPORT_SYMBOL(BSP_GetIcVersion);

