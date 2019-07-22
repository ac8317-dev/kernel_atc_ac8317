/*
 *  linux/arch/arm/mach-MT85xx/cache_operation.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * cache opertaion.
 */

#ifndef _CACHE_OPERATION_H_
#define _CACHE_OPERATION_H_

//#include <x_typedef.h>

/*
 * BSP_FlushDCacheRange(uint32_t u4Start, uint32_t u4End)
 *
 *    Clean and Invalidate Data Cache Range
 *    - u4Start : virtual start address (inclusive)
 *    - u4Len : length
 */
extern void BSP_FlushDCacheRange(uint32_t u4Start, uint32_t u4Len);
extern void BSP_CleanDCacheRange(uint32_t u4Start, uint32_t u4Len);
extern void BSP_InvDCacheRange(uint32_t u4Start, uint32_t u4Len);
extern asmlinkage long sys_imb(void);
extern asmlinkage long sys_imb_range(unsigned long u4Start, unsigned long u4Len);

#endif // _CACHE_OPERATION_H_
