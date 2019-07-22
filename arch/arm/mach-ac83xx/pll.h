/*
 *  linux/arch/arm/mach-ac83xx/pll.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * pll settings.
 */

#ifndef _PLL_H_
#define _PLL_H_

extern void v_ac83xx_calibrate_plls(void); 
extern uint32_t u4_get_bus_clock(void);

#endif // _PLL_H_
