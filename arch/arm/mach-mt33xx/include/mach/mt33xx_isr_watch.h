/*
 *  linux/arch/arm/mach-MT85xx/mt85xx_isr_watch.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * mt85xx isr watch.
 */

#ifndef _MT85XX_ISR_WATCH_H_
#define _MT85XX_ISR_WATCH_H_

//#include <x_typedef.h>
#include <mach/mt33xx_64b_timer.h>

/* define & config */
#define CONFIG_MT85XX_ISR_WATCH 0

#define ISRW_EVT_START 0
#define ISRW_EVT_END   1

/* struct */
typedef struct
{
	uint32_t u4Count;
	HAL_TIME_T rTotalTime;
	HAL_TIME_T rMaxTime;
	T64b_RAW_TIME_T _rt_start;
	T64b_RAW_TIME_T _rt_end;
}MT85XX_ISR_OVERHEAD_T;


/* global functions */
extern void mt85xx_isr_watch_init(void);
extern void mt85xx_isrw_proc_init(void);
extern void mt85xx_isr_watch_trigger(uint32_t u4Vector, uint32_t event);

#endif // _MT85XX_ISR_WATCH_H_
