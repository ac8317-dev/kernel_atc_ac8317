/*
 *  linux/arch/arm/mach-ac83xx/ac83xx_CS_watch.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * ac83xx Critical Section watch.
 */

//==================================================
// header files
//==================================================

#include <mach/ac83xx_CS_watch.h>
#include <linux/module.h>

#include <linux/module.h>
#include <linux/signal.h>
#include <linux/spinlock.h>
#include <linux/personality.h>
#include <linux/kallsyms.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kprobes.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/traps.h>
#include <asm/io.h>
#include <asm/ptrace.h>


#include <mach/ac83xx_64b_timer.h>
#if CONFIG_AC83XX_CS_WATCH
//==================================================
// Define
//==================================================
//============================================================================
// IRQ/FIQ related definitions
//============================================================================
// PSR bit definitions
#define PSR_IRQ_DISABLE             (1 << 7)
#define PSR_FIQ_DISABLE             (1 << 6)

/* struct */
typedef struct
{
	HAL_TIME_T t_MaxPeriod; 
	HAL_TIME_T t_Level;
	T64b_RAW_TIME_T rt_MaxPeriod;	
	T64b_RAW_TIME_T rt_start;
	T64b_RAW_TIME_T rt_end;	
	uint32_t regLR; 		
	BOOL _fg_CS_watch_on;
	BOOL _fg_CS_watch_dump;	
}AC83XX_CS_WATCH_T;

//==================================================
// Local variable
//==================================================

//==================================================
// Global variable
//==================================================
//static HAL_TIME_T t_MaxPeriod, t_Level={2,100}; // 2sec+100ms
//static T64b_RAW_TIME_T rt_start, rt_end, rt_MaxPeriod;
//static BOOL _fg_CS_watch_on = FALSE;
//static uint32_t regLR; 

static AC83XX_CS_WATCH_T _t_cs_watch;

//==================================================
// Private functions
//==================================================

//==================================================
// public functions
//==================================================

void ac83xx_CS_watch_init(void)
{
    _t_cs_watch.t_Level.u4Seconds= 0;
    _t_cs_watch.t_Level.u4Micros = 1000;
	_t_cs_watch._fg_CS_watch_on = 1;
}

void ac83xx_CS_watch_start(uint32_t u4Flag)
{
	if(_t_cs_watch._fg_CS_watch_on)
	{
		if((u4Flag & PSR_IRQ_DISABLE) != PSR_IRQ_DISABLE) 
		{
			/* get start raw time */
			if(i_hal_t64b_get_raw_time(&_t_cs_watch.rt_start))
			{
				printk("[ac83xx_CS_watch_start] Fail.\n");
			}
		}	
	}
}

void ac83xx_CS_watch_end(uint32_t u4Flag)
{
	T64b_RAW_TIME_T rt_delta;
	HAL_TIME_T t_delta;

	if(_t_cs_watch._fg_CS_watch_on)
	{
		if((u4Flag & PSR_IRQ_DISABLE) != PSR_IRQ_DISABLE) 
		{
			/* get end raw time */
			if(i_hal_t64b_get_raw_time(&_t_cs_watch.rt_end))
			{
				while(1);
			}
			/* check if start > end : error */
			if(i_hal_t64b_cmp_raw_time(&_t_cs_watch.rt_end, &_t_cs_watch.rt_start) < 0)
			{
				while(1);
			}
			/* get delta raw time */
			if(i_hal_t64b_get_delta_raw_time(&rt_delta, &_t_cs_watch.rt_start, &_t_cs_watch.rt_end))			
			{
				while(1);
			}	
			if(i_hal_t64b_cmp_raw_time(&rt_delta, &_t_cs_watch.rt_MaxPeriod) == 1)
			{
				_t_cs_watch.rt_MaxPeriod = rt_delta;
				/* get time */
				if(i_hal_t64b_raw_to_time(&rt_delta, &t_delta))				
				{
					while(1);
				}		
				
				if (i_hal_t64b_cmp_time(&t_delta, &_t_cs_watch.t_Level) == 1)
				{
					_t_cs_watch.t_MaxPeriod = t_delta;
					// Record LR
		  			_t_cs_watch.regLR = (void *)(__builtin_return_address(0));
					// here is a software breakpoint.
					_t_cs_watch._fg_CS_watch_dump = 1;
				    {
						unsigned int dscr;
						asm("mrc p14, 0, %0, c0, c1, 0" : "=r" (dscr) : : "cc");
						if (dscr & (1 << 14))
						{
							// here is a software breakpoint.
							asm("bkpt 0");
							
							// if you want to leave this break function, set PC to here.
						}
					}
				}
				/* set initial */
				_t_cs_watch.rt_start.u4L = 0;
				_t_cs_watch.rt_start.u4H = 0;
			}
		}		
	}		
}

void volatile ac83xx_CS_watch_dump(void)
{
	uint32_t regSP;
	
	if((_t_cs_watch._fg_CS_watch_dump))
	{
		_t_cs_watch._fg_CS_watch_dump = 0;
		__asm__ volatile(
		"   MOV %0, SP \n"
		:
		: "r" (regSP)
		);	
		die("==Critical Section too long==", regSP, 0);
	}
}
EXPORT_SYMBOL(ac83xx_CS_watch_start);
EXPORT_SYMBOL(ac83xx_CS_watch_end);
EXPORT_SYMBOL(ac83xx_CS_watch_dump);
#endif // CONFIG_AC83XX_CS_WATCH

