/*
 *  
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * 
 */

#ifndef _MT85XX_64B_TIMER_H_
#define _MT85XX_64B_TIMER_H_

//#include <x_typedef.h>
#include <linux/types.h>

//==================================================
// define
//==================================================
#define BIM_64b_TIMER_NUM 2
#define BIM_T64b_LO_0 0x728
#define BIM_T64b_HI_0 0x72C
#define BIM_T64b_EN_0 0x730
#define BIM_T64b_LO_1 0x734
#define BIM_T64b_HI_1 0x738
#define BIM_T64b_EN_1 0x73C

typedef struct
{
  uint32_t    u4L;                    //Number of timer interrupts from startup
  uint32_t    u4H;                   //System cycles from last timer interrupt
} T64b_RAW_TIME_T;

typedef struct
{
  uint32_t    u4Seconds;                  //Number of seconds from startup
  uint32_t    u4Micros;                   //Remainder in microsecond
} HAL_TIME_T;


//==================================================
// public functions
//==================================================
extern int i_hal_t64b_init(void);
extern int i_hal_t64b_get_raw_time(T64b_RAW_TIME_T* pRawTime);
extern int i_hal_t64b_raw_to_time(const T64b_RAW_TIME_T* pRawTime, HAL_TIME_T* pTime);
extern int i_hal_t64b_get_time(HAL_TIME_T* pTime);
extern int i_hal_t64b_get_delta_raw_time(T64b_RAW_TIME_T* pResult, const T64b_RAW_TIME_T* pT0, const T64b_RAW_TIME_T* pT1);
extern int i_hal_t64b_get_delta_time(HAL_TIME_T* pResult, HAL_TIME_T* pT0, HAL_TIME_T* pT1);
/*
 * return value:
 *		0  -- pT0 == pT1
 *		1  -- pT0 >  pT1
 *		-1 -- pT0 <  pT1
 */
extern int i_hal_t64b_cmp_raw_time(const T64b_RAW_TIME_T* pT0,const T64b_RAW_TIME_T* pT1);
extern int i_hal_t64b_cmp_time(const HAL_TIME_T* pT0,const HAL_TIME_T* pT1);
extern int i_hal_t64b_add_time(HAL_TIME_T* pResult, const HAL_TIME_T* pT0, const HAL_TIME_T* pT1);

#endif // _MT85XX_64B_TIMER_H_
