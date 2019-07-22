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

//==================================================
// header files
//==================================================
#include <config/arch/chip_ver.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <mach/mt85xx_64b_timer.h>
#include "pll.h"


//==================================================
// variables
//==================================================


//==================================================
// Define
//==================================================
#define SELECTED_TIMER_ID 0
typedef struct _t64b_reg
{
    uint32_t u4RegL;
	uint32_t u4RegH;
	uint32_t u4RegEn;
}T64b_REG;

T64b_REG _ar_t64b_reg[BIM_64b_TIMER_NUM]=
{
    {BIM_T64b_LO_0, BIM_T64b_HI_0, BIM_T64b_EN_0},
    {BIM_T64b_LO_1, BIM_T64b_HI_1, BIM_T64b_EN_1}
};

//==================================================
// Private functions
//==================================================
BOOL _fg_is_timer_enabled(uint32_t u4TimerId)
{
    /* error check */
	if(u4TimerId >= BIM_64b_TIMER_NUM)
	{
	    printk("[_v_bim_reset_64b_timer] ERROR. u4TimerId=%d\n",u4TimerId);
		return FALSE;
	}

	/* check enabled */
	if((BIM_READ32(_ar_t64b_reg[u4TimerId].u4RegEn) & 1) != 0)
	{
		return TRUE;
	}

	return FALSE;
}

int _i_bim_reset_64b_timer(uint32_t u4TimerId)
{
    T64b_REG* t64bReg;

    /* error check */
	if(u4TimerId >= BIM_64b_TIMER_NUM)
	{
	    printk("[_v_bim_reset_64b_timer] ERROR. u4TimerId=%d\n",u4TimerId);
		return -1;
	}

	/* reset */
	t64bReg = &(_ar_t64b_reg[u4TimerId]);
    BIM_WRITE32(t64bReg->u4RegEn, 0);
    BIM_WRITE32(t64bReg->u4RegL, 0);
    BIM_WRITE32(t64bReg->u4RegH, 0);
    BIM_WRITE32(t64bReg->u4RegEn, 1);

	return 0;
}

int _i_bim_get_64b_timer(uint32_t u4TimerId, T64b_RAW_TIME_T* pRawTime)
{
    T64b_REG* t64bReg;
	uint32_t u4H_pre, u4H_next, u4L;

    /* error check */
	if(u4TimerId >= BIM_64b_TIMER_NUM)
	{
	    printk("[_v_bim_reset_64b_timer] ERROR. u4TimerId=%d\n",u4TimerId);
		return -1;
	}

	/* get timer data */
    t64bReg = &(_ar_t64b_reg[u4TimerId]);
	do
	{
	    u4H_pre  = BIM_READ32(t64bReg->u4RegH);
		u4L      = BIM_READ32(t64bReg->u4RegL);
	    u4H_next = BIM_READ32(t64bReg->u4RegH);
	}while(u4H_pre != u4H_next);
	
    pRawTime->u4L = u4L;
    pRawTime->u4H = u4H_next;

	return 0;
}

//==================================================
// public functions
//==================================================
int i_hal_t64b_init(void)
{
    int ret;

	/* return ok if enabled */
    if(_fg_is_timer_enabled(SELECTED_TIMER_ID) == TRUE)
    {
        return 0;
    }
	
	/* reset timer */
    ret = _i_bim_reset_64b_timer(SELECTED_TIMER_ID);
	if(ret!=0)
	{
	    printk("[i_hal_t64b_init] ERROR. reset timer fail. ret=%d\n", ret);
		return ret;
	}

	return 0;
}

int i_hal_t64b_get_raw_time(T64b_RAW_TIME_T* pRawTime)
{
    return _i_bim_get_64b_timer(SELECTED_TIMER_ID, pRawTime);
}
EXPORT_SYMBOL(i_hal_t64b_get_raw_time);

int i_hal_t64b_raw_to_time(const T64b_RAW_TIME_T* pRawTime, HAL_TIME_T* pTime)
{
    UINT64 u8Cycles;
	uint32_t _u4SysClock = u4_get_bus_clock();
	
    if((pRawTime == NULL) || (pTime == NULL))
    {
        printk("[i_hal_t64b_raw_to_time] ERROR. null pointer. pRawTime=0x%08x, pTime=0x%08x\n", (uint32_t)pRawTime, (uint32_t)pTime);
		return -1;
    }

    u8Cycles = pRawTime->u4H;
    u8Cycles = u8Cycles << 31;
    u8Cycles = u8Cycles + (pRawTime->u4L & 0x7FFFFFFF);

	if(_u4SysClock==0)
	{
	    printk("[i_hal_t64b_raw_to_time] ERROR. div 0 (_u4SysClock)\n");
		return -1;
	}

    pTime->u4Seconds = (uint32_t)(u8Cycles / (UINT64)_u4SysClock);
    pTime->u4Micros = (uint32_t)(((u8Cycles % (UINT64)_u4SysClock) * 1000000L) / _u4SysClock);

	return 0;
}
EXPORT_SYMBOL(i_hal_t64b_raw_to_time);

int i_hal_t64b_get_time(HAL_TIME_T* pTime)
{
    T64b_RAW_TIME_T rt;
	int ret;

    ret = i_hal_t64b_get_raw_time(&rt);
    if(ret != 0)
    {
        return ret;
    }

	ret = i_hal_t64b_raw_to_time(&rt, pTime);
    if(ret != 0)
    {
        return ret;
    }

	return 0;
}
EXPORT_SYMBOL(i_hal_t64b_get_time);

int i_hal_t64b_get_delta_raw_time(T64b_RAW_TIME_T* pResult, const T64b_RAW_TIME_T* pT0,
    const T64b_RAW_TIME_T* pT1)
{
    UINT64 u8New;
    UINT64 u8Old;
    UINT64 u8Result;
  
    if((pResult==NULL)||(pT0==NULL)||(pT1==NULL))
    {
        printk("[i_hal_t64b_get_delta_raw_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (uint32_t)pResult, (uint32_t)pT0, (uint32_t)pT1);
		return -1;
    }

    u8New = ((UINT64)pT1->u4H << 32) | (pT1->u4L);
    u8Old = ((UINT64)pT0->u4H << 32) | (pT0->u4L);

    if(u8New > u8Old)
    {
        u8Result = u8New - u8Old;
    }
	else
	{
        u8Result = u8Old - u8New;
	}

	pResult->u4H = u8Result >> 32;
	pResult->u4L = u8Result & 0xFFFFFFFF;

	return 0;
}
EXPORT_SYMBOL(i_hal_t64b_get_delta_raw_time);

int i_hal_t64b_get_delta_time(HAL_TIME_T* pResult, HAL_TIME_T* pT0,
    HAL_TIME_T* pT1)
{
    HAL_TIME_T* pNewer;
    HAL_TIME_T* pOlder;
  
    if((pResult==NULL)||(pT0==NULL)||(pT1==NULL))
    {
        printk("[i_hal_t64b_get_delta_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (uint32_t)pResult, (uint32_t)pT0, (uint32_t)pT1);
		return -1;
    }

	if((pT0->u4Micros >= 1000000)||(pT1->u4Micros >= 1000000))
	{
        printk("[i_hal_t64b_get_delta_time] ERROR. pT0->u4Micros=0x%08x, pT1->u4Micros=0x%08x\n", pT1->u4Micros, pT1->u4Micros);
		return -1;
	}

    //Decide which one is newer
    if((pT0->u4Seconds > pT1->u4Seconds) || ((pT0->u4Seconds == pT1->u4Seconds) && (pT0->u4Micros > pT1->u4Micros)))
    {
        pNewer = pT0;
        pOlder = pT1;
    }
    else
    {
        pNewer = pT1;
        pOlder = pT0;
    }

    // count delta time
	pResult->u4Seconds = pNewer->u4Seconds - pOlder->u4Seconds;
	if(pNewer->u4Micros >= pOlder->u4Micros)
	{
	    pResult->u4Micros = pNewer->u4Micros - pOlder->u4Micros;
	}
	else
	{
	    pResult->u4Micros = 1000000 + pNewer->u4Micros - pOlder->u4Micros;
		pResult->u4Seconds--;
	}
	
    return 0;
}
EXPORT_SYMBOL(i_hal_t64b_get_delta_time);

int i_hal_t64b_cmp_raw_time(const T64b_RAW_TIME_T* pT0,const T64b_RAW_TIME_T* pT1)
{
    UINT64 u8T0;
    UINT64 u8T1;

	u8T0 = ((UINT64)pT0->u4H << 32) | (pT0->u4L);
	u8T1 = ((UINT64)pT1->u4H << 32) | (pT1->u4L);

	if(u8T0 > u8T1) return 1;
	if(u8T0 == u8T1) return 0;

	return -1;
}
EXPORT_SYMBOL(i_hal_t64b_cmp_raw_time);

int i_hal_t64b_cmp_time(const HAL_TIME_T* pT0,const HAL_TIME_T* pT1)
{
    if((pT0->u4Seconds > pT1->u4Seconds) || ((pT0->u4Seconds == pT1->u4Seconds) && (pT0->u4Micros > pT1->u4Micros)))
    	return 1;

	if((pT0->u4Seconds == pT1->u4Seconds) && (pT0->u4Micros == pT1->u4Micros))
		return 0;

	return -1;
}
EXPORT_SYMBOL(i_hal_t64b_cmp_time);

int i_hal_t64b_add_time(HAL_TIME_T* pResult, const HAL_TIME_T* pT0, const HAL_TIME_T* pT1)
{
    if((pResult==NULL)||(pT0==NULL)||(pT1==NULL))
    {
        printk("[i_hal_t64b_add_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (uint32_t)pResult, (uint32_t)pT0, (uint32_t)pT1);
		return -1;
    }
	
    pResult->u4Micros = pT0->u4Micros + pT1->u4Micros;
	if(pResult->u4Micros >= 1000000)
	{
	    pResult->u4Micros = pResult->u4Micros - 1000000;
		pResult->u4Seconds = pT0->u4Seconds + pT1->u4Seconds + 1;
	}
	else
	{
		pResult->u4Seconds = pT0->u4Seconds + pT1->u4Seconds;
	}

	return 0;
}
EXPORT_SYMBOL(i_hal_t64b_add_time);

