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
#include <mach/chip_ver.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <asm/io.h>

#include <mach/ac83xx_64b_timer.h>



//==================================================
// variables
//==================================================


//==================================================
// Define
//==================================================
#define SELECTED_TIMER_ID 0

typedef struct _t64b_reg
{
    UINT32 u4RegL;
	UINT32 u4RegH;
	UINT32 u4RegEn;
}T64b_REG;

T64b_REG _ar_t64b_reg[BIM_64b_TIMER_NUM]=
{
    {BIM_T64b_LO_0, BIM_T64b_HI_0, BIM_T64b_EN_0},

};


//==================================================
// Private functions
//==================================================
BOOL _fg_is_timer_enabled(UINT32 u4TimerId)
{
    /* error check */
	if(u4TimerId >= BIM_64b_TIMER_NUM)
	{
	    printk("[_v_bim_reset_64b_timer] ERROR. u4TimerId=%d\n",u4TimerId);
		return FALSE;
	}

	/* check enabled */
	if((__bim_readl(_ar_t64b_reg[u4TimerId].u4RegEn) & TMR_AUTOLD_EN(u4TimerId)) != 0)
	{
		return TRUE;
	}

	return FALSE;
}

int _i_bim_reset_64b_timer(UINT32 u4TimerId)
{
    T64b_REG* t64bReg;

    /* error check */
  if(u4TimerId == 0)
  {
  	  return 0;
  }
	if(u4TimerId >= BIM_64b_TIMER_NUM)
	{
	    printk("[_v_bim_reset_64b_timer] ERROR. u4TimerId=%d\n",u4TimerId);
		return -1;
	}

	  /* reset */
	  t64bReg = &(_ar_t64b_reg[u4TimerId]);
    __bim_writel(__bim_readl(t64bReg->u4RegEn)&(~(TMR_AUTOLD_EN(u4TimerId))),t64bReg->u4RegEn);
    __bim_writel(0,t64bReg->u4RegL);
    __bim_writel(0,t64bReg->u4RegH);
    
    __bim_writel(__bim_readl(t64bReg->u4RegEn)|(TMR_AUTOLD_EN(u4TimerId)),t64bReg->u4RegEn);
     

	return 0;
}

int _i_bim_get_64b_timer(UINT32 u4TimerId, T64b_RAW_TIME_T* pRawTime)
{
    T64b_REG* t64bReg;
	UINT32 u4H_pre, u4H_next, u4L;

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
 
	    u4H_pre  = __bim_readl(t64bReg->u4RegH);
		u4L      = __bim_readl(t64bReg->u4RegL);
	    u4H_next = __bim_readl(t64bReg->u4RegH);
 
	}while(u4H_pre != u4H_next);
 
	
    pRawTime->u4L = u4L;
    pRawTime->u4H = u4H_next;

	return 0;
}

unsigned long long _u8_hal_t64b_raw_to_tick(const T64b_RAW_TIME_T* pRawTime)
{
    unsigned long long u8Cycles = pRawTime->u4H;
 
    u8Cycles = u8Cycles << 32;
    u8Cycles = u8Cycles +pRawTime->u4L;
 
    return u8Cycles;
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

unsigned long long u8_hal_t64b_raw_to_ns(const T64b_RAW_TIME_T* pRawTime)
{
    unsigned long long tick;
    UINT32 c;

    tick= _u8_hal_t64b_raw_to_tick(pRawTime);
    c = 27000000;

    tick = tick * 1000000L;
 
    return (tick / (unsigned long long)c) * 1000L;
 
}

int i_hal_t64b_raw_to_time(const T64b_RAW_TIME_T* pRawTime, HAL_TIME_T* pTime)
{
    UINT64 u8Cycles;
	UINT32 _u4SysClock ;
	
 
    _u4SysClock = 27000000;
 

    if((pRawTime == NULL) || (pTime == NULL))
    {
        printk("[i_hal_t64b_raw_to_time] ERROR. null pointer. pRawTime=0x%08x, pTime=0x%08x\n", (UINT32)pRawTime, (UINT32)pTime);
		return -1;
    }

    u8Cycles = _u8_hal_t64b_raw_to_tick(pRawTime);
	if(_u4SysClock==0)
	{
	    printk("[i_hal_t64b_raw_to_time] ERROR. div 0 (_u4SysClock)\n");
		return -1;
	}

    pTime->u4Seconds = (UINT32)(u8Cycles / (UINT64)_u4SysClock);
    pTime->u4Micros = (UINT32)(((u8Cycles % (UINT64)_u4SysClock) * 1000000L) / _u4SysClock);

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
        printk("[i_hal_t64b_get_delta_raw_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (UINT32)pResult, (UINT32)pT0, (UINT32)pT1);
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
        printk("[i_hal_t64b_get_delta_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (UINT32)pResult, (UINT32)pT0, (UINT32)pT1);
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
        printk("[i_hal_t64b_add_time] ERROR. pResult=0x%08x, pT0=0x%08x, pT1=0x%08x\n", (UINT32)pResult, (UINT32)pT0, (UINT32)pT1);
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

