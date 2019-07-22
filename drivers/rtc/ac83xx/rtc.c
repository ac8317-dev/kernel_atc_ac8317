/*
 *	Real Time Clock interface for Linux on Atmel AT91RM9200
 *
 *	Copyright (C) 2002 Rick Bronson
 *
 *	Converted to RTC class model by Andrew Victor
 *
 *	Ported to Linux 2.6 by Steven Scholz
 *	Based on s3c2410-rtc.c Simtec Electronics
 *
 *	Based on sa1100-rtc.c by Nils Faerber
 *	Based on rtc.c by Paul Gortmaker
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>

#include <asm/uaccess.h>
#include <linux/spinlock_types.h>
#include "rtc_hal.h"


int enable_output = 0;
int log_time = 0;
extern int g_bInitOK;
extern int g_bFirstBooting;

//#define AT91_RTC_EPOCH		1900UL	/* just like arch/arm/common/rtctime.c */

//static DECLARE_COMPLETION(at91_rtc_updated);
//static unsigned int at91_alarm_year = AT91_RTC_EPOCH;

//------------------------------------------------------------------------------
// Defines
#define RTC_YEAR_DATUM (uint16_t)(2000)
#define RTC_YEAR_END (uint16_t)(2127)

spinlock_t ac83xx_rtc_lock; 

#if 0
//-----------------------------------------------------------------------------
//
// Function:  IsLeapYear
//
// This function check whether the year is leap.
//
//
bool IsLeapYear(
    uint16_t Year
    )
{
    bool bLeap = FALSE;

    if ((Year % 4) == 0)
    {
        bLeap = TRUE;

        if ((Year % 100 == 0))
        {
            bLeap = FALSE;

            if ((Year % 400) == 0)
            {
                bLeap = TRUE;
            }
        }
    }

    return bLeap;
}
#endif

//-----------------------------------------------------------------------------
//
// Function:  VerifyDateTime
//
// This function verify real timer data
//
//
bool VerifyDateTime(struct rtc_time *tm)
{
#if 0
    bool rc =TRUE;
    uint16_t monthDays;

    if (pTime == NULL)
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid argument\r\n"));
        rc = FALSE;
        goto cleanUp;
    }

    // The RTC will only support a BCD year value of 0 - 127.  The year datum is
    // 2000, so any dates greater than 2127 will fail unless the datum is
    // adjusted.
    if (pTime->wYear < RTC_YEAR_DATUM || pTime->wYear > RTC_YEAR_END)
    {
        OALMSG(OAL_RTC &&OAL_ERROR, ( 
            L"RTC cannot support a year greater than %d or less than %d (value %d)\r\n",
            RTC_YEAR_END, RTC_YEAR_DATUM, pTime->wYear));    
        rc = FALSE;
        goto cleanUp;
    }

    if ((pTime->wMonth > 12) || (pTime->wMonth == 0))
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Month : %d\r\n", pTime->wMonth));
        rc = FALSE;
        goto cleanUp;
    }

    if ((pTime->wDay == 0))
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Day of Month : %d\r\n", pTime->wMonth));
        rc = FALSE;
        goto cleanUp;
    }
    
    if (pTime->wHour >= 24)
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Hour : %d\r\n", pTime->wHour));
        rc = FALSE;
        goto cleanUp;
    }
    if (pTime->wMinute >= 60)
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Minutes : %d\r\n", pTime->wMinute));
        rc = FALSE;
        goto cleanUp;
    }

    if (pTime->wSecond >= 60)
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Seconds : %d\r\n", pTime->wSecond));
        rc = FALSE;
        goto cleanUp;
    }

    if (pTime->wDayOfWeek >= 7)
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Day of Week : %d\r\n", pTime->wDayOfWeek ));
        rc = FALSE;
        goto cleanUp;
    }

    if ((pTime->wMonth == 4) || (pTime->wMonth == 6) || (pTime->wMonth == 9) || (pTime->wMonth == 11))
    {
        monthDays = 30;
    }
    else if (pTime->wMonth == 2)
    {
        monthDays = (IsLeapYear(pTime->wYear)) ? 29:28;
    }
    else
    {
        monthDays = 31;
    }

    if ((pTime->wDay > monthDays))
    {
        OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid Day of Month : %d\r\n", pTime->wDay));
        rc = FALSE;
        goto cleanUp;
    }
#else
  bool rc =TRUE;
  int32_t MaxDayOfMonth;
  int32_t DayOfMonthArray[12] = {31,0,31,30,31,30,31,31,30,31,30,31};

  	if (tm == NULL)
	{
	    //OALMSG(OAL_RTC && OAL_ERROR, (L"Invalid argument\r\n"));
	    printk("Invalid argument\r\n");
	    rc = FALSE;
	    goto cleanUp;
	}
  

    if(((tm->tm_year + 1900)< 2000) || ((tm->tm_year + 1900)> 2127))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 1 : Year [2000-2127]\n"));
		printk("Parameter 1 : Year [2000-2127]\n");
		rc = FALSE;
        goto cleanUp;
	}

	if(((tm->tm_mon + 1)< 1) || ((tm->tm_mon + 1)> 12))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 2 : Month [1-12]\n"));
		printk("Parameter 2 : Month [1-12]\n");
		rc = FALSE;
    goto cleanUp;
	}

	if((tm->tm_wday< 0) || (tm->tm_wday >= 7))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 3 : Day of Week [1-7]\n"));
		printk("Parameter 3 : Day of Week [1-7]\n");
		rc = FALSE;
    goto    cleanUp;
	}

	if((((tm->tm_year + 1900) % 400) == 0) || ((((tm->tm_year + 1900) % 4) == 0) && (((tm->tm_year + 1900)% 100) != 0)))
	{
		DayOfMonthArray[1] = 29;	 
	}
	else
	{
		DayOfMonthArray[1] = 28;	
	}

	MaxDayOfMonth = DayOfMonthArray[tm->tm_mon];
	
	if((tm->tm_mday < 1) || (tm->tm_mday > MaxDayOfMonth))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"The Max day of %d-%d is %d\n",pTime->wYear,pTime->wMonth,MaxDayOfMonth));		
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 4 : Day of Mon [1-%d]\n",MaxDayOfMonth));
		printk("The Max day of %d-%d is %d\n",(tm->tm_year + 1900),(tm->tm_mon +1),MaxDayOfMonth);
        printk("Parameter 4 : Day of Mon [1-%d]\n",MaxDayOfMonth);
		rc = FALSE;;
    goto cleanUp;
	}

	
	if((tm->tm_hour < 0) || (tm->tm_hour > 23))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 5 : Hour [0-23]\n"));
		printk("Parameter 5 : Hour [0-23]\n");
		rc = FALSE;
    goto cleanUp;
	}


	
	if((tm->tm_min < 0) || (tm->tm_min) > 59)
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 6 : Min [0-59]\n"));
		printk("Parameter 6 : Min [0-59]\n");
		rc = FALSE;
    goto cleanUp;
	}


	
	if((tm->tm_sec < 0) || (tm->tm_sec > 59))
	{
		//OALMSG(OAL_RTC && OAL_ERROR, (L"Parameter 7 : Sec [0-59]\n"));
		printk("Parameter 7 : Sec [0-59]\n");
		rc = FALSE;
    goto cleanUp;
	}

#endif
    rc = TRUE;

cleanUp:
    //OALMSG(OAL_RTC && OAL_FUNC, (L"VerifyDateTime(rc = %d)\r\n", rc));
    printk("VerifyDateTime(rc = %d)\r\n", rc);
    return rc;
}


#if 0
//------------------------------------------------------------------------------
//
//  Function:  OALIoCtlHalInitRTC
//
//  This function is called by WinCE OS to initialize the time after boot.
//  Input buffer contains SYSTEMTIME structure with default time value.
//  If hardware has persistent real time clock it will ignore this value
//  (or all call).
//
bool OALIoCtlHalInitRTC(
    Uint32_t code, VOID *pInpBuffer, Uint32_t inpSize, VOID *pOutBuffer, 
    Uint32_t outSize, Uint32_t *pOutSize)
    {
    bool rc = FALSE;
    SYSTEMTIME *pTime = (SYSTEMTIME*)pInpBuffer;

    OALMSG(OAL_IOCTL&&OAL_FUNC, (L"+OALIoCtlHalInitRTC(...)\r\n"));

    if (pOutSize) {
        *pOutSize = 0;
    }

    // Validate inputs
    if (pInpBuffer == NULL || inpSize < sizeof(SYSTEMTIME)) {
        NKSetLastError(ERROR_INVALID_PARAMETER);
        OALMSG(OAL_ERROR, (
            L"ERROR: OALIoCtlHalInitRTC: Invalid parameter\r\n"
        ));
        goto cleanUp;
    }

    // Add static mapping for RTC alarm
    OALIntrStaticTranslate(SYSINTR_RTC_ALARM, VECTOR_RTC);

    // Befoer setting the real time, RTC_Init must be called
    if ((rc = RTCHWInit()) == FALSE)
    {
        OALMSG(OAL_ERROR, (
            L"ERROR: OALIoCtlHalRTC_Init: Invalid RTC init sequence\r\n"
        ));
        goto cleanUp;
    }
    
    // Set real time
    #if 0
    if (g_bFirstBooting == TRUE)
    {
        rc = OEMSetRealTime(pTime);
    }
    #endif
    
cleanUp:
    OALMSG(OAL_IOCTL&&OAL_FUNC, (L"-OALIoCtlHalInitRTC(rc = %d)\r\n", rc));
    return rc;
}
#endif

static int ac83xx_rtc_output_32k(void)
{
	int rc = -1;
	unsigned int out_32k = 0;

	out_32k = RTC_IO_READ32(0x64);
	out_32k = out_32k | 0x400;
	printk("ac83xx_rtc_output_32k, out_32k [0x%x]\r\n", out_32k);
	RTC_IO_WRITE32(0x64, out_32k);

	out_32k = RTC_IO_READ32(0x60);
	printk("ac83xx_rtc_output_32k, reg(0x60) = [0x%x]\r\n", out_32k);

	rc = 0;

	return rc;
}


//------------------------------------------------------------------------------
//
//  Function:  ac83xx_rtc_readtime
//
//  Reads the current RTC value and returns a system time.
//
static int ac83xx_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
    int rc = -1;

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		tm->tm_sec = 0x00;
		tm->tm_min = 0x00;
		tm->tm_hour = 0x00;
		tm->tm_mday = 0x01;
		tm->tm_mon  = 0;
		tm->tm_wday = 6;
		tm->tm_year = 0;
		tm->tm_year = tm->tm_year + RTC_YEAR_DATUM - 1900;
		tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
		
		printk("!ac83xx_rtc_readtime: %4d-%02d-%02d %02d:%02d:%02d\n",
			1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
		return 0;
	}
	#endif
		
    if (tm == NULL) goto cleanUp;
	//printk(KERN_INFO "+ac83xx_rtc_readtime into tm ->1.\n");
	//spin_lock_irq(&ac83xx_rtc_lock);
    if (RTCHWInit() == FALSE) goto cleanUp;
    //printk(KERN_INFO "+ac83xx_rtc_readtime into tm ->2.\n");
    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    RTCTOCORE();
	//printk("+++++++ ac83xx_rtc_readtime, rtc to core !!!!!\n");
	
    tm->tm_sec = RTCSecRead();
	tm->tm_sec = tm->tm_sec & 0x3F;
	
    tm->tm_min = RTCMinRead();
	tm->tm_min = tm->tm_min & 0x3F;
	
    tm->tm_hour = RTCHourRead();
	tm->tm_hour = tm->tm_hour & 0x1F;
	
    tm->tm_mday = RTCDOMRead();
	tm->tm_mday = tm->tm_mday & 0x1F;
	
    tm->tm_mon  = RTCMonthRead();
	tm->tm_mon = tm->tm_mon & 0x0F;
	tm->tm_mon = tm->tm_mon - 1;
	
    // Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.    
    tm->tm_wday = RTCDOWRead();
	tm->tm_wday = tm->tm_wday & 0x07;
	tm->tm_wday = tm->tm_wday % 7;
	
    tm->tm_year = RTCYearRead();
	tm->tm_year = tm->tm_year & 0x7F;
	tm->tm_year = tm->tm_year + RTC_YEAR_DATUM - 1900;

	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	//printk(KERN_INFO "+ac83xx_rtc_readtime into tm ->4.\n");
    //INTERRUPTS_ENABLE(enabled);
    //spin_unlock_irq(&ac83xx_rtc_lock);
	//printk(KERN_INFO "+ac83xx_rtc_readtime into tm ->5.\n");
    //pTime->wMilliseconds = 0;
    
    printk("ac83xx_rtc_readtime: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    // Done
    rc = 0;
    
cleanUp:   
    //printk("-ac83xx_rtc_readtime(rc = %d)\r\n", rc);
    return rc;
}


//------------------------------------------------------------------------------
//
//  Function:  ac83xx_rtc_settime
//
//  Updates the RTC with the specified system time.
//
static int ac83xx_rtc_settime(struct device *dev, struct rtc_time *tm)
{
    int rc = -1;

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		return 0;
	}
	#endif
	
    printk("ac83xx_rtc_settime: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    if (tm == NULL) goto cleanUp;

	#if 0
    if (!VerifyDateTime(tm)) goto cleanUp;
	#else
	if (!VerifyDateTime(tm)){
		printk("RTC time is not right!! \n");
		return 0;
	}
	#endif
	
    //spin_lock_irq(&ac83xx_rtc_lock);
    if (RTCHWInit() == FALSE) goto cleanUp;
    
    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    RTCProtect(0);

    RTCSecSet(tm->tm_sec);
    RTCMinSet(tm->tm_min);
    RTCHourSet(tm->tm_hour);
    RTCDOMSet(tm->tm_mday);
    RTCMonthSet(tm->tm_mon + 1);
    // Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.        
    RTCDOWSet((tm->tm_wday == 0) ? 7 : tm->tm_wday);
    RTCYearSet(tm->tm_year + 1900 - RTC_YEAR_DATUM);

    RTCProtect(1);

    //INTERRUPTS_ENABLE(enabled);
    //spin_unlock_irq(&ac83xx_rtc_lock);
    
    // Done
    rc = 0;
    
cleanUp:
    //printk("-ac83xx_rtc_settime(rc = %d)\r\n", rc);
    return rc;
}


//------------------------------------------------------------------------------
//
//  Function:  ac83xx_rtc_setalarm
//
//  Set the RTC alarm time.
//
static int ac83xx_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    int rc = -1;
	struct rtc_time *tm = &alrm->time;

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		return 0;
	}
	#endif
	
    if(!tm) goto cleanUp;
    
    printk("ac83xx_rtc_setalarm: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	#if 0
    if (!VerifyDateTime(tm)) goto cleanUp;
	#else
	if (!VerifyDateTime(tm)){
		printk("RTC time is not right!! \n");
		return 0;
	}
	#endif

	//spin_lock_irq(&dev->devres_lock);
    if (RTCHWInit() == FALSE) goto cleanUp;

    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    RTCProtect(0);

    // 1. Set the alarm time register
    RTCAlarmSecSet(tm->tm_sec);
    RTCAlarmMinSet(tm->tm_min);
    RTCAlarmHourSet(tm->tm_hour);
    RTCAlarmDOMSet(tm->tm_mday);
    RTCAlarmMonthSet(tm->tm_mon+1);
    // Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.    
    RTCAlarmDOWSet((tm->tm_wday == 0) ? 7 : tm->tm_mday);
    RTCAlarmYearSet(tm->tm_year + 1900 - RTC_YEAR_DATUM);

    // 2. Clear ALARM_HIT
    //RTC_READ32(RTC_IRQ_STA);  //oeminterrupter clear

    // 3. Enable alarm controller
    if (alrm->enabled) {
		printk("ac83xx_rtc_setalarm: enabled alarm interrupt.\n");
		RTCAlarmAllCMPR(1);
		RTCEnableAlarmInt(1);
	    RTCIntAutoReset(1);	
	}
	
    RTCProtect(1);

    //INTERRUPTS_ENABLE(enabled);
    //spin_unlock_irq(&dev->devres_lock);
 
    // Done
    rc = 0;
    
cleanUp:
    //OALMSG(OAL_RTC&&OAL_FUNC, (L"-OEMSetAlarmTime(rc = %d)\r\n", rc));
    //printk("-ac83xx_rtc_setalarm(rc = %d)\r\n", rc);

    return rc;
}

//------------------------------------------------------------------------------

/*
 * Read alarm time and date in RTC
 */
static int ac83xx_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    int rc = -1;
	struct rtc_time *tm = &alrm->time;

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		tm->tm_sec = 0x00;
		tm->tm_min = 0x00;
		tm->tm_hour = 0x00;
		tm->tm_mday = 0x01;
		tm->tm_mon  = 0;
		tm->tm_wday = 6;
		tm->tm_year = 0;
		tm->tm_year = tm->tm_year + RTC_YEAR_DATUM - 1900;

		tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
		
		printk("!ac83xx_rtc_readalarm: %4d-%02d-%02d %02d:%02d:%02d\n",
			1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);
		return 0;
	}
	#endif

	if(!tm) goto cleanUp;
	
	//spin_lock_irq(&dev->devres_lock);
	if (RTCHWInit() == FALSE) goto cleanUp;
	// To avoid the interrupt to affect the rtc time
	//enabled = INTERRUPTS_ENABLE(FALSE);
	RTCProtect(0);

	// 1. Set the alarm time register
	tm->tm_sec = RTCAlarmSecRead();
	tm->tm_sec = tm->tm_sec & 0x3F;
	
	tm->tm_min = RTCAlarmMinRead();
	tm->tm_min = tm->tm_min & 0x3F;
	
	tm->tm_hour = RTCAlarmHourRead();
	tm->tm_hour = tm->tm_hour & 0x1F;
	
	tm->tm_mday = RTCAlarmDOMRead();
	tm->tm_mday = tm->tm_mday & 0x1F;
	
	tm->tm_mon = RTCAlarmMonthRead();
	tm->tm_mon = tm->tm_mon & 0x0F;
	tm->tm_mon = tm->tm_mon - 1;
	
	// Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.
	tm->tm_wday = RTCAlarmDOWRead();
	tm->tm_wday = tm->tm_wday & 0x07;
	tm->tm_wday = tm->tm_wday % 7;
	
    tm->tm_year = RTCAlarmYearRead();
	tm->tm_year = tm->tm_year & 0x7F;
	tm->tm_year = tm->tm_year + RTC_YEAR_DATUM - 1900;
	
    	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	alrm->enabled = (RTC_READ32(RTC_IRQ_STA) & ALSTA) ? 1 : 0;
	RTCProtect(1);
	//spin_unlock_irq(&dev->devres_lock);

	printk("ac83xx_rtc_readalarm: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    rc = 0;
    
cleanUp:
	//OALMSG(OAL_RTC&&OAL_FUNC, (L"-OEMSetAlarmTime(rc = %d)\r\n", rc));
	//printk("-ac83xx_rtc_readalarm(rc = %d)\r\n", rc);

	return rc;
}

/*
 * Handle commands from user-space
 */
static int ac83xx_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;

	printk("ac83xx_rtc_ioctl(): cmd=%08x, arg=%08lx.\n", cmd, arg);

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		return 0;
	}
	#endif

	/* important:  scrub old status before enabling IRQs */
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		spin_lock_irq(&ac83xx_rtc_lock);
		RTCEnableAlarmInt(0);
		spin_unlock_irq(&ac83xx_rtc_lock);
		break;
	case RTC_AIE_ON:	/* alarm on */
		spin_lock_irq(&ac83xx_rtc_lock);
        RTCEnableAlarmInt(1);
		spin_unlock_irq(&ac83xx_rtc_lock);
		break;
	case RTC_UIE_OFF:	/* update off */
		break;
	case RTC_UIE_ON:	/* update on */
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static const struct rtc_class_ops ac83xx_rtc_ops = {
	.ioctl		= ac83xx_rtc_ioctl,
	.read_time	= ac83xx_rtc_readtime,
	.set_time	= ac83xx_rtc_settime,
	.read_alarm	= ac83xx_rtc_readalarm,
	.set_alarm	= ac83xx_rtc_setalarm,
//	.proc		= at91_rtc_proc,
};


/*
 * IRQ handler for the RTC
 */
extern ac83xx_mask_ack_bim_irq(uint32_t irq);
static irqreturn_t ac83xx_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr;
	unsigned long events = 0;

	#if 0
	if (g_bInitOK == FALSE) {
		printk("RTC is not init OK !! \n");
		return 0;
	}
	#endif

	rtsr = RTC_READ32(RTC_IRQ_STA);
	if (rtsr) {		/* this interrupt is shared!  Is it ours? */
		RTCProtect(0);
		RTCClearIrqSta();
		ac83xx_mask_ack_bim_irq(irq);
		
		if (rtsr & ALSTA) {
			events |= (RTC_AF | RTC_IRQF);
			printk("ac83xx_rtc_interrupt(): mark alarm, disable alarm interrupt \n");
			RTCAlarmAllCMPR(0);
			RTCEnableAlarmInt(0);
		}

		if (rtsr & TCSTA) {
			events |= (RTC_IRQF);
			printk("ac83xx_rtc_interrupt(): disable tc interrupt \n");
			RTCEnableTCInt(0);
		}
		RTCProtect(1);
		rtc_update_irq(rtc, 1, events);

		printk("ac83xx_rtc_interrupt(): num=%ld, events=0x%02lx\n", 
			events >> 8, events & 0x000000FF);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;		/* not handled */
}


static ssize_t ac83xx_rtc_sysfs_show_flag(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	printk("++ ac83xx_rtc, show [%d] \n", enable_output);
	return sprintf(buf, "%d\n", enable_output);
}

static ssize_t ac83xx_rtc_sysfs_store_flag(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	int adj, err;

	printk("++ ac83xx_rtc, store [%s], count %d \n", buf, count);
	if (sscanf(buf, "%i", &adj) != 1) {
		return -EINVAL;
	}

	if (adj == 0) {
		enable_output = 0;
	} else if (adj == 1) {
		enable_output = 1;
	} else {
		enable_output = 1;
	}

	return count;
}

static DEVICE_ATTR(printk_flag, S_IRUGO | S_IWUSR,
		   ac83xx_rtc_sysfs_show_flag,
		   ac83xx_rtc_sysfs_store_flag);

static int ac83xx_rtc_sysfs_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_printk_flag);
}

static void ac83xx_rtc_sysfs_unregister(struct device *dev)
{
	device_remove_file(dev, &dev_attr_printk_flag);
}

//
static ssize_t ac83xx_rtc_sysfs_show_log_time_flag(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	printk("++ ac83xx_rtc, show log time flag [%d] \n", log_time);
	return sprintf(buf, "%d\n", log_time);
}

static ssize_t ac83xx_rtc_sysfs_store_log_time_flag(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	int adj, err;

	printk("++ ac83xx_rtc, store log time flag [%s], count %d \n", buf, count);
	if (sscanf(buf, "%i", &adj) != 1) {
		return -EINVAL;
	}

	if (adj == 0) {
		log_time = 0;
	} else if (adj == 1) {
		log_time = 1;
	} else {
		log_time = 1;
	}

	return count;
}

static DEVICE_ATTR(log_time_flag, S_IRUGO | S_IWUSR,
		   ac83xx_rtc_sysfs_show_log_time_flag,
		   ac83xx_rtc_sysfs_store_log_time_flag);

static int ac83xx_rtc_sysfs_logtime_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_log_time_flag);
}

static void ac83xx_rtc_sysfs_logtime_unregister(struct device *dev)
{
	device_remove_file(dev, &dev_attr_log_time_flag);
}

//

/*
 * Initialize and install RTC driver
 */
static int __init ac83xx_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret;
	//bool rc = FALSE;

	ret = RTCHWInit();
	if (ret == FALSE)
		printk(KERN_INFO "rtc init err!\n");
	
	/* Disable all interrupts */
	if (ret == TRUE) {
		RTCEnableAlarmInt(0);
		RTCEnableTCInt(0);

		//RTCIntAutoReset(0);
		//RTCClareAllInt();
		//RTCClareAllAlarmInt();

		//RTC_CLR_BIT(RTC_IRQ_EN, TC_EN);
		//RTC_CLR_BIT(RTC_IRQ_EN, AL_EN);
		//RTC_CLR_BIT(RTC_IRQ_EN, ONESHOT);

		/* output 32k */
		ac83xx_rtc_output_32k();
	}

	rtc = rtc_device_register("ac83xx-rtc", &pdev->dev,
				&ac83xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		return PTR_ERR(rtc);
	}

	platform_set_drvdata(pdev, rtc);
	if (ret == TRUE) {
		ret = request_irq(VECTOR_RTC, ac83xx_rtc_interrupt,
					0,
					"ac83xx_rtc", pdev);
		#if 0
		ret = request_irq(VECTOR_RTC, ac83xx_rtc_interrupt,
					IRQF_DISABLED,
					"ac83xx_rtc", pdev);
		#endif
		if (ret) {
			printk(KERN_ERR "ac83xx1_rtc: IRQ %d already in use.\n",
					VECTOR_RTC);
			rtc_device_unregister(rtc);
			platform_set_drvdata(pdev, NULL);
			return ret;
		}
	}

	ret = ac83xx_rtc_sysfs_register(&pdev->dev);
	if (ret) {
		printk(KERN_ERR "ac83xx1_rtc: creat sys file error \n");
	}

	/* for output log to sd card */
	ret = ac83xx_rtc_sysfs_logtime_register(&pdev->dev);
	if (ret) {
		printk(KERN_ERR "ac83xx1_rtc: creat sys file error \n");
	}

	printk(KERN_INFO "AC83xx Real Time Clock driver.\n");
	return 0;
}

/*
 * Disable and remove the RTC driver
 */
static int __exit ac83xx_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
    
    /* Disable all interrupts */
	RTCEnableAlarmInt(0);
  	RTCEnableTCInt(0);
  	//RTCClareAllInt();
  	//RTCClareAllAlarmInt();

	/* for output log to sd card */
  	#if 1
	ac83xx_rtc_sysfs_unregister(&pdev->dev);
	#endif
	ac83xx_rtc_sysfs_logtime_unregister(&pdev->dev);
	/* end  */
    
    free_irq(VECTOR_RTC, pdev);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ac83xx_rtc_suspend(struct platform_device *dev, pm_message_t state)
{
	printk(KERN_INFO "ac83xx_rtc_suspend.\n");
	g_bFirstBooting = TRUE;

	return 0;
}

static int ac83xx_rtc_resume(struct platform_device *dev)
{
	int ret;

	printk(KERN_INFO "ac83xx_rtc_resume.\n");
	
	ret = RTCHWInit();
	if (ret == FALSE)
		printk(KERN_INFO "rtc init err!\n");
	
	/* Disable all interrupts */
	if (ret == TRUE) {
		RTCEnableAlarmInt(0);
		RTCEnableTCInt(0);

		/* output 32k */
		ac83xx_rtc_output_32k();
	}

	return 0;
}


static struct platform_driver ac83xx_rtc_driver = {
	.probe		= ac83xx_rtc_probe,
	.remove		= __exit_p(ac83xx_rtc_remove),
	.suspend    = ac83xx_rtc_suspend,
	.resume		= ac83xx_rtc_resume,
	.driver		= {
		.name	= "ac83xx_rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init ac83xx_rtc_init(void)
{
	return platform_driver_register(&ac83xx_rtc_driver);
}

static void __exit ac83xx_rtc_exit(void)
{
	platform_driver_unregister(&ac83xx_rtc_driver);
}

module_init(ac83xx_rtc_init);
module_exit(ac83xx_rtc_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("RTC driver for AC83XX");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ac83xx_rtc");
