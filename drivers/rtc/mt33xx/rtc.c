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


//#define AT91_RTC_EPOCH		1900UL	/* just like arch/arm/common/rtctime.c */

//static DECLARE_COMPLETION(at91_rtc_updated);
//static unsigned int at91_alarm_year = AT91_RTC_EPOCH;

//------------------------------------------------------------------------------
// Defines
#define RTC_YEAR_DATUM (uint16_t)(2000)
#define RTC_YEAR_END (uint16_t)(2127)

spinlock_t mt3360_rtc_lock; 

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


//------------------------------------------------------------------------------
//
//  Function:  mt3360_rtc_readtime
//
//  Reads the current RTC value and returns a system time.
//
static int mt85xx_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
    int rc = -1;
    
    if (tm == NULL) goto cleanUp;
	//printk(KERN_INFO "+mt3360_rtc_readtime into tm ->1.\n");
    if (RTCHWInit() == FALSE) goto cleanUp;
    //printk(KERN_INFO "+mt3360_rtc_readtime into tm ->2.\n");
    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    spin_lock_irq(&mt3360_rtc_lock);
	//printk(KERN_INFO "+mt3360_rtc_readtime into tm ->3.\n");
    tm->tm_sec = RTCSecRead();
    tm->tm_min = RTCMinRead();
    tm->tm_hour = RTCHourRead();
    tm->tm_mday = RTCDOMRead();
    tm->tm_mon  = RTCMonthRead()-1;
    // Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.    
    tm->tm_wday = RTCDOWRead() % 7;
    tm->tm_year = RTCYearRead() + RTC_YEAR_DATUM - 1900;
    tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	//printk(KERN_INFO "+mt3360_rtc_readtime into tm ->4.\n");
    //INTERRUPTS_ENABLE(enabled);
    spin_unlock_irq(&mt3360_rtc_lock);
	//printk(KERN_INFO "+mt3360_rtc_readtime into tm ->5.\n");
    //pTime->wMilliseconds = 0;
    
    printk("mt85xx_rtc_readtime: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    // Done
    rc = 0;
    
cleanUp:   
    printk("-mt85xx_rtc_readtime(rc = %d)\r\n", rc);
    return rc;
}


//------------------------------------------------------------------------------
//
//  Function:  mt3360_rtc_settime
//
//  Updates the RTC with the specified system time.
//
static int mt85xx_rtc_settime(struct device *dev, struct rtc_time *tm)
{
    int rc = -1;
    printk("mt85xx_rtc_settime: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    if (tm == NULL) goto cleanUp;

    if (!VerifyDateTime(tm)) goto cleanUp;

    if (RTCHWInit() == FALSE) goto cleanUp;
    
    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    spin_lock_irq(&mt3360_rtc_lock);
    
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
    spin_unlock_irq(&mt3360_rtc_lock);
    
    // Done
    rc = 0;
    
cleanUp:
    printk("-mt85xx_rtc_settime(rc = %d)\r\n", rc);
    return rc;
}


//------------------------------------------------------------------------------
//
//  Function:  mt3360_rtc_setalarm
//
//  Set the RTC alarm time.
//
static int mt85xx_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    int rc = -1;
	struct rtc_time *tm = &alrm->time;
    if(!tm) goto cleanUp;
    
    printk("mt85xx_rtc_setalarm: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    if (!VerifyDateTime(tm)) goto cleanUp;

    if (RTCHWInit() == FALSE) goto cleanUp;

    // To avoid the interrupt to affect the rtc time
    //enabled = INTERRUPTS_ENABLE(FALSE);
    spin_lock_irq(&dev->devres_lock);
    
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
		RTCAlarmAllCMPR(1);
		RTCEnableAlarmInt(1);
	    RTCIntAutoReset(1);	
	}
	
    RTCProtect(1);

    //INTERRUPTS_ENABLE(enabled);
    spin_unlock_irq(&dev->devres_lock);
 
    // Done
    rc = 0;
    
cleanUp:
    //OALMSG(OAL_RTC&&OAL_FUNC, (L"-OEMSetAlarmTime(rc = %d)\r\n", rc));
    printk("-mt85xx_rtc_setalarm(rc = %d)\r\n", rc);

    return rc;
}

//------------------------------------------------------------------------------

/*
 * Read alarm time and date in RTC
 */
static int mt85xx_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
    int rc = -1;
	struct rtc_time *tm = &alrm->time;

	if(!tm) goto cleanUp;
	if (RTCHWInit() == FALSE) goto cleanUp;

	// To avoid the interrupt to affect the rtc time
	//enabled = INTERRUPTS_ENABLE(FALSE);
	spin_lock_irq(&dev->devres_lock);
	
	RTCProtect(0);

	// 1. Set the alarm time register
	tm->tm_sec = RTCAlarmSecRead();
	tm->tm_min = RTCAlarmMinRead();
	tm->tm_hour = RTCAlarmHourRead();
	tm->tm_mday = RTCAlarmDOMRead();
	tm->tm_mon = RTCAlarmMonthRead() - 1;
	// Hardware DayOfWeek is 1~7 but WinCE is 0~6, 0 is Sunday.
	tm->tm_wday = RTCAlarmDOWRead() % 7;
    tm->tm_year = RTCAlarmYearRead() + RTC_YEAR_DATUM - 1900;
    tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	alrm->enabled = (RTC_READ32(RTC_IRQ_STA) & ALSTA) ? 1 : 0;
	RTCProtect(1);
	spin_unlock_irq(&dev->devres_lock);

	printk("mt85xx_rtc_readalarm: %4d-%02d-%02d %02d:%02d:%02d\n",
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);
    rc = 0;
    
cleanUp:
    //OALMSG(OAL_RTC&&OAL_FUNC, (L"-OEMSetAlarmTime(rc = %d)\r\n", rc));
    printk("-mt85xx_rtc_readalarm(rc = %d)\r\n", rc);

    return rc;
}

/*
 * Handle commands from user-space
 */
static int mt85xx_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;

	printk("mt85xx_rtc_ioctl(): cmd=%08x, arg=%08lx.\n", cmd, arg);

	/* important:  scrub old status before enabling IRQs */
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		RTCEnableAlarmInt(0);
		break;
	case RTC_AIE_ON:	/* alarm on */
        RTCEnableAlarmInt(1);
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

static const struct rtc_class_ops mt85xx_rtc_ops = {
	.ioctl		= mt85xx_rtc_ioctl,
	.read_time	= mt85xx_rtc_readtime,
	.set_time	= mt85xx_rtc_settime,
	.read_alarm	= mt85xx_rtc_readalarm,
	.set_alarm	= mt85xx_rtc_setalarm,
//	.proc		= at91_rtc_proc,
};


/*
 * IRQ handler for the RTC
 */
static irqreturn_t mt85xx_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr;
	unsigned long events = 0;

	rtsr = RTC_READ32(RTC_IRQ_STA);
	if (rtsr) {		/* this interrupt is shared!  Is it ours? */
		if (rtsr & ALSTA)
			events |= (RTC_AF | RTC_IRQF);
		rtc_update_irq(rtc, 1, events);

		printk("mt85xx_rtc_interrupt(): num=%ld, events=0x%02lx\n", 
			events >> 8, events & 0x000000FF);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;		/* not handled */
}

/*
 * Initialize and install RTC driver
 */
static int __init mt85xx_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret;
	//bool rc = FALSE;
	if ((RTCHWInit()) == FALSE)
		printk(KERN_INFO "rtc init err!\n");
	
	/* Disable all interrupts */
	RTCEnableAlarmInt(0);
  	RTCEnableTCInt(0);
	//RTCIntAutoReset(0);
  	//RTCClareAllInt();
  	//RTCClareAllAlarmInt();
  	
	//RTC_CLR_BIT(RTC_IRQ_EN, TC_EN);
	//RTC_CLR_BIT(RTC_IRQ_EN, AL_EN);
	//RTC_CLR_BIT(RTC_IRQ_EN, ONESHOT);

	rtc = rtc_device_register("mt85xx-rtc", &pdev->dev,
				&mt85xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);
	
	ret = request_irq(VECTOR_RTC, mt85xx_rtc_interrupt,
				0,
				"mt85xx_rtc", pdev);
	
	#if 0
	ret = request_irq(VECTOR_RTC, mt85xx_rtc_interrupt,
				IRQF_DISABLED,
				"mt85xx_rtc", pdev);
	#endif
	
	if (ret) {
		printk(KERN_ERR "mt85xx1_rtc: IRQ %d already in use.\n",
				VECTOR_RTC);
		rtc_device_unregister(rtc);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}
	
	
	printk(KERN_INFO "MT85xx Real Time Clock driver.\n");
	return 0;
}

/*
 * Disable and remove the RTC driver
 */
static int __exit mt85xx_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);
    
    /* Disable all interrupts */
	RTCEnableAlarmInt(0);
  	RTCEnableTCInt(0);
  	//RTCClareAllInt();
  	//RTCClareAllAlarmInt();
    
    free_irq(VECTOR_RTC, pdev);
	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mt85xx_rtc_driver = {
	.probe		= mt85xx_rtc_probe,
	.remove		= __exit_p(mt85xx_rtc_remove),
	.driver		= {
		.name	= "mt85xx_rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init mt85xx_rtc_init(void)
{
	//return platform_driver_probe(&mt85xx_rtc_driver, mt85xx_rtc_probe);
	return platform_driver_register(&mt85xx_rtc_driver);
}

static void __exit mt85xx_rtc_exit(void)
{
	platform_driver_unregister(&mt85xx_rtc_driver);
}

module_init(mt85xx_rtc_init);
module_exit(mt85xx_rtc_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("RTC driver for MT3360");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mt3360_rtc");
