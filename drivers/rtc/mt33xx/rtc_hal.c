#if 0
#include "x_typedef.h"


#include "x_dramc.h"
#include "x_hal_ic.h"
#include "x_printf.h"
#include "x_rtos.h"
#include "x_os.h"
#include "x_assert.h"
#include "x_timer.h"
#include "x_bim.h"
#include "x_lint.h"
#include "x_drv_cli.h"
#include "x_stl_lib.h"
#include "x_debug.h"


#include "x_stl_lib.h"

#include "x_bim.h"
#include "x_kmem.h"
#endif

//#include <mach/mt8530.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include "rtc_hal.h"

#define ADDRSETBIT(addr,bit)  IO_WRITE32(IO_UCV_BASE,addr,IO_READ32(IO_UCV_BASE,addr) | (1 << bit)) 
#define ADDRCLRBIT(addr,bit)  IO_WRITE32(IO_UCV_BASE,addr,IO_READ32(IO_UCV_BASE,addr) & (~(1 << bit))) 


int g_bFirstBooting = TRUE;


uint32_t u4RTC_DBG_LVL = 5; // Log Level contorll


uint32_t RtcGetDeviceInfo (void)
{
#if 1   
    return FALSE;
#else
	return TURE
#endif
}




/************************************************************************
  Function : void vRTCSecSet(int32_t bSec)
  Description : Set RTC Second
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCSecSet(int32_t bSec)
{
 	RTC_WRITE32(RTC_TC_SEC,bSec);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCMinSet(int32_t bMin)
  Description : Set RTC Minute
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCMinSet(int32_t bMin)
{
	RTC_WRITE32(RTC_TC_MIN,bMin);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCHourSet(int32_t bHour)
  Description : Set RTC Hour
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCHourSet(int32_t bHour)
{
  	RTC_WRITE32(RTC_TC_HOU, bHour);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCDOMSet(int32_t bDay)
  Description : Set RTC Day of Month
  Parameter : bDay
  Return    : NONE
************************************************************************/
void RTCDOMSet(int32_t bDay)
{
  	RTC_WRITE32(RTC_TC_DOM, bDay);
	//CORETORTC();
}

/************************************************************************
  Function : void vRTCDaySet(int32_t bDay)
  Description : Set RTC Day of week
  Parameter : bDay
  Return    : NONE
************************************************************************/
void RTCDOWSet(int32_t bDay)
{
  	RTC_WRITE32(RTC_TC_DOW, bDay);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCMonthSet(int32_t bMonth)
  Description : Set RTC Month
  Parameter : bMonth
  Return    : NONE
************************************************************************/
void RTCMonthSet(int32_t bMonth)
{
  	RTC_WRITE32(RTC_TC_MTH, bMonth);
	//CORETORTC();
}



/************************************************************************
  Function : void vRTCYearSet(int32_t bYear)
  Description : Set RTC Year
  Parameter : bYear
  Return    : NONE
************************************************************************/
void RTCYearSet(int32_t bYear)
{
	RTC_WRITE32(RTC_TC_YEA, bYear);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCSecSet(int32_t bSec)
  Description : Set RTC Second
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCSecRead(void)
{
	//RTCTOCORE();
 	return (RTC_READ32(RTC_TC_SEC));	
}


/************************************************************************
  Function : void vRTCMinSet(int32_t bMin)
  Description : Set RTC Minute
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCMinRead(void)
{
	//RTCTOCORE();
	return (RTC_READ32(RTC_TC_MIN));
}


/************************************************************************
  Function : void vRTCHourSet(int32_t bHour)
  Description : Set RTC Hour
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCHourRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_TC_HOU));
}


/************************************************************************
  Function : void vRTCDOMSet(int32_t bDay)
  Description : Set RTC Day of Month
  Parameter : bDay
  Return    : NONE
************************************************************************/
int32_t RTCDOMRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_TC_DOM));
}

/************************************************************************
  Function : void vRTCDaySet(int32_t bDay)
  Description : Set RTC Day of week
  Parameter : bDay
  Return    : NONE
************************************************************************/
int32_t RTCDOWRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_TC_DOW));
}


/************************************************************************
  Function : void vRTCMonthSet(int32_t bMonth)
  Description : Set RTC Month
  Parameter : bMonth
  Return    : NONE
************************************************************************/
int32_t RTCMonthRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_TC_MTH));
}


/************************************************************************
  Function : void vRTCYearSet(int32_t bYear)
  Description : Set RTC Year
  Parameter : bYear
  Return    : NONE
************************************************************************/
int32_t RTCYearRead(void)
{
	//RTCTOCORE();
	return (RTC_READ32(RTC_TC_YEA));
}



/************************************************************************
  Function : void vRTCSecSet(int32_t bSec)
  Description : Set RTC Second
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCAlarmSecSet(int32_t bSec)
{
 	RTC_WRITE32(RTC_AL_SEC,bSec);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCMinSet(int32_t bMin)
  Description : Set RTC Minute
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCAlarmMinSet(int32_t bMin)
{
	RTC_WRITE32(RTC_AL_MIN,bMin);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCHourSet(int32_t bHour)
  Description : Set RTC Hour
  Parameter : NONE
  Return    : NONE
************************************************************************/
void RTCAlarmHourSet(int32_t bHour)
{
  	RTC_WRITE32(RTC_AL_HOU, bHour);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCDOMSet(int32_t bDay)
  Description : Set RTC Day of Month
  Parameter : bDay
  Return    : NONE
************************************************************************/
void RTCAlarmDOMSet(int32_t bDay)
{
  	RTC_WRITE32(RTC_AL_DOM, bDay);
	//CORETORTC();
}

/************************************************************************
  Function : void vRTCDaySet(int32_t bDay)
  Description : Set RTC Day of week
  Parameter : bDay
  Return    : NONE
************************************************************************/
void RTCAlarmDOWSet(int32_t bDay)
{
  	RTC_WRITE32(RTC_AL_DOW, bDay);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCMonthSet(int32_t bMonth)
  Description : Set RTC Month
  Parameter : bMonth
  Return    : NONE
************************************************************************/
void RTCAlarmMonthSet(int32_t bMonth)
{
  	RTC_WRITE32(RTC_AL_MTH, bMonth);
	//CORETORTC();
}



/************************************************************************
  Function : void vRTCYearSet(int32_t bYear)
  Description : Set RTC Year
  Parameter : bYear
  Return    : NONE
************************************************************************/
void RTCAlarmYearSet(int32_t bYear)
{
	RTC_WRITE32(RTC_AL_YEA, bYear);
	//CORETORTC();
}


/************************************************************************
  Function : void vRTCSecSet(int32_t bSec)
  Description : Set RTC Second
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCAlarmSecRead(void)
{
	//RTCTOCORE();
 	return (RTC_READ32(RTC_AL_SEC));
}


/************************************************************************
  Function : void vRTCMinSet(int32_t bMin)
  Description : Set RTC Minute
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCAlarmMinRead(void)
{
	//RTCTOCORE();
	return (RTC_READ32(RTC_AL_MIN));
}


/************************************************************************
  Function : void vRTCHourSet(int32_t bHour)
  Description : Set RTC Hour
  Parameter : NONE
  Return    : NONE
************************************************************************/
int32_t RTCAlarmHourRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_AL_HOU));
}


/************************************************************************
  Function : void vRTCDOMSet(int32_t bDay)
  Description : Set RTC Day of Month
  Parameter : bDay
  Return    : NONE
************************************************************************/
int32_t RTCAlarmDOMRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_AL_DOM));
}

/************************************************************************
  Function : void vRTCDaySet(int32_t bDay)
  Description : Set RTC Day of week
  Parameter : bDay
  Return    : NONE
************************************************************************/
int32_t RTCAlarmDOWRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_AL_DOW));
}


/************************************************************************
  Function : void vRTCMonthSet(int32_t bMonth)
  Description : Set RTC Month
  Parameter : bMonth
  Return    : NONE
************************************************************************/
int32_t RTCAlarmMonthRead(void)
{
	//RTCTOCORE();
  	return (RTC_READ32(RTC_AL_MTH));
}


/************************************************************************
  Function : void vRTCYearSet(int32_t bYear)
  Description : Set RTC Year
  Parameter : bYear
  Return    : NONE
************************************************************************/
int32_t RTCAlarmYearRead(void)
{
	//RTCTOCORE();
	return (RTC_READ32(RTC_AL_YEA));
}



void RTCAlarmAllCMPR(int32_t bEnable)
{
	if(bEnable)
  		RTC_WRITE32(RTC_AL_MASK, RTC_READ32(RTC_AL_MASK) & (~AL_MSK));
  	else
		  RTC_WRITE32(RTC_AL_MASK, RTC_READ32(RTC_AL_MASK) | AL_MSK);
	CORETORTC();
}


/************************************************************************
  Function : void vRTCAlarmYearCMPR(int32_t bEnable)
  Description : Alarm compare YEAR enable
  Parameter : bEnable 1:enable 0:disable
  Return    : NONE
************************************************************************/
void RTCAlarmYearCMPR(int32_t bEnable)
{
	if(bEnable)
  		RTC_CLR_BIT(RTC_AL_MASK, YEA_MSK);
  	else
		RTC_SET_BIT(RTC_AL_MASK, YEA_MSK);
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmMonthCMPR(int32_t bEnable)
  Description : Alarm compare MONTH enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmMonthCMPR(int32_t bEnable)
{
	if(bEnable)
  		RTC_CLR_BIT(RTC_AL_MASK, MTH_MSK);
  	else
		RTC_SET_BIT(RTC_AL_MASK, MTH_MSK);
	CORETORTC();
}


/************************************************************************
     Function : void vRTCAlarmDayCMPR(int32_t bEnable)
  Description : Alarm compare DAY enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmDowCMPR(int32_t bEnable)
{
	if(bEnable)
		RTC_CLR_BIT(RTC_AL_MASK, DOW_MSK);
	else
		RTC_SET_BIT(RTC_AL_MASK, DOW_MSK);
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmWeekCMPR(int32_t bEnable)
  Description : Alarm compare WEEK enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmDomCMPR(int32_t bEnable)
{
	if(bEnable)
		RTC_CLR_BIT(RTC_AL_MASK, DOM_MSK);
	else
		RTC_SET_BIT(RTC_AL_MASK, DOM_MSK);
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmHourCMPR(int32_t bEnable)
  Description : Alarm compare HOUR enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmHourCMPR(int32_t bEnable)
{
	if(bEnable)
		RTC_CLR_BIT(RTC_AL_MASK, HOU_MSK);
	else
		RTC_SET_BIT(RTC_AL_MASK, HOU_MSK);
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmMinCMPR(int32_t bEnable)
  Description : Alarm compare MIN enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmMinCMPR(int32_t bEnable)
{
	if(bEnable)
		RTC_CLR_BIT(RTC_AL_MASK, MIN_MSK);
	else
		RTC_SET_BIT(RTC_AL_MASK, MIN_MSK);
	CORETORTC();

}

/************************************************************************
     Function : void vRTCAlarmSecCMPR(int32_t bEnable)
  Description : Alarm compare SEC enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCAlarmSecCMPR(int32_t bEnable)
{

  if(bEnable)
	  RTC_CLR_BIT(RTC_AL_MASK, SEC_MSK);
  else
	  RTC_SET_BIT(RTC_AL_MASK, SEC_MSK);
  CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmInt(int32_t bEnable)
  Description : Alarm interrupt enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCEnableAlarmInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTC_SET_BIT(RTC_IRQ_EN, AL_EN);		
	}
	else
	{
		RTC_CLR_BIT(RTC_IRQ_EN, AL_EN);
	}
	CORETORTC();
}
#if 0
/************************************************************************
  Function : void vRTCAlarmYearCMPR(int32_t bEnable)
  Description : Alarm compare YEAR enable
  Parameter : bEnable 1:enable 0:disable
  Return    : NONE
************************************************************************/
void RTCYearInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();
		RTC_SET_BIT(RTC_CII_EN, YEACII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, YEACII);
	}
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmMonthCMPR(int32_t bEnable)
  Description : Alarm compare MONTH enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCMonthInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();
		RTC_SET_BIT(RTC_CII_EN, MTHCII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, MTHCII);
	}
	CORETORTC();
}


/************************************************************************
     Function : void vRTCAlarmDayCMPR(int32_t bEnable)
  Description : Alarm compare DAY enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCDowInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();	
		RTC_SET_BIT(RTC_CII_EN, DOWCII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, DOWCII);
	}
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmWeekCMPR(int32_t bEnable)
  Description : Alarm compare WEEK enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCDomInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();	
		RTC_SET_BIT(RTC_CII_EN, DOMCII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, DOMCII);
	}
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmHourCMPR(int32_t bEnable)
  Description : Alarm compare HOUR enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCHourInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();	
		RTC_SET_BIT(RTC_CII_EN, HOUCII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, HOUCII);
	}
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmMinCMPR(int32_t bEnable)
  Description : Alarm compare MIN enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCMinInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTCClareAllInt();	
		RTC_SET_BIT(RTC_CII_EN, MINCII);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, MINCII);
	}
	CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmSecCMPR(int32_t bEnable)
  Description : Alarm compare SEC enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCSecInt(int32_t bEnable)
{
  if(bEnable)
  {
	  RTCClareAllInt();   
	  RTC_SET_BIT(RTC_CII_EN, SECCII);
  }
  else	  
  {
	  RTC_CLR_BIT(RTC_CII_EN, SECCII);
  }
  CORETORTC();
}


/************************************************************************
     Function : void vRTCAlarmSecCMPR(int32_t bEnable)
  Description : Alarm compare SEC enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCSecInt_8_1(int32_t bEnable)
{
  if(bEnable)
  {
	  RTCClareAllInt();   
	  RTC_SET_BIT(RTC_CII_EN, SECCII_8_1);
  }
  else	  
  {
	  RTC_CLR_BIT(RTC_CII_EN, SECCII_8_1);
  }
  CORETORTC();
}


/************************************************************************
     Function : void vRTCAlarmSecCMPR(int32_t bEnable)
  Description : Alarm compare SEC enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCSecInt_4_1(int32_t bEnable)
{
  if(bEnable)
  {
	  RTCClareAllInt(); 
	  RTC_SET_BIT(RTC_CII_EN, SECCII_4_1);
  }
  else	  
  {
	  RTC_CLR_BIT(RTC_CII_EN, SECCII_4_1);
  }
  CORETORTC();
}

/************************************************************************
     Function : void vRTCAlarmSecCMPR(int32_t bEnable)
  Description : Alarm compare SEC enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCSecInt_2_1(int32_t bEnable)
{
	if(bEnable)
	{
		RTC_WRITE32(RTC_CII_EN,(RTC_READ32(RTC_CII_EN) & 0xFFFFFC00));	
		RTC_SET_BIT(RTC_CII_EN, SECCII_2_1);
	}
	else	
	{
		RTC_CLR_BIT(RTC_CII_EN, SECCII_2_1);
	}
	CORETORTC();
}

#endif


void RTCIntAutoReset(int32_t bEnable)
{
  if(bEnable)
  {
    RTC_SET_BIT(RTC_IRQ_EN, ONESHOT);		
  }
  else
  {
    RTC_CLR_BIT(RTC_IRQ_EN, ONESHOT);
  }
  CORETORTC();
}


/************************************************************************
     Function : void RTCEnableTCInt(int32_t bEnable)
  Description : Alarm interrupt enable
    Parameter : bEnable 1:enable 0:disable
    Return    : NONE
************************************************************************/
void RTCEnableTCInt(int32_t bEnable)
{
	if(bEnable)
	{
		RTC_SET_BIT(RTC_IRQ_EN, TC_EN);		
	}
	else
	{
		RTC_CLR_BIT(RTC_IRQ_EN, TC_EN);
		//BIM_DisableIrq(VECTOR_RTCAL);
	}
	CORETORTC();
}


#if 0
/************************************************************************
     Function : vRtcAlarmIsr(uint16_t u2Vector)
  Description : Rtc Isr
    Parameter : Isr Vector
    Return    : NONE
************************************************************************/

void RtcIsr(uint16_t u2Vector)
{
	
	
	int32_t RTCIRQStatus; 
	#if 1
	int32_t u4Year,u4Month,u4DOW,u4DOM,u4Hour,u4Min,u4Sec;
	char *Week[7] = {"Mon","Tue","Wed","Thur","Fri","Sat","Sun"};
	#endif

	#if 0
	ADDRSETBIT(0x8238,14);
	ADDRSETBIT(0x8230,14);
	#endif

	RTCIRQStatus = RTC_READ32(RTC_IRQ_STA);
	
	//TC Int
	if(RTCIRQStatus & 0x02)
	{	
		//MSG(MSG_LVL_TITLE,"hello,TC\r\n");
		#if 1
	    WAITBUSY();	
		u4Sec = RTCSecRead();
		u4Min = RTCMinRead();
		u4Hour = RTCHourRead();
		u4DOM = RTCDOMRead();
		u4DOW = RTCDOWRead();
		u4Month = RTCMonthRead();
		u4Year = RTCYearRead(); 
		MSG(MSG_LVL_TITLE,"TC Int occurred,The time is: %d-%d-%d,%d:%d %d %s\n",
		(2000 + u4Year),u4Month,u4DOM,u4Hour,u4Min,u4Sec,Week[u4DOW - 1]);	
		#endif
	}
	
	//Alarm Int
	if(RTCIRQStatus & 0x01)
	{
		//MSG(MSG_LVL_TITLE,"hello,Alarm\r\n");
		#if 1
		WAITBUSY(); 
		u4Sec = RTCAlarmSecRead();
		u4Min = RTCAlarmMinRead();
		u4Hour = RTCAlarmHourRead();
		u4DOM = RTCAlarmDOMRead();
		u4DOW = RTCAlarmDOWRead();
		u4Month = RTCAlarmMonthRead();
		u4Year = RTCAlarmYearRead();
		MSG(MSG_LVL_TITLE,"Alarm Int occurred, the alarm time is set as: %d-%d-%d,%d:%d %d %s\n",
			(2000 + u4Year),u4Month,u4DOM,u4Hour,u4Min,u4Sec,Week[u4DOW - 1]);	
		#endif
	}
}

int32_t RegRtcInt(INT32 bEnable)
{
	x_os_isr_fct pfnOldIsr;
	uint32_t u4Vect;
	u4Vect = VECTOR_RTC;
	if(bEnable)
	{
	
		if (OSR_OK != x_reg_isr(u4Vect, RtcIsr, &pfnOldIsr))
		{
			MSG(MSG_LVL_ERR,"[RTC]Failed to enable IRQ.\r\n");
			return 0;
		}	
	}
	else
	{
		//BIM_ClearIrq(u4Vect);
		if (OSR_OK != x_reg_isr(u4Vect, NULL, &pfnOldIsr))
		{
			MSG(MSG_LVL_ERR,"[RTC]Failed to disable IRQ.\r\n");
			return 0;
		}		
	}
	return 1;
}
#endif

/****************************************************************************
     Function : RTC initial check for AC on standby
  Description : Initialize the RTC.
    Parameter : NONE
     Return   : TRUE: RTC is reset, FALSE: RTC is not reset
*****************************************************************************/
int32_t RtcFirstRunChk(void)
{
	RTC_WRITE32(RTC_POWERKEY1, 0xa357);
	RTC_WRITE32(RTC_POWERKEY2, 0x67d2);
	CORETORTC();
	//WAITBUSY();	
	//RTCProtect(0);
	RTCSecSet(RTC_DEFAULT_SEC);	
	RTCMinSet(RTC_DEFAULT_MIN);
	RTCHourSet(RTC_DEFAULT_HOUR);
	RTCDOMSet(RTC_DEFAULT_DOM);
	RTCDOWSet(RTC_DEFAULT_DOW);
	RTCMonthSet(RTC_DEFAULT_MONTH);
	RTCYearSet(RTC_DEFAULT_YEAR);
	RTCAlarmSecSet(RTC_DEFAULT_SEC);	
	RTCAlarmMinSet(RTC_DEFAULT_MIN);
	RTCAlarmHourSet(RTC_DEFAULT_HOUR);
	RTCAlarmDOMSet(RTC_DEFAULT_DOM);
	RTCAlarmDOWSet(RTC_DEFAULT_DOW);
	RTCAlarmMonthSet(RTC_DEFAULT_MONTH);
	RTCAlarmYearSet(RTC_DEFAULT_YEAR);

  RTCEnableAlarmInt(0);
  RTCEnableTCInt(0);
  RTCClareAllInt();
  RTCClareAllAlarmInt();
  
	//CORETORTC();
	//RTCProtect(1);
	return 1;
}



/************************************************************************
  Function : void vRTCHWInit(void)
  Description : RTC init.  
  Parameter : NONE
  Return    : NONE
************************************************************************/
int RTCHWInit(void)
{	
  int32_t i;

  if(!g_bFirstBooting) return(TRUE);
  printk(KERN_INFO "RTCHWInit ->1.\n");
	//RTCProtect(0);
  for(i = 0;i < RTCINTMAX; i++) 
	{
  	while(RTC_READ32(RTC_BBPU) & 0x80);
  	WAITBUSY();	
    printk(KERN_INFO "RTCHWInit ->2.\n");
  	RTCProtect(0);
  	
  	if(0x43 != (uint16_t)(RTC_READ32(RTC_BBPU) >> 8))
  	{
  		printk(KERN_INFO "RTCHWInit ->3.\n");
  		RTC_WRITE32(RTC_BBPU,(RTC_READ32(RTC_BBPU) | 0x4302));  //bbpu bit[1] write_en
  		CORETORTC();
		printk(KERN_INFO "RTCHWInit ->4.\n");
  	}

    //RTCTOCORE();//new chip modify reload
		
	if((0xa357 == (uint16_t)RTC_READ32(RTC_POWERKEY1)) && (0x67d2 == (uint16_t)RTC_READ32(RTC_POWERKEY2)))
	{
		  //DEBUGMSG(1,(L"****** RTC BUG ********  RTC_POWERKEY is fit %d times.\n",(i + 1)));
		printk(KERN_INFO "****** RTC BUG ********  RTC_POWERKEY is fit %d times.\n",(i + 1));
		break;		
	}

		//DEBUGMSG(1,(L"RTC_POWERKEY is not fit %d times.\n",(i + 1)));
		printk(KERN_INFO "RTC_POWERKEY is not fit %d times.\n",(i + 1));
		
	}
	
	if((0xa357 != (uint16_t)RTC_READ32(RTC_POWERKEY1)) || (0x67d2 != (uint16_t)RTC_READ32(RTC_POWERKEY2)))
	{
		RtcFirstRunChk();			
	}
  g_bFirstBooting = FALSE;
	
	RTCProtect(1);

  return TRUE;
}




/************************************************************************
  Function : void vRTCProtect(int32_t bEnable)
  Description : RTC Protection enable
  Parameter : bEnable 1:enable 0:disable
  Return    : NONE
************************************************************************/
void RTCProtect(int32_t bEnable)
{

  if(bEnable)
  {
      RTC_WRITE32(RTC_PROT, 0x00);
	  CORETORTC();
  }
  else
  {
  	  RTC_WRITE32(RTC_PROT, 0x9136);
	  CORETORTC();
	  RTC_WRITE32(RTC_PROT, 0x586A);
	  CORETORTC();
  }  
}



void RTCTrim(int32_t TrimData)
{
	if((TrimData > (-2048)) && (TrimData < 2045))
	{
		RTC_WRITE32(RTC_DIFF,TrimData);	
	}
	else
	{
		//OALMSG(1,(L"The Trim Data must be from -2048 to 2045.\n"));
		pr_debug("The Trim Data must be from -2048 to 2045.\n");
	}
	CORETORTC();
}

void CORETORTC(void)
{
	//WAITBUSY();	
	RTC_WRITE32(RTC_WRTGR,0x01);
	//WAIT_COND(!(RTC_READ32(RTC_BBPU) & 0x40),TIMEOUT_LIMIT, OALMSG(1,(L"Core to RTC fail\r\n")));
	WAITBUSY();
}

void RTCTOCORE(void)
{
  //WAITBUSY();
	RTC_WRITE32(RTC_BBPU,((RTC_READ32(RTC_BBPU)) | 0x4322));
  RTC_WRITE32(RTC_WRTGR,0x01);
	//WAIT_COND(!(RTC_READ32(RTC_BBPU) & 0x40),TIMEOUT_LIMIT,OALMSG(1,(L"RTC to Core fail\r\n")));
	WAITBUSY();
}


void SetRTCBitData(uint32_t addr,uint32_t start_bit, uint32_t BitNum,uint32_t InputData)
{
	uint32_t u4Tmp1 = 0;
	uint32_t u4Tmp2 = 0;
	uint32_t i;
	u4Tmp1 = IO_READ32(IO_BASE,addr);
	for(i = 0; i < BitNum; i++)
	{
		u4Tmp2 |= (1 << i); 	
	}
	u4Tmp2 <<= start_bit;
	u4Tmp1 &= (~u4Tmp2);
	InputData <<= start_bit;
	u4Tmp1 |= InputData;
	u4Tmp1 |= 0x02;
	IO_WRITE32(IO_BASE,addr, u4Tmp1);
	//CORETORTC();
}



void RtcGPIOCmd(void)
{
	SetRTCBitData(0x5c,0,3,0);
	SetRTCBitData(0x5c,26,3,0);
	SetRTCBitData(0x60,24,2,0);
	SetRTCBitData(0x60,28,2,0);
	SetRTCBitData(0x64,16,1,0);
	SetRTCBitData(0x64,10,1,1);
}




