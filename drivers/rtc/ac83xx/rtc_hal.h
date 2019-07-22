/*****************************************************************************
 *
 * Filename:
 * ---------
 *   $Workfile: rtc_hal.h $
 *
 * Description:
 * ------------
 *   Real Time Clock header file
 *
 * Author:
 * -------
 *   
 *
****************************************************************************/
#ifndef _HW_RTC_HAL_H_
#define _HW_RTC_HAL_H_

#include <linux/types.h>
#include "rtc_hw.h"

//#include "x_timer.h"
//#include "x_debug.h"

/****************************************************************************
** Configuration
****************************************************************************/


// *********************************************************************
// Constant definitions
// *********************************************************************

//#define VECTOR_RTC         100

#define  FALSE 0
#define  TRUE  1

#define TIMEOUT_LIMIT		(1)   // 1 Seconds
#define RTCINTMAX           (3)




#define RTC_DEFAULT_YEAR   12         
#define RTC_DEFAULT_MONTH  3    
#define RTC_DEFAULT_DOW    5 
#define RTC_DEFAULT_DOM    23  
#define RTC_DEFAULT_HOUR   12         
#define RTC_DEFAULT_MIN    0         
#define RTC_DEFAULT_SEC    0  



#define MSG_LVL_OFF								0
#define MSG_LVL_FATAL							1
#define MSG_LVL_ERR								2
#define MSG_LVL_WARN							3
#define MSG_LVL_TITLE							4
#define MSG_LVL_INFO							5
#define MSG_LVL_CMD								6
#define MSG_LVL_DBG								7
#define MSG_LVL_TRACE							8
#define MSG_LVL_REGRW							9



#define YEA_MSK              (1 << 6)
#define MTH_MSK              (1 << 5)
#define DOW_MSK              (1 << 4)
#define DOM_MSK              (1 << 3)
#define HOU_MSK              (1 << 2)
#define MIN_MSK              (1 << 1)
#define SEC_MSK              (1 << 0)

#define AL_MSK              (YEA_MSK|MTH_MSK|DOW_MSK|DOM_MSK|HOU_MSK|MIN_MSK|SEC_MSK)


#define AL_EN                (1 << 0)
#define TC_EN                (1 << 1)
#define ONESHOT              (1 << 2)

#define SECCII_8_1           (1 << 9)
#define SECCII_4_1           (1 << 8)
#define SECCII_2_1           (1 << 7)
#define YEACII               (1 << 6)
#define MTHCII               (1 << 5)
#define DOWCII               (1 << 4)
#define DOMCII               (1 << 3)
#define HOUCII               (1 << 2)
#define MINCII               (1 << 1)
#define SECCII               (1 << 0)


extern uint32_t _u4RTC_DBG_LVL;
#if 0
#define MSG(lvl, formatStr, args...)\
{ \
	if (lvl <= _u4RTC_DBG_LVL) {\
		UTIL_Printf("[%d] ", lvl);\
		UTIL_Printf(formatStr, ##args);\
	}\
}

#endif

#if 1
#define WAIT_COND(cond, timeout, log)\
	do {\
		uint32_t rTimeStart = 0; \
	    uint32_t rTimeEnd = 0; \
	    uint32_t rTimeDelta = 0; \
	    rTimeStart = OEMGetTickCount();\
		while (1) {\
		   rTimeEnd = OEMGetTickCount();\
		   rTimeDelta = rTimeEnd - rTimeStart;\
			if (cond) break;\
			else if (rTimeDelta > timeout*1000) {\
					OALMSG(1, (L"Timeout in %s (line %d), timeout : %d sec\r\n", __FUNCTION__, __LINE__, timeout));\
					log;\
					break;\
			}\
		}\
	} while(0);
#endif

#define RTCDELAY(miniS) \
	do{ \
		uint32_t rTimeStart = 0; \
		uint32_t rTimeEnd = 0; \
		rTimeStart = OEMGetTickCount();\
		while(1) { \
			rTimeEnd = OEMGetTickCount();\
			if (rTimeEnd - rTimeStart > miniS) break; \
		} \
	}while(0) \

#if 0
#define WAITBUSY() \
	WAIT_COND(!(RTC_READ32(RTC_BBPU) & 0x40),TIMEOUT_LIMIT,\
		OALMSG(1,(L"RTC is always Busy.")))	

#else
#define WAITBUSY() \
{ \ 
	int count = 0; \
	mdelay(5); \
	while((RTC_READ32(RTC_BBPU) & 0x40)) \
	{ \
		count++; \
		if (count > 20) { \
			count = 0; \
			break; \
		} \
		mdelay(5); \
	} \
}
#endif



#define RTCClareAllInt() RTC_WRITE32(RTC_CII_EN,(RTC_READ32(RTC_CII_EN) & 0xFFFFFC00));
	                     //CORETORTC()
	                     
#define RTCClareAllAlarmInt() RTC_WRITE32(RTC_AL_MASK,(RTC_READ32(RTC_AL_MASK) | 0x3F));
	                     //CORETORTC()

extern uint32_t OEMGetTickCount(void);

// *********************************************************************
// Function Prototype
// *********************************************************************
extern uint32_t u4RTC_DBG_LVL;  // Log Level contorll
extern int g_bFirstBooting;
extern uint32_t RtcGetDeviceInfo (void);
extern void RTCSecSet(int32_t bSec);
extern void RTCMinSet(int32_t bMin);
extern void RTCHourSet(int32_t bHour);
extern void RTCDOMSet(int32_t bDay);
extern void RTCDOWSet(int32_t bDay);
extern void RTCMonthSet(int32_t bMonth);
extern void RTCYearSet(int32_t bYear);
extern int32_t RTCSecRead(void);
extern int32_t RTCMinRead(void);
extern int32_t RTCHourRead(void);
extern int32_t RTCDOMRead(void);
extern int32_t RTCDOWRead(void);
extern int32_t RTCMonthRead(void);
extern int32_t RTCYearRead(void);
extern void RTCAlarmSecSet(int32_t bSec);
extern void RTCAlarmMinSet(int32_t bMin);
extern void RTCAlarmHourSet(int32_t bHour);
extern void RTCAlarmDOMSet(int32_t bDay);
extern void RTCAlarmDOWSet(int32_t bDay);
extern void RTCAlarmMonthSet(int32_t bMonth);
extern void RTCAlarmYearSet(int32_t bYear);
extern int32_t RTCAlarmSecRead(void);
extern int32_t RTCAlarmMinRead(void);
extern int32_t RTCAlarmHourRead(void);
extern int32_t RTCAlarmDOMRead(void);
extern int32_t RTCAlarmDOWRead(void);
extern int32_t RTCAlarmMonthRead(void);
extern int32_t RTCAlarmYearRead(void);
extern void RTCAlarmAllCMPR(int32_t bEnable);
extern void RTCAlarmYearCMPR(int32_t bEnable);
extern void RTCAlarmMonthCMPR(int32_t bEnable);
extern void RTCAlarmDowCMPR(int32_t bEnable);
extern void RTCAlarmDomCMPR(int32_t bEnable);
extern void RTCAlarmHourCMPR(int32_t bEnable);
extern void RTCAlarmMinCMPR(int32_t bEnable);
extern void RTCAlarmSecCMPR(int32_t bEnable);
extern void RTCEnableAlarmInt(int32_t bEnable);
extern void RTCYearInt(int32_t bEnable);
extern void RTCMonthInt(int32_t bEnable);
extern void RTCDowInt(int32_t bEnable);
extern void RTCDomInt(int32_t bEnable);
extern void RTCHourInt(int32_t bEnable);
extern void RTCMinInt(int32_t bEnable);
extern void RTCSecInt(int32_t bEnable);
extern void RTCSecInt_8_1(int32_t bEnable);
extern void RTCSecInt_4_1(int32_t bEnable);
extern void RTCSecInt_2_1(int32_t bEnable);
extern void RTCIntAutoReset(int32_t bEnable);
extern void RtcIsr(uint16_t u2Vector);
extern void RTCEnableTCInt(int32_t bEnable);
extern int32_t RegRtcInt(int32_t bEnable);
extern int32_t RtcFirstRunChk(void);
extern int RTCHWInit(void);
extern void RTCProtect(int32_t bEnable);
extern void RTCTrim(int32_t TrimData);
extern void CORETORTC(void);
extern void RTCTOCORE(void);
extern void SetRTCBitData(uint32_t addr,uint32_t start_bit, uint32_t BitNum,uint32_t InputData);
extern void RtcGPIOCmd(void);
extern void RTCClearIrqSta(void);

#endif 
