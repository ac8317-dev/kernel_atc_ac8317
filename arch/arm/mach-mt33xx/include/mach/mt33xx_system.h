/*
 *  linux/include/asm-arm/arch-mt85xx/system.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */
#ifndef __ASM_ARCH_MT85xx_SYSTEM_H
#define __ASM_ARCH_MT85xx_SYSTEM_H

#include <mach/hardware.h>
#include <mach/ic_version.h>

#define		EXIT_ID									8530
#define		ENGINE_MASTER_T8032			1
#define 	ENGINE_ALL							0
#define 	ENGINE_ETHE							1
#define 	ENGINE_UART							2
#define 	ENGINE_CEC							3
#define 	SRAM_MASTER_T8032				0x1

#define 	REG_RW_WAKEN									0x080
#define 	REG_RW_WDTSET									0x004
#define 	REG_RW_WDT										0x008
#define		REG_RW_INTSTA									0x140
#define 	REG_RW_INTEN				  				0x144
#define 	REG_RW_INTCLR				  				0x148
#define 	REG_RW_UP_CFG	    						0x188
#define		REG_RW_ARM_IND_INT	    			0x1cc
#define		REG_RW_ARM_IND_DATA0					0x1c4

#define 	RW_ARM_IND_INT_ARM_INT				(1U << 0)
#define 	RW_ARM_IND_INT_AUXINT_STA		  (1U << 3)
#define 	RW_ARM_IND_INT_AUXINT_CLR		  (1U << 2)
#define 	RW_ARM_IND_INT_ARM_RW		  		(1U << 1)

#define   REG_RW_AUX_IND_DATA0	    	  0x1d0                     //DATA REGISTER 0 FROM T8032
    #define RW_AUX_IND_DATA0_AUX_DATA0_MASK       0x000000FF
    #define RW_AUX_IND_DATA0_AUX_DATA0_OFFSET	  0
    #define RW_AUX_IND_DATA0_BIT7                 (1U << 7)	//demo no use
    #define RW_AUX_IND_DATA0_BIT6                 (1U << 6)	//demo no use
    #define RW_AUX_IND_DATA0_BIT5                 (1U << 5)	//demo no use
    #define RW_AUX_IND_DATA0_KEY_HOME             (1U << 4)
    #define RW_AUX_IND_DATA0_KEY_DISC             (1U << 3)
    #define RW_AUX_IND_DATA0_KEY_PLAY             (1U << 2)
    #define RW_AUX_IND_DATA0_KEY_EJECT            (1U << 1)
    #define RW_AUX_IND_DATA0_KEY_POWER            (1U << 0)

#define 	RW_UP_CFG_ETNET_EN						28
#define 	RW_UP_CFG_DBG_UART_EN					24
#define 	RW_UP_CFG_FAST_CK_EN					20
#define 	RW_UP_CFG_ENGINE_EN						16
#define 	RW_UP_CFG_CEC_EN							12
#define 	RW_UP_CFG_RAM_CK_SEL_MASK     0x00000300
#define 	RW_UP_CFG_RAM_CK_SEL_OFFSET	  8
#define 	RW_UP_CFG_T8032_RST           (1U << 0)

#define 	RW_CLR_CFG_INT_RST						(1U << 20)

#define		RW_UP_CFG_CK_SEL							0x245

#define		STDBY_FILE_NAME			      		"/tmp/stdby_log.txt"
#define 	bl_ver_procfs_name							"bl_info"
#define	STDBY_CHECK_VALUE						0x24000164

#define 	FL_STATUS_A2_FINISHED					0x3
#define 	REG_RW_SINFOD_REG   					0x0234
#define		REG_RW_RESRV1									0x164

extern int mt85xx_bl_ver_proc_init(void);
extern bool BIM_GETHWSemaphore(uint32_t u4Number, uint32_t u4TimeOut);
extern bool BIM_ReleaseHWSemaphore(uint32_t u4Number);

#endif
