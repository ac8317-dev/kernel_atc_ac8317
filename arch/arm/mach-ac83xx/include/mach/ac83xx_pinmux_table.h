/*
 * arch/arm/mach-ac83xx/include/mach/ac83xx_pinmux_table.h
 *
 * Copyright (C) 2011 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */
#ifndef __AC83XX_PINMUX_TABLE_H
#define __AC83XX_PINMUX_TABLE_H

#include <linux/types.h>
#include "ac83xx_gpio_pinmux.h"

#define PINMUX_LEVEL_GPIO_END_FLAG 0xFF
#define PINMUX_LEVEL_INVALID_FLAG 0xFE
#define PINMUX_GROUP_INVALID_FLAG 0xFE
#define PINMUX_GPIO_LEVEL_MAX 12
#define PINMUX_GPIO_PARAM 2
#define GPIO_PINMUX_TOTAL_FUNC (PINMUX_GPIO_PARAM*PINMUX_GPIO_LEVEL_MAX)
#define INVALID_PUD_GPIO 0xFF
#define INVALID_PIN_FUNCTION 0xFF

//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------
//GPIO 47,68,69,174,175,195 need tread special
static const uint8_t _au1PinmuxFunctionSel[205][GPIO_PINMUX_TOTAL_FUNC] =
{
    //#define PIN_GPIO0           0  //DONE
    {   
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        PWM0_SEL,                                  PINMUX_FUNCTION2,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO1           1 //Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        PWM1_SEL,                                  PINMUX_FUNCTION2,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO2           2//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        PWM2_SEL,                                  PINMUX_FUNCTION1,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO3           3//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION3,
        LVDS_SEL,                                  PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO4           4  //doing
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION2,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION2,
        //DVD_GPIO_SEL0,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG 
    },
    //#define PIN_GPIO5           5//doing
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION2,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION2,
        //DVD_GPIO_SEL1,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO6           6//doing
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION3,
        //DVD_GPIO_SEL2,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO7           7//doing
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION3,
        //DVD_GPIO_SEL3,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN0_L              8//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION2,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN0_R                  9//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION2,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN1_L                 10//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION2,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN1_R                 11//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION2,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN2_L                 12//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION3,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION3,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN2_R                 13//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION3,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION3,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN3_L                 14//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION3,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION3,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_ANI3_R                 15//done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION3,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION3,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN4_L                 16//done
    {
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AIN4_R                 17//done
    {
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AL0                    18//DONE
    {
        I2S_OUT0_SEL,                               PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AL1                    19//Done
    {
        I2S_OUT0_SEL,                               PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AL2                    20//Done
    {
        I2S_OUT1_SEL,                               PINMUX_FUNCTION3,
        I2S_OUT0_SEL,                               PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AL3                    21//Done
    {
        I2S_OUT1_SEL,                               PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AL4                    22//Done
    {
        AMUTE_F_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AMUTE_F                23//Done    
    {
        AMUTE_R_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    }, 
    //#define PIN_AMUTE_GPS              24 //done    
    {
        I2S_OUT0_SEL,                              PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AMUTE_R                25 //Done
    {
        I2S_OUT0_SEL,                               PINMUX_FUNCTION2,
        //
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AR0                    26//Done
    {
        I2S_OUT1_SEL ,                              PINMUX_FUNCTION3,
        I2S_OUT0_SEL ,                              PINMUX_FUNCTION2,
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AR1                    27//Done
    {
        I2S_OUT1_SEL ,                              PINMUX_FUNCTION3,
        //
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO28                 28//Done
    {
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO29                 29 //Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        //
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO30                 30 //Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        CA7_DFD_SEL,                               PINMUX_FUNCTION1,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION3,
        PT110_EINT4_SEL,                           PINMUX_FUNCTION2,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO31                 31//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION3,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },

    //#define PIN_AR2                    32//Done
    {
        TTL_DE_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AR3                    33//Done
    {
        PWM3_SEL,                                  PINMUX_FUNCTION2,
        CCIR601_TIMING_DEIN_SEL,                   PINMUX_FUNCTION1,
        //
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_AR4                    34//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION4,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION4,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION4,
        TS_SEL3,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DCLK                   35//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION2,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION2,
        CA7_DFD_SEL,                               PINMUX_FUNCTION3,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION1,
        EINT0_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DCLK_IN                36//DONE
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION2,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION2,
        CA7_DFD_SEL,                               PINMUX_FUNCTION3,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION1,
        EINT1_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DE                     37//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION2,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION2,
        CA7_DFD_SEL,                               PINMUX_FUNCTION3,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION1,
        EINT2_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DE_IN                  38    //Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION2,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION2,
        CA7_DFD_SEL,                               PINMUX_FUNCTION3,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION1,
        EINT3_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DEMOD_RST              39 //Done 
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION6,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION6,
        CA7_DFD_SEL,                               PINMUX_FUNCTION1,
        HDMI_CEC_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DVD_SF_CK              40//going
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION6,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION6,
        CA7_DFD_SEL,                               PINMUX_FUNCTION1,
        HDMI_HDP_SEL,                              PINMUX_FUNCTION1,// HDP or HPD
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DVD_SF_CS              41//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION6,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION6,
        CA7_DFD_SEL,                               PINMUX_FUNCTION1,
        HDMI_I2C_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO42                 42//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        EINT7_SEL,                                 PINMUX_FUNCTION3,
        PWM2_SEL,                                  PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO43                 43//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO44                 44//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        CA7_DFD_SEL,                               PINMUX_FUNCTION3,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION1,
        T8032_EINT2_SEL,                           PINMUX_FUNCTION3,
        LVDS_SEL,                                  PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_DVD_SF_DI              45 //Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION6,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION6,
        CA7_DFD_SEL,                               PINMUX_FUNCTION1,
        HDMI_I2C_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
       //                          46//Done    
    {
       // DVD_RS232_SEL,                          PINMUX_FUNCTION10,//MOre than 7
       //AP_RS232_SEL,                            PINMUX_FUNCTION10,//MOre than 7
       DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
       AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
       UART5_SEL,                                 PINMUX_FUNCTION2,
       CCIR601_TIMING_VSIN_SEL,                   PINMUX_FUNCTION1,
       PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_EINT0                   47 //Done
    {
        // DVD_RS232_SEL,                          PINMUX_FUNCTION10,//MOre than 7
        //AP_RS232_SEL,                            PINMUX_FUNCTION10,//MOre than 7
        DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
        AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
        UART5_SEL,                                 PINMUX_FUNCTION2,
        CCIR601_TIMING_HSIN_SEL,                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_EINT1                  48//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION1,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION1,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION1,
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION3,
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,
        //TEST_ABIST_SEL,                          PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_EINT2                  49//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION1,
           I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION1,
           I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION1,
           SGM_MIC_IN_SEL,                            PINMUX_FUNCTION3,
           I2S_OUT0_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_EINT3                  50//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION1,
           I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION1,
           I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION1,
           I2S_OUT1_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_HSYNC                  51//Done      
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION1,
           I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION1,
           I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION1,
           SGM_MIC_IN_SEL,                            PINMUX_FUNCTION3,
           I2S_OUT1_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_HSYNC_IN               52 //Done       
    {
        I2S_OUT1_SEL,                              PINMUX_FUNCTION2,
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO53                 53//done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION1,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION1,
        CA7_DFD_SEL,                               PINMUX_FUNCTION2,
        T8032_EINT2_SEL,                           PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO54                 54//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_TEST_CPUM,                             PINMUX_FUNCTION1,
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION1,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION1,
        CA7_DFD_SEL,                               PINMUX_FUNCTION2,
        T8032_EINT3_SEL,                           PINMUX_FUNCTION2,
        PT110_EINT1_SEL,                           PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO55                 55//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_TEST_CPUM,                             PINMUX_FUNCTION1,
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION1,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION1,
        CA7_DFD_SEL,                               PINMUX_FUNCTION2,
        T8032_EINT4_SEL,                           PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO56                 56//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_TEST_CPUM,                             PINMUX_FUNCTION1,
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION1,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION1,
        CA7_DFD_SEL,                               PINMUX_FUNCTION2,
        T8032_EINT5_SEL,                           PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO57                 57//Done
    {
        PINMUX_LEVEL_INVALID_FLAG,                 PINMUX_GROUP_INVALID_FLAG,
        DVD_TEST_CPUM,                             PINMUX_FUNCTION1,
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION1,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION1,
        CA7_DFD_SEL,                               PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_IR                     58 //dONE     
    {
        I2S_OUT1_SEL,                              PINMUX_FUNCTION2,
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,        
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_0_CLK             59//Done
    {
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,    
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_0_CMD             60 //Done 
    {
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,    
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_0_DAT0            61//Done
    {
        I2S_OUT1_SEL,                              PINMUX_FUNCTION2,            
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_0_DAT1            62 //Done
    {
        I2S_OUT1_SEL,                              PINMUX_FUNCTION2,            
        I2S_OUT0_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define IR                        63//Done
    {
        PT110_EINT1_SEL,                           PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_0_DAT3            64 //Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION6,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION6,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION6,
        EINT7_SEL,                                 PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_CLK             65//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION6,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION6,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION6,
        EINT6_SEL,                                 PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_CMD             66//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION6,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION6,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION6,
        EINT5_SEL,                                 PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_DAT0            67//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION6,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION6,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION6,
        EINT4_SEL,                                 PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_DAT1            68//Done
    {
        // DVD_RS232_SEL,                          PINMUX_FUNCTION9,//MOre than 7
        //AP_RS232_SEL,                            PINMUX_FUNCTION9,//MOre than 7
        DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
        AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_DAT2            69//Done
    {
        //DVD_RS232_SEL,                           PINMUX_FUNCTION9,//MOre than 7
        //AP_RS232_SEL,                            PINMUX_FUNCTION9,//MOre than 7
        DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
        AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO70                 70//dONE
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION2,
        EINT4_SEL,                                 PINMUX_FUNCTION3,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION3,        
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO71                 71//DONE
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION2,
        EINT3_SEL,                                 PINMUX_FUNCTION3,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION3, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO72                 72//Done
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION1,
        EINT6_SEL,                                 PINMUX_FUNCTION3,
        T8032_EINT4_SEL,                           PINMUX_FUNCTION3, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_1_DAT3            73//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION5,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION5,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO74                 74//DONE
    {
        T8032_EINT1_SEL,                           PINMUX_FUNCTION2,
        T8032_EINT3_SEL,                           PINMUX_FUNCTION3,
        RTC_OUT_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_2_CLK             75//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION5,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION5,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_2_CMD             76//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION5,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION5,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_2_DAT0            77//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION5,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION5,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_2_DAT1            78//Done
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_MSDC_2_DAT2            79//Done
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION3,
        MHL_SENSE_SEL,                             PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //------------------------------------------------------------------------
    //#define                            80//DONE
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFALE                  81//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFCEN0                 82//Done  
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFCEN1                 83 //Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFCLE                  84//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFRBN                  85//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFRBN2                 86//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFREN                  87//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NFWEN                  88//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD0                   89 //Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD1                   90//Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD2                   91 //Done
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //----------------------------------------------------------------------
    //#define PIN_NLD3                   92//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,                 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD4                   93//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD5                   94 //Going
    {
       // NAND_FLASH_2ND_SEL,                      PINMUX_FUNCTION1,//not find
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD6                   95 //Done 
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_NLD7                   96//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    
    //#define PIN_PCM_CLK                97//Going
    {
        // NAND_FLASH_2ND_SEL,                     PINMUX_FUNCTION1,//not find
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_PCM_IN                 98//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_PCM_OUT                99//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SYNC                  100//Done    
    
    {
        SP0_SEL,                                   PINMUX_FUNCTION3,
        SP1_SEL,                                   PINMUX_FUNCTION3,
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
   
    //#define PIN_SCL0                  101//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION3,
        SP1_SEL,                                   PINMUX_FUNCTION3,
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SCL1                  102//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION3,
        SP1_SEL,                                   PINMUX_FUNCTION3,
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SDA0                  103//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION3,
        SP1_SEL,                                   PINMUX_FUNCTION3,
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },

    //#define PIN_SDA1                  104//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SP0_CLK               105//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SP0_CS                106//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SP0_SI                107//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SP0_SO                108//Done
    {
        PCM_SEL,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SP1_CLK               109//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },   
    //#define PIN_SP1_CS                110//Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },   
    //#define PIN_SP1_SI                111 //Done
    {
        NAND_FLASH_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },   
    //#define PIN_SP1_SO                112//Done
    {
        USB_I2C_SEL,                               PINMUX_FUNCTION1,
        I2C0_SEL,                                  PINMUX_FUNCTION1,
        I2C1_SEL,                                  PINMUX_FUNCTION2,
        PT110_EINT2_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },   
    //#define PIN_SPDIF                 113//Done
    {
        USB_I2C_SEL,                               PINMUX_FUNCTION2,
        HDMI_I2C_SEL,                              PINMUX_FUNCTION2,
        VGA_I2C_SEL,                               PINMUX_FUNCTION2,
        I2C0_SEL,                                  PINMUX_FUNCTION2,
        I2C1_SEL,                                  PINMUX_FUNCTION1,
        PT110_EINT2_SEL,                           PINMUX_FUNCTION2,
        PT110_EINT4_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_CLKIN              114//Done
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION2,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D0                 115//Done
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION2,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D1                 116//Done
    {
        CA7_MBIST_SEL,                             PINMUX_FUNCTION2,
        CA7_MBIST_SEL,                             PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D2                 117//Done
    {
        USB_I2C_SEL,                               PINMUX_FUNCTION1,
        I2C0_SEL,                                  PINMUX_FUNCTION1,
        I2C1_SEL,                                  PINMUX_FUNCTION2,
        PT110_EINT3_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D3                 118//Done
    {
        USB_I2C_SEL,                               PINMUX_FUNCTION2,
        HDMI_I2C_SEL,                              PINMUX_FUNCTION2,
        VGA_I2C_SEL,                               PINMUX_FUNCTION2,
        I2C0_SEL,                                  PINMUX_FUNCTION2,
        I2C1_SEL,                                  PINMUX_FUNCTION1,
        PT110_EINT3_SEL,                           PINMUX_FUNCTION2,   
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D4                 119//Done
    
    {
        SP0_SEL,                                   PINMUX_FUNCTION1,
        SP1_SEL,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D5                 120//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION1,
        SP1_SEL,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D6                 121//done
    {
        SP0_SEL,                                   PINMUX_FUNCTION1,
        SP1_SEL,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_D7                 122//DONE
    {
        SP0_SEL,                                   PINMUX_FUNCTION1,
        SP1_SEL,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_DEN                123//Done
    {
        AP_SF_SEL,                                 PINMUX_FUNCTION1,
        SP0_SEL,                                   PINMUX_FUNCTION4,
        SP1_SEL,                                   PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO124               124//Done
    {
        UART3_SEL,                                 PINMUX_FUNCTION2,
        PWM2_SEL,                                  PINMUX_FUNCTION1,
        T8032_EINT1_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO_125              125//Done
    {
        UART3_SEL,                                 PINMUX_FUNCTION2,
        PWM3_SEL,                                  PINMUX_FUNCTION1,
        T8032_EINT2_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_TS_SYNC               126//Done
    {
        AP_SF_SEL,                                 PINMUX_FUNCTION1,
        SP0_SEL,                                   PINMUX_FUNCTION4,
        SP1_SEL,                                   PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_URXD0                 127//Done
    {
        AP_SF_SEL,                                 PINMUX_FUNCTION1,
        SP0_SEL,                                   PINMUX_FUNCTION4,
        SP1_SEL,                                   PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },

    //#define PIN_URXD1                 128//Done
    {
        AP_SF_SEL,                                 PINMUX_FUNCTION1,
        SP0_SEL,                                   PINMUX_FUNCTION4,
        SP1_SEL,                                   PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_URXD2                 129//Done
    {
        SPDIF_SEL,                                 PINMUX_FUNCTION1,
        HDMI_SPDIF_SEL,                            PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_URXD3                 130//Done
    { 
        SP0_SEL,                                   PINMUX_FUNCTION5,
        SP1_SEL,                                   PINMUX_FUNCTION5,
        TS_SEL0,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_URXD4                 131//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION5,
        SP1_SEL,                                   PINMUX_FUNCTION5,
        TS_SEL0,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_URXD5                 132//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION7,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION7,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION7,
        SP0_SEL,                                   PINMUX_FUNCTION2,
        SP1_SEL,                                   PINMUX_FUNCTION2,
        TS_SEL1,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_USB_DM_P0             133 //Done   
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION2,
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION7,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION7,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION7,
        SP0_SEL,                                   PINMUX_FUNCTION2,
        SP1_SEL,                                   PINMUX_FUNCTION2,
        TS_SEL1,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },         
    //#define PIN_USB_DM_P1             134 //Done
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION2,
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION7,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION7,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION7,
        SP0_SEL,                                   PINMUX_FUNCTION2,
        SP1_SEL,                                   PINMUX_FUNCTION2,
        TS_SEL1,                                   PINMUX_FUNCTION1,
        T8032_EINT1_SEL,                           PINMUX_FUNCTION3,
        TEST_IN_SEL,                               PINMUX_FUNCTION1, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },         
    //#define PIN_USB_DP_P0             135 //Done
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION2,
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION7,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION7,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION7,
        SP0_SEL,                                   PINMUX_FUNCTION2,
        SP1_SEL,                                   PINMUX_FUNCTION2,
        TS_SEL1,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },         
    //#define PIN_USB_DP_P1             136 //Done
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION2,
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION4,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION4,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION4,
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION1,
        TS_SEL2,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },         
    //#define PIN_UTXD0                 137//Done
    {
        DVD_TEST_CPUM,                             PINMUX_FUNCTION2,
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION4,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION4,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION4,
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION1,
        TS_SEL2,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_UTXD1                 138//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION4,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION4,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION4,
        SGM_MIC_IN_SEL,                            PINMUX_FUNCTION1,
        TS_SEL2,                                   PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_UTXD2                 139//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION5,
        SP1_SEL,                                   PINMUX_FUNCTION5,
        TS_SEL0,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_UTXD3                 140//Done
    {
        SP0_SEL,                                   PINMUX_FUNCTION5,
        SP1_SEL,                                   PINMUX_FUNCTION5,
        TS_SEL0,                                   PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD0                 141//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION1,
        AP_RS232_SEL,                              PINMUX_FUNCTION1,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART0_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD1                142//Done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION2,
        AP_RS232_SEL,                              PINMUX_FUNCTION2,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART1_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION1,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION1,        
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD2                   143//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION3,
        AP_RS232_SEL,                              PINMUX_FUNCTION3,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART2_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD3                  144//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION4,
        AP_RS232_SEL,                              PINMUX_FUNCTION4,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART3_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD4                   145//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION5,
        AP_RS232_SEL,                              PINMUX_FUNCTION5,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART4_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //URXD5                   146 //done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION6,
        AP_RS232_SEL,                              PINMUX_FUNCTION6,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART5_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VB4                   147//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION4,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VB5                   148//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION4,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO149               149//Done
    {
        T8032_EINT3_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO150               150//done
    {
        PWM1_SEL,                                  PINMUX_FUNCTION1,
        T8032_EINT3_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO151               151//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION4,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    
    //#define PIN_VB6                   152
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION4,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION4,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD0                  153//Done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION1,
        AP_RS232_SEL,                              PINMUX_FUNCTION1,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART0_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP0_SEL,                         PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD1                   154//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION2,
        AP_RS232_SEL,                              PINMUX_FUNCTION2,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART1_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP1_SEL,                         PINMUX_FUNCTION1,
        DVD_T8032_UP3_SEL,                         PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD2                   155//DONE
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION3,
        AP_RS232_SEL,                              PINMUX_FUNCTION3,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART2_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD3                   156//Done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION4,
        AP_RS232_SEL,                              PINMUX_FUNCTION4,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART3_SEL,                                 PINMUX_FUNCTION1,
        DVD_T8032_UP2_SEL,                         PINMUX_FUNCTION2,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD4                   157//done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION5,
        AP_RS232_SEL,                              PINMUX_FUNCTION5,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART4_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //UTXD5                   158 //done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION6,
        AP_RS232_SEL,                              PINMUX_FUNCTION6,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        UART5_SEL,                                 PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VG5                   159//Done
    {   
        PWM0_SEL,                                  PINMUX_FUNCTION3,
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VG6                   160//Done
    {
        PWM1_SEL,                                  PINMUX_FUNCTION3,
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VG7                   161//Done
    {
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO162               162//Done
    {
        PWM0_SEL,                                  PINMUX_FUNCTION1,
        PT110_EINT1_SEL,                           PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN0                  163//Done
    {
        PWM3_SEL,                                  PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN1                  164//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION5,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION5,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION5,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },          
    //#define PIN_VIN2                  165  //Done 
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION5,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION5,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION5,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },          
    //#define PIN_VIN3                  166//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION5,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION5,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION5,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN4                  167//Done
    {
        I2S_MIC_IN_SEL,                            PINMUX_FUNCTION5,
        I2S_LINE1_IN_SEL,                          PINMUX_FUNCTION5,
        I2S_LINE0_IN_SEL,                          PINMUX_FUNCTION5,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN5                  168//Done
    {
        EINT4_SEL,                                 PINMUX_FUNCTION1,
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN6                  169//Done
    {
        EINT5_SEL,                                 PINMUX_FUNCTION1,
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VIN7                  170//Done
    {
        EINT6_SEL,                                 PINMUX_FUNCTION1,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR0                   171//Done
    {
        EINT7_SEL,                                 PINMUX_FUNCTION1,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR1                   172//Done
    {
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        LVDS_SEL,                                  PINMUX_FUNCTION2, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR2                   173//Done
    {
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        LVDS_SEL,                                  PINMUX_FUNCTION2, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR3                   174//DONE
    {
       // DVD_RS232_SEL,                           PINMUX_FUNCTION8,//MOre than 7
       //AP_RS232_SEL,                             PINMUX_FUNCTION8,//MOre than 7
        DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
        AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        LVDS_SEL,                                  PINMUX_FUNCTION3, 
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR4                   175//DONE
    {
        // DVD_RS232_SEL,                          PINMUX_FUNCTION8,//MOre than 7
        //AP_RS232_SEL,                            PINMUX_FUNCTION8,//MOre than 7
        DVD_RS232_SEL_ADDITION,                    PINMUX_FUNCTION1,//MOre than 7, So need fix DVD_RS232_SEL
        AP_RS232_SEL_ADDITION,                     PINMUX_FUNCTION1,//MOre than 7, So need fix AP_RS232_SEL
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        LVDS_SEL,                                  PINMUX_FUNCTION3,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR5                   176//Done
    {
        VGA_I2C_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR6                   177//Done
    {
        VGA_I2C_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VR7                   178//Done
    {
        UART0_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VSYNC                 179//Done
    {
        UART0_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_VSYNC_IN              180//Done
    {
        UART1_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },

    //#define PIN_SCL0_IN         181//Done
    {
        UART1_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SDA0_IN         182 //Done
    {
        UART2_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SCL1_IN         183//Done
    {
        UART2_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,       
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_SDA1_IN         184//Done
    {
        UART4_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,        
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO150_IN      185//Done
    {
        UART4_SEL,                                 PINMUX_FUNCTION2,
        CCIR656_601_DATAIN_SEL,                    PINMUX_FUNCTION1,
        TEST_IN_SEL,                               PINMUX_FUNCTION1,  
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO151_IN      186//Done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION7,
        AP_RS232_SEL,                              PINMUX_FUNCTION7,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //                              187//Done
    {
        DVD_RS232_SEL,                             PINMUX_FUNCTION7,
        AP_RS232_SEL,                              PINMUX_FUNCTION7,
        DVD_RS232_SEL_ADDITION_CLEAR,              PINMUX_FUNCTION0,//
        AP_RS232_SEL_ADDITION_CLAER,               PINMUX_FUNCTION0,//
        TTL_8B_SEL,                                PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //#define PIN_GPIO149_IN      188//Done
    {
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                189//Done
    {   
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION3,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                      190//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION3,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                   191 //Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION3,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              192 //Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION3,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              193//Done
    {
        ARM11_JTAG_SEL,                            PINMUX_FUNCTION3,
        ARM9_JTAG_SEL,                             PINMUX_FUNCTION3,
        TTL_6_8B_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              194//Done
    {
        TTL_SYNC_SEL,                              PINMUX_FUNCTION1,
        TEST_OUT_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
   //#define PIN_DVD_SF_DO                   195//Done
    {
        TTL_SYNC_SEL,                              PINMUX_FUNCTION1,
        TEST_BUS_SEL,                              PINMUX_FUNCTION1,
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //                                              196// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
    //                                          197// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              198// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              199// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              200 // not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              201// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              202// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              203// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    },
//                                              204// not GPIO
    {
        PINMUX_LEVEL_GPIO_END_FLAG,                PINMUX_GROUP_INVALID_FLAG
    }
       
};


static const uint8_t _au1PinmuxFunctionMasks[256] = 
{
//--------------0x54--------------------
  7,  0,  0,  3,  0,  0,  0,  0,  1,  0, 
  0,  0,  0,  0,  0,  0,  0,  0,  3,  0,
  0,  0,  1,  0,  1,  1,  1,  1,  7,  0,
  0,  0,  
//--------------0x58--------------------
  3,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  7,  0,  0,  7,  0,
  0,  1,  1,  0,  1,  7,  0,  0,  3,  0,
  0,  0,
//--------------0x5c--------------------
  3,  0,  0,  0,  1,  0,  1,  1,  1,  1,
  1,  0,  1,  0,  1,  0,  7,  0,  0,  7,
  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,
  1,  1,
//--------------0x60--------------------
  1,  0,  0,  0,  0,  0,  0,  0,  3,  0,
  1,  0,  3,  0,  0,  0,  3,  0,  3,  0,
  0,  0,  3,  0,  3,  0,  3,  0,  3,  0,
  3,  0,
//--------------0x64--------------------
  3,  0,  3,  0,  3,  0,  3,  0,  3,  0,
  1,  1,  1,  0,  0,  0,  0,  0,  1,  1,
  0,  0,  0,  0,  3,  0,  0,  0,  0,  0,
  0,  0,
//--------------0x68--------------------
  1,  0,  1,  0,  1,  0,  1,  0,  3,  0,
  0,  0,  3,  0,  0,  0,  3,  0,  0,  3,
  0,  0,  3,  0,  0,  3,  0,  0,  0,  0,
  0,  0,
//--------------0x6c--------------------
  0,  0,  0,  3,  0,  0,  0,  0,  0,  0,
  0,  3,  0,  3,  0,  0,  3,  0,  3,  0,
  3,  0,  0,  3,  0,  0,  3,  0,  0,  0,
  0,  0,
//--------------0x70--------------------
  3,  0,  0,  1,  1,  1,  3,  0,  1,  0,
  0,  1,  3,  0,  3,  0,  7,  0,  0,  0,
  7,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0
};

//137 pad_port not find
static const uint8_t _au1PinPullUDOffset[205] =
{
   //0~31 
    0, 1, 2, 3, 4, 5, 6, 7, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,    
    255, 255, 123, 124, 255, 255, 255, 255, 63, 109,
    8, 9,
   //32~63
    62, 108, 29, 66, 67, 68, 69, 112, 115, 113,   //41 
    10, 11, 12, 114, 110, 111, 119, 121, 120, 118, //51
    125, 13, 14, 15, 16, 17, 126, 127, 128, 129,   //61
    130, 255,
   //64~95
    255, 255, 255, 255, 255, 255, 18, 19, 20, 255,  //73
    21, 255, 255, 255, 117, 116, 255, 255, 255, 255,//83
    255, 255, 255, 255, 255, 255, 255, 255, 86, 84, //93
    85, 255, 
   //96~127
    83, 255, 82, 87, 255, 255, 255, 255, 255, 255,  //105
    255, 255, 72, 71, 73, 70, 133, 135, 255, 255,   //115
    255, 134, 136, 74, 77, 75, 76, 78, 22, 23,      //125
    81, 79,
   //128~159
    80, 122, 26, 30, 31, 32, 33, 34, 35, 36,        //137
    37, 27,  28, 88, 89, 90, 91, 92, 93, 255,//147
    255, 255, 24, 255, 255, 94, 95, 96, 97, 98,//157
    99, 54,
   //160~191
    55, 56, 25, 57, 58, 59, 60, 61, 46, 47,//169
    48, 49, 50, 51, 52, 53, 132, 131, 100, 101,//179
    102, 103, 104, 105, 106, 107, 38, 39, 40, 41,//189
    42, 43,
   //192~204
    44, 45, 65, 64, 255, 255, 255, 255, 255, 255,
    255, 255, 255
};


static const uint8_t _au1IntSel[205][3] =
{
    //#define PIN_GPIO0           0  //DONE
    {0,0,0},{1,0,1},{2,0,2},{3,0,3},{4,0,4},{5,0,5},{6,0,6},{7,0,7},{8,0,8},{9,0,9},
    {10,0,10},{11,0,11},{12,0,12},{13,0,13},{14,0,14},{15,0,15},{16,0,16},{17,0,17},{18,0,18},{19,0,19},
    {20,0,20},{21,0,21},{22,0,22},{23,0,23},{24,0,24},{25,0,25},{26,0,26},{27,0,27},{28,0,63},{29,1,6},
    {30,0,28},{31,0,29},{32,0,64},{33,1,7},{34,0,30},{35,1,10},{36,1,11},{37,1,12},{38,1,13},{39,2,6},
    {40,2,7},{41,2,8},{42,0,31},{43,0,32},{44,0,33},{45,2,9},{46,0,65},{47,1,8},{48,1,18},{49,1,19},
    {50,1,20},{51,1,21},{52,1,22},{53,0,34},{54,0,35},{55,0,36},{56,0,37},{57,0,38},{58,1,23},{59,1,24},
    {60,1,25},{61,1,26},{62,1,27},{63,0,91},{64,0,51},{65,0,52},{66,0,53},{67,0,54},{68,0,49},{69,0,50},
    {70,0,39},{71,0,40},{72,0,41},{73,0,45},{74,0,42},{75,0,46},{76,0,47},{77,0,48},{78,0,43},{79,0,44},
    {80,1,28},{81,1,29},{82,1,30},{83,1,31},{84,1,32},{85,1,33},{86,1,34},{87,1,35},{88,1,36},{89,1,37},
    {90,1,38},{91,1,39},{92,1,47},{93,1,48},{94,1,49},{95,1,50},{96,1,51},{97,1,52},{98,1,53},{99,1,54},
    {100,1,55},{101,1,56},{102,1,57},{103,1,58},{104,1,59},{105,1,60},{106,1,61},{107,1,62},{108,1,63},{109,1,64},
    {110,1,65},{111,1,66},{112,1,14},{113,1,15},{114,0,55},{115,0,56},{116,0,57},{117,1,16},{118,1,17},{119,1,67},
    {120,1,68},{121,1,69},{122,1,70},{123,1,71},{124,0,58},{125,0,59},{126,1,72},{127,1,73},{128,1,74},{129,1,75},
    {130,1,76},{131,1,79},{132,1,80},{133,1,81},{134,1,82},{135,1,83},{136,1,84},{137,1,85},{138,1,86},{139,1,77},
    {140,1,78},{141,1,87},{142,1,88},{143,1,89},{144,1,90},{145,1,91},{146,1,92},{147,1,93},{148,1,94},{149,0,60},
    {150,0,61},{151,1,95},{152, 0,0},{153,2,0},{154,2,1},{155,2,2},{156,2,3},{157,2,4},{158,2,5},{159,0,67},
    {160,0,68},{161,0,69},{162,0,62},{163,0,70},{164,0,71},{165,0,72},{166,0,73},{167,0,74},{168,0,75},{169,0,76},
    {170,0,77},{171,0,78},{172,0,79},{173,0,80},{174,0,81},{175,0,82},{176,0,92},{177,0,93},{178,0,94},{179,0,95},
    {180,1,0},{181,1,1},{182,1,2},{183,1,3},{184,1,4},{185,1,5},{186,0,83},{187,0,84},{188,0,85},{189,0,86},
    {190,0,87},{191,0,88},{192,0,89},{193,0,90},{194,0,66},{195,1,9},{196,1,40},{197,1,41},{198,1,42},{199,1,43},
    {200,1,44},{201,1,45},{202,1,46},{203,2,10},{204,2,11}
   

};




//137 pad_port not find
static const uint8_t PIN_PULL_UP_OR_DOWN_OFFSET[205] =
{
   //0~31 
    0, 1, 2, 3, 4, 5, 6, 7, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,    
    255, 255, 123, 124, 255, 255, 255, 255, 63, 109,
    8, 9,
   //32~63
    62, 108, 29, 66, 67, 68, 69, 112, 115, 113,     //41 
    10, 11, 12, 114, 110, 111, 119, 121, 120, 118,  //51
    125, 13, 14, 15, 16, 17, 126, 127, 128, 129,    //61
    130, 255,
   //64~95
    255, 255, 255, 255, 255, 255, 18, 19, 20, 255,  //73
    21, 255, 255, 255, 117, 116, 255, 255, 255, 255,//83
    255, 255, 255, 255, 255, 255, 255, 255, 86, 84, //93
    85, 255, 
   //96~127
    83, 255, 82, 87, 255, 255, 255, 255, 255, 255,  //105
    255, 255, 72, 71, 73, 70, 133, 135, 255, 255,   //115
    255, 134, 136, 74, 77, 75, 76, 78, 22, 23,      //125
    81, 79,
   //128~159
    80, 122, 26, 30, 31, 32, 33, 34, 35, 36,        //137
    37, 27,  28, 88, 89, 90, 91, 92, 93, 255,       //147
    255, 255, 24, 255, 255, 94, 95, 96, 97, 98,     //157
    99, 54,
   //160~191
    55, 56, 25, 57, 58, 59, 60, 61, 46, 47,         //169
    48, 49, 50, 51, 52, 53, 132, 131, 100, 101,     //179
    102, 103, 104, 105, 106, 107, 38, 39, 40, 41,   //189
    42, 43,
   //192~204
    44, 45, 65, 64, 255, 255, 255, 255, 255, 255,   //201
    255, 255, 255
};

#endif /* __AC83XX_PINMUX_TABLE_H */

