/*
 * arch/arm/mach-mt85xx/include/mach/mt8555_gpio_pinmux.h
 *
 * Copyright (C) 2010 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */

#ifndef __MT3360_GPIO_PINMUX_H
#define __MT3360_GPIO_PINMUX_H

//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------
//CKGEN
//#define PAD_CFG_0               0x0C0
#define GPIO_EN0_OFFSET         0x074 
#define GPIO_OUT0_OFFSET        0x0E0
#define GPIO_IN0_OFFSET         0x100
//#define PAD_FECTL_0             0x280
//#define PAD_PWMCTL_0            0x288

#define TOTAL_GPIO_IDX          5
#define GPIO_INDEX_MASK         ((1 << 5) - 1)

// GPIO name definitions
#define PIN_GPIO0           0
#define PIN_GPIO1           1
#define PIN_GPIO2           2
#define PIN_GPIO3           3
#define PIN_GPIO4           4
#define PIN_GPIO5           5
#define PIN_GPIO6           6
#define PIN_GPIO7           7
#define PIN_AIN0_L                  8
#define PIN_AIN0_R                  9
#define PIN_AIN1_L                 10
#define PIN_AIN1_R                 11
#define PIN_AIN2_L                 12
#define PIN_AIN2_R                 13
#define PIN_AIN3_L                 14
#define PIN_ANI3_R                 15
#define PIN_AIN4_L                 16
#define PIN_AIN4_R                 17
#define PIN_AL0                    18
#define PIN_AL1                    19
#define PIN_AL2                    20
#define PIN_AL3                    21
#define PIN_AL4                    22
#define PIN_AMUTE_F                23     
#define PIN_AMUTE_GPS              24     
#define PIN_AMUTE_R                25
#define PIN_AR0                    26
#define PIN_AR1                    27
#define PIN_GPIO28                 28
#define PIN_GPIO29                 29       
#define PIN_GPIO30                 30 
#define PIN_GPIO32                 31

#define PIN_AR2                    32
#define PIN_AR3                    33
#define PIN_AR4                    34
#define PIN_DCLK                   35
#define PIN_DCLK_IN                36
#define PIN_DE                     37
#define PIN_DE_IN                  38
#define PIN_DEMOD_RST              39   
#define PIN_DVD_SF_CK              40
#define PIN_DVD_SF_CS              41
#define PIN_GPIO42                 42
#define PIN_GPIO43                 43
#define PIN_GPIO44                 44
#define PIN_DVD_SF_DI              45
#define PIN_DVD_DF_DO              46
#define PIN_EINT0                  47      
#define PIN_EINT1                  48
#define PIN_EINT2                  49
#define PIN_EINT3                  50
#define PIN_HSYNC                  51  
#define PIN_HSYNC_IN               52   
#define PIN_GPIO53                 53
#define PIN_GPIO54                 54
#define PIN_GPIO55                 55
#define PIN_GPIO56                 56
#define PIN_GPIO57                 57
#define PIN_IR                     58      
#define PIN_MSDC_0_CLK             59
#define PIN_MSDC_0_CMD             60      
#define PIN_MSDC_0_DAT0            61
#define PIN_MSDC_0_DAT1            62     
#define PIN_MSDC_0_DAT2            63

#define PIN_MSDC_0_DAT3            64      
#define PIN_MSDC_1_CLK             65
#define PIN_MSDC_1_CMD             66
#define PIN_MSDC_1_DAT0            67
#define PIN_MSDC_1_DAT1            68
#define PIN_MSDC_1_DAT2            69
#define PIN_GPIO70                 70
#define PIN_GPIO71                 71
#define PIN_GPIO72                 72
#define PIN_MSDC_1_DAT3            73
#define PIN_GPIO74                 74
#define PIN_MSDC_2_CLK             75
#define PIN_MSDC_2_CMD             76
#define PIN_MSDC_2_DAT0            77
#define PIN_MSDC_2_DAT1            78
#define PIN_MSDC_2_DAT2            79
#define PIN_MSDC_2_DAT3            80
#define PIN_NFALE                  81
#define PIN_NFCEN0                 82  
#define PIN_NFCEN1                 83   
#define PIN_NFCLE                  84
#define PIN_NFRBN                  85
#define PIN_NFRBN2                 86
#define PIN_NFREN                  87
#define PIN_NFWEN                  88
#define PIN_NLD0                   89      
#define PIN_NLD1                   90
#define PIN_NLD2                   91      
#define PIN_NLD3                   92
#define PIN_NLD4                   93     
#define PIN_NLD5                   94     
#define PIN_NLD6                   95     

#define PIN_NLD7                   96
#define PIN_PCM_CLK                97
#define PIN_PCM_IN                 98
#define PIN_PCM_OUT                99
#define PIN_SYNC                  100
#define PIN_SCL0                  101
#define PIN_SCL1                  102
#define PIN_SDA0                  103
#define PIN_SDA1                  104
#define PIN_SP0_CLK               105
#define PIN_SP0_CS                106
#define PIN_SP0_SI                107
#define PIN_SP0_SO                108
#define PIN_SP1_CLK               109
#define PIN_SP1_CS                110
#define PIN_SP1_SI                111    
#define PIN_SP1_SO                112
#define PIN_SPDIF                 113
#define PIN_TS_CLKIN              114
#define PIN_TS_D0                 115
#define PIN_TS_D1                 116
#define PIN_TS_D2                 117
#define PIN_TS_D3                 118
#define PIN_TS_D4                 119
#define PIN_TS_D5                 120
#define PIN_TS_D6                 121
#define PIN_TS_D7                 122
#define PIN_TS_DEN                123
#define PIN_GPIO124               124
#define PIN_GPIO_125              125
#define PIN_TS_SYNC               126
#define PIN_URXD0                 127

#define PIN_URXD1                 128
#define PIN_URXD2                 129
#define PIN_URXD3                 130
#define PIN_URXD4                 131
#define PIN_URXD5                 132
#define PIN_USB_DM_P0             133             
#define PIN_USB_DM_P1             134
#define PIN_USB_DP_P0             135
#define PIN_USB_DP_P1             136
#define PIN_UTXD0                 137
#define PIN_UTXD1                 138
#define PIN_UTXD2                 139
#define PIN_UTXD3                 140
#define PIN_UTXD4                 141
#define PIN_UTXD5                 142
#define PIN_VB0                   143
#define PIN_VB1                   144
#define PIN_VB2                   145
#define PIN_VB3                   146
#define PIN_VB4                   147
#define PIN_VB5                   148
#define PIN_GPIO149               149
#define PIN_GPIO150               150
#define PIN_GPIO151               151
#define PIN_VB6                   152
#define PIN_VB7                   153
#define PIN_VG0                   154
#define PIN_VG1                   155
#define PIN_VG2                   156
#define PIN_VG3                   157
#define PIN_VG4                   158
#define PIN_VG5                   159


#define PIN_VG6                   160
#define PIN_VG7                   161
#define PIN_GPIO162               162
#define PIN_VIN0                  163
#define PIN_VIN1                  164
#define PIN_VIN2                  165             
#define PIN_VIN3                  166
#define PIN_VIN4                  167
#define PIN_VIN5                  168
#define PIN_VIN6                  169
#define PIN_VIN7                  170
#define PIN_VR0                   171
#define PIN_VR1                   172
#define PIN_VR2                   173
#define PIN_VR3                   174
#define PIN_VR4                   175
#define PIN_VR5                   176
#define PIN_VR6                   177
#define PIN_VR7                   178
#define PIN_VSYNC                 179
#define PIN_VSYNC_IN              180


//pdwnc
#define PDWNC_PAD_PINMUX1   0x0F4
#define PDWNC_PAD_PINMUX2   0x0F8
#define PDWNC_PAD_PINMUX3   0x0FC
#define PDWNC_GPIOIN        0x0D0
#define PDWNC_GPIOEN        0x0D4
#define PDWNC_GPIOOUT       0x0D8

#define PIN_SCL0_IN         181
#define PIN_SDA0_IN         182 
#define PIN_SCL1_IN         183
#define PIN_SDA1_IN         184
#define PIN_GPIO150_IN      185
#define PIN_GPIO151_IN      186
//                                              187
#define PIN_GPIO149_IN      188
//                                              189
//                                              190
//                                              191 
//                                              192 
//                                              193
//                                              194
//                                              195
//                                              196
#define PIN_IR_IN           197
//                                              198
//                                              199
//                                              200  
//                                              201
//                                              202 
//                                              203
//                                              204
//                                              205 
//                                              206
//                                              207 
//                                              208
//                                              209 
//                                              210 
#define PIN_UARXD_IN        211 
#define PIN_UATXD_IN        212 

   
#define PIN_UNKNOWN         255

// Total GPIO is 173.
#define TOTAL_GPIO_NUM      212



//-----------------------------------------------------------------------------
// Macro definitions
//-----------------------------------------------------------------------------
// Macros for register read/write
//#define PAD_CFG0_OFFSET			0xC0

#define PINMUX_FUNCTION0		0
#define PINMUX_FUNCTION1		1
#define PINMUX_FUNCTION2		2
#define PINMUX_FUNCTION3		3
#define PINMUX_FUNCTION4		4
#define PINMUX_FUNCTION5		5
#define PINMUX_FUNCTION6		6
#define PINMUX_FUNCTION7		7
#define PINMUX_FUNCTION8		8
#define PINMUX_FUNCTION9		9
#define PINMUX_FUNCTION10	 	10
#define PINMUX_FUNCTION11		11
#define PINMUX_FUNCTION12		12
#define PINMUX_FUNCTION13		13
#define PINMUX_FUNCTION14		14
#define PINMUX_FUNCTION15		15
#define MAX_PINMUX_FUNCTION     15

//MT3360 Pin mux function definitions
//PADMUX 0 -- 0X54
#define I2S_MIC_IN_SEL            0
#define I2C0_SEL                  3
#define AP_SF_SEL                 8
#define I2S_OUT2_M_LR_SEL         12
#define I2S_OUT2_D_BCK_SEL        15
#define USB_I2C_SEL               18
#define SGM_MIC_IN_SEL            20
#define DVD_SF_SEL                22
#define TS_SEL0                   24
#define TS_SEL1                   25
#define TS_SEL2                   26
#define TS_SEL3                   27
#define ARM11_JTAG_SEL            28

//PADMUX 1 -- 0X58
#define SMART_CARD_SEL            (32 + 4)
#define DVD_RS232_SEL             (32 + 15)
#define AP_RS232_SEL              (32 + 18)
#define AP_8032_RS232_SEL         (32 + 21)
#define PCM_SEL                   (32 + 24)
#define ARM9_JTAG_SEL             (32 + 25)
#define PWM0_SEL                  (32 + 28)

//PADMUX 2 -- 0X5C
#define PWM1_SEL                  (64 + 0)
#define TTL_6_8B_SEL              (64 + 6)
#define TTL_8B_SEL                (64 + 7)
#define TTL_SYNC_SEL              (64 + 8)
#define TTL_DE_SEL                (64 + 9)
#define CCIR656_601_DATAIN_SEL    (64 + 10)
#define CCIR601_TIMING_IN_SEL     (64 + 12)
#define CCIR656_601_DATAOUT_SEL   (64 + 16)
#define CCIR601_TIMING_OUT_SEL    (64 + 17)
#define SP0_SEL                   (64 + 18)
#define SP1_SEL                   (64 + 20)
#define NAND_FLASH_SEL            (64 + 22)
#define NAND_FLASH_16BIT_SEL      (64 + 23)
#define NAND_FLASH_CEN2_SEL       (64 + 24)
#define NAND_FLASH_CEN3_SEL       (64 + 25)
#define IR_SEL                    (64 + 26)
#define I2S_OUT0_SEL              (64 + 29)
#define SPDIF_SEL                 (64 + 30)
#define AMUTE_R_SEL               (64 + 31)

//PADMUX 3 -- 0X60
#define AMUTE_F_SEL               (96 + 0)
#define AMUTE_GPS_SEL             (96 + 1)
#define I2S_LINE_IN_SEL           (96 + 8)
#define TEST_BUS_SEL              (96 + 10)
#define DVD_T8032_UP0_SEL         (96 + 12)
#define DVD_T8032_UP1_SEL         (96 + 16)
#define DVD_T8032_UP2_SEL         (96 + 18)
#define DVD_T8032_UP3_SEL         (96 + 22)
#define T8032_EINT1_SEL           (96 + 24)
#define T8032_EINT2_SEL           (96 + 26)
#define T8032_EINT3_SEL           (96 + 28)
#define T8032_EINT4_SEL           (96 + 30)

//PADMUX 4 -- 0X64
#define T8032_EINT5_SEL           (128 + 0)
#define PT110_EINT1_SEL           (128 + 2)
#define PT110_EINT2_SEL           (128 + 4)
#define PT110_EINT3_SEL           (128 + 6)
#define PT110_EINT4_SEL           (128 + 8)
#define RTC_OUT_SEL               (128 + 10)
#define TEST_IN_SEL               (128 + 11)
#define TEST_OUT_SEL              (128 + 12)
#define EF_VDDQ_SEL               (128 + 16)
#define LVDS_SEL                  (128 + 24)

//PADMUX 5 -- 0X68
#define EINT0_SEL                 (160 + 0)
#define EINT1_SEL                 (160 + 2)
#define EINT2_SEL                 (160 + 4)
#define EINT3_SEL                 (160 + 6)
#define EINT4_SEL                 (160 + 8)
#define EINT5_SEL                 (160 + 12)
#define EINT6_SEL                 (160 + 16)
#define EINT7_SEL                 (160 + 19)
#define PWM2_SEL                  (160 + 22)
#define PWM3_SEL                  (160 + 25)
#define PWM4_SEL                  (160 + 28)

//PADMUX 6 -- 0X6C
#define PWM5_SEL                  (192 + 0)
#define I2S_OUT1_M_LR_SEL         (192 + 3)
#define I2S_OUT1_D_BCK_SEL        (192 + 6)
#define UART0_SEL                 (192 + 11)
#define UART1_SEL                 (192 + 13)
#define UART2_SEL                 (192 + 16)
#define UART3_SEL                 (192 + 18)
#define UART4_SEL                 (192 + 20)
#define UART5_SEL                 (192 + 23)
#define I2C1_SEL                  (192 + 26)


#define MAX_PINMUX_SEL          (I2C1_SEL + 1) // 219

  
#endif /* __MT3360_GPIO_PINMUX_H */

