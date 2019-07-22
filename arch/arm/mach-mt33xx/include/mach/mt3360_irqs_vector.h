/*
 *  linux/include/asm-arm/arch-mt85xx/irqs.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */
#ifndef __IRQS_VECTOR_3360_
#define __IRQS_VECTOR_3360_

//----------------------------------------------------------------------------
// IRQ Vectors
#define   VECTOR_GRPC                 0
#define   VECTOR_GRPB                 1
#define   VECTOR_GRPD           2
#define   VECTOR_EXT                  3
#define   VECTOR_SPI2                 4
#define   VECTOR_EXT2                 5
#define   VECTOR_VDOIN                6   //VECTOR_DEMUX in 8550
#define   VECTOR_GRAPH                7
#define   VECTOR_JPGDEC               8
//#define   VECTOR_FONT                 9
//#define   VECTOR_IOMMU                10
#define   VECTOR_DDMANEW              11
#define   VECTOR_JAVA                 12
#define   VECTOR_RLE                  13
#define   VECTOR_RS232_1              14   //3360
#define   VECTOR_DSP                  15   //3360 DSPA2RC
#define   VECTOR_SPD                  16   //3360 SPDF_RC
//#define   VECTOR_DISP_VSYNC           17
#define   VECTOR_VDOUTREAR            17
//#define   VECTOR_SVO_IFINT            18
//#define   VECTOR_SVOIF                18
#define   VECTOR_SFDMAI               19
#define   VECTOR_DDMAI                20
#define   VECTOR_PL310                21
#define   VECTOR_USB                  22
#define   VECTOR_TOCORISC               23
#define   VECTOR_AXI64_WR             24
#define   VECTOR_T2                   25
#define   VECTOR_T1                   26
#define   VECTOR_T0                   27
#define   VECTOR_DSPC                 28    //3360 DSPC2RC
#define   VECTOR_SVO_FE1INT           29
#define   VECTOR_SVO_FE0INT           30
#define   VECTOR_VSYNC                31
//===============================================
//#define                               32
#define   VECTOR_NFI                  33
#define   VECTOR_MALI_3D0             34
#define   VECTOR_MALI_3D1             35
#define   VECTOR_MALI_3D2             36
#define   VECTOR_MALI_3D3             37
//#define   VECTOR_EXT4                 37
//#define   VECTOR_CEC                  38  //(VECTOR_CEC=VECTOR_AVLNK)
#define   VECTOR_EXT4                 39
#define   VECTOR_DRAMC                40
//#define   VECTOR_RESIZER2             41
#define   VECTOR_RESIZER1             42
#define   VECTOR_RESIZER0             43
#define   VECTOR_SMARTCARD            44
#define   VECTOR_PANEL_SCALER         45
//#define   VECTOR_DISP_VSYNC           46  //8555 change to VECTOR 17
#define   VECTOR_SPIINT               47
#define   VECTOR_DDIINT               48
#define   VECTOR_OSD3R                49
#define   VECTOR_DMXINT               50
#define   VECTOR_VENC                 51
#define   VECTOR_PWDNC                52
#define   VECTOR_OSD2R                56   //There is only one ATA in 8555
//#define   VECTOR_(Reserved)           54
//#define   VECTOR_(Reserved)           55
//#define   VECTOR_SPU                  56
//#define   VECTOR_PVR                  57   //VECTOR_PVR=VECTOR_DEMUX
//#define   VECTOR_TSMUX                58
//#define   VECTOR_FMC                  59
#define   VECTOR_FLASH                60
#define   VECTOR_UART_3				62 	 //VECTOR_RS232_4              61   //3360
#define   VECTOR_UART_4 				61   //VECTOR_RS232_5              62   //3360
#define   VECTOR_LZHS                 63
//===============================================
#define   VECTOR_UART_5 				64    //VECTOR_RS232_6              64    //3360
#define   VECTOR_UART_2 				65    //VECTOR_RS232_3              65    //3360
//#define   VECTOR_RS232_3 				65    
//#define   VECTOR_PARSER1              66
//#define   VECTOR_PARSER               67
#define   VECTOR_DISP_END_R           68
//#define   VECTOR_DRAMC1               69
#define   VECTOR_UART_1 				70    //VECTOR_RS232_2              70    //3360
//#define   VECTOR_RS232_2 				70    
#define   VECTOR_PNG2                 71
#define   VECTOR_PNG1                 72
#define   VECTOR_GIF2                 73
#define   VECTOR_GIF1                 74
#define   VECTOR_PSTQ                 75
#define   VECTOR_SRCQ                 76
#define   VECTOR_OSD5_UNFLW           77
#define   VECTOR_OSD4_UNFLW           78
#define   VECTOR_OSD3_UNFLW           79
#define   VECTOR_OSD2_UNFLW           80
#define   VECTOR_OSD1_UNFLW           81
#define   VECTOR_MALI_3D4             82
#define   VECTOR_DISP_END_F             83
//#define   VECTOR_TCPIP                84
#define   VECTOR_USB2                 85
//#define   VECTOR_EPHY                 86
#define   VECTOR_UART_6 				87    //VECTOR_RS232_7              87    //3360
#define   VECTOR_DISPVDO2_UNDERRUN    88
//#define   VECTOR_ENET                 89
//#define   VECTOR_VDLIT                90
#define   VECTOR_VDFUL                91
#define   VECTOR_DSPB2RISC            92       //3360 DSPB2RC
#define   VECTOR_DISPVDO_UNDERRUN     93
#define   VECTOR_PNG                  94
#define   VECTOR_GIF                  95
//===============================================
#define   VECTOR_EXT3                 96
#define   VECTOR_EXT5                 97
#define   VECTOR_EXT6                 98  
#define   VECTOR_EXT7                 99
#define   VECTOR_EXT8                 100
#define   VECTOR_PT110AP0             101
#define   VECTOR_PT110AP1             102
#define   VECTOR_PT110AP2             103
#define   VECTOR_PT110AP3             104
#define   VECTOR_UP2AP0               105
#define   VECTOR_PWNFB                106
#define   VECTOR_KPI                  107
#define   VECTOR_TSI                  108
//#define   VECTOR_M3D                  109
#define   VECTOR_RTC                  110
#define   VECTOR_SPI1                 111
#define   VECTOR_ASRC_AP              112  //3360
#define   VECTOR_ASRC_GPS             113  //3360
#define   VECTOR_PWM_REAR             114  //3360
#define   VECTOR_PWM_FRONT1           115  //3360
#define   VECTOR_PWM_FRONT2           116  //3360
#define   VECTOR_PWM_FRONT3	          117  //3360
#define   VECTOR_PWM_GPS              118  //3360
#define   VECTOR_AOUT_GPS_RC          119  //3360
#define   VECTOR_AOUT_2ND_RC          120  //3360
#define   VECTOR_MIC_RC               121  //3360
#define   VECTOR_AOUT_BT_RC           122  //3360
#define   VECTOR_MSDC0                123
#define   VECTOR_MSDC1                124
#define   VECTOR_MSDC2                125
#define   VECTOR_LVDSTOP              126
//#define   VECTOR_(Reserved)           127
//#define   VECTOR_(Reserved)           128
//===============================================
#define   VECTOR_INT_P_GPIO0          128  //3360
#define   VECTOR_INT_P_GPIO1          129  //3360
#define   VECTOR_INT_P_GPIO2          130  //3360
#define   VECTOR_INT_P_GPIO3          131  //3360
#define   VECTOR_INT_P_GPIO4          132  //3360
#define   VECTOR_INT_P_GPIO5          133  //3360
#define   VECTOR_INT_P_GPIO6          134  //3360
#define   VECTOR_INT_P_GPIO7          135  //3360
//#define   VECTOR_INT_P_xx             136  
#define   VECTOR_INT_P_DBG_UART       137  //3360
#define   VECTOR_INT_P_SIFS           138  //3360
#define   VECTOR_INT_P_CEC            139  //3360
#define   VECTOR_INT_P_ETNET          140  //3360
#define   VECTOR_INT_P_IR             141  //3360
#define   VECTOR_INT_P_SIFM           142  //3360
#define   VECTOR_INT_P_DDCCI          143  //3360

#define VECTOR_INT_DUAL1            144
#define VECTOR_INT_DUAL2            145
#define VECTOR_INT_DUAL3            146
#define VECTOR_INT_DUAL4            147
#define VECTOR_INT_DUAL5            148

#endif /*#ifndef __IRQS_VECTOR_3360_*/

