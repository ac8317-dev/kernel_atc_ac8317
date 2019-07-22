/*
 *  linux/include/asm-arm/arch-mt85xx/irqs.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 */
#ifndef __IRQS_VECTOR_8555_
#define __IRQS_VECTOR_8555_

//----------------------------------------------------------------------------
// IRQ Vectors
  #define   VECTOR_GRPC                 0
  #define   VECTOR_GRPB                 1
  //#define   VECTOR_(Reserved)           2
  #define   VECTOR_EXT                  3
  #define   VECTOR_FLASHCARD            4
  #define   VECTOR_EXT2                 5
  #define   VECTOR_VDOIN                6   //VECTOR_DEMUX in 8550
  #define   VECTOR_GRAPH                7
  #define   VECTOR_JPGDEC               8
  #define   VECTOR_FONT                 9
  //#define   VECTOR_(Reserved)           10
  //#define   VECTOR_(Reserved)           11
  #define   VECTOR_JAVA                 12
  #define   VECTOR_RLE                  13
  #define   VECTOR_RS232_1              14
  #define   VECTOR_DSP                  15
  #define   VECTOR_SPD                  16
  #define   VECTOR_DISP_VSYNC           17
  #define   VECTOR_SVO_IFINT            18
  #define   VECTOR_SVOIF                18
  #define   VECTOR_SFDMAI               19
  #define   VECTOR_DDMAI                20
  #define   VECTOR_PL310                21
  #define   VECTOR_USB                  22
  #define   VECTOR_CORISC               23
  #define   VECTOR_AXI64_WR             24
  #define   VECTOR_T2                   25
  #define   VECTOR_T1                   26
  #define   VECTOR_T0                   27
  #define   VECTOR_DSPC                 28
  #define   VECTOR_SVO_FE1INT           29
  #define   VECTOR_SVO_FE0INT           30
  #define   VECTOR_VSYNC                31
  //===============================================
  #define   VECTOR_GCPU                 32
  #define   VECTOR_NFI                  33
  #define   VECTOR_PWMIP_2              34
  #define   VECTOR_PWMIP_1              35
  #define   VECTOR_SPDIF                36
  #define   VECTOR_NR                   37
  #define   VECTOR_CEC                  38  //(VECTOR_CEC=VECTOR_AVLNK)
  #define   VECTOR_EXT4                 39
  #define   VECTOR_DRAMC                40
  #define   VECTOR_RESIZER2             41
  #define   VECTOR_RESIZER1             42
  #define   VECTOR_RESIZER0             43
  #define   VECTOR_HDMI                 44
  #define   VECTOR_DISP2_VSYNC          45
  //#define   VECTOR_DISP_VSYNC           46  //8555 change to VECTOR 17
  #define   VECTOR_SPIINT               47
  #define   VECTOR_DDIINT               48
  #define   VECTOR_VDOUTAUX0            49
  #define   VECTOR_DMXINT               50
  #define   VECTOR_VENC                 51
  #define   VECTOR_PWDNC                52
  #define   VECTOR_ATA1                 53   //There is only one ATA in 8555
  //#define   VECTOR_(Reserved)           54
  //#define   VECTOR_(Reserved)           55
  #define   VECTOR_SPU                  56
  #define   VECTOR_PVR                  57   //VECTOR_PVR=VECTOR_DEMUX
  #define   VECTOR_TSMUX                58
  #define   VECTOR_FMC                  59
  #define   VECTOR_FLASH                60
  #define   VECTOR_DHMIRXSCD            61
  #define   VECTOR_HDMIRXINT            62
  #define   VECTOR_LZHS                 63
  //===============================================
  #define   VECTOR_PERI                 64
  #define   VECTOR_RS232_3              65
  #define   VECTOR_PARSER1              66
  #define   VECTOR_PARSER               67
  #define   VECTOR_DISP_END2            68
  #define   VECTOR_DRAMC1               69
  #define   VECTOR_RS232_2              70
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
  #define   VECTOR_USB3                 82
  #define   VECTOR_DISP_END             83
  #define   VECTOR_TCPIP                84
  #define   VECTOR_USB2                 85
  #define   VECTOR_EPHY                 86
  #define   VECTOR_SACD                 87
  #define   VECTOR_DISPVDO2_UNDERRUN    88
  #define   VECTOR_ENET                 89
  #define   VECTOR_VDLIT                90
  #define   VECTOR_VDFUL                91
  #define   VECTOR_DSPB2RISC            92
  #define   VECTOR_DISPVDO_UNDERRUN     93
  #define   VECTOR_PNG                  94
  #define   VECTOR_GIF                  95

  #define VECTOR_INT_P_GPIO0            96
  #define VECTOR_INT_P_GPIO1            97
  #define VECTOR_INT_P_GPIO2            98
  #define VECTOR_INT_P_GPIO3            99
  #define VECTOR_INT_P_GPIO4           100
  #define VECTOR_INT_P_GPIO5           101
  #define VECTOR_INT_P_GPIO6           102
  #define VECTOR_INT_P_GPIO7           103
  //#define VECTOR_INT_P_xx              104
  #define VECTOR_INT_P_DBG_UART        105
  #define VECTOR_INT_P_SIFS            106
  #define VECTOR_INT_P_CEC             107
  #define VECTOR_INT_P_ETNET           108
  #define VECTOR_INT_P_IR              109
  #define VECTOR_INT_P_SIFM            110
  #define VECTOR_INT_P_DDCCI           111
  #define VECTOR_INT_P_SFFE            112
  #define VECTOR_INT_P_DDCCIS          113
  //#define VECTOR_INT_P_xx            113
  //#define VECTOR_INT_P_xx            114
  //#define VECTOR_INT_P_xx            115
  //#define VECTOR_INT_P_xx            116
  //#define VECTOR_INT_P_xx            117
  //#define VECTOR_INT_P_xx            118
  //#define VECTOR_INT_P_xx            119
  //#define VECTOR_INT_P_xx            120
  //#define VECTOR_INT_P_xx            121
  //#define VECTOR_INT_P_xx            122
  //#define VECTOR_INT_P_xx            123
  //#define VECTOR_INT_P_xx            124
  //#define VECTOR_INT_P_xx            125
  //#define VECTOR_INT_P_xx            126
  //#define VECTOR_INT_P_xx            127

#endif /*#ifndef __IRQS_VECTOR_8555_*/

