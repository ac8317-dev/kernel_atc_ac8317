/*
 *  linux/arch/arm/arch-ac83xxi/include/mach/83xx_irqs_vector.h
 *
 *  Copyright (C) 2013 MediaTek Inc.
 */
#ifndef __IRQS_VECTOR_83XX_
#define __IRQS_VECTOR_83XX_


#define IRQ_BIM_GIC_OFFSET				64

//----------------------------------------------------------------------------
// IRQ Vectors
  #define   VECTOR_GRPC                 (IRQ_BIM_GIC_OFFSET + 0)
  #define   VECTOR_GRPB                 (IRQ_BIM_GIC_OFFSET + 1)
  #define   VECTOR_GRPD                 (IRQ_BIM_GIC_OFFSET + 2)
  #define   VECTOR_EXT                  (IRQ_BIM_GIC_OFFSET + 3)
  #define   VECTOR_SPI2INT              (IRQ_BIM_GIC_OFFSET + 4)
  #define   VECTOR_SPI2                 (IRQ_BIM_GIC_OFFSET + 4)	
  
  #define   VECTOR_EXT2                 (IRQ_BIM_GIC_OFFSET + 5)
  #define   VECTOR_VDOIN                (IRQ_BIM_GIC_OFFSET + 6)   //VECTOR_DEMUX in 8550
  #define   VECTOR_GRAPH                (IRQ_BIM_GIC_OFFSET + 7)
  #define   VECTOR_JPGDEC               (IRQ_BIM_GIC_OFFSET + 8)
  #define   VECTOR_GRAPH_MMU            (IRQ_BIM_GIC_OFFSET + 9)
  #define   VECTOR_EXT3                 (IRQ_BIM_GIC_OFFSET + 10)
  #define   VECTOR_IRT_DMA              (IRQ_BIM_GIC_OFFSET + 11)
//  #define   VECTOR_JAVA                 12
  #define   VECTOR_RLE                  (IRQ_BIM_GIC_OFFSET + 13)
  #define   VECTOR_RS232_1              (IRQ_BIM_GIC_OFFSET + 14)
  #define   VECTOR_DSP                  (IRQ_BIM_GIC_OFFSET + 15)
  #define   VECTOR_SPD                  (IRQ_BIM_GIC_OFFSET + 16)
  #define   VECTOR_VDOUTREAR            (IRQ_BIM_GIC_OFFSET + 17)
  #define   VECTOR_SVO_IFINT            (IRQ_BIM_GIC_OFFSET + 18)
  #define   VECTOR_SVOIF                (IRQ_BIM_GIC_OFFSET + 18)
  #define   VECTOR_SFDMAI               (IRQ_BIM_GIC_OFFSET + 19)
//  #define   VECTOR_DDMAI                20
//  #define   VECTOR_PL310                21
  #define   VECTOR_USB                  (IRQ_BIM_GIC_OFFSET + 22)
  #define   VECTOR_CORISC               (IRQ_BIM_GIC_OFFSET + 23)
  #define   VECTOR_TOCORISC             (IRQ_BIM_GIC_OFFSET + 23)
//  #define   VECTOR_AXI64_WR             24
  #define   VECTOR_T2                   (IRQ_BIM_GIC_OFFSET + 25)
  #define   VECTOR_T1                   (IRQ_BIM_GIC_OFFSET + 26)
  #define   VECTOR_T0                   (IRQ_BIM_GIC_OFFSET + 27)
  #define   VECTOR_DSPC                 (IRQ_BIM_GIC_OFFSET + 28)
  #define   VECTOR_YPBPRINT             (IRQ_BIM_GIC_OFFSET + 29)
  #define   VECTOR_CBUSINT              (IRQ_BIM_GIC_OFFSET + 30)
  #define   VECTOR_VSYNC                (IRQ_BIM_GIC_OFFSET + 31)
  //===============================================
  #define   VECTOR_HDMIEDID             (IRQ_BIM_GIC_OFFSET + 32)
  #define   VECTOR_NFI                  (IRQ_BIM_GIC_OFFSET + 33)
  #define   VECTOR_MALI3D_0             (IRQ_BIM_GIC_OFFSET + 34)
  #define   VECTOR_MALI3D_1             (IRQ_BIM_GIC_OFFSET + 35)
  #define   VECTOR_MALI3D_2             (IRQ_BIM_GIC_OFFSET + 36)
  #define   VECTOR_MALI3D_3             (IRQ_BIM_GIC_OFFSET + 37)
  #define   VECTOR_MALI3D_4             (IRQ_BIM_GIC_OFFSET + 38)
  #define   VECTOR_MALI3D_5             (IRQ_BIM_GIC_OFFSET + 39)
  
  #define   VECTOR_SPDIF                (IRQ_BIM_GIC_OFFSET + 36)
  #define   VECTOR_NR                   (IRQ_BIM_GIC_OFFSET + 37)
  #define   VECTOR_CEC                  (IRQ_BIM_GIC_OFFSET + 38)  //(VECTOR_CEC=VECTOR_AVLNK)


  
  #define   VECTOR_DRAMC                (IRQ_BIM_GIC_OFFSET + 40)
  #define   VECTOR_HDMI                 (IRQ_BIM_GIC_OFFSET + 41)
  #define   VECTOR_RESIZER1             (IRQ_BIM_GIC_OFFSET + 42)
  #define   VECTOR_RESIZER0             (IRQ_BIM_GIC_OFFSET + 43)
  #define   VECTOR_SMARTCARD            (IRQ_BIM_GIC_OFFSET + 44)
  #define   VECTOR_PANEL_SCALER         (IRQ_BIM_GIC_OFFSET + 45) //for build error
  #define   VECTOR_HDMI_CEC             (IRQ_BIM_GIC_OFFSET + 46)  
//  #define   VECTOR_SPIINT               47
  #define   VECTOR_DDIINT               (IRQ_BIM_GIC_OFFSET + 48)
  #define   VECTOR_OSD3_R_UNFLW         (IRQ_BIM_GIC_OFFSET + 49)
  #define   VECTOR_DMXINT               (IRQ_BIM_GIC_OFFSET + 50)
  #define   VECTOR_WCHNL                (IRQ_BIM_GIC_OFFSET + 51)
  #define   VECTOR_PWDNC                (IRQ_BIM_GIC_OFFSET + 52)
  #define   VECTOR_SIF0                 (IRQ_BIM_GIC_OFFSET + 53)
  #define   VECTOR_SIF1                 (IRQ_BIM_GIC_OFFSET + 54)
  #define   VECTOR_WCHNL2               (IRQ_BIM_GIC_OFFSET + 55)
  #define   VECTOR_OSD2_R_UNFLW         (IRQ_BIM_GIC_OFFSET + 56)
  #define   VECTOR_FLASH                (IRQ_BIM_GIC_OFFSET + 60)
  #define   VECTOR_UART4                (IRQ_BIM_GIC_OFFSET + 61)
  #define   VECTOR_UART3                (IRQ_BIM_GIC_OFFSET + 62)
  #define   VECTOR_LZHS                 (IRQ_BIM_GIC_OFFSET + 63)
  //===============================================
  #define   VECTOR_UART5                (IRQ_BIM_GIC_OFFSET + 64)
  #define   VECTOR_UART2                (IRQ_BIM_GIC_OFFSET + 65)
 
  #define   VECTOR_DISP_END2            (IRQ_BIM_GIC_OFFSET + 68)
//  #define   VECTOR_DRAMC1               69
  #define   VECTOR_UART1                (IRQ_BIM_GIC_OFFSET + 70)
  #define   VECTOR_PNG2                 (IRQ_BIM_GIC_OFFSET + 71)
  #define   VECTOR_PNG1                 (IRQ_BIM_GIC_OFFSET + 72)
  #define   VECTOR_GIF2                 (IRQ_BIM_GIC_OFFSET + 73)
  #define   VECTOR_GIF1                 (IRQ_BIM_GIC_OFFSET + 74)
  #define   VECTOR_PSTQ                 (IRQ_BIM_GIC_OFFSET + 75)
  #define   VECTOR_SRCQ                 (IRQ_BIM_GIC_OFFSET + 76)
  #define   VECTOR_OSD5_UNFLW           (IRQ_BIM_GIC_OFFSET + 77)
  #define   VECTOR_OSD4_UNFLW           (IRQ_BIM_GIC_OFFSET + 78)
  #define   VECTOR_OSD3_UNFLW           (IRQ_BIM_GIC_OFFSET + 79)
  #define   VECTOR_OSD2_UNFLW           (IRQ_BIM_GIC_OFFSET + 80)
  #define   VECTOR_OSD1_UNFLW           (IRQ_BIM_GIC_OFFSET + 81)
 
  #define   VECTOR_DISP_END             (IRQ_BIM_GIC_OFFSET + 83)
  #define   VECTOR_TCPIP                (IRQ_BIM_GIC_OFFSET + 84)
  #define   VECTOR_USB2                 (IRQ_BIM_GIC_OFFSET + 85)
//  #define   VECTOR_EPHY                 86
  #define   VECTOR_UART6                (IRQ_BIM_GIC_OFFSET + 87)
  #define   VECTOR_DISPVDO2_UNDERRUN    (IRQ_BIM_GIC_OFFSET + 88)
//  #define   VECTOR_ENET                 89
  #define   VECTOR_VDLIT                (IRQ_BIM_GIC_OFFSET + 90)
  #define   VECTOR_VDFUL                (IRQ_BIM_GIC_OFFSET + 91)
  #define   VECTOR_DSPB2RISC            (IRQ_BIM_GIC_OFFSET + 92)
  #define   VECTOR_DISPVDO_UNDERRUN     (IRQ_BIM_GIC_OFFSET + 93)
  #define   VECTOR_PNG                  (IRQ_BIM_GIC_OFFSET + 94)
  #define   VECTOR_GIF                  (IRQ_BIM_GIC_OFFSET + 95)




  #define   VECTOR_EXT4                 (IRQ_BIM_GIC_OFFSET + 96)  
  #define   VECTOR_EXT5                 (IRQ_BIM_GIC_OFFSET + 97)  
  #define   VECTOR_EXT6                 (IRQ_BIM_GIC_OFFSET + 98) 
  #define   VECTOR_EXT7                 (IRQ_BIM_GIC_OFFSET + 99) 
  #define   VECTOR_EXT8                 (IRQ_BIM_GIC_OFFSET + 100)  
  #define   VECTOR_PT110AP0             (IRQ_BIM_GIC_OFFSET + 101)  // GPIO1
  #define   VECTOR_PT110AP1             (IRQ_BIM_GIC_OFFSET + 102)  // HDMI_HTPLG
  #define   VECTOR_PT110AP2             (IRQ_BIM_GIC_OFFSET + 103)  // HDMI_CEC
  #define   VECTOR_PT110AP3             (IRQ_BIM_GIC_OFFSET + 104)  // AUX 8032
  #define   VECTOR_UP2AP0               (IRQ_BIM_GIC_OFFSET + 105)
  #define   VECTOR_KP_PWMFB             (IRQ_BIM_GIC_OFFSET + 106)
  #define   VECTOR_PWNFB                (IRQ_BIM_GIC_OFFSET + 106)
  #define   VECTOR_INT_P_CEC            (IRQ_BIM_GIC_OFFSET + 107)
  #define   VECTOR_KPI                  (IRQ_BIM_GIC_OFFSET + 107)
  #define   VECTOR_TS                   (IRQ_BIM_GIC_OFFSET + 108)
  #define   VECTOR_TSI                  (IRQ_BIM_GIC_OFFSET + 108)
  #define   VECTOR_INT_P_IR             (IRQ_BIM_GIC_OFFSET + 109) //remove form ac8317
  #define   VECTOR_RTC                  (IRQ_BIM_GIC_OFFSET + 110)
  #define   VECTOR_SPI1                (IRQ_BIM_GIC_OFFSET + 111)
  #define   VECTOR_ASRC_AP              (IRQ_BIM_GIC_OFFSET + 112)
  #define   VECTOR_ASRC_GPS             (IRQ_BIM_GIC_OFFSET + 113)
  #define   VECTOR_PWM_REAR             (IRQ_BIM_GIC_OFFSET + 114)
  #define   VECTOR_PWM_FRONT1           (IRQ_BIM_GIC_OFFSET + 115)  
  #define   VECTOR_PWM_FRONT2           (IRQ_BIM_GIC_OFFSET + 116)  
  #define   VECTOR_PWM_FRONT3	        (IRQ_BIM_GIC_OFFSET + 117)  
  #define   VECTOR_LIN2_RCINT           (IRQ_BIM_GIC_OFFSET + 118)  
  #define   VECTOR_LIN_MULTI_RCINT      (IRQ_BIM_GIC_OFFSET + 119)
  #define   VECTOR_AOUT_GPS_RC          (IRQ_BIM_GIC_OFFSET + 119)
  #define   VECTOR_AOUT_2ND_RC          (IRQ_BIM_GIC_OFFSET + 120)        
  #define   VECTOR_MIC_RC               (IRQ_BIM_GIC_OFFSET + 121)   
  #define   VECTOR_AOUT_BT_RC           (IRQ_BIM_GIC_OFFSET + 122)   
  #define 	VECTOR_MSDC0				(IRQ_BIM_GIC_OFFSET + 123) 	
  #define 	VECTOR_MSDC1                (IRQ_BIM_GIC_OFFSET + 124) 
  #define   VECTOR_MSDC2				(IRQ_BIM_GIC_OFFSET + 125) 
  #define   VECTOR_LVDS_TOP				(IRQ_BIM_GIC_OFFSET + 126)
  #define   VECTOR_DVD_DSP				(IRQ_BIM_GIC_OFFSET + 127)

// build error
#define   VECTOR_INT_P_GPIO4            124   // GPIO7
 


	#define MAX_VECTOR_NUM    256	  
#endif /*#ifndef __IRQS_VECTOR_83XX_*/

