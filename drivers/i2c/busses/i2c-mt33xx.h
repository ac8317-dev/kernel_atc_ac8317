/*
 * @file i2c-mt33xx.h
 *
 * @par LEGAL DISCLAIMER
 *
 * (Header of MediaTek Software/Firmware Release or Documentation)
 *
 * BY OPENING OR USING THIS FILE, USER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND
 * AGREES THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * ARE PROVIDED TO USER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS
 * ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED
 * IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND USER AGREES TO LOOK ONLY TO
 * SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL
 * ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO USER'S
 * SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 * USER HEREBY ACKNOWLEDGES THE CONFIDENTIALITY OF MEDIATEK SOFTWARE AND AGREES
 * NOT TO DISCLOSE OR PERMIT DISCLOSURE OF ANY MEDIATEK SOFTWARE TO ANY THIRD
 * PARTY OR TO ANY OTHER PERSON, EXCEPT TO DIRECTORS, OFFICERS, EMPLOYEES OF
 * USER WHO ARE REQUIRED TO HAVE THE INFORMATION TO CARRY OUT THE PURPOSE OF
 * OPENING OR USING THIS FILE.
 *
 * @par Project
 *    MT33xx
 *
 * @par Description
 *    FM Trsansmitter driverl
 *
 * @par Author_Name
 *    SS.He mtk40499
 *
 */


#ifndef __DRIVERS_I2C_BUSSES_I2C_MT33XX_H
#define __DRIVERS_I2C_BUSSES_I2C_MT33XX_H
/**************************************************
    register define
    ************************************************/
#define I2C_BASE_VA      0xFD024000


#define SIF_PDMISC             ((unsigned int)0x48)
#define DDCCI_RST            ((unsigned int)0x1<<7)
#define SIFM_RST             ((unsigned int)0x1<<6)
#define SIFN_EN             ((unsigned int)0x1<<2)

#define SIF_PAD_PU             ((unsigned int)0xEC)
#define SIF_PAD_PD             ((unsigned int)0xF0)
#define SIF_PAD_PINMUX1        ((unsigned int)0xF4)
#define HDMI_SD             ((unsigned int)1<<24)
#define HDMI_SCK            ((unsigned int)1<<23) 
#define SIF_PAD_PINMUX2        ((unsigned int)0xF8)
#define SIFM_SD_LCDRD       ((unsigned int)1<<3)
#define SIFM_SCK_VFD_DATA   ((unsigned int)1<<2)   
#define SIFS_SD_VFD_CLK     ((unsigned int)1<<1)
#define SIFS_SCK_VFD_STB    ((unsigned int)1<<0)

#define SIF_PAD_PINMUX3        ((unsigned int)0xFC)
#define SIFM_SD_SEL_IOM     ((unsigned int)1<<3)
#define SIFM_SCL_SEL_IOM    ((unsigned int)1<<2)
#define SIFS_SD_SEL_IOM     ((unsigned int)1<<1)
#define SIFS_SCL_SEL_IOM    ((unsigned int)1<<0)

#define SIF_INTSTA             ((unsigned int)0x140)
#define DDCCI_INT            ((unsigned int)0x1<<15)
#define SIFM_INT             ((unsigned int)0x1<<14)
#define SIFS_INT             ((unsigned int)0x1<<10)

#define SIF_INTEN              ((unsigned int)0x144)
#define DDCCI_INTEN          ((unsigned int)0x1<<15)
#define SIFM_INTEN           ((unsigned int)0x1<<14)

#define UP_CFG                 ((unsigned int)0x188)
#define FAST_CK_EN           ((unsigned int)0x1<<20)

#define SIF_INTCLR             ((unsigned int)0x148)
#define DDCCI_INTCLR         ((unsigned int)0x1<<15)
#define SIFM_INTCLR          ((unsigned int)0x1<<14)

#define SIF_SIFMCTL0           ((unsigned int)0x440)  
#define SIFM_ODRAIN          ((unsigned int)(unsigned int)0x1<<31)
#define SIFM_CLK_DIV_OFFSET  ((unsigned int)16)
#define SIFM_CLK_DIV_MASK    ((unsigned int)0xFFF<<16)
#define SIFM_PSEL            ((unsigned int)0x1<<5)
#define SIFM_CS_STATUS       ((unsigned int)0x1<<4)
#define SIFM_SCL_STATE       ((unsigned int)0x1<<3)
#define SIFM_SDA_STATE       ((unsigned int)0x1<<2)
#define SIFM_SM0EN           ((unsigned int)0x1<<1)
#define SIFM_SCL_STRECH      ((unsigned int)0x1<<0)

#define SIF_SIFMCTL1           ((unsigned int)0x444)
#define SIFM_ACK_OFFSET      ((unsigned int)16)
#define SIFM_ACK_MASK        ((unsigned int)0xFF<<16)
#define SIFM_PGLEN_OFFSET    ((unsigned int)8)
#define SIFM_PGLEN_MASK      ((unsigned int)0x7<<8)
#define SIFM_SIF_MODE_OFFSET ((unsigned int)4)
#define SIFM_SIF_MODE_MASK   ((unsigned int)0x7<<4)
#define SIFM_START            ((unsigned int)0x1)
#define SIFM_WRITE_DATA       ((unsigned int)0x2)
#define SIFM_STOP             ((unsigned int)0x3)
#define SIFM_READ_DATA_NO_ACK ((unsigned int)0x4)
#define SIFM_READ_DATA_ACK    ((unsigned int)0x5)
#define SIFM_TRI             ((unsigned int)0x1<<0)
#define SIFM_BUSY             ((unsigned int)0x1<<0)

#define SIF_SIFMD0             ((unsigned int)0x448)  
#define SIFM_DATA3_OFFSET    ((unsigned int)24)
#define SIFM_DATA3_MASK      ((unsigned int)0xFF<<24)
#define SIFM_DATA2_OFFSET    ((unsigned int)16)
#define SIFM_DATA2_MASK      ((unsigned int)0xFF<<16)
#define SIFM_DATA1_OFFSET    ((unsigned int)8)
#define SIFM_DATA1_MASK      ((unsigned int)0xFF<<8)
#define SIFM_DATA0_OFFSET    ((unsigned int)0)
#define SIFM_DATA0_MASK      ((unsigned int)0xFF<<0)
  
#define SIF_SIFMD1             ((unsigned int)0x44C)  
 #define SIFM_DATA7_OFFSET	  ((unsigned int)24)
 #define SIFM_DATA7_MASK	  ((unsigned int)0xFF<<24)
 #define SIFM_DATA6_OFFSET	  ((unsigned int)16)
 #define SIFM_DATA6_MASK	  ((unsigned int)0xFF<<16)
 #define SIFM_DATA5_OFFSET	  ((unsigned int)8)
 #define SIFM_DATA5_MASK	  ((unsigned int)0xFF<<8)
 #define SIFM_DATA4_OFFSET	  ((unsigned int)0)
 #define SIFM_DATA4_MASK	  ((unsigned int)0xFF<<0)
 /******************************************************************************
* Local macro
******************************************************************************/
#define SIF_READ32(u4Addr)                  (*((volatile uint32_t*)(I2C_BASE_VA + u4Addr)))
#define SIF_WRITE32(u4Addr, u4Val)      (*((volatile uint32_t*)(I2C_BASE_VA + u4Addr)) = (u4Val))

#define SIF_SET_BIT(u4Addr, u4Val)  SIF_WRITE32((u4Addr), (SIF_READ32(u4Addr) | (u4Val)))
#define SIF_CLR_BIT(u4Addr, u4Val)  SIF_WRITE32((u4Addr), (SIF_READ32(u4Addr) & (~(u4Val))))

#define IS_SIF_BIT(u4Addr, u4Val)   ((SIF_READ32(u4Addr) & (u4Val)) == (u4Val)) 

#define SIF_WRITE_MASK(u4Addr, u4Mask, u4Offet, u4Val)  SIF_WRITE32(u4Addr, ((SIF_READ32(u4Addr) & (~(u4Mask))) | (((u4Val) << (u4Offet)) & (u4Mask))))
#define SIF_READ_MASK(u4Addr, u4Mask, u4Offet)  ((SIF_READ32(u4Addr) & (u4Mask)) >> (u4Offet))

#define SIFM_DATA0_READ()   SIF_READ_MASK(SIF_SIFMD0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET)  
#define SIFM_DATA1_READ()   SIF_READ_MASK(SIF_SIFMD0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET)  
#define SIFM_DATA2_READ()   SIF_READ_MASK(SIF_SIFMD0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET)  
#define SIFM_DATA3_READ()   SIF_READ_MASK(SIF_SIFMD0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET)  
#define SIFM_DATA4_READ()   SIF_READ_MASK(SIF_SIFMD1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET)  
#define SIFM_DATA5_READ()   SIF_READ_MASK(SIF_SIFMD1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET)  
#define SIFM_DATA6_READ()   SIF_READ_MASK(SIF_SIFMD1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET)  
#define SIFM_DATA7_READ()   SIF_READ_MASK(SIF_SIFMD1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET)  

#define SIFM_DATA0_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET, u4Val)  
#define SIFM_DATA1_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET, u4Val)  
#define SIFM_DATA2_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET, u4Val)  
#define SIFM_DATA3_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET, u4Val)  
#define SIFM_DATA4_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET, u4Val)  
#define SIFM_DATA5_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET, u4Val)  
#define SIFM_DATA6_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET, u4Val)  
#define SIFM_DATA7_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFMD1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET, u4Val) 

#define SIFM_CLK_DIV_READ()        SIF_READ_MASK(SIF_SIFMCTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET)
#define SIFM_CLK_DIV_WRITE(u4Val)  SIF_WRITE_MASK(SIF_SIFMCTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET, u4Val)

#define SIFM_ACK_READ()            SIF_READ_MASK(SIF_SIFMCTL1, SIFM_ACK_MASK, SIFM_ACK_OFFSET)

#define SIFM_PGLEN_READ()          SIF_READ_MASK(SIF_SIFMCTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET)
#define SIFM_PGLEN_WRITE(u4Val)    SIF_WRITE_MASK(SIF_SIFMCTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET, u4Val)

#define SIFM_SIF_MODE_READ()          SIF_READ_MASK(SIF_SIFMCTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET)
#define SIFM_SIF_MODE_WRITE(u4Val)    SIF_WRITE_MASK(SIF_SIFMCTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET, u4Val)

#define CLK_SRC_IS_27M()   ((SIF_READ32(UP_CFG) & FAST_CK_EN) == FAST_CK_EN)

#endif
