
#ifndef _I2C_I2C_AC83XX_H
#define _I2C_I2C_AC83XX_H
/**************************************************
    register define
    ************************************************/
#define I2C_BASE_VA      0xFD030000
#define IO_BASE_VA       0xFD000000


#define SIF_PDMISC             ((unsigned int)0x48)
#define DDCCI_RST              ((unsigned int)0x1<<7)
#define SIFM_RST               ((unsigned int)0x1<<6)
#define SIFN_EN                ((unsigned int)0x1<<2)

#define SIF_PAD_PU             ((unsigned int)0xEC)
#define SIF_PAD_PD             ((unsigned int)0xF0)
#define SIF_PAD_PINMUX1        ((unsigned int)0xF4)
#define HDMI_SD                ((unsigned int)1<<24)
#define HDMI_SCK               ((unsigned int)1<<23) 
#define SIF_PAD_PINMUX2        ((unsigned int)0xF8)
#define SIFM_SD_LCDRD          ((unsigned int)1<<3)
#define SIFM_SCK_VFD_DATA      ((unsigned int)1<<2)   
#define SIFS_SD_VFD_CLK        ((unsigned int)1<<1)
#define SIFS_SCK_VFD_STB       ((unsigned int)1<<0)

#define SIF_PAD_PINMUX3        ((unsigned int)0xFC)
#define SIFM_SD_SEL_IOM        ((unsigned int)1<<3)
#define SIFM_SCL_SEL_IOM       ((unsigned int)1<<2)
#define SIFS_SD_SEL_IOM        ((unsigned int)1<<1)
#define SIFS_SCL_SEL_IOM       ((unsigned int)1<<0)


#define UP_CFG                 ((unsigned int)0x188)
#define FAST_CK_EN             ((unsigned int)0x1<<20)

#define PAD_PINMUX0            ((unsigned int)0x54)   
   #define SIF0_SEL_SCL0_SDA0    ((unsigned int)0x01<<3)     
   #define SIF0_SEL_SCL1_SDA1    ((unsigned int)0x02<<3)
#define PAD_PINMUX6              ((unsigned int)0x6C)     
   #define SIF1_SEL_SCL1_SDA1    ((unsigned int)0x01<<26)     
   #define SIF1_SEL_SCL0_SDA0    ((unsigned int)0x02<<27)   
#define SIF_SEL                  (0x94)     
   #define SIF_SEL_M0M1          (0x00<<4)     
   #define SIF_SEL_S0M1          (0x01<<4)     
   #define SIF_SEL_M0S1          (0x02<<4)     
   #define SIF_SEL_S0S1          (0x03<<4)
#define SIF_CLOCK                ((0xA8))     
   #define SIFM0_CLOCK           ((unsigned int)(unsigned int)0x1<<28)     
   #define SIFM1_CLOCK           ((unsigned int)(unsigned int)0x1<<29)     
   #define SIFS0_CLOCK           ((unsigned int)(unsigned int)0x1<<30)     
   #define SIFS1_CLOCK           ((unsigned int)(unsigned int)0x1<<31)     
#define SIF_RESET                ((0xC4))    
   #define SIFM0_RESET           ((unsigned int)(unsigned int)0x1<<28)    
   #define SIFM1_RESET           ((unsigned int)(unsigned int)0x1<<29)    
   #define SIFS0_RESET           ((unsigned int)(unsigned int)0x1<<30)    
   #define SIFS1_RESET           ((unsigned int)(unsigned int)0x1<<31)



//***
#define SIFM_INTCLR          ((unsigned int)0x410)
#define SIFM_INTEN           ((unsigned int)0x414)
#define SIFM_INTSTA          ((unsigned int)0x418)

#define SIFM1_INTCLR          ((unsigned int)0x810)
#define SIFM1_INTEN           ((unsigned int)0x814)
#define SIFM1_INTSTA          ((unsigned int)0x818)

//*****
//*****master0
#define SIF_SIFM0CTL0        ((unsigned int)0x400)  
#define SIF_SIFM1CTL0        ((unsigned int)0x800)
  #define SIFM_ODRAIN          ((unsigned int)(unsigned int)0x1<<31)
  #define SIFM_CLK_DIV_OFFSET  ((unsigned int)16)
  #define SIFM_CLK_DIV_MASK    ((unsigned int)0xFFF<<16)
  #define SIFM_PSEL            ((unsigned int)0x1<<5)
  #define SIFM_CS_STATUS       ((unsigned int)0x1<<4)
  #define SIFM_SCL_STATE       ((unsigned int)0x1<<3)
  #define SIFM_SDA_STATE       ((unsigned int)0x1<<2)
  #define SIFM_SM0EN           ((unsigned int)0x1<<1)
  #define SIFM_SCL_STRECH      ((unsigned int)0x1<<0)

#define SIF_SIFM0CTL1        ((unsigned int)0x404)
#define SIF_SIFM1CTL1        ((unsigned int)0x804)
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
  #define SIFM_TRI              ((unsigned int)0x1<<0)
  #define SIFM_BUSY             ((unsigned int)0x1<<0)

#define SIF_SIFM0D0          ((unsigned int)0x408)
#define SIF_SIFM1D0          ((unsigned int)0x808)
  #define SIFM_DATA3_OFFSET    ((unsigned int)24)
  #define SIFM_DATA3_MASK      ((unsigned int)0xFF<<24)
  #define SIFM_DATA2_OFFSET    ((unsigned int)16)
  #define SIFM_DATA2_MASK      ((unsigned int)0xFF<<16)
  #define SIFM_DATA1_OFFSET    ((unsigned int)8)
  #define SIFM_DATA1_MASK      ((unsigned int)0xFF<<8)
  #define SIFM_DATA0_OFFSET    ((unsigned int)0)
  #define SIFM_DATA0_MASK      ((unsigned int)0xFF<<0)
  
#define SIF_SIFM0D1           ((unsigned int)0x40C)  
#define SIF_SIFM1D1           ((unsigned int)0x80C)  
  #define SIFM_DATA7_OFFSET	  ((unsigned int)24)
  #define SIFM_DATA7_MASK	  ((unsigned int)0xFF<<24)
  #define SIFM_DATA6_OFFSET	  ((unsigned int)16)
  #define SIFM_DATA6_MASK	  ((unsigned int)0xFF<<16)
  #define SIFM_DATA5_OFFSET	  ((unsigned int)8)
  #define SIFM_DATA5_MASK	  ((unsigned int)0xFF<<8)
  #define SIFM_DATA4_OFFSET	  ((unsigned int)0)
  #define SIFM_DATA4_MASK	  ((unsigned int)0xFF<<0)
 //end master0
 /******************************************************************************
* Local macro
******************************************************************************/
#define HAL_WRITE32(_reg_, _val_)   		(*((volatile uint32_t*)(_reg_)) = (_val_))
#define HAL_READ32(_reg_)           		(*((volatile uint32_t*)(_reg_)))

#define SIF_READ32(u4Addr)                 	HAL_READ32(I2C_BASE_VA + u4Addr) 
#define SIF_WRITE32(u4Addr, u4Val)		    HAL_WRITE32(I2C_BASE_VA + u4Addr,  u4Val)

#define SIF_IO_READ32(u4Addr) 	 	        HAL_READ32(IO_BASE_VA + u4Addr)
#define SIF_IO_WRITE32(u4Addr, u4Val) 	    HAL_WRITE32(IO_BASE_VA + u4Addr, u4Val)

#define SIF_SET_BIT(u4Addr, u4Val)          SIF_WRITE32((u4Addr), (SIF_READ32(u4Addr) | (u4Val)))
#define SIF_CLR_BIT(u4Addr, u4Val)          SIF_WRITE32((u4Addr), (SIF_READ32(u4Addr) & (~(u4Val))))

#define SIF_IO_SET_BIT(u4Addr, u4Val)       SIF_IO_WRITE32((u4Addr), (SIF_READ32(u4Addr) | (u4Val)))
#define SIF_IO_CLR_BIT(u4Addr, u4Val)       SIF_IO_WRITE32((u4Addr), (SIF_READ32(u4Addr) & (~(u4Val))))

#define IS_SIF_BIT(u4Addr, u4Val)          ((SIF_READ32(u4Addr) & (u4Val)) == (u4Val)) 

#define SIF_WRITE_MASK(u4Addr, u4Mask, u4Offet, u4Val)  SIF_WRITE32(u4Addr, ((SIF_READ32(u4Addr) & (~(u4Mask))) | (((u4Val) << (u4Offet)) & (u4Mask))))
#define SIF_READ_MASK(u4Addr, u4Mask, u4Offet)          ((SIF_READ32(u4Addr) & (u4Mask)) >> (u4Offet))

#define SIFM_DATA0_READ()   SIF_READ_MASK(SIF_SIFM0D0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET)  
#define SIFM_DATA1_READ()   SIF_READ_MASK(SIF_SIFM0D0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET)  
#define SIFM_DATA2_READ()   SIF_READ_MASK(SIF_SIFM0D0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET)  
#define SIFM_DATA3_READ()   SIF_READ_MASK(SIF_SIFM0D0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET)  
#define SIFM_DATA4_READ()   SIF_READ_MASK(SIF_SIFM0D1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET)  
#define SIFM_DATA5_READ()   SIF_READ_MASK(SIF_SIFM0D1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET)  
#define SIFM_DATA6_READ()   SIF_READ_MASK(SIF_SIFM0D1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET)  
#define SIFM_DATA7_READ()   SIF_READ_MASK(SIF_SIFM0D1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET)  

#define SIFM1_DATA0_READ()   SIF_READ_MASK(SIF_SIFM1D0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET)  
#define SIFM1_DATA1_READ()   SIF_READ_MASK(SIF_SIFM1D0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET)  
#define SIFM1_DATA2_READ()   SIF_READ_MASK(SIF_SIFM1D0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET)  
#define SIFM1_DATA3_READ()   SIF_READ_MASK(SIF_SIFM1D0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET)  
#define SIFM1_DATA4_READ()   SIF_READ_MASK(SIF_SIFM1D1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET)  
#define SIFM1_DATA5_READ()   SIF_READ_MASK(SIF_SIFM1D1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET)  
#define SIFM1_DATA6_READ()   SIF_READ_MASK(SIF_SIFM1D1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET)  
#define SIFM1_DATA7_READ()   SIF_READ_MASK(SIF_SIFM1D1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET)  


#define SIFM_DATA0_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET, u4Val)  
#define SIFM_DATA1_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET, u4Val)  
#define SIFM_DATA2_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET, u4Val)  
#define SIFM_DATA3_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET, u4Val)  
#define SIFM_DATA4_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET, u4Val)  
#define SIFM_DATA5_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET, u4Val)  
#define SIFM_DATA6_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET, u4Val)  
#define SIFM_DATA7_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM0D1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET, u4Val) 

#define SIFM1_DATA0_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D0, SIFM_DATA0_MASK, SIFM_DATA0_OFFSET, u4Val)  
#define SIFM1_DATA1_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D0, SIFM_DATA1_MASK, SIFM_DATA1_OFFSET, u4Val)  
#define SIFM1_DATA2_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D0, SIFM_DATA2_MASK, SIFM_DATA2_OFFSET, u4Val)  
#define SIFM1_DATA3_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D0, SIFM_DATA3_MASK, SIFM_DATA3_OFFSET, u4Val)  
#define SIFM1_DATA4_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D1, SIFM_DATA4_MASK, SIFM_DATA4_OFFSET, u4Val)  
#define SIFM1_DATA5_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D1, SIFM_DATA5_MASK, SIFM_DATA5_OFFSET, u4Val)  
#define SIFM1_DATA6_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D1, SIFM_DATA6_MASK, SIFM_DATA6_OFFSET, u4Val)  
#define SIFM1_DATA7_WRITE(u4Val)   SIF_WRITE_MASK(SIF_SIFM1D1, SIFM_DATA7_MASK, SIFM_DATA7_OFFSET, u4Val) 


#define SIFM_CLK_DIV_READ()        SIF_READ_MASK(SIF_SIFM0CTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET)
#define SIFM_CLK_DIV_WRITE(u4Val)  SIF_WRITE_MASK(SIF_SIFM0CTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET, u4Val)

#define SIFM1_CLK_DIV_READ()        SIF_READ_MASK(SIF_SIFM1CTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET)
#define SIFM1_CLK_DIV_WRITE(u4Val)  SIF_WRITE_MASK(SIF_SIFM1CTL0, SIFM_CLK_DIV_MASK, SIFM_CLK_DIV_OFFSET, u4Val)

#define SIFM_ACK_READ()             SIF_READ_MASK(SIF_SIFM0CTL1, SIFM_ACK_MASK, SIFM_ACK_OFFSET)
#define SIFM1_ACK_READ()            SIF_READ_MASK(SIF_SIFM1CTL1, SIFM_ACK_MASK, SIFM_ACK_OFFSET)

#define SIFM_PGLEN_READ()           SIF_READ_MASK(SIF_SIFM0CTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET)
#define SIFM_PGLEN_WRITE(u4Val)     SIF_WRITE_MASK(SIF_SIFM0CTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET, u4Val)
#define SIFM1_PGLEN_READ()          SIF_READ_MASK(SIF_SIFM1CTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET)
#define SIFM1_PGLEN_WRITE(u4Val)    SIF_WRITE_MASK(SIF_SIFM1CTL1, SIFM_PGLEN_MASK, SIFM_PGLEN_OFFSET, u4Val)

#define SIFM_SIF_MODE_READ()           SIF_READ_MASK(SIF_SIFM0CTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET)
#define SIFM_SIF_MODE_WRITE(u4Val)     SIF_WRITE_MASK(SIF_SIFM0CTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET, u4Val)
#define SIFM1_SIF_MODE_READ()          SIF_READ_MASK(SIF_SIFM1CTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET)
#define SIFM1_SIF_MODE_WRITE(u4Val)    SIF_WRITE_MASK(SIF_SIFM1CTL1, SIFM_SIF_MODE_MASK, SIFM_SIF_MODE_OFFSET, u4Val)

#define CLK_SRC_IS_27M()   ((SIF_READ32(UP_CFG) & FAST_CK_EN) == FAST_CK_EN)

#endif 
