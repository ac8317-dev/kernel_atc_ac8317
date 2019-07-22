#ifndef __AC83XX_SPI_H
#define __AC83XX_SPI_H __FILE__


//-----------------------------------------------------------------------------
// Constant definitions
//-----------------------------------------------------------------------------
#define IO_BASE_VA 0xFD000000
#define IO_BASE_SPI 0xFD000000
#define SPI_MOTO1
//#ifdef SPI_MOTO1
#define SPI_MOTO_REG_BASE                    (IO_BASE_VA+0x23400)
//#else   //SPI_MOTO2
#define SPI_MOTO2_REG_BASE                    (IO_BASE_VA+0x23800)
//#endif
#define HAL_WRITE32(_reg_, _val_)   		(*((volatile uint32_t*)(_reg_)) = (_val_))
#define HAL_READ32(_reg_)           		(*((volatile uint32_t*)(_reg_)))

#define SPI_IO_READ32(u4Addr) 	 	        HAL_READ32(IO_BASE_SPI+ u4Addr)
#define SPI_IO_WRITE32(u4Addr, u4Val) 	    HAL_WRITE32(IO_BASE_SPI+ u4Addr, u4Val)

#define SPI_MOTO_WRITE32(offset, value)       IO_WRITE32(SPI_MOTO_REG_BASE, (offset), (value))
#define SPI_MOTO_READ32(offset)               IO_READ32(SPI_MOTO_REG_BASE, (offset))

#define SPI_MOTO2_WRITE32(offset, value)       IO_WRITE32(SPI_MOTO2_REG_BASE, (offset), (value))
#define SPI_MOTO2_READ32(offset)               IO_READ32(SPI_MOTO2_REG_BASE, (offset))

#define AP_SELECT_CLOCK             ((UINT32)0x30)
  #define SPI_MOTO_SEL_CLK_MASK     ((UINT32)0x3<<8)
  #define SPI_MOTO_SEL_CLK27M       ((UINT32)0x00<<8)
  #define SPI_MOTO_SEL_SYSPLL_D3    ((UINT32)0x01<<8)
  #define SPI_MOTO_SEL_SYSPLL_D6    ((UINT32)0x02<<8)
  #define SPI_MOTO_SEL_APLL2_D2     ((UINT32)0x03<<8)

#define PAD_PINMUX2                 ((UINT32)0x5C)
  #define SPI_MOTO1_SEL_MASK        ((UINT32)0x7<<16)
  #define SPI_MOTO1_SEL_SP0         ((UINT32)0x01<<16)
  #define SPI_MOTO1_SEL_TD_D1       ((UINT32)0x02<<16)
  #define SPI_MOTO1_SEL_NLD0        ((UINT32)0x03<<16)
  #define SPI_MOTO1_SEL_SP1         ((UINT32)0x04<<16)
  #define SPI_MOTO1_SEL_TS_CLKIN    ((UINT32)0x05<<16)
  
  #define SPI_MOTO2_SEL_MASK        ((UINT32)0x7<<19)
  #define SPI_MOTO2_SEL_SP0         ((UINT32)0x01<<19)
  #define SPI_MOTO2_SEL_TS_D1       ((UINT32)0x02<<19)
  #define SPI_MOTO2_SEL_NLD0        ((UINT32)0x03<<19)
  #define SPI_MOTO2_SEL_SP1         ((UINT32)0x04<<19)
  #define SPI_MOTO2_SEL_TS_CLKIN    ((UINT32)0x05<<19)

#define MISC_CONTROL                ((UINT32)0x94)
  #define SPI_SEL_MASK              ((UINT32)0x3<<30)
  #define SPI_SEL_NONE              ((UINT32)0x00<<30)
  #define SPI_SEL_MOTO1             ((UINT32)0x01<<30)
  #define SPI_SEL_MOTO2             ((UINT32)0x02<<30)
  #define SPI_SEL_MOTO1_MOTO2       ((UINT32)0x03<<30)

#define SPI_MOTO_CLOCK              ((0xA8))
  #define SPI_MOTO1_CLOCK           ((UINT32)(UINT32)0x1<<22)
  #define SPI_MOTO2_CLOCK           ((UINT32)(UINT32)0x1<<23)

#define SPI_MOTO_RESET              ((0xC4))
  #define SPI_MOTO1_RESET           ((UINT32)(UINT32)0x1<<22)
  #define SPI_MOTO2_RESET           ((UINT32)(UINT32)0x1<<23)

//#define SPI_MOTO_REG_BASE           (IO_BASE+0x23400)
//SPI_MOTO1//#define SPI_MOTO_REG_BASE           (IO_BASE+0x23800) //SPI_MOTO2



#define SPI_FIFO_SIZE       32
#define SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES  0x400
#define SPI_INTERFACE_MAX_PKT_COUNT_PER_TIMES   0x100



#define SPI_CFG0                     0x00
#define _SPI_CS_SETUP_COUNT(x)          ((UINT32)(x & 0xFF)<<24)
#define _SPI_CS_HOLD_COUNT(x)           ((UINT32)(x & 0xFF)<<16)
#define _SPI_SCK_LOW_COUNT(x)           ((UINT32)(x & 0xFF)<<8)
#define _SPI_SCK_HIGH_COUNT(x)          ((UINT32)(x & 0xFF))

    
#define SPI_CFG1                        0x04
#define _SPI_PACKET_LENGTH(x)          ((UINT32)(x & 0x3FF)<<16)
#define _SPI_PACKET_LOOP_CNT(x)        ((UINT32)(x & 0xFF)<<8)
#define _SPI_CS_IDLE_COUNT(x)          ((UINT32)(x & 0xFF))
    
#define SPI_TX_SRC                      0x08
#define _SPI_TX_SRC(x)                 ((UINT32)(x))
    
#define SPI_RX_DST                      0x0C
#define _SPI_RX_DST(x)                 ((UINT32)(x))

#define SPI_TX_DATA                     0x10
#define _SPI_TX_DATA(x)                 ((UINT32)(x))

#define SPI_RX_DATA                     0x14
#define _SPI_RX_DATA(x)                 ((UINT32)(x))

#define SPI_CMD                         0x18

#define SPI_CMD_BIT_PAUSE_IE_MASK       ((UINT32)1<<17)
#define SPI_CMD_BIT_FINISH_IE_MASK      ((UINT32)1<<16)
#define SPI_CMD_BIT_TX_ENDIAN_MASK      ((UINT32)1<<15)
#define SPI_CMD_BIT_RX_ENDIAN_MASK      ((UINT32)1<<14)
#define SPI_CMD_BIT_RX_MSB_MASK         ((UINT32)1<<13)
#define SPI_CMD_BIT_TX_MSB_MASK         ((UINT32)1<<12)
#define SPI_CMD_BIT_TX_DMA_EN_MASK      ((UINT32)1<<11)
#define SPI_CMD_BIT_RX_DMA_EN_MASK      ((UINT32)1<<10)
#define SPI_CMD_BIT_CPOL_MASK           ((UINT32)1<<9)
#define SPI_CMD_BIT_CPHA_MASK           ((UINT32)1<<8)
#define SPI_CMD_BIT_CS_DEASSERT_EN_MASK ((UINT32)1<<5)
#define SPI_CMD_BIT_PAUSE_EN_MASK       ((UINT32)1<<4)
#define SPI_CMD_BIT_RESET_MASK          ((UINT32)1<<2)
#define SPI_CMD_BIT_RESUME_MASK         ((UINT32)1<<1)
#define SPI_CMD_BIT_CMD_ACT_MASK        ((UINT32)1<<0)

#define SPI_SKYPINE_MODE_INIT_TX		((UINT32)(SPI_CMD_BIT_TX_DMA_EN_MASK)|(SPI_CMD_BIT_TX_MSB_MASK)|(SPI_CMD_BIT_CPOL_MASK)|(SPI_CMD_BIT_CPHA_MASK))

#define SPI_SKYPINE_MODE_INIT_RX	((UINT32)(SPI_CMD_BIT_RX_DMA_EN_MASK)|(SPI_CMD_BIT_RX_MSB_MASK)|(SPI_CMD_BIT_CPOL_MASK)|(SPI_CMD_BIT_CPHA_MASK))


#define SPI_STATUS0                     0x1c

#define SPI_STATUS1_PAUSE_MASK          ((UINT32)1<<1)
#define SPI_STATUS1_FINISH_MASK         ((UINT32)1<<0)

#define SPI_STATUS1                     0x20
#define SPI_STATUS2_BUSY_MASK           ((UINT32)1<<0)

#define SPI_INT_STATUS_TMP              0x24
#define SPI_INT_STATUS                  ((UINT32)1<<0)
#define SPI_INT_CLEAR                   ((UINT32)1<<1)


struct ac83xx_spi_info {	
	int			 pin_cs;	/* simple gpio cs */
	unsigned int		 num_cs;/* total chipselects */
	int			 bus_num;       /* bus number to use. */
	unsigned int		 use_fiq:1;	/* use fiq */
	void (*gpio_setup)(struct ac83xx_spi_info *spi, int enable);
	void (*set_cs)(struct ac83xx_spi_info *spi, int cs, int pol);
};



#endif /* __LINUX_SPI_AC83XX_H */

