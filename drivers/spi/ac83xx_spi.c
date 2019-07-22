#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <mach/dma.h>
#include <asm/cacheflush.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <ac83xx_spi.h>
#include <ac83xx_spi_sw.h>
//#include "x_typedef.h"
#include "x_hal_ic.h"
#include <mach/ac83xx_gpio_pinmux.h>
#include <mach/ac83xx_pinmux_table.h>
#include <mach/pinmux.h>
#include "x_ver.h"
#define MODEBITS (SPI_CPOL|SPI_CPHA|SPI_CS_HIGH)
//version info
#define SPI_VER_NAME    "SPI"
#define SPI_VER_MAIN     01
#define SPI_VER_MINOR  00
#define SPI_VER_REV       00

struct ac83xx_spi{
    struct completion  done;

    void *regs;
    int   irq;
    int   len;

    void  (*set_cs)(struct ac83xx_spi_info *spi,int cs, int pol);

    struct clk *clk;
    struct resource   *ioarea;
    struct spi_master *master;
    struct spi_device *curdev;
    struct device     *dev;
    struct ac83xx_spi_info *pdata;
    struct resource *res;
	u8			*dmaTXbuffer;
	u8			*dmaRXbuffer;
    u32         dmaTXphyAddr;
    u32         dmaRXphyAddr;
    
};

u32 Set_SKYPINE_MODE_INIT_TX = 0;
u32 Set_SKYPINE_MODE_INIT_RX = 0;

UINT32 SPI_IsFinished(void)
{
    UINT32 u4Counter;

    u4Counter = 0;
    while(u4Counter < SPI_MOTIO_OPERATION_TIMEOUT)
    {
        if(SPI_MOTO_READ32(SPI_STATUS0)&SPI_STATUS1_FINISH_MASK)
        {
            return SPI_MOTO_OK;
        }
        else
        {
            u4Counter++;
        }
    }

    return SPI_MOTO_TIMEOUT;
}


static int spi_dma_init(struct ac83xx_spi *hw)
{
    
    if (hw->master->bus_num == 0) 
    {
        hw->dmaTXbuffer = dma_alloc_coherent(NULL, 
            SPI_DMA_BUF_SIZE , 
            &hw->dmaTXphyAddr, 
            GFP_KERNEL);    
        if (!hw->dmaTXbuffer) {
//            printk("Alloc DMA buffer fail at %d \n",__LINE__);
            return 0;
        }
//        printk("SPI0 hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
        memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE);

        if (hw->dmaTXphyAddr % 0x10)
        {
            dma_free_coherent(NULL, SPI_DMA_BUF_SIZE, (void *)hw->dmaTXbuffer, hw->dmaTXphyAddr);
            hw->dmaTXbuffer = 0;
            hw->dmaTXphyAddr = 0;
            hw->dmaTXbuffer = dma_alloc_coherent(NULL, 
                SPI_DMA_BUF_SIZE + 0x10, 
                &hw->dmaTXphyAddr, 
                GFP_KERNEL);    
            if (!hw->dmaTXbuffer) {
//                printk("Alloc DMA buffer fail at %d \n",__LINE__);
                return 0;
            }
            
//            printk("SPI0 hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
            memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE + 0x10);
            
            hw->dmaTXphyAddr = (hw->dmaTXphyAddr + 0x10)& 0xFFFFFFF0;
            hw->dmaTXbuffer = ((UINT32)(hw->dmaTXbuffer) + 0x10)& 0xFFFFFFF0;
//            printk("SPI0:Adjust hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
        }

        hw->dmaRXbuffer = dma_alloc_coherent(NULL, 
            SPI_DMA_BUF_SIZE , 
            &hw->dmaRXphyAddr, 
            GFP_KERNEL);    
        if (!hw->dmaRXbuffer ) {
//            printk("Alloc DMA buffer fail at Line%d \n",__LINE__);
            return 0;
        }
        memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE );
//        printk("SPI0 hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);

        if (hw->dmaRXphyAddr % 0x10)
        {
            dma_free_coherent(NULL, SPI_DMA_BUF_SIZE, (void *)hw->dmaRXbuffer, hw->dmaRXphyAddr);
            hw->dmaRXbuffer = 0;
            hw->dmaRXphyAddr = 0;
            hw->dmaRXbuffer = dma_alloc_coherent(NULL, 
                SPI_DMA_BUF_SIZE + 0x10, 
                &hw->dmaRXphyAddr, 
                GFP_KERNEL);    
            if (!hw->dmaRXbuffer) {
//                printk("Alloc DMA buffer fail at %d \n",__LINE__);
                return 0;
            }
//            printk("SPI0 hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);
            memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE + 0x10);
            
            hw->dmaRXphyAddr = (hw->dmaRXphyAddr + 0x10)& 0xFFFFFFF0;
            hw->dmaRXbuffer = ((UINT32)(hw->dmaRXbuffer) + 0x10)& 0xFFFFFFF0;
//            printk("SPI0:Adjust hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);
        }


    }

    if (hw->master->bus_num == 1) 
    {
        hw->dmaTXbuffer = dma_alloc_coherent(NULL, 
            SPI_DMA_BUF_SIZE , 
            &hw->dmaTXphyAddr, 
            GFP_KERNEL);    
        if (!hw->dmaTXbuffer) {
//            printk("Alloc DMA buffer fail at Line%d \n",__LINE__);
            return 0;
        }
//        printk("SPI1 hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
        memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE );

        if (hw->dmaTXphyAddr % 0x10)
        {
            dma_free_coherent(NULL, SPI_DMA_BUF_SIZE, (void *)hw->dmaTXbuffer, hw->dmaTXphyAddr);
            hw->dmaTXbuffer = 0;
            hw->dmaTXphyAddr = 0;
            hw->dmaTXbuffer = dma_alloc_coherent(NULL, 
                SPI_DMA_BUF_SIZE + 0x10, 
                &hw->dmaTXphyAddr, 
                GFP_KERNEL);    
            if (!hw->dmaTXbuffer) {
//                printk("Alloc DMA buffer fail at %d \n",__LINE__);
                return 0;
            }
            
//            printk("SPI1 hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
            memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE + 0x10);
            
            hw->dmaTXphyAddr = (hw->dmaTXphyAddr + 0x10)& 0xFFFFFFF0;
            hw->dmaTXbuffer = ((UINT32)(hw->dmaTXbuffer) + 0x10)& 0xFFFFFFF0;
//            printk("SPI1:Adjust hw->dmaTXbuffer = 0x%08X hw->dmaTXphyAddr = 0x%08x \n",hw->dmaTXbuffer,hw->dmaTXphyAddr);
        }

        hw->dmaRXbuffer = dma_alloc_coherent(NULL, 
            SPI_DMA_BUF_SIZE , 
            &hw->dmaRXphyAddr, 
            GFP_KERNEL);    
        if (!hw->dmaRXbuffer ) {
//            printk("Alloc DMA buffer fail at %d \n",__LINE__);
            return 0;
        }
        memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE );
//        printk("SPI1 hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);
        if (hw->dmaRXphyAddr % 0x10)
        {
            dma_free_coherent(NULL, SPI_DMA_BUF_SIZE, (void *)hw->dmaRXbuffer, hw->dmaRXphyAddr);
            hw->dmaRXbuffer = 0;
            hw->dmaRXphyAddr = 0;
            hw->dmaRXbuffer = dma_alloc_coherent(NULL, 
                SPI_DMA_BUF_SIZE + 0x10, 
                &hw->dmaRXphyAddr, 
                GFP_KERNEL);    
            if (!hw->dmaRXbuffer) {
//                printk("Alloc DMA buffer fail at %d \n",__LINE__);
                return 0;
            }
            
//            printk("SPI1 hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);
            memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE + 0x10);
            
            hw->dmaRXphyAddr = (hw->dmaRXphyAddr + 0x10)& 0xFFFFFFF0;
            hw->dmaRXbuffer = ((UINT32)(hw->dmaRXbuffer) + 0x10)& 0xFFFFFFF0;
//            printk("SPI1:Adjust hw->dmaRXbuffer = 0x%08X hw->dmaRXphyAddr = 0x%08x \n",hw->dmaRXbuffer,hw->dmaRXphyAddr);
        }

    }
    return 0;

}

static int spi_init(struct ac83xx_spi *sdd)
{
    UINT32 u4Tmp = 0;
    
    if(sdd->master->bus_num == 0)
    {
        u4Tmp = SPI_IO_READ32(MISC_CONTROL);
        u4Tmp &= ~SPI_SEL_MASK;
        u4Tmp |= SPI_SEL_MOTO1_MOTO2;
        SPI_IO_WRITE32(MISC_CONTROL, u4Tmp);

        gpio_request(PIN_119_SP0_CLK,"PIN_119_SP0_CLK");
        GPIO_MultiFun_Set(PIN_119_SP0_CLK, SP0_SEL);
        gpio_request(PIN_120_SP0_CS,"PIN_120_SP0_CS");
        GPIO_MultiFun_Set(PIN_120_SP0_CS,   SP0_SEL);
        gpio_request(PIN_121_SP0_SI,"PIN_121_SP0_SI");
        GPIO_MultiFun_Set(PIN_121_SP0_SI,    SP0_SEL);
        gpio_request(PIN_122_SP0_SO,"PIN_122_SP0_SO");
        GPIO_MultiFun_Set(PIN_122_SP0_SO,   SP0_SEL);

        u4Tmp = SPI_IO_READ32(SPI_MOTO_CLOCK);
        u4Tmp |= SPI_MOTO1_CLOCK;
        SPI_IO_WRITE32(SPI_MOTO_CLOCK, u4Tmp);
       
        u4Tmp = SPI_IO_READ32(SPI_MOTO_RESET);
        u4Tmp |= SPI_MOTO1_RESET;
        SPI_IO_WRITE32(SPI_MOTO_RESET, u4Tmp);

        u4Tmp = SPI_IO_READ32(AP_SELECT_CLOCK);
        u4Tmp &= ~SPI_MOTO_SEL_CLK_MASK;
        u4Tmp |= SPI_MOTO_SEL_CLK27M;
        SPI_IO_WRITE32(AP_SELECT_CLOCK, u4Tmp);

        SPI_MOTO_WRITE32(SPI_CFG0, SPI_CF0_DEFAULT_VALUE);
        SPI_MOTO_WRITE32(SPI_CFG1,SPI_CF1_DEFAULT_VALUE);
        
    }
    if(sdd->master->bus_num == 1)
    {
        u4Tmp = SPI_IO_READ32(MISC_CONTROL);
        u4Tmp &= ~SPI_SEL_MASK;
        u4Tmp |= SPI_SEL_MOTO1_MOTO2;
        SPI_IO_WRITE32(MISC_CONTROL, u4Tmp);

        gpio_request(PIN_123_SP1_CLK,"PIN_123_SP1_CLK");
        GPIO_MultiFun_Set(PIN_123_SP1_CLK, SP1_SEL);
        gpio_request(PIN_126_SP1_CS,"PIN_126_SP1_CS");
        GPIO_MultiFun_Set(PIN_126_SP1_CS,   SP1_SEL);
        gpio_request(PIN_127_SP1_SI,"PIN_127_SP1_SI");
        GPIO_MultiFun_Set(PIN_127_SP1_SI,    SP1_SEL);
        gpio_request(PIN_128_SP1_SO,"PIN_128_SP1_SO");
        GPIO_MultiFun_Set(PIN_128_SP1_SO,   SP1_SEL);

        u4Tmp = SPI_IO_READ32(SPI_MOTO_CLOCK);
        u4Tmp |= SPI_MOTO2_CLOCK;
        SPI_IO_WRITE32(SPI_MOTO_CLOCK, u4Tmp);
        
        u4Tmp = SPI_IO_READ32(SPI_MOTO_RESET);
        u4Tmp |= SPI_MOTO2_RESET;
        SPI_IO_WRITE32(SPI_MOTO_RESET, u4Tmp);

        u4Tmp = SPI_IO_READ32(AP_SELECT_CLOCK);
        u4Tmp &= ~SPI_MOTO_SEL_CLK_MASK;
        u4Tmp |= SPI_MOTO_SEL_CLK27M;
        SPI_IO_WRITE32(AP_SELECT_CLOCK, u4Tmp);

        SPI_MOTO2_WRITE32(SPI_CFG0,SPI_CF0_DEFAULT_VALUE);
        SPI_MOTO2_WRITE32(SPI_CFG1,SPI_CF1_DEFAULT_VALUE);
       
    }
    
    return 0;
}


static inline struct ac83xx_spi *to_hw(struct spi_device *sdev)
{
    return spi_master_get_devdata(sdev->master);
}
static void ac83xx_spi_gpiocs(struct ac83xx_spi_info *spi,int cs, int pol)
{
    gpio_set_value(spi->pin_cs,pol);
}
UINT32 SPI_PopFifo(SPI_DIRECTION_TYPE const direction, UINT32 * data)
{

    if (data == NULL)
    {
        return 0;
    }

    switch (direction)
    {
    case SPI_TX:
        *data = SPI_MOTO_READ32(SPI_TX_DATA);  
        break;
    case SPI_RX:
        *data = SPI_MOTO_READ32(SPI_RX_DATA);
        break;
    default:    
        return 0;
    }

    return 1;
}

static void ac83xx_spi_chipsel(struct spi_device *spi)
{
    unsigned int spcon;
    if(spi->master->bus_num == 0)
    {
        spcon = SPI_MOTO_READ32(SPI_CMD);
    

        if(spi->mode&SPI_CPHA)
            spcon |=SPI_CMD_BIT_CPHA_MASK; 
        else 
            spcon &=~SPI_CMD_BIT_CPHA_MASK;

        if(spi->mode&SPI_CPOL)
            spcon|=SPI_CMD_BIT_CPOL_MASK;
        else
            spcon&=~SPI_CMD_BIT_CPOL_MASK;
        
        SPI_MOTO_WRITE32(SPI_CMD, spcon);
       
    }
    if(spi->master->bus_num == 1)
    {
        spcon = SPI_MOTO2_READ32(SPI_CMD);
    

        if(spi->mode&SPI_CPHA)
            spcon |=SPI_CMD_BIT_CPHA_MASK; 
        else 
            spcon &=~SPI_CMD_BIT_CPHA_MASK;

        if(spi->mode&SPI_CPOL)
            spcon|=SPI_CMD_BIT_CPOL_MASK;
        else
            spcon&=~SPI_CMD_BIT_CPOL_MASK;
            
        SPI_MOTO2_WRITE32(SPI_CMD, spcon);
      
    }
    

}
static int ac83xx_spi_setupxfer(struct spi_device *spi,struct spi_transfer *t)
{
    //struct ac83xx_spi *hw = to_hw(spi);
    unsigned int bpw;
    unsigned int hz;

    bpw = t?t->bits_per_word:spi->bits_per_word;
    hz  = t?t->speed_hz:spi->max_speed_hz;

    if(bpw!=8&&bpw!=16&&bpw!=32)
    {
        dev_err(&spi->dev,"invalid bit-per-word(%d)\n",bpw);
        return -1;//EINVAl;
    }
    
    ac83xx_spi_chipsel(spi);
    
    return 0;
}
static int ac83xx_spi_setup(struct spi_device *spi)
{
    int ret;

    if(!spi->bits_per_word)//default value is 8
        spi->bits_per_word = 8;

    if(spi->mode&~MODEBITS)
    {
        dev_dbg(&spi->dev,"setup:unsuooort mode bit %x\n",spi->mode&~MODEBITS);
        return -1;//EINVAl;
    }
//    printk("ac83xx_spi_set_up start\n");
    ret = ac83xx_spi_setupxfer(spi,NULL);
    if(ret<0)
    {
        dev_err(&spi->dev,"setupxfer returned %d\n",ret);
        return ret;
    }
    dev_dbg(&spi->dev,"%d:mode, %u bpw, %d hz\n",spi->mode,spi->bits_per_word,spi->max_speed_hz);

    return 0;
}
void SPI_SetPauseMode(UINT32 const status)
{

    UINT32 value;

    value = SPI_MOTO_READ32(SPI_CMD);

    if (status == 1)
       SPI_MOTO_WRITE32(SPI_CMD, value|SPI_CMD_BIT_PAUSE_EN_MASK);
   else
       SPI_MOTO_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_PAUSE_EN_MASK)); 

}
void SPI2_SetPauseMode(UINT32 const status)
{

    UINT32 value;

    value = SPI_MOTO2_READ32(SPI_CMD);

    if (status == 1)
       SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_CMD_BIT_PAUSE_EN_MASK);
   else
       SPI_MOTO2_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_PAUSE_EN_MASK)); 

}

void SPI_PauseModeResume(UINT32 const status)
{

    UINT32 value;

    value = SPI_MOTO_READ32(SPI_CMD);
    value = value|(SPI_CMD_BIT_RESUME_MASK|SPI_CMD_BIT_RESET_MASK);
    if (status == 1) 
	   SPI_MOTO_WRITE32(SPI_CMD, value|SPI_CMD_BIT_PAUSE_EN_MASK);
    else
       SPI_MOTO_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_PAUSE_EN_MASK));
}
void SPI2_PauseModeResume(UINT32 const status)
{

    UINT32 value;

    value = SPI_MOTO2_READ32(SPI_CMD);
	value = value|(SPI_CMD_BIT_RESUME_MASK|SPI_CMD_BIT_RESET_MASK);
	if (status == 1) 
	   SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_CMD_BIT_PAUSE_EN_MASK);
    else
       SPI_MOTO2_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_PAUSE_EN_MASK));

}

void SPI_Activate(void)  
{

    UINT32 value;

    value = SPI_MOTO_READ32(SPI_CMD);
    value = value|(SPI_CMD_BIT_CMD_ACT_MASK|SPI_CMD_BIT_RESET_MASK);
    SPI_MOTO_WRITE32(SPI_CMD, value);

}

void SPI2_Activate(void)  
{

    UINT32 value;

    value = SPI_MOTO2_READ32(SPI_CMD);
    value = value|(SPI_CMD_BIT_CMD_ACT_MASK|SPI_CMD_BIT_RESET_MASK);
    SPI_MOTO2_WRITE32(SPI_CMD, value);

}

long GetTickcount(void)
{
    struct timespec tv;

    getnstimeofday(&tv);

    return tv.tv_sec;
}

UINT32 SPI_SelectMode(SPI_DIRECTION_TYPE const type, SPI_MODE const mode)
{

    UINT32 value;

    value = SPI_MOTO_READ32(SPI_CMD);

    if ((mode != SPI_MODE_DMA) && (mode != SPI_MODE_FIFO))
    {
//        printk("SPI_SelectMode: Incorrect spi_mode parameter.\r\n");
        return 0;
    }

    switch (type)

    {

        case SPI_TX:
            if (mode == SPI_MODE_DMA)
            {//skypine start
                //SPI_MOTO_WRITE32(SPI_CMD, value|SPI_CMD_BIT_TX_DMA_EN_MASK);
				//printk("Set_SKYPINE_MODE_INIT_TX = %d\n", Set_SKYPINE_MODE_INIT_TX);
                if(Set_SKYPINE_MODE_INIT_TX == 0)
                	SPI_MOTO_WRITE32(SPI_CMD, value|SPI_SKYPINE_MODE_INIT_TX);
                else
                	SPI_MOTO_WRITE32(SPI_CMD, (value & 0xFCFF) | (Set_SKYPINE_MODE_INIT_TX & 0xFFFF));
            }//skypine end
            else
                SPI_MOTO_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_TX_DMA_EN_MASK));      
            break;

        case SPI_RX:
            if (mode == SPI_MODE_DMA)
            {//skypine start
                //SPI_MOTO_WRITE32(SPI_CMD, value|SPI_CMD_BIT_RX_DMA_EN_MASK);
				//printk("Set_SKYPINE_MODE_INIT_RX = %d\n", Set_SKYPINE_MODE_INIT_RX);
                if(Set_SKYPINE_MODE_INIT_RX == 0)
                	SPI_MOTO_WRITE32(SPI_CMD, value|SPI_SKYPINE_MODE_INIT_RX);
                else
                	SPI_MOTO_WRITE32(SPI_CMD, (value & 0xFCFF) | (Set_SKYPINE_MODE_INIT_RX & 0xFFFF));
            }//skypine end
            else
                SPI_MOTO_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_RX_DMA_EN_MASK));            
            break;

        default:
//           printk("SPI_SelectMode: Incorrect spi_direction_type parameter.\r\n");
           return 0;
    }

   return 1;
}

UINT32 SPI2_SelectMode(SPI_DIRECTION_TYPE const type, SPI_MODE const mode)
{

    UINT32 value;

    value = SPI_MOTO2_READ32(SPI_CMD);

    if ((mode != SPI_MODE_DMA) && (mode != SPI_MODE_FIFO))
    {
//        printk("SPI_SelectMode: Incorrect spi_mode parameter.\r\n");
        return 0;
    }

    switch (type)

    {

        case SPI_TX:
            if (mode == SPI_MODE_DMA)
            {//skypine start
                //SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_CMD_BIT_TX_DMA_EN_MASK);
                if(Set_SKYPINE_MODE_INIT_TX == 0)
                	SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_SKYPINE_MODE_INIT_TX);
                else
                	SPI_MOTO2_WRITE32(SPI_CMD, (value & 0xFCFF) | (Set_SKYPINE_MODE_INIT_TX & 0xFFFF));
            }//start end
            else
                SPI_MOTO2_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_TX_DMA_EN_MASK));      
            break;

        case SPI_RX:
            if (mode == SPI_MODE_DMA)
            {//skypine start
                //SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_CMD_BIT_RX_DMA_EN_MASK);
                 if(Set_SKYPINE_MODE_INIT_RX == 0)
                	SPI_MOTO2_WRITE32(SPI_CMD, value|SPI_SKYPINE_MODE_INIT_RX);
                 else
                 	SPI_MOTO2_WRITE32(SPI_CMD, (value & 0xFCFF) | (Set_SKYPINE_MODE_INIT_RX & 0xFFFF));
            }//skypine end
            else
                SPI_MOTO2_WRITE32(SPI_CMD, value&(~SPI_CMD_BIT_RX_DMA_EN_MASK));            
            break;

        default:
//           printk("SPI_SelectMode: Incorrect spi_direction_type parameter.\r\n");
           return 0;
    }

   return 1;
}

UINT32 SPI_ClearFifo(SPI_DIRECTION_TYPE const direction)
{

    UINT32 i;
    UINT32 volatile tmp;  

    for (i = 0; i < (SPI_FIFO_SIZE/4); ++i)
    {
        switch (direction)
        {
        case SPI_TX:
            SPI_MOTO_WRITE32(SPI_TX_DATA, 0x0);
            break;
        case SPI_RX:
            tmp = SPI_MOTO_READ32(SPI_RX_DATA);
            break;
        default:     
//           printk("SPI_ClearFifo: Incorrect spi_direction_type parameter.\r\n");
            return 0;
        }
    }

    return 1;

}

UINT32 SPI2_ClearFifo(SPI_DIRECTION_TYPE const direction)
{

    UINT32 i;
    UINT32 volatile tmp;  

    for (i = 0; i < (SPI_FIFO_SIZE/4); ++i)
    {
        switch (direction)
        {
        case SPI_TX:
            SPI_MOTO2_WRITE32(SPI_TX_DATA, 0x0);
            break;
        case SPI_RX:
            tmp = SPI_MOTO2_READ32(SPI_RX_DATA);
            break;
        default:     
//           printk("SPI_ClearFifo: Incorrect spi_direction_type parameter.\r\n");
            return 0;
        }
    }

    return 1;

}

UINT32 SPI_SetRWAddr(SPI_DIRECTION_TYPE const type, UINT32 addr)
{

    if ((type != SPI_TX) && (type != SPI_RX))
    {
//        printk("SPI_SetRWAddr: Incorrect SPI_Direction_Type parameter.\r\n");
        return 0;
    }

    if (0 == addr)
    {
//        printk("SPI_SetRWAddr: Incorrect address parameter.\r\n");
        return 0;
    }
    
    if ((addr & 0x3) != 0)
    {
//        printk("SPI_SetRWAddr: Incorrect address illegal.\r\n");
        return 0;
    }
    
    if (SPI_TX == type)
    {
        SPI_MOTO_WRITE32(SPI_TX_SRC, addr);
    }
    else
    {
        SPI_MOTO_WRITE32(SPI_RX_DST, addr);    
    }

    return 1;

}

UINT32 SPI2_SetRWAddr(SPI_DIRECTION_TYPE const type, UINT32 addr)
{

    if ((type != SPI_TX) && (type != SPI_RX))
    {
//        printk("SPI_SetRWAddr: Incorrect SPI_Direction_Type parameter.\r\n");
        return 0;
    }

    if (0 == addr)
    {
//        printk("SPI_SetRWAddr: Incorrect address parameter.\r\n");
        return 0;
    }
    
    if ((addr & 0x3) != 0)
    {
//        printk("SPI_SetRWAddr: Incorrect address illegal.\r\n");
        return 0;
    }
    
    if (SPI_TX == type)
    {
        SPI_MOTO2_WRITE32(SPI_TX_SRC, addr);
    }
    else
    {
        SPI_MOTO2_WRITE32(SPI_RX_DST, addr);    
    }

    return 1;

}

UINT32 SPI_SetDesiredSize(UINT16 const pkg_length, UINT16 const pkg_count)
{

    UINT32 u4Value;

    if ((pkg_length > SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES) || (0 == pkg_length))
    {
//        printk("SPI_SetDesiredSize: pkg_length is illegal.\r\n");
        return 0;
    }

    if ((pkg_count > SPI_INTERFACE_MAX_PKT_COUNT_PER_TIMES) || (0 == pkg_count))
    {
//        printk("SPI_SetDesiredSize: pkg_count is illegal.\r\n");
        return 0;
    }

    if (0 == pkg_length)
    {
        return 0;
    }

    if (0 == pkg_count)
    {
        return 0;
    }

    u4Value = SPI_MOTO_READ32(SPI_CFG1);
    u4Value &=~0x3FFFF00;
    u4Value |=((pkg_count - 1) << 8);
    u4Value |=((pkg_length - 1) << 16);

    /* Set 'PACKET_LOOP_CNT' field. */
    SPI_MOTO_WRITE32(SPI_CFG1,u4Value);

    return 1;
}

UINT32 SPI2_SetDesiredSize(UINT16 const pkg_length, UINT16 const pkg_count)
{

    UINT32 u4Value;

    if ((pkg_length > SPI_INTERFACE_MAX_PKT_LENGTH_PER_TIMES) || (0 == pkg_length))
    {
//        printk("SPI_SetDesiredSize: pkg_length is illegal.\r\n");
        return 0;
    }

    if ((pkg_count > SPI_INTERFACE_MAX_PKT_COUNT_PER_TIMES) || (0 == pkg_count))
    {
//        printk("SPI_SetDesiredSize: pkg_count is illegal.\r\n");
        return 0;
    }

    if (0 == pkg_length)
    {
        return 0;
    }

    if (0 == pkg_count)
    {
        return 0;
    }

    u4Value = SPI_MOTO2_READ32(SPI_CFG1);
    u4Value &=~0x3FFFF00;
    u4Value |=((pkg_count - 1) << 8);
    u4Value |=((pkg_length - 1) << 16);

    /* Set 'PACKET_LOOP_CNT' field. */
    SPI_MOTO2_WRITE32(SPI_CFG1,u4Value);

    return 1;
}

UINT32 SPI_WaitFinished(void)
{

    long counter,counter1;

    counter1 = 0;
    counter = GetTickcount();

    while(counter1 < (SPI_MOTIO_OPERATION_TIMEOUT/1000))
    {
        if(SPI_MOTO_READ32(SPI_STATUS1))
        {
            return SPI_MOTO_OK;
        }
        else
        {
            counter1 = GetTickcount() - counter;
        }
    }

    return SPI_MOTO_TIMEOUT; 
}

UINT32 SPI2_WaitFinished(void)
{

    long counter,counter1;

    counter1 = 0;
    counter = GetTickcount();

    while(counter1 < (SPI_MOTIO_OPERATION_TIMEOUT/1000))
    {
        if(SPI_MOTO2_READ32(SPI_STATUS1))
        {
            return SPI_MOTO_OK;
        }
        else
        {
            counter1 = GetTickcount() - counter;
        }
    }

    return SPI_MOTO_TIMEOUT; 
}

BOOL SPI_SetTransactionLength(UINT32 dwCount)
{
    UINT16 u2PkgLength = 0, u2PkgCount = 0;
    UINT32 dwTmp;

    if (dwCount > MAX_TRANSCATION_BYTE)
    {
//        printk("SPI_SetTransactionLength: Incorrect dwCount parameter.\r\n");
        return FALSE;
    }

    // find a good pair for the tarnsaction length
    for (u2PkgCount = 1; u2PkgCount <= MAX_PACKET_LOOP_CNT; u2PkgCount++)
    {
        dwTmp = (dwCount / u2PkgCount);
        if (dwTmp <= MAX_PACKET_LENGTH)
        {
            u2PkgLength = (UINT16)dwTmp;
            break;
        }
    }

    return SPI_SetDesiredSize(u2PkgLength, u2PkgCount);
}

BOOL SPI2_SetTransactionLength(UINT32 dwCount)
{
    UINT16 u2PkgLength = 0, u2PkgCount = 0;
    UINT32 dwTmp;

    if (dwCount > MAX_TRANSCATION_BYTE)
    {
//        printk("SPI_SetTransactionLength: Incorrect dwCount parameter.\r\n");
        return FALSE;
    }

    // find a good pair for the tarnsaction length
    for (u2PkgCount = 1; u2PkgCount <= MAX_PACKET_LOOP_CNT; u2PkgCount++)
    {
        dwTmp = (dwCount / u2PkgCount);
        if (dwTmp <= MAX_PACKET_LENGTH)
        {
            u2PkgLength = (UINT16)dwTmp;
            break;
        }
    }

    return SPI2_SetDesiredSize(u2PkgLength, u2PkgCount);
}



SPI_STATE SPI_StartTransaction(SPI_STATE spiState, UINT32 dwCount)
{
    SPI_STATE state;

    if (dwCount > MAX_TRANSCATION_BYTE)
    {
        state = (spiState == SPI_STATE_IDLE2IDLE)?SPI_STATE_IDLE2PAUSE: SPI_STATE_PAUSE2PAUSE;
    }
    else
    {
        state = (spiState == SPI_STATE_PAUSE2PAUSE)?SPI_STATE_PAUSE2IDLE: SPI_STATE_IDLE2IDLE;
    }

    return state;
}


UINT32 SPI_StartTransactions(UINT32 dwCount, LPVOID pBufferTx, LPVOID pBufferRx)
{
    UINT32 dwTranscationLength = 0;
    UINT32 bRet = 1;
    
    UINT32 pTx=0, pRx=0;
    UINT32 spiState = SPI_STATE_IDLE2IDLE;
    UINT32 dwRty = 0;
    UINT32 dwLeft = 0;
	UINT32 dwTanscationAgain = 0;
    
     if(pBufferRx == NULL && pBufferTx == NULL)
    {
//        printk("pBufferRx and pBufferTx is NULL\n");
        return 0;
    }
    if(NULL != pBufferTx)
        pTx = (UINT32)pBufferTx;
    if(NULL != pBufferRx)
        pRx = (UINT32)pBufferRx;

    if(dwCount > 1024)
		dwTanscationAgain = 2;
    
#if 0
    if (pTx && pRx)
    {
        TX_RX_InterruptMode = RXInterruptMode; //RX
    }
    else if (pTx)    // Tx with small transaction bytes for firmware download
    {
        TX_RX_InterruptMode = TXInterruptMode; //TX
    }
    else        // Rx
    {
        TX_RX_InterruptMode = RXInterruptMode; //RX
    }
#endif    
//    printk("set bufferTx and bufferRx\n");
    flush_cache_all();
    while(dwCount > 0)
    {
        dwTranscationLength = ((dwCount > MAX_TRANSCATION_BYTE) ? MAX_TRANSCATION_BYTE: dwCount);
        dwRty = dwTranscationLength / 1024;
        if (dwRty > 0)
        {
            dwLeft = dwTranscationLength - dwRty*1024;
        }
        else
        {
            dwLeft = 0;
        }

        if(dwTanscationAgain == 2)
		{
			spiState = SPI_STATE_IDLE2PAUSE;
		}
		else if(dwTanscationAgain == 1)
		{
			spiState = SPI_STATE_PAUSE2IDLE;
		}
        SPI_SetTransactionLength(dwTranscationLength - dwLeft);

        if (pBufferTx != NULL)
        {
            SPI_SetRWAddr(SPI_TX, (UINT32)pTx);
            pTx+= (dwTranscationLength - dwLeft);
//            printk("send Tx\n");
        }
    
        if (pBufferRx != NULL)
        {
            SPI_SetRWAddr(SPI_RX, (UINT32)pRx);
            pRx+= (dwTranscationLength - dwLeft);
//            printk("rescive Rx\n");
        }


        switch (spiState)
        {
        case SPI_STATE_IDLE2IDLE:
            SPI_SetPauseMode(0);
            SPI_Activate( );
//            printk("SPI Activate:SPI_STATE_IDLE2IDLE\n");
            break;
        case SPI_STATE_IDLE2PAUSE:
            SPI_SetPauseMode(1);
            SPI_Activate( );
//            printk("SPI Activate:SPI_STATE_IDLE2PAUSE\n");
            break;
        case SPI_STATE_PAUSE2PAUSE:
            SPI_PauseModeResume(1);
//            printk("SPI Resume:SPI_STATE_PAUSE2PAUSE\n");
            break;
        case SPI_STATE_PAUSE2IDLE:
            SPI_PauseModeResume(0);
//            printk("SPI Resume:SPI_STATE_PAUSE2IDLE\n");
            break;
        default:
            break;
        }    
#if 0
        if (WaitComplete() == FALSE)
        {
            printk("SPI_StartTransactions:WaitComplete failed.\r\n");
            bRet = 0;
            break;
        }
#else
        bRet = SPI_WaitFinished();  
        if(bRet == SPI_MOTO_TIMEOUT)
        {
//            printk("SPI DMA WRITE TIMEOUT.\r\n");
            break;
        }
        
#endif
        
        dwCount -= (dwTranscationLength - dwLeft);
        dwTanscationAgain --;
      
    }
    flush_cache_all();
    
    return bRet;
}

UINT32 SPI2_StartTransactions(UINT32 dwCount, LPVOID pBufferTx, LPVOID pBufferRx)
{
    UINT32 dwTranscationLength = 0;
    UINT32 bRet = 1;
    UINT32 pTx=0, pRx=0;
    UINT32 spiState = SPI_STATE_IDLE2IDLE;
    UINT32 dwRty = 0;
    UINT32 dwLeft = 0;
	UINT32 dwTanscationAgain = 0;
    

    if(pBufferRx == NULL && pBufferTx == NULL)
    {
//        printk("pBufferRx and pBufferTx is NULL\n");
        return 0;
    }
    if(NULL != pBufferTx)
        pTx = pBufferTx;
    if(NULL != pBufferRx)
        pRx = pBufferRx;

	if(dwCount > 1024)
		dwTanscationAgain = 2;
    
#if 0
    if (pTx && pRx)
    {
        TX_RX_InterruptMode = RXInterruptMode; //RX
    }
    else if (pTx)    // Tx with small transaction bytes for firmware download
    {
        TX_RX_InterruptMode = TXInterruptMode; //TX
    }
    else        // Rx
    {
        TX_RX_InterruptMode = RXInterruptMode; //RX
    }
#endif    
//    printk("set bufferTx and bufferRx\n");
    flush_cache_all();
    while(dwCount > 0)
    {
        dwTranscationLength = ((dwCount > MAX_TRANSCATION_BYTE) ? MAX_TRANSCATION_BYTE: dwCount);
        dwRty = dwTranscationLength / 1024;
        if (dwRty > 0)
        {
            dwLeft = dwTranscationLength - dwRty*1024;
        }
        else
        {
            dwLeft = 0;

        }
		
		if(dwTanscationAgain == 2)
		{
			spiState = SPI_STATE_IDLE2PAUSE;
		}
		else if(dwTanscationAgain == 1)
		{
			spiState = SPI_STATE_PAUSE2IDLE;
		}
        SPI2_SetTransactionLength(dwTranscationLength - dwLeft);

        if (pBufferTx != NULL)
        {
            SPI2_SetRWAddr(SPI_TX, (UINT32)pTx);
            pTx+= (dwTranscationLength - dwLeft);
//            printk("send Tx\n");
        }
    
        if (pBufferRx != NULL)
        {
            SPI2_SetRWAddr(SPI_RX, (UINT32)pRx);
            pRx+= (dwTranscationLength - dwLeft);
//            printk("rescive Rx\n");
        }

        switch (spiState)
        {
        case SPI_STATE_IDLE2IDLE:
            SPI2_SetPauseMode(0);
            SPI2_Activate( );
//            printk("SPI 2 Activate:SPI_STATE_IDLE2IDLE\n");
            break;
        case SPI_STATE_IDLE2PAUSE:
            SPI2_SetPauseMode(1);
            SPI2_Activate( );
//            printk("SPI 2 Activate:SPI_STATE_IDLE2PAUSE\n");
            break;
        case SPI_STATE_PAUSE2PAUSE:
            SPI2_PauseModeResume(1);
//            printk("SPI 2 Resume:SPI_STATE_PAUSE2PAUSE\n");
            break;
        case SPI_STATE_PAUSE2IDLE:
            SPI2_PauseModeResume(0);
//            printk("SPI 2 Resume:SPI_STATE_PAUSE2IDLE\n");
            break;
        default:
            break;
        }    
#if 0
        if (WaitComplete() == FALSE)
        {
            printk("SPI_StartTransactions:WaitComplete failed.\r\n");
            bRet = 0;
            break;
        }
#else
        bRet = SPI2_WaitFinished();  
        if(bRet == SPI_MOTO_TIMEOUT)
        {
//            printk("SPI 2 DMA WRITE TIMEOUT.\r\n");
            break;
        }
        
#endif
        
        dwCount -= (dwTranscationLength - dwLeft);
		dwTanscationAgain --;
    }
    flush_cache_all();
    
    return bRet;
}

static int ac83xx_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct ac83xx_spi *hw = to_hw(spi);
    struct spi_transfer *xfer;
//    printk("transfer start\n");
    int spiRet = 0;
    if(hw == NULL)
    {
//        printk("hw is null \n");
        return -EIO;
    }

    init_completion(&hw->done);
    if(spi->master->bus_num == 0)
    {
        SPI_SelectMode(SPI_TX,SPI_MODE_FIFO);
        SPI_ClearFifo(SPI_TX);

        SPI_SelectMode(SPI_TX,SPI_MODE_DMA);
        SPI_SelectMode(SPI_RX,SPI_MODE_DMA);
//        printk("mode select ok !\n");
        list_for_each_entry(xfer,&msg->transfers,transfer_list)
        {

//            printk("SPI0: xfer->tx_buf = 0x%08X xfer->rx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->tx_buf,xfer->rx_buf,xfer->len);
            memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE);
            memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE);

            if (xfer->tx_buf && xfer->len <= SPI_DMA_BUF_SIZE)
                memcpy(hw->dmaTXbuffer,xfer->tx_buf,xfer->len);
            else
               ;// printk("SPI0:empty buf or low size, xfer->tx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->tx_buf,xfer->len);
                
            hw->len= xfer->len;
            if (SPI_StartTransactions(hw->len, (LPVOID)hw->dmaTXphyAddr, (LPVOID)hw->dmaRXphyAddr) == 1)
            {
                spiRet = 0;
                
                if (xfer->rx_buf && xfer->len <= SPI_DMA_BUF_SIZE)
                    memcpy(xfer->rx_buf,hw->dmaRXbuffer,xfer->len);
                else
                   ;// printk("SPI0:SPI:empty buf or low size,xfer->rx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->rx_buf,xfer->len);
            }
            else
            {
                spiRet = -1;
//                printk("SPI: Incorrect Transactions.\r\n");
            }   
        }
    }
    if(spi->master->bus_num == 1)
    {
        SPI2_SelectMode(SPI_TX,SPI_MODE_FIFO);
        SPI2_ClearFifo(SPI_TX);

        SPI2_SelectMode(SPI_TX,SPI_MODE_DMA);
        SPI2_SelectMode(SPI_RX,SPI_MODE_DMA);
//        printk("mode select ok !\n");
        list_for_each_entry(xfer,&msg->transfers,transfer_list)
        {
//            printk("SPI1: xfer->tx_buf = 0x%08X xfer->rx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->tx_buf,xfer->rx_buf,xfer->len);
            memset(hw->dmaTXbuffer,0,SPI_DMA_BUF_SIZE);
            memset(hw->dmaRXbuffer,0,SPI_DMA_BUF_SIZE);

            if (xfer->tx_buf && xfer->len <= SPI_DMA_BUF_SIZE)
                memcpy(hw->dmaTXbuffer,xfer->tx_buf,xfer->len);
            else
               ;// printk("SPI1:empty buf or low size, xfer->tx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->tx_buf,xfer->len);
                
            hw->len= xfer->len;

            if (SPI2_StartTransactions(hw->len, (LPVOID)hw->dmaTXphyAddr, (LPVOID)hw->dmaRXphyAddr) == 1)
            {
                spiRet = 0;
                
                if (xfer->rx_buf && xfer->len <= SPI_DMA_BUF_SIZE)
                    memcpy(xfer->rx_buf,hw->dmaRXbuffer,xfer->len);
                else
                   ;// printk("SPI1:SPI:empty buf or low size,xfer->rx_buf = 0x%08X xfer->len = 0x%08x \n",xfer->rx_buf,xfer->len);
            }
            else
            {
                spiRet = -1;
           //     printk("SPI1: Incorrect Transactions.\r\n");
            }   

        }
    }

    
    msg->status = 0;
    msg->complete(msg->context);
    complete(&hw->done);
    
//    printk("tansfer end spiRet: %d\n",spiRet);
    
    return spiRet;
}

static int ac83xx_spi_probe(struct platform_device *pdev)
{
    struct ac83xx_spi_info *pdata;
    struct ac83xx_spi *hw;
    struct spi_master *master;
    struct resource *res;
    int err=0;
    
    if (pdev->id < 0) {
        dev_err(&pdev->dev,
                "Invalid platform device id-%d\n", pdev->id);
        return -ENODEV;
    }
    master = spi_alloc_master(&pdev->dev,sizeof(struct ac83xx_spi));
    if(master==NULL)
    {
        dev_err(&pdev->dev,"No memory for spi master\n");
        err=-ENOMEM;
        goto err_nomem;
    }
    hw = spi_master_get_devdata(master);
    if(hw ==NULL)
    {
//        printk("hw  is null spi master get devdata fail\n");
        return -EIO;
    }
    memset(hw,0,sizeof(struct ac83xx_spi));

    hw->master = spi_master_get(master);
    hw->pdata = pdata = pdev->dev.platform_data;
    hw->dev = &pdev->dev;

    platform_set_drvdata(pdev,hw);
    init_completion(&hw->done);

    master->num_chipselect = 1;
    master->bus_num = pdev->id;
    master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
    
    master->setup = ac83xx_spi_setup;
    master->transfer = ac83xx_spi_transfer;
    
    
    res = platform_get_resource(pdev,IORESOURCE_MEM,0);
    hw->res = res;
    
    if(res==NULL)
    {
        dev_err(&pdev->dev,"Cannot get IORESOURCE_MEM\n");
        err = -ENOENT;
        goto err_no_iores;
    } 
    spi_init(hw);
    
    //alloc DMA memory here
    spi_dma_init(hw);
    
    err = spi_register_master(master);  
    if(err)
    {
        dev_err(&pdev->dev,"Fail to register SPI master\n");
        goto err_register;
    }
//    printk("register SPI master ok");
    return 0;

err_register:
    if(hw->set_cs==ac83xx_spi_gpiocs)
        gpio_free(pdata->pin_cs);
    clk_disable(hw->clk);
    clk_put(hw->clk);
err_no_iores:

err_nomem:
    return err;
}
static int ac83xx_spi_remove(struct platform_device *dev)
{
    struct ac83xx_spi *hw = platform_get_drvdata(dev);
    platform_set_drvdata(dev, NULL);
    if(hw !=NULL)
    {
        spi_unregister_master(hw->master);
        clk_disable(hw->clk);
        clk_put(hw->clk);
        
        if(hw->set_cs==ac83xx_spi_gpiocs)
            gpio_free(hw->pdata->pin_cs);

        spi_master_put(hw->master);
    }
    
    return 0;
}
static int ac83xx_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
    struct ac83xx_spi *hw = platform_get_drvdata(pdev);
    if(hw !=NULL)
    {
         if(hw->pdata && hw->pdata->gpio_setup)
         hw->pdata->gpio_setup(hw->pdata,0);

         clk_disable(hw->clk);
    }
//    printk("[SPI] suspend\n");
    return 0;
}
static int ac83xx_spi_resume(struct platform_device *pdev)
{
    struct ac83xx_spi *hw = platform_get_drvdata(pdev);
    if(hw !=NULL)
    {
         spi_init(hw);
    }
//    printk("[SPI] resume\n");
    return 0;
}
static struct platform_driver ac83xx_spi_driver = {
    .probe  = ac83xx_spi_probe,
    .remove = ac83xx_spi_remove,
    .suspend= ac83xx_spi_suspend,
    .resume = ac83xx_spi_resume,
    .driver = {
        .name = "ac83xx-spi",
        .owner= THIS_MODULE,
    },
};
static int __init ac83xx_spi_init(void)
{
   int ret;
   MOD_VERSION_INFO(SPI_VER_NAME,SPI_VER_MAIN,SPI_VER_MINOR,SPI_VER_REV);
//    printk("AC83XX SPI: init\n");
    ret = platform_driver_register(&ac83xx_spi_driver);
//    if(ret)
//        printk("spi register failed\n");
//    printk("spi register ok\n");
    return 0;
}

subsys_initcall(ac83xx_spi_init);

static void __exit ac83xx_spi_exit(void)
{
    platform_driver_unregister(&ac83xx_spi_driver);
}

module(ac83xx_spi_exit);

MODULE_AUTHOR("AutoChips");
MODULE_DESCRIPTION("AC83XX SPI Controller Driver");
MODULE_LICENSE("GPL");


