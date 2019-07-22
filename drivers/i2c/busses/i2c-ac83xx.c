#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include "i2c-ac83xx.h"
#include <mach/ac83xx_gpio_pinmux.h>
#include <mach/ac83xx_pinmux_table.h>
#include <mach/pinmux.h>
#include <linux/gpio.h>
#include "x_ver.h"

#ifdef I2C_DEBUG 
#define I2C_DBG(format ...) printk(format ##_VA_ARGS_)
#else
#define I2C_DBG(format ...)
#endif
//version info
#define I2C_VER_NAME    "I2C"
#define I2C_VER_MAIN     01
#define I2C_VER_MINOR  00
#define I2C_VER_REV       00

/* timeout waiting for the controller to respond */
#define AC83XX_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define  CONFIG_SUSPEND    ((unsigned int)1)
/******************************************************************************
* Local variable
******************************************************************************/
int  _SifMIsrInitiated = 0;

/*========================================================================*/

struct ac83xx_i2c_dev {
    struct device       *dev;
    int                  irq;
    u32                  speed;/* Speed of bus in Khz */
    u32                  suspended;
    u8                   *buf;
    u8                   *regs;
    size_t               buf_len;
    struct i2c_adapter   adapter;
};


static int SIFM_TrigMode(u32 u4Mode)
{
    SIFM_SIF_MODE_WRITE(u4Mode);
    SIF_SET_BIT(SIF_SIFM0CTL1,SIFM_TRI);
    while(IS_SIF_BIT(SIF_SIFM0CTL1,SIFM_TRI));
       // usleep(1);

    return (0);
}
static int SIFM1_TrigMode(u32 u4Mode)
{
    SIFM1_SIF_MODE_WRITE(u4Mode);
    SIF_SET_BIT(SIF_SIFM1CTL1,SIFM_TRI);
    while(IS_SIF_BIT(SIF_SIFM1CTL1,SIFM_TRI));
       // usleep(1);

    return (0);
}

/////////

static int ac83xx_i2c_init(struct ac83xx_i2c_dev *dev)
{
    u32 u4Tmp = 0;
    printk(KERN_ERR " ac83xx_i2c_init\n");
    if (_SifMIsrInitiated == 0)
    {
        /* select master0 & master1*/
         u4Tmp = SIF_IO_READ32(SIF_SEL);            
        // u4Tmp |= SIF_SEL_M0M1;         
         SIF_IO_WRITE32(SIF_SEL,u4Tmp); 
         
        /* master0 set pinmux and clock*/      
         gpio_request(PIN_112_SCL0,"PIN_112_SCL0");
         GPIO_MultiFun_Set(PIN_112_SCL0, I2C0_SEL);
         gpio_request(PIN_117_SDA0,"PIN_117_SDA0");
         GPIO_MultiFun_Set(PIN_117_SDA0, I2C0_SEL);
                                
         u4Tmp = SIF_IO_READ32(SIF_CLOCK);                  
         u4Tmp |= SIFM0_CLOCK;          
         SIF_IO_WRITE32(SIF_CLOCK,u4Tmp);       
                                    
         u4Tmp = SIF_IO_READ32(SIF_RESET);                  
         u4Tmp |= SIFM0_RESET;          
         SIF_IO_WRITE32(SIF_RESET,u4Tmp);
         
        /* master1 set pinmux and clock*/   
        // gpio_request(PIN_113_SCL1,"PIN_113_SCL1");
        // GPIO_MultiFun_Set(PIN_113_SCL1, I2C1_SEL);
        // gpio_request(PIN_118_SDA1,"PIN_118_SDA1");
        // GPIO_MultiFun_Set(PIN_118_SDA1, I2C1_SEL);

         u4Tmp = SIF_IO_READ32(SIF_CLOCK);                  
        // u4Tmp |= SIFM1_CLOCK;          
         SIF_IO_WRITE32(SIF_CLOCK,u4Tmp);       
                                    
         u4Tmp = SIF_IO_READ32(SIF_RESET);                  
       //  u4Tmp |= SIFM1_RESET;          
         SIF_IO_WRITE32(SIF_RESET,u4Tmp);
       
         SIF_SET_BIT(SIF_SIFM0CTL0, SIFM_SM0EN);      //enable sif master0
         SIF_SET_BIT(SIF_SIFM0CTL0, SIFM_ODRAIN);     //output pull-high
         SIF_SET_BIT(SIF_SIFM0CTL0, SIFM_SCL_STATE);  //init SCL line value
         SIF_SET_BIT(SIF_SIFM0CTL0, SIFM_SDA_STATE);  //init SDA line value
 
       //  SIF_SET_BIT(SIF_SIFM1CTL0, SIFM_SM0EN);      //enable sif master1
       //  SIF_SET_BIT(SIF_SIFM1CTL0, SIFM_ODRAIN);     //output pull-high
       //  SIF_SET_BIT(SIF_SIFM1CTL0, SIFM_SCL_STATE);  //init SCL line value
       //  SIF_SET_BIT(SIF_SIFM1CTL0, SIFM_SDA_STATE);  //init SDA line value
         
         SIF_CLR_BIT(SIFM_INTEN, 1);
         SIF_SET_BIT(SIFM_INTCLR, 1);

       //  SIF_CLR_BIT(SIFM1_INTEN, 1);
       //  SIF_SET_BIT(SIFM1_INTCLR, 1);
         
         //set cloclk speed,default is 400k
//         SIFM_CLK_DIV_WRITE(68);---commented by others
		 SIFM_CLK_DIV_WRITE(540);

        // SIFM1_CLK_DIV_WRITE(900);
         // init local variable
         _SifMIsrInitiated = 1;

         // enable gloabl ISR
         //SIF_SET_BIT(SIF_INTEN, SIFM_INTEN);

    }

    return (0);
}

/*
 * Waiting on Bus Busy
 */
static int ac83xx_i2c_wait_for_bb(struct ac83xx_i2c_dev *dev)
{
    unsigned long timeout;
    timeout = jiffies + AC83XX_I2C_TIMEOUT;
    if( dev->adapter.nr == 0)
    {
       while (SIF_READ32(SIF_SIFM0CTL1) & SIFM_BUSY)
       {
            if (time_after(jiffies, timeout))
            {
                printk( "[I2C]timeout waiting for bus ready\n");
                return -ETIMEDOUT;
            }
            msleep(1);
       } 
    }
    if( dev->adapter.nr == 1)
    {
       while (SIF_READ32(SIF_SIFM1CTL1) & SIFM_BUSY)
       {
            if (time_after(jiffies, timeout))
            {
                printk( "[I2C]timeout waiting for bus ready\n");
                return -ETIMEDOUT;
            }
            msleep(1);
       } 
    }
    return 0;
}

static int SifMRead(u32 ucDev,  u8 *pucValue, u32 u4Count, u32 NoRDAck)
{
    u32 u4Ack, ucReadCount, ucIdx, ucAckCount, ucAckFinal, ucTmpCount;
    if ((pucValue == NULL) ||   (u4Count == 0))
    {
        printk(KERN_INFO "Data is not right\n");
        return -EIO;
    }

    ucIdx = 0; 

    SIFM_DATA0_WRITE(((ucDev<<1) + 1));
    SIFM_PGLEN_WRITE(0x00);
    SIFM_TrigMode(SIFM_WRITE_DATA);
    u4Ack = SIFM_ACK_READ();
    if(u4Ack != 0x1)
    {
        printk(KERN_INFO "MASTER0 READ ACK FAILURE\n");
        return -EIO;
    }

    ucAckCount = (u4Count-1)/8;
    ucAckFinal = 0;
    while (u4Count > 0)
    {

        if(ucAckCount > 0)
        {
            ucReadCount = 8;
            ucAckFinal = 0;
            ucAckCount --;
        }
        else
        {
            ucReadCount = u4Count;
            ucAckFinal = 1;
        }

        SIFM_PGLEN_WRITE((ucReadCount - 1));
        if(NoRDAck)
            SIFM_TrigMode(SIFM_READ_DATA_NO_ACK);
        else
        {
            SIFM_TrigMode((ucAckFinal == 1)? SIFM_READ_DATA_NO_ACK: SIFM_READ_DATA_ACK);

            u4Ack = SIFM_ACK_READ();
            for(ucTmpCount = 0; ((u4Ack & (1 << ucTmpCount)) != 0) && (ucTmpCount < 8); ucTmpCount++){}

            if(((ucAckFinal == 1) && ((ucTmpCount) != (ucReadCount-1)))||((ucAckFinal == 0) && (ucTmpCount != ucReadCount)))
            {
                break;
            }
        }
        
        switch(ucReadCount)
        {
            case 8:
                pucValue[ucIdx + 7] = SIFM_DATA7_READ();
            case 7:
                pucValue[ucIdx + 6] = SIFM_DATA6_READ();
            case 6:
                pucValue[ucIdx + 5] = SIFM_DATA5_READ();
            case 5:
                pucValue[ucIdx + 4] = SIFM_DATA4_READ();
            case 4:
                pucValue[ucIdx + 3] = SIFM_DATA3_READ();
            case 3:
                pucValue[ucIdx + 2] = SIFM_DATA2_READ();
            case 2:
                pucValue[ucIdx + 1] = SIFM_DATA1_READ();
            case 1:
                pucValue[ucIdx + 0] = SIFM_DATA0_READ();
            default:
                break;
        }

        u4Count -= ucReadCount;
        ucIdx += ucReadCount;
    }

   // printk(KERN_INFO "MASTER0 READ OKAY\n");
    return 0;
}

static int SifM1Read(u32 ucDev,  u8 *pucValue, u32 u4Count, u32 NoRDAck)
{
    u32 u4Ack, ucReadCount, ucIdx, ucAckCount, ucAckFinal, ucTmpCount;
   
    if ((pucValue == NULL) ||   (u4Count == 0))
    {
        printk(KERN_INFO "Data is not right\n");
        return -EIO;
    }

    ucIdx = 0; 

    SIFM1_DATA0_WRITE(((ucDev<<1) + 1));
    SIFM1_PGLEN_WRITE(0x00);
    SIFM1_TrigMode(SIFM_WRITE_DATA);
    u4Ack = SIFM1_ACK_READ();
    if(u4Ack != 0x1)
    {
        printk(KERN_INFO "MASTER1 READ ACK FAILURE\n");
        return -EIO;
    }

    ucAckCount = (u4Count-1)/8;
    ucAckFinal = 0;
    while (u4Count > 0)
    {

        if(ucAckCount > 0)
        {
            ucReadCount = 8;
            ucAckFinal = 0;
            ucAckCount --;
        }
        else
        {
            ucReadCount = u4Count;
            ucAckFinal = 1;
        }

        SIFM1_PGLEN_WRITE((ucReadCount - 1));
        if(NoRDAck)
            SIFM1_TrigMode(SIFM_READ_DATA_NO_ACK);
        else
        {
            SIFM1_TrigMode((ucAckFinal == 1)? SIFM_READ_DATA_NO_ACK: SIFM_READ_DATA_ACK);

            u4Ack = SIFM1_ACK_READ();
            for(ucTmpCount = 0; ((u4Ack & (1 << ucTmpCount)) != 0) && (ucTmpCount < 8); ucTmpCount++){}

            if(((ucAckFinal == 1) && ((ucTmpCount) != (ucReadCount-1)))||((ucAckFinal == 0) && (ucTmpCount != ucReadCount)))
            {
                break;
            }
        }
        
        switch(ucReadCount)
        {
            case 8:
                pucValue[ucIdx + 7] = SIFM1_DATA7_READ();
            case 7:
                pucValue[ucIdx + 6] = SIFM1_DATA6_READ();
            case 6:
                pucValue[ucIdx + 5] = SIFM1_DATA5_READ();
            case 5:
                pucValue[ucIdx + 4] = SIFM1_DATA4_READ();
            case 4:
                pucValue[ucIdx + 3] = SIFM1_DATA3_READ();
            case 3:
                pucValue[ucIdx + 2] = SIFM1_DATA2_READ();
            case 2:
                pucValue[ucIdx + 1] = SIFM1_DATA1_READ();
            case 1:
                pucValue[ucIdx + 0] = SIFM1_DATA0_READ();
            default:
                break;
        }

        u4Count -= ucReadCount;
        ucIdx += ucReadCount;
    }

   // printk(KERN_INFO "MASTER1 READ OKAY\n");
    return 0;
}

///////////////////////
///////////////////////
static int SifMWrite(u32 ucDev, const u8 *pucValue, u32 u4Count)
{
    u32 u4Ack, ucWriteCount, ucIdx, ucTmpCount;
   
    ucIdx = 0; 

    if ((pucValue == NULL) ||(u4Count == 0))
    {
        return -EIO;
    }

    SIFM_DATA0_WRITE((ucDev<<1));
    SIFM_PGLEN_WRITE(0x00);
    SIFM_TrigMode(SIFM_WRITE_DATA);
    u4Ack = SIFM_ACK_READ();
    if(u4Ack != 0x1)
    {
        printk(KERN_INFO "MASTER0 WRITE ACK FAILURE\n");
        return -EIO;
    }

    while (u4Count > 0)
    {
        ucWriteCount = (u4Count > 8) ? 8 : (u4Count);

        switch(ucWriteCount)
        {
        case 8:
            SIFM_DATA7_WRITE(pucValue[ucIdx + 7]);
        case 7:
            SIFM_DATA6_WRITE(pucValue[ucIdx + 6]);
        case 6:
            SIFM_DATA5_WRITE(pucValue[ucIdx + 5]);
        case 5:
            SIFM_DATA4_WRITE(pucValue[ucIdx + 4]);
        case 4:
            SIFM_DATA3_WRITE(pucValue[ucIdx + 3]);
        case 3:
            SIFM_DATA2_WRITE(pucValue[ucIdx + 2]);
        case 2:
            SIFM_DATA1_WRITE(pucValue[ucIdx + 1]);
        case 1:
            SIFM_DATA0_WRITE(pucValue[ucIdx + 0]);
        default:
            break;
        }

        SIFM_PGLEN_WRITE((ucWriteCount - 1));
        SIFM_TrigMode(SIFM_WRITE_DATA);

        u4Ack = SIFM_ACK_READ();
        for(ucTmpCount = 0; ((u4Ack & (1 << ucTmpCount)) != 0) && (ucTmpCount < 8); ucTmpCount++){}
        if(ucTmpCount != ucWriteCount)
        {
            break;
        }

        u4Count -= ucWriteCount;
        ucIdx += ucWriteCount;
    }

    I2C_DBG(KERN_INFO "MASTER0 WRITE OKAY\n");
    return 0;
}

static int SifM1Write(u32 ucDev, const u8 *pucValue, u32 u4Count)
{
    u32 u4Ack, ucWriteCount, ucIdx, ucTmpCount;
   
    ucIdx = 0; 

    if ((pucValue == NULL) ||(u4Count == 0))
    {
        return -EIO;
    }

    SIFM1_DATA0_WRITE((ucDev<<1));
    SIFM1_PGLEN_WRITE(0x00);
    SIFM1_TrigMode(SIFM_WRITE_DATA);
    u4Ack = SIFM1_ACK_READ();
    if(u4Ack != 0x1)
    {
        printk(KERN_INFO "MASTER1 WRITE ACK FAILURE\n");
        return -EIO;
    }

    while (u4Count > 0)
    {
        ucWriteCount = (u4Count > 8) ? 8 : (u4Count);

        switch(ucWriteCount)
        {
        case 8:
            SIFM1_DATA7_WRITE(pucValue[ucIdx + 7]);
        case 7:
            SIFM1_DATA6_WRITE(pucValue[ucIdx + 6]);
        case 6:
            SIFM1_DATA5_WRITE(pucValue[ucIdx + 5]);
        case 5:
            SIFM1_DATA4_WRITE(pucValue[ucIdx + 4]);
        case 4:
            SIFM1_DATA3_WRITE(pucValue[ucIdx + 3]);
        case 3:
            SIFM1_DATA2_WRITE(pucValue[ucIdx + 2]);
        case 2:
            SIFM1_DATA1_WRITE(pucValue[ucIdx + 1]);
        case 1:
            SIFM1_DATA0_WRITE(pucValue[ucIdx + 0]);
        default:
            break;
        }

        SIFM1_PGLEN_WRITE((ucWriteCount - 1));
        SIFM1_TrigMode(SIFM_WRITE_DATA);

        u4Ack = SIFM1_ACK_READ();
        for(ucTmpCount = 0; ((u4Ack & (1 << ucTmpCount)) != 0) && (ucTmpCount < 8); ucTmpCount++){}
        if(ucTmpCount != ucWriteCount)
        {
            break;
        }

        u4Count -= ucWriteCount;
        ucIdx += ucWriteCount;
    }

    printk(KERN_INFO "MASTER1 WRITE OKAY\n");
    return 0;
}

//////////////////////////////////
//////////////////////////////////
/*
 * Low level master read/write transaction.
 */
static int ac83xx_i2c_xfer_msg(struct i2c_adapter *adap,
                 struct i2c_msg *msg, int stop)
{
    struct ac83xx_i2c_dev *dev = i2c_get_adapdata(adap);
    u32 NoStart,NoRdAck;
    int ret = -1;
    

    I2C_DBG( "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n", msg->addr, msg->len, msg->flags, stop);
    
    if (msg->len == 0)
        return -EINVAL;
    if(dev == NULL)
    {
        printk("i2c_dev is null, get_adapdate fail\n");
        return -EIO;
    }
    if(dev->suspended)
    return -EIO;

    dev->buf = msg->buf;
    dev->buf_len = msg->len;
    NoStart = ((msg->flags) & I2C_M_NOSTART);
    NoRdAck=((msg->flags) & I2C_M_NO_RD_ACK);

/***********************************************************************/
    if(adap->nr == 0)
    {
       
       //start bit
        if(0 == NoStart)
           SIFM_TrigMode(SIFM_START);
   
        if((msg->flags)&I2C_M_RD)
           ret = SifMRead(msg->addr, dev->buf, dev->buf_len,NoRdAck);
        else
           ret = SifMWrite(msg->addr,  dev->buf, dev->buf_len);
   
        //stop bit
        if(stop)
           SIFM_TrigMode(SIFM_STOP);
        if (!ret)
            return msg->len;
        else 
            return ret; 
    }
    if(adap->nr == 1)
    {
         //start bit
        if(0 == NoStart)
           SIFM1_TrigMode(SIFM_START);
   
        if((msg->flags)&I2C_M_RD)
           ret = SifM1Read(msg->addr, dev->buf, dev->buf_len,NoRdAck);
        else
           ret = SifM1Write(msg->addr,  dev->buf, dev->buf_len);
   
        //stop bit
        if(stop)
           SIFM1_TrigMode(SIFM_STOP);
        if (!ret)
            return msg->len;
        else 
            return ret; 
    }
      
}


/*
 * Prepare controller for a transaction and call ac83xx_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int ac83xx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct ac83xx_i2c_dev *dev = i2c_get_adapdata(adap);
    int i;
    int r;
    int count = 0;
    if(dev == NULL)
    {
         printk("dev is null, i2c_get_adapdata fail\n");
         return -EIO;
    }
    r = ac83xx_i2c_wait_for_bb(dev);
    if (r < 0)
        goto out;

    for (i = 0; i < num; i++)
    {
        r = ac83xx_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
        if (r < 0)
        return -EIO;
        count+= r;
    }

    r = ac83xx_i2c_wait_for_bb(dev);
    if (r == 0)
        r = num;
out:
    return r;
}

static u32 ac83xx_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}



static const struct i2c_algorithm ac83xx_i2c_algo = {
    .master_xfer    = ac83xx_i2c_xfer,
    .functionality  = ac83xx_i2c_func,
};

static int __devinit
ac83xx_i2c_probe(struct platform_device *pdev)
{
    struct ac83xx_i2c_dev *dev;
    struct i2c_adapter  *adap;
    int r;
    printk(KERN_ERR "ac83xx_i2c_probe\n");

    dev = kzalloc(sizeof(struct ac83xx_i2c_dev), GFP_KERNEL);
    if (!dev) {
        r = -ENOMEM;
        goto err_free_mem;
    }

    dev->speed = 50;   //400kHz
    dev->dev = &pdev->dev;
    dev->suspended = 0;

    platform_set_drvdata(pdev, dev);
    ac83xx_i2c_init(dev);

    adap = &dev->adapter;
    i2c_set_adapdata(adap, dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON;
    strlcpy(adap->name, "ac83xx I2C adapter", sizeof(adap->name));
    adap->algo = &ac83xx_i2c_algo;
    adap->dev.parent = &pdev->dev;

    /* i2c device drivers may be active on return from add_adapter() */
    adap->nr = pdev->id;
    r = i2c_add_numbered_adapter(adap);
    if (r) {
        printk("failure adding adapter\n");
        goto err_free_irq;
    }
    printk("ac83xx_i2c probe sucess\n");

    return 0;

err_free_irq:
//    free_irq(dev->irq, dev);
err_free_mem:
    platform_set_drvdata(pdev, NULL);
    kfree(dev);

    return r;
}



static int
ac83xx_i2c_remove(struct platform_device *pdev)
{
    struct ac83xx_i2c_dev *i2c = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);
    i2c_del_adapter(&i2c->adapter);
    //free_irq(i2c->irq, i2c);
    kfree(i2c);
    
    return 0;
}

#ifdef CONFIG_SUSPEND
static int ac83xx_i2c_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev); 
    struct ac83xx_i2c_dev *i2c = platform_get_drvdata(pdev); 
    if(i2c == NULL)
    {
        printk("i2c_dev is null, get_drvdate fail\n");
        return -EIO;
    }
    i2c->suspended = 1;
        _SifMIsrInitiated = 0;
    printk("[I2C] suspend\n");
    return 0;
}

static int ac83xx_i2c_resume(struct device *dev)
{

    struct platform_device *pdev = to_platform_device(dev); 
    struct ac83xx_i2c_dev  *i2c = platform_get_drvdata(pdev);   
    if(i2c == NULL)
    {
        printk("i2c_dev is null, get_drvdate fail\n");
        return -EIO;
    }
    i2c->suspended = 0;  
    ac83xx_i2c_init(i2c);   
    printk("[I2C] resume\n");
    return 0;
}

static struct dev_pm_ops ac83xx_i2c_pm_ops = {
    .suspend = ac83xx_i2c_suspend,
    .resume = ac83xx_i2c_resume,
};
#define ac83xx_I2C_PM_OPS (&ac83xx_i2c_pm_ops)
#else
#define ac83xx_I2C_PM_OPS NULL
#endif

static struct platform_driver ac83xx_i2c_driver = {
    .probe      = ac83xx_i2c_probe,
    .remove     = ac83xx_i2c_remove,
    .driver     = {
        .name   = "ac83xx_i2c",
        .owner  = THIS_MODULE,
        .pm = ac83xx_I2C_PM_OPS,
    },
};

/* I2C may be needed to bring up other drivers */
static int __init
ac83xx_i2c_init_driver(void)
{
    int ret;
    MOD_VERSION_INFO(I2C_VER_NAME,I2C_VER_MAIN,I2C_VER_MINOR,I2C_VER_REV);
    printk(KERN_ERR "i2c-ac83xx: probe  \n");
    ret = platform_driver_register(&ac83xx_i2c_driver);
    if (ret)
        printk(KERN_ERR "i2c-ac83xx: probe failed: %d\n", ret);

    printk(KERN_ERR "i2c-ac83xx: probe OKAY \n");
    return ret;
}
subsys_initcall(ac83xx_i2c_init_driver);

static void __exit ac83xx_i2c_exit_driver(void)
{
    platform_driver_unregister(&ac83xx_i2c_driver);
}
module_exit(ac83xx_i2c_exit_driver);

MODULE_AUTHOR("Mediatek");
MODULE_DESCRIPTION("ATC ac83xx I2C bus adapter");
MODULE_LICENSE("GPL");
