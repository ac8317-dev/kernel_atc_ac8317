/*
 * @file i2c-mt33xx.c
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
#include "i2c-mt33xx.h"


/* timeout waiting for the controller to respond */
#define MT33XX_I2C_TIMEOUT (msecs_to_jiffies(1000))

/******************************************************************************
* Local variable
******************************************************************************/
int  _SifMIsrInitiated = 0;

/*========================================================================*/

struct mt33xx_i2c_dev {
    struct device       *dev;
    int                         irq;
    u32                         speed;		/* Speed of bus in Khz */
    u8                          *buf;
    u8                          *regs;
    size_t		        buf_len;
    struct i2c_adapter	adapter;
};

static int SIFM_TrigMode(u32 u4Mode)
{
    SIFM_SIF_MODE_WRITE(u4Mode);
    SIF_SET_BIT(SIF_SIFMCTL1,SIFM_TRI);

    while(IS_SIF_BIT(SIF_SIFMCTL1,SIFM_TRI));
       // usleep(1);

    return (0);
}

static irqreturn_t  mt33xx_i2c_isr(int this_irq, void *dev_id)
{

    SIF_SET_BIT(SIF_INTCLR, SIFM_INTCLR);

    return IRQ_HANDLED;
}

static int mt33xx_i2c_init(struct mt33xx_i2c_dev *dev)
{

    if (_SifMIsrInitiated == 0)
    {

        SIF_SET_BIT(SIF_SIFMCTL0, SIFM_SM0EN);  //enable sif master
        SIF_SET_BIT(SIF_SIFMCTL0, SIFM_ODRAIN); //output pull-high
        SIF_SET_BIT(SIF_SIFMCTL0, SIFM_SCL_STATE);  //init SCL line value
        SIF_SET_BIT(SIF_SIFMCTL0, SIFM_SDA_STATE);  //init SDA line value

        SIF_CLR_BIT(SIF_PAD_PINMUX3,(SIFM_SD_SEL_IOM + SIFM_SCL_SEL_IOM));
        SIF_SET_BIT(SIF_PAD_PINMUX2,(SIFM_SD_SEL_IOM + SIFM_SCL_SEL_IOM));

        // disable IMR, clear ISR
        SIF_CLR_BIT(SIF_INTEN, SIFM_INTEN);
        SIF_SET_BIT(SIF_INTCLR, SIFM_INTCLR);

        //set cloclk speed,default is 400k
        SIFM_CLK_DIV_WRITE(68);

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
static int mt33xx_i2c_wait_for_bb(struct mt33xx_i2c_dev *dev)
{
    unsigned long timeout;

    timeout = jiffies + MT33XX_I2C_TIMEOUT;
    while (SIF_READ32(SIF_SIFMCTL1) & SIFM_BUSY) {
        if (time_after(jiffies, timeout)) {
            printk( "[I2C]timeout waiting for bus ready\n");
            return -ETIMEDOUT;
        }
        msleep(1);
    }

    return 0;
}

static int SifMRead(u32 ucDev,  u8 *pucValue, u32 u4Count, u32 NoRDAck)
{
    u32 u4Ack, ucReadCount, ucIdx, ucAckCount, ucAckFinal, ucTmpCount;

    if ((pucValue == NULL) ||  	(u4Count == 0))
    {
    	return -EIO;
    }

    ucIdx = 0; 

    SIFM_DATA0_WRITE(((ucDev<<1) + 1));
    SIFM_PGLEN_WRITE(0x00);
    SIFM_TrigMode(SIFM_WRITE_DATA);
    u4Ack = SIFM_ACK_READ();
    if(u4Ack != 0x1)
    {
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

    return 0;
}

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

    return 0;
}



/*
 * Low level master read/write transaction.
 */
static int mt33xx_i2c_xfer_msg(struct i2c_adapter *adap,
			     struct i2c_msg *msg, int stop)
{
    struct mt33xx_i2c_dev *dev = i2c_get_adapdata(adap);
    u32 NoStart,NoRdAck;
    

    //printk( "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n", msg->addr, msg->len, msg->flags, stop);

    if (msg->len == 0)
        return -EINVAL;

    dev->buf = msg->buf;
    dev->buf_len = msg->len;
    NoStart = ((msg->flags) & I2C_M_NOSTART);
    NoRdAck=((msg->flags) & I2C_M_NO_RD_ACK);

/***********************************************************************/
//start bit
    if(0 == NoStart)
        SIFM_TrigMode(SIFM_START);

    if((msg->flags)&I2C_M_RD)
        SifMRead(msg->addr, dev->buf, dev->buf_len,NoRdAck);
    else
        SifMWrite(msg->addr,  dev->buf, dev->buf_len);

//stop bit
    if(stop)
        SIFM_TrigMode(SIFM_STOP);
    return 0;
}


/*
 * Prepare controller for a transaction and call mt33xx_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int mt33xx_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct mt33xx_i2c_dev *dev = i2c_get_adapdata(adap);
    int i;
    int r;

    r = mt33xx_i2c_wait_for_bb(dev);
    if (r < 0)
    	goto out;

    for (i = 0; i < num; i++) {
        r = mt33xx_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
        if (r != 0)
            break;
    }

    if (r == 0)
        r = num;
    r = mt33xx_i2c_wait_for_bb(dev);

out:
    return r;
}

static u32 mt33xx_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;
}



static const struct i2c_algorithm mt33xx_i2c_algo = {
	.master_xfer	= mt33xx_i2c_xfer,
	.functionality	= mt33xx_i2c_func,
};

static int __devinit
mt33xx_i2c_probe(struct platform_device *pdev)
{
    struct mt33xx_i2c_dev *dev;
    struct i2c_adapter  *adap;
    //struct resource     *irq;
    //struct mt33xx_i2c_bus_platform_data *pdata = pdev->dev.platform_data;
    irq_handler_t isr;
    int r;
    printk("mt33xx_i2c_probe\n");
#if 0
    /* NOTE: driver uses the static register mapping */
    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!irq) {
        printk("no irq resource?\n");
        return -ENODEV;
    }
#endif
    dev = kzalloc(sizeof(struct mt33xx_i2c_dev), GFP_KERNEL);
    if (!dev) {
        r = -ENOMEM;
        goto err_free_mem;
    }

    dev->speed = 400;   //400kHz
    dev->dev = &pdev->dev;
    //dev->irq = irq->start;

    platform_set_drvdata(pdev, dev);

    //pm_runtime_enable(&pdev->dev);

    mt33xx_i2c_init(dev);
#if 0
    isr = mt33xx_i2c_isr ;
    r = request_irq(dev->irq, isr, 0, pdev->name, dev);

    if (r) {
        printk( "failure requesting irq %i\n", dev->irq);
        goto err_free_mem;
    }
#endif
    adap = &dev->adapter;
    i2c_set_adapdata(adap, dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON;
    strlcpy(adap->name, "MT33XX I2C adapter", sizeof(adap->name));
    adap->algo = &mt33xx_i2c_algo;
    adap->dev.parent = &pdev->dev;

    /* i2c device drivers may be active on return from add_adapter() */
    adap->nr = pdev->id;
    r = i2c_add_numbered_adapter(adap);
    if (r) {
        printk("failure adding adapter\n");
        goto err_free_irq;
    }
    printk("mt33xx_i2c probe sucess\n");

    return 0;

err_free_irq:
//    free_irq(dev->irq, dev);
err_free_mem:
    platform_set_drvdata(pdev, NULL);
    kfree(dev);

    return r;
}



static int
mt33xx_i2c_remove(struct platform_device *pdev)
{
    struct mt33xx_i2c_dev *i2c = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);
    i2c_del_adapter(&i2c->adapter);
    free_irq(i2c->irq, i2c);

    kfree(i2c);
    
    return 0;
}

#if 0//def CONFIG_SUSPEND
static int mt33xx_i2c_suspend(struct device *dev)
{

	return 0;
}

static int mt33xx_i2c_resume(struct device *dev)
{

	return 0;
}

static struct dev_pm_ops mt33xx_i2c_pm_ops = {
	.suspend = mt33xx_i2c_suspend,
	.resume = mt33xx_i2c_resume,
};
#define mt33xx_I2C_PM_OPS (&mt33xx_i2c_pm_ops)
#else
#define mt33xx_I2C_PM_OPS NULL
#endif

static struct platform_driver mt33xx_i2c_driver = {
	.probe		= mt33xx_i2c_probe,
	.remove		= mt33xx_i2c_remove,
	.driver		= {
		.name	= "mt33xx_i2c",
		.owner	= THIS_MODULE,
		.pm	= mt33xx_I2C_PM_OPS,
	},
};

/* I2C may be needed to bring up other drivers */
static int __init
mt33xx_i2c_init_driver(void)
{
    int ret;

    ret = platform_driver_register(&mt33xx_i2c_driver);
    if (ret)
        printk(KERN_ERR "i2c-mtk: probe failed: %d\n", ret);

    return ret;
}
subsys_initcall(mt33xx_i2c_init_driver);

static void __exit mt33xx_i2c_exit_driver(void)
{
	platform_driver_unregister(&mt33xx_i2c_driver);
}
module_exit(mt33xx_i2c_exit_driver);

MODULE_AUTHOR("Mediatek");
MODULE_DESCRIPTION("MTK mt33xx I2C bus adapter");
MODULE_LICENSE("GPL");
