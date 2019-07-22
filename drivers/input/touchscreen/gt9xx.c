/* drivers/input/touchscreen/gt9xx.c
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Linux Foundation chooses to take subject only to the GPLv2 license
 * terms, and distributes only under these terms.
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.8
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/04/25
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F.
 *                  By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          2. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 */

//#include <linux/regulator/consumer.h>
#include "gt9xx.h"

//#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/debugfs.h>
#include <mach/ac83xx_gpio_pinmux.h>
#include <mach/ac83xx_pinmux_table.h>
#include <mach/ac83xx.h>
#include <mach/ac83xx_basic.h>

#define GOODIX_I2C_NAME "ac83xx_tsadcc"
#define GOODIX_DEV_NAME "ac83xx_tsadcc"
#define CFG_MAX_TOUCH_POINTS      5
#define GOODIX_COORDS_ARR_SIZE  4
#define MAX_BUTTONS                           4

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

#define GOODIX_VTG_MIN_UV   2600000
#define GOODIX_VTG_MAX_UV   3300000
#define GOODIX_I2C_VTG_MIN_UV   1800000
#define GOODIX_I2C_VTG_MAX_UV   1800000
#define GOODIX_VDD_LOAD_MIN_UA  0
#define GOODIX_VDD_LOAD_MAX_UA  10000
#define GOODIX_VIO_LOAD_MIN_UA  0
#define GOODIX_VIO_LOAD_MAX_UA  10000

#define RESET_DELAY_T3_US   200 /* T3: > 100us */
#define RESET_DELAY_T4          20  /* T4: > 5ms */

#define PHY_BUF_SIZE            32
#define PROP_NAME_SIZE      24

#define GTP_MAX_TOUCH       5
#define GTP_ESD_CHECK_CIRCLE_MS 2000

#if GTP_HAVE_TOUCH_KEY
static const u16 touch_key_array[] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};
#define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))

#endif

#ifdef DEBUG
//#define dev_dbg(dev, format, ...)  printk(format, ##__VA_ARGS__)
//#define dev_err(dev, format, ...)  printk(format, ##__VA_ARGS__)
//#define dev_warn(dev, format, ...)  printk(format, ##__VA_ARGS__)
#endif

extern int gpio_get_value(unsigned);
extern void gpio_free(unsigned);
extern int gpio_set_value(unsigned, int);
extern int bsp_pinset(unsigned,unsigned);
extern int gpio_inout_sel(unsigned,int);
extern int gpio_direction_output(unsigned, int);
extern int gpio_direction_input(unsigned);
extern int GPIO_MultiFun_Set(unsigned, unsigned);
extern int gpio_request(unsigned gpio, const char *label);

static void gtp_int_sync(struct goodix_ts_data *ts, int ms);
static int gtp_i2c_test(struct i2c_client *client);

extern void BIM_SetEInt(unsigned int EIntNumber, unsigned int type, unsigned int debunceTime);
extern void BIM_EnableEInt(unsigned int EIntNumber);
extern void BIM_DisableEInt(unsigned int EIntNumber);

#ifdef DEBUG
static int gtp_i2c_config_read(struct i2c_client *client, unsigned int addr);
static void gtp_i2c_config_reads(struct i2c_client *client);
#endif

void gtp_reset_guitar(struct goodix_ts_data *ts, int ms);
static int goodix_power_off(struct goodix_ts_data *ts);
static int goodix_power_on(struct goodix_ts_data *ts);
static int goodix_ts_suspend(struct device *dev);
static int goodix_ts_resume(struct device *dev);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *work);
static int gtp_init_ext_watchdog(struct i2c_client *client);
#endif

#if GTP_SLIDE_WAKEUP
enum doze_status {
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
};
static enum doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif
bool init_done;
static u8 chip_gt9xxs;  /* true if ic is gt9xxs, like gt915s */
u8 grp_cfg_version;
struct i2c_client  *i2c_connect_client;
struct goodix_ts_data *ts_data;
EXPORT_SYMBOL(i2c_connect_client);
#define GTP_DEBUGFS_DIR         "ts_debug"
#define GTP_DEBUGFS_FILE_SUSPEND    "suspend"


#define GT_WRITE(_reg, _val)    (*((volatile uint32_t*)(_reg)) = (_val))
#define GT_READ(_reg)       (*((volatile uint32_t*)(_reg)))

static void set_irq_pin(struct goodix_ts_data *ts, int func)
{
    if(ts->client->irq) {
    } else {
        ts->client->irq = INT_PORT;
    }
    GPIO_MultiFun_Set(INT_GPIO, PINMUX_LEVEL_GPIO_END_FLAG);
}


#ifdef DEBUG
unsigned long simple_strtoul(const char *cp,char **endp,unsigned int base)
{
    unsigned long result = 0,value;

    if (*cp == '0') {
        cp++;
        if ((*cp == 'x') && isxdigit(cp[1])) {    //  isxdigit 的参数是不是 0~9 的数字   
            base = 16;
            cp++;
        }
        if (!base) {
            base = 8;
        }
    }
    if (!base) {
        base = 10;
    }
    while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp-'0' : (islower(*cp)   // islower 判断是不是小写的字母  
        ? toupper(*cp) : *cp)-'A'+10) < base) {    //  toupper  将小写的字母转换成大写的字母  
        result = result*base + value;
        cp++;
    }
    if (endp)
        *endp = (char *)cp;
    return result;
}

long simple_strtol(const char *cp,char **endp,unsigned int base)
{
    if(*cp=='-')
        return -simple_strtoul(cp+1,endp,base);
    return simple_strtoul(cp,endp,base);
}

static inline int str2long(char *p, ulong *num)
{
    char *endptr;

    *num = simple_strtoul(p, &endptr, 16);
    return (*p != '\0' && *endptr == '\0') ? 1 : 0;
}

static ssize_t gtp_read_store(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t n)
{
    unsigned long gtp_reg = 0;
    unsigned long gtp_reg_value = 0;

    str2long((char*)buf, &gtp_reg);

    printk(KERN_ALERT "register= %x\n", (unsigned int)gtp_reg);
    gtp_reg_value = GT_READ(gtp_reg);
    printk(KERN_ERR "GT_READ(reg)=%x\n", (unsigned int)gtp_reg_value);
    
    return (n);
}

static ssize_t gtp_read_show(struct kobject *kobj, struct kobj_attribute *attr,
                                   char *buf)
{
    char *buff = buf;
    return buff - buf;
}

static unsigned long gtp_write_value = 0;
static ssize_t gtp_write_store(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t n)
{
    unsigned long gtp_reg = 0;
    unsigned long gtp_reg_value = 0;

    str2long(buf, &gtp_reg);

    printk(KERN_ALERT "register= %x\n", gtp_reg);
    gtp_reg_value = GT_READ(gtp_reg);
    printk(KERN_ERR "GT_READ(reg)=%x\n", gtp_reg_value);
    printk(KERN_ERR "Write %x to %x\n", gtp_write_value, gtp_reg);
    GT_WRITE(gtp_reg, gtp_write_value);
    gtp_write_value = 0;

    return (n);
}

static ssize_t gtp_write_show(struct kobject *kobj, struct kobj_attribute *attr,
                                   char *buf)
{
    char *buff = buf;
    return buff - buf;
}

static ssize_t gtp_value_store(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t n)
{

    str2long(buf, &gtp_write_value);

    printk(KERN_ERR "Write Value %x\n", gtp_write_value);
    return (n);
}

static ssize_t gtp_value_show(struct kobject *kobj, struct kobj_attribute *attr,
                                   char *buf)
{
    char *buff = buf;
    return buff - buf;
}

static ssize_t gtp_test_store(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t n)
{
    return (n);
}

static ssize_t gtp_test_show(struct kobject *kobj, struct kobj_attribute *attr,
                                   char *buf)
{
    char *buff = buf;
    return buff - buf;
}

#define gt_attr(_name)                              \
static struct kobj_attribute _name##_attr = {       \
    .attr   = {                                     \
        .name = __stringify(_name),                 \
        .mode = 0644,                               \
    },                                              \
    .show   = _name##_show,                         \
    .store  = _name##_store,                        \
}

gt_attr(gtp_read);
gt_attr(gtp_write);
gt_attr(gtp_value);
gt_attr(gtp_test);

static struct attribute * g[] = {
    &gtp_read_attr.attr,
    &gtp_write_attr.attr,
    &gtp_value_attr.attr,
    &gtp_test_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = g,
};

static struct kobject *gtp_kobj = NULL;
#endif

#define GPIO_EN0_OFFSET         0x074 
#define GPIO_OUT0_OFFSET        0x0E0
#define GPIO_IN0_OFFSET         0x100
#define OUTPUT      1

#define GPIO_EN_WRITE(idx,val)          CKGEN_WRITE32((GPIO_EN0_OFFSET+(4*(idx))), (val))
#define GPIO_OUT_WRITE(idx,val)         CKGEN_WRITE32((GPIO_OUT0_OFFSET+(4*(idx))), (val))
#define GPIO_IN_WRITE(idx,val)          CKGEN_WRITE32((GPIO_IN0_OFFSET+(4*(idx))), (val))
#define GPIO_OUT_REG(idx)               CKGEN_REG32(GPIO_OUT0_OFFSET+(4*(idx)))
#define GPIO_EN_REG(idx)                CKGEN_REG32(GPIO_EN0_OFFSET+(4*(idx)))
#define GPIO_IN_REG(idx)                CKGEN_REG32(GPIO_IN0_OFFSET+(4*(idx)))


#define TEST_GPIO   0

void gtp_gpio_set_value(unsigned gpio, int value)
{
#if TEST_GPIO
    unsigned val, idx, offset;
    offset = gpio % 32;
    idx  = gpio / 32;
    val = GT_READ(0xFD0000E0 + (4 * idx));
    val = (value == 1) ? (val | (1U << offset)) : (val & ~(1U << offset));
    GT_WRITE(0xFD0000E0 + (4 * idx), val);
#else
    gpio_set_value(gpio, value);
#endif
}

int gtp_gpio_get_value(unsigned gpio)
{
#if TEST_GPIO
    unsigned val, idx, offset;
    offset = gpio % 32;
    idx  = gpio / 32;

    val = GT_READ(0xFD000100 + (4 * idx));

    return ((val & (1U << offset)) ? 1 : 0);
#else
    return gpio_get_value(gpio);
#endif
}

void gtp_gpio_inout_sel(unsigned gpio, int dir)
{
#if TEST_GPIO
    unsigned val, idx, offset;
    offset = gpio % 32;
    idx = gpio / 32;

    val = GT_READ(0xFD000074 + (4 * idx));    
    val = (dir == OUTPUT) ? (val | (1U << offset)) : (val & ~(1U << offset));
    GT_WRITE(0xFD000074 + (4 * idx), val);
#else
    gpio_inout_sel(gpio, dir);
#endif
}

int gtp_gpio_direction_output(unsigned gpio, int value)
{
#if TEST_GPIO
    gtp_gpio_set_value(gpio, value);
    gtp_gpio_inout_sel(gpio, 1);   //OUTPUT = 1
    return 0;
#else
    return gpio_direction_output(gpio, value);
#endif
}

int gtp_gpio_direction_input(unsigned gpio)
{
#if TEST_GPIO
    gtp_gpio_inout_sel(gpio, 0);   //INPUT = 1
    return gtp_gpio_get_value(gpio);
#else
    return gpio_direction_input(gpio);
#endif
}

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
        2: succeed, otherwise: failed
*********************************************************/
int gtp_i2c_read(struct i2c_client *client, u8 *buf, int len)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    int ret = -EIO;
    u8 retries;
    if(ts == NULL)
    {
         printk("[gtp_i2c_read]ts is null, goodix get clientdata fail\n");
         return -EIO;
    }
    struct i2c_msg msgs[2] = {
        {
            .flags  = !I2C_M_RD,
            .addr   = client->addr,
            .len    = GTP_ADDR_LENGTH,
            .buf    = &buf[0],
        },
        {
            .flags  = I2C_M_RD,
            .addr   = client->addr,
            .len    = len - GTP_ADDR_LENGTH,
            .buf    = &buf[GTP_ADDR_LENGTH],
        },
    };

    for (retries = 0; retries < GTP_I2C_RETRY_5; retries++) {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret == 2)
            break;
        dev_err(&client->dev, "I2C retry: %d\n", retries + 1);
    }
    if (retries == GTP_I2C_RETRY_5) {
#if GTP_SLIDE_WAKEUP
        /* reset chip would quit doze mode */
        if (DOZE_ENABLED == doze_status)
            return ret;
#endif
        if (init_done)
            gtp_reset_guitar(ts, 10);
        else
            dev_warn(&client->dev,
                "gtp_reset_guitar exit init_done=%d:\n",
                init_done);
    }
    return ret;
}

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
    1: succeed, otherwise: failed
*********************************************************/
int gtp_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    int ret = -EIO;
    u8 retries;
    if(ts == NULL)
    {
         printk("[gtp_i2c_write]ts is null, goodix get clientdata fail\n");
         return -EIO;
    }
    struct i2c_msg msg = {
        .flags = !I2C_M_RD,
        .addr = client->addr,
        .len = len,
        .buf = buf,
    };

    for (retries = 0; retries < GTP_I2C_RETRY_5; retries++) {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)
            break;
        dev_err(&client->dev, "I2C retry: %d\n", retries + 1);
    }
    if ((retries == GTP_I2C_RETRY_5)) {
#if GTP_SLIDE_WAKEUP
        if (DOZE_ENABLED == doze_status)
            return ret;
#endif
        if (init_done)
            gtp_reset_guitar(ts, 10);
        else
            dev_warn(&client->dev,
                "gtp_reset_guitar exit init_done=%d:\n",
                init_done);
    }
    return ret;
}

/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
int gtp_i2c_read_dbl_check(struct i2c_client *client,
            u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    while (retry++ < GTP_I2C_RETRY_3) {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len + 2))
            break;
    }
    if (retry < GTP_I2C_RETRY_3) {
        memcpy(rxbuf, confirm_buf + 2, len);
        return SUCCESS;
    } else {
        dev_err(&client->dev,
            "i2c read 0x%04X, %d bytes, double check failed!",
            addr, len);
        return FAIL;
    }
}
/*******************************************************
Function:
    Send config data.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
    > 0: succeed, otherwise: failed
*********************************************************/
int gtp_send_cfg(struct goodix_ts_data *ts)
{
    int ret;
#if GTP_DRIVER_SEND_CFG
    int retry = 0;

    if (ts->fixed_cfg) {
        dev_dbg(&ts->client->dev,
            "Ic fixed config, no config sent!");
        ret = 2;
    } else {
        for (retry = 0; retry < GTP_I2C_RETRY_5; retry++) {
            ret = gtp_i2c_write(ts->client,
                ts->config_data,
                GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
            if (ret > 0)
                break;
        }
    }
#endif

    return ret;
}
/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disabled) {
        ts->irq_is_disabled = true;
        disable_irq_nosync(ts->client->irq);
                BIM_DisableEInt(EINT_NUM);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disabled) {
        enable_irq(ts->client->irq);
        ts->irq_is_disabled = false;
                BIM_EnableEInt(EINT_NUM);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data *ts, int id, int x, int y,
        int w)
{
#if GTP_CHANGE_X2Y
    swap(x, y);
#endif
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts, int id)
{
    input_mt_slot(ts->input_dev, id);
    input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
}



/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
void ac83xx_mask_ack_bim_irq(uint32_t irq);
static void goodix_ts_work_func(struct work_struct *work)
{
    u8 end_cmd[3] = { GTP_READ_COOR_ADDR >> 8,
            GTP_READ_COOR_ADDR & 0xFF, 0};
    u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {
            GTP_READ_COOR_ADDR >> 8,
            GTP_READ_COOR_ADDR & 0xFF};
    u8 touch_num = 0;
    u8 finger = 0;
    static u16 pre_touch;
    static u8 pre_key;
#if GTP_WITH_PEN
    static u8 pre_pen;
#endif
    u8 key_value = 0;
    u8 *coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i = 0;
    int ret = -1;
    struct goodix_ts_data *ts = NULL;

#if GTP_SLIDE_WAKEUP
    u8 doze_buf[3] = {0x81, 0x4B};
#endif
    ts = container_of(work, struct goodix_ts_data, work);
#ifdef CONFIG_GT9XX_TOUCHPANEL_UPDATE
    if (ts->enter_update)
        return;
#endif

#if GTP_SLIDE_WAKEUP
    if (DOZE_ENABLED == doze_status) {
        ret = gtp_i2c_read(ts->client, doze_buf, 3);
        if (ret > 0) {
            if (doze_buf[2] == 0xAA) {
                dev_dbg(&ts->client->dev,
                    "Slide(0xAA) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(
                    ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(
                    ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                /* clear 0x814B */
                doze_buf[2] = 0x00;
                gtp_i2c_write(ts->client, doze_buf, 3);
            } else if (doze_buf[2] == 0xBB) {
                dev_dbg(&ts->client->dev,
                    "Slide(0xBB) To Light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                /* clear 0x814B*/
                doze_buf[2] = 0x00;
                gtp_i2c_write(ts->client, doze_buf, 3);
            } else if (0xC0 == (doze_buf[2] & 0xC0)) {
                dev_dbg(&ts->client->dev,
                    "double click to light up the screen!");
                doze_status = DOZE_WAKEUP;
                input_report_key(ts->input_dev, KEY_POWER, 1);
                input_sync(ts->input_dev);
                input_report_key(ts->input_dev, KEY_POWER, 0);
                input_sync(ts->input_dev);
                /* clear 0x814B */
                doze_buf[2] = 0x00;
                gtp_i2c_write(ts->client, doze_buf, 3);
            } else {
                gtp_enter_doze(ts);
            }
        }
        if (ts->use_irq)
            gtp_irq_enable(ts);

        return;
    }
#endif
    ret = gtp_i2c_read(ts->client, point_data, 12);
    if (ret < 0) {
        dev_err(&ts->client->dev,
                "I2C transfer error. errno:%d\n ", ret);
        goto exit_work_func;
    }
    finger = point_data[GTP_ADDR_LENGTH];
        if (finger == 0x00) {
            if (ts->use_irq)
        {
                     gtp_irq_enable(ts);
                }
                return;
         }

    if ((finger & 0x80) == 0)
        goto exit_work_func;

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
        goto exit_work_func;

    if (touch_num > 1) {
        u8 buf[8 * GTP_MAX_TOUCH] = { (GTP_READ_COOR_ADDR + 10) >> 8,
                (GTP_READ_COOR_ADDR + 10) & 0xff };

        ret = gtp_i2c_read(ts->client, buf,
                2 + 8 * (touch_num - 1));
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];

    if (key_value || pre_key) {
        for (i = 0; i < ts->pdata->num_button; i++) {
            input_report_key(ts->input_dev,
                ts->pdata->button_map[i],
                key_value & (0x01<<i));
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
    pre_key = key_value;

#if GTP_WITH_PEN
    if (pre_pen && (touch_num == 0)) {
        dev_dbg(&ts->client->dev, "Pen touch UP(Slot)!");
        input_report_key(ts->input_dev, BTN_TOOL_PEN, 0);
        input_mt_slot(ts->input_dev, 5);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
        pre_pen = 0;
    }
#endif
    if (pre_touch || touch_num) {
        s32 pos = 0;
        u16 touch_index = 0;

        coor_data = &point_data[3];
        if (touch_num) {
            id = coor_data[pos] & 0x0F;
#if GTP_WITH_PEN
            id = coor_data[pos];
            if (id == 128) {
                dev_dbg(&ts->client->dev,
                        "Pen touch DOWN(Slot)!");
                input_x  = coor_data[pos + 1]
                    | (coor_data[pos + 2] << 8);
                input_y  = coor_data[pos + 3]
                    | (coor_data[pos + 4] << 8);
                input_w  = coor_data[pos + 5]
                    | (coor_data[pos + 6] << 8);

                input_report_key(ts->input_dev,
                    BTN_TOOL_PEN, 1);
                input_mt_slot(ts->input_dev, 5);
                input_report_abs(ts->input_dev,
                    ABS_MT_TRACKING_ID, 5);
                input_report_abs(ts->input_dev,
                    ABS_MT_POSITION_X, input_x);
                input_report_abs(ts->input_dev,
                    ABS_MT_POSITION_Y, input_y);
                input_report_abs(ts->input_dev,
                    ABS_MT_TOUCH_MAJOR, input_w);
                dev_dbg(&ts->client->dev,
                    "Pen/Stylus: (%d, %d)[%d]",
                    input_x, input_y, input_w);
                pre_pen = 1;
                pre_touch = 0;
            }
#endif

            touch_index |= (0x01<<id);
        }

        for (i = 0; i < GTP_MAX_TOUCH; i++) {//GTP_MAX_TOUCH
#if GTP_WITH_PEN
            if (pre_pen == 1)
                break;
#endif
            if (touch_index & (0x01<<i)) {
                input_x = coor_data[pos + 1] |
                        coor_data[pos + 2] << 8;
                input_y = coor_data[pos + 3] |
                        coor_data[pos + 4] << 8;
                input_w = coor_data[pos + 5] |
                        coor_data[pos + 6] << 8;
                gtp_touch_down(ts, id,
                        input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                pos += 8;
                id = coor_data[pos] & 0x0F;
                touch_index |= (0x01<<id);
            } else {
                gtp_touch_up(ts, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }

    input_sync(ts->input_dev);
    //printk("*****\n");
exit_work_func:
    if (!ts->gtp_rawdiff_mode) {
        ret = gtp_i2c_write(ts->client, end_cmd, 3);
        if (ret < 0)
            dev_warn(&ts->client->dev, "I2C write end_cmd error!\n");

    }
    if (ts->use_irq)
        gtp_irq_enable(ts);
    return;
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
    HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data
        *ts = container_of(timer, struct goodix_ts_data, timer);

    queue_work(ts->goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000),
            HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
    IRQ_HANDLED: interrupt handled successfully
*********************************************************/

static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    struct goodix_ts_data *ts = dev_id;

    gtp_irq_disable(ts);
    queue_work(ts->goodix_wq, &ts->work);
    ac83xx_mask_ack_bim_irq(irq);
    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(struct goodix_ts_data *ts, int ms)
{
    gtp_gpio_direction_output(ts->pdata->irq_gpio, 0);
    msleep(ms);
    gtp_gpio_direction_input(ts->pdata->irq_gpio);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond, must >10ms
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct goodix_ts_data *ts, int ms)
{
    int ret = -1000;
    /* This reset sequence will selcet I2C slave address */
    set_irq_pin(ts, 1);
    ret  = GPIO_MultiFun_Set(RESET_PORT, PINMUX_LEVEL_GPIO_END_FLAG);
    ret  = gtp_gpio_direction_output(ts->pdata->reset_gpio, 0);
    msleep(ms);
    //debug
    if (ts->client->addr == GTP_I2C_ADDRESS_HIGH) {
        ret = gtp_gpio_direction_output(ts->pdata->irq_gpio, 1);
        }
    else {
        ret  = gtp_gpio_direction_output(ts->pdata->irq_gpio, 0);
    }

    //usleep(RESET_DELAY_T3_US);
    msleep(200);
    ret  = gtp_gpio_direction_output(ts->pdata->reset_gpio, 1);
    msleep(RESET_DELAY_T4);

    gtp_int_sync(ts, 500);

#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(ts->client);
#endif
        printk("[TP] Reset\n");
}
EXPORT_SYMBOL(gtp_reset_guitar);
#if GTP_SLIDE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    int ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {
        (u8)(GTP_REG_SLEEP >> 8),
        (u8)GTP_REG_SLEEP, 8};

#if GTP_DBL_CLK_WAKEUP
    i2c_control_buf[2] = 0x09;
#endif
    gtp_irq_disable(ts);

    while (retry++ < GTP_I2C_RETRY_3) {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret < 0) {
            dev_err(&ts->client->dev,
                "failed to set doze flag into 0x8046, %d",
                retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
        if (ret > 0) {
            doze_status = DOZE_ENABLED;
            dev_dbg(&ts->client->dev,
                "GTP has been working in doze mode!");
            gtp_irq_enable(ts);
            return ret;
        }
        msleep(20);
    }
    dev_err(&ts->client->dev, "GTP send doze cmd failed.\n");
    gtp_irq_enable(ts);
    return ret;
}
#else
/**
 * gtp_enter_sleep - Enter sleep mode
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static u8 gtp_enter_sleep(struct goodix_ts_data *ts)
{
    int ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {
        (u8)(GTP_REG_SLEEP >> 8),
        (u8)GTP_REG_SLEEP, 5};

    ret = gtp_gpio_direction_output(ts->pdata->irq_gpio, 0);
    if (ret)
        dev_err(&ts->client->dev,
            "GTP sleep: Cannot reconfig gpio %d.\n",
            ts->pdata->irq_gpio);
    if (ts->pdata->enable_power_off) {
        ret = gtp_gpio_direction_output(ts->pdata->reset_gpio, 0);
        if (ret)
            dev_err(&ts->client->dev,
                "GTP sleep: Cannot reconfig gpio %d.\n",
                ts->pdata->reset_gpio);
        ret = goodix_power_off(ts);
        if (ret) {
            dev_err(&ts->client->dev, "GTP power off failed.\n");
            return ret;
        }
        return 0;
    } else {
        msleep(50);
        while (retry++ < GTP_I2C_RETRY_5) {
            ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
            if (ret == 1) {
                dev_dbg(&ts->client->dev, "GTP enter sleep!");
                return 0;
            }
            msleep(20);
        }
        dev_err(&ts->client->dev, "GTP send sleep cmd failed.\n");
        return ret;
    }
}
#endif /* !GTP_SLIDE_WAKEUP */

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
    >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
    u8 retry = 0;
    s8 ret = -1;

    if (ts->pdata->enable_power_off) {
        ret = gtp_gpio_direction_output(ts->pdata->irq_gpio, 0);
        if (ret)
            dev_err(&ts->client->dev,
                "GTP wakeup: Cannot reconfig gpio %d.\n",
                ts->pdata->irq_gpio);
        ret = gtp_gpio_direction_output(ts->pdata->reset_gpio, 0);
        if (ret)
            dev_err(&ts->client->dev,
                "GTP wakeup: Cannot reconfig gpio %d.\n",
                ts->pdata->reset_gpio);
        ret = goodix_power_on(ts);
        if (ret) {
            dev_err(&ts->client->dev, "GTP power on failed.\n");
            return 0;
        }

        gtp_reset_guitar(ts, 20);
        ret = gtp_send_cfg(ts);
        if (ret < 0) {
            dev_err(&ts->client->dev,
                "GTP wakeup sleep failed.\n");
            return ret;
        } else {

                   ret = 0;
                }

        dev_dbg(&ts->client->dev,
                "Wakeup sleep send config success.");
    } else {
err_retry:
#if GTP_SLIDE_WAKEUP
        /* wakeup not by slide */
        if (DOZE_WAKEUP != doze_status)
            gtp_reset_guitar(ts, 10);
        else
            /* wakeup by slide */
            doze_status = DOZE_DISABLED;
#else
        if (chip_gt9xxs == 1) {
            gtp_reset_guitar(ts, 10);
        } else {
            ret = gtp_gpio_direction_output(ts->pdata->irq_gpio, 1);
            msleep(50);
        }
#endif
        ret = gtp_i2c_test(ts->client);
        if (ret == 2) {
            dev_dbg(&ts->client->dev, "GTP wakeup sleep.");
#if (!GTP_SLIDE_WAKEUP)
            if (chip_gt9xxs == 0) {
                gtp_int_sync(ts, 25);
                msleep(20);
#if GTP_ESD_PROTECT
                gtp_init_ext_watchdog(ts->client);
#endif
            }
#endif
            return ret;
        }
        gtp_reset_guitar(ts, 20);
        if (retry++ < 10)
            goto err_retry;
        dev_err(&ts->client->dev, "GTP wakeup sleep failed.\n");
    }
    return ret;
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
    > =0: succeed, otherwise: failed
*******************************************************/
static int gtp_init_panel(struct goodix_ts_data *ts)
{
    struct i2c_client *client = ts->client;
    unsigned char *config_data;
    int ret = -EIO;

#if GTP_DRIVER_SEND_CFG
    int i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;

    for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++)
        dev_dbg(&client->dev, "Config Groups(%d) Lengths: %d",
            i, ts->pdata->config_data_len[i]);

    ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
    if (SUCCESS == ret) {
        if (opr_buf[0] != 0xBE) {
            ts->fw_error = 1;
            dev_err(&client->dev,
                "Firmware error, no config sent!");
            return -EINVAL;
        }
    }

    for (i = 1; i < GOODIX_MAX_CFG_GROUP; i++) {
        if (ts->pdata->config_data_len[i])
            break;
    }
    if (i == GOODIX_MAX_CFG_GROUP) {
        sensor_id = 0;
    } else {
        ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_SENSOR_ID,
            &sensor_id, 1);
        if (SUCCESS == ret) {
            if (sensor_id >= GOODIX_MAX_CFG_GROUP) {
                dev_err(&client->dev,
                    "Invalid sensor_id(0x%02X), No Config Sent!",
                    sensor_id);
                return -EINVAL;
            }
        } else {
            dev_err(&client->dev,
                "Failed to get sensor_id, No config sent!");
            return -EINVAL;
        }
    }

    printk("[TP] Sensor ID selected: %d", sensor_id);

    if (ts->pdata->config_data_len[sensor_id] < GTP_CONFIG_MIN_LENGTH ||
        !ts->pdata->config_data[sensor_id]) {
        dev_err(&client->dev,
                "Sensor_ID(%d) matches with NULL or invalid config group!\n",
                sensor_id);
        return -EINVAL;
    }

    ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
        &opr_buf[0], 1);
    if (ret == SUCCESS) {
        if (opr_buf[0] < 90) {
            /* backup group config version */
            grp_cfg_version =
            ts->pdata->config_data[sensor_id][GTP_ADDR_LENGTH];
            ts->pdata->config_data[sensor_id][GTP_ADDR_LENGTH] =
                0x00;
            ts->fixed_cfg = 0;
        } else {
            /* treated as fixed config, not send config */
            dev_warn(&client->dev,
                "Ic fixed config with config version(%d, 0x%02X)",
                opr_buf[0], opr_buf[0]);
            ts->fixed_cfg = 1;
        }
    } else {
        dev_err(&client->dev,
            "Failed to get ic config version!No config sent!");
        return -EINVAL;
    }

    config_data = ts->pdata->config_data[sensor_id];
    ts->config_data = ts->pdata->config_data[sensor_id];
    ts->gtp_cfg_len = ts->pdata->config_data_len[sensor_id];

#if GTP_CUSTOM_CFG
    config_data[RESOLUTION_LOC] =
    (unsigned char)(GTP_MAX_WIDTH && 0xFF);
    config_data[RESOLUTION_LOC + 1] =
    (unsigned char)(GTP_MAX_WIDTH >> 8);
    config_data[RESOLUTION_LOC + 2] =
    (unsigned char)(GTP_MAX_HEIGHT && 0xFF);
    config_data[RESOLUTION_LOC + 3] =
    (unsigned char)(GTP_MAX_HEIGHT >> 8);

    if (GTP_INT_TRIGGER == 0)
        config_data[TRIGGER_LOC] &= 0xfe;
    else if (GTP_INT_TRIGGER == 1)
        config_data[TRIGGER_LOC] |= 0x01;
#endif  /* !GTP_CUSTOM_CFG */

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++)
        check_sum += config_data[i];

    config_data[ts->gtp_cfg_len] = (~check_sum) + 1;

#else /* DRIVER NOT SEND CONFIG */
    ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(ts->client, config_data,
            ts->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0) {
        dev_err(&client->dev,
                "Read Config Failed, Using DEFAULT Resolution & INT Trigger!\n");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }
#endif /* !DRIVER NOT SEND CONFIG */

    if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
        ts->abs_x_max = (config_data[RESOLUTION_LOC + 1] << 8)
                + config_data[RESOLUTION_LOC];
        ts->abs_y_max = (config_data[RESOLUTION_LOC + 3] << 8)
                + config_data[RESOLUTION_LOC + 2];
        ts->int_trigger_type = (config_data[TRIGGER_LOC]) & 0x03;
        
        ts->abs_x_max = ts->pdata->x_max;
        ts->abs_y_max = ts->pdata->y_max;


    }
        printk("[TP]gtp_send_cfg\n");
    ret = gtp_send_cfg(ts);
    if (ret < 0)
        dev_err(&client->dev, "%s: Send config error.\n", __func__);

    msleep(20);
    return ret;
}

/*******************************************************
Function:
    Read firmware version
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
    0: succeed, otherwise: failed
*******************************************************/
static int gtp_read_fw_version(struct i2c_client *client, u16 *version)
{
    int ret = 0;
    u8 buf[GTP_FW_VERSION_BUFFER_MAXSIZE] = {
        GTP_REG_FW_VERSION >> 8, GTP_REG_FW_VERSION & 0xff };

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0) {
        dev_err(&client->dev, "GTP read version failed.\n");
        return -EIO;
    }

    if (version)
        *version = (buf[3] << 8) | buf[2];

    return ret;
}
/*******************************************************
Function:
    Read and check chip id.
Input:
    client:  i2c device
Output:
    read operation return.
    0: succeed, otherwise: failed
*******************************************************/
static int gtp_check_product_id(struct i2c_client *client)
{
    int ret = 0;
    char product_id[GTP_PRODUCT_ID_MAXSIZE];
    struct goodix_ts_data *ts = i2c_get_clientdata(client);
    /* 04 bytes are used for the Product-id in the register space.*/
    u8 buf[GTP_PRODUCT_ID_BUFFER_MAXSIZE] = {
        GTP_REG_PRODUCT_ID >> 8, GTP_REG_PRODUCT_ID & 0xff };

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0) {
        dev_err(&client->dev, "GTP read product_id failed.\n");
        return -EIO;
    }

    if (buf[5] == 0x00) {
        /* copy (GTP_PRODUCT_ID_MAXSIZE - 1) from buffer. Ex: 915 */
        strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE - 1);
    } else {
        if (buf[5] == 'S' || buf[5] == 's')
            chip_gt9xxs = 1;
        /* copy GTP_PRODUCT_ID_MAXSIZE from buffer. Ex: 915s */
        strlcpy(product_id, &buf[2], GTP_PRODUCT_ID_MAXSIZE);
    }

    if(ts)
    {
         ret = strcmp(product_id, ts->pdata->product_id);
         if (ret != 0)
              return -EINVAL;
    }
 
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
    2: succeed, otherwise failed.
*******************************************************/
static int gtp_i2c_test(struct i2c_client *client)
{
    u8 buf[3] = { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };
    int retry = GTP_I2C_RETRY_5;
    int ret = -EIO;

    while (retry--) {
        ret = gtp_i2c_read(client, buf, 3);
        if (ret > 0)
            return ret;
        dev_err(&client->dev, "GTP i2c test failed time %d.\n", retry);
        msleep(20);
    }
    return ret;
}


#ifdef DEBUG
#define SHAKE_COUNT     (GTP_REG_CONFIG_DATA + 10)
#define TOUCH_LEVEL     (GTP_REG_CONFIG_DATA + 14)
#define LEAVE_LEVEL     (GTP_REG_CONFIG_DATA + 15)
#define REFRESH_RATE        (GTP_REG_CONFIG_DATA + 17)

static int gtp_i2c_config_read(struct i2c_client *client, unsigned int addr)
{
    u8 buf[3] = { addr >> 8, addr & 0xff };
    int retry = GTP_I2C_RETRY_5;
    int ret = -EIO;

    while (retry--) {
        ret = gtp_i2c_read(client, buf, 3);
        if (ret > 0) {
            printk(KERN_ERR "read buf=%x\n", buf[2]);
            return ret;
        }
        dev_err(&client->dev, "GTP i2c test failed time %d.\n", retry);
        msleep(20);
    }
    return ret;
}


static void gtp_i2c_config_reads(struct i2c_client *client)
{
    gtp_i2c_config_read(client, SHAKE_COUNT);
    gtp_i2c_config_read(client, TOUCH_LEVEL);
    gtp_i2c_config_read(client, LEAVE_LEVEL);
    gtp_i2c_config_read(client, REFRESH_RATE);
}

#endif
/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
    = 0: succeed, != 0: failed
*******************************************************/
static int gtp_request_io_port(struct goodix_ts_data *ts)
{
    struct i2c_client *client = ts->client;
    struct goodix_ts_platform_data *pdata = ts->pdata;
    int ret= 0;

    ret = gpio_request(pdata->irq_gpio, "goodix_ts_irq_gpio");
    if (ret) {
        dev_err(&client->dev, "Unable to request irq gpio [%d]\n",
            pdata->irq_gpio);
        goto out;
    }

    ret = gpio_request(pdata->reset_gpio, "goodix_ts_reset_gpio");
    if (ret) {
        dev_err(&client->dev, "Unable to request reset gpio [%d] ret: %d\n",
            pdata->reset_gpio, ret);
        gpio_free(pdata->irq_gpio);
        goto out;
    }
out:
    return ret;
}

static void gtp_set_eint()
{
#define EINT_TYPE_POSEDGE             (0 << 10)
#define EINT_TYPE_NEGEDGE             (1 << 10)
#define EINT_TYPE_HIGHLEVEL           (2 << 10)
#define EINT_TYPE_LOWLEVEL            (3 << 10)
#define EINT_TYPE_DUALEDGE            (4 << 10)
 
    GPIO_MultiFun_Set(INT_GPIO, INT_FUNCTION);
        BIM_SetEInt(EINT_NUM, EINT_TYPE_POSEDGE, 1);
}
    
/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
    0: succeed, -1: failed.
*******************************************************/
static int gtp_request_irq(struct goodix_ts_data *ts)
{
    int ret = 0;
        gtp_set_eint();
        ret = request_irq(ts->client->irq, goodix_ts_irq_handler,
            IRQ_TYPE_LEVEL_HIGH, ts->client->name, ts);
    ret  = 0;
    if (ret) {
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        ts->use_irq = false;
        return ret;
    } else {
        gtp_irq_disable(ts);
        ts->use_irq = true;
        return ret;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
    0: succeed, otherwise: failed.
*******************************************************/
static int gtp_request_input_dev(struct goodix_ts_data *ts)
{
    int ret;
    char phys[PHY_BUF_SIZE];
#if GTP_HAVE_TOUCH_KEY
    int index = 0;
#endif

    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) {
        dev_err(&ts->client->dev,
                "Failed to allocate input device.\n");
        return -ENOMEM;
    }

    ts->input_dev->evbit[0] =
        BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    input_mt_init_slots(ts->input_dev, 10);/* in case of "out of memory" */


#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < ts->pdata->num_button; index++) {
        input_set_capability(ts->input_dev,
                EV_KEY, ts->pdata->button_map[index]);
    }
#endif

#if GTP_SLIDE_WAKEUP
    input_set_capability(ts->input_dev, EV_KEY, KEY_POWER);
#endif

#if GTP_WITH_PEN
    /* pen support */
    __set_bit(BTN_TOOL_PEN, ts->input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
    __set_bit(INPUT_PROP_POINTER, ts->input_dev->propbit);
#endif

#if GTP_CHANGE_X2Y
    swap(ts->abs_x_max, ts->abs_y_max);
#endif


    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
                0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
                0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
                0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
                0, 255, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
                0, 255, 0, 0);

//  snprintf(phys, PHY_BUF_SIZE, "input/ts");
     snprintf(phys, PHY_BUF_SIZE, "%s/input0", dev_name(&ts->client->dev));

    ts->input_dev->name = GOODIX_DEV_NAME;
    ts->input_dev->phys = phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0xDEAD;
    ts->input_dev->id.product = 0xBEEF;
    ts->input_dev->id.version = 10427;

    ret = input_register_device(ts->input_dev);
    if (ret) {
        dev_err(&ts->client->dev,
                "Register %s input device failed.\n",
                ts->input_dev->name);
        goto exit_free_inputdev;
    }

    return 0;

exit_free_inputdev:
    input_free_device(ts->input_dev);
    ts->input_dev = NULL;
    return ret;
}
/*
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
    return (regulator_count_voltages(reg) > 0) ?
        regulator_set_optimum_mode(reg, load_uA) : 0;
    return 0;
}
*/
/**
 * goodix_power_on - Turn device power ON
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_on(struct goodix_ts_data *ts)
{
    int ret = 0;
/*
    if (ts->power_on) {
        dev_dbg(&ts->client->dev,
                "Device already power on\n");
        return 0;
    }

    if (!IS_ERR(ts->avdd)) {
        ret = reg_set_optimum_mode_check(ts->avdd,
            GOODIX_VDD_LOAD_MAX_UA);
        if (ret < 0) {
            dev_err(&ts->client->dev,
                "Regulator avdd set_opt failed rc=%d\n", ret);
            goto err_set_opt_avdd;
        }
        ret = regulator_enable(ts->avdd);
        if (ret) {
            dev_err(&ts->client->dev,
                "Regulator avdd enable failed ret=%d\n", ret);
            goto err_enable_avdd;
        }
    }

    if (!IS_ERR(ts->vdd)) {
        ret = regulator_set_voltage(ts->vdd, GOODIX_VTG_MIN_UV,
                       GOODIX_VTG_MAX_UV);
        if (ret) {
            dev_err(&ts->client->dev,
                "Regulator set_vtg failed vdd ret=%d\n", ret);
            goto err_set_vtg_vdd;
        }
        ret = reg_set_optimum_mode_check(ts->vdd,
            GOODIX_VDD_LOAD_MAX_UA);
        if (ret < 0) {
            dev_err(&ts->client->dev,
                "Regulator vdd set_opt failed rc=%d\n", ret);
            goto err_set_opt_vdd;
        }
        ret = regulator_enable(ts->vdd);
        if (ret) {
            dev_err(&ts->client->dev,
                "Regulator vdd enable failed ret=%d\n", ret);
            goto err_enable_vdd;
        }
    }

    if (!IS_ERR(ts->vcc_i2c)) {
        ret = regulator_set_voltage(ts->vcc_i2c, GOODIX_I2C_VTG_MIN_UV,
                       GOODIX_I2C_VTG_MAX_UV);
        if (ret) {
            dev_err(&ts->client->dev,
                "Regulator set_vtg failed vcc_i2c ret=%d\n",
                ret);
            goto err_set_vtg_vcc_i2c;
        }
        ret = reg_set_optimum_mode_check(ts->vcc_i2c,
            GOODIX_VIO_LOAD_MAX_UA);
        if (ret < 0) {
            dev_err(&ts->client->dev,
                "Regulator vcc_i2c set_opt failed rc=%d\n",
                ret);
            goto err_set_opt_vcc_i2c;
        }
        ret = regulator_enable(ts->vcc_i2c);
        if (ret) {
            dev_err(&ts->client->dev,
                "Regulator vcc_i2c enable failed ret=%d\n",
                ret);
            regulator_disable(ts->vdd);
            goto err_enable_vcc_i2c;
            }
    }

    ts->power_on = true;
    return 0;

err_enable_vcc_i2c:
err_set_opt_vcc_i2c:
    if (!IS_ERR(ts->vcc_i2c))
        regulator_set_voltage(ts->vcc_i2c, 0, GOODIX_I2C_VTG_MAX_UV);
err_set_vtg_vcc_i2c:
    if (!IS_ERR(ts->vdd))
        regulator_disable(ts->vdd);
err_enable_vdd:
err_set_opt_vdd:
    if (!IS_ERR(ts->vdd))
        regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
err_set_vtg_vdd:
    if (!IS_ERR(ts->avdd))
        regulator_disable(ts->avdd);
err_enable_avdd:
err_set_opt_avdd:
    ts->power_on = false;
*/
    return ret;
}

/**
 * goodix_power_off - Turn device power OFF
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_off(struct goodix_ts_data *ts)
{
    int ret = 0;
/*
    if (!ts->power_on) {
        dev_dbg(&ts->client->dev,
                "Device already power off\n");
        return 0;
    }

    if (!IS_ERR(ts->vcc_i2c)) {
        ret = regulator_set_voltage(ts->vcc_i2c, 0,
            GOODIX_I2C_VTG_MAX_UV);
        if (ret < 0)
            dev_err(&ts->client->dev,
                "Regulator vcc_i2c set_vtg failed ret=%d\n",
                ret);
        ret = regulator_disable(ts->vcc_i2c);
        if (ret)
            dev_err(&ts->client->dev,
                "Regulator vcc_i2c disable failed ret=%d\n",
                ret);
    }

    if (!IS_ERR(ts->vdd)) {
        ret = regulator_set_voltage(ts->vdd, 0, GOODIX_VTG_MAX_UV);
        if (ret < 0)
            dev_err(&ts->client->dev,
                "Regulator vdd set_vtg failed ret=%d\n", ret);
        ret = regulator_disable(ts->vdd);
        if (ret)
            dev_err(&ts->client->dev,
                "Regulator vdd disable failed ret=%d\n", ret);
    }

    if (!IS_ERR(ts->avdd)) {
        ret = regulator_disable(ts->avdd);
        if (ret)
            dev_err(&ts->client->dev,
                "Regulator avdd disable failed ret=%d\n", ret);
    }

    ts->power_on = false;
*/
    return ret;
}

/**
 * goodix_power_init - Initialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_init(struct goodix_ts_data *ts)
{
    int ret = 0;

//  ts->avdd = regulator_get(&ts->client->dev, "avdd");
//  if (IS_ERR(ts->avdd)) {
//      ret = PTR_ERR(ts->avdd);
//      dev_dbg(&ts->client->dev,
//          "Regulator get failed avdd ret=%d\n", ret);
//  }
//
//  ts->vdd = regulator_get(&ts->client->dev, "vdd");
//  if (IS_ERR(ts->vdd)) {
//      ret = PTR_ERR(ts->vdd);
//      dev_dbg(&ts->client->dev,
//          "Regulator get failed vdd ret=%d\n", ret);
//  }
//
//  ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc-i2c");
//  if (IS_ERR(ts->vcc_i2c)) {
//      ret = PTR_ERR(ts->vcc_i2c);
//      dev_dbg(&ts->client->dev,
//          "Regulator get failed vcc_i2c ret=%d\n", ret);
//  }
//
    return ret;
}

/**
 * goodix_power_deinit - Deinitialize device power
 * @ts: driver private data
 *
 * Returns zero on success, else an error.
 */
static int goodix_power_deinit(struct goodix_ts_data *ts)
{
//  regulator_put(ts->vdd);
//  regulator_put(ts->vcc_i2c);
//  regulator_put(ts->avdd);

    return 0;
}

static ssize_t gtp_fw_name_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);

    if (!strlen(ts->fw_name))
        return snprintf(buf, GTP_FW_NAME_MAXSIZE - 1,
            "No fw name has been given.");
    else
        return snprintf(buf, GTP_FW_NAME_MAXSIZE - 1,
            "%s\n", ts->fw_name);
}

static ssize_t gtp_fw_name_store(struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t size)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);
    
    if(ts == NULL)
    {
        printk("ts is null, goodix get drvdata fail\n");
        return 0;
    }
    if (size > GTP_FW_NAME_MAXSIZE - 1) {
        dev_err(dev, "FW name size exceeds the limit.");
        return -EINVAL;
    }

    strlcpy(ts->fw_name, buf, size);
    if (ts->fw_name[size-1] == '\n')
        ts->fw_name[size-1] = '\0';

    return size;
}

static DEVICE_ATTR(fw_name, (S_IRUGO | S_IWUSR | S_IWGRP),
            gtp_fw_name_show,
            gtp_fw_name_store);

static struct attribute *gtp_attrs[] = {
    &dev_attr_fw_name.attr,
    NULL
};

static const struct attribute_group gtp_attr_grp = {
    .attrs = gtp_attrs,
};

static int gtp_debug_suspend_set(void *_data, u64 val)
{
    struct goodix_ts_data *ts = _data;

    mutex_lock(&ts->input_dev->mutex);
    if (val)
        goodix_ts_suspend(&ts->client->dev);
    else
        goodix_ts_resume(&ts->client->dev);
    mutex_unlock(&ts->input_dev->mutex);

    return 0;
}

static int gtp_debug_suspend_get(void *_data, u64 *val)
{
    struct goodix_ts_data *ts = _data;

    mutex_lock(&ts->input_dev->mutex);
    *val = ts->gtp_is_suspend;
    mutex_unlock(&ts->input_dev->mutex);

    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, gtp_debug_suspend_get,
            gtp_debug_suspend_set, "%lld\n");

static int gtp_debugfs_init(struct goodix_ts_data *data)
{
    data->debug_base = debugfs_create_dir(GTP_DEBUGFS_DIR, NULL);

    if (IS_ERR_OR_NULL(data->debug_base)) {
        pr_err("Failed to create debugfs dir.\n");
            return -EINVAL;
    }

    if ((IS_ERR_OR_NULL(debugfs_create_file(GTP_DEBUGFS_FILE_SUSPEND,
                    S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
                    data->debug_base,
                    data,
                    &debug_suspend_fops)))) {
        pr_err("Failed to create suspend file.\n");
        debugfs_remove_recursive(data->debug_base);
        return -EINVAL;
    }

    return 0;
}

/*
static int goodix_ts_get_dt_coords(struct device *dev, char *name,
                struct goodix_ts_platform_data *pdata)
{
    struct property *prop;
    struct device_node *np = dev->of_node;
    int rc;
    u32 coords[GOODIX_COORDS_ARR_SIZE];

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    rc = of_property_read_u32_array(np, name, coords,
        GOODIX_COORDS_ARR_SIZE);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read %s\n", name);
        return rc;
    }

    if (!strcmp(name, "goodix,panel-coords")) {
        pdata->panel_minx = coords[0];
        pdata->panel_miny = coords[1];
        pdata->panel_maxx = coords[2];
        pdata->panel_maxy = coords[3];
    } else if (!strcmp(name, "goodix,display-coords")) {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    } else {
        dev_err(dev, "unsupported property %s\n", name);
        return -EINVAL;
    }

    return 0;
}

static int goodix_parse_dt(struct device *dev,
            struct goodix_ts_platform_data *pdata)
{
    int rc;
    struct device_node *np = dev->of_node;
    struct property *prop;
    u32 temp_val, num_buttons;
    u32 button_map[MAX_BUTTONS];
    char prop_name[PROP_NAME_SIZE];
    int i, read_cfg_num;

    rc = goodix_ts_get_dt_coords(dev, "goodix,panel-coords", pdata);
    if (rc && (rc != -EINVAL))
        return rc;

    rc = goodix_ts_get_dt_coords(dev, "goodix,display-coords", pdata);
    if (rc)
        return rc;

    pdata->i2c_pull_up = of_property_read_bool(np,
                        "goodix,i2c-pull-up");

    pdata->no_force_update = of_property_read_bool(np,
                        "goodix,no-force-update");

    pdata->enable_power_off = of_property_read_bool(np,
                        "goodix,enable-power-off");
    pdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios",
                0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        return pdata->reset_gpio;

    pdata->irq_gpio = of_get_named_gpio_flags(np, "interrupt-gpios",
                0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        return pdata->irq_gpio;

    rc = of_property_read_string(np, "goodix,product-id",
                        &pdata->product_id);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Failed to parse product_id.");
        return -EINVAL;
    }

    rc = of_property_read_string(np, "goodix,fw_name",
                        &pdata->fw_name);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Failed to parse firmware name.\n");
        return -EINVAL;
    }

    prop = of_find_property(np, "goodix,button-map", NULL);
    if (prop) {
        num_buttons = prop->length / sizeof(temp_val);
        if (num_buttons > MAX_BUTTONS)
            return -EINVAL;

        rc = of_property_read_u32_array(np,
            "goodix,button-map", button_map,
            num_buttons);
        if (rc) {
            dev_err(dev, "Unable to read key codes\n");
            return rc;
        }
        pdata->num_button = num_buttons;
        memcpy(pdata->button_map, button_map,
            pdata->num_button * sizeof(u32));
    }

    read_cfg_num = 0;
    for (i = 0; i < GOODIX_MAX_CFG_GROUP; i++) {
        snprintf(prop_name, sizeof(prop_name), "goodix,cfg-data%d", i);
        prop = of_find_property(np, prop_name,
            &pdata->config_data_len[i]);
        if (!prop || !prop->value) {
            pdata->config_data_len[i] = 0;
            pdata->config_data[i] = NULL;
            continue;
        }
        pdata->config_data[i] = devm_kzalloc(dev,
                GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH,
                GFP_KERNEL);
        if (!pdata->config_data[i]) {
            dev_err(dev,
                "Not enough memory for panel config data %d\n",
                i);
            return -ENOMEM;
        }
        pdata->config_data[i][0] = GTP_REG_CONFIG_DATA >> 8;
        pdata->config_data[i][1] = GTP_REG_CONFIG_DATA & 0xff;
        memcpy(&pdata->config_data[i][GTP_ADDR_LENGTH],
                prop->value, pdata->config_data_len[i]);
        read_cfg_num++;
    }
    dev_dbg(dev, "%d config data read from device tree.\n", read_cfg_num);

    return 0;
}

*/
/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
    0: succeed.
*******************************************************/

static int goodix_ts_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
    struct goodix_ts_platform_data *pdata;
    struct goodix_ts_data *ts;
    u16 version_info;
    int ret = 0;

    dev_dbg(&client->dev, "GTP I2C Address: 0x%02x\n", client->addr);
    if (client->dev.of_node) {
        pdata = NULL;
    /*
        pdata = devm_kzalloc(&client->dev,
            sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev,
                "GTP Failed to allocate memory for pdata\n");
            return -ENOMEM;
        }

        ret = goodix_parse_dt(&client->dev, pdata);
        if (ret)
            return ret;
    */
    } else {
        pdata = client->dev.platform_data;
    }

    if (!pdata) {
        dev_err(&client->dev, "GTP invalid pdata\n");
        return -EINVAL;
    }

    i2c_connect_client = client;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "GTP I2C not supported\n");
        return -ENODEV;
    }

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
    if (!ts) {
        dev_err(&client->dev, "GTP not enough memory for ts\n");
        return -ENOMEM;
    }

    memset(ts, 0, sizeof(*ts));
    ts->client = client;
    ts->pdata = pdata;
    /* For 2.6.39 & later use spin_lock_init(&ts->irq_lock)
     * For 2.6.39 & before, use ts->irq_lock = SPIN_LOCK_UNLOCKED
     */
    spin_lock_init(&ts->irq_lock);
    i2c_set_clientdata(client, ts);
    ts->gtp_rawdiff_mode = 0;
    ts->power_on = false;
    ts_data = ts;

    ret = gtp_request_io_port(ts);
    if (ret) {
        dev_err(&client->dev, "GTP request IO port failed.\n");
        goto exit_free_client_data;
    }

    ret = goodix_power_init(ts);
    if (ret) {
        dev_err(&client->dev, "GTP power init failed\n");
        goto exit_free_io_port;
    }

    ret = goodix_power_on(ts);
    if (ret) {
        dev_err(&client->dev, "GTP power on failed\n");
        goto exit_deinit_power;
    }

    gtp_reset_guitar(ts, 200);

#ifdef DEBUG
    ret = gtp_i2c_test(client);

    if (ret != 2) {
        dev_err(&client->dev, "I2C communication ERROR!\n");
    }
#endif
    if (pdata->fw_name)
        strlcpy(ts->fw_name, pdata->fw_name,
                        strlen(pdata->fw_name) + 1);
                        
#ifdef CONFIG_GT9XX_TOUCHPANEL_UPDATE
        ret = gup_init_update_proc(ts);
        if (ret < 0) {
            dev_err(&client->dev,
                "GTP Create firmware update thread error.\n");
        }
#endif

    ret = gtp_init_panel(ts);
    if (ret < 0) {
        dev_err(&client->dev, "GTP init panel failed.\n");
        ts->abs_x_max = GTP_MAX_WIDTH;
        ts->abs_y_max = GTP_MAX_HEIGHT;
        ts->int_trigger_type = GTP_INT_TRIGGER;
    }

    ret = gtp_request_input_dev(ts);
    if (ret) {
        dev_err(&client->dev, "GTP request input dev failed.\n");
        goto exit_free_inputdev;
    }
    input_set_drvdata(ts->input_dev, ts);

    mutex_init(&ts->lock);
#if defined(CONFIG_FB)
    ts->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts->fb_notif);
    if (ret)
        dev_err(&ts->client->dev,
            "Unable to register fb_notifier: %d\n",
            ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = goodix_ts_early_suspend;
    ts->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    ts->goodix_wq = create_singlethread_workqueue("goodix_wq");
    INIT_WORK(&ts->work, goodix_ts_work_func);

    ret = gtp_request_irq(ts);
    if (ret)
        dev_dbg(&client->dev, "GTP request irq failed %d.\n", ret);
    else
        dev_dbg(&client->dev, "GTP works in interrupt mode.\n");

    ret = gtp_read_fw_version(client, &version_info);
    if (ret != 2)
        dev_err(&client->dev, "GTP firmware version read failed.\n");
    printk("fw_version=0x%x\n",version_info);
    ret = gtp_check_product_id(client);
    if (ret != 0) {
        dev_err(&client->dev, "GTP Product id doesn't match.\n");
        goto exit_free_irq;
    }
    if (ts->use_irq)
        gtp_irq_enable(ts);

#ifdef CONFIG_GT9XX_TOUCHPANEL_DEBUG
    init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
    ret = sysfs_create_group(&client->dev.kobj, &gtp_attr_grp);
    if (ret < 0) {
        dev_err(&client->dev, "sys file creation failed.\n");
        goto exit_free_irq;
    }

    ret = gtp_debugfs_init(ts);
    if (ret != 0) {
        dev_err(&client->dev, "Failed to create debugfs entries, %d\n",
                        ret);
        goto exit_remove_sysfs;
    }

    
    init_done = true;
    return 0;
exit_free_irq:
    mutex_destroy(&ts->lock);
#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts->fb_notif))
        dev_err(&client->dev,
            "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts->early_suspend);
#endif
    if (ts->use_irq)
        free_irq(client->irq, ts);
    else
        hrtimer_cancel(&ts->timer);
    cancel_work_sync(&ts->work);
    if(ts->goodix_wq !=NULL)
    {
        flush_workqueue(ts->goodix_wq);
       destroy_workqueue(ts->goodix_wq);
    }
    
exit_remove_sysfs:
    sysfs_remove_group(&ts->input_dev->dev.kobj, &gtp_attr_grp);

    if (ts->input_dev) {
        input_unregister_device(ts->input_dev);
        input_free_device(ts->input_dev);
        ts->input_dev = NULL;
    }
exit_free_inputdev:
    kfree(ts->config_data);
    goodix_power_off(ts);
exit_deinit_power:
    goodix_power_deinit(ts);
exit_free_io_port:
        gpio_free(pdata->reset_gpio);
        gpio_free(pdata->irq_gpio);
exit_free_client_data:
    i2c_set_clientdata(client, NULL);
    return ret;
}

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    struct goodix_ts_platform_data *pdata;
    struct goodix_ts_data *ts = i2c_get_clientdata(client);

    if(ts == NULL)
    {
        printk("ts is null, goodix get clientdata fail\n");
        return 0;
    }
#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts->fb_notif))
        dev_err(&client->dev,
            "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts->early_suspend);
#endif
    mutex_destroy(&ts->lock);

#ifdef CONFIG_GT9XX_TOUCHPANEL_DEBUG
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    cancel_work_sync(gtp_esd_check_workqueue);
    flush_workqueue(gtp_esd_check_workqueue);
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    if (ts) {
        if (ts->use_irq)
            free_irq(client->irq, ts);
        else
            hrtimer_cancel(&ts->timer);

        cancel_work_sync(&ts->work);
        flush_workqueue(ts->goodix_wq);
        destroy_workqueue(ts->goodix_wq);

        if (ts->input_dev) {
            input_unregister_device(ts->input_dev);
            input_free_device(ts->input_dev);
            ts->input_dev = NULL;
        }

            gpio_free(ts->pdata->reset_gpio);
            gpio_free(ts->pdata->irq_gpio);

        goodix_power_off(ts);
        goodix_power_deinit(ts);
        i2c_set_clientdata(client, NULL);
    }
    
    debugfs_remove_recursive(ts->debug_base);
   
    pdata = client->dev.platform_data;
    kfree(pdata->product_id);
    kfree(pdata->fw_name);
    kfree(pdata);
    devm_kfree(&client->dev, ts);
    printk("[TP] goodix_ts_remove\n");
    return 0;
    
}

/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int goodix_ts_suspend(struct device *dev)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);
    int ret = 0, i;
    if(ts == NULL)
    {
        printk("ts is null , goodix get drvdata fail\n");
        return 0;
    }
    if (ts->gtp_is_suspend) {
        dev_dbg(&ts->client->dev, "Already in suspend state.\n");
        return 0;
    }

    mutex_lock(&ts->lock);
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_SLIDE_WAKEUP
    ret = gtp_enter_doze(ts);
#else
    if (ts->use_irq) {
                ac83xx_mask_ack_bim_irq(ts->client->irq);
        gtp_irq_disable(ts);
         }
    else
        hrtimer_cancel(&ts->timer);

    for (i = 0; i < GTP_MAX_TOUCH; i++)
        gtp_touch_up(ts, i);

    input_sync(ts->input_dev);

    ret = gtp_enter_sleep(ts);
#endif
    if (ret < 0)
        dev_err(&ts->client->dev, "GTP early suspend failed.\n");
    /* to avoid waking up while not sleeping,
     * delay 48 + 10ms to ensure reliability
     */
    msleep(58);
    mutex_unlock(&ts->lock);
    ts->gtp_is_suspend = 1;
        printk("[TP] goodix_ts_suspend ret=%d\n", ret);
    return ret;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static int goodix_ts_resume(struct device *dev)
{
    struct goodix_ts_data *ts = dev_get_drvdata(dev);
    int ret = 0;
    if(ts == NULL)
    {
        printk("ts is null , goodix get drvdata fail\n");
        return 0;
    }
    if (!ts->gtp_is_suspend) {
        dev_dbg(&ts->client->dev, "Already in awake state.\n");
        return 0;
    }

    mutex_lock(&ts->lock);
    ret = gtp_wakeup_sleep(ts);

#if GTP_SLIDE_WAKEUP
    doze_status = DOZE_DISABLED;
#endif

    if (ret <= 0)
        dev_err(&ts->client->dev, "GTP resume failed.\n");

    if (ts->use_irq) {
            gtp_set_eint();
        gtp_irq_enable(ts);
        } else {
        hrtimer_start(&ts->timer,
            ktime_set(1, 0), HRTIMER_MODE_REL);
        }
#if GTP_ESD_PROTECT
    gtp_esd_switch(ts->client, SWITCH_ON);
#endif
    mutex_unlock(&ts->lock);
    ts->gtp_is_suspend = 0;

        printk("[TP] goodix_ts_resume ret=%d\n", ret);
    return ret;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct goodix_ts_data *ts =
        container_of(self, struct goodix_ts_data, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
            ts && ts->client) {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK)
            goodix_ts_resume(&ts->client->dev);
        else if (*blank == FB_BLANK_POWERDOWN)
            goodix_ts_suspend(&ts->client->dev);
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    struct goodix_ts_data *ts;

    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_suspend(&ts->client->dev);
    return;
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
    struct goodix_ts_data *ts;

    ts = container_of(h, struct goodix_ts_data, early_suspend);
    goodix_ts_late_resume(ts);
    return;
}
#endif

#if GTP_ESD_PROTECT
/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on: SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, int on)
{
    struct goodix_ts_data *ts;

    ts = i2c_get_clientdata(client);
    if (SWITCH_ON == on) {
        /* switch on esd  */
        if (!ts->esd_running) {
            ts->esd_running = 1;
            dev_dbg(&client->dev, "Esd started\n");
            queue_delayed_work(gtp_esd_check_workqueue,
                &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
        }
    } else {
        /* switch off esd */
        if (ts->esd_running) {
            ts->esd_running = 0;
            dev_dbg(&client->dev, "Esd cancelled\n");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
    }
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
        1: succeed, otherwise: failed
*********************************************************/
static int gtp_init_ext_watchdog(struct i2c_client *client)
{
    /* in case of recursively reset by calling gtp_i2c_write*/
    struct i2c_msg msg;
    u8 opr_buffer[4] = {0x80, 0x40, 0xAA, 0xAA};
    int ret;
    int retries = 0;

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = 4;
    msg.buf   = opr_buffer;

    while (retries < GTP_I2C_RETRY_5) {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)
            return 1;
        retries++;
    }
    if (retries == GTP_I2C_RETRY_5)
        dev_err(&client->dev, "init external watchdog failed!");
    return 0;
}

/*******************************************************
Function:
    Esd protect function.
    Added external watchdog by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 retry;
    s32 ret = -1;
    struct goodix_ts_data *ts = NULL;
    u8 test[4] = {0x80, 0x40};

    ts = i2c_get_clientdata(i2c_connect_client);

    if (ts->gtp_is_suspend) {
        dev_dbg(&ts->client->dev, "Esd terminated!\n");
        ts->esd_running = 0;
        return;
    }
#ifdef CONFIG_GT9XX_TOUCHPANEL_UPDATE
    if (ts->enter_update)
        return;
#endif

    for (retry = 0; retry < GTP_I2C_RETRY_3; retry++) {
        ret = gtp_i2c_read(ts->client, test, 4);

        if ((ret < 0)) {
            /* IC works abnormally..*/
            continue;
        } else {
            if ((test[2] == 0xAA) || (test[3] != 0xAA)) {
                /* IC works abnormally..*/
                retry = GTP_I2C_RETRY_3;
                break;
            } else {
                /* IC works normally, Write 0x8040 0xAA*/
                test[2] = 0xAA;
                gtp_i2c_write(ts->client, test, 3);
                break;
            }
        }
    }
    if (retry == GTP_I2C_RETRY_3) {
        dev_err(&ts->client->dev,
            "IC Working ABNORMALLY, Resetting Guitar...\n");
        gtp_reset_guitar(ts, 50);
    }

    if (!ts->gtp_is_suspend)
        queue_delayed_work(gtp_esd_check_workqueue,
            &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
    else {
        dev_dbg(&ts->client->dev, "Esd terminated!\n");
        ts->esd_running = 0;
    }

    return;
}
#endif

static const struct dev_pm_ops goodix_ts_dev_pm_ops = {
    .suspend = goodix_ts_suspend,
    .resume = goodix_ts_resume,
};

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

static struct of_device_id goodix_match_table[] = {
    { .compatible = "goodix,gt9xx", },
    { },
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_early_suspend,
    .resume     = goodix_ts_late_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
        .of_match_table = goodix_match_table,
        .pm = &goodix_ts_dev_pm_ops,
    },
};

#ifdef DEBUG
static s32 i2c_write_bytes(struct i2c_client *client,u8 *data,s32 len)
{
    struct i2c_msg msg;
    s32 ret=-1;
    
    msg.flags=!I2C_M_RD;
    msg.addr=client->addr;
    msg.len=len;
    msg.buf=data;        

    ret=i2c_transfer(client->adapter,&msg, 1);
    return !ret;
}

static s32 i2c_read_bytes(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;

    msgs[0].flags=!I2C_M_RD; 
    msgs[0].addr=client->addr;
    msgs[0].len=2;
    msgs[0].buf=&buf[0];
    //first msg end
    msgs[1].flags=I2C_M_RD;
    msgs[1].addr=client->addr;
    msgs[1].len=len - 2;
    msgs[1].buf=&buf[2];
    //second msg end
    
    ret=i2c_transfer(client->adapter,msgs, 2);
    return !ret;
}
#endif

/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit goodix_ts_init(void)
{
    struct goodix_ts_platform_data *pdata;
    struct i2c_board_info info;
    struct i2c_adapter *adapter;
    struct i2c_client *client;
    int ret = 0;

    pdata = kzalloc(sizeof(struct goodix_ts_platform_data), GFP_KERNEL);
    if (!pdata) {
        printk(KERN_ERR "GTP Failed to allocate memory for pdata\n");
        return -ENOMEM;
    }
    /* initialise the default value for platform data */
    pdata->irq_gpio = INT_GPIO;
    //pdata->irq_gpio_flags = 0;
    pdata->reset_gpio = RESET_PORT;
    pdata->reset_gpio_flags = 0;
    
    pdata->product_id = kzalloc(GTP_PRODUCT_ID_MAXSIZE, GFP_KERNEL);
    if(!pdata->product_id) {
        printk(KERN_ERR "GTP Failed to allocate memory for pdata->product_id\n");
        kfree(pdata);
        return -ENOMEM;
    }
    strlcpy((char *)pdata->product_id, "911", 4);
    pdata->fw_name = kzalloc(GTP_FW_NAME_MAXSIZE, GFP_KERNEL);
    if(!pdata->fw_name) {
        printk(KERN_ERR "GTP Failed to allocate memory for pdata->fw_name\n");
        kfree(pdata->product_id);
        kfree(pdata);
        return -ENOMEM;
    }
    strlcpy((char *)pdata->fw_name, "_goodix_update_.bin", 20);
    pdata->x_max = DISPLAY_MAX_WIDTH;
    pdata->y_max = DISPLAY_MAX_HEIGHT;
    pdata->x_min = 0;
    pdata->y_min = 0;
    pdata->panel_minx = 0;
    pdata->panel_miny = 0 ;
    pdata->panel_maxx = DISPLAY_MAX_HEIGHT;
    pdata->panel_maxy = DISPLAY_MAX_WIDTH;
    pdata->no_force_update = true;
    pdata->i2c_pull_up = false;
    pdata->enable_power_off = true;

    pdata->config_data_len[0] = CFG_GROUP_LEN(cfg_group0_data);
    pdata->config_data_len[1] = CFG_GROUP_LEN(cfg_group1_data);
    pdata->config_data_len[2] = CFG_GROUP_LEN(cfg_group2_data);
    pdata->config_data_len[3] = CFG_GROUP_LEN(cfg_group3_data);
    pdata->config_data_len[4] = CFG_GROUP_LEN(cfg_group4_data);
    pdata->config_data_len[5] = CFG_GROUP_LEN(cfg_group5_data);

    pdata->config_data[0] = cfg_group0_data;
    pdata->config_data[1] = cfg_group1_data;
    pdata->config_data[2] = cfg_group2_data;
    pdata->config_data[3] = cfg_group3_data;
    pdata->config_data[4] = cfg_group4_data;
    pdata->config_data[5] = cfg_group5_data;

    pdata->num_button = 0;

    memset(&info, 0, sizeof(struct i2c_board_info));

    info.addr = GTP_I2C_ADDRESS_LOW;
    info.platform_data=pdata;
    strlcpy(info.type, GOODIX_I2C_NAME, I2C_NAME_SIZE);

    adapter = i2c_get_adapter(0);
    if (!adapter)
    {
        printk(KERN_ERR "Can't get i2c adapter 0\n");
        kfree(pdata->product_id);
        kfree(pdata->fw_name);
        kfree(pdata);
        ret = -ENODEV;
    }

    client = i2c_new_device(adapter, &info);
    i2c_put_adapter(adapter);
    if (!client)
    {
        printk(KERN_ERR "Can't add i2c device at 0x%x\n", (unsigned int)info.addr);
        kfree(pdata->product_id);
        kfree(pdata->fw_name);
        kfree(pdata);
        ret = -ENOMEM;
        goto exit;
    }
#if GTP_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif
    ret = i2c_add_driver(&goodix_ts_driver);
exit:
    printk(KERN_INFO "[GTP] Touch Panel Driver Loading Okay!\n");
    return ret;
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    i2c_del_driver(&goodix_ts_driver);
        i2c_unregister_device(i2c_connect_client);
}

module_init(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
