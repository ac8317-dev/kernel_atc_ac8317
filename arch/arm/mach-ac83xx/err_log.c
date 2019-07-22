/*
 *  linux/arch/arm/mach-ac83xx/error_log.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * Print fatal error log to nand.
 */
 
#include <mach/chip_ver.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/debug_locks.h>
#include <linux/vmalloc.h>
#include <linux/sort.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <linux/genhd.h>
#include <linux/param.h>
#include <mtd/mtd-abi.h>


/* Error log related */
#define ERR_LOG_INFO_FILE_PATH       ("/log_data/err_log_info.txt")
#define ERR_LOG_DATA_FILE_PATH       ("/log_data/err_log_data.txt")
#define ERR_LOG_INFO_FILE_MAX_LEN    (50)
#define ERR_LOG_DATA_FILE_MAX_LEN    (1024 * 13)


static void *err_log_start(struct seq_file *m, loff_t *pos);
static void err_log_stop(struct seq_file *m, void *v);
static void *err_log_next(struct seq_file *m, void *v, loff_t *pos);
static int err_log_show(struct seq_file *m, void *v);
static int err_log_open(struct inode *inode, struct file *file);
static ssize_t err_log_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static int err_log_release(struct inode *inode, struct file *file);


static const struct file_operations proc_err_log_operations = {
	.open		= err_log_open,
	.write		= err_log_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= err_log_release,
};


static struct seq_operations err_log_ops = {
	.start	= err_log_start,
	.next	= err_log_next,
	.stop	= err_log_stop,
	.show	= err_log_show,
};


/*-----------------------------------------------------------------------------
 * Name             - _err_log_read_pos
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int32_t _err_log_read_pos(struct file* filp_info, int32_t* pi4_log_pos)
{
    char      ac_log_info[ERR_LOG_INFO_FILE_MAX_LEN] = {0};
    int32_t     i4_read_len = 0;
    loff_t    t_cur_pos = 0;

    printk("<ERR_LOG> %s() IN!\n", __FUNCTION__);
    
    if (IS_ERR(filp_info))
    {
        printk("<ERR_LOG> FILE(%s), LINE(%d), filp_info = %d\n", 
            __FILE__, __LINE__, filp_info);
        
        return -1;
    }
    else
    {
        /* Set the cursor to the begining of the log info file */
        t_cur_pos = vfs_llseek(filp_info, 0, SEEK_SET);
        
        /* Read log info file */
        i4_read_len = vfs_read(
            filp_info, 
            ac_log_info, 
            ERR_LOG_INFO_FILE_MAX_LEN, 
            &t_cur_pos);
        
        printk("<ERR_LOG> i4_read_len = %d\n", i4_read_len);
        printk("<ERR_LOG> ac_log_info = %s\n", ac_log_info);
        
        if (0 == strcmp(ac_log_info, ""))
        {
            *pi4_log_pos = 0;
        }
        else
        {
            /* Get current position */
            *pi4_log_pos = (int32_t)simple_strtol(ac_log_info, NULL, 10);
        }

        printk("<ERR_LOG> *pi4_log_pos = %d\n", *pi4_log_pos);
    }

    printk("<ERR_LOG> %s() OUT!\n", __FUNCTION__);
    return 0;
}


/*-----------------------------------------------------------------------------
 * Name             - _err_log_store_pos
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int32_t _err_log_store_pos(struct file* filp_info, int32_t i4_log_pos)
{
    loff_t    t_cur_pos = 0;
    char      ac_log_info[ERR_LOG_INFO_FILE_MAX_LEN] = {0};

    printk("<ERR_LOG> %s() IN!\n", __FUNCTION__);
    printk("<ERR_LOG> i4_log_pos = %d\n", i4_log_pos);
    
    if (IS_ERR(filp_info))
    {
        printk("<ERR_LOG> FILE(%s), LINE(%d), filp_info = %d\n", 
            __FILE__, __LINE__, filp_info);
        
        return -1;
    }
    else
    {
        /* Convert digit to ascii */
        snprintf(ac_log_info, sizeof(ac_log_info), "%d", i4_log_pos);

        /* Write the current position to file */
        vfs_write(filp_info, ac_log_info, sizeof(ac_log_info), &t_cur_pos);
    }

    printk("<ERR_LOG> %s() OUT!\n", __FUNCTION__);
    return 0;
}


/*-----------------------------------------------------------------------------
 * Name             - print_err_log
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
void print_err_log(const char* ps_err_str)
{
    char*           pc_time = NULL;
    int32_t           i4_log_pos = 0;
    loff_t          t_cur_pos = 0;
    uint32_t          ui4_err_str_len = 0;
    char            ac_log_tmp[ERR_LOG_INFO_FILE_MAX_LEN] = {0};
    int32_t           i4_ret = 0;
    struct file*    filp_data;
    struct file*    filp_info;
    mm_segment_t    old_fs;
    static int32_t    i4_item_cnt = 0;
#if 0
    if (!h_s_log_sema)
    {
        /* Use semaphore to make it thread safe */
        if (OSR_OK != x_sema_create(&h_s_log_sema, X_SEMA_TYPE_BINARY, X_SEMA_STATE_UNLOCK))
        {
            h_s_log_sema = NULL_HANDLE;
            return;
        }
    }
#endif
    //x_sema_lock(h_s_log_sema, X_SEMA_OPTION_WAIT);

    if (NULL == ps_err_str)
    {
        goto _END;
    }

    printk("<ERR_LOG> =======================================================\n");

    if (1)//(!b_s_IFcon_MW_init)
    {
        /* Can not get system time, use a fake one */
        snprintf(
            ac_log_tmp, 
            sizeof(ac_log_tmp), 
            "[%03d] %s [ErrCode:%s]\n", 
            i4_item_cnt,
            "Clock is off",
            ps_err_str);
    }
    else
    {
        /* Get system time */
        //_ifcon_get_time_str(&pc_time);

        snprintf(
            ac_log_tmp, 
            sizeof(ac_log_tmp), 
            "[%03d] %s [ErrCode:%s]\n", 
            i4_item_cnt,
            pc_time, 
            ps_err_str);
    }
    
    i4_item_cnt++;
    if (i4_item_cnt == 256)
    {
        i4_item_cnt = 0;
    }
    
    printk("<ERR_LOG> ac_log_tmp = %s\n", ac_log_tmp);
    printk("<ERR_LOG> strlen(ac_log_tmp) = %d\n", strlen(ac_log_tmp));
    
    /* Get Log string length */
    ui4_err_str_len = strlen(ac_log_tmp);
    printk("<ERR_LOG> ui4_err_str_len = %d\n", ui4_err_str_len);

    /* Set FS for kernel space */
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    /* Open log info file */
    filp_info = filp_open(
        ERR_LOG_INFO_FILE_PATH, 
        O_RDWR, 
        0);
    if (IS_ERR(filp_info)) /* No log info file */
    {
        printk("<ERR_LOG> FILE(%s), LINE(%d), filp_open() filp_info = %d\n", 
            __FILE__, __LINE__, filp_info);

        i4_log_pos = 0;
    }
    else /* Log info file already created */
    {
        /* Get log position from log info file */
        i4_ret = _err_log_read_pos(filp_info, &i4_log_pos);
        if (0 != i4_ret)
        {
            printk("<ERR_LOG> FILE(%s), LINE(%d), _err_log_read_pos() return = %d\n", 
                __FILE__, __LINE__, i4_ret);
            
            filp_close(filp_info, NULL);
            set_fs(old_fs);
            goto _END;
        }

        filp_close(filp_info, NULL);
    }
    printk("<ERR_LOG> i4_log_pos = %d\n", i4_log_pos);

    if ((i4_log_pos + ui4_err_str_len) > ERR_LOG_DATA_FILE_MAX_LEN)
    {
        printk("<ERR_LOG> Redirect to file start!\n");
        i4_log_pos = 0;
    }

    /* Open/Create log data file */
    filp_data = filp_open(
        ERR_LOG_DATA_FILE_PATH, 
        O_CREAT | O_SYNC | O_WRONLY, 
        0);
    if (IS_ERR(filp_data))
    {
        printk("<ERR_LOG> FILE(%s), LINE(%d), filp_open() filp_info = %d\n", 
            __FILE__, __LINE__, filp_data);
        
        set_fs(old_fs);
        goto _END;
    }
    else
    {
        printk("<ERR_LOG> Log data file created successully!\n");
    }

    /* Seek position */
    t_cur_pos = vfs_llseek(filp_data, (loff_t)i4_log_pos, SEEK_SET);
    printk("<ERR_LOG> Before write log t_cur_pos = %d\n", (int32_t)t_cur_pos);

    /* Write Log data to nand */
    vfs_write(filp_data, ac_log_tmp, ui4_err_str_len, &t_cur_pos);
    filp_close(filp_data, NULL);

    printk("<ERR_LOG> After write log t_cur_pos = %d\n", (int32_t)t_cur_pos);
    i4_log_pos = t_cur_pos;

    /* Open log info file */
    filp_info = filp_open(
        ERR_LOG_INFO_FILE_PATH, 
        O_CREAT | O_SYNC | O_WRONLY | O_TRUNC,
        0);
    if (IS_ERR(filp_info))
    {
        printk("<ERR_LOG> FILE(%s), LINE(%d), filp_open() filp_info = %d\n", 
            __FILE__, __LINE__, filp_info);

        set_fs(old_fs);
        goto _END;
    }
    
    /* Store current position to log position file */
    _err_log_store_pos(filp_info, i4_log_pos);

    filp_close(filp_data, NULL);
    set_fs(old_fs);
    
    printk("<ERR_LOG> =======================================================\n");
    
_END:
    
    //x_sema_unlock(h_s_log_sema);
    return;
}
EXPORT_SYMBOL(print_err_log);


/*-----------------------------------------------------------------------------
 * Name             - err_log_start
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static void *err_log_start(struct seq_file *m, loff_t *pos)
{
	return NULL;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_next
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static void *err_log_next(struct seq_file *m, void *v, loff_t *pos)
{
	return NULL;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_stop
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static void err_log_stop(struct seq_file *m, void *v)
{}


/*-----------------------------------------------------------------------------
 * Name             - err_log_show
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int err_log_show(struct seq_file *m, void *v)
{
	return 0;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_open
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int err_log_open(struct inode *inode, struct file *file)
{
    int res = seq_open(file, &err_log_ops);
	if (res) 
	{
	    printk("seq_open err log fail\n");
	}
	
	return 0;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_write
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static ssize_t err_log_write(
    struct file *file, 
    const char __user *buf,
    size_t count, 
    loff_t *ppos)
{
    char    c_tmp_buf[20] = {0};
    int     i_cnt = 0;

    for (i_cnt = 0; i_cnt < count; i_cnt++)
    {
        c_tmp_buf[i_cnt] = buf[i_cnt];
    }
    
    c_tmp_buf[count - 1] = '\0';

    print_err_log(c_tmp_buf);
    
    return count;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_release
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int err_log_release(struct inode *inode, struct file *file)
{
	return 0;
}


/*-----------------------------------------------------------------------------
 * Name             - err_log_proc_init
 * Description      - 
 * Input arguments  -
 * Output arguments -
 * Returns          - 
 *                    
 *---------------------------------------------------------------------------*/
static int __init err_log_proc_init(void)
{
	int ret = 0;
	proc_create("err_log", S_IRUSR | S_IWUSR, NULL, &proc_err_log_operations);


	return ret;
}
__initcall(err_log_proc_init);



