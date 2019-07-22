/*
 *  linux/arch/arm/mach-ac83xx/ac83xx_isr_watch.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * ac83xx isr watch.
 */

//==================================================
// header files
//==================================================
#include <mach/ac83xx_isr_watch.h>
#include <mach/irqs.h>

//#include <linux/string.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>

#if CONFIG_AC83XX_ISR_WATCH

//==================================================
// Define
//==================================================


//==================================================
// Local variable
//==================================================
static BOOL _fg_isr_watch_on = FALSE;
//static T64b_RAW_TIME_T _rt_start, _rt_end;

//==================================================
// Global variable
//==================================================
AC83XX_ISR_OVERHEAD_T ar_isr_overhead_vector[NR_IRQS];


//==================================================
// Private functions
//==================================================
void _ac83xx_isrw_reset(void)
{
	_fg_isr_watch_on = FALSE;
	memset(ar_isr_overhead_vector, 0, sizeof(ar_isr_overhead_vector));
	_fg_isr_watch_on = TRUE;
}

//==================================================
// public functions
//==================================================
int isrw_show(struct seq_file *p, void *v)
{
	int ret = 0;
	int i = *(loff_t *) v;

	/*header*/
	if (i==0)
	{
		seq_printf(p, "irq,\t count,\ttotal,\t\tmax\n");
	}
	
	/*data*/
	if((i < NR_IRQS) && (ar_isr_overhead_vector[i].u4Count > 0))
	{
		seq_printf(p, "%03d:\t%d,\t %d.%06d,\t %d.%06d\n", 
					i, 
					ar_isr_overhead_vector[i].u4Count,
					ar_isr_overhead_vector[i].rTotalTime.u4Seconds,
					ar_isr_overhead_vector[i].rTotalTime.u4Micros,
					ar_isr_overhead_vector[i].rMaxTime.u4Seconds,
					ar_isr_overhead_vector[i].rMaxTime.u4Micros);
	}

	/*end*/
	if (i == NR_IRQS)
	{
		seq_printf(p, "End.\n");
	}

	return ret;
}

static void *isrw_seq_start(struct seq_file *f, loff_t *pos)
{
	return (*pos <= NR_IRQS) ? pos : NULL;
}
static void *isrw_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos > NR_IRQS)
		return NULL;
	return pos;
}
static void isrw_seq_stop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static const struct seq_operations isrw_seq_ops = {
	.start = isrw_seq_start,
	.next  = isrw_seq_next,
	.stop  = isrw_seq_stop,
	.show  = isrw_show
};

static int isrw_open(struct inode *inode, struct file *filp)
{
	return seq_open(filp, &isrw_seq_ops);
}

#define MAX_CMD_LEN 16
static int isrw_write(struct file * filep, const char __user * data, size_t size, loff_t *pos)
{
	char kbuf[MAX_CMD_LEN];
	
	if(copy_from_user(kbuf, data, MAX_CMD_LEN) != MAX_CMD_LEN)
	{
		// never happen.
	}

	// reset cmd
	if(strncmp("reset", kbuf, 5)==0)
	{
		_ac83xx_isrw_reset();
	}

	// start cmd
	if(strncmp("start", kbuf, 5)==0)
	{
		_fg_isr_watch_on = TRUE;
	}

	// stop cmd
	if(strncmp("stop", kbuf, 4)==0)
	{
		_fg_isr_watch_on = FALSE;
	}
	
	return size;
}

static const struct file_operations proc_isrw_ops = {
	.open		= isrw_open,
	.read		= seq_read,
	.write		= isrw_write,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

void ac83xx_isr_watch_init(void)
{
	_ac83xx_isrw_reset();
}

void ac83xx_isrw_proc_init(void)
{
//	struct proc_dir_entry * entry;
	
	proc_create("isrw", 0666, NULL, &proc_isrw_ops);
//	entry = create_proc_entry("isrw", 0666, NULL);
//	entry->read_proc = isrw_proc_read;
}

void ac83xx_isr_watch_trigger(uint32_t u4Vector, uint32_t event)
{
	T64b_RAW_TIME_T rt_delta;
	HAL_TIME_T      t_delta;
	AC83XX_ISR_OVERHEAD_T *pr_isr_over_head;

	if(_fg_isr_watch_on==FALSE)
		return;

	pr_isr_over_head = &(ar_isr_overhead_vector[u4Vector]);
	
	switch(event)
	{
		case ISRW_EVT_START:
			/* get start raw time */
			if(i_hal_t64b_get_raw_time(&(pr_isr_over_head->_rt_start)) != 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: get start raw time fail.\n");
				return;
			}
			break;
		case ISRW_EVT_END:
			/* get end raw time */
			if(i_hal_t64b_get_raw_time(&(pr_isr_over_head->_rt_end)) != 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: get end raw time fail.\n");
				return;
			}

			/* check if start > end : error */
			if(i_hal_t64b_cmp_raw_time(&(pr_isr_over_head->_rt_start), &(pr_isr_over_head->_rt_end)) > 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: start < end.\n");
				return;
			}

			/* get delta raw time */
			if(i_hal_t64b_get_delta_raw_time(&rt_delta, &(pr_isr_over_head->_rt_start), &(pr_isr_over_head->_rt_end)) != 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: get end time fail.\n");
				return;
			}

			/* get delta time */
			if(i_hal_t64b_raw_to_time(&rt_delta, &t_delta) != 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: raw to time fail.\n");
				return;
			}

			/* set isr overhead data */
			(pr_isr_over_head->u4Count)++;
			if(i_hal_t64b_add_time(	&(pr_isr_over_head->rTotalTime),
									&(pr_isr_over_head->rTotalTime),
									&t_delta) != 0)
			{
				printk(KERN_ERR "[ac83xx_isr_watch_trigger] ERROR: add time fail.\n");
				return;
			}
			if(i_hal_t64b_cmp_time(	&(pr_isr_over_head->rMaxTime),
									&t_delta) < 0)
			{
				pr_isr_over_head->rMaxTime.u4Seconds = t_delta.u4Seconds;
				pr_isr_over_head->rMaxTime.u4Micros  = t_delta.u4Micros;
			}
			
			break;
		default:
			break;
	}
}



#endif // CONFIG_AC83XX_ISR_WATCH

