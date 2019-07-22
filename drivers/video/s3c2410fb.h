/*
 * linux/drivers/video/s3c2410fb.h
 *	Copyright (c) 2004 Arnaud Patard
 *
 *  S3C2410 LCD Framebuffer Driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
*/

#ifndef __S3C2410FB_H
#define __S3C2410FB_H

enum s3c_drv_type {
	DRV_S3C2410,
	DRV_S3C2412,
};

struct s3c2410fb_info {
	struct device		*dev;    //设备  
	struct clk		*clk;        //时钟  

	struct resource		*mem;    //资源  
	void __iomem		*io;     //IO地址  
	void __iomem		*irq_base;  //IRQ基数  

	enum s3c_drv_type	drv_type;   //驱动类型，S3C2410或S3C2412
	struct s3c2410fb_hw	regs;       //s3c2410 lcd硬件寄存器  

	unsigned long		clk_rate;
	unsigned int		palette_ready;   //调色板是否就绪标志位

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif

	/* keep these registers in case we need to re-write palette */
	u32			palette_buffer[256];    //调色板缓冲区  
	u32			pseudo_pal[16];         //伪颜色表  
};

#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */

int s3c2410fb_init(void);

#endif
