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
	struct device		*dev;    //�豸  
	struct clk		*clk;        //ʱ��  

	struct resource		*mem;    //��Դ  
	void __iomem		*io;     //IO��ַ  
	void __iomem		*irq_base;  //IRQ����  

	enum s3c_drv_type	drv_type;   //�������ͣ�S3C2410��S3C2412
	struct s3c2410fb_hw	regs;       //s3c2410 lcdӲ���Ĵ���  

	unsigned long		clk_rate;
	unsigned int		palette_ready;   //��ɫ���Ƿ������־λ

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif

	/* keep these registers in case we need to re-write palette */
	u32			palette_buffer[256];    //��ɫ�建����  
	u32			pseudo_pal[16];         //α��ɫ��  
};

#define PALETTE_BUFF_CLEAR (0x80000000)	/* entry is clear/invalid */

int s3c2410fb_init(void);

#endif
