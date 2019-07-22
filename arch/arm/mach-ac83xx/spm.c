#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <mach/ac83xx_system.h>
#include <asm/barrier.h>
#define SPM_READ32(REG)            __raw_readl(__io(SPM_BASE_VA + REG))
#define SPM_WRITE32(VAL, REG)      __raw_writel(VAL, __io(SPM_BASE_VA + REG))
#define VDEC_RDREG_TIMES 0x5000
static DEFINE_SPINLOCK(spm_lock);


//mali init & uninit
static void spm_clear_onebit(uint32_t reg_offset, uint32_t bit_idx)
{
	uint32_t val;
	
	val = SPM_READ32(reg_offset);
	val &= 0x0000FFFF;
	val &= (~(1 << bit_idx));
	val |= (0x0286 << 16);
	SPM_WRITE32(val, reg_offset);
}

static void spm_set_onebit(uint32_t reg_offset, uint32_t bit_idx)
{
	uint32_t val;
	
	val = SPM_READ32(reg_offset);
	val &= 0x0000FFFF;
	val |= (1 << bit_idx);
	val |= (0x0286 << 16);
	SPM_WRITE32(val, reg_offset);
}

static void spm_clear(uint32_t reg_offset, uint32_t width)
{
	uint32_t i;
	
	for (i = 0; i < width; i++) 
        {
		spm_clear_onebit(reg_offset, i);
	}
}

static void spm_set(uint32_t reg_offset, uint32_t width)
{
	uint32_t i;
	
	for (i = 0; i < width; i++) 
        {
		spm_set_onebit(reg_offset, i);
	}
}
void spm_power_mali_all(uint32_t sw_hw_sel, uint32_t on_off)
{
	uint32_t val =0 , value =0;
	
	if (on_off) 
        { //power on
	
		if (sw_hw_sel) 
                { //power on with hardware control
			
			/* enable spm clock */
		//	SPM_WRITE32(0x02860001, 0);
			
			/* write mfg_hier_pwr_on(bit2) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 2);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack(bit22) == 1 */
			while (!(SPM_READ32(0x7C) & (1 << 22)));
	
			udelay(1);
			
			/* write mfg_hier_pwr_on_s(bit3) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 3);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack_s(bit21) == 1 */
			while (!(SPM_READ32(0x7C) & (1 << 21)));
	
			/* write mfg_hier_clock_dis(bit4) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 4));	
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_mem_pd_sel(bit6) = 1 
			 * write mfg_hier_mem_pd_hw(bit5) = 0 
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 5));
			val |= (1 << 6);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* wait mfg_sram_pwr_down_ack(bit20) = 0 */
			while ((SPM_READ32(0x7C) & (1 << 20)));
			
			/* write mfg_pp1_hier_mem_pd_sel(bit14) = 1
			 * write mfg_pp1_hier_mem_pd_hw(bit13) = 0
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 13));
			val |= (1 << 14);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait pp1_sram_power_down_ack(bit16) == 0 */
			while ((SPM_READ32(0x7C) & (1 << 16)));
				
			/* write mfg_hier_pwr_iso(bit1) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 1));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_pwr_rst_(bit0) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 0);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			//SPM_WRITE32(0x02860000, 0);
			
		} 
                else 
                { //power on with software control
			
			/* enable spm clock */
			//SPM_WRITE32(0x02860001, 0);
			
			/* write mfg_hier_pwr_on(bit2) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 2);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack(bit22) == 1 */
			while (!(SPM_READ32(0x7C) & (1 << 22)));
	
			udelay(1);
			
			/* write mfg_hier_pwr_on_s(bit3) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 3);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack_s(bit21) == 1 */
			while (!(SPM_READ32(0x7C) & (1 << 21)));
	
			/* write mfg_hier_clock_dis(bit4) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 4));	
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_mem_pd_sel(bit6) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 6));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* write mfg_hier_sft_sram_pd_l = 0x0000
			 * write mfg_hier_sft_sram_pd_h(7bits) = 0x00
			 * there are 23bits.
			 */
			spm_clear(0x24, 16);
			spm_clear(0x28, 7);
			
			/* wait mfg_hier_sram_pd = 0x00000000 */
			while (SPM_READ32(0x98) != 0x0);
			
			/* write mfg_pp1_hier_mem_pd_sel(bit14) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 14));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* write mfg_pp1_hier_sft_sram_pd_l(9bits) = 0000 */
			spm_clear(0x2C, 9);
			
			/* wait mfg_hier_pp1_sram_pd == 0(9bits) */
			while (SPM_READ32(0x9C) != 0);
			
			/* write mfg_hier_pwr_iso(bit1) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 1));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_pwr_rst_(bit0) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 0);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			//SPM_WRITE32(0x02860000, 0);			
		}
	
	} 
        else 
	{ //power off
		
		if (sw_hw_sel) 
                 { //power off with hardware control
			
			/* enable spm clock */
			//SPM_WRITE32(0x02860001, 0);
			
			/* write mfg_hier_mem_pd_hw(bit5) = 1 
			 * write mfg_hier_mem_pd_sel(bit6) = 1
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 5);
			val |= (1 << 6);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* wait mfg_sram_pwr_down_ack(bit20) = 1 */
			while (!(SPM_READ32(0x7C) & (1 << 20)));
			
			/* write mfg_pp1_hier_mem_pd_sel(bit14) = 1
			 * write mfg_pp1_hier_mem_pd_hw(bit13) = 1
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 13);
			val |= (1 << 14);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait pp1_sram_power_down_ack(bit16) == 1 */
			while (!(SPM_READ32(0x7C) & (1 << 16)));
			
			/* write mfg_hier_pwr_iso(bit1) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 1);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_clock_dis(bit4) = 1
			 * write mfg_hier_pwr_rst_(bit0) = 0 
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 0));
			val |= (1 << 4);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_pwr_on(bit2) = 0 
			 * write mfg_hier_pwr_on_s(bit3) = 0
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 2));
			val &= (~(1 << 3));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack(bit22) == 0 */
			while ((SPM_READ32(0x7C) & (1 << 22)));
	
			/* wait mfg_pwr_on_ack_s(bit21) == 0 */
			while ((SPM_READ32(0x7C) & (1 << 21)));
						
			//SPM_WRITE32(0x02860000, 0);
			
		} 
                else 
                { //power off with software control
			
			/* enable spm clock */
			//SPM_WRITE32(0x02860001, 0);
			
			/* 
			 * write mfg_hier_mem_pd_sel(bit6) = 0
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 6));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* write mfg_hier_sft_sram_pd_l = 0xFFFF
			 * write mfg_hier_sft_sram_pd_h(7bits) = 0x7F
			 * there are 23bits.
			 */
			spm_set(0x24, 16);
			spm_set(0x28, 7);
			
			/* wait mfg_hier_sram_pd = 0x007FFFFF */
			while (SPM_READ32(0x98) != 0x007FFFFF);
			
			/* write mfg_pp1_hier_mem_pd_sel(bit14) = 0 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 14));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			
			/* write mfg_pp1_hier_sft_sram_pd_l(9bits) = 01FF*/
			spm_set(0x2C, 9);
			
			/* wait mfg_hier_pp1_sram_pd == 1FF(9bits) */
			while (SPM_READ32(0x9C) != 0x01FF);
			
			
			/* write mfg_hier_pwr_iso(bit1) = 1 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val |= (1 << 1);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_clock_dis(bit4) = 1
			 * write mfg_hier_pwr_rst_(bit0) = 0 
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 0));
			val |= (1 << 4);
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
	
			/* write mfg_hier_pwr_on(bit2) = 0 
			 * write mfg_hier_pwr_on_s(bit3) = 0
			 */
			val = SPM_READ32(0x08);
			val &= 0x0000FFFF;
			val &= (~(1 << 2));
			val &= (~(1 << 3));
			val |= (0x0286 << 16);
			SPM_WRITE32(val, 0x08);
			/* wait mfg_pwr_on_ack(bit22) == 0 */
			while ((SPM_READ32(0x7C) & (1 << 22)));
	
			/* wait mfg_pwr_on_ack_s(bit21) == 0 */
			while ((SPM_READ32(0x7C) & (1 << 21)));
						
			//SPM_WRITE32(0x02860000, 0);			
		}	
	}
}

void spm_poweron_vdec(void)
{
    uint32_t val = 0, cnt = 0;
    // power on with software control
    /* 
         * write vdec_hier_mem_pd_sel(bit14) = 0 
         */
    val = SPM_READ32(0x04);
    val &= 0x0000FFFF;
    val &= (~(1 << 14));
    val |= (0x0286 << 16);
    SPM_WRITE32(val, 0x04);

    /* 
       * write vdec_hier_sft_sram_pd_l = 0x0000
       * write vdec_hier_sft_sram_pd_h = 0x0000
       */
    spm_clear(0x1C, 16);
    spm_clear(0x20, 16);

    // wait vdec_hier_sram_pd == 0
    while (SPM_READ32(0x94) != 0x0)
    {
        cnt++;
        if (cnt > VDEC_RDREG_TIMES)
        {
          printk("++spm_poweron_vdec failed after try morn than %d times\n", VDEC_RDREG_TIMES);
            break;
        }
    }

}

void spm_powerdown_vdec(void)
{
    uint32_t val = 0, cnt = 0;
    // power off with software control
            
    /*
         * write vdec_hier_mem_pd_sel(bit14) = 0
         */
    val = SPM_READ32(0x04);
    val &= 0x0000FFFF;
    val &= (~(1 << 14));
    val |= (0x0286 << 16);
    SPM_WRITE32(val, 0x04);
            
    /*
         * write vdec_hier_sft_sram_pd_l = 0xFFFF
         * write vdec_hier_sft_sram_pd_h = 0xFFFF
         */
    spm_set(0x1C, 16);
    spm_set(0x20, 16);
    
    /* wait vdec_hier_sram_pd == 0xFFFFFFFF */
    while (SPM_READ32(0x94) != 0xFFFFFFFF)
    {
        cnt++;
        if (cnt > VDEC_RDREG_TIMES)
        {
          printk("++spm_powerdown_vdec failed after try morn than %d times\n", VDEC_RDREG_TIMES);
            break;
        }
    }
    
}

static int ac83xx_spm_poweroff(unsigned int module)
{
    uint32_t rval;

    if (module == SPM_MODULE_CPU1) {
        /*
         * power down core1 sram with hardware control
         */
        /* write core1_mem_pd_sel =1  and core1_mem_pd_hw = 1 */
        rval = SPM_READ32(0x0C);
	rmb();
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 6) | (1 << 5));
        SPM_WRITE32(rval, 0x0C);
	wmb();
        /* wait core1 sram power down ack == 1*/
        while (!(SPM_READ32(0x7C) & (1 << 12)));

        /*
         * power down core1
         */
        /* write core1_pwr_iso = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= ((1 << 1));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
        wmb();
        /* write core1_pwr_rst_ = 0 and core1_clock_dis = 1 */
        rval = SPM_READ32(0x0C);
	rmb();
        rval &= 0x0000FFFF;
        rval &= (~0x01);
        rval |= ((0x0286 << 16) | (1 << 4));
        SPM_WRITE32(rval, 0x0C);
        wmb();
        /* write core1_pwr_on = 0 and core1_pwr_on_s = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~0xC);
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
	wmb();
        /* wait core1 power down ack == 0 */
        while ((SPM_READ32(0x7C) & (1 << 14)));
        /* wait core1 power down ack_s == 0 */
        while ((SPM_READ32(0x7C) & (1 << 13)));
    } else if (module == SPM_MODULE_CPU2) {
        /*
         * power down core2 sram with hardware control
         */
        /* write core2_mem_pd_sel =1  and core2_mem_pd_hw = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 14) | (1 << 13));
        SPM_WRITE32(rval, 0x0C);
        /* wait core2 sram power down ack == 1*/
        while (!(SPM_READ32(0x7C) & (1 << 8)));

        /*
         * power down core2
         */
        /* write core2_pwr_iso = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= ((1 << 9));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);

        /* write core2_pwr_rst_ = 0 and core2_clock_dis = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 8));
        rval |= ((0x0286 << 16) | (1 << 12));
        SPM_WRITE32(rval, 0x0C);

        /* write core2_pwr_on = 0 and core2_pwr_on_s = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(3 << 10));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
        /* wait core2 power down ack == 0 */
        while ((SPM_READ32(0x7C) & (1 << 10)));
        /* wait core2 power down ack_s == 0 */
        while ((SPM_READ32(0x7C) & (1 << 9)));
    } else if (module == SPM_MODULE_CPU3) {
        /*
         * power down core3 sram with hardware control
         */
        /* write core3_mem_pd_sel =1  and core3_mem_pd_hw = 1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 6) | (1 << 5));
        SPM_WRITE32(rval, 0x10);
        /* wait core3 sram power down ack == 1*/
        while (!(SPM_READ32(0x7C) & (1 << 4)));

        /*
         * power down core3
         */
        /* write core3_pwr_iso = 1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval |= ((1 << 1));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);

        /* write core3_pwr_rst_ = 0 and core3_clock_dis = 1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval &= (~0x01);
        rval |= ((0x0286 << 16) | (1 << 4));
        SPM_WRITE32(rval, 0x10);

        /* write core3_pwr_on = 0 and core3_pwr_on_s = 0 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval &= (~0xC);
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);
        /* wait core3 power down ack == 0 */
        while ((SPM_READ32(0x7C) & (1 << 6)));
        /* wait core3 power down ack_s == 0 */
        while ((SPM_READ32(0x7C) & (1 << 5)));
    }else if(module == SPM_MODULE_G3D){

        spm_power_mali_all(1,0);

    }else if (module == SPM_MODULE_VDEC){
        spm_powerdown_vdec();
       

    }

//    SPM_WRITE32(0x02860000, 0);

    return 1;
}

static void ac83xx_spm_powerup(unsigned int module)
{
    volatile uint32_t delayn = 5000;
    uint32_t rval = 0;

    if (module == SPM_MODULE_CPU1) {
        /* write core1_pwr_on =1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 2));
        SPM_WRITE32(rval, 0x0C);
        /* wait core1_pwr_on_ack == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 14)));

        /* delat at least 1us */
	 udelay(1);
        /* write core1_pwr_on_s =1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 3));
        SPM_WRITE32(rval, 0x0C);
        /* wait core1_pwr_on_ack_s == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 13)));

        /* write core1_clock_dis =0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 4));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);

        /* write core1_mem_pd_hw = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 5));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
        /* wait core1_sram_pwr_down_ack == 0 */
        while((SPM_READ32(0x7C) & (1 << 12)));

        /* write core1_pwr_iso = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 1));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);

        /* write core1_pwr_rst_ = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= (1 << 0);
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
    }else if (module == SPM_MODULE_CPU2) {
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        /* write core2_pwr_on =1 */
        rval |= ((0x0286 << 16) | (1 << 10));
        SPM_WRITE32(rval, 0x0C);
        /* wait core2_pwr_on_ack == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 10)));

        /* delat at least 1us */
        //while (delayn--);
        mdelay(10);
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        /* write core2_pwr_on_s =1 */
        rval |= ((0x0286 << 16) | (1 << 11));
        SPM_WRITE32(rval, 0x0C);
        /* wait core2_pwr_on_ack_s == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 9)));

        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        /* write core2_clock_dis =0 */
        rval &= (~(1 << 12));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);

        /* write core2_mem_pd_hw = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 13));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
        /* wait core2_sram_pwr_down_ack == 0 */
        while((SPM_READ32(0x7C) & (1 << 8)));

        /* write core2_pwr_iso = 0 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 9));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);

        /* write core2_pwr_rst_ = 1 */
        rval = SPM_READ32(0x0C);
        rval &= 0x0000FFFF;
        rval |= (1 << 8);
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x0C);
    } else if (module == SPM_MODULE_CPU3) {
        /* write core3_pwr_on =1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 2));
        SPM_WRITE32(rval, 0x10);
        /* wait core3_pwr_on_ack == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 6)));

        /* delat at least 1us */
        //while (delayn--);
        mdelay(10);
        /* write core3_pwr_on_s =1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval |= ((0x0286 << 16) | (1 << 3));
        SPM_WRITE32(rval, 0x10);
        /* wait core3_pwr_on_ack_s == 1 */
        while(!(SPM_READ32(0x7C) & (1 << 5)));

        /* write core3_clock_dis =0 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 4));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);

        /* write core3_mem_pd_hw = 0 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 5));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);
        /* wait core3_sram_pwr_down_ack == 0 */
        while((SPM_READ32(0x7C) & (1 << 4)));

        /* write core3_pwr_iso = 0 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval &= (~(1 << 1));
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);

        /* write core3_pwr_rst_ = 1 */
        rval = SPM_READ32(0x10);
        rval &= 0x0000FFFF;
        rval |= (1 << 0);
        rval |= (0x0286 << 16);
        SPM_WRITE32(rval, 0x10);
    }else if(module == SPM_MODULE_G3D){
        spm_power_mali_all(1,1);
    }else if(module == SPM_MODULE_VDEC){

           spm_poweron_vdec();

    }
}
int spm_set_power(int module,bool pwron)
{
  unsigned long flags = 0; 
  char buf[512]; 
  uint32_t rval;


spin_lock_irqsave(&spm_lock,flags);  
  SPM_WRITE32(0x02860001, 0);
   if(pwron){
       
      ac83xx_spm_powerup(module);    

   }else{
 
      ac83xx_spm_poweroff(module);

   }
   SPM_WRITE32(0x02860000, 0);
 spin_unlock_irqrestore(&spm_lock,flags);

   
   rval = __raw_readl(__io(BIM_BASE_VA + 0x118));
   
   sprintf(buf,"0x118:0x%x\n",rval);
   printk("0x118:0x%x\n",rval);

   return 0;  

}

EXPORT_SYMBOL(spm_set_power);
