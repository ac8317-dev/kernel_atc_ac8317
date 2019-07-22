/*
 *  linux/arch/arm/arch-mt85xx/mt85xx_system.c
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */

#include <mach/hardware.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
#include <mach/BL_VERSION_33xx.h>
#endif

#include <mach/mt33xx_system.h>

extern void free_mt85xx_reserve_mem(void);

void arch_reset(char mode, const char *cmd)
{
	/*
	 * use powerdown watch dog to reset system
	 */
	uint32_t u4Test;

	//printk("MTK Reboot is working now.\n");

	PDWNC_WRITE32(REG_RW_RESRV1, STDBY_CHECK_VALUE);

	PDWNC_WRITE32(REG_RW_WDT, 0xff000000);

	for(u4Test = 0; u4Test < 10000; u4Test++)
	{

	}
	PDWNC_WRITE32(REG_RW_WDTSET, 1);

}

EXPORT_SYMBOL(arch_reset);

static void arch_pwr_off_engine_set(uint32_t fEngineID, uint32_t fMaster)
{
    uint32_t u4Tmp;

    BIM_GETHWSemaphore(HSMPHE_PWR_UP_CFG, 0);

    u4Tmp = PDWNC_READ32(REG_RW_UP_CFG);
    switch(fEngineID) {
        case ENGINE_ALL:
            PDWNC_WRITE32(REG_RW_UP_CFG, (fMaster<<RW_UP_CFG_ENGINE_EN) | (u4Tmp & (~(1<<RW_UP_CFG_ENGINE_EN))));
            break;

        case ENGINE_ETHE:
            PDWNC_WRITE32(REG_RW_UP_CFG, (fMaster<<RW_UP_CFG_ETNET_EN) | (u4Tmp & (~(1<<RW_UP_CFG_ETNET_EN))));
            break;

        case ENGINE_UART:
            PDWNC_WRITE32(REG_RW_UP_CFG, (fMaster<<RW_UP_CFG_DBG_UART_EN) | (u4Tmp & (~(1<<RW_UP_CFG_DBG_UART_EN))));
            break;

        case ENGINE_CEC:
            PDWNC_WRITE32(REG_RW_UP_CFG, (fMaster<<RW_UP_CFG_CEC_EN) | (u4Tmp & (~(1<<RW_UP_CFG_CEC_EN))));
            break;

        default:
            break;
    }

    BIM_ReleaseHWSemaphore(HSMPHE_PWR_UP_CFG);
}

static void arch_pwr_off_set_ethernet_pad_by_pwdn(void)
{
}	

void arch_pwr_Up(void)
{
	/*
	 *  Enable arm interrupt
	 */
	uint32_t u4Tmp1;

  BIM_GETHWSemaphore(HSMPHE_PWR_UP_CFG, 0);

	u4Tmp1 = PDWNC_READ32(REG_RW_UP_CFG);
	//printk(KERN_DEBUG"U4TMP = %x\n", u4Tmp1);
	//PDWNC_WRITE32(REG_RW_UP_CFG, RW_UP_CFG_CK_SEL);
	PDWNC_WRITE32(REG_RW_UP_CFG, u4Tmp1 | 0x45);

  BIM_ReleaseHWSemaphore(HSMPHE_PWR_UP_CFG);

	PDWNC_WRITE32(REG_RW_INTEN, 0x0);
	PDWNC_WRITE32(REG_RW_INTCLR, 0xFFFF);
	PDWNC_WRITE32(REG_RW_INTSTA, 0xFFFF);
	PDWNC_WRITE32(REG_RW_ARM_IND_INT, RW_ARM_IND_INT_AUXINT_CLR);

	u4Tmp1 = BIM_READ32(REG_IRQST2);
	printk(KERN_DEBUG"U4TMP for clear register= %x\n", u4Tmp1);

	BIM_WRITE32(REG_IRQCL2, u4Tmp1 |RW_CLR_CFG_INT_RST);	// Clear mcu interrupt
	BIM_WRITE32(REG_IRQST2, u4Tmp1 |RW_CLR_CFG_INT_RST);

	//u4Tmp1 = BIM_READ32(REG_IRQST2);
	//printk(KERN_DEBUG"U4TMP for clear register= %x\n", u4Tmp1);

	PDWNC_WRITE32(REG_RW_WAKEN, 0x000);   // Disable All way wake up

}

EXPORT_SYMBOL(arch_pwr_Up);

static void arch_pwr_off_mcu_reset(void)
{
	/*
	 *  reset mcu before off
	 */

	uint32_t u4Tmp1;

  BIM_GETHWSemaphore(HSMPHE_PWR_UP_CFG, 0);

	u4Tmp1 = PDWNC_READ32(REG_RW_UP_CFG);
	PDWNC_WRITE32(REG_RW_UP_CFG, u4Tmp1 | RW_UP_CFG_T8032_RST);

	u4Tmp1 = PDWNC_READ32(REG_RW_UP_CFG);
	PDWNC_WRITE32(REG_RW_UP_CFG, (SRAM_MASTER_T8032<<RW_UP_CFG_RAM_CK_SEL_OFFSET) | (u4Tmp1 & (~RW_UP_CFG_RAM_CK_SEL_MASK)) );

	u4Tmp1 = PDWNC_READ32(REG_RW_UP_CFG);
	PDWNC_WRITE32(REG_RW_UP_CFG, u4Tmp1 & (~RW_UP_CFG_T8032_RST));

  BIM_ReleaseHWSemaphore(HSMPHE_PWR_UP_CFG);
}

void arch_pwr_off(void)
{
   // u32 i;
	/* Use MCU to turn off the whole electricity */
	//printk("MTK pwr off is needed start!\n");

	PDWNC_WRITE32(REG_RW_WAKEN, 0xD00);

	arch_pwr_off_engine_set(ENGINE_ALL, ENGINE_MASTER_T8032);
	arch_pwr_off_set_ethernet_pad_by_pwdn();
	arch_pwr_off_engine_set(ENGINE_ETHE, ENGINE_MASTER_T8032);
	arch_pwr_off_engine_set(ENGINE_UART, ENGINE_MASTER_T8032);
	arch_pwr_off_engine_set(ENGINE_CEC, ENGINE_MASTER_T8032);

	arch_pwr_off_mcu_reset();

	PDWNC_WRITE32(REG_RW_ARM_IND_INT, RW_ARM_IND_INT_ARM_INT);
	//printk("MTK pwr off is needed end!\n");
}

EXPORT_SYMBOL(arch_pwr_off);

static struct proc_dir_entry *BL_Ver_Proc_File;

int bl_ver_procfile_read(char *buffer,
	      char **buffer_location,
	      off_t offset, int buffer_length, int *eof, void *data)
{
	int ret = 0;

	ret = sprintf(buffer, "%d", BL_VERSION);	// u-boot , fastlogo, preloader
	//printk(KERN_INFO "procfile_read (/proc/%s) called\n", bl_ver_procfs_name);
	return ret;
}

int  mt85xx_bl_ver_proc_init(void)
{
	BL_Ver_Proc_File = create_proc_entry(bl_ver_procfs_name, 0644, NULL);

	if (BL_Ver_Proc_File == NULL) {
		remove_proc_entry(bl_ver_procfs_name, BL_Ver_Proc_File);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
		       bl_ver_procfs_name);
		return -ENOMEM;
	}

	BL_Ver_Proc_File->read_proc = bl_ver_procfile_read;
	//BL_Ver_Proc_File->owner 	 = THIS_MODULE;
	BL_Ver_Proc_File->mode 	 = S_IFREG | S_IRUGO;
	BL_Ver_Proc_File->uid 	 = 0;
	BL_Ver_Proc_File->gid 	 = 0;
	BL_Ver_Proc_File->size 	 = 37;

	printk(KERN_INFO "/proc/%s created\n", bl_ver_procfs_name);
	return 0;	/* everything is ok */
}

EXPORT_SYMBOL(mt85xx_bl_ver_proc_init);

void  mt85xx_bl_ver_proc_exit(void)
{
	remove_proc_entry(bl_ver_procfs_name, NULL);
	printk(KERN_INFO "/proc/%s removed\n", bl_ver_procfs_name);
}

EXPORT_SYMBOL(mt85xx_bl_ver_proc_exit);

void free_mem_fastlogo(void)
{
	unsigned int ret;

	ret = BIM2_READ32(REG_RW_SINFOD_REG);
	if ( ret == FL_STATUS_A2_FINISHED)
	{
		printk("Freeing Memory Fast Logo (TO BE FIXED mtk40505) : %x\n", ret);
		//free_mt85xx_reserve_mem();  //commented by mtk40505
	}
	else
	{
		printk("No Fastlogo \n");
	}
}

EXPORT_SYMBOL(free_mem_fastlogo);

