/*
 * board.h
 *
 *  Created on: Mar 5, 2012
 *      Author: mtk40334
 */

#ifndef BOARD_H_
#define BOARD_H_

struct ac83xx_nand_platform {
	/* NAND chip information */
	unsigned short		page_size;
	unsigned short		data_width;

	/* RD/WR strobe delay timing information, all times in SCLK cycles */
	unsigned short		rd_dly;
	unsigned short		wr_dly;

	/* NAND MTD partition information */
	int                     nr_partitions;
	struct mtd_partition    *partitions;
};


//================================================================================

typedef void (*sdio_irq_handler_t)(void*);  /* external irq handler */
typedef void (*pm_callback_t)(pm_message_t state, void *data);

#define MSDC_CD_PIN_EN      (1 << 0)  /* card detection pin is wired   */
#define MSDC_WP_PIN_EN      (1 << 1)  /* write protection pin is wired */
#define MSDC_RST_PIN_EN     (1 << 2)  /* emmc reset pin is wired       */
#define MSDC_SDIO_IRQ       (1 << 3)  /* use internal sdio irq (bus)   */
#define MSDC_EXT_SDIO_IRQ   (1 << 4)  /* use external sdio irq         */
#define MSDC_REMOVABLE      (1 << 5)  /* removable slot                */
#define MSDC_SYS_SUSPEND    (1 << 6)  /* suspended by system           */
#define MSDC_HIGHSPEED      (1 << 7)  /* high-speed mode support       */
#define MSDC_UHS1           (1 << 8)  /* uhs-1 mode support            */
#define MSDC_DDR            (1 << 9)  /* ddr mode support              */
#define MSDC_INTERNAL_CLK   (1 << 11)  /* Force Internal clock */
#define MSDC_GPIO_EINT_CD   (1 << 12)  /* Use GPIO as eint for card detect */
#define MSDC_EMMC_HS200     (1 << 13)  /* eMMC HS200 mode support            */
#define MSDC_SD_NEED_POWER  (1 << 31) /* for Yecon board, need SD power always on!! or cannot recognize the sd card*/

#define MSDC_SMPL_RISING    (0)
#define MSDC_SMPL_FALLING   (1)

#define MSDC_CMD_PIN        (0)
#define MSDC_DAT_PIN        (1)
#define MSDC_CD_PIN         (2)
#define MSDC_WP_PIN         (3)
#define MSDC_RST_PIN        (4)

#define MSDC_DATA1_INT      1

enum {
    MSDC_CLKSRC_200MHZ = 0,
	MSDC_CLKSRC_196MHZ = 1,
	MSDC_CLKSRC_189MHZ = 2,
	MSDC_CLKSRC_162MHZ = 3,
	MSDC_CLKSRC_147MHZ = 4,
	MSDC_CLKSRC_135MHZ = 5,
	MSDC_CLKSRC_108MHZ = 6,
	MSDC_CLKSRC_27MHZ = 7,
    MSDC_CLKSRC_AUTO = 8,
};

#define MSDC_BOOT_EN (1)
#define MSDC_CD_HIGH (1)
#define MSDC_CD_LOW  (0)

enum {
    MSDC_EMMC = 0,
    MSDC_SD   = 1,
    MSDC_SDIO = 2
};

#define MSDC_RESISTOR_10K			(0)
#define MSDC_RESISTOR_50K			(1)

#define TUNE_EACH_DAT_LINE			(0)		// Each data line has its own delay
#define TUNE_ALL_DAT_LINE			(1)		// All data lines use same delay

#define MSDC_CD_METHOD_NONE					(0)
#define MSDC_CD_METHOD_INTERRUPT			(1)
#define MSDC_CD_METHOD_POLLING				(2)
#define MSDC_CD_METHOD_BOTH					(3)


struct msdc_hw {
    unsigned char  clk_src;          /* host clock source */
    unsigned int   clk_max;         /* host max clock */
    unsigned char  cmd_edge;         /* command latch edge */
    unsigned char  rdata_edge;        /* read data latch edge */
	unsigned char  wdata_edge;        /* write data latch edge */
    unsigned char  clk_drv;          /* clock pad driving */
    unsigned char  cmd_drv;          /* command pad driving */
    unsigned char  dat_drv;          /* data pad driving */
	unsigned char  slew_rate;
	unsigned char  clk_resistor;
	unsigned char  cmd_resistor;
	unsigned char  dat_resistor;
	unsigned char  clk_drv_sd_18;          /* clock pad driving for SD card at 1.8v sdr104 mode */
    unsigned char  cmd_drv_sd_18;          /* command pad driving for SD card at 1.8v sdr104 mode */
    unsigned char  dat_drv_sd_18;          /* data pad driving for SD card at 1.8v sdr104 mode */
	unsigned char  clk_drv_sd_18_sdr50;    /* clock pad driving for SD card at 1.8v sdr50 mode */
    unsigned char  cmd_drv_sd_18_sdr50;    /* command pad driving for SD card at 1.8v sdr50 mode */
    unsigned char  dat_drv_sd_18_sdr50;    /* data pad driving for SD card at 1.8v sdr50 mode */
	unsigned char  clk_drv_sd_18_ddr50;    /* clock pad driving for SD card at 1.8v ddr50 mode */
    unsigned char  cmd_drv_sd_18_ddr50;    /* command pad driving for SD card at 1.8v ddr50 mode */
    unsigned char  dat_drv_sd_18_ddr50;    /* data pad driving for SD card at 1.8v ddr50 mode */
    unsigned long  flags;            /* hardware capability flags */
    unsigned long  data_pins;        /* data pins */
    unsigned long  data_offset;      /* data address offset */
	unsigned long  tuning_method;    /* tuning method select */
	
    unsigned char  ddlsel;    // data line delay line fine tune selecion
	unsigned char  rdsplsel;  // read: data line rising or falling latch fine tune selection 
	unsigned char  wdsplsel;  // write: data line rising or falling latch fine tune selection

	unsigned char  dat0rddly; //read; range: 0~31
	unsigned char  dat1rddly; //read; range: 0~31
	unsigned char  dat2rddly; //read; range: 0~31
	unsigned char  dat3rddly; //read; range: 0~31
	unsigned char  dat4rddly; //read; range: 0~31
	unsigned char  dat5rddly; //read; range: 0~31
	unsigned char  dat6rddly; //read; range: 0~31
	unsigned char  dat7rddly; //read; range: 0~31
	
	unsigned char  datwrddly; //write; range: 0~31
	unsigned char  cmdrrddly; //cmd; range: 0~31
	unsigned char  cmdrddly; //cmd; range: 0~31
	
	unsigned int   wrdat_crcs_ta_cntr;
	unsigned int   cmd_rsp_ta_cntr;

	unsigned int   read_pre_setting_en;
	unsigned int   read_dat_latch_ck_sel;
	unsigned int   read_ckgen_delay_sel;
	unsigned int   read_sample_edge;
	unsigned int   read_pad_delay;
	unsigned int   ddr_read_dat_latch_ck_sel;
	unsigned int   ddr_read_ckgen_delay_sel;
	unsigned int   read_patch_bit1_wrdat_crcs;
	unsigned int   read_patch_bit1_cmdrsp_ta;
	unsigned int   read_rxdly0;
	unsigned int   read_rxdly1;

	unsigned int   write_pre_setting_en;
	unsigned int   write_dat_latch_ck_sel;
	unsigned int   write_ckgen_delay_sel;
	unsigned int   write_sample_edge;
	unsigned int   write_pad_delay;
	unsigned int   write_internal_delay;
	unsigned int   ddr_write_dat_latch_ck_sel;
	unsigned int   ddr_write_ckgen_delay_sel;
	unsigned int   write_patch_bit1_wrdat_crcs;
	unsigned int   write_patch_bit1_cmdrsp_ta;
	unsigned int   write_rxdly0;

	unsigned long  host_function;	 /* define host function*/
	bool		   boot;			 /* define boot host*/ 
	bool		   cd_level;		 /* card detection level*/
	unsigned int   cd_method;		/* card detect method: 0- Interrupt mode, 1- GPIO polling mode, 2- Interrupt with GPIO together*/
    /* config gpio pull mode */
    //void (*config_gpio_pin)(int type, int pull);

    /* external power control for card */
    void (*ext_power_on)(void);
    void (*ext_power_off)(void);

    /* external sdio irq operations */
    void (*request_sdio_eirq)(sdio_irq_handler_t sdio_irq_handler, void *data);
    void (*enable_sdio_eirq)(void);
    void (*disable_sdio_eirq)(void);

    /* external cd irq operations */
    void (*request_cd_eirq)(sdio_irq_handler_t cd_irq_handler, void *data);
    void (*enable_cd_eirq)(void);
    void (*disable_cd_eirq)(void);
    int  (*get_cd_status)(void);
    
	void (*request_emmc_wp_eirq)(irq_handler_t *irq_handler, void *data, unsigned int *level,unsigned int *gpio);
    /* power management callback for external module */
    void (*register_pm)(pm_callback_t pm_cb, void *data);
};

extern struct msdc_hw msdc0_hw;
extern struct msdc_hw msdc1_hw;
extern struct msdc_hw msdc2_hw;
#endif /* BOARD_H_ */
