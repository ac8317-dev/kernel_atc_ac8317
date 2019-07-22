#ifndef __MSDC_HW_ETT_H
#define __MSDC_HW_ETT_H

//#include <mach/board.h>


struct msdc_sdio_ett_settings{
	unsigned char clk;
	unsigned char clk_drv;
	unsigned char cmd_drv;
	unsigned char dat_drv;
	unsigned int cmd_rsp_ta_cntr;
	unsigned int int_dat_latch_ck_sel;
	unsigned int wrdat_crcs_ta_cntr;
	unsigned int ckgen_dly_sel;

};

struct msdc_sdio_ett_settings demo_6630_board[]={
		{
		 .clk=MSDC_CLKSRC_108MHZ,
		 .clk_drv=0x2f,
		 .cmd_drv=0x2f,
		 .dat_drv=0x2f,
		 .cmd_rsp_ta_cntr=2,
		 .int_dat_latch_ck_sel=1,
		 .wrdat_crcs_ta_cntr=2,
		 .ckgen_dly_sel=1,
		},
		{
		 .clk=MSDC_CLKSRC_189MHZ,
		 .clk_drv=0x2d,
		 .cmd_drv=0x2d,
		 .dat_drv=0x2d,
		 .cmd_rsp_ta_cntr=4,
		 .int_dat_latch_ck_sel=1,
		 .wrdat_crcs_ta_cntr=3,
		 .ckgen_dly_sel=5,
		},
		{
		 .clk=MSDC_CLKSRC_200MHZ,
		 .clk_drv=0x2f,
		 .cmd_drv=0x2f,
		 .dat_drv=0x2f,
		 .cmd_rsp_ta_cntr=2,
		 .int_dat_latch_ck_sel=1,
		 .wrdat_crcs_ta_cntr=2,
		 .ckgen_dly_sel=1,
		},
};

#endif
