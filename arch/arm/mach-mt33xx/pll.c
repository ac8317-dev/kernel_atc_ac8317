/*
 *  linux/arch/arm/mach-MT85XX/pll.c
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * pll settings.
 */

//==================================================
// header files
//==================================================
#include <mach/chip_ver.h>
#include <linux/module.h>
#include <mach/hardware.h>
#include "pll.h"


//==================================================
// Define
//==================================================
//typedef unsigned long  uint32_t;
#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
typedef enum
{
    e_CLK_RISC,                 //0
    e_CLK_ADSP,                 //1
    e_CLK_ADSP2,                //2
    e_CLK_FLASH,                //3
    e_CLK_NFLASH,               //4
    e_CLK_SACD,                 //5
    e_CLK_RISC2,                //6
    e_CLK_SLOW_RISC,            //7
    e_CLK_VFD,                  //8
    e_CLK_VDEC,                 //9
    e_CLK_MC,                   //10
    e_CLK_OSD,                  //11
    e_CLK_PNG,                  //12
    e_CLK_RESZ,                 //13
    e_CLK_GRAPH,                //14
    e_CLK_NR,                   //15
    e_CLK_DEMUX,                //16
    e_CLK_IR_DIV,               //17
    e_CLK_SD,                   //18
    e_CLK_MS,                   //19
    e_CLK_ABIST2,               //20
    e_CLK_ABIST,                //21
    e_CLK_SVO_STDBY,            //22
    e_CLK_GCPU,                 //23
    e_CLK_MVDO2,                //24
    e_CLK_TSOUT,                //25
    e_CLK_SPI,                  //26
    e_CLK_SIF,                  //27
    e_CLK_VENC_2F,              //28
    e_CLK_BCLK,                 //29
    e_CLK_SPDIFIN,              //30
    e_CLK_MUXED_TS0,            //31
    e_CLK_MUXED_TS1,            //32
    e_CLK_ESIF,                 //33
    e_CLK_MAX                   //34

} e_CLK_T;
#endif // end of #if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)

#define RISC_DRAM_CLOCK_27M             (27000000)   // 27MHz

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
#define REG_RW_ARMPLL_CFG0              0x0004              //ARMPLL Configuration 0 Register
  #define ARMPLL_ARMPLL_DIV1                     (1U << 16) //Feedback divider  0:/RG_ARMPLL_DIV[7:0]+2)  1:/1
  #define ARMPLL_ARMPLL_DIV_MASK                 0x0000FE00
  #define ARMPLL_ARMPLL_DIV_OFFSET               9          // /(RG_ARMPLL_DIV[7:0]+2)
  #define ARMPLL_ARMPLL_PREDIV_MASK              0x00000003
  #define ARMPLL_ARMPLL_PREDIV_OFFSET            0          //00:/1  01:/2  10:/4  11:/8
#define REG_RW_PLL_STA1                 0x002C              //PLL Group Status 1
  #define PLL_STA1_SYSPLL2_VCOCALOK              (1U << 20)
#define REG_RW_SYSPLL1_CFG0             0x0040
  #define SYSPLL1_SYSPLL1_DIV1                   (1U << 8)  //Feedback divider  0:/(SYSPLL_DIV[7:0]+2)  1:/1
  #define SYSPLL1_SYSPLL1_DIV_MASK               0x000000FC
  #define SYSPLL1_SYSPLL1_DIV_OFFSET             2          // /(SYSPLL_DIV[7:0]+2)
  #define SYSPLL1_SYSPLL1_PREDIV_MASK            0x00000003
  #define SYSPLL1_SYSPLL1_PREDIV_OFFSET          0          //00:/1  01:/2  10:/4  11:/8
#define REG_RW_SYSPLL1_CFG1             0x0044
#define REG_RW_SYSPLL2_CFG0             0x0048
  #define SYSPLL2_SYSPLL2_DIV1                   (1U << 8)  //Feedback divider  0:/(SYSPLL_DIV[7:0]+2)  1:/1
  #define SYSPLL2_SYSPLL2_DIV_MASK               0x000000FC
  #define SYSPLL2_SYSPLL2_DIV_OFFSET             2          // /(SYSPLL_DIV[7:0]+2)
  #define SYSPLL2_SYSPLL2_PREDIV_MASK            0x00000003
  #define SYSPLL2_SYSPLL2_PREDIV_OFFSET          0          //00:/1  01:/2  10:/4  11:/8
#define REG_RW_SYSPLL2_CFG1             0x004C
#endif // end of #if (CONFIG_CHIPI_VER_CURR == CONFIG_CHIP_VER_MT3360)

#define REG_RW_TST_CFG0                 0x005C             //Test Configuration 0
  #define TST_CFG0_AXICLK_RATIO_MASK             0x001C0000
  #define TST_CFG0_AXICLK_RATIO_OFFSET           18
  #define TST_CFG0_BCLK_RATIO_MASK               0x00000070
  #define TST_CFG0_BCLK_RATIO_OFFSET             4

  #define CLK_CLK_RISC_SEL_27M                   0
  #define CLK_CLK_RISC_SEL_ARMPLL                1
  #define CLK_CLK_RISC_SEL_SYSPLL1               2
  #define CLK_CLK_RISC_SEL_SYSPLL2               3
  #define CLK_CLK_RISC_SEL_SLOW_RISC_CLK         4
  #define CLK_CLK_RISC_SEL_DMPLL                 5

#define REG_RW_TST_CFG1                 0x0060        //Test Configuration Register 1
  #define TST_CFG1_RISCCLK_RATIO_MASK            0x00001C00  //ARMCK RATIO Mask
  #define TST_CFG1_RISCCLK_RATIO_OFFSET          10        //ARMCK RATIO Offset

#define REG_RW_CLK_CFG0                 0x0070             //Clock Selection Configuration 0
  #define CLK_SLOW_RISC_SEL_MASK                 0x70000000
  #define CLK_SLOW_RISC_SEL_OFFSET               28         //Select divisor used to divide 27MHz clock, for RISC clock slow option
  #define CLK_CLK_RISC_SEL_MASK                  0x00000007
  #define CLK_CLK_RISC_SEL_OFFSET                0          //Selection of RISC clock frequency
  
#define REG_RW_CLK_CFG3                 0x007C             //Clock Selection Configuration 3
  #define CLK_BCLK_SEL_MASK                      0x70000000
  #define CLK_BCLK_SEL_OFFSET                    28
  #define BCLK_Default                           0x00
  #define BCLK_DMPLL                             0x01
  #define BCLK_SYSPLL1_HALF                      0x02
  #define BCLK_SYSPLL2_HALF                      0x03
  #define BCLK_SVOIPLL_HALF                      0x04
  #define BCLK_SATAPLL_HALF                      0x05


//==================================================
// variables
//==================================================
static uint32_t _u4_bus_clock = 0;


//==================================================
// Private functions
//==================================================
static bool _CalAPLL(uint32_t u4Clock)
{
    bool fgRet = true;
    uint32_t u4Delay;

    //Use ACD default value : 294.912Hz, need 0.25ms, 756MHz*0.25ms=198181
    IO_WRITE32(AUD_VIRT, 0x204, 0x300088CC);
    IO_WRITE32(AUD_VIRT, 0x208, 0x0C49BA02);
    IO_WRITE32(AUD_VIRT, 0x20C, 0x0F0F0F39);
    IO_WRITE32(AUD_VIRT, 0x210, 0x00000F0F);
    for (u4Delay = 198181; u4Delay > 0; u4Delay--) {
    }

    return fgRet;
}

static bool _CalSYSPLL1(uint32_t u4Clock)
{
  bool fgRet = true;
    uint32_t u4Delay;

    //Use ACD default value : 432Hz, need 0.625ms, 756MHz*0.625ms=495453
    for (u4Delay = 495453; u4Delay > 0; u4Delay--) {
    }
    return fgRet;
}

static bool _CalSYSPLL2(uint32_t u4Clock)
{
  bool fgRet = true;
  return fgRet;
}

static uint32_t _GetARMPLL(void)
{
    uint32_t u4Clock = 0;
    uint32_t u4Tmp;
    uint32_t u4PREDIV;
    uint32_t u4DIV;

    u4Tmp = CKGEN_READ32(REG_RW_ARMPLL_CFG0);

    u4PREDIV = (u4Tmp & ARMPLL_ARMPLL_PREDIV_MASK)>>ARMPLL_ARMPLL_PREDIV_OFFSET;

    switch(u4PREDIV) {
        case 0:
            u4PREDIV = 1;
            break;
        case 1:
            u4PREDIV = 2;
            break;
        case 2:
            u4PREDIV = 4;
            break;
        case 3:
            u4PREDIV = 8;
            break;
    }

    if (u4Tmp & ARMPLL_ARMPLL_DIV1) {
         u4DIV = 1;
    } else {
        u4DIV = (u4Tmp & ARMPLL_ARMPLL_DIV_MASK)>>ARMPLL_ARMPLL_DIV_OFFSET;
        u4DIV = u4DIV + 2;
    }

    u4Clock = (27000000/u4PREDIV)*(u4DIV);

    return u4Clock;
}

static uint32_t _GetSYSPLL1(void)
{
    uint32_t u4Clock = 0;
    uint32_t u4Tmp;
    uint32_t u4PREDIV;
    uint32_t u4DIV;

    u4Tmp = CKGEN_READ32(REG_RW_SYSPLL1_CFG0);

    u4PREDIV = (u4Tmp & SYSPLL1_SYSPLL1_PREDIV_MASK)>>SYSPLL1_SYSPLL1_PREDIV_OFFSET;

    switch(u4PREDIV) {
        case 0:
            u4PREDIV = 1;
            break;
        case 1:
            u4PREDIV = 2;
            break;
        case 2:
            u4PREDIV = 4;
            break;
        case 3:
            u4PREDIV = 8;
            break;
    }

    if (u4Tmp & SYSPLL1_SYSPLL1_DIV1) {
         u4DIV = 1;
    } else {
        u4DIV = (u4Tmp & SYSPLL1_SYSPLL1_DIV_MASK)>>SYSPLL1_SYSPLL1_DIV_OFFSET;
        u4DIV = u4DIV + 2;
    }

    u4Clock = (27000000/u4PREDIV)*(u4DIV);

    return u4Clock;
}

static uint32_t _GetSYSPLL2(void)
{
    uint32_t u4Clock = 0;
    uint32_t u4Tmp;
    uint32_t u4PREDIV;
    uint32_t u4DIV;

    u4Tmp = CKGEN_READ32(REG_RW_SYSPLL2_CFG0);

    u4PREDIV = (u4Tmp & SYSPLL2_SYSPLL2_PREDIV_MASK)>>SYSPLL2_SYSPLL2_PREDIV_OFFSET;

    switch(u4PREDIV) {
        case 0:
            u4PREDIV = 1;
            break;
        case 1:
            u4PREDIV = 2;
            break;
        case 2:
            u4PREDIV = 4;
            break;
        case 3:
            u4PREDIV = 8;
            break;
    }

    if (u4Tmp & SYSPLL2_SYSPLL2_DIV1) {
        u4DIV = 1;
    } else {
        u4DIV = (u4Tmp & SYSPLL2_SYSPLL2_DIV_MASK)>>SYSPLL2_SYSPLL2_DIV_OFFSET;
        u4DIV = u4DIV + 2;
    }

    u4Clock = (27000000/u4PREDIV)*(u4DIV);

    return u4Clock;
}


uint32_t u4_ckgen_agt_get_clk(e_CLK_T eAgt)
{
    uint32_t u4Tmp;
    uint32_t u4Sel;
    uint32_t ret = 0;

    if (eAgt < e_CLK_VDEC) {    /* CONFG 0: RISC, ADSP, ADSP2, FLASH, NFLASH, SACD, RISC2, SLOW_RISC*/
        u4Tmp = CKGEN_READ32(REG_RW_CLK_CFG0);
        switch (eAgt) {
            case e_CLK_RISC:
                u4Sel = (u4Tmp& CLK_CLK_RISC_SEL_MASK) >> CLK_CLK_RISC_SEL_OFFSET;
                break;
            case e_CLK_SLOW_RISC:
                u4Sel = (u4Tmp& CLK_SLOW_RISC_SEL_MASK) >> CLK_SLOW_RISC_SEL_OFFSET;
                break;
            default:
                ret = 1;
        }
    } else if (eAgt < e_CLK_IR_DIV) {    /* CONFG 1: VDEC, MC,OSD, PNG, RESZ, GRAPH, NR, DEMUX*/
        ret = 1;
    } else if (eAgt < e_CLK_MAX) {    /* CONFG 2: IR, SD, MS, ABIST*/
        ret = 1;
    } else {
        ret = 1;
    }
    if (ret) {
        return 0xFF;
    } else {
        return u4Sel;
    }
}

uint32_t u4_get_risc_clock(void)
{
  uint32_t u4Tmp;
  uint32_t u4Clock = 0;

  uint32_t u4RiscRatio;

  u4Tmp =  u4_ckgen_agt_get_clk(e_CLK_RISC);

  switch(u4Tmp)
  {
    case CLK_CLK_RISC_SEL_27M:
      u4Clock = RISC_DRAM_CLOCK_27M;
      break;

    case CLK_CLK_RISC_SEL_ARMPLL:
      u4Clock =_GetARMPLL();
      break;

    case CLK_CLK_RISC_SEL_SYSPLL1:
      u4Clock = _GetSYSPLL1();
      break;

    case CLK_CLK_RISC_SEL_SYSPLL2:
      u4Clock = _GetSYSPLL2();
      break;

    case CLK_CLK_RISC_SEL_SLOW_RISC_CLK:
      u4Tmp = u4_ckgen_agt_get_clk(e_CLK_SLOW_RISC);
      u4Clock = (RISC_DRAM_CLOCK_27M >> 4)/(u4Tmp+1);    // SLOW_RISC_SEL  27MHz/divisor
      break;
  }

  u4RiscRatio = CKGEN_READ32(REG_RW_TST_CFG1);

  u4RiscRatio = u4RiscRatio & TST_CFG1_RISCCLK_RATIO_MASK;

  u4RiscRatio = u4RiscRatio >> TST_CFG1_RISCCLK_RATIO_OFFSET;

  u4RiscRatio = u4RiscRatio + 1;

  u4Clock = u4Clock/u4RiscRatio;

//    ASSERT(u4Clock!=0);
  // error
  if(u4Clock==0)
  {
    printk(KERN_ERR "risc clock 0 !!!\n");
  }

  return u4Clock;
}

//==================================================
// public functions
//==================================================
void v_mt85xx_calibrate_plls(void)
{
    // APLL
    printk("APLL Setting\n");
    _CalAPLL(0);
}

uint32_t u4_get_bus_clock(void)
{
  uint32_t u4RiscClock;
  uint32_t u4AXIRatio;
  uint32_t u4BCLKRatio;

  uint32_t u4Clock;

#if (CONFIG_CHIP_VER_CURR == CONFIG_CHIP_VER_MT3360)
    return 324000000;
#endif

  u4RiscClock = u4_get_risc_clock();

  u4AXIRatio = CKGEN_READ32(REG_RW_TST_CFG0);

  u4AXIRatio = u4AXIRatio & TST_CFG0_AXICLK_RATIO_MASK;

  u4AXIRatio = u4AXIRatio >> TST_CFG0_AXICLK_RATIO_OFFSET;

  u4AXIRatio = u4AXIRatio + 1;

  u4BCLKRatio = CKGEN_READ32(REG_RW_TST_CFG0);

  u4BCLKRatio = u4BCLKRatio & TST_CFG0_BCLK_RATIO_MASK;

  u4BCLKRatio = u4BCLKRatio >> TST_CFG0_BCLK_RATIO_OFFSET;

  u4BCLKRatio = u4BCLKRatio + 1;

  u4Clock = (u4RiscClock/u4AXIRatio)/u4BCLKRatio;

  // error
  if(u4Clock==0)
  {
    printk(KERN_ERR "bus clock 0 !!!\n");
  }

  return u4Clock;
}
EXPORT_SYMBOL(u4_get_bus_clock);



