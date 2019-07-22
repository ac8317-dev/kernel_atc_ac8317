/*
* arch/arm/mach-mt33xx/pm.c
*
* Copyright (C) 2010 MediaTek Inc
*
* This program is not free software; you can not redistribute it
* or modify it without license from MediaTek Inc.
*
*  mtk40005
*  zhanyong.wang
*/
#include <mach/chip_ver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <mach/mt33xx.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <linux/io.h>

#define DELAY_LOOP(x)                 for(i=0;i<(x);i++);
#define MT33XX_WAKEUP_ENABLE(n)      (1ul << ((n)%32))

#define MT33XX_PMSAVE_IRQEN          ((BIM_VIRT)+(REG_IRQEN))
#define MT33XX_PMSAVE_IRQEN2         ((BIM_VIRT)+(REG_IRQEN2))
#define MT33XX_PMSAVE_IRQEN3         ((BIM_VIRT)+(REG_IRQEN3))
#define MT33XX_PMSAVE_IRQEN4         ((BIM_VIRT)+(REG_IRQEN4))
#define MT33XX_PMSAVE_PDWNCIRQEN     ((PDWNC_VIRT)+(REG_RW_INTEN))
/* if no irq yet defined, but still want to mask */
#define RESERVE_WAKEUPIRQ            (0xFFFFFFFF)

/**
* struct mt33xx_wakeup_mask - wakeup mask information
* @irq: The interrupt associated with this wakeup.
* @bit: The bit, as a (1 << bitno) controlling this source.
*/
struct mt33xx_wakeup_mask
{
    unsigned int    reg;
    unsigned int    irq;
    unsigned int    bit;
};

/**
* struct sleep_save - save information for shared peripherals.
* @reg: Pointer to the register to save.
* @val: Holder for the value saved from reg.
*
* This describes a list of registers which is used by the pm core and
* other subsystem to save and restore register values over suspend.
*/
struct backup_irqcfg_save
{
    unsigned int    reg;
    unsigned long   val;
};
#define BACK_IRQREG(x) { .reg = (x) }
static struct backup_irqcfg_save irqen_save[] =
{
    BACK_IRQREG(MT33XX_PMSAVE_IRQEN),
    BACK_IRQREG(MT33XX_PMSAVE_IRQEN2),
    BACK_IRQREG(MT33XX_PMSAVE_IRQEN3),
    BACK_IRQREG(MT33XX_PMSAVE_IRQEN4),
    BACK_IRQREG(MT33XX_PMSAVE_PDWNCIRQEN)
};
/* mapping of interrupts to parts of the wakeup mask */
static struct mt33xx_wakeup_mask wake_irqs[NR_IRQS] =
{
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_GRPC,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_GRPB,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_GRPD,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_EXT,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_SPI2,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_EXT2,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_VDOIN,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_GRAPH,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_JPGDEC,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_DDMANEW,         .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(10)},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(11)},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_JAVA,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_RLE,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_RS232_1,         .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_DSP,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_SPD,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_VDOUTREAR,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(18)},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_SFDMAI,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_DDMAI,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_PL310,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_USB,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_TOCORISC,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_AXI64_WR,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_T2,              .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_T1,              .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_T0,              .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_DSPC,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_SVO_FE1INT,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_SVO_FE0INT,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN,      .irq = VECTOR_VSYNC,           .bit = 0},  /*32*/
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(32)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_NFI,             .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_MALI_3D0,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_MALI_3D1,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_MALI_3D2,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_MALI_3D3,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(38)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_EXT4,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_DRAMC,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(41)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_RESIZER1,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_RESIZER0,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_SMARTCARD,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_PANEL_SCALER,    .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(46)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_SPIINT,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_DDIINT,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_OSD3R,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_DMXINT,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_VENC,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_PWDNC,           .bit = MT33XX_WAKEUP_ENABLE(VECTOR_PWDNC)}, /* extra interrupt source */
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(53)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(54)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(55)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_OSD2R,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(57)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(58)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(59)},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_FLASH,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_UART_4,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_UART_3,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN2,     .irq = VECTOR_LZHS,            .bit = 0},/*64*/
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_UART_5,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_UART_2,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(66)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(67)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_DISP_END_R,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(69)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_UART_1,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_PNG2,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_PNG1,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_GIF2,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_GIF1,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_PSTQ,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_SRCQ,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_OSD5_UNFLW,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_OSD4_UNFLW,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_OSD3_UNFLW,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_OSD2_UNFLW,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_OSD1_UNFLW,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_MALI_3D4,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_DISP_END_F,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(84)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_USB2,            .bit = MT33XX_WAKEUP_ENABLE(VECTOR_USB2)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,      .bit = MT33XX_WAKEUP_ENABLE(86)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_UART_6,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_DISPVDO2_UNDERRUN, .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(89)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(90)},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_VDFUL,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_DSPB2RISC,      .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_DISPVDO_UNDERRUN, .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_PNG,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN3,     .irq = VECTOR_GIF,            .bit = 0},/*96*/
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_EXT3,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_EXT5,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_EXT6,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_EXT7,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_EXT8,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PT110AP0,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PT110AP1,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PT110AP2,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PT110AP3,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_UP2AP0,         .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWNFB,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_KPI,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_TSI,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(109)},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_RTC,            .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_SPI1,           .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_ASRC_AP,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_ASRC_GPS,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWM_REAR,       .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWM_FRONT1,     .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWM_FRONT2,     .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWM_FRONT3,     .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_PWM_GPS,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_AOUT_GPS_RC,    .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_AOUT_2ND_RC,    .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_AOUT_BT_RC,     .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_MIC_RC,         .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_MSDC0,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_MSDC1,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_MSDC2,          .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = VECTOR_LVDSTOP,        .bit = 0},
    { .reg = MT33XX_PMSAVE_IRQEN4,     .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(127)},/*128*/
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO0,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO1,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO2,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO3,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO4,    .bit = MT33XX_WAKEUP_ENABLE(VECTOR_INT_P_GPIO4)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO5,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO6,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_GPIO7,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(136)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_DBG_UART, .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_SIFS,     .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_CEC,      .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_ETNET,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_IR,       .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_SIFM,     .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = VECTOR_INT_P_DDCCI,    .bit = 0},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(144)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(145)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(146)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(147)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(148)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(149)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(150)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(151)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(152)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(163)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(154)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(155)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(156)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(157)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(158)},
    { .reg = MT33XX_PMSAVE_PDWNCIRQEN, .irq = RESERVE_WAKEUPIRQ,     .bit = MT33XX_WAKEUP_ENABLE(159)},/*160*/
};



extern void  oalentersleepmode(void);
extern void oalcpupoweroff(void);
extern void oalinterrupton(void);
//#define SUSPEND_TO_MEM	//real suspend to mem

void mt33xx_wakeup_operation(struct backup_irqcfg_save *pIrqcfg, int count, bool fgSave)
{
    for (; pIrqcfg && (count > 0);  count--, pIrqcfg++)
    {
        if(fgSave)
        {
            pIrqcfg->val = HAL_READ32(pIrqcfg->reg);
        }
        else
        {
            HAL_WRITE32(pIrqcfg->reg, pIrqcfg->val);
        }
    }
}



/**
* mt33xx_sync_wakemask - sync wakeup mask information for pm
* @masks: The list of masks to use.
* @nr_masks: The number of entries pointed to buy @masks.
*
* Synchronise the wakeup mask information at suspend time from the list
* of interrupts and control bits in @masks. We do this at suspend time
* as overriding the relevant irq chips is harder and the register is only
* required to be correct before we enter sleep.
*/
void mt33xx_buildup_wakemask(struct mt33xx_wakeup_mask *mask,int cb_masks)
{
    u32 val=0;
    unsigned int reg=0;
    struct irq_desc *desc=NULL;

    for (; cb_masks > 0; cb_masks --, mask++)
    {
        if((cb_masks%32) == 0)
        {
            if(reg != 0)
            {
                printk(KERN_INFO "wakemask [0x%08X] = %08x => %08x\n", reg, HAL_READ32(reg), val);
                HAL_WRITE32(reg, val);
            }
            val = 0; //HAL_READ32(mask->reg);
            reg = mask->reg;
        }

        if (mask->irq == RESERVE_WAKEUPIRQ)
        {
            val &= ~mask->bit;
            continue;
        }

        #if 0
            desc = irq_to_desc(mask->irq);
            if(desc)
            {
                /* bit of a liberty to read this directly from irq_desc. */
                //printk(KERN_INFO "[]desc.depth      = %u\n", mask->irq,desc->depth);
                //printk(KERN_INFO "[]desc.wake_depth = %u\n", mask->irq,desc->wake_depth);
                if (desc->wake_depth > 0)
                {
                    val |= mask->bit;
                }
                else
                {
                    val &= ~mask->bit;
                }
            }
            else
            {
               // printk(KERN_INFO "irq_to_desc(%u) failed\n", mask->irq);
                val |= mask->bit;
            }
        #else
            val |= mask->bit;
        #endif

    }
}

int mt33xx_wakeup_configure(unsigned int u4Irq, bool fgSet)
{
    int fgResult = false;
    struct mt33xx_wakeup_mask *prWakeUp = NULL;
    if(u4Irq < NR_IRQS)
    {
        prWakeUp = &wake_irqs[u4Irq];
        if((prWakeUp!=NULL) && (prWakeUp->irq != RESERVE_WAKEUPIRQ))
        {
            fgResult = true;
            prWakeUp->bit = 0;
            if(fgSet)
            {
                prWakeUp->bit = MT33XX_WAKEUP_ENABLE(u4Irq);
            }
        }
    }
    return fgResult;
}
EXPORT_SYMBOL(mt33xx_wakeup_configure);

static void mt33xx_wakeup_prepare(void)
{
    mt33xx_buildup_wakemask(wake_irqs, ARRAY_SIZE(wake_irqs));

    /* ensure previous wakeup state is cleared before sleeping */
    BIM_WRITE32(REG_IRQST ,0);
    BIM_WRITE32(REG_IRQST2,0);
    BIM_WRITE32(REG_IRQST3,0);
    BIM_WRITE32(REG_IRQST4,0);
    //PDWNC_WRITE32(REG_RW_INTSTA,0);
}

static int mt33xx_pm_valid(suspend_state_t state)
{
    int fgResult;
    fgResult = ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
    printk(KERN_INFO "mt33xx_pm_valid %s (%u) %s\n", \
           (!state)   ?"PM_SUSPEND_ON":\
           (1==state) ?"PM_SUSPEND_STANDBY":\
           (3==state) ?"PM_SUSPEND_MEM":"UNKNOW",
           state,
           fgResult?"support":"unsupport");

    return fgResult;
}


static int mt33xx_pm_begin(suspend_state_t state)
{
    printk(KERN_INFO "mt33xx_pm_recover: Nothing to do!\n");

    return 0;
}

static int mt33xx_pm_prepare( void)
{
    printk(KERN_INFO "mt33xx_pm_prepare\n");

    // Save interrupt masks
    mt33xx_wakeup_operation(irqen_save, ARRAY_SIZE(irqen_save),true);

    // mask all interrupts

    BIM_WRITE32(REG_IRQEN      ,0);
    BIM_WRITE32(REG_IRQEN2     ,0);
    BIM_WRITE32(REG_IRQEN3     ,0);
    BIM_WRITE32(REG_IRQEN4     ,0);
    //PDWNC_WRITE32(REG_RW_INTEN ,0);

    // Setup wakeup source
    mt33xx_wakeup_prepare();

    return 0;
}

static int mt33xx_pm_prepare_late(void)
{
    u32 u4Clock;

    printk(KERN_INFO "mt33xx_pm_prepare_late");
#ifndef SUSPEND_TO_MEM
    return 0;
#endif
    //switch ARM11 to 27M, then trun off ARM11 PLL
    u4Clock = CKGEN_READ32(REG_RW_AP_REG3);
    u4Clock = u4Clock & (~CLK_REG3_CPU1_SEL_MASK);
    u4Clock = u4Clock | CLK_REG3_CPU1_SEL_CLK27M;
    CKGEN_WRITE32(REG_RW_AP_REG3, u4Clock);

    u4Clock = CKGEN_READ32(REG_RW_ANA7_PLLGP_CFG7);
    u4Clock = u4Clock & (~ARMPLL_PWD_MASK);
    u4Clock = u4Clock | ARMPLL_PWD_DISABLE;
    CKGEN_WRITE32(REG_RW_ANA7_PLLGP_CFG7, u4Clock);

    //switch ARM9 to 27M, then trun off ARM9 PLL
    u4Clock = CKGEN_READ32(REG_RW_AP_REG3);
    u4Clock = u4Clock & (~CLK_REG3_CPU2_SEL_MASK);
    u4Clock = u4Clock | CLK_REG3_CPU2_SEL_CLK27M;
    CKGEN_WRITE32(REG_RW_AP_REG3, u4Clock);

    u4Clock = CKGEN_READ32(REG_RW_ANA7_PLLGP_CFG8);
    u4Clock = u4Clock & (~ARM9PLL_PWD_MASK);
    u4Clock = u4Clock | ARM9PLL_PWD_DISABLE;
    CKGEN_WRITE32(REG_RW_ANA7_PLLGP_CFG8, u4Clock);

    return 0;
}


static int mt33xx_pm_enter(suspend_state_t state)
{
    printk(KERN_INFO "mt33xx_pm_enter %s (%u)\n",\
           (!state)   ?"PM_SUSPEND_ON":\
           (1==state) ?"PM_SUSPEND_STANDBY":\
           (3==state) ?"PM_SUSPEND_MEM":"UNKNOW", state);

    switch(state)
    {
        case PM_SUSPEND_MEM:
        #ifdef SUSPEND_TO_MEM
                oalcpupoweroff();
                break;
            #endif
            case PM_SUSPEND_STANDBY:
                oalentersleepmode();
            //  printk(KERN_INFO "I myself check which module hangup flowchart here just(nothing)!\n");
            break;
        #ifdef CONFIG_EARLYSUSPEND
            case PM_SUSPEND_ON:
                break;
        #endif
        default:
            break;
    }
    return 0;
}

static void mt33xx_pm_wake(void)
{
    int i=0;
    u32 u4Clock;

    printk(KERN_INFO "mt33xx_pm_wake\n");
#ifndef SUSPEND_TO_MEM
    return;
#endif
    //trun on ARM11 PLL, then switch ARM11 to ARM11 PLL
    u4Clock  = CKGEN_READ32(REG_RW_ANA7_PLLGP_CFG7);
    u4Clock &= (~ARMPLL_PWD_MASK);
    CKGEN_WRITE32(REG_RW_ANA7_PLLGP_CFG7, u4Clock);

    DELAY_LOOP(0x21C);

    u4Clock  = CKGEN_READ32(REG_RW_AP_REG3);
    u4Clock &= (~CLK_REG3_CPU1_SEL_MASK);
    u4Clock |= (CLK_REG3_CPU1_SEL_CLK_ARMPLL << CLK_REG3_CPU1_SEL_OFFSET);
    CKGEN_WRITE32(REG_RW_AP_REG3, u4Clock);
    //trun on ARM9 PLL, then switch ARM9 to ARM9 PLL
    u4Clock  = CKGEN_READ32(REG_RW_ANA7_PLLGP_CFG8);
    u4Clock &= (~ARM9PLL_PWD_MASK);
    CKGEN_WRITE32(REG_RW_ANA7_PLLGP_CFG8, u4Clock);

    DELAY_LOOP(0x21C);

    u4Clock  = CKGEN_READ32(REG_RW_AP_REG3);
    u4Clock &= (~CLK_REG3_CPU2_SEL_MASK);
    u4Clock |= (CLK_REG3_CPU2_SEL_CLK_ARMPLL2 << CLK_REG3_CPU2_SEL_OFFSET);
    CKGEN_WRITE32(REG_RW_AP_REG3, u4Clock);
}

static void mt33xx_pm_finish(void)
{
    printk(KERN_INFO "mt33xx_pm_finish");

    // Restore interrupt masks
    mt33xx_wakeup_operation(irqen_save, ARRAY_SIZE(irqen_save),false);
#ifdef SUSPEND_TO_MEM    
    oalinterrupton();
#endif
}

static void mt33xx_pm_end(void)
{
    printk(KERN_INFO "mt335xx_pm_recover: Nothing to do!\n");
}

static void mt33xx_pm_recover(void)
{
    printk(KERN_INFO "mt33xx_pm_recover: Nothing to do!\n");
}

// begin->*prepare->*prepare_late->*enter->*wake->*finish->end
static struct platform_suspend_ops mt33xx_pm_ops =
{
    .valid        = mt33xx_pm_valid,
    .begin        = mt33xx_pm_begin,
    .prepare      = mt33xx_pm_prepare,
    .prepare_late = mt33xx_pm_prepare_late,
    .enter        = mt33xx_pm_enter,
    .wake         = mt33xx_pm_wake,
    .finish       = mt33xx_pm_finish,
    .end          = mt33xx_pm_end,
    .recover      = mt33xx_pm_recover,
};

static int __init mt33xx_pm_init(void)
{
    #ifdef CONFIG_PM
        printk(KERN_INFO "mt33xx_pm_init\n");
        suspend_set_ops(&mt33xx_pm_ops);
    #endif
    return 0;
}

late_initcall(mt33xx_pm_init);
