/*
 *  linux/arch/arm/mach-MT85xx/mt85xx_CS_watch.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * mt85xx Critical Section watch.
 */

#ifndef _MT85XX_CS_WATCH_H_
#define _MT85XX_CS_WATCH_H_


/* define & config */
#define CONFIG_MT85XX_CS_WATCH         0


/* global functions */
extern void mt85xx_CS_watch_init(void);
extern void mt85xx_CS_watch_start(unsigned int u4Flag);
extern void mt85xx_CS_watch_end(unsigned int u4Flag);
extern void mt85xx_CS_watch_dump(void);
#endif // _MT85XX_CS_WATCH_H_
