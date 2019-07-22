/*
 *  linux/arch/arm/mach-ac83xx/include/mach/ac83xx_CS_watch.h
 *
 * Copyright (C) 2009 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 * ac83xx Critical Section watch.
 */

#ifndef _AC83XX_CS_WATCH_H_
#define _AC83XX_CS_WATCH_H_


/* define & config */
#define CONFIG_AC83XX_CS_WATCH         0


/* global functions */
extern void ac83xx_CS_watch_init(void);
extern void ac83xx_CS_watch_start(unsigned int u4Flag);
extern void ac83xx_CS_watch_end(unsigned int u4Flag);
extern void ac83xx_CS_watch_dump(void);
#endif // _AC83XX_CS_WATCH_H_
