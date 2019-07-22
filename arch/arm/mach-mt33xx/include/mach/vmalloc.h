/*
 *  linux/include/asm-arm/arch-mt85xx/vmalloc.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */

/*
 * Just any arbitrary offset to the start of the vmalloc VM area: the
 * current 8MB value just means that there will be a 8MB "hole" after the
 * physical memory until the kernel virtual memory starts.  That means that
 * any out-of-bounds memory accesses will hopefully be caught.
 * The vmalloc() routines leaves a hole of 4kB between each vmalloced
 * area for the same reason. ;)
 */

//#define VMALLOC_END      0xf8000000

//#define VMALLOC_END      0xe8000000
#define VMALLOC_END      0xf0000000

