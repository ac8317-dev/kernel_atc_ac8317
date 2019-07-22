/*
 *  File: linux/arch/arm/mach-mt85xx/inlude/mach/mt85xx_memory.h
 *
 *  Copyright:  (C) 2011 MediaTek Inc.
 *
 *  This program is not free software; you can not redistribute it
 *  or modify it without license from MediaTek Inc.
 *
 *  Discription: Thie file is used to management the memory reserved for user.
 */


// Note: the size of your area must be mutiply of a page, now a page size is 4k
// When you add a area, you should add it to MULTIMEDIA_IO_SIZE
 
#ifndef __ARCH_ARM_MACH_MT85XX_MEMORY_H
#define __ARCH_ARM_MACH_MT85XX_MEMORY_H
//#include "autoconf.h"

#define TOTAL_DRAM_SIZE             (255*1024*1024UL)
//#define TOTAL_DRAM_SIZE             (511*1024*1024UL)

#define YC_BUFFER_MEM_SIZE          (1024*1024*3UL) // 3M for Y C
#define YC_BUFFER_PA                (TOTAL_DRAM_SIZE -  YC_BUFFER_MEM_SIZE)

#define IMAGE_RESIZE_MEM_SIZE       (1024*1024*3UL)     // 3M image resize  Buffer
#define IMAGE_RESIZE_MEM_PA         (YC_BUFFER_PA - IMAGE_RESIZE_MEM_SIZE)

#define AUDIO_ASRC_MEM_SIZE         (1024*256UL)     // 256K audio  Buffer
#define AUDIO_ASRC_MEM_PA           (IMAGE_RESIZE_MEM_PA - AUDIO_ASRC_MEM_SIZE)

#define SD_DMA_MEM_SIZE             (1024*256UL)  // 256K SD DMA Buffer
#define SD_DMA_MEM_PA               (AUDIO_ASRC_MEM_PA - SD_DMA_MEM_SIZE)

#define FRAMEBUFFER_MEM_SIZE        (1024*1024*19UL)
#define FRAMEBUFFER_MEM_PA          (SD_DMA_MEM_PA - FRAMEBUFFER_MEM_SIZE)

#define SUBTITLE_MEM_SIZE           (1024*1024UL)
#define SUBTITLE_MEM_PA             (FRAMEBUFFER_MEM_PA - SUBTITLE_MEM_SIZE)

#define RESERVED_MEM_SIZE           (1024*1024*3UL)//beacause AUDIO_DSP_FIFO AND DVP fifo must in a 16M bank, reserve 3M
#define RESERVED_MEM_PA             (SUBTITLE_MEM_PA - RESERVED_MEM_SIZE)

  #define METAZONE_MEM_SIZE         (1024*64)  // alloc 64k for metazone from reserved memory
  #define METAZONE_MEM_PA           (SUBTITLE_MEM_PA - METAZONE_MEM_SIZE)

#define AUDIO_FIFO_MEM_SIZE         (1024*1024*3/2UL)   // 1.5M AFIFO
#define AUDIO_FIFO_MEM_PA           (RESERVED_MEM_PA - AUDIO_FIFO_MEM_SIZE)

#define AUDIO_DSP_MEM_SIZE          (1024*1024*6UL)     // 6M DSP Working Buffer
#define AUDIO_DSP_MEM_PA            (AUDIO_FIFO_MEM_PA - AUDIO_DSP_MEM_SIZE)

#ifdef CONFIG_DVP_RESERVED_MEM
#define DVP_BUFFER_MEM_SIZE         (1024*1024*10UL)
#else
#define DVP_BUFFER_MEM_SIZE         (0UL)
#endif
#define DVP_BUFFER_PA               (AUDIO_DSP_MEM_PA - DVP_BUFFER_MEM_SIZE)

#define TESTBACKWARD_MEM_SIZE         (1024*1024*1UL)  //test 
#define TESTBACK_MEM_PA           (DVP_BUFFER_PA - TESTBACKWARD_MEM_SIZE)

#define MULTIMEDIA_MEM_SIZE         (1024*1024*36UL)  //management by OS 
#define MULTIMEDIA_MEM_PA           (TESTBACK_MEM_PA - MULTIMEDIA_MEM_SIZE)

#define TESTFORWARD_MEM_SIZE         (1024*1024*1UL)  //management by OS 
#define TESTFORWARD_MEM_PA           (MULTIMEDIA_MEM_PA - TESTFORWARD_MEM_SIZE)

#define MULTIMEDIA_IO_VIRT_TOP      0xF8000000
#define MULTIMEDIA_IO_SIZE          (FRAMEBUFFER_MEM_SIZE     \
                                      + SUBTITLE_MEM_SIZE     \
                                      + MULTIMEDIA_MEM_SIZE   \
                                      + AUDIO_DSP_MEM_SIZE    \
                                      + AUDIO_ASRC_MEM_SIZE   \
                                      + IMAGE_RESIZE_MEM_SIZE \
                                      + SD_DMA_MEM_SIZE       \
                                      + AUDIO_FIFO_MEM_SIZE   \
                                      + YC_BUFFER_MEM_SIZE    \
                                      + DVP_BUFFER_MEM_SIZE   \
                                      + RESERVED_MEM_SIZE    \
                                      + TESTBACKWARD_MEM_SIZE    \
                                      + TESTFORWARD_MEM_SIZE)

#define MULTIMEDIA_IO_VIRT          (MULTIMEDIA_IO_VIRT_TOP - MULTIMEDIA_IO_SIZE) 
#define MULTIMEDIA_IO_PHYS          (TOTAL_DRAM_SIZE - MULTIMEDIA_IO_SIZE)

#define MEMRSV_PHY_TO_VIRT(x)       (MULTIMEDIA_IO_VIRT_TOP - (TOTAL_DRAM_SIZE - (x)))
#define MEMRSV_VIRT_TO_PHY(x)       (TOTAL_DRAM_SIZE - (MULTIMEDIA_IO_VIRT_TOP - (x)))
#endif //__ARCH_ARM_MACH_MT85XX_MEMORY_H
