/*
 *  File: linux/arch/arm/mach-ac83xx/inlude/mach/ac83xx_memory.h
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

#ifndef __ARCH_ARM_MACH_AC83XX_MEMORY_H
#define __ARCH_ARM_MACH_AC83XX_MEMORY_H
//#include "autoconf.h"
#ifdef CONFIG_DVP_RESERVED_MEM
#undef CONFIG_DVP_RESERVED_MEM
#endif

#define CONFIG_DVP_RESERVED_MEM 1
#define CONFIG_AUDIN_RESERVED_MEM  1
#define CONFIG_NEW_MEM_LAYOUT  1

#if CONFIG_NEW_MEM_LAYOUT

#define DRAM_START_ADDR             (256*1024*1024UL)
#define TOTAL_DRAM_SIZE             (511*1024*1024UL)

#if CONFIG_DVP_RESERVED_MEM
#define DVP_BUFFER_MEM_SIZE         (1024*1024*11UL)
#else
#error 20
#define DVP_BUFFER_MEM_SIZE         (0UL)
#endif
#define DVP_BUFFER_PA               DRAM_START_ADDR

#define AUDIO_DSP_MEM_SIZE          (1024*1024*6UL)     // 6M DSP Working Buffer
#define AUDIO_DSP_MEM_PA            (DVP_BUFFER_PA + DVP_BUFFER_MEM_SIZE)

#define AUDIO_FIFO_MEM_SIZE         (1024*1024*3/2UL)   // 1.5M AFIFO
#define AUDIO_FIFO_MEM_PA           (AUDIO_DSP_MEM_PA + AUDIO_DSP_MEM_SIZE )

#define MULTIMEDIA_MEM_SIZE         (1024*1024*39UL)  //management by OS
#define MULTIMEDIA_MEM_PA           (AUDIO_FIFO_MEM_PA + AUDIO_FIFO_MEM_SIZE)

#define EXTERNAL_DISPLAY_FB_MEM_SIZE   ( 0x400000 )
#define EXTERNAL_DISPLAY_FB_MEM_PA     (MULTIMEDIA_MEM_PA + MULTIMEDIA_MEM_SIZE)

#define RESERVED_MEM_SIZE           (1024*1024*0UL)//Don't need align address anymore
#define RESERVED_MEM_PA             (EXTERNAL_DISPLAY_FB_MEM_PA + EXTERNAL_DISPLAY_FB_MEM_SIZE)

#define SUBTITLE_MEM_SIZE           (1024*1024*3UL)
#define SUBTITLE_MEM_PA             (RESERVED_MEM_PA + RESERVED_MEM_SIZE)

  #define METAZONE_MEM_SIZE         (1024*64)  // alloc 64k for metazone from reserved memory
  #define METAZONE_MEM_PA           (SUBTITLE_MEM_PA + SUBTITLE_MEM_SIZE - METAZONE_MEM_SIZE)
  
  #define MM_OSE_LIST_MEM_SIZE      (1024*16)
  #define MM_OSE_LIST_MEM_PA        (METAZONE_MEM_PA - MM_OSE_LIST_MEM_SIZE)

#define FRAMEBUFFER_MEM_SIZE        (1024*1024*34UL)
#define FRAMEBUFFER_MEM_PA          (SUBTITLE_MEM_PA + SUBTITLE_MEM_SIZE)

#define SD_DMA_MEM_SIZE             (0UL)  // 256K SD DMA Buffer
#define SD_DMA_MEM_PA               (FRAMEBUFFER_MEM_PA + FRAMEBUFFER_MEM_SIZE)	

#define AUDIO_PCM_MEM_SIZE         (1024*512UL)     // 256K audio  Buffer
#define AUDIO_PCM_MEM_PA           (SD_DMA_MEM_PA + SD_DMA_MEM_SIZE)

#define IMAGE_RESIZE_MEM_SIZE       (1024*1024*20UL)     // 20M image resize  Buffer
#define IMAGE_RESIZE_MEM_PA         (AUDIO_PCM_MEM_PA + AUDIO_PCM_MEM_SIZE)

#define YC_BUFFER_MEM_SIZE          (1024*1024*3UL) // 3M for Y C
#define YC_BUFFER_PA                (IMAGE_RESIZE_MEM_PA + IMAGE_RESIZE_MEM_SIZE)

#if CONFIG_AUDIN_RESERVED_MEM
#define AUDIO_IN_MEM_SIZE         (1024*1024*2UL) // 2M audioin Buffer
#else
#define AUDIO_IN_MEM_SIZE         (0UL)
#endif
#define AUDIO_IN_MEM_PA           (YC_BUFFER_PA + YC_BUFFER_MEM_SIZE)

#if 1
#define TESTBACKWARD_MEM_SIZE         (0UL)  //test
#define TESTBACK_MEM_PA           (AUDIO_IN_MEM_PA + AUDIO_IN_MEM_SIZE)

#define TESTFORWARD_MEM_SIZE         (0UL)  //management by OS
#define TESTFORWARD_MEM_PA           (TESTBACK_MEM_PA + TESTBACKWARD_MEM_SIZE)
#endif

#define WCH_MEM_TVD_SIZE          (0x00300000) // 5*720*576*3/2
#define WCH_MEM_TVD_PA            (AUDIO_IN_MEM_PA + AUDIO_IN_MEM_SIZE)

#define WCH_MEM_YPBPR_VGA_SIZE    (0x00800000) // 5*720*576*3/2 YPBPR; 3*800*600*3/2 VGA
#define WCH_MEM_YPBPR_VGA_PA      (WCH_MEM_TVD_PA + WCH_MEM_TVD_SIZE)

#define WCH_MEM_HDMI_SIZE         (0x00900000) // 3*1920*1080*3/2 HD; 5*720*576*3/2 SD
#define WCH_MEM_HDMI_PA           (WCH_MEM_YPBPR_VGA_PA + WCH_MEM_YPBPR_VGA_SIZE)

#define WCH_MEM_DGI_SIZE          (0x00300000) // 5*720*576*3/2
#define WCH_MEM_DGI_PA            (WCH_MEM_HDMI_PA + WCH_MEM_HDMI_SIZE)

#define WCH_MEM_DISP_SIZE         (0x00480000) // 3*1280*800*3/2 FRONT; 3*720*576*3/2 REAR
#define WCH_MEM_DISP_PA           (WCH_MEM_DGI_PA + WCH_MEM_DGI_SIZE)

#define ARM2_RESERVED_MEM_SIZE    (1024*1024*2UL)
#define ARM2_RESERVED_MEM_PA      (WCH_MEM_DISP_PA + WCH_MEM_DISP_SIZE)

#define RESV_MEM_IO_VIRT_START      0xF0000000

#define RESV_MEM_IO_SIZE          (FRAMEBUFFER_MEM_SIZE     \
                                      + SUBTITLE_MEM_SIZE     \
                                      + MULTIMEDIA_MEM_SIZE   \
                                      + AUDIO_DSP_MEM_SIZE    \
                                      + AUDIO_PCM_MEM_SIZE   \
                                      + IMAGE_RESIZE_MEM_SIZE \
                                      + SD_DMA_MEM_SIZE       \
                                      + AUDIO_FIFO_MEM_SIZE   \
                                      + YC_BUFFER_MEM_SIZE    \
                                      + DVP_BUFFER_MEM_SIZE   \
                                      + AUDIO_IN_MEM_SIZE    \
                                      + RESERVED_MEM_SIZE    \
                                      + EXTERNAL_DISPLAY_FB_MEM_SIZE \
				      + TESTBACKWARD_MEM_SIZE    \
                                      + TESTFORWARD_MEM_SIZE \
				      + WCH_MEM_TVD_SIZE \
                                      + WCH_MEM_YPBPR_VGA_SIZE \
                                      + WCH_MEM_HDMI_SIZE \
                                      + WCH_MEM_DGI_SIZE \
                                      + WCH_MEM_DISP_SIZE\
                                      + ARM2_RESERVED_MEM_SIZE)

#define RESV_MEM_IO_VIRT_TOP      RESV_MEM_IO_VIRT_START + RESV_MEM_IO_SIZE

#define RESV_MEM_IO_VIRT          RESV_MEM_IO_VIRT_START
#define RESV_MEM_IO_PHYS          DRAM_START_ADDR

#define MEMRSV_PHY_TO_VIRT(x)       (RESV_MEM_IO_VIRT + (x) - RESV_MEM_IO_PHYS)
#define MEMRSV_VIRT_TO_PHY(x)       (RESV_MEM_IO_PHYS + (x) - RESV_MEM_IO_VIRT)

#define REVERSE_MEM_PHY_ADDR        AUDIO_FIFO_MEM_PA // (Base AFIFO)
#define REVERSE_MEM_SIZE            (AUDIO_FIFO_MEM_SIZE + MULTIMEDIA_MEM_SIZE)  // Multimedia used reserved memory size(AFIFO+MultimediaSize)

#else //CONFIG_NEW_MEM_LAYOUT
#define TOTAL_DRAM_SIZE             (511*1024*1024UL)
//#define TOTAL_DRAM_SIZE             (767*1024*1024UL)

#define YC_BUFFER_MEM_SIZE          (1024*1024*3UL) // 3M for Y C
#define YC_BUFFER_PA                (TOTAL_DRAM_SIZE -  YC_BUFFER_MEM_SIZE)

#define IMAGE_RESIZE_MEM_SIZE       (1024*1024*3UL)     // 3M image resize  Buffer
#define IMAGE_RESIZE_MEM_PA         (YC_BUFFER_PA - IMAGE_RESIZE_MEM_SIZE)

#define AUDIO_PCM_MEM_SIZE         (1024*512UL)     // 256K audio  Buffer
#define AUDIO_PCM_MEM_PA           (IMAGE_RESIZE_MEM_PA - AUDIO_PCM_MEM_SIZE)

#define SD_DMA_MEM_SIZE             (0UL)  // 256K SD DMA Buffer
#define SD_DMA_MEM_PA               (AUDIO_PCM_MEM_PA - SD_DMA_MEM_SIZE)

#define FRAMEBUFFER_MEM_SIZE        (1024*1024*34UL)
#define FRAMEBUFFER_MEM_PA          (SD_DMA_MEM_PA - FRAMEBUFFER_MEM_SIZE)

#define SUBTITLE_MEM_SIZE           (1024*1024*3UL)
#define SUBTITLE_MEM_PA             (FRAMEBUFFER_MEM_PA - SUBTITLE_MEM_SIZE)

#define RESERVED_MEM_SIZE           (1024*1024*1UL)//beacause AUDIO_DSP_FIFO AND DVP fifo must in a 16M bank, reserve 3M
#define RESERVED_MEM_PA             (SUBTITLE_MEM_PA - RESERVED_MEM_SIZE)

  #define METAZONE_MEM_SIZE         (1024*64)  // alloc 64k for metazone from reserved memory
  #define METAZONE_MEM_PA           (SUBTITLE_MEM_PA - METAZONE_MEM_SIZE)
  
  #define MM_OSE_LIST_MEM_SIZE      (1024*16)
  #define MM_OSE_LIST_MEM_PA        (METAZONE_MEM_PA - MM_OSE_LIST_MEM_SIZE)

#define AUDIO_FIFO_MEM_SIZE         (1024*1024*3/2UL)   // 1.5M AFIFO
#define AUDIO_FIFO_MEM_PA           (RESERVED_MEM_PA - AUDIO_FIFO_MEM_SIZE)

#define AUDIO_DSP_MEM_SIZE          (1024*1024*6UL)     // 6M DSP Working Buffer
#define AUDIO_DSP_MEM_PA            (AUDIO_FIFO_MEM_PA - AUDIO_DSP_MEM_SIZE)

#if CONFIG_DVP_RESERVED_MEM
#define DVP_BUFFER_MEM_SIZE         (1024*1024*11UL)
#else
#error 20
#define DVP_BUFFER_MEM_SIZE         (0UL)
#endif
#define DVP_BUFFER_PA               (AUDIO_DSP_MEM_PA - DVP_BUFFER_MEM_SIZE)

#define AUDIO_IN_MEM_SIZE         (1024*1024*2UL)     // 2M audioin  Buffer
#define AUDIO_IN_MEM_PA           (DVP_BUFFER_PA - AUDIO_IN_MEM_SIZE)

#define TESTBACKWARD_MEM_SIZE         (1024*1024*1UL)  //test
#define TESTBACK_MEM_PA           (AUDIO_IN_MEM_PA - TESTBACKWARD_MEM_SIZE)

#define MULTIMEDIA_MEM_SIZE         (1024*1024*39UL)  //management by OS
#define MULTIMEDIA_MEM_PA           (TESTBACK_MEM_PA - MULTIMEDIA_MEM_SIZE)

#define TESTFORWARD_MEM_SIZE         (1024*1024*1UL)  //management by OS
#define TESTFORWARD_MEM_PA           (MULTIMEDIA_MEM_PA - TESTFORWARD_MEM_SIZE)

#define EXTERNAL_DISPLAY_FB_MEM_SIZE   ( 0x400000 )
#define EXTERNAL_DISPLAY_FB_MEM_PA    (TESTFORWARD_MEM_PA - EXTERNAL_DISPLAY_FB_MEM_SIZE)

#define WCH_MEM_TVD_SIZE          (0x00300000) // 5*720*576*3/2
#define WCH_MEM_TVD_PA            (EXTERNAL_DISPLAY_FB_MEM_PA - WCH_MEM_TVD_SIZE)

#define WCH_MEM_YPBPR_VGA_SIZE    (0x00300000) // 5*720*576*3/2 YPBPR; 3*800*600*3/2 VGA
#define WCH_MEM_YPBPR_VGA_PA      (WCH_MEM_TVD_PA - WCH_MEM_YPBPR_VGA_SIZE)

#define WCH_MEM_HDMI_SIZE         (0x00900000) // 3*1920*1080*3/2 HD; 5*720*576*3/2 SD
#define WCH_MEM_HDMI_PA           (WCH_MEM_YPBPR_VGA_PA - WCH_MEM_HDMI_SIZE)

#define WCH_MEM_DGI_SIZE          (0x00300000) // 5*720*576*3/2
#define WCH_MEM_DGI_PA            (WCH_MEM_HDMI_PA - WCH_MEM_DGI_SIZE)

#define WCH_MEM_DISP_SIZE         (0x00480000) // 3*1280*800*3/2 FRONT; 3*720*576*3/2 REAR
#define WCH_MEM_DISP_PA           (WCH_MEM_DGI_PA - WCH_MEM_DISP_SIZE)


#define MULTIMEDIA_IO_VIRT_TOP      0xF8000000
#define MULTIMEDIA_IO_SIZE          (FRAMEBUFFER_MEM_SIZE     \
                                      + SUBTITLE_MEM_SIZE     \
                                      + MULTIMEDIA_MEM_SIZE   \
                                      + AUDIO_DSP_MEM_SIZE    \
                                      + AUDIO_PCM_MEM_SIZE   \
                                      + IMAGE_RESIZE_MEM_SIZE \
                                      + SD_DMA_MEM_SIZE       \
                                      + AUDIO_FIFO_MEM_SIZE   \
                                      + YC_BUFFER_MEM_SIZE    \
                                      + DVP_BUFFER_MEM_SIZE   \
                                      + RESERVED_MEM_SIZE    \
                                      + TESTBACKWARD_MEM_SIZE    \
                                      + TESTFORWARD_MEM_SIZE \
                                      + EXTERNAL_DISPLAY_FB_MEM_SIZE \
                                      + AUDIO_IN_MEM_SIZE \
                                      + WCH_MEM_TVD_SIZE \
                                      + WCH_MEM_YPBPR_VGA_SIZE \
                                      + WCH_MEM_HDMI_SIZE \
                                      + WCH_MEM_DGI_SIZE \
                                      + WCH_MEM_DISP_SIZE)

#define MULTIMEDIA_IO_VIRT          (MULTIMEDIA_IO_VIRT_TOP - MULTIMEDIA_IO_SIZE)
#define MULTIMEDIA_IO_PHYS          (TOTAL_DRAM_SIZE - MULTIMEDIA_IO_SIZE)

#define MEMRSV_PHY_TO_VIRT(x)       (MULTIMEDIA_IO_VIRT_TOP - (TOTAL_DRAM_SIZE - (x)))
#define MEMRSV_VIRT_TO_PHY(x)       (TOTAL_DRAM_SIZE - (MULTIMEDIA_IO_VIRT_TOP - (x)))
#define REVERSE_MEM_PHY_ADDR (TOTAL_DRAM_SIZE - 0x8000000 + 1024 * 1024UL) 
#define REVERSE_MEM_SIZE (128*0x100000)  //128M

#define ARM2_RESERVED_MEM_PA 0x1FF00000

#endif //CONFIG_NEW_MEM_LAYOUT

#endif //__ARCH_ARM_MACH_AC83XX_MEMORY_H
