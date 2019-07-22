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
#include <linux/module.h>
#include <linux/linkage.h>
#include <mach/cache_operation.h>
#include <asm/cacheflush.h>
#include <mach/mt33xx.h>

//==================================================
// Define
//==================================================
#ifndef CONFIG_OUTER_CACHE
#define FLUSH_RANGE_MAX      0x00008000
#define CLEAN_RANGE_MAX      0x00008000
#define INV_RANGE_MAX        0x00008000
#define CORE_FLUSH_RANGE_MAX FLUSH_RANGE_MAX
#define CORE_CLEAN_RANGE_MAX CLEAN_RANGE_MAX
#define CORE_INV_RANGE_MAX   INV_RANGE_MAX
#else
#define FLUSH_RANGE_MAX      0x00003800
#define CLEAN_RANGE_MAX      0x00003000
#define INV_RANGE_MAX        0x00003800
#define CORE_FLUSH_RANGE_MAX 0x00008000
#define CORE_CLEAN_RANGE_MAX 0x00008000
#define CORE_INV_RANGE_MAX   0x00008000
#endif

extern void l2x0_clean_all(void);
extern void l2x0_inv_all(void);
extern void l2x0_flush_all(void);
extern void l2x0_flush_all_by_index(void);
//==================================================
// Private functions
//==================================================
inline void _hal_clean_d_cache_range(uint32_t u4Start, uint32_t u4End)
{
	int32_t r = 0;
    asm volatile(	
		"mcrr	p15, 0, %1, %0, c12\n"
		"mcr 	p15, 0, %2, c7, c10, 4\n"
		:
		: "r" (u4Start), "r" (u4End), "r"(r)
		: "cc"
    );
}
inline void _hal_inv_d_cache_range(uint32_t u4Start, uint32_t u4End)
{
    int32_t r = 0;
    asm volatile(	
		"mcrr	p15, 0, %1, %0, c6\n"
		"mcr 	p15, 0, %2, c7, c10, 4\n"
		:
		: "r" (u4Start), "r" (u4End), "r"(r)
		: "cc"
    );
}
inline void _hal_flush_d_cache_range(uint32_t u4Start, uint32_t u4End)
{
    int32_t r = 0;
    asm volatile(	
		"mcrr	p15, 0, %1, %0, c14\n"
		"mcr 	p15, 0, %2, c7, c10, 4\n"
		:
		: "r" (u4Start), "r" (u4End), "r"(r)
		: "cc"
    );
}

inline void _hal_clean_d_cache()
{
  uint32_t r = 0;

  __asm__ volatile(
    "mcr 	p15, 0, %0, c7, c10, 0\n"
		"mcr 	p15, 0, %0, c7, c10, 4\n"
    :
    : "r" (r)
    : "cc"
  );
}
/*   invalidate all is forbidden
inline void _hal_inv_d_cache()
{
  uint32_t r = 0;

  __asm__ volatile(
    "mcr 	p15, 0, %0, c7, c6, 0\n"
		"mcr 	p15, 0, %0, c7, c10, 4\n"
    :
    : "r" (r)
    : "cc"
  );
}
*/
inline void _hal_flush_d_cache()
{
  uint32_t r = 0;

  __asm__ volatile(
    "mcr 	p15, 0, %0, c7, c14, 0\n"
		"mcr 	p15, 0, %0, c7, c10, 4\n"
    :
    : "r" (r)
    : "cc"
  );
}

static inline void _imb(void)
{
}

static inline void _imb_range(uint32_t u4Start, uint32_t u4End)
{
	int32_t r = 0;
	
    asm volatile(	
		"mcrr	p15, 0, %1, %0, c12\n"
		"mcrr	p15, 0, %1, %0, c5\n"
		"mcr 	p15, 0, %2, c7, c10, 5\n"
		"mcr	p15, 0, %2, c7, c5, 6\n"
		:
		: "r" (u4Start), "r" (u4End), "r"(r)
		: "cc"
    );
}
//==================================================
// Debug functions
//==================================================
#if MT85XX_CACHE_OPERATION_DEBUG  //at mt8530.h
#define cacheline_size 32
uint32_t u4IMP_COP=0;  //improper cache operation

__attribute__((optimize("O0"))) uint32_t BSP_IsCriticalSection(void)
{
  uint32_t r = 0;

  __asm__ volatile(
    "    MRS     %0, CPSR \n"
    :
    : "r"(r)
    : "cc"
  );
  r&=0x80;  // [7]:IRQ [6]FIQ
  return r;
}

__attribute__((optimize("O0"))) void BSP_CACHEOPERATION_CHECK(uint32_t u4Start, uint32_t u4Len, uint32_t u4Limit)
{
	uint32_t u4End;
	u4End=(u4Start + u4Len - 0x1);
	//UN-CACHELINE-ALIGNMENT
	if(u4Start & (cacheline_size-1))
		{
			u4IMP_COP=1;
		}
	if(u4Len & (cacheline_size-1))
		{
			u4IMP_COP=2;
		}	
	//UN_CONTINUOUS
	if((u4Start< 0xC0000000) || (u4End > 0xDFFFFFFF))
		{
			u4IMP_COP=3;
		}
	//HUGE_FLUSH_IN_CRITICAL_SECTIOIN
  if(BSP_IsCriticalSection() && (u4Len > u4Limit))
  	{
  		u4IMP_COP=4;
  	}
	//HUGE_FLUSH_IN_ISR
	if(in_interrupt()&& (u4Len > u4Limit))
		{
			u4IMP_COP=5;
		}
}

#endif
//==================================================
// public functions
//==================================================
#ifdef CONFIG_OUTER_CACHE
static DEFINE_SPINLOCK(bdp_l2x0_lock);

void mtk_flush_L1(uint32_t u4Start, uint32_t u4Len)
{
	if(u4Len > FLUSH_RANGE_MAX)
	{
		flush_cache_all();
	}
	else
	{
		_hal_flush_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
void mtk_flush_L1L2(uint32_t u4Start, uint32_t u4Len)
{
	uint32_t u4End;  
	unsigned long flags;
  u4End=(u4Start + u4Len - 0x1);
	if(u4Len > FLUSH_RANGE_MAX)
	{
		spin_lock_irqsave(&bdp_l2x0_lock, flags);    // Prevent ERRATA 727915
		_hal_clean_d_cache();
		l2x0_flush_all();
		_hal_flush_d_cache();
		spin_unlock_irqrestore(&bdp_l2x0_lock, flags);
	}
	else
	{
		_hal_flush_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
		outer_cache.flush_range(__pa(u4Start), __pa(u4End));
	}
}
void mtk_clean_L1(uint32_t u4Start, uint32_t u4Len)
{
	if(u4Len > CLEAN_RANGE_MAX)
	{
		_hal_clean_d_cache();
	}
	else
	{
		_hal_clean_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
void mtk_clean_L1L2(uint32_t u4Start, uint32_t u4Len)
{
	uint32_t u4End;  
  u4End=(u4Start + u4Len - 0x1);
	if(u4Len > CLEAN_RANGE_MAX)
	{
		_hal_clean_d_cache();
		l2x0_clean_all();
	}
	else
	{
		_hal_clean_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
		outer_cache.clean_range(__pa(u4Start), __pa(u4End));
	}
}

void mtk_inv_L1(uint32_t u4Start, uint32_t u4Len)
{
	if(u4Len > INV_RANGE_MAX)
	{
		flush_cache_all();             //inv all is forbidden
	}
	else
	{
		_hal_inv_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
void mtk_inv_L1L2(uint32_t u4Start, uint32_t u4Len)
{
	unsigned long flags;
	uint32_t u4End;  
  u4End=(u4Start + u4Len - 0x1);
	if(u4Len > INV_RANGE_MAX)
	{
		spin_lock_irqsave(&bdp_l2x0_lock, flags);    // Prevent ERRATA 727915
		_hal_clean_d_cache();
		l2x0_flush_all();
		_hal_flush_d_cache();
		spin_unlock_irqrestore(&bdp_l2x0_lock, flags);
	}
	else                               //PL310 TRM suggest inv L2 then L1
	{
		outer_cache.inv_range(__pa(u4Start), __pa(u4End));       
		_hal_inv_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
void (*mtk_flush_operation)(unsigned long, unsigned long)=mtk_flush_L1;
void (*mtk_clean_operation)(unsigned long, unsigned long)=mtk_clean_L1;
void (*mtk_inv_operation)(unsigned long, unsigned long)=mtk_inv_L1;
void __init BSP_L2Cache_Operation_Init()
{
    	mtk_flush_operation=mtk_flush_L1L2;
    	mtk_clean_operation=mtk_clean_L1L2;
	  	mtk_inv_operation=mtk_inv_L1L2;    	
	  	printk(KERN_INFO "BSP_L2Cache_Operation_Init\n");
}

#endif
void BSP_FlushDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
	#if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,FLUSH_RANGE_MAX);
	#endif
  /* flush data cache range */
	#ifndef CONFIG_OUTER_CACHE       
	if(u4Len > FLUSH_RANGE_MAX)
	{
		flush_cache_all();
	}
	else
	{
		_hal_flush_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
	#else                             
  mtk_flush_operation(u4Start, u4Len);
	#endif
}
EXPORT_SYMBOL(BSP_FlushDCacheRange);

void BSP_CleanDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
  #if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,CLEAN_RANGE_MAX);
	#endif
  /* clean data cache range */
	#ifndef CONFIG_OUTER_CACHE        
	if(u4Len > CLEAN_RANGE_MAX)
	{
		_hal_clean_d_cache();
	}
	else
	{
		_hal_clean_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
	#else                              
  mtk_clean_operation(u4Start, u4Len);
	#endif
}
EXPORT_SYMBOL(BSP_CleanDCacheRange);

void BSP_InvDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
	#if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,INV_RANGE_MAX);
	#endif
  /* inv data cache range */
	#ifndef CONFIG_OUTER_CACHE       
	if(u4Len > INV_RANGE_MAX)
	{
		flush_cache_all();             //inv all is forbidden
	}
	else
	{
		_hal_inv_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
	#else                          
  mtk_inv_operation(u4Start, u4Len);
	#endif
}
EXPORT_SYMBOL(BSP_InvDCacheRange);

/*For DualCore Share data*/
void Core_FlushDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
	#if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,CORE_FLUSH_RANGE_MAX);
	#endif
  /* flush data cache range */
	if(u4Len > CORE_FLUSH_RANGE_MAX)
	{
		flush_cache_all();
	}
	else
	{
		_hal_flush_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
EXPORT_SYMBOL(Core_FlushDCacheRange);

void Core_CleanDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
	#if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,CORE_CLEAN_RANGE_MAX);
	#endif
  /* clean data cache range */
  if(u4Len > CORE_CLEAN_RANGE_MAX)
	{
		_hal_clean_d_cache();
	}
	else
	{
		_hal_clean_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
EXPORT_SYMBOL(Core_CleanDCacheRange);

void Core_InvDCacheRange(uint32_t u4Start, uint32_t u4Len)
{
	#if MT85XX_CACHE_OPERATION_DEBUG
	BSP_CACHEOPERATION_CHECK(u4Start,u4Len,CORE_INV_RANGE_MAX);
	#endif
  /* inv data cache range */
  	if(u4Len > CORE_INV_RANGE_MAX)
	{
		flush_cache_all();             //inv all is forbidden
	}
	else
	{
		_hal_inv_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
	}
}
EXPORT_SYMBOL(Core_InvDCacheRange);

/* Below are the test code*/
// Range operation
void TST_BSP_Clean_L1_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
		  _hal_clean_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
}
EXPORT_SYMBOL(TST_BSP_Clean_L1_DCacheRange);

void TST_BSP_Clean_L2_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
      #ifdef CONFIG_OUTER_CACHE
		  outer_cache.clean_range(__pa(u4Start), __pa(u4End)); 
		  #endif
}
EXPORT_SYMBOL(TST_BSP_Clean_L2_DCacheRange);

void TST_BSP_Invalidate_L1_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
		  _hal_inv_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
}
EXPORT_SYMBOL(TST_BSP_Invalidate_L1_DCacheRange);

void TST_BSP_Invalidate_L2_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
      #ifdef CONFIG_OUTER_CACHE
		  outer_cache.inv_range(__pa(u4Start), __pa(u4End)); 
		  #endif
}
EXPORT_SYMBOL(TST_BSP_Invalidate_L2_DCacheRange);

void TST_BSP_Flush_L1_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
		  _hal_flush_d_cache_range(u4Start, (u4Start + u4Len - 0x1));
}
EXPORT_SYMBOL(TST_BSP_Flush_L1_DCacheRange);

void TST_BSP_Flush_L2_DCacheRange(uint32_t u4Start, uint32_t u4Len) //Test only
{
	    uint32_t u4End;  
      u4End=(u4Start + u4Len - 0x1);
      #ifdef CONFIG_OUTER_CACHE
		  outer_cache.flush_range(__pa(u4Start), __pa(u4End)); 
		  #endif
}
EXPORT_SYMBOL(TST_BSP_Flush_L2_DCacheRange);

// Entire operation
void TST_BSP_Clean_L1_DCache() //Test only
{
		  _hal_clean_d_cache();
}
EXPORT_SYMBOL(TST_BSP_Clean_L1_DCache);

void TST_BSP_Clean_L2_DCache(uint32_t u4Start, uint32_t u4Len) //Test only
{
	 #ifdef CONFIG_OUTER_CACHE
   l2x0_clean_all(); 
   #endif
}
EXPORT_SYMBOL(TST_BSP_Clean_L2_DCache);
/*
void TST_BSP_Invalidate_L1_DCache() //Test only
{
		  _hal_inv_d_cache();
}
EXPORT_SYMBOL(TST_BSP_Invalidate_L1_DCache);
*/
void TST_BSP_Invalidate_L2_DCache(uint32_t u4Start, uint32_t u4Len) //Test only
{
	#ifdef CONFIG_OUTER_CACHE
  l2x0_inv_all(); 
  #endif
}
EXPORT_SYMBOL(TST_BSP_Invalidate_L2_DCache);

void TST_BSP_Flush_L1_DCache() //Test only
{
		  _hal_flush_d_cache();
}
EXPORT_SYMBOL(TST_BSP_Flush_L1_DCache);

void TST_BSP_Flush_L2_DCache(uint32_t u4Start, uint32_t u4Len) //Test only
{
			#ifdef CONFIG_OUTER_CACHE
		  l2x0_flush_all(); 
		  #endif
}
EXPORT_SYMBOL(TST_BSP_Flush_L2_DCache);

void TST_BSP_Flush_L2_DCache_BY_Index(uint32_t u4Start, uint32_t u4Len) //Test only
{
			#ifdef CONFIG_OUTER_CACHE
		  l2x0_flush_all_by_index(); 
		  #endif
}
EXPORT_SYMBOL(TST_BSP_Flush_L2_DCache_BY_Index);
asmlinkage long sys_imb(void)
{
	_imb();
	return 0;
}
EXPORT_SYMBOL(sys_imb);

asmlinkage long sys_imb_range(unsigned long u4Start, unsigned long u4Len)
{
	_imb_range(u4Start, (u4Start + u4Len - 0x1));
	return 0;
}
EXPORT_SYMBOL(sys_imb_range);
