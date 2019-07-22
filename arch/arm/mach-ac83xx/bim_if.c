/********************************************************************************************
 *     LEGAL DISCLAIMER 
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES 
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED 
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS 
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED, 
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR 
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY 
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, 
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK 
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION 
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *     
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH 
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, 
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE 
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE. 
 *     
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS 
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.  
 ************************************************************************************************/

//============================================================================
// Include files
//============================================================================
#include <linux/module.h> 
#include <mach/ac83xx_basic.h>

uint32_t u4HWSemWaitCount = 0;
//extern uint32_t u4HWSemWaitCount ;

#ifndef NOHWSEMAPHORE
bool BIM_GETHWSemaphore(uint32_t u4Number, uint32_t u4TimeOut)
{
  uint32_t u4Count; 
  u4Count = 0;

  if(u4TimeOut == 0)
  {
    do
    {
      BIM_WRITE32(REG_RW_HSMPHE, u4Number);
      u4HWSemWaitCount++;    
    }
    while(((BIM_REG32(REG_RW_HSMPHE) & u4Number) != u4Number));  
    
    return true;
  }	
  else
  {
    do
    {
      BIM_WRITE32(REG_RW_HSMPHE, u4Number);
      u4Count++;
      u4HWSemWaitCount++;    
    }
    while(((BIM_REG32(REG_RW_HSMPHE) & u4Number) != u4Number) && (u4TimeOut > u4Count));
  
    if(u4TimeOut <= u4Count)
    {
      return false;
    }  	
    else
    {
      return true;
    }	
  } 
}
#else
bool BIM_GETHWSemaphore(uint32_t u4Number, uint32_t u4TimeOut)
{
  return true;
}
#endif // !NOHWSEMAPHORE

EXPORT_SYMBOL(BIM_GETHWSemaphore);

bool BIM_ReleaseHWSemaphore(uint32_t u4Number)
{
  BIM_WRITE32(REG_RW_HSMPHE, u4Number);
  return true;  
}

EXPORT_SYMBOL(BIM_ReleaseHWSemaphore);

/*
bool BIM_IsIrqPending(uint32_t u4Vector)
{
  u32 regval32;
  unsigned long flags;
	
  if (!_bim_is_vector_valid(u4Vector))
  {
       return false;
  }

  local_irq_save(flags);
  
  if(u4Vector < 32)
  {
	  regval32 = BIM_REG32(REG_IRQST);
	  regval32 &= (1 << u4Vector);
  }
  else if((u4Vector >= 32) && (u4Vector < 64))
  {
	  u4Vector = u4Vector - 32;
	  regval32 = BIM_REG32(REG_IRQST2);
	  regval32 &= (1 << u4Vector);
  }
  else if((u4Vector >= 64) && (u4Vector < 96))
  {
	  u4Vector = u4Vector - 64;
	  regval32 = BIM_REG32(REG_IRQST3);
	  regval32 &= (1 << u4Vector);
  }
  else
  {
	  // pdwn extend 
	  u4Vector = u4Vector - 96;
	  regval32 = PDWNC_READ32(REG_RW_INTSTA);
	  regval32 &= (1 << u4Vector);
  }
  
  local_irq_restore(flags);

  return (regval32 != 0); 
}
EXPORT_SYMBOL(BIM_IsIrqPending);
*/
