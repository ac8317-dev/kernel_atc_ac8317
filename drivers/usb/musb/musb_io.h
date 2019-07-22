/*
 * MUSB OTG driver register I/O
 *
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (C) 2006-2007 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MUSB_LINUX_PLATFORM_ARCH_H__
#define __MUSB_LINUX_PLATFORM_ARCH_H__

#include <linux/io.h>
#include <linux/spinlock.h>

extern int workaround_counter;
extern spinlock_t workaround_lock;
extern int musbhdrc_rev;

extern unsigned AHBMON;

extern int md_lock(unsigned time_out_cycles);
extern void md_unlock(void);
extern void DISABLE_USB_IRQ(void);

#if 0
#define MD_LOCK_DECLARE()	unsigned long workaround_flag=0;

#define MD_LOCK_IRQ()                                                               				\
if(musbhdrc_rev==2){												\
	do{ 													\
		spin_lock_irqsave(&workaround_lock, workaround_flag);						\
		if(workaround_counter==0){									\
			if(md_lock(4)<0){									\
				DISABLE_USB_IRQ();								\
				spin_unlock_irqrestore(&workaround_lock, workaround_flag);			\
				return IRQ_HANDLED;								\
			}											\
			*(volatile unsigned *)(AHBMON)=0x1;\
		}												\
		workaround_counter++;										\
	       	rmb();												\
	}while(0);												\
}

#define MD_UNLOCK_IRQ() 	   			                                                       	\
if(musbhdrc_rev==2){												\
	do{													\
	        wmb();                                                                  			\
		workaround_counter--;										\
		if(workaround_counter==0){									\
				md_unlock();									\
				*(volatile unsigned *)(AHBMON)=0x4;\
				if(*(volatile unsigned *)(AHBMON+0x4) > 61666){						\
					printk("XXXXXXXXXXXXXX %d %s:%d\n", *(volatile unsigned *)(AHBMON+0x4), __FILE__, __LINE__);\
				}										\
		}												\
		spin_unlock_irqrestore(&workaround_lock, workaround_flag);					\
	}while(0);												\
}

#define MD_LOCK()                                                               				\
if(musbhdrc_rev==2){												\
	do{ 													\
		spin_lock_irqsave(&workaround_lock, workaround_flag);						\
		if(workaround_counter==0){									\
			md_lock(0);										\
			*(volatile unsigned *)(AHBMON)=0x1;\
		}												\
		workaround_counter++;										\
	       	rmb();												\
	}while(0);												\
}
#elif 1
#define MD_LOCK_DECLARE()	unsigned long workaround_flag=0;

#define MD_LOCK_IRQ()                                                               				\
{												\
	do{ 													\
		spin_lock_irqsave(&workaround_lock, workaround_flag);						\
		workaround_counter++;										\
	       	rmb();												\
	}while(0);												\
}

#define MD_UNLOCK_IRQ() 	   			                                                       	\
{												\
	do{													\
	        wmb();                                                                  			\
		workaround_counter--;										\
		spin_unlock_irqrestore(&workaround_lock, workaround_flag);					\
	}while(0);												\
}

#define MD_LOCK()                                                               				\
{												\
	do{ 													\
		spin_lock_irqsave(&workaround_lock, workaround_flag);						\
		workaround_counter++;										\
	       	rmb();												\
	}while(0);												\
}

#else
#define MD_LOCK_DECLARE()
#define MD_LOCK_IRQ()
#define MD_UNLOCK_IRQ() 	   			                                                       	
#define MD_LOCK()                                                               				
#endif

extern void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *src);

#define MD_UNLOCK() MD_UNLOCK_IRQ()

#if !defined(CONFIG_ARM) && !defined(CONFIG_SUPERH) \
	&& !defined(CONFIG_AVR32) && !defined(CONFIG_PPC32) \
	&& !defined(CONFIG_PPC64) && !defined(CONFIG_BLACKFIN)
static inline void readsl(const void __iomem *addr, void *buf, int len)
	{ insl((unsigned long)addr, buf, len); }
static inline void readsw(const void __iomem *addr, void *buf, int len)
	{ insw((unsigned long)addr, buf, len); }
static inline void readsb(const void __iomem *addr, void *buf, int len)
	{ insb((unsigned long)addr, buf, len); }

static inline void writesl(const void __iomem *addr, const void *buf, int len)
	{ outsl((unsigned long)addr, buf, len); }
static inline void writesw(const void __iomem *addr, const void *buf, int len)
	{ outsw((unsigned long)addr, buf, len); }
static inline void writesb(const void __iomem *addr, const void *buf, int len)
	{ outsb((unsigned long)addr, buf, len); }

#endif

#ifndef CONFIG_BLACKFIN

/* NOTE:  these offsets are all in bytes */

static inline u16 musb_readw(const void __iomem *addr, unsigned offset)
{ 
	//if(workaround_counter<=0){ printk("musb_readw %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
	//return(__raw_readw(addr + offset)); 
    //ben for test
    return ioread16((void *)(addr + offset));
}

static inline u32 musb_readl(const void __iomem *addr, unsigned offset)
{
	//if(workaround_counter<=0){ printk("musb_readl %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
	//return(__raw_readl(addr + offset));
    //ben for test
    return ioread32((void *)(addr + offset));
}

static inline void musb_writew(void __iomem *addr, unsigned offset, u16 data)
{
	//if(workaround_counter<=0){ printk("musb_writew %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
	//__raw_writew(data, addr + offset); 
    //ben for test
    //iowrite16(data, (void *)(addr + offset));

    volatile uint32_t i4TmpVar;
	
    i4TmpVar = ioread32((void *)(addr + ((offset) & 0xFFFFFFFC)));
    i4TmpVar &= ~(((uint32_t)0xFFFF) << (8*((offset) & 0x03)));    
    i4TmpVar |= (uint32_t)((data) << (8*((offset) & 0x03)));

    iowrite32(i4TmpVar, (void *)(((uint32_t)addr) + ((offset) & 0xFFFFFFFC)));
}

static inline void musb_writel(void __iomem *addr, unsigned offset, u32 data)
{
	//if(workaround_counter<=0){ printk("musb_writel %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
	//__raw_writel(data, addr + offset); 
    //ben for test
    iowrite32(data, (void *)(addr + offset));
}

#ifdef CONFIG_USB_TUSB6010

/*
 * TUSB6010 doesn't allow 8-bit access; 16-bit access is the minimum.
 */
static inline u8 musb_readb(const void __iomem *addr, unsigned offset)
{
	u16 tmp;
	u8 val;

	tmp = __raw_readw(addr + (offset & ~1));
	if (offset & 1)
		val = (tmp >> 8);
	else
		val = tmp & 0xff;

	return val;
}

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	u16 tmp;

	tmp = __raw_readw(addr + (offset & ~1));
	if (offset & 1)
		tmp = (data << 8) | (tmp & 0xff);
	else
		tmp = (tmp & 0xff00) | data;

	__raw_writew(tmp, addr + (offset & ~1));
}

#else

static inline u8 musb_readb(const void __iomem *addr, unsigned offset)
{
	//if(workaround_counter<=0){ printk("musb_readb %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
    //    return (__raw_readb(addr + offset));
    //ben for test
    return ioread8((void *)(addr + offset));
}

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)
{
	//if(workaround_counter<=0){ printk("musb_writeb %p %d\n", musb_write_fifo, workaround_counter); *(volatile unsigned *)0=0;}
    //    __raw_writeb(data, addr + offset);
    //ben for test
    //iowrite8(data, (void *)(addr + offset));
    volatile uint32_t i4TmpVar;
	
    i4TmpVar = ioread32((void *)(addr + ((offset) & 0xFFFFFFFC)));
    i4TmpVar &= ~(((uint32_t)0xFF) << (8*((offset) & 0x03)));    
    i4TmpVar |= (uint32_t)(((data) & 0xFF) << (8*((offset) & 0x03)));

    iowrite32(i4TmpVar, (void *)(((uint32_t)addr) + ((offset) & 0xFFFFFFFC)));
}

#endif	/* CONFIG_USB_TUSB6010 */

#else

static inline u8 musb_readb(const void __iomem *addr, unsigned offset)
	{ return (u8) (bfin_read16(addr + offset)); }

static inline u16 musb_readw(const void __iomem *addr, unsigned offset)
	{ return bfin_read16(addr + offset); }

static inline u32 musb_readl(const void __iomem *addr, unsigned offset)
	{ return (u32) (bfin_read16(addr + offset)); }

static inline void musb_writeb(void __iomem *addr, unsigned offset, u8 data)
	{ bfin_write16(addr + offset, (u16) data); }

static inline void musb_writew(void __iomem *addr, unsigned offset, u16 data)
	{ bfin_write16(addr + offset, data); }

static inline void musb_writel(void __iomem *addr, unsigned offset, u32 data)
	{ bfin_write16(addr + offset, (u16) data); }

#endif /* CONFIG_BLACKFIN */

#endif
