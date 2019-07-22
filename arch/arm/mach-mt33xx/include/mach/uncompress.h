/*
 *  linux/include/asm-arm/arch-mt85xx/uncompress.h
 *
 *  Copyright (C) 2009 MediaTek Inc.
 *
 */

#ifndef __ARCH_ARM_UNCOMPRESS_H
#define __ARCH_ARM_UNCOMPRESS_H


#define IO_BASE             	    0x70000000
#define SERIAL_BASE         	    (IO_BASE + 0xc000)
#define REG_SER_PORT                  (*((volatile unsigned char*)(SERIAL_BASE + 0x00)))
#define REG_SER_STATUS                (*((volatile unsigned char*)(SERIAL_BASE + 0x04)))
  #define SER_READ_ALLOW                0x1
  #define SER_WRITE_ALLOW               0x2


/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay (unsigned long loops)
{
        __asm__ volatile ("1:\n" "subs %0, %1, #1\n"
                "bne 1b":"=r" (loops):"0" (loops));
}

/*
 * This does not append a newline
 */
 
static inline void SerPollPutChar(int cc)
{
  while((REG_SER_STATUS & SER_WRITE_ALLOW) != SER_WRITE_ALLOW) {}

  REG_SER_PORT = cc;
}
 
static inline void putc(int c)
{
    static int prev = 0;
    
	if ((c < ' ') && (c != '\r') && (c != '\n') && (c != '\t') && (c != '\b'))
	{
		return;
	}

	if ((c == '\n') && (prev != '\r'))
	{
		SerPollPutChar('\r');
	}
	
	prev = c;
	
    SerPollPutChar(c);
}

static inline void flush(void)
{
    /* wait for empty slot */
    //delay(5000); 
    return;
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()

static inline void putc1()
{
	putc('1');
}



#endif /* __ARCH_ARM_UNCOMPRESS_H */

