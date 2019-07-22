
/*
 * 
 *
 * Copyright (C) 2011 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 * 
 *
 *  
 *
 */


#ifndef __DRV_SER_MT33XX_UART_H
#define __DRV_SER_MT33XX_UART_H

#include <mach/hardware.h>


/* uart general */
/*
#define UART_FLASH_EN           0x008       // flash enable
    #define FLASH_EN_CMD            0x3c        // enable flash
    #define FLASH_DI_CMD            0x0         // disable flash
    #define FLASH_EN_BIT            0x1         // check bit
*/
#define UART0_INT_EN             0x00c       // interrupt enable
#define UART0_INT_STATUS         0x010       // interrupt INT ID

//    #define U1_RERR     (1 <<  0)   // u1 rx error
                                    // parity, end, overrun and break
//    #define U1_RBUF     (1 <<  1)   // u1 rx buffer full
//    #define U1_TOUT     (1 <<  2)   // u1 rx time out
//    #define U1_TBUF     (1 <<  3)   // u1 tx buffer empty
//    #define U1_MODEM    (1 <<  4)   // u1 modem state change
//    #define U1_OVRUN    (1 <<  5)   // u1 rx buffer overflow
//    #define U0_RERR     (1 << 16)   // u0 rx error
    #define U0_RBUF     (1 << 7)   // u0 rx buffer full
    #define U0_TOUT     (1 << 6)   // u0 TXD Data is accepted by RFI
    #define U0_TBUF     (1 << 5)   // u0 tx buffer empty
//    #define U0_MODEM    (1 << 20)   // u0 modem state change
//    #define U0_OVRUN    (1 << 21)   // u0 rx buffer overflow
//    #define U0_RISCD    (1 << 22)   // u0 debug port risc data in
//    #define U1_INTALL   (U1_TOUT | U1_RERR | U1_RBUF)
  
//    #define U0_INTALL   (U0_TOUT | U0_RERR | U0_RBUF)
    #define U0_INTALL   (U0_TOUT | U0_RBUF)

#define SER01_WRITE_ALLOW             (0x1<<3)
#define SER01_READ_ALLOW              (0x1<<1)
    
#define REG_SER_INT_EN              0x0c       // Interrupt enable register
#define REG_SER_INT_STATUS          0x10       // Interrupt status register

#define INT_SER1_TX_EMPTY           (0x1<<3)
#define INT_SER1_READ_ALLOW         (0x1<<1)

#define INT_SER1_RX_TOUT            (0x1<<2)
#define INT_SER1_RX_ERR             (0x1<<0)
#define INT_SER1_RX_OVERFLOW        (0x1<<4)



/* uart 0 / debug port */
#define UART0_DATA              0x000           // u0 data
#define UART0_DATA_BYTE         (UART0_DATA)    // u0 byte / word data
#define UART0_DATA_WORD         (UART0_DATA)

//confirmed by cmtu:
#define UART0_STATUS        0x004           // old status
    #define U0_RD_ALLOW             (1 << 0)        // can read
    #define U0_WR_ALLOW             (1 << 1)        // can write
    #define U0_END_ERR              (1 << 4)        // rx error (end)
    #define U0_PARITY_ERR           (1 << 5)        // rx error (parity)
    #define U0_TRANSPARENT          (1 << 6)        // uart 0 transparent mode

/*
#define UART0_BUFFER_SIZE       0x01c           // u0 buffer size selection
    #define U0_FIFO_SIZE            (32 - 1)        // 32 bytes
*/

#define UART0_COMMCTRL          0x004           // u0 comm control

    /* baud */
#define U_BAUD_RATE_MASK		0x00000F00  

    #define SETBAUD(x)      (((x) & 0xf) << 8)
    #define GETBAUD(x)      (((x) >> 8) & 0xf)

#define U_RATE_DIVISOR_MASK 	0x00FFF000		  
#define U_RATE_DIVISOR_OFFSET	12		  
#define U_SET_RATE_DIVISOR(x)	(((x) << U_RATE_DIVISOR_OFFSET) & U_RATE_DIVISOR_MASK)
#define U_GET_RATE_DIVISOR(x)	(((x) & U_RATE_DIVISOR_MASK) >> U_RATE_DIVISOR_OFFSET)	
#define U_DMA_EN				(1 <<24)

	
	
    #define BAUD_MASK       (0xf << 8)
    #define BAUD_X1         0
    #define BAUD_X2         1
    #define BAUD_X4         2
    #define BAUD_X8         3
    #define BAUD_57600      4
	//#define BAUD_28800      5
    #define BAUD_38400      5
	#define BAUD_19200      6
    #define BAUD_9600       7
	#define BAUD_4800       8
	#define BAUD_2400       9
	#define BAUD_1200       10
    #define U_BUAD_300	    11	// 1011B
    #define U_BAUD_110	    12	// 1100B
    #define U_BAUD_TEST     13	// 1101B

	
    #define BAUD_115200     BAUD_X1
    #define BAUD_230400     BAUD_X2
    #define BAUD_460800     BAUD_X4
    #define BAUD_921600     BAUD_X8

    /* parity check */
/*    
    #define PARITY_MASK     (3 << 4)
    #define PARITY_NONE     (0 << 4)
    #define PARITY_ODD      (1 << 4)
    #define PARITY_EVEN     (3 << 4)
    #define IS_PARITY(REG,FLAG)     ((REG) & (FLAG) == (FLAG))
*/    
    /* break */
//    #define CONTROL_BREAK   (1 << 7)
#define UART0_BASE              0x000           // u1 base
#define UART1_BASE              0x040           // u1  base
#define UART2_BASE              0x0c0           // u2  base
#define UART3_BASE              0x100           // u3  base
#define UART4_BASE              0x140           // u4  base
#define UART5_BASE              0x180           // u5  base
#define UART6_BASE              0x1C0           // u6  base


#define UART_DATA              (0x00)           // ux data
#define UART_DATA_BYTE         (0x00)          // ux byte data
#define UART_DATA_WORD         (0x04)          //ux word data
#define UART_COMMCTRL          (0x08)           // ux comm control


#define UART_STATUS                (0x0c)           // u0 status
    // uart1
    #define UART1_FIFO_SIZE         2048
	#define UART1_RX_BUF_SIZE       UART1_FIFO_SIZE
    #define UART1_TX_BUF_SIZE       UART1_FIFO_SIZE
    #define UART1_RX_BUF_MASK       (UART1_RX_BUF_SIZE - 1)
    #define UART1_TX_BUF_MASK       ((UART1_TX_BUF_SIZE-1) << 14)

	#define UART1_PARITY_ERR              (1 <<  11)
    #define UART1_END_ERR                 (1 <<  12)
    #define UART1_BREAK_ERR               (1 <<  13)

	
	//uart2 ~uart 6
    #define UART_FIFO_SIZE          512
    #define RX_BUF_SIZE             UART_FIFO_SIZE
    #define TX_BUF_SIZE             UART_FIFO_SIZE
    #define RX_BUF_MASK             (RX_BUF_SIZE - 1)
    #define TX_BUF_MASK             ((TX_BUF_SIZE-1) << 12)
	
    #define UART_PARITY_ERR              (1 <<  9)
    #define UART_END_ERR                 (1 <<  10)
    #define UART_BREAK_ERR               (1 <<  11)

#define UART_BUFCTRL               (0x10)           // u0 buffer control
    #define RX_TRIG_LVL(x)          ((x) & 0x1f)
    #define RX_TOUT_CYCLE(x)        (((x) & 0xff) << 8)
    #define CLEAR_RBUF              (1 << 6)
    #define CLEAR_TBUF              (1 << 7)
    #define BUFCTRL_INIT            (CLEAR_TBUF | CLEAR_RBUF | RX_TOUT_CYCLE(0xf) | RX_TRIG_LVL(0x1a))


#define UART_INT_EN               (0x18)          //u1 interrupt enable

#define UART_INT_STATUS           (0x1C)        //u1 interrupt status
    #define RXD_ERRE1       (1 <<  0)
    #define RBUF_E1         (1 <<  1)
    #define TOUT_E1         (1 <<  2)
    #define TBUF_E1         (1 <<  3)
    #define OVERRUN_EN1     (1 <<  4)


#define UART_INTALL   0x1F//(TBUF_E1 | RBUF_E1)
//#define U1_INTALL   (RBUF_E1)

/* others */
    /* system level, not related to hardware */
    
    #define UST_FRAME_ERROR     (1 << 0)
    #define UST_PARITY_ERROR    (1 << 1)
    #define UST_BREAK_ERROR     (1 << 2)
    #define UST_OVRUN_ERROR     (1 << 3)
    #define UST_DUMMY_READ      (1 << 31)


#define PORT_MT33XX         36


#endif /* __DRV_SER_MT33XX_UART_H */

