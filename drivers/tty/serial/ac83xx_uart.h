#ifndef __DRV_SER_AC83XX_UART_H
#define __DRV_SER_AC83XX_UART_H

#include <mach/hardware.h>
#include <linux/slab.h>

#define PORT_AC83XX         36

#define UART_PORT_1		1
#define UART0_BASE              0x000           // u1 base
#define UART1_BASE              0x040           // u1  base
#define UART2_BASE              0x0c0           // u2  base
#define UART3_BASE              0x100           // u3  base
#define UART4_BASE              0x140           // u4  base
#define UART5_BASE              0x180           // u5  base
#define UART6_BASE              0x1C0           // u6  base

#define R_RD_ALLOW		(1 << 0)        // can read
#define R_WR_ALLOW		(1 << 1)        // can write
#define R_END_ERR		(1 << 4)        // rx error (end)
#define R_PARITY_ERR		(1 << 5)        // rx error (parity)
#define R_TRANSPARENT		(1 << 6)        // rs232 transparent mode
#define R_SET_TP		0xE2 // Set TP mode.
#define R_SET_NORMAL		0x40 // Set Normal mode. not equal 0xE2.
#define R_BAUD_RATE_MASK	0x0700
#define R_BAUD_RATE_OFFSET	8            
#define R_SET_BAUD(x)		(((x) << R_BAUD_RATE_OFFSET) & R_BAUD_RATE_MASK)
#define R_GET_BAUD(x)		(((x) & R_BAUD_RATE_MASK) >> R_BAUD_RATE_OFFSET)	
#define R_BAUD_115200		0	// 000B
#define R_BAUD_230400		1	// 001B
#define R_BAUD_460800		2	// 010B
#define R_BAUD_921600		3	// 011B
#define R_BAUD_57600		4	// 100B
#define R_BAUD_28800		5	// 101B
#define R_BAUD_9600		6	// 110B
#define R_BUAD_CUST		7	// 111B
#define R_RATE_DIVISOR_MASK	0x00FF0000        
#define R_RATE_DIVISOR_OFFSET	16        
#define R_SET_RATE_DIVISOR(x)	(((x) << R_RATE_DIVISOR_OFFSET) & R_RATE_DIVISOR_MASK)
#define R_GET_RATE_DIVISOR(x)	(((x) & R_RATE_DIVISOR_MASK) >> R_RATE_DIVISOR_OFFSET)	
#define R_RXBUF_EN		(1 << 7)   
#define R_TX_ACCEPT_EN    	(1 << 6)          
#define R_TXBUF_EN     		(1 << 5) 
#define RS232_INT_STA           0x010       // interrupt identification   
#define R_RXBUF_STA       	(1 << 7)   // rs232 rx buffer full
#define R_TX_ACCEPT_STA    	(1 << 6)   // rs232 TXD Data is accepted by RFI
#define R_TXBUF_STA       	(1 << 5)   // rs232 tx buffer empty 

#define UART0_COMMCTRL		0x004           // u0 comm control

#define RS232_DATA		0x000           // rs232 data
#define RS232_STATUS		0x004           // old status
#define RS232_INT_EN         	0x00C       // interrupt enable

#define RS232_READ32(offset)		    IO_READ32(RS232_BASE_VA, (offset))
#define RS232_WRITE32(offset, value)	    IO_WRITE32(RS232_BASE_VA, (offset), (value))

#define RS232_SET_BIT(offset, Bit)          RS232_WRITE32(offset, RS232_READ32(offset) | (Bit))
#define RS232_CLR_BIT(offset, Bit)          RS232_WRITE32(offset, RS232_READ32(offset) & (~(Bit)))

#define UART_PORT_OFFSET 	0x0040
#define UART_BASE(port)		(RS232_BASE_VA + ((port==UART_PORT_0) ? UART_PORT_OFFSET : (UART_PORT_OFFSET <<1)))

#define UART_DATA_BYTE		0x000	// 0x040, 0x0C0, 0x100, 0x140, 0x180, 0x1C0
#define UART_DATA_DWORD		0x004	// 0x044, 0x0C4, 0x104, 0x144, 0x184, 0x1C4
#define UART_CTRL_REG		0x008   // 0x048, 0x0C8, 0x108, 0x148, 0x188, 0x1C8
#define U_BAUD_RATE_MASK		0x0F00        
#define U_BAUD_RATE_OFFSET	 	8        
#define U_SET_BAUD(x)      		(((x) << U_BAUD_RATE_OFFSET) & U_BAUD_RATE_MASK)
#define U_GET_BAUD(x)      		(((x) & U_BAUD_RATE_MASK) >> U_BAUD_RATE_OFFSET)	
#define U_BAUD_115200	   	0	// 0000B
#define U_BAUD_230400	   	1	// 0001B
#define U_BAUD_460800	   	2	// 0010B
#define U_BAUD_921600	   	3	// 0011B
#define U_BAUD_57600       	4	// 0100B
#define U_BAUD_38400       	5	// 0101B
#define U_BAUD_19200       	6	// 0110B
#define U_BUAD_9600 		7	// 0111B
#define U_BAUD_4800       	8	// 1000B
#define U_BAUD_2400       	9	// 1001B
#define U_BUAD_1200 		10	// 1010B
#define U_BUAD_300	 	11	// 1011B
#define U_BAUD_110       	12	// 1100B
#define U_BAUD_TEST       	13	// 1101B

#define U_DMA_EN				(1 <<24)

#define UART_STA_REG			0x00C  
#define U_RX_BUF_NUM_MASK(port)		((port==UART_PORT_1) ? 0x07FF : 0x01FF)
#define U_GET_RX_BUF_NUM(port, x)	((x) & U_RX_BUF_NUM_MASK(port))
#define U_TX_BUF_SLOT_MASK(port)	((port==UART_PORT_1) ? 0x01FFC000 : 0x001FF000)
#define U_TX_BUF_SLOT_OFFSET(port)	((port==UART_PORT_1) ? 14 : 12)
#define U_GET_TX_BUF_SLOT(port, x)	(((x) & U_TX_BUF_SLOT_MASK(port)) >>U_TX_BUF_SLOT_OFFSET(port))
#define UART1_FIFO_SIZE         	2048
#define UART_FIFO_SIZE          	512


#define U_RXD_TRIG_LEVEL(port)		((port==UART_PORT_1) ? 0x07FF  : 0x01FF)
#define U_W_CLR_RXBUF 			(1 << 16)
#define U_W_CLR_TXBUF			(1 << 17)
#define U_W_TIMEOUT_CYCLE_MASK		(0x0FFF0000)
#define U_SET_TIMEOUT_CYCLE(x)  	(((x) & 0x0FFF) <<16)
#define U_GET_TIMEOUT_CYCLE(port, x) 	(((x) >>((port==UART_PORT_1 ? 14 : 12))& 0x0FFF)

#define UART_INT_EN			0x018
#define U_RXERR_EN			(1 << 0)   
#define U_RXBUF_EN     			(1 << 1)      
#define U_TXOUT_EN     			(1 << 2) 
#define U_TXBUF_EN			(1 << 3)             
#define U_OVRUN_EN			(1 << 4)
#define U_DMA_R_EN			(1 << 5)             
#define U_DMA_W_EN			(1 << 6)
#define UART_INTALL         		0x1F

#define U_RXERR_STA			(1 << 0)   
#define U_RXBUF_STA		    	(1 << 1)          
#define U_TIMEOUT_STA     		(1 << 2)
#define U_TXBUF_STA			(1 << 3)             
#define U_OVRUN_STA			(1 << 4)	
#define U_DMA_R_STA			(1 << 5)             
#define U_DMA_W_STA			(1 << 6)

#define UART_DMA_WFIFO_CTRL		0x020   // 0x060, 0x0E0, 0x120, 0x160, 0x1A0, 0x1E0
#define U_LAST_WDATA_EN			(1 << 0)
#define U_GET_START_MOVE_DATA(port)	(((UART_READ32(port, UART_DMA_WFIFO_CTRL)) >>1) & 0x01)	// Read Only
#define U_GET_WFIFO_REQ_CNT(x)		(((x) >>3) & 0x1F)
#define U_WFIFO_TRIG_LEVEL_MASK		0x1F
#define U_GET_WFIFO_TRIG_LEVEL(x)	(((x) >>8) & 0x1F)	
#define U_SET_WFIFO_TRIG_LEVEL(x)	(((x) & 0x1F) <<8)
#define U_GET_WFIFO_SIZE(x)		(((x) >>16) & 0x1F)	
#define U_SET_WFIFO_SIZE(x)		(((x) & 0x1F) <<16)
#define U_WFIFO_BURST_LEN_MASK		0x1F
#define U_GET_WFIFO_BURST_LEN(x)	(((x) >>24) & 0x1F)	
#define U_SET_WFIFO_BURST_LEN(x)	(((x) & 0x1F) <<24)
#define UART_DMA_WFIFO_ADDR		0x024   // 0x064, 0x0E4, 0x124, 0x164, 0x1A4, 0x1E4
#define U_DMA_WFIFO_ADDR(x)		((x) & 0x01FFFFFF)

#define UART_DMA_RFIFO_CTRL		0x028   // 0x068, 0x0E8, 0x128, 0x168, 0x1A8, 0x1E8
#define U_DATA_READY			(1 << 0)
#define U_INHIBIT_REG_TRIG		(1 << 2)
#define U_INHIBIT_SEND			(1 << 3)
#define U_GET_RFIFO_EMPTY_BYTE_NUM(x)	(((x) >>4) & 0x0F)	
#define U_SET_RFIFO_EMPTY_BYTE_NUM(x)	(((x) & 0x0F) <<4)
#define U_RFIFO_TRIG_LEVEL_MASK		0x1F
#define U_GET_RFIFO_TRIG_LEVEL(x)	(((x) >>8) & 0x1F)	
#define U_SET_RFIFO_TRIG_LEVEL(x)	(((x) & 0x1F) <<8)
#define U_GET_RFIFO_SIZE(x)		(((x) >>16) & 0x1F)	
#define U_SET_RFIFO_SIZE(x)		(((x) & 0x1F) <<16)
#define U_GET_RFIFO_BURST_LEN(x)	(((x) >>24) & 0x1F)	
#define U_SET_RFIFO_BURST_LEN(x)	(((x) & 0x1F) <<24)
#define UART_DMA_RFIFO_ADDR		0x02C   // 0x06C, 0x0EC, 0x12C, 0x16C, 0x1AC, 0x1EC
#define U_DMA_RFIFO_ADDR(x)		((x) & 0x01FFFFFF)

#define UART_TRAN_DATA_NUM		0x030   //0X70, 0Xf0
/*TODO

#define UART_READ32(port, offset)	    IO_READ32(UART_BASE(port)+(UART_PORT_OFFSET*(port)), (offset))
#define UART_WRITE32(port, offset, value)   IO_WRITE32(UART_BASE(port)+(UART_PORT_OFFSET*(port)), (offset), (value))

#define UART_SET_BIT(port, offset, Bit)		UART_WRITE32(port, offset, UART_READ32(port, offset) | (Bit))
#define UART_CLR_BIT(port, offset, Bit)		UART_WRITE32(port, offset, UART_READ32(port, offset) & (~(Bit)))
*/
#define DMA_MODE_INT_ENABLE_VALUE     0x75
#define DMA_MODE_INT_CLEAR_VALUE      0xFF

#define DMA_WFIFO_REQ_BUSRT_LENGTH	(0x10 << 24)
#define DMA_WFIFO_TRIGGER_LEVEL		(0x10 << 8)
#define DMA_WFIFO_SIZE_ON_MEM		(0x1F << 16)
#define UART_DMA_MAX_BURST_LEN		(0x1F)
#define UART_DMA_MAX_TRIG_LEVEL		(0x1F)
#define UART_DMA_MAX_MEM_SIZE		(0x1F)
#define UART_DMA_MAX_BYTE_NUM		((UART_DMA_MAX_BURST_LEN*UART_DMA_MAX_TRIG_LEVEL* 16))
#define UART_DMA_MODE_ENABLE		(0x1 << 24)
#define UART_CLEAR_RX_BUFFER		(1 << 16)
#define UART_CLEAR_TX_BUFFER		(1 << 17)

#define UART0_INT_EN             0x00c       // interrupt enable
#define UART0_INT_STATUS         0x010       // interrupt INT ID

#define U0_RBUF     		(1 << 7)   // u0 rx buffer full
#define U0_TOUT			(1 << 6)   // u0 TXD Data is accepted by RFI
#define U0_TBUF			(1 << 5)   // u0 tx buffer empty
#define U0_INTALL		(U0_TOUT | U0_RBUF)

/* uart 0 / debug port */
#define UART0_DATA              0x000           // u0 data
#define UART0_DATA_BYTE         (UART0_DATA)    // u0 byte / word data
#define UART0_DATA_WORD         (UART0_DATA)

#define UART0_STATUS        	0x004           // old status
#define U0_RD_ALLOW             (1 << 0)        // can read
#define U0_WR_ALLOW             (1 << 1)        // can write
#define U0_END_ERR              (1 << 4)        // rx error (end)
#define U0_PARITY_ERR           (1 << 5)        // rx error (parity)
#define U0_TRANSPARENT          (1 << 6)        // uart 0 transparent mode

#define SETBAUD(x)      (((x) & 0xf) << 8)
#define GETBAUD(x)      (((x) >> 8) & 0xf)

#define U_RATE_DIVISOR_MASK 	0x00FFF000		  
#define U_RATE_DIVISOR_OFFSET	12		  
#define U_SET_RATE_DIVISOR(x)	(((x) << U_RATE_DIVISOR_OFFSET) & U_RATE_DIVISOR_MASK)
#define U_GET_RATE_DIVISOR(x)	(((x) & U_RATE_DIVISOR_MASK) >> U_RATE_DIVISOR_OFFSET)	
	
#define BAUD_X1         0
#define BAUD_X2         1
#define BAUD_X4         2
#define BAUD_X8         3
#define BAUD_57600      4
#define BAUD_38400      5
#define BAUD_19200      6
#define BAUD_9600       7
#define BAUD_4800       8
#define BAUD_2400       9
#define BAUD_1200       10
#define U_BUAD_300	11	// 1011B
#define U_BAUD_110	12	// 1100B
#define U_BAUD_TEST     13	// 1101B

    
#define BAUD_115200     BAUD_X1
#define BAUD_230400     BAUD_X2
#define BAUD_460800     BAUD_X4
#define BAUD_921600     BAUD_X8

#define UART_DATA_WORD         (0x04)
#define UART_COMMCTRL          (0x08)

#define UART_STATUS             (0x0c)
#define UART1_FIFO_SIZE         2048
#define UART1_RX_BUF_SIZE       UART1_FIFO_SIZE
#define UART1_TX_BUF_SIZE       UART1_FIFO_SIZE
#define UART1_RX_BUF_MASK       (UART1_RX_BUF_SIZE - 1)
#define UART1_TX_BUF_MASK       ((UART1_TX_BUF_SIZE-1) << 14)

#define UART1_PARITY_ERR              (1 <<  11)
#define UART1_END_ERR                 (1 <<  12)
#define UART1_BREAK_ERR               (1 <<  13)

#define UART_FIFO_SIZE          512
#define RX_BUF_SIZE             UART_FIFO_SIZE
#define TX_BUF_SIZE             UART_FIFO_SIZE
#define RX_BUF_MASK             (RX_BUF_SIZE - 1)
#define TX_BUF_MASK             ((TX_BUF_SIZE-1) << 12)
    
#define UART_PARITY_ERR              (1 <<  9)
#define UART_END_ERR                 (1 <<  10)
#define UART_BREAK_ERR               (1 <<  11)

#define UART_BUF_CTRL            (0x10)           // u0 buffer control
#define RX_TRIG_LVL(x)          ((x) & 0x1f)
#define RX_TOUT_CYCLE(x)        (((x) & 0xff) << 8)
#define CLEAR_RBUF              (1 << 6)
#define CLEAR_TBUF              (1 << 7)

#define UART_INT_STA			(0x1C)        //u1 interrupt status
#define RXD_ERRE1       (1 <<  0)
#define RBUF_E1         (1 <<  1)
#define TOUT_E1         (1 <<  2)
#define TBUF_E1         (1 <<  3)
#define OVERRUN_EN1     (1 <<  4)


/* system level, not related to hardware */
#define UST_FRAME_ERROR     (1 << 0)
#define UST_PARITY_ERROR    (1 << 1)
#define UST_BREAK_ERROR     (1 << 2)
#define UST_OVRUN_ERROR     (1 << 3)
#define UST_DUMMY_READ      (1 << 31)

#endif /* __DRV_SER_AC83XX_UART_H */

