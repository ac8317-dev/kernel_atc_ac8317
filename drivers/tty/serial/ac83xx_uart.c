#if defined(CONFIG_SERIAL_AC83XX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <mach/ac83xx_gpio_pinmux.h>
#include <mach/pinmux.h>
#include "x_ver.h"
#include "ac83xx_uart.h"
static unsigned int clicmdflag = 0;
static unsigned int linuxcmdflag = 0;
static unsigned int inputenable = 0;
static struct timer_list input_timer;

static DEFINE_SPINLOCK(RS232_lock);
#define UART_DMA_VER_MAIN	0
#define UART_DMA_VER_MINOR	1
#define	UART_DMA_VER_REV	1

//#define AC83XX_UART_DEBUG	1
#define UART_DMA_WORKQUEUE	1
//#define UART_DMA_ACCOUNTING	1
#define PRINT_STRING	"[KERNEL UART] "
#ifdef AC83XX_UART_DEBUG
#define uart_dbg(fmt, ...) 					\
	({							\
		printk(KERN_INFO PRINT_STRING fmt, ##__VA_ARGS__); \
	})

#else
#define uart_dbg(fmt, ...)
#endif  // AC83XX_UART_DEBUG

#define uart_info(fmt, ...) 					\
	({							\
		printk(KERN_INFO PRINT_STRING fmt, ##__VA_ARGS__); \
	})

#define uart_err(fmt, ...) 					\
	({							\
		printk(KERN_ERR PRINT_STRING " function at %s, line %d", __func__, __LINE__); \
		printk(KERN_ERR PRINT_STRING fmt, ##__VA_ARGS__); \
	})						

/* support uart0(debug port) and uart 1~6  */
#define UART_NR                 (7)

#define SERIAL_AC83XX_MAJOR     204
#define SERIAL_AC83XX_MINOR     16

#define ATC_SERIAL_RINGSIZE (32768)

struct atc_uart_char {
	u16		status;
	u16		ch;
};

#define UART_PORT0		0 
#define UART_PORT1		1
#define UART_PORT2		2 
#define UART_PORT3		3
#define UART_PORT4		4 
#define UART_PORT5		5
#define UART_PORT6		6

#define BACKCAR_PORT		4	

int UartxBaseAddr[UART_NR][2]=
{
	{UART_PORT0, UART0_BASE},
#if UART_NR>1
	{UART_PORT1, UART1_BASE},
#endif    
#if UART_NR>2    
	{UART_PORT2, UART2_BASE},
#endif
#if UART_NR>3 
	{UART_PORT3, UART3_BASE},
#endif  
#if UART_NR>4
	{UART_PORT4, UART4_BASE},
#endif
#if UART_NR>5
	{UART_PORT5, UART5_BASE},
#endif
#if UART_NR>6
	{UART_PORT6, UART6_BASE}
#endif
};

static int _irq_allocated[UART_NR];

uint32_t g_u4enable_log_droped = 0;
EXPORT_SYMBOL(g_u4enable_log_droped);
static bool fgUartRegisterd = false;

/*
 * Port
 */
struct ac83xx_uart_port
{
	struct uart_port		port;
	int				nport;
	unsigned int			ms_enable;

	unsigned int			(*fn_read_allow)(int);
	unsigned int			(*fn_write_allow)(int);
	void				(*fn_int_enable)(int,int enable);
	void				(*fn_empty_int_enable)(int,int enable);
	unsigned int			(*fn_read_byte)(int);
	void				(*fn_write_byte)(int,unsigned int byte);
	void				(*fn_flush)(int);
	void				(*fn_get_top_err)(int,int *p_parity, int *p_end, int *p_break);
	struct tasklet_struct		uart_tasklet_rx;
	struct tasklet_struct		uart_tasklet_tx;
	/* Force to stop rx/tx tasklset in case something is wrong */
	bool				stop_fifo_tasklet;

#ifdef	UART_DMA_WORKQUEUE
	struct work_struct		uart_work_tx;
	struct workqueue_struct		*uart_workqueue_tx;
	bool				cancel_work;
#endif
#ifdef	UART_DMA_ACCOUNTING
	uint64_t 			rx_count;
	uint64_t 			tx_count;
#endif
	struct circ_buf			rx_ring;
	spinlock_t			rx_ring_lock;
	uint32_t 			dma_tx_pa;
	uint32_t 			dma_tx_va;
	uint32_t 			dma_rx_pa;
	uint32_t 			dma_rx_va;
	bool				dma_mode;
	int 				dma_curr_read_offset;

};

/*
 * Locals
 */

static unsigned int ac83xx_get_uart_base_addr(int nport);
static void _ac83xx_uart_stop_tx(struct uart_port *port);

#define UART_READ32(REG)			__raw_readl(__io(RS232_BASE_VA + REG))
#define UART_WRITE32(VAL, REG)						\
	do {								\
		 __raw_writel(VAL, __io(RS232_BASE_VA + REG));		\
	} while(0)

#define CKGEN_VIRT_READ32(REG)			__raw_readl(__io(CKGEN_BASE_VA + REG))
#define CKGEN_VIRT_WRITE32(VAL, REG)		__raw_writel(VAL, __io(CKGEN_BASE_VA + REG))
#define CKGEN_READ32(REG)		__raw_readl(__io(CKGEN_BASE_VA + REG))
#define CKGEN_WRITE32(VAL, REG)		__raw_writel(VAL, __io(CKGEN_BASE_VA + REG))


/*
 * Macros
 */
#define UART_REG_BITCLR(BITS, REG)	UART_WRITE32(UART_READ32(REG) & ~(BITS), REG)
#define UART_REG_BITSET(BITS, REG)	UART_WRITE32(UART_READ32(REG) | (BITS), REG)

#define UART0_FLUSH()		UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), UART0_BUFCTRL)
#define UART1_FLUSH()		UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), UART1_BUFCTRL)


#define UART1_PINMUX_BIT		(11)
#define UART2_PINMUX_BIT		(13)
#define UART3_PINMUX_BIT		(16)
#define UART4_PINMUX_BIT		(18)
#define UART5_PINMUX_BIT		(20)
#define UART6_PINMUX_BIT		(23)


#define UART_PINMUX_OFFSET		(0x6C)
#define UART_PINMUX_WRITE(value)	CKGEN_VIRT_WRITE32((value), UART_PINMUX_OFFSET)
#define UART_PINMUX_REG()		CKGEN_VIRT_READ32(UART_PINMUX_OFFSET)

//extern functions.
extern void ac83xx_mask_ack_bim_irq(unsigned int irq);

/**
 * @Desc:
 *      UART DMA
 */
#define UART_DMA_HW_BUF_SIZE    0x4000

//#ifdef DUMP_REG
#if 1
#define	ac83xx_uart_dump_reg(port)	\
	do {								\
		int base_addr = ac83xx_get_uart_base_addr(port);	\
		printk(KERN_INFO PRINT_STRING "function at %s, line %d.\n", __func__, __LINE__);	\
		uart_info("port %d UART_CTRL_REG register = 0x%x.\n", port, UART_READ32(base_addr + UART_CTRL_REG));	\
		uart_info("port %d UART_STA_REG register = 0x%x.\n", port, UART_READ32(base_addr + UART_STA_REG));	\
		uart_info("port %d UART_BUF_CTRL register = 0x%x.\n", port, UART_READ32(base_addr + UART_BUF_CTRL));	\
		uart_info("port %d UART_INT_EN register = 0x%x.\n", port, UART_READ32(base_addr + UART_INT_EN));	\
		uart_info("port %d UART_INT_STA register = 0x%x.\n", port, UART_READ32(base_addr + UART_INT_STA));	\
		uart_info("port %d UART_DMA_WFIFO_CTRL register = 0x%x.\n", port, UART_READ32(base_addr + UART_DMA_WFIFO_CTRL));	\
		uart_info("port %d UART_DMA_WFIFO_ADDR register = 0x%x.\n", port, UART_READ32(base_addr + UART_DMA_WFIFO_ADDR));	\
		uart_info("port %d UART_DMA_RFIFO_CTRL register = 0x%x.\n", port, UART_READ32(base_addr + UART_DMA_RFIFO_CTRL));	\
		uart_info("port %d UART_DMA_RFIFO_ADDR register = 0x%x.\n", port, UART_READ32(base_addr + UART_DMA_RFIFO_ADDR));	\
		uart_info("port %d UART_TRAN_DATA_NUM register = 0x%x.\n", port, UART_READ32(base_addr + UART_TRAN_DATA_NUM));	\
	} while(0)
#else
#define	ac83xx_uart_dump_reg(port)
#endif

/**
 *  DMA mode by default
 *  0x01	-> PORT1 -> BT
 *  0x02	-> PORT2 -> GPS
 *  0x04	-> PORT3 -> DVP
 *  0x08	-> PORT4 -> BACKCAR
 *  0x10	-> PORT5 
 *  ...
 */
#define PORT1_DMA_ENABLED	(0x01)
#define PORT2_DMA_ENABLED	(0x02)
#define PORT3_DMA_ENABLED	(0x04)
#define PORT4_DMA_ENABLED	(0x08)
#define PORT5_DMA_ENABLED	(0x10)
//#define PORT6_DMA_ENABLED	(0x20)
//static int port_dma_enabled = PORT2_DMA_ENABLED;
static int port_dma_enabled = 0;
//static int port_dma_enabled = PORT1_DMA_ENABLED | PORT2_DMA_ENABLED;
				// | PORT3_DMA_ENABLED | PORT4_DMA_ENABLED 
				// | PORT5_DMA_ENABLED ;

static void ac83xx_uart_pin_select(int port, int pinsel)
{
	uint32_t  pinmux = UART_PINMUX_REG();
	uart_dbg("port %d, current pinsel %d, set to %d.\n", port, pinmux, pinsel);
	switch(port) {
		case UART_PORT1:  {
			pinmux &= (~(3 << UART1_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART1_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART1_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}
			break;
		}
		
		case UART_PORT2:  {
			pinmux &= (~(3 << UART2_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART2_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART2_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}

			break;
		}

		case UART_PORT3:  {
			pinmux &= (~(3 << UART3_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART3_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART3_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}

			break;
		}

		case UART_PORT4:  {
			pinmux &= (~(3 << UART4_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART4_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART4_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}

			break;
		}

		case UART_PORT5:  {
			pinmux &= (~(3 << UART5_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART5_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART5_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}

			break;
		}

		case UART_PORT6:  {
			pinmux &= (~(3 << UART6_PINMUX_BIT));
			if (pinsel == 0) {
				UART_PINMUX_WRITE(pinmux);
				uart_dbg("port 1 pinsel set to 0.\n");
			} else if (pinsel == 1) {
				UART_PINMUX_WRITE(pinmux | ( 1 << UART6_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 1.\n");
			} else if (pinsel == 2) {
				UART_PINMUX_WRITE(pinmux | ( 2 << UART6_PINMUX_BIT));
				uart_dbg("port 1 pinsel set to 2.\n");
			} else {
				uart_err("port %d pinsel %d error.\n", port, pinsel);
			}

			break;
		}
	}
	uart_dbg("pinsel is %d.\n", UART_PINMUX_REG());
	
}

#define DUMP_R_FILE	"/system/drivers/uart_dump_r_file"
#define DUMP_W_FILE	"/system/drivers/uart_dump_w_file"
static char *dump_r_file = DUMP_R_FILE;
static char *dump_w_file = DUMP_W_FILE;

static void ac83xx_uart_dump_file(char *dump_file, char *buf, size_t len)
{
#if	0
	int i = 0;
	while ( len-- > 0) {
		printk("0x%x ", buf[i]);
		i++;
	}
	printk("\n");

	struct file *fp = NULL;
	mm_segment_t fs;
    	int ret = 0;
	fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(dump_file, O_RDWR | O_CREAT | O_APPEND , S_IRUSR);
	if(IS_ERR(fp)) {
		uart_err("dump file open failure.\n");	
		return;
	}
	
	if (buf) {
		if ((ret = fp->f_op->write(fp, buf, len, &fp->f_pos)) < 0) {
			uart_info("Write file error\n");
		}
	}
     
	if (!IS_ERR(fp)) {
		filp_close(fp, NULL);
	}
	set_fs(fs);
#endif
}

static bool ac83xx_uart_dma_is_enable(int port)
{
	if (port < 1)
		return false;
	return port_dma_enabled & (0x01 << (port -1));
}

static void ac83xx_uart_dma_clear_interrupt(int port, unsigned int dest_status) 
{
	int base_addr = ac83xx_get_uart_base_addr(port);
	UART_WRITE32(dest_status, base_addr + UART_INT_STA);
}

static void ac83xx_uart_dma_clear_tx_status(struct ac83xx_uart_port *mport)
{
	uint32_t tx_dma_ctl = 0;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);

	tx_dma_ctl = UART_READ32(base_addr + UART_DMA_RFIFO_CTRL);

	if(0 != (tx_dma_ctl & U_INHIBIT_SEND)) {
		tx_dma_ctl &= (~(U_INHIBIT_SEND | U_DATA_READY /*| U_INHIBIT_REG_TRIG*/  ));
		UART_WRITE32(tx_dma_ctl, base_addr + UART_DMA_RFIFO_CTRL);// not ready, clear inhibit send
	}
	else {
		tx_dma_ctl &= (~U_DATA_READY);
		UART_WRITE32(tx_dma_ctl, base_addr + UART_DMA_RFIFO_CTRL);// not ready  
	}
}

static void ac83xx_uart_dma_clear_tx_buf(struct ac83xx_uart_port *mport, bool wait)
{

	uint8_t uart_port = mport->nport;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);
	
	if (wait) {
RETRY:
	if ( U_GET_TX_BUF_SLOT(uart_port, UART_READ32(base_addr + UART_STA_REG)) != 
			(U_TX_BUF_SLOT_MASK(uart_port) >>U_TX_BUF_SLOT_OFFSET(uart_port))) {
goto RETRY;
	}
}
	UART_WRITE32(UART_READ32(base_addr + UART_BUF_CTRL) | UART_CLEAR_TX_BUFFER, base_addr + UART_BUF_CTRL);

}

static void ac83xx_uart_dma_enable_interrupt(uint8_t port, uint32_t enable) 
{      
	int base_addr = ac83xx_get_uart_base_addr(port);
	UART_WRITE32(enable, base_addr + UART_INT_EN);
}

static void ac83xx_uart_dma_deinit(struct ac83xx_uart_port *mport)
{
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);
	uint32_t tmp_bufctl = 0;


	//UART_WRITE32(~UART_DMA_MODE_ENABLE, base_addr + UART_CTRL_REG);
	UART_WRITE32(0xFF, base_addr + UART_INT_STA);
	UART_WRITE32(0, base_addr + UART_INT_EN);
	UART_WRITE32(0, base_addr + UART_CTRL_REG);

	tmp_bufctl = UART_READ32(base_addr + UART_BUF_CTRL);
	UART_WRITE32((tmp_bufctl |UART_CLEAR_RX_BUFFER | UART_CLEAR_TX_BUFFER), base_addr + UART_BUF_CTRL);
	//UART_WRITE32(0xFF, base_addr + UART_INT_STA);
	//UART_WRITE32(0, base_addr + UART_INT_EN);

	if(mport->rx_ring.buf) {
		free_pages((unsigned long)mport->rx_ring.buf, get_order(ATC_SERIAL_RINGSIZE));
		mport->rx_ring.buf = NULL;

		if (mport->dma_tx_va) {
			dma_free_coherent(NULL, UART_DMA_HW_BUF_SIZE, (void *)mport->dma_tx_va, mport->dma_tx_pa);
			mport->dma_tx_va = 0;
			mport->dma_tx_pa = 0;
		
			
		}
		if (mport->dma_rx_va) {
			dma_free_coherent(NULL, UART_DMA_HW_BUF_SIZE, (void *)mport->dma_rx_va, mport->dma_rx_pa);
			mport->dma_rx_va = 0;
			mport->dma_rx_pa = 0;
		}

	}

#ifdef	UART_DMA_WORKQUEUE
	mport->cancel_work = true;
	cancel_work_sync(&mport->uart_work_tx);
	flush_workqueue(mport->uart_workqueue_tx);
	destroy_workqueue(mport->uart_workqueue_tx);
#else
	tasklet_disable(&mport->uart_tasklet_tx);
	tasklet_kill(&mport->uart_tasklet_tx);
#endif
	tasklet_disable(&mport->uart_tasklet_rx);
	tasklet_kill(&mport->uart_tasklet_rx);
}

static uint32_t ac83xx_uart_dma_init(struct ac83xx_uart_port *mport)
{  
	uint32_t tmp_bufctl = 0;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);

	if ((0 == mport->dma_rx_pa) && (0 == mport->dma_rx_va)) {
	
		mport->dma_rx_va = (uint32_t)dma_alloc_coherent(NULL,
					UART_DMA_HW_BUF_SIZE,
					&mport->dma_rx_pa,
					GFP_KERNEL);

		if (0 == mport->dma_rx_va) {
			uart_err("dma_alloc_coherent  failed.\r\n");
			return ENOMEM;
		}

		memset((char *)mport->dma_rx_va, 0, UART_DMA_HW_BUF_SIZE);//lll
		UART_WRITE32((((uint32_t)mport->dma_rx_pa)>>4), base_addr + UART_DMA_WFIFO_ADDR);
	}
	
	if ( (0 == mport->dma_tx_pa) && (0 == mport->dma_tx_va) ) {
		mport->dma_tx_va = (uint32_t)dma_alloc_coherent(NULL,
				UART_DMA_HW_BUF_SIZE,
				&mport->dma_tx_pa,
				GFP_KERNEL);

		if (0 == mport->dma_tx_va) {
			uart_err("dma_alloc_coherent  failed.\r\n");
			if (mport->dma_rx_va) {
				dma_free_coherent(NULL, UART_DMA_HW_BUF_SIZE, (void *)mport->dma_rx_va, mport->dma_rx_pa);
				mport->dma_rx_va = 0;
				mport->dma_rx_pa = 0;
			}
			return ENOMEM;
		}
		memset((char *)mport->dma_tx_va, 0, UART_DMA_HW_BUF_SIZE);

		ac83xx_uart_dma_clear_tx_buf(mport, true);
		UART_WRITE32((((uint32_t)mport->dma_tx_pa)>>4), base_addr + UART_DMA_RFIFO_ADDR);
	}
	
	tmp_bufctl = UART_READ32(base_addr + UART_BUF_CTRL);
	ac83xx_uart_dma_enable_interrupt(mport->nport,0x00);

	//clear the FIFO 
	UART_WRITE32((tmp_bufctl |UART_CLEAR_RX_BUFFER | UART_CLEAR_TX_BUFFER), base_addr + UART_BUF_CTRL);
	// RX
	UART_WRITE32(0x0, base_addr + UART_DMA_WFIFO_CTRL); 
	//UART_WRITE32(0x41f1000, base_addr + UART_DMA_WFIFO_CTRL);
	UART_WRITE32(0x021F1000, base_addr + UART_DMA_WFIFO_CTRL);

	ac83xx_uart_dma_enable_interrupt(mport->nport,0x75);//enable Interr
	UART_WRITE32(UART_DMA_MODE_ENABLE, base_addr + UART_CTRL_REG);
	return 0;
}

/*
 * Note:  must be called during interrupt context
 */
static uint32_t ac83xx_uart_dma_put_into_circ(struct circ_buf *ring, void *buf, uint32_t count)
{
	int c, ret = 0;

	while (1) {
		c = CIRC_SPACE_TO_END(ring->head, ring->tail, ATC_SERIAL_RINGSIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(ring->buf + ring->head, buf, c);
		ring->head = (ring->head + c) & (ATC_SERIAL_RINGSIZE - 1);
		buf += c;
		count -= c;
		ret += c;
	}
	/* no enough space for save data */
	if (count > 0) { 
		uart_info("Ring Buffer is not enough for saving read data\n");
	}
	return ret;
}
 
static uint32_t ac83xx_uart_dma_read_data(struct ac83xx_uart_port *mport)
{

	struct circ_buf *ring = &mport->rx_ring;
	uint32_t dma_rx_fifo_size;
	uint32_t dma_rx_num =0, tmp_rx_num =0;
	uint8_t *dma_curr_read_addr = NULL;
    	uint8_t uart_port = mport->nport;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);
	int ret;

	spin_lock(&mport->rx_ring_lock);
	dma_rx_fifo_size = ((UART_READ32(base_addr + UART_DMA_WFIFO_CTRL)>>16) & (0x1F));
	dma_rx_fifo_size = (dma_rx_fifo_size + 1) << 9;
	dma_rx_num = ((UART_READ32(base_addr + UART_TRAN_DATA_NUM)) & 0x7FFF);

	uart_dbg("uart_port = %d, dma_rx_fifo_size=%d, dma_rx_num=%d.\n", uart_port, dma_rx_fifo_size, dma_rx_num);

	if (dma_rx_num == 0) {
		uart_dbg("READ DATA size is 0.\n");
		return 0;
	}

#ifdef UART_DMA_ACCOUNTING
	uart_info("uart_port %d has read %d bytes data already.\n", uart_port, mport->rx_count);
	uart_info("current offset %d.\n", mport->dma_curr_read_offset);
	ac83xx_uart_dump_reg(mport->nport);
#endif

	dma_curr_read_addr = (uint8_t*)(mport->dma_rx_va + mport->dma_curr_read_offset);

	if( (mport->dma_curr_read_offset + dma_rx_num) <= dma_rx_fifo_size ) {

		if (uart_port == UART_PORT1)
			ac83xx_uart_dump_file(dump_r_file, (char *)dma_curr_read_addr, dma_rx_num); 

		ret = ac83xx_uart_dma_put_into_circ(ring, dma_curr_read_addr, dma_rx_num);
		dma_curr_read_addr += ret;

		mport->dma_curr_read_offset += dma_rx_num;

		if (mport->dma_curr_read_offset >= dma_rx_fifo_size) {
			if(mport->dma_curr_read_offset == dma_rx_fifo_size) {
				mport->dma_curr_read_offset = 0;
				uart_dbg("dma_curr_read_offset = dma_rx_fifo_size is %d.\n", mport->dma_curr_read_offset);
			} else {
				mport->dma_curr_read_offset -= dma_rx_fifo_size;
				uart_err("should never get here \n");
        		}
		}

	} else { // > dma_rx_fifo_size

		tmp_rx_num = dma_rx_fifo_size - mport->dma_curr_read_offset;

		if (uart_port == UART_PORT1)
			ac83xx_uart_dump_file(dump_r_file, (char *)dma_curr_read_addr, tmp_rx_num); 

		ret = ac83xx_uart_dma_put_into_circ(ring, dma_curr_read_addr, tmp_rx_num);

		tmp_rx_num = (mport->dma_curr_read_offset+ dma_rx_num) - dma_rx_fifo_size; 
		mport->dma_curr_read_offset = 0;

		dma_curr_read_addr = (uint8_t*)(mport->dma_rx_va + mport->dma_curr_read_offset);

		if (uart_port == UART_PORT1)
			ac83xx_uart_dump_file(dump_r_file, (char *)dma_curr_read_addr, tmp_rx_num); 

		ret = ac83xx_uart_dma_put_into_circ(ring, dma_curr_read_addr, tmp_rx_num);
		mport->dma_curr_read_offset = tmp_rx_num;

	}
	spin_unlock(&mport->rx_ring_lock);

#ifdef UART_DMA_ACCOUNTING
	mport->rx_count += dma_rx_num;
	ac83xx_uart_dump_reg(mport->nport);
#endif
	return dma_rx_num;
}

static void ac83xx_uart_dma_tasklet_rx(unsigned long arg)
{
	struct uart_port *port = (struct uart_port *)arg; 
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)arg;

	struct tty_struct *tty = mport->port.state->port.tty;
#ifdef AC83XX_UART_DEBUG
	uint8_t uart_port = mport->nport;
#endif
	struct circ_buf *ring = &mport->rx_ring;
	unsigned long flags;
	int c, count, copied, copied_all = 0;
	uart_dbg("uart_port = %d, ac83xx_uart_dma_tasklet_rx.\n", uart_port);

#ifdef UART_DMA_ACCOUNTING
	ac83xx_uart_dump_reg(mport->nport);
#endif

	count = CIRC_CNT(ring->head, ring->tail, ATC_SERIAL_RINGSIZE);

	while(1) {
		c = CIRC_CNT_TO_END(ring->head, ring->tail, ATC_SERIAL_RINGSIZE);

		if (count < c) {
			c = count;
		}

		if (c <=0) {
			break;
		}

		spin_lock_irqsave(&port->lock, flags);
		copied = tty_insert_flip_string(tty, (ring->buf + ring->tail), c);
		ring->tail = (ring->tail + copied) & (ATC_SERIAL_RINGSIZE - 1);
		spin_unlock_irqrestore(&port->lock, flags);

		port->icount.rx += copied;
		copied_all += copied;
		count -= copied;

		tty_flip_buffer_push(mport->port.state->port.tty);

		if(copied != c) {
			uart_info("APP Do not read data from tty layer in time.\n");
			break;
		}
	}

	uart_dbg("uart port = %d. insert %d bytes, %d left \n", uart_port, copied_all, count);

#ifdef UART_DMA_ACCOUNTING
	ac83xx_uart_dump_reg(mport->nport);
#endif
}

static void ac83xx_uart_dma_tx_ctrl
(struct ac83xx_uart_port *mport, uint8_t burst_len, uint8_t fifo_size, uint8_t trig_level, uint8_t fill_num, uint8_t data_ready)
{
	uint32_t tmp_reg = 0x0;
	uint32_t mask  = 0x0;
	uint32_t tx_ctrl     = 0x0;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);
	
	tmp_reg |= U_SET_RFIFO_BURST_LEN(burst_len);
	tmp_reg |= U_SET_RFIFO_SIZE(fifo_size);
	tmp_reg |= U_SET_RFIFO_TRIG_LEVEL(trig_level);
	tmp_reg |= (fill_num != 0) ? (U_SET_RFIFO_EMPTY_BYTE_NUM(fill_num) | 
			U_INHIBIT_SEND | U_INHIBIT_REG_TRIG) : U_SET_RFIFO_EMPTY_BYTE_NUM(fill_num);
	tmp_reg |= data_ready & U_DATA_READY;
	
	mask |= (burst_len != 0) ? U_SET_RFIFO_BURST_LEN(0x1F) : 0;
	mask |= (fifo_size != 0) ? U_SET_RFIFO_SIZE(0x1F) : 0;
	mask |= (trig_level != 0) ? U_SET_RFIFO_TRIG_LEVEL(0x1F) : 0;
	mask |= (fill_num != 0) ? (U_SET_RFIFO_EMPTY_BYTE_NUM(0x0F) | 
				U_INHIBIT_SEND | U_INHIBIT_REG_TRIG) : U_SET_RFIFO_EMPTY_BYTE_NUM(0x0F);
	mask |= U_DATA_READY;

	tx_ctrl |= (tmp_reg & mask);
	UART_WRITE32(tx_ctrl, base_addr + UART_DMA_RFIFO_CTRL);
}

#ifdef UART_DMA_WORKQUEUE
static void ac83xx_uart_dma_workqueue_tx(struct work_struct *work)
{
	struct ac83xx_uart_port *mport = container_of(work,struct ac83xx_uart_port,uart_work_tx);
	struct uart_port *port = &mport->port;
#else
static void ac83xx_uart_dma_tasklet_tx(unsigned long arg)
{
	struct uart_port *port = (struct uart_port *)arg;
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)arg;
#endif
	struct circ_buf *xmit 		= &port->state->xmit;
	uint32_t ret  			= 0;
	uint32_t rest_len		= 0;
	uint8_t  burst_len		= 0;
	uint8_t  trig_level		= 0;
	uint8_t  fifo_size		= 0; 
	uint8_t  fill_num		= 0;

	uint8_t uart_port = mport->nport;
	unsigned long flags;
	int base_addr = ac83xx_get_uart_base_addr(mport->nport);
	uint32_t pa_addr =  mport->dma_tx_pa;
	uint32_t va_addr =  mport->dma_tx_va;
	uint32_t write_len = CIRC_CNT(xmit->head, xmit->tail, UART_XMIT_SIZE);
	int count = write_len;
	int c = 0;
	int offset = 0;
	
#ifdef UART_DMA_ACCOUNTING
#ifdef CONFIG_SMP
	uart_info("uart_port %d has write %d bytes data already on cpu%d.\n", uart_port, mport->tx_count, get_current()->on_cpu);
#else
	uart_info("uart_port %d has write %d bytes data already.\n", uart_port, mport->tx_count);
#endif
	mport->tx_count += write_len;
#endif
	uart_dbg("uart port = %d, write_len %d %s loop.\n", uart_port, write_len, 
		(write_len > CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE)) ? "YES" : "NO");

	while(1) {
		c = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

		if (count < c) {
			c = count;
		}

		if (c <=0) {
			break;
		}

		memcpy(((void *)va_addr + offset), &(xmit->buf[xmit->tail]), c);
		if (uart_port == UART_PORT1)
			ac83xx_uart_dump_file(dump_w_file, (char *)(&(xmit->buf[xmit->tail])), c);

		spin_lock_irqsave(&port->lock, flags);
		xmit->tail = (xmit->tail + c) & (UART_XMIT_SIZE - 1);
		port->icount.tx += c;
		spin_unlock_irqrestore(&port->lock, flags);

		offset += c;
		count -= c;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) { 
		uart_write_wakeup(port);
	}


#ifdef CONFIG_SMP
	uart_dbg("uart port = %d, write data len =%d on cpu core %d.\n", uart_port, write_len, get_current()->on_cpu);
#else
	uart_dbg("uart port = %d, write data len =%d.\n", uart_port, write_len);
#endif
	fifo_size = UART_DMA_MAX_MEM_SIZE;

	for (rest_len = write_len; rest_len > 0;) {
		if(mport->cancel_work)
			break;

		if (rest_len >= UART_DMA_MAX_BYTE_NUM) {   

			burst_len = UART_DMA_MAX_BURST_LEN;
			trig_level = UART_DMA_MAX_TRIG_LEVEL;
			fill_num = 0;
			ret += UART_DMA_MAX_BYTE_NUM;
			rest_len -= UART_DMA_MAX_BYTE_NUM; 
			
			uart_dbg("uart port = %d, rest_len=%d\n", uart_port, rest_len);
			UART_WRITE32(pa_addr>>4, base_addr + UART_DMA_RFIFO_ADDR);
			ac83xx_uart_dma_clear_tx_buf(mport, true);
			pa_addr += UART_DMA_MAX_BYTE_NUM;

		} else if (rest_len >= (UART_DMA_MAX_TRIG_LEVEL * 16)) {

			burst_len = UART_DMA_MAX_BURST_LEN;
			trig_level = rest_len /(burst_len * 16);           
			fill_num = 0;
			ret += (burst_len * trig_level * 16);
			rest_len -= (burst_len * trig_level * 16);
			
			uart_dbg("uart port = %d, rest_len=%d\n", uart_port, rest_len);
			UART_WRITE32(pa_addr>>4, base_addr + UART_DMA_RFIFO_ADDR);
			ac83xx_uart_dma_clear_tx_buf(mport, true);
			pa_addr += (burst_len * trig_level * 16);
		} else {
			ret += rest_len;
			fill_num = ((0x10 - (rest_len & 0x0F)) & 0x0F); 
			rest_len += fill_num;
			burst_len = (rest_len >> 4);
			trig_level = 1;
			rest_len -= (burst_len * trig_level * 16);
			uart_dbg("uart port = %d, rest_len=%d\n", uart_port, rest_len);
			UART_WRITE32(pa_addr>>4, base_addr + UART_DMA_RFIFO_ADDR);

			ac83xx_uart_dma_clear_tx_buf(mport, true);
			ac83xx_uart_dma_tx_ctrl(mport, burst_len, fifo_size, trig_level, fill_num, 0);//dram to sram

		}

		ac83xx_uart_dma_tx_ctrl(mport, burst_len, fifo_size, trig_level, fill_num, 1);//we ready

		uart_dbg("uart port = %d, starting to write.....\n", uart_port);
		while (((U_GET_TX_BUF_SLOT(uart_port, UART_READ32(base_addr + UART_STA_REG)) 
			!= (U_TX_BUF_SLOT_MASK(uart_port)>>U_TX_BUF_SLOT_OFFSET(uart_port))) 
			|| ((UART_READ32(base_addr + UART_DMA_RFIFO_CTRL) & U_INHIBIT_SEND) != 0)) 
			||((UART_READ32(base_addr + UART_DMA_RFIFO_CTRL) & U_DATA_READY) != 0)) {

			if(mport->cancel_work)
				break;

			msleep_interruptible(1);
			//msleep(1);

		}
		uart_dbg("uart port = %d, write %d success.\n", uart_port, write_len);
	}
	ac83xx_uart_dma_clear_tx_buf(mport, true);
}

static void _ac83xx_uart_hwinit(void)
{
	int nport;
	int base_addr = 0;

	// UART0 is set to transparent by boot loader.
	// _ac83xx_u0_set_trans_mode_on();

	CKGEN_WRITE32(CKGEN_READ32(0xB0)|0x3F0, 0xB0);
	CKGEN_WRITE32(CKGEN_READ32(0xc)|0x8000000, 0xc);
	CKGEN_WRITE32(CKGEN_READ32(0x8)|0x555000, 0x8);
	UART_WRITE32(0x2207E2, UART0_COMMCTRL);

	uart_info("uart0 baud = %d,clk = %x\r\n",UART_READ32(UART0_COMMCTRL),CKGEN_READ32(0xc));  
	/* disable interrupt */
	UART_WRITE32(0, UART0_INT_EN);
	ac83xx_uart_pin_select(UART_PORT6, 2);
	
	// port1 ~port 6
	for(nport = 1 ; nport < UART_NR ; nport ++ ) {
		if(nport == BACKCAR_PORT)
			continue;
		base_addr = ac83xx_get_uart_base_addr(nport) ;
		uart_info("hardware init ,nport = %d ,uart base = %x\r\n",nport,base_addr);  
		/* disable interrupt */
		UART_WRITE32(0, base_addr + UART_INT_EN);
		/* set baud rate  to default value */
		UART_WRITE32(0, base_addr + UART_COMMCTRL);
	}
}



/*********************************************************************
// Function : void _ac83xx_uart0_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
// Description : set uart0 property such as baud rate ,date length ,stop bit ,parity
// Parameter : uCOMMCTRL :common port control register
//           
// Return    : None
 **********************************************************************/
static void _ac83xx_uart0_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
{

	int cur_reg_val ;
	cur_reg_val = UART_READ32(uCOMMCTRL);
	uart_info("uart0 Set curreg= %x ,uCOMMCTRL=%d,baud=%d\r\n",cur_reg_val,uCOMMCTRL,baud);
	switch (baud)
	{
		case 115200:
			UART_REG_BITSET(SETBAUD(BAUD_115200)|0xe2, uCOMMCTRL);
			break;
		case 230400:
			UART_REG_BITSET(SETBAUD(BAUD_230400)|0xe2, uCOMMCTRL);
			break;
		case 460800:
			UART_REG_BITSET(SETBAUD(BAUD_460800)|0xe2, uCOMMCTRL);
			break;
		case 921600:
			UART_REG_BITSET(SETBAUD(BAUD_921600)|0xe2, uCOMMCTRL);
			break;
		case 57600:
			UART_REG_BITSET(SETBAUD(BAUD_57600)|0xe2, uCOMMCTRL);
			break;
		default:
			break;
	}
}

static uint32_t  uart_soucce_clk = 32400000;
static void _ac83xx_uartx_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
{

	int temp;
	uint32_t u4Rate_Div = 0;
	uart_dbg("uartx Set uCOMMCTRL =0x%x,baud=%d,\r\n",uCOMMCTRL,baud);
	u4Rate_Div = (uart_soucce_clk*10)/baud;
	u4Rate_Div +=5;
	u4Rate_Div /=10;
	u4Rate_Div -=1;

	UART_REG_BITCLR(U_BAUD_RATE_MASK,uCOMMCTRL);
	UART_REG_BITSET(SETBAUD(0XD),uCOMMCTRL);
	UART_REG_BITCLR(U_RATE_DIVISOR_MASK,uCOMMCTRL);

	UART_REG_BITSET(U_SET_RATE_DIVISOR(u4Rate_Div),uCOMMCTRL);
	UART_REG_BITCLR(0x000000FF,uCOMMCTRL);

	temp = UART_READ32(uCOMMCTRL);

	uart_dbg("uartx Set regCOMMCTRL =0x%x\r\n",temp);
}

static void _ac83xx_uart_get(unsigned int uCOMMCTRL, int *p_baud, int *p_datalen, int *p_stop, int *p_parity)
{
	int baud, datalen, stop, parity;

	switch (GETBAUD(UART_READ32(uCOMMCTRL)))
	{
		case BAUD_X1:
			baud = 115200;
			break;
		case BAUD_X2:
			baud = 230400;
			break;
		case BAUD_X4:
			baud = 460800;
			break;
		case BAUD_X8:
			baud = 921600;
			break;
		case BAUD_57600:
			baud = 57600;
			break;
		case BAUD_38400:
			baud = 38400;
			break;
		case BAUD_19200:
			baud = 19200;
			break;
		case BAUD_9600:
			baud = 9600;
			break;
		case BAUD_4800:
			baud = 4800;
			break;
		case BAUD_2400:
			baud = 2400;
			break;
		default:
			baud = 115200;
			break;
	}

	// We currently don't set these values
	datalen = 0;
	stop = 0;
	parity = 0;
	*p_baud = baud ;
}

unsigned int _ac83xx_u0_trans_mode_on(void)
{
	return UART_READ32(UART0_STATUS) & U0_TRANSPARENT;
}

/*
 * uart member functions
 */

static unsigned int _ac83xx_u0_read_allow(int nport )
{
	return UART_READ32(UART0_STATUS) & U0_RD_ALLOW;
}

static unsigned int _ac83xx_u0_write_allow(int nport)
{
	return UART_READ32(UART0_STATUS) & U0_WR_ALLOW;
}

static void _ac83xx_u0_int_enable(int nport,int enable)
{
	if (enable) {
		UART_REG_BITSET(U0_INTALL, UART0_INT_EN);
	}
	else {
		UART_REG_BITCLR(U0_INTALL, UART0_INT_EN);
	}
}

static void _ac83xx_u0_empty_int_enable(int nport,int enable)
{
	if (enable) {
		UART_REG_BITSET(U0_TBUF, UART0_INT_EN);
	}
	else {
		UART_REG_BITCLR(U0_TBUF, UART0_INT_EN);
	}
}

static unsigned int _ac83xx_u0_read_byte(int nport)
{
	return UART_READ32(UART0_DATA_BYTE);
}

static void _ac83xx_u0_write_byte(int nport,unsigned int byte)
{
	UART_WRITE32(byte, UART0_DATA_BYTE);
}

#ifdef CONFIG_CONSOLE_POLL

static unsigned int _ac83xx_u0_read_char(int nport,struct uart_port *port)
{   
	while(!_ac83xx_u0_read_allow(UART_PORT0));
	return _ac83xx_u0_read_byte(UART_PORT0);
}

static void _ac83xx_u0_write_char(int nport,struct uart_port *port, unsigned int byte)
{
	while(!_ac83xx_u0_write_allow(UART_PORT0));
	_ac83xx_u0_write_byte(UART_PORT0,byte);
}
#endif

static void _ac83xx_u0_flush(int nport)
{
	//u0 doesn't have buffer to be flushed
	return;
}

static void _ac83xx_u0_get_top_err(int nport,int *p_parity, int *p_end, int *p_break)
{
	*p_parity = (UART_READ32(UART0_STATUS) & U0_PARITY_ERR) ? 1 : 0;
	*p_end    = (UART_READ32(UART0_STATUS) & U0_END_ERR)    ? 1 : 0;
	*p_break  = 0;//u0 doesn't have break to get
}

/*********************************************************
 *  from uart 1 ~ uart 6
 *
 *
 ***********************************************************/

static unsigned int ac83xx_get_uart_base_addr(int nport)
{
	if (nport < UART_NR) {
		return UartxBaseAddr[nport][1];
	}
	else {
		uart_err("get_uart_base_addr invalide, nport =0x%x\n",nport);
		return -EINVAL;
	}

}


static unsigned int _ac83xx_uartx_read_allow(int nport)
{
	int ret_val;
	int data_size ;
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport);
	if (nport == 1) {
		data_size = UART_READ32(base_addr  + UART_STATUS) & UART1_RX_BUF_MASK;

	} else {
		data_size = UART_READ32(base_addr  + UART_STATUS) & RX_BUF_MASK;
        }

	if(data_size > 0 )
		ret_val = 1;
	else
		ret_val = 0; 

	return ret_val;
}

static unsigned int _ac83xx_uartx_write_allow(int nport)
{
	int allow = 0;
	int data_size ;
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport);
	data_size = UART_READ32(base_addr + UART_STATUS);

	if (nport == 1){
		data_size = (data_size & UART1_TX_BUF_MASK);
		if(data_size == 0x01FFC000)
			allow = 1;
	}else{
		data_size = (data_size & TX_BUF_MASK);
		if(data_size == 0x001FF000)
			allow = 1;
	}

	return allow;
}

void _ac83xx_uartx_int_enable(int nport,int enable)
{
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport) ;

	if (enable) {
		UART_REG_BITSET(0x17, base_addr +UART_INT_EN);
	}
	else {
		UART_REG_BITCLR(UART_INTALL, base_addr +UART_INT_EN);
	}
}

void _ac83xx_uartx_empty_int_enable(int nport,int enable)
{
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport) ;

	if (enable) {
		UART_REG_BITSET(TBUF_E1, base_addr + UART_INT_EN);
	}
	else {
		UART_REG_BITCLR(TBUF_E1, base_addr + UART_INT_EN);
	}
}

unsigned int _ac83xx_uartx_read_byte(int nport)
{
	unsigned int r_uart_data;
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport);

	r_uart_data = UART_READ32(base_addr + UART_DATA_BYTE);
	return r_uart_data;
}

void _ac83xx_uartx_write_byte(int nport,unsigned int byte)
{
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(nport);
	UART_WRITE32(byte, base_addr + UART_DATA_BYTE );
}

void _ac83xx_uartx_flush(int nport)
{
	/*UART1_FLUSH();*/
	int base_addr;
	base_addr = ac83xx_get_uart_base_addr(nport);

	UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), base_addr + UART_BUF_CTRL);
	return;
}

void _ac83xx_uartx_get_top_err(int nport,int *p_parity, int *p_end, int *p_break)
{
	int reg_addr;

	reg_addr = ac83xx_get_uart_base_addr(nport) + UART_STATUS ;

	if(nport == 1) {
		*p_parity = (UART_READ32(reg_addr) & UART1_PARITY_ERR) ? 1 : 0;
		*p_end    = (UART_READ32(reg_addr) & UART1_END_ERR)    ? 1 : 0;
		*p_break  = (UART_READ32(reg_addr) & UART1_BREAK_ERR)    ? 1 : 0;
	}
	else {
		*p_parity = (UART_READ32(reg_addr) & UART_PARITY_ERR) ? 1 : 0;
		*p_end    = (UART_READ32(reg_addr) & UART_END_ERR)    ? 1 : 0;
		*p_break  = (UART_READ32(reg_addr) & UART_BREAK_ERR)    ? 1 : 0;
	}
}
/*end uart x */

static void clicmd_prefix(struct ac83xx_uart_port *mport)
{
	int i;
	unsigned int data_byte;
	char const strecho[5] = "echo ";
	struct tty_struct *tty = mport->port.state->port.tty;
	unsigned int flag = TTY_NORMAL;
	for(i = 0; i < 5; i++)
	{
		mport->port.icount.rx++;
		data_byte = (unsigned int)strecho[i];
		uart_insert_char(&mport->port, 0, 0, data_byte, flag);
		tty_flip_buffer_push(tty);
	}
	clicmdflag = 1;
}

static void clicmd_suffix(struct ac83xx_uart_port *mport)
{
	int i;
	unsigned int data_byte;
	char const strecho[20] = " > /sys/cli/commands";
	struct tty_struct *tty = mport->port.state->port.tty;
	unsigned int flag = TTY_NORMAL;
	for(i = 0; i < 20; i++)
	{
		mport->port.icount.rx++;
		data_byte = strecho[i];
		uart_insert_char(&mport->port, 0, 0, data_byte, flag);
		tty_flip_buffer_push(tty);
	}
	clicmdflag = 0;
}
static void _ac83xx_uart0_rx_chars(struct ac83xx_uart_port *mport)
{
	struct tty_struct *tty = mport->port.state->port.tty;
	unsigned int data_byte;
	unsigned int flag;
	int err_parity, err_end, err_break;

	if (inputenable == 0)
	{
		return;
	}
	while (mport->fn_read_allow(mport->nport))
	{
		{
			/* in ac83xx, process error before read byte */
			mport->fn_get_top_err(mport->nport,&err_parity, &err_end, &err_break);

			/* read the byte */
			data_byte = mport->fn_read_byte(mport->nport);
			if ((linuxcmdflag == 0) && (clicmdflag == 0))//the first #, set linuxcmdflag=1
			{
				if (data_byte == 35)
				{
					linuxcmdflag = 1;//'#'should be not pass tty
					return;
				}
			}

			
			if (linuxcmdflag == 1)
			{
				if (data_byte == 13)
				{
					linuxcmdflag = 0;
				}
			}
			else
			{
				if (clicmdflag == 0)
				{
					clicmd_prefix(mport);
				}
				
				if (data_byte == 13)
				{
					clicmd_suffix(mport);
				}
			}
			if(mport->nport == 0)
			{
				/*------------------------------------
				  Because linux will disable ctrl+c, so I change its
				  ASCII from 3 to 9 for switch CLI state in CLI
				  module.

				  modify by Xiaohui.fang, 8/18/2009
				  --------------------------------------*/
				if (3 == data_byte)
				{
					data_byte = 9;
				}
				else if (9 == data_byte)
				{
					data_byte = 4;
				}

			}

			mport->port.icount.rx++;
			flag = TTY_NORMAL;

			/* error handling routine */
			if (err_break)
			{
				mport->port.icount.brk++;
				if (uart_handle_break(&mport->port))
				{
					continue;
				}
				flag = TTY_BREAK;
			}
			else if (err_parity)
			{
				mport->port.icount.parity++;
				flag = TTY_PARITY;
			}
			else if (err_end)
			{
				mport->port.icount.frame++;
				flag = TTY_FRAME;
			}
		}

		if (uart_handle_sysrq_char(&mport->port, data_byte))
		{
			continue;
		}

		//2.6.18. no overrun support, set status and mask to 0
		uart_insert_char(&mport->port, 0, 0, data_byte, flag);

	}
	tty_flip_buffer_push(tty);
}


/*
 * Stores the incoming character in the ring buffer
 */
static void
atc_buffer_rx_char(struct ac83xx_uart_port *mport, unsigned int ch, unsigned int flag)
{
	struct circ_buf *ring = &mport->rx_ring;
	struct atc_uart_char *c;

	if (CIRC_SPACE(ring->head, ring->tail, ATC_SERIAL_RINGSIZE)) {

		c = &((struct atc_uart_char *)ring->buf)[ring->head];
		c->status = flag;
		c->ch = ch;

		/* Make sure the character is stored before we update head. */
		smp_wmb();
		ring->head = (ring->head + 1) & (ATC_SERIAL_RINGSIZE - 1);

         } else {
		/* Buffer overflow, ignore char */
		uart_err("Buffer overflow, ignore char\n");
		return;
	}

}


/*
 * interrupt handling
 */
static void _ac83xx_uart_rx_chars(struct ac83xx_uart_port *mport)
{
	unsigned int data_byte;
	unsigned int flag;
	int base_addr;
	base_addr = ac83xx_get_uart_base_addr(mport->nport);

	if (mport->nport == 1)
	{
        while( (UART_READ32(base_addr  + UART_STATUS) & UART1_RX_BUF_MASK) )
		{
			/* read the byte */
			data_byte = mport->fn_read_byte(mport->nport);

			mport->port.icount.rx++;
			flag = TTY_NORMAL;
			atc_buffer_rx_char(mport, data_byte, flag);
		}
	}
	else
	{
        while( (UART_READ32(base_addr  + UART_STATUS) & RX_BUF_MASK))
		{
			/* read the byte */
			data_byte = mport->fn_read_byte(mport->nport);

			mport->port.icount.rx++;
			flag = TTY_NORMAL;			
			atc_buffer_rx_char(mport, data_byte, flag);
		}
	}


}
static void _ac83xx_uart0_tx_chars(struct ac83xx_uart_port *mport)
{
	struct uart_port *port = &mport->port;
	struct circ_buf *xmit = &port->state->xmit;
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(mport->nport);


	if (port->x_char) {
		if (UART_READ32(UART0_STATUS) & U0_TRANSPARENT) {
			mport->fn_write_byte(mport->nport,port->x_char);
		}
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	/* stop tx if circular buffer is empty or this port is stopped */
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		_ac83xx_uart_stop_tx(port);
		return;
	}
	do {
		spin_lock(&RS232_lock);	
		if (UART_READ32(UART0_STATUS) & U0_TRANSPARENT) {
			mport->fn_write_byte(mport->nport,xmit->buf[xmit->tail]);
		}
        
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit)) 
		{
		   spin_unlock(&RS232_lock);
			break;
		}
			
		if (!mport->fn_write_allow(mport->nport)) {
			spin_unlock(&RS232_lock);
			break;
		}
		spin_unlock(&RS232_lock);
	} while (mport->fn_write_allow(mport->nport));
	

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		uart_write_wakeup(port);
	}

	if (uart_circ_empty(xmit)) {
		_ac83xx_uart_stop_tx(port);
	}
}

static void _ac83xx_uart_tx_chars(struct ac83xx_uart_port *mport)
{
	struct uart_port *port = &mport->port;
	struct circ_buf *xmit = &port->state->xmit;
	int base_addr;

	base_addr = ac83xx_get_uart_base_addr(mport->nport);

	//deal with x_char first 
	if (port->x_char) 
	{
		mport->fn_write_byte(mport->nport,port->x_char);

		port->icount.tx++;
		port->x_char = 0;
		uart_info("[_ac83xx_uart_tx_chars ]prot is %d and deal with x_char first \r\n", mport->nport);
		return;
	}
    if (uart_circ_empty(xmit) ) 
	{
	    uart_info("[_ac83xx_uart_tx_chars ]prot is %d and   uart_circ_empty \r\n", mport->nport);
		return;
	}
	//stop tx if circular buffer is empty or this port is stopped 
	//if (uart_circ_empty(xmit) || uart_tx_stopped(port)) 
	if ( uart_tx_stopped(port)) 
	{
	    uart_info("[_ac83xx_uart_tx_chars ]prot is %d and uart_tx_stopped is \r\n", mport->nport);
		return;
	}
	do 
	{
	    if (mport->nport == UART_PORT1)
	    {
          while(!(((UART_READ32(base_addr + UART_STATUS))&0x1FFC000)>>14));	
		}
		else
		{
          while(!(((UART_READ32(base_addr + UART_STATUS))&0x1FF000)>>12));	

		}
		
		mport->fn_write_byte(mport->nport,xmit->buf[xmit->tail]);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit)) 
		{      
			break;
		}
	} while (1);

	switch (mport->nport) 
	{
		case UART_PORT1:
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
				uart_write_wakeup(port);
			}
			break;
		case UART_PORT2:
		case UART_PORT3:
		case UART_PORT4:
		case UART_PORT5:
		case UART_PORT6:		
			if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
				uart_write_wakeup(port);
			}
			break;
		default:
			uart_err("Something wrong with PORT number.\n");
			dump_stack();
			break;
	}
}

static void ac83xx_uart_fifo_tasklet_rx(unsigned long arg)
{

	struct uart_port *port = (struct uart_port *)arg;
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)arg;
	struct atc_uart_char c;
	struct circ_buf *ring = &mport->rx_ring;

	spin_lock(&port->lock);
	while (ring->head != ring->tail)
	{
		if (mport->stop_fifo_tasklet)
		{
			break;
		}
		smp_rmb();
		c = ((struct atc_uart_char *)ring->buf)[ring->tail];
		ring->tail = (ring->tail + 1) & (ATC_SERIAL_RINGSIZE - 1);
		if (uart_handle_sysrq_char(&mport->port, c.ch)) 
		{
			continue;
		}

		//2.6.18. no overrun support, set status and mask to 0
		uart_insert_char(&mport->port, 0, 0, c.ch, c.status);
		spin_unlock(&port->lock);
		tty_flip_buffer_push(mport->port.state->port.tty);
		spin_lock(&port->lock);

	}
	spin_unlock(&port->lock);
}


static void ac83xx_uart_fifo_tasklet_tx(unsigned long arg)
{

	struct ac83xx_uart_port *mport;
    struct uart_port *port = NULL;
	mport = (struct ac83xx_uart_port *)arg;
	port = &mport->port;
	/* tx mode */
	//mdelay(50);
	while(1)
	{
		if (mport->stop_fifo_tasklet) 
		{
			break;
		}
		if (mport->fn_write_allow(mport->nport))
		{       
			//_ac83xx_uart_tx_chars(mport);
			break;
		}
	}
}


static irqreturn_t _ac83xx_uart0_interrupt(int irq, void *dev_id)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)dev_id;
	unsigned int uart_int_ident;
	int base_addr;
	base_addr = ac83xx_get_uart_base_addr(mport->nport);
	uart_int_ident = UART_READ32(UART0_INT_STATUS);
	if (!(uart_int_ident & (unsigned int)(U0_INTALL | U0_TBUF))) {
		return IRQ_HANDLED;
	}
	if(uart_int_ident & 0x20) {

		UART_WRITE32(0x20,base_addr+UART0_INT_STATUS);
		_ac83xx_uart0_tx_chars(mport);
	}
	if(uart_int_ident & 0x80){

		UART_WRITE32(0x20,base_addr+UART0_INT_STATUS);
		_ac83xx_uart0_rx_chars(mport);
	}
	ac83xx_mask_ack_bim_irq(irq);

	return IRQ_HANDLED;
}


static irqreturn_t ac83xx_uart_interrupt(int irq, void *dev_id)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)dev_id;
	unsigned int uart_int_ident;
	int base_addr;
	int data_len = 0;

#ifdef	UART_DMA_ACCOUNTING 
	int i;
	enum INTERRUPT_BIT{
		INTERRUPT_RXD_ERR,
		INTERRUPT_FIFO_RX,
		INTERRUPT_TIMEOUT,
		INTERRUPT_FIFO_TX,
		INTERRUPT_OVERRUN,
		INTERRUPT_DMA_TX,
		INTERRUPT_DMA_RX,
		}; 
	char* interrupt_string[] = {
		"INTERRUPT_RXD_ERR",
		"INTERRUPT_FIFO_RX",
		"INTERRUPT_TIMEOUT",
		"INTERRUPT_FIFO_TX",
		"INTERRUPT_OVERRUN",
		"INTERRUPT_DMA_TX",
		"INTERRUPT_DMA_RX",
		};
#endif
	base_addr = ac83xx_get_uart_base_addr(mport->nport);

	switch (mport->nport) 
	{
		case UART_PORT1:
		case UART_PORT2:
		case UART_PORT3:
		case UART_PORT4:
		case UART_PORT5:
		case UART_PORT6:
			uart_int_ident = UART_READ32(base_addr + UART_INT_STA);
			if(mport->dma_mode) 
			{
				if (!(uart_int_ident & (unsigned int)(0x75))) 
				{
					ac83xx_mask_ack_bim_irq(irq);
					return IRQ_HANDLED;
				}
#ifdef UART_DMA_ACCOUNTING 
				for(i = 0; i< 7; i++) 
				{
					if(uart_int_ident & (1 << i)) 
					{
						uart_info("current port%d interrupt is %s.\n", mport->nport, interrupt_string[i]);
					}
				}
#endif
               UART_WRITE32(uart_int_ident &  DMA_MODE_INT_ENABLE_VALUE, base_addr + UART_INT_STA);///////
				
				//UART_WRITE32(uart_int_ident &  DMA_MODE_INT_ENABLE_VALUE, base_addr + UART_INT_STA);

				// RX  || RX timeout
				if( (uart_int_ident & 0x40) || (uart_int_ident & 0x4) ) 
				{ 
					data_len  = UART_READ32(base_addr + UART_TRAN_DATA_NUM) & 0x7FFF;
					if(data_len > 0) 
					{
						ac83xx_uart_dma_read_data(mport);
						tasklet_hi_schedule(&mport->uart_tasklet_rx);
					}
				}

				if(uart_int_ident & U_DMA_R_EN)
				{ //TX 
					ac83xx_uart_dma_clear_tx_status(mport);
				}
				if(uart_int_ident & 0x10) // OVERRUN
			    {
					uart_err("[ac83xx_uart_interrupt][DMA]port %d OVERRUN INTERRUPT\n", mport->nport);
					uart_info("[ac83xx_uart_interrupt][DMA]port %d OVERRUN INTERRUPT\n", mport->nport);
					ac83xx_uart_dump_reg(mport->nport);
				}
				if(uart_int_ident & 0x01) // ERROR
			    {
					uart_info("[ac83xx_uart_interrupt][DMA]port %d  RXD_ERRE . statu Reg is 0x%x \n", mport->nport, UART_READ32(base_addr + UART_STATUS) );
					ac83xx_uart_dump_reg(mport->nport);
				}
				
#ifdef	UART_DMA_WORKQUEUE

                //if(uart_int_ident & 0x8)//TX
				//{
                //   tasklet_schedule(&mport->uart_tasklet_tx);
				//}	
#else
                if(uart_int_ident & 0x8)//TX
				{
                   tasklet_schedule(&mport->uart_tasklet_tx);
				}						
#endif		

			} 
			else
			{
			      /* FIFO mode :clear interrupt status*/
					UART_WRITE32(uart_int_ident, (base_addr + UART_INT_STA));
					
					if(uart_int_ident & 0x01) // ERROR
				    {
						uart_info("[ac83xx_uart_interrupt][FIFO]port %d  RXD_ERRE . statu Reg is 0x%x \r\n", mport->nport, UART_READ32(base_addr + UART_STATUS) );
						uart_info("port %d [RXD_ERRE][RX] head is %d  tail is %d \r\n", mport->nport,mport->rx_ring.head,mport->rx_ring.tail);
						uart_info("port %d [RXD_ERRE][TX] head is %d  tail is %d \r\n",mport->nport, mport->port.state->xmit.head,mport->port.state->xmit.tail);
						ac83xx_uart_dump_reg(mport->nport);
					}
					

					if(uart_int_ident & 0x10) // OVRUN
				    {
						uart_info("port %d OVERRUN INTERRUPT\n", mport->nport);
						uart_info("[ac83xx_uart_interrupt][FIFO]port %d OVERRUN INTERRUPT. status Reg is 0x%x \r\n", mport->nport, UART_READ32(base_addr + UART_STATUS));
						uart_info("port %d [OVERRUN][RX] head is %d  tail is %d \r\n", mport->nport,mport->rx_ring.head,mport->rx_ring.tail);
						uart_info("port %d [OVERRUN][TX] head is %d  tail is %d \r\n",mport->nport, mport->port.state->xmit.head,mport->port.state->xmit.tail);
						ac83xx_uart_dump_reg(mport->nport);
					}
					
					if(uart_int_ident & 0x2 || uart_int_ident & 0x4)//RX
					{	/* rx mode */
	
						//uart_info("uart_int_ident=0x%x,==RX or Timeout=== status Reg is 0x%x \n", uart_int_ident, UART_READ32(base_addr + UART_STATUS));
						_ac83xx_uart_rx_chars(mport);
						tasklet_schedule(&mport->uart_tasklet_rx);

					}
					
					if(uart_int_ident & 0x8)//TX
					{
	                  // tasklet_schedule(&mport->uart_tasklet_tx);
					}
				
				    if (!(uart_int_ident & (unsigned int)(UART_INTALL)))
					{
						ac83xx_mask_ack_bim_irq(irq);
						uart_info("[ac83xx_uart_interrupt][FIFO]port %d  IRQ_HANDLED \n", mport->nport);
						return IRQ_HANDLED;
					}					  
			}


			//	if(uart_int_ident & 0x8)//TX
			//	{
            //       tasklet_schedule(&mport->uart_tasklet_tx);
			//	}				
				/* tx mode :clear interrupt status*/
				//UART_WRITE32(0x1f,base_addr + UART_INT_STA);
			break;
			
		default:
			/* no others can do RS232 IRQ, tell kernel I handled that well */
			uart_err("Something wrong with PORT number.\n");
			ac83xx_mask_ack_bim_irq(irq);
			return IRQ_HANDLED;
	}
	
	ac83xx_mask_ack_bim_irq(irq);
	return IRQ_HANDLED;
}
static irqreturn_t ac83xx_uart1_interrupt(int irq, void *dev_id)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)dev_id;
	unsigned int uart_int_ident;
	unsigned int uart_int_enable;
	int base_addr = UART1_BASE;
	unsigned int  RxNum = 0;
	uint32_t u4Value;
	unsigned int data_byte;


	uart_int_ident = UART_READ32(base_addr + UART_INT_STA);
	uart_int_enable= UART_READ32(base_addr + UART_INT_EN);
	
   /* FIFO mode :clear interrupt status*/
	UART_WRITE32(uart_int_ident, (base_addr + UART_INT_STA));
	uart_int_ident &= uart_int_enable;
	
	if(uart_int_ident & 0x01) // ERROR
    {
		uart_info("[ac83xx_uart1_interrupt][FIFO]port %d  RXD_ERRE . statu Reg is 0x%x \r\n", mport->nport,  UART_READ32(base_addr + UART_STATUS) );
		uart_err("port %d [RXD_ERRE][RX] head is %d  tail is %d \r\n",  mport->nport,mport->rx_ring.head,mport->rx_ring.tail);
		uart_err("port %d [RXD_ERRE][TX] head is %d  tail is %d \r\n",mport->nport, mport->port.state->xmit.head,mport->port.state->xmit.tail);
		ac83xx_uart_dump_reg(mport->nport);
	}

	if(uart_int_ident & 0x10) // OVRUN
    {
		uart_info("[ac83xx_uart1_interrupt][FIFO]port %d OVERRUN INTERRUPT. uart_int_ident=0x%x, uart_int_enable=0x%x \r\n", mport->nport, uart_int_ident, uart_int_enable);
		uart_err("port %d [OVERRUN][RX] head is %d  tail is %d \r\n",  mport->nport,mport->rx_ring.head,mport->rx_ring.tail);
		uart_err("port %d [OVERRUN][TX] head is %d  tail is %d \r\n",mport->nport,  mport->port.state->xmit.head,mport->port.state->xmit.tail);
		ac83xx_uart_dump_reg(mport->nport);
	}
	
	if(uart_int_ident & 0x2 )//RX
	{	/* rx mode */
        RxNum = 0;
        RxNum = (UART_READ32(base_addr  + UART_STATUS) & UART1_RX_BUF_MASK);	

        while( RxNum > 0)
		{
			/* read the byte */
			data_byte = UART_READ32(base_addr + UART_DATA_BYTE);
            RxNum--;
			mport->port.icount.rx++;		
			atc_buffer_rx_char(mport, data_byte, TTY_NORMAL);
		}

		if (0x02 == (UART_READ32(base_addr+UART_INT_STA) & 0x02)) 
		{
			//uart_info("[ac83xx_uart1_interrupt ][Rx][Clear] Currect Irq status=0x%x\n", UART_READ32(base_addr+UART_INT_STA));
		    UART_WRITE32(0x02, base_addr + UART_INT_STA);  // clearn Rx irq
	    }
	}
	
	if( uart_int_ident & 0x4)//Timeout
	{	/* Timeout mode */

        while( (UART_READ32(base_addr  + UART_STATUS) & UART1_RX_BUF_MASK) )
		{
			/* read the byte */
			data_byte = UART_READ32(base_addr + UART_DATA_BYTE);
			mport->port.icount.rx++;		
			atc_buffer_rx_char(mport, data_byte, TTY_NORMAL);
		}
	}
	
	if(uart_int_ident & 0x8)//TX
	{
       // tasklet_schedule(&mport->uart_tasklet_tx);
        uart_info("[ac83xx_uart1_interrupt ][Tx] uart_int_ident=0x%x, uart_int_enable=0x%x. Currect Irq status=0x%x\n", uart_int_ident, uart_int_enable, UART_READ32(base_addr+UART_INT_STA));
      
	}

	ac83xx_mask_ack_bim_irq(irq);
	tasklet_schedule(&mport->uart_tasklet_rx);	
 
    return IRQ_HANDLED;
}


/*
 * uart ops
 */

static unsigned int _ac83xx_uart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT; //Set to not supporting now
}

static void _ac83xx_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	return;
}

static unsigned int _ac83xx_uart_get_mctrl(struct uart_port *port)
{
		unsigned int result = 0;

	result = TIOCM_CTS | TIOCM_CAR | TIOCM_DSR;
	return result;
}

static void _ac83xx_uart_stop_tx(struct uart_port *port)
{
	return;
}

static void _ac83xx_uart_start_tx(struct uart_port *port)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)port;
	if (mport->port.line == 0) {
		fgUartRegisterd = true;
		if (mport->fn_write_allow(mport->nport)) {
			_ac83xx_uart0_tx_chars(mport);
		}
	} else {
		if(mport->dma_mode) {
#ifdef	UART_DMA_WORKQUEUE
			queue_work(mport->uart_workqueue_tx,&mport->uart_work_tx);
#endif
		} else {
	        	while (!mport->fn_write_allow(mport->nport));
			_ac83xx_uart_tx_chars(mport);
		}
	}
	
}

static void _ac83xx_uart_stop_rx(struct uart_port *port)
{

	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)port;
	int base_addr = 0;
	base_addr = ac83xx_get_uart_base_addr(mport->nport);
	/* Force to stop rx/tx tasklset in case something is wrong */
	mport->stop_fifo_tasklet = true;

	//Disable INT
	if(mport->dma_mode) {
		UART_WRITE32(0xFF, base_addr + UART_INT_STA);
		UART_WRITE32(0, base_addr + UART_INT_EN);
	} else {
		mport->fn_int_enable(mport->nport,0);
	}

	if(mport->nport != 0) {
	//if(mport->nport == 1) {
		uart_info("UART PORT %d STOP RX\n", mport->nport);
		if(mport->dma_mode) {
				UART_WRITE32(0x40100FF, base_addr + UART_BUF_CTRL);
				uart_dbg("UART_BUF_CTRL = 0x%x.\n", UART_READ32(base_addr + UART_BUF_CTRL));
		} else {
			if(mport->nport == 1) {
				UART_WRITE32(0x80100FF, base_addr + UART_BUF_CTRL );
			} else {	
				UART_WRITE32(0x40100FF, base_addr + UART_BUF_CTRL);
			}
		}
	}

}

static void _ac83xx_uart_enable_ms(struct uart_port *port)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)port;
	mport->ms_enable = 1;
}

static void _ac83xx_uart_break_ctl(struct uart_port *port, int break_state)
{
	return;
}



static int _ac83xx_uart_startup(struct uart_port *port)
{
	struct ac83xx_uart_port *mport = container_of(port, struct ac83xx_uart_port, port);
	int retval;
	int base_addr;
	uint32_t u4Value,tmp;
	void *data;
	unsigned int restatus;

	base_addr = ac83xx_get_uart_base_addr(mport->nport) ;
  
	/*  For BT crash Recovery
		clear Tx Rx Buffer before interrupt enable
	    sometimes buffer is not empty after clean only once 
	*/
	if (mport->nport == UART_PORT1)
	{
		uint32_t tmp_bufctl;
		
		ac83xx_uart_dump_reg(mport->nport);
		
		tmp_bufctl = UART_READ32(base_addr + UART_BUF_CTRL);
		UART_WRITE32((tmp_bufctl | UART_CLEAR_RX_BUFFER | UART_CLEAR_TX_BUFFER),
			base_addr + UART_BUF_CTRL);//clear buffer

		ac83xx_uart_dump_reg(mport->nport);

		tmp_bufctl = UART_READ32(base_addr + UART_BUF_CTRL);
		UART_WRITE32((tmp_bufctl | UART_CLEAR_RX_BUFFER | UART_CLEAR_TX_BUFFER),
			base_addr + UART_BUF_CTRL);//clear buffer
		
		ac83xx_uart_dump_reg(mport->nport);
	}
  
	mport->stop_fifo_tasklet = false;
	uart_info("[_ac83xx_uart_startup][start]port number is %d \r\n", mport->nport);
	
	if(mport->nport > 0) 
	{
		if (ac83xx_uart_dma_is_enable(mport->nport)) 
		{
			uart_info("[_ac83xx_uart_startup]PORT %d use DMA mode \r\n", mport->nport);
			mport->dma_mode = 1;
			data = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, get_order(ATC_SERIAL_RINGSIZE));
			if(data==NULL) 
			{
				uart_err("_ac83xx_uart_startup __get_free_pages fail\r\n");	 
				return ENOMEM;
			} 
			else 
			{
				memset(&mport->rx_ring, 0, sizeof(struct circ_buf));
				mport->rx_ring.buf = data;
				spin_lock_init(&mport->rx_ring_lock);
			}
			
#ifdef	UART_DMA_WORKQUEUE
			mport->cancel_work = false;
			mport->uart_workqueue_tx = create_singlethread_workqueue("uart_dma_tx");
			if(mport->uart_workqueue_tx == NULL) {
				uart_err("Create workqueue failed\n");
				return -1; 
			}
			INIT_WORK(&mport->uart_work_tx, ac83xx_uart_dma_workqueue_tx);
#else
			tasklet_init(&mport->uart_tasklet_tx, ac83xx_uart_dma_tasklet_tx, (ulong)mport);
#endif
			tasklet_init(&mport->uart_tasklet_rx, ac83xx_uart_dma_tasklet_rx, (ulong)mport);

			/* for backcar 
			if(mport->nport == BACKCAR_PORT) {
				UART_WRITE32(0, base_addr + UART_INT_EN);
				UART_WRITE32(0, base_addr + UART_COMMCTRL);
				u4Value = IO_READ32(0xFD00040C,0);
				IO_WRITE32(0xFD00040C,0,(u4Value|0x780));
			}
			*/
		} 
		else 
		{
			uart_info("[_ac83xx_uart_startup]PORT %d use FIFO mode.\r\n", mport->nport);

			tasklet_init(&mport->uart_tasklet_rx, ac83xx_uart_fifo_tasklet_rx, (ulong)mport);
			tasklet_init(&mport->uart_tasklet_tx, ac83xx_uart_fifo_tasklet_tx, (ulong)mport);

			if (mport->rx_ring.buf == NULL)
			{
				uart_info("uart[%d]rx_ring.buf == NULL\r\n", mport->nport);
				data = kzalloc(sizeof(struct atc_uart_char) * ATC_SERIAL_RINGSIZE, GFP_KERNEL);
				if(data==NULL) {         
					uart_info("_ac83xx_uart_startup kzalloc fail\r\n");
	                return ENOMEM;
	            } else {
				    memset(&mport->rx_ring, 0, sizeof(struct circ_buf));
				    mport->rx_ring.buf = data;
	            }
			}

			if (mport->nport == UART_PORT1)
			{
              /* disable interrupt */
				UART_WRITE32(0, base_addr + UART_INT_EN);
			}

			/* for backcar */
			if(mport->nport == BACKCAR_PORT) 
			{
				/* disable interrupt */
				UART_WRITE32(0, base_addr + UART_INT_EN);
				/* set baud rate  to default value */
				UART_WRITE32(0, base_addr + UART_COMMCTRL);
				u4Value = IO_READ32(0xFD00040C,0);
				IO_WRITE32(0xFD00040C,0,(u4Value|0x780));

			}
		}

	}

	uart_info("ac83xxuart startup mport->nport = %d ,port->irq = %d\r\n",mport->nport,port->irq);

	_irq_allocated[mport->nport] ++;

	if (_irq_allocated[mport->nport] == 1) 
	{
		if(mport->nport!=0) 
		{
		     if(mport->nport== 1)
		     {
		         //------------------------------------------------------------
            	/* allocate irq, two ports share the same interrupt number */
             if (ac83xx_uart_dma_is_enable(mport->nport)) 
             {
                retval = request_irq(port->irq, ac83xx_uart_interrupt, IRQF_SHARED,	"AC83xx Serial", port);
             }
             else
             {
                retval = request_irq(port->irq, ac83xx_uart1_interrupt, IRQF_SHARED,	"AC83xx Serial", port);
             }
				irq_set_affinity(port->irq,get_cpu_mask(1));//link only cpu1
		        //--------------------------------------------------------------
				if (retval)
				{
					_irq_allocated[mport->nport] --;
					return retval;
				}
			 }
			 else
			 {
                	retval = request_irq(port->irq, ac83xx_uart_interrupt, IRQF_SHARED,	"AC83xx Serial", port);
					if (retval)
					{
						_irq_allocated[mport->nport] --;
						return retval;
					}
			 }
		
		}
		else
		{
			restatus = *(volatile uint32_t *)0xFD00C010;
			while((restatus&0x80)==0x80) 
			{
				restatus = *(volatile uint32_t *)0xFD00C010;               
				tmp =UART_READ32(UART0_DATA_BYTE);
				*(volatile uint32_t *)0xFD00C010 = (*(volatile uint32_t *)0xFD00C010)&0xFFFFFFDF;
				ac83xx_mask_ack_bim_irq(VECTOR_RS232_1);
			}
			retval = request_irq(VECTOR_RS232_1, _ac83xx_uart0_interrupt, IRQF_SHARED, 	"AC83xx Serial", port);
			if (retval)
			{
				_irq_allocated[mport->nport] --;
				return retval;
			}
		}
	}

	switch(mport->nport) {
	case UART_PORT0:
		UART_WRITE32(0xA0,base_addr+UART0_INT_STATUS);
		/* enable interrupt */
		mport->fn_empty_int_enable(mport->nport,1);
		mport->fn_int_enable(mport->nport,1);
		break;
	case UART_PORT1:
	case UART_PORT2:
	case UART_PORT3:
	case UART_PORT4:
	case UART_PORT5:
	case UART_PORT6:
		if (ac83xx_uart_dma_is_enable(mport->nport)) 
		{
			// clear Interr status write 1 to clear
			ac83xx_uart_dma_clear_interrupt(mport->nport, 0x75);

			// set DMA timeout & trigger level
			// note that dma buffer size 0.5K for all uart ports
			UART_WRITE32(0x40100FF, base_addr + UART_BUF_CTRL );

			mport->dma_curr_read_offset = 0;

			retval = ac83xx_uart_dma_init(mport);
		} 
		else 
		{
			if (mport->nport != UART_PORT1) 
			{
				UART_WRITE32(0x40000FF, base_addr + UART_BUF_CTRL );
			} 
			else 
			{
				UART_WRITE32(0x80000FF, base_addr + UART_BUF_CTRL );
				//UART_WRITE32(0x80001FF, base_addr + UART_BUF_CTRL );
			}
			UART_WRITE32(0x1f, base_addr + UART_INT_STA);///////
			/* enable interrupt */
			//mport->fn_empty_int_enable(mport->nport,1);//modify not enable  ////////
			mport->fn_int_enable(mport->nport,1);
		}
		break;
	default:
		uart_err("port num overflow.\n");
		break;
	}
	ac83xx_uart_dump_reg(mport->nport);
    uart_info("[_ac83xx_uart_startup][end]retval is %d \r\n",retval);
	return retval;
}

static void _ac83xx_uart_shutdown(struct uart_port *port)
{

    int base_addr;/////////
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)port;
	mport->stop_fifo_tasklet = true;

	
	base_addr = ac83xx_get_uart_base_addr(mport->nport) ;////////

	 uart_info("[_ac83xx_uart_shutdown][start]port is %d \r\n",mport->nport);
	 ac83xx_uart_dump_reg(mport->nport);
	/*
	 * FIXME: disable BIM IRQ enable bit if all ports are shutdown
	 */
	if (mport->dma_mode)
	{
		ac83xx_uart_dma_deinit(mport);
		mport->dma_mode = 0;

	} 
	else 
	{
	    UART_WRITE32(0x1f, base_addr + UART_INT_STA);///////////////
		/* disable interrupt and disable port */
		mport->fn_int_enable(mport->nport,0);

		tasklet_disable(&mport->uart_tasklet_rx);
		tasklet_kill(&mport->uart_tasklet_rx);
		tasklet_disable(&mport->uart_tasklet_tx);
		tasklet_kill(&mport->uart_tasklet_tx);
	}

	_irq_allocated[mport->nport] --;

	if (!_irq_allocated[mport->nport]) 
	{
		free_irq(port->irq, port);
	}

    uart_info("[_ac83xx_uart_shutdown][end] port is %d \r\n",mport->nport);
}


/*
   Change the port parameters, including word length, parity, stop
   bits.  Update read_status_mask and ignore_status_mask to indicate
   the types of events we are interested in receiving.  Relevant
   termios->c_cflag bits are:
   CSIZE	- word size
   CSTOPB	- 2 stop bits
   PARENB	- parity enable
   PARODD	- odd parity (when PARENB is in force)
   CREAD	- enable reception of characters (if not set,
   still receive characters from the port, but
   throw them away.
   CRTSCTS	- if set, enable CTS status change reporting
   CLOCAL	- if not set, enable modem status change
   reporting.
   Relevant termios->c_iflag bits are:
   INPCK	- enable frame and parity error events to be
   passed to the TTY layer.
   BRKINT
   PARMRK	- both of these enable break events to be
   passed to the TTY layer.

   IGNPAR	- ignore parity and framing errors
   IGNBRK	- ignore break errors,  If IGNPAR is also
   set, ignore overrun errors as well.
   The interaction of the iflag bits is as follows (parity error
   given as an example):
   Parity error	INPCK	IGNPAR
   n/a		0	n/a	character received, marked as
   TTY_NORMAL
   None		1	n/a	character received, marked as
   TTY_NORMAL
   Yes		1	0	character received, marked as
   TTY_PARITY
   Yes		1	1	character discarded

   Other flags may be used (eg, xon/xoff characters) if your
   hardware supports hardware "soft" flow control.

Locking: none.
Interrupts: caller dependent.
This call must not sleep
*/
static void _ac83xx_uart_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old)
{
	struct ac83xx_uart_port *mport = (struct ac83xx_uart_port *)port;
	unsigned long flags;
	int baud;
	int datalen;
	int parity = 0;
	int stopbit = 1;
	unsigned int uCOMMCTRL;

	if(mport->nport == 0)
		uCOMMCTRL = UART0_COMMCTRL;
	else   
		uCOMMCTRL = ac83xx_get_uart_base_addr(mport->nport) + UART_COMMCTRL ;

	/* calculate baud rate */
	baud = (int)uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);

	/* datalen : default 8 bit */
	switch(termios->c_cflag & CSIZE)
	{
		case CS5:
			datalen = 5;
			break;
		case CS6:
			datalen = 6;
			break;
		case CS7:
			datalen = 7;
			break;
		case CS8:
		default:
			datalen = 8;
			break;
	}
	/* stopbit : default 1 */
	if (termios->c_cflag & CSTOPB) {
		stopbit = 2;
	}
	/* parity : default none */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD) {
			parity = 1; /* odd */
		}
		else {
			parity = 2; /* even */
		}
	}
	/* lock from here */
	spin_lock_irqsave(&port->lock, flags);
	/* update per port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);
	/* read status mask */
	if (termios->c_iflag & INPCK) {
		/* frame error, parity error */
		port->read_status_mask |= UST_FRAME_ERROR | UST_PARITY_ERROR;
	}
	if (termios->c_iflag & (BRKINT | PARMRK)) {
		/* break error */
		port->read_status_mask |= UST_BREAK_ERROR;
	}
	/* status to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR) {
		port->ignore_status_mask |= UST_FRAME_ERROR | UST_PARITY_ERROR;
	}
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= UST_BREAK_ERROR;
		if (termios->c_iflag & IGNPAR) {
			port->ignore_status_mask |= UST_OVRUN_ERROR;
		}
	}
	if ((termios->c_cflag & CREAD) == 0) {
		// dummy read
		port->ignore_status_mask |= UST_DUMMY_READ;
	}

	if (mport->nport == 0)
		_ac83xx_uart0_set(uCOMMCTRL, baud, datalen, stopbit, parity);
	else
		_ac83xx_uartx_set(uCOMMCTRL, baud, datalen, stopbit, parity);

	/* disable modem status interrupt */
	mport->ms_enable = 0;
	if (UART_ENABLE_MS(port, termios->c_cflag)) {
		// enable modem status interrupt
		mport->ms_enable = 1;
	}
	/* unlock here */
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *_ac83xx_uart_type(struct uart_port *port)
{
	return "AC83XX Serial";
}

static void _ac83xx_uart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, AC83XX_UART_SIZE);
}

static int _ac83xx_uart_request_port(struct uart_port *port)
{
	void *pv_region;

	pv_region = request_mem_region(port->mapbase, AC83XX_UART_SIZE,
				"AC83XX Uart IO Mem");

	return pv_region != NULL ? 0 : -EBUSY;
}

static void _ac83xx_uart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_AC83XX;
		_ac83xx_uart_request_port(port);
	}
}

static int _ac83xx_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_AC83XX) {
		ret = -EINVAL;
	}
	if (ser->irq != port->irq) {
		ret = -EINVAL;
	}
	if (ser->baud_base < 110) {
		ret = -EINVAL;
	}
	return ret;
}

static struct uart_ops _ac83xx_uart_ops =
{
	.tx_empty       = _ac83xx_uart_tx_empty,
	.set_mctrl      = _ac83xx_uart_set_mctrl,
	.get_mctrl      = _ac83xx_uart_get_mctrl,
	.stop_tx        = _ac83xx_uart_stop_tx,
	.start_tx       = _ac83xx_uart_start_tx,
	/* .send_xchar */
	.stop_rx        = _ac83xx_uart_stop_rx,
	.enable_ms      = _ac83xx_uart_enable_ms,
	.break_ctl      = _ac83xx_uart_break_ctl,
	.startup        = _ac83xx_uart_startup,
	.shutdown       = _ac83xx_uart_shutdown,
	.set_termios    = _ac83xx_uart_set_termios,
	/* .pm */
	/* .set_wake */
	.type           = _ac83xx_uart_type,
	.release_port   = _ac83xx_uart_release_port,
	.request_port   = _ac83xx_uart_request_port,
	.config_port    = _ac83xx_uart_config_port,
	.verify_port    = _ac83xx_uart_verify_port,
	/* .ioctl */
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char  = _ac83xx_u0_write_char,
	.poll_get_char  = _ac83xx_u0_read_char,
#endif
};

static struct ac83xx_uart_port _ac83xx_uart_ports[] =
{
	[0] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_RS232_1,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 0,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[0].port.lock),
		},
		.nport              = 0,
		.ms_enable          = 0,
		.fn_read_allow      = _ac83xx_u0_read_allow,
		.fn_write_allow     = _ac83xx_u0_write_allow,
		.fn_int_enable      = _ac83xx_u0_int_enable,
		.fn_empty_int_enable    = _ac83xx_u0_empty_int_enable,
		.fn_read_byte       = _ac83xx_u0_read_byte,
		.fn_write_byte      = _ac83xx_u0_write_byte,
		.fn_flush           = _ac83xx_u0_flush,
		.fn_get_top_err     = _ac83xx_u0_get_top_err,
	},
#if UART_NR > 1
	[1] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART1,
			.uartclk        = 3200000 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 1,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[1].port.lock),
		},
		.nport              = 1,
		.ms_enable          = 0,
		.fn_read_allow      = _ac83xx_uartx_read_allow,
		.fn_write_allow     = _ac83xx_uartx_write_allow,
		.fn_int_enable      = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte       = _ac83xx_uartx_read_byte,
		.fn_write_byte      = _ac83xx_uartx_write_byte,
		.fn_flush           = _ac83xx_uartx_flush,
		.fn_get_top_err     = _ac83xx_uartx_get_top_err,
	},
#endif    

#if UART_NR > 2
	[2] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART2,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 2,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[2].port.lock),
		},
		.nport              = 2,
		.ms_enable          = 0,
		.fn_read_allow      = _ac83xx_uartx_read_allow,
		.fn_write_allow     = _ac83xx_uartx_write_allow,
		.fn_int_enable      = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte       = _ac83xx_uartx_read_byte,
		.fn_write_byte      = _ac83xx_uartx_write_byte,
		.fn_flush           = _ac83xx_uartx_flush,
		.fn_get_top_err     = _ac83xx_uartx_get_top_err,
	},
#endif

#if UART_NR > 3
	[3] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART3,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 3,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[3].port.lock),
		},
		.nport                  = 3,
		.ms_enable              = 0,
		.fn_read_allow          = _ac83xx_uartx_read_allow,
		.fn_write_allow         = _ac83xx_uartx_write_allow,
		.fn_int_enable          = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte           = _ac83xx_uartx_read_byte,
		.fn_write_byte          = _ac83xx_uartx_write_byte,
		.fn_flush               = _ac83xx_uartx_flush,
		.fn_get_top_err         = _ac83xx_uartx_get_top_err,
	},
#endif
#if UART_NR > 4
	[4] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART4,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 4,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[4].port.lock),
		},
		.nport                  = 4,
		.ms_enable              = 0,
		.fn_read_allow          = _ac83xx_uartx_read_allow,
		.fn_write_allow         = _ac83xx_uartx_write_allow,
		.fn_int_enable          = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte           = _ac83xx_uartx_read_byte,
		.fn_write_byte          = _ac83xx_uartx_write_byte,
		.fn_flush               = _ac83xx_uartx_flush,
		.fn_get_top_err         = _ac83xx_uartx_get_top_err,
	},
#endif
#if UART_NR > 5
	[5] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART5,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 5,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[5].port.lock),
		},
		.nport                  = 5,
		.ms_enable              = 0,
		.fn_read_allow          = _ac83xx_uartx_read_allow,
		.fn_write_allow         = _ac83xx_uartx_write_allow,
		.fn_int_enable          = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte           = _ac83xx_uartx_read_byte,
		.fn_write_byte          = _ac83xx_uartx_write_byte,
		.fn_flush               = _ac83xx_uartx_flush,
		.fn_get_top_err         = _ac83xx_uartx_get_top_err,
	},
#endif

#if UART_NR > 6
	[6] = {
		.port =
		{
			.membase        = (void*)AC83XX_VA_UART,
			.mapbase        = AC83XX_PA_UART,
			.iotype         = SERIAL_IO_MEM,
			.irq            = VECTOR_UART6,
			.uartclk        = 921600 * 16,
			.fifosize       = UART_FIFO_SIZE,
			.ops            = &_ac83xx_uart_ops,
			.flags          = ASYNC_BOOT_AUTOCONF,
			.line           = 6,
			.lock           = __SPIN_LOCK_UNLOCKED(_ac83xx_uart_ports[6].port.lock),
		},
		.nport                  = 6,
		.ms_enable              = 0,
		.fn_read_allow          = _ac83xx_uartx_read_allow,
		.fn_write_allow         = _ac83xx_uartx_write_allow,
		.fn_int_enable          = _ac83xx_uartx_int_enable,
		.fn_empty_int_enable    = _ac83xx_uartx_empty_int_enable,
		.fn_read_byte           = _ac83xx_uartx_read_byte,
		.fn_write_byte          = _ac83xx_uartx_write_byte,
		.fn_flush               = _ac83xx_uartx_flush,
		.fn_get_top_err         = _ac83xx_uartx_get_top_err,
	}
#endif
};

/*
 * console
 */
#ifdef CONFIG_SERIAL_AC83XX_CONSOLE
static void _ac83xx_uart_console_write(struct console *co, const char *s, unsigned int count)
{
	struct ac83xx_uart_port *mport;
	struct uart_port *port;
	struct circ_buf *xmit; 
	int i;
	int c = 0;
	int c_all = 0;
	int total = (int)count;
	unsigned long flags;
	char *buf =( char *)s;
	uint32_t u4Value;
	

	if (!_ac83xx_u0_trans_mode_on()) {
		return;
	}

	if (co->index >= UART_NR) {
		return;
	}

	mport = &_ac83xx_uart_ports[co->index];
	port  = &mport->port;

	if ((fgUartRegisterd)&&(co->cflag != 0xFF)) 
	{
		
	    spin_lock(&RS232_lock);
		xmit = &port->state->xmit;
		c     = CIRC_SPACE_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
		c_all = CIRC_SPACE(xmit->head, xmit->tail, UART_XMIT_SIZE);
		if (total <=c)
		{
	        c = total;
		  	memcpy(xmit->buf + xmit->head, buf, c);
			xmit->head = (xmit->head + c) & (UART_XMIT_SIZE - 1);
			buf += c;
			total -= c;

		}
		else if (total <= c_all )
		{
	          memcpy(xmit->buf + xmit->head, buf, c);
			  xmit->head = (xmit->head + c) & (UART_XMIT_SIZE - 1);
			  buf += c;
			  total -= c;
	          if (total >0)
	          {
	              memcpy(xmit->buf + xmit->head, buf, total);
				  xmit->head = (xmit->head + total) & (UART_XMIT_SIZE - 1);
				  buf += total;
				  total -= total;
			  }
			  					 
		}
		else if (total > c_all)
		{
	          memcpy(xmit->buf + xmit->head, buf, c);
			  xmit->head = (xmit->head + c) & (UART_XMIT_SIZE - 1);
			  buf += c;
			  total -= c;
	          if (total >0)
	          {
	              memcpy(xmit->buf + xmit->head, buf, (c_all - c) );
				  xmit->head = (xmit->head + (c_all - c)) & (UART_XMIT_SIZE - 1);
				  buf += (c_all - c);
				  total -= (c_all - c);
			  }
			  

		}
		else
		{

		}
		spin_unlock(&RS232_lock);	

		_ac83xx_uart_start_tx((struct uart_port *)mport);

	}
	else 
	{
		//for flush circ_buf
		if (fgUartRegisterd) 
		{
			local_irq_save(flags);

			while (!uart_circ_empty(xmit))
			{
				if (mport->fn_write_allow(mport->nport)) 
				{
					_ac83xx_uart_tx_chars(mport);
				}
			}
			
			local_irq_restore(flags);
		}

		for (i = 0; i < count; i++) 
		{
			while (!mport->fn_write_allow(mport->nport)) 
			{
				barrier();
			}

			mport->fn_write_byte(mport->nport,s[i]);


			if (s[i] == '\n') 
			{
				while (!mport->fn_write_allow(mport->nport))
				{
					barrier();
				}
				mport->fn_write_byte(mport->nport,'\r');

			}
		}
	}

}


static void __init _ac83xx_uart_console_get_options(struct uart_port *port,
		int *pbaud, int *pparity, int *pbits)
{
	int baud=0, parity=0, stopbit, datalen;
	unsigned int uCOMMCTRL;


	uCOMMCTRL = UART0_COMMCTRL;

	_ac83xx_uart_get(uCOMMCTRL, &baud, &datalen, &stopbit, &parity);

	if (pbaud) {
		*pbaud = baud;
	}

	if (pparity) {
		switch (parity) {
		case 1:
			*pparity = 'o';
			break;
		case 2:
			*pparity = 'e';
			break;
		case 0:
		default:
			*pparity = 'n';
			break;
		}
	}

	if (pbits) {
		*pbits = datalen;
	}

}

static int __init _ac83xx_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud    = 115200;
	int bits    = 8;
	int parity  = 'n';
	int flow    = 'n';
	int stopbit;
	int ret;

	if (co->index >= UART_NR)
		co->index = 0;
	port = (struct uart_port *)&_ac83xx_uart_ports[co->index];

	if (options) 
	{
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	} 
	else 
	{
		_ac83xx_uart_console_get_options(port, &baud, &parity, &stopbit);
	}

	ret = uart_set_options(port, co, baud, parity, bits, flow);
	uart_info("ac83xx console setup : uart_set_option port(%d) "
			"baud(%d) parity(%c) bits(%d) flow(%c) - ret(%d)\n",
			co->index, baud, parity, bits, flow, ret);
	return ret;
}

static struct uart_driver _ac83xx_uart_reg;

static struct console _ac83xx_uart_console =
{
	.name       = "ttyMT",
	.write      = _ac83xx_uart_console_write,
	.device     = uart_console_device,
	.setup      = _ac83xx_uart_console_setup,
	.flags      = CON_PRINTBUFFER,
	.index      = -1,
	.data       = &_ac83xx_uart_reg,
};

static int __init _ac83xx_uart_console_init(void)
{
	// set to transparent if boot loader doesn't do this
	/*_ac83xx_u0_set_trans_mode_on();*/
	register_console(&_ac83xx_uart_console);

	return 0;
}

console_initcall(_ac83xx_uart_console_init);

static int __init _ac83xx_late_console_init(void)
{
	if (!(_ac83xx_uart_console.flags & CON_ENABLED)) 
	{
		register_console(&_ac83xx_uart_console);
	}

	return 0;
}

late_initcall(_ac83xx_late_console_init);

#define AC83XX_CONSOLE &_ac83xx_uart_console
#else

#define AC83XX_CONSOLE NULL

#endif /* CONFIG_SERIAL_AC83XX_CONSOLE */

static struct uart_driver _ac83xx_uart_reg =
{
	.owner          = THIS_MODULE,
	.driver_name    = "AC83XX serial",
	.dev_name       = "ttyMT",
	.major          = SERIAL_AC83XX_MAJOR,
	.minor          = SERIAL_AC83XX_MINOR,
	.nr             = UART_NR,
	.cons           = AC83XX_CONSOLE,
};

static int _ac83xx_uart_probe(struct platform_device *pdev)
{
	return 0;
}

static int _ac83xx_uart_remove(struct platform_device *pdev)
{

	int ret = 0;
	int i;
	for (i = 0; i < UART_NR; i++) {
		ret = uart_remove_one_port(&_ac83xx_uart_reg, &_ac83xx_uart_ports[i].port);
		uart_info("remove uart port %d , ret = %d\n", i, ret);
	}
	return 0;
}

static int _ac83xx_uart_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef	UART_DMA_ACCOUNTING
	int i;
	for (i = 0; i < UART_NR; i++) {  
		_ac83xx_uart_ports[i].rx_count = 0;
		_ac83xx_uart_ports[i].tx_count = 0;
	}
#endif
	uart_info("UART Suspend.\n");
	return 0;
}

static int _ac83xx_uart_resume(struct platform_device *pdev)
{
    uart_info("ac83xx_uart_resume come\n");
	_ac83xx_uart_hwinit(); 
	ac83xx_mask_ack_bim_irq( VECTOR_RS232_1);
	_ac83xx_u0_empty_int_enable(0,1); 
	_ac83xx_u0_int_enable(0,1);   
    GPIO_MultiFun_Set(PIN_143_URXD2,UART2_SEL);
    GPIO_MultiFun_Set(PIN_155_UTXD2,UART2_SEL);
	// HAL_WRITE32(0xFD00C00C,0xA0);
	uart_info("UART Resume.\n");
	return 0;
}

static struct platform_driver _ac83xx_uart_driver =
{
	.remove         = _ac83xx_uart_remove,
	.suspend        = _ac83xx_uart_suspend,
	.resume         = _ac83xx_uart_resume,

	.driver  = {
		.name           = "AC83XX-UART",
		.owner = THIS_MODULE,
	},
};


static void inputtimer_fun(void)
{
	inputenable = 1;
	printk("###fun inputenable=%d\n", inputenable);
}

static void inputtimer(void)
{
	init_timer(&input_timer);
	input_timer.data = 0;
	input_timer.expires = jiffies + 2*HZ;
	input_timer.function = inputtimer_fun;
	add_timer(&input_timer);
}
/*
 * init, exit and module
 */
static struct platform_device *pdev = NULL;

#define DRIVER_VERSION		"AC83XX UART driver $Revision: #24\n"
static int __init _ac83xx_uart_init(void)
{
	int ret;
	int i;
	void *data;

	MOD_VERSION_INFO("ac83xx_uart",UART_DMA_VER_MAIN,UART_DMA_VER_MINOR,UART_DMA_VER_REV);
	/* reset hardware */
	_ac83xx_uart_hwinit();  /* all port 115200, no int */

	uart_info(DRIVER_VERSION);
	ret = uart_register_driver(&_ac83xx_uart_reg);

	if (ret) {
		uart_err("uart_register_driver failure %d\n", ret);
		goto out;
	}

	for (i = 0; i < UART_NR; i++)
		_irq_allocated[i] = 0 ;

	for (i = 0; i < UART_NR; i++) {
        data = NULL;
		data = kzalloc(sizeof(struct atc_uart_char) * ATC_SERIAL_RINGSIZE, GFP_KERNEL);
		if(data==NULL) {         
			uart_info("_ac83xx_uart_probe kzalloc fail\r\n");
			goto error;
	    } else {
		    uart_info(KERN_INFO "kzalloc uart port[%d]\n", i);
		    memset(&_ac83xx_uart_ports[i].rx_ring, 0, sizeof(struct circ_buf));
		    _ac83xx_uart_ports[i].rx_ring.buf = data;
	    }
	}

	for (i = 0; i < UART_NR; i++) {  
		ret = uart_add_one_port(&_ac83xx_uart_reg, &_ac83xx_uart_ports[i].port);
		uart_info(KERN_INFO "add uart port %d , line =%d  ret = %x\n", i, _ac83xx_uart_ports[i].port.line, ret);
	}
	pdev = platform_device_alloc("AC83XX-UART", 0);
	platform_device_add(pdev);
	platform_driver_probe(&_ac83xx_uart_driver, _ac83xx_uart_probe);
	inputtimer();
out:
	return ret;
error:
	for (i = 0; i < UART_NR; i++) {
		if (_ac83xx_uart_ports[i].rx_ring.buf != NULL) {
			kfree(_ac83xx_uart_ports[i].rx_ring.buf); 
			_ac83xx_uart_ports[i].rx_ring.buf = NULL;
		}
	}
	return ENOMEM;
}

static void __exit _ac83xx_uart_exit(void)
{
	uart_unregister_driver(&_ac83xx_uart_reg);
}

module_init(_ac83xx_uart_init);
module_exit(_ac83xx_uart_exit);

MODULE_AUTHOR("AutoChips Inc.");
MODULE_DESCRIPTION("AC83XX serial port driver.");
MODULE_LICENSE("GPL");
