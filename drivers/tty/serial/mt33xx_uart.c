
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


#if defined(CONFIG_SERIAL_MT33XX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
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
//#include <mach/hardware.h>
//#include <x_typedef.h>
#include "mt33xx_uart.h"


#define MT33XX_UART_DEBUG   0

/* support uart0(debug port) and uart 1~6  */
#define MT3360_UART_NR          6
#define UART_NR                 (MT3360_UART_NR)

#define SERIAL_MT33XX_MAJOR     204
#define SERIAL_MT33XX_MINOR     16

/*defines for log Async mode: only for UART0*/
#define SW_FIFO_SIZE	        16384//65536//1024//256

#define UART_PORT0              0 
#define UART_PORT1              1
#define UART_PORT2              2 
#define UART_PORT3              3
#define UART_PORT4              4 
#define UART_PORT5              5
#define UART_PORT6              6

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

#if 0 // comment by mtk40505
extern uint32_t disable_print;
#endif
struct SWFIFO_T {
    uint8_t    aData[SW_FIFO_SIZE];
    uint32_t   u4Wp;
    uint32_t   u4Rp;
    uint32_t   u4DiscardCnt;
    uint32_t   u4IntCnt;
    uint32_t   u4Overflow;
    uint32_t   u4TotalRWCnt;
    //HANDLE_T  h_sema_hdl;	//semaphore handler
};

static struct SWFIFO_T _arTxSWFIFO_U0;  //UART0
static bool fgUartRegisterd = false;

bool g_UartAsyncMode = false;

#define TXFIFO_DATASIZE(u1Port) (((_arTxSWFIFO_U0.u4Wp >= _arTxSWFIFO_U0.u4Rp)) ? \
                                 (_arTxSWFIFO_U0.u4Wp - _arTxSWFIFO_U0.u4Rp) : \
                                 ((SW_FIFO_SIZE + _arTxSWFIFO_U0.u4Wp) - _arTxSWFIFO_U0.u4Rp))
#define TXFIFO_FREESIZE(u1Port) ((SW_FIFO_SIZE - TXFIFO_DATASIZE( u1Port)) - 1)
#define TXFIFO_EMPTY(u1Port)    (_arTxSWFIFO_U0.u4Wp == _arTxSWFIFO_U0.u4Rp)
#define TXFIFO_PW_DATA(u1Port)  (_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Wp])
#define TXFIFO_PR_DATA(u1Port)  (_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Rp])

#define RXFIFO_DATASIZE(u1Port) (((_arTxSWFIFO_U0.u4Wp >=_arTxSWFIFO_U0.u4Rp)) ? \
                                 (_arTxSWFIFO_U0.u4Wp - _arTxSWFIFO_U0.u4Rp) : \
                                 ((SW_FIFO_SIZE + _arTxSWFIFO_U0.u4Wp) - _arTxSWFIFO_U0.u4Rp))
#define RXFIFO_FREESIZE(u1Port) ((SW_FIFO_SIZE - RXFIFO_DATASIZE( u1Port)) - 1)
#define RXFIFO_EMPTY(u1Port)    (_arTxSWFIFO_U0.u4Wp == _arTxSWFIFO_U0.u4Rp)
#define RXFIFO_PW_DATA(u1Port)  (_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Wp])
#define RXFIFO_PR_DATA(u1Port)  (_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Rp])
#define RXFIFO_CLEAR_BUF(u1Port)  (_arTxSWFIFO_U0.u4Wp = _arTxSWFIFO_U0.u4Rp)

/*
 * Port
 */
struct mt33xx_uart_port
{
    struct uart_port port;
    int nport;
    unsigned int old_status;
    unsigned int tx_stop;
    unsigned int rx_stop;
    unsigned int ms_enable;

    unsigned int    (*fn_read_allow)  (int);
    unsigned int    (*fn_write_allow) (int);
    void            (*fn_int_enable)  (int,int enable);
    void            (*fn_empty_int_enable) (int,int enable);
    unsigned int    (*fn_read_byte)   (int);
    void            (*fn_write_byte)  (int,unsigned int byte);
    void            (*fn_flush)       (int);
    void            (*fn_get_top_err) (int,int *p_parity, int *p_end, int *p_break);
};

/*
 * Locals
 */

static unsigned int _mt33xx_get_uartx_base_addr(int nport);
static void _mt33xx_uart_stop_tx(struct uart_port *port);

/*
 * HW functions of mt3360 uart
 */
 
/*
#define UART_READ8(REG)             __raw_readb(RS232_VIRT + REG)
#define UART_READ16(REG)            __raw_readw(RS232_VIRT + REG)
#define UART_WRITE8(VAL, REG)       __raw_writeb(VAL, RS232_VIRT + REG)
#define UART_WRITE16(VAL, REG)      __raw_writew(VAL, RS232_VIRT + REG)
*/

#define UART_READ32(REG)            __raw_readl(RS232_VIRT + REG)
#define UART_WRITE32(VAL, REG)      __raw_writel(VAL, RS232_VIRT + REG)

#define CKGEN_VIRT_READ32(REG)            __raw_readl(CKGEN_VIRT + REG)
#define CKGEN_VIRT_WRITE32(VAL, REG)      __raw_writel(VAL, CKGEN_VIRT + REG)

/*
 * Macros
 */
#define UART_REG_BITCLR(BITS, REG)      UART_WRITE32(UART_READ32(REG) & ~(BITS), REG)
#define UART_REG_BITSET(BITS, REG)      UART_WRITE32(UART_READ32(REG) | (BITS), REG)
#define UART0_FLUSH()   UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), UART0_BUFCTRL)
#define UART1_FLUSH()   UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), UART1_BUFCTRL)


//#define  UART0_EN  (1L<<11)
#define  UART1_PINMUX_EN  (1L<<11)
#define  UART2_PINMUX_EN  (1L<<13)
#define  UART3_PINMUX_EN  (1L<<16)
#define  UART4_PINMUX_EN  (1L<<18)
#define  UART5_PINMUX_EN  (1L<<20)
#define  UART6_PINMUX_EN  (1L<<23)


#define UART_PINMUX_OFFSET            (0x006C)
#define UART_PINMUX_WRITE(value)      CKGEN_VIRT_WRITE32((value), UART_PINMUX_OFFSET)
#define UART_PINMUX_REG()             CKGEN_VIRT_READ32(UART_PINMUX_OFFSET)

/*
 * used for dsp debug only
 */
 #define  UART_DSP_DEBUG        1
 #define  CONFIG_DRV_MT8520     0
 
 #if UART_DSP_DEBUG

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

 #define  DRAM_READ32(addr)               HAL_READ32(phys_to_virt((unsigned long)(addr)))
 #define  DRAM_WRITE32(addr, val)         HAL_WRITE32(phys_to_virt((unsigned long)(addr)), (val))

 #define  USE_MEMORY_MAPING       1
 
#if (CONFIG_DRV_MT8520) 
#define DSP_CKGEN_BASE           0xD0000
#else
#define DSP_CKGEN_BASE           0x00000
#endif
#define MAX_RS232W_NUM          80

#define R232_READREG            0
#define R232_WRITEREG           1
#define R232_WRITEDRAM          5
#define R232_READDRAM           9
#define R232_RISC_WRITEDRAM     12
#define R232_RISC_READDRAM      13
#define RS232_WRITE_CACHE       0x1C

//---------------------------------------------------------------------------
// Static variables
//---------------------------------------------------------------------------
//static bool _fgUart2Initialed = FALSE;
//static bool _fgRs232Int;
static uint32_t _dwR232Addr;
static uint32_t _dwR232Length;
static uint32_t _dwRS232ResetStatus;

static uint32_t _dwR232OpCnt;
static uint32_t _dwR232OpCode;
static uint32_t dwCnt=0;

//extern functions.
extern void mt85xx_mask_ack_irq(unsigned int irq);

//---------------------------------------------------------------------------
// local functions
//---------------------------------------------------------------------------

// *********************************************************************
// Function : void vRs232WriteData(DWRD dData)
// Description : Write to RS232 port
// Parameter : dData: Data to write
// Return    : None
// *********************************************************************
void vRs232WriteData(uint32_t dData)
{
  //UART_WRITE32(0, dData);
  UART_WRITE32(dData, UART0_DATA_BYTE);
}

// *********************************************************************
// Function : void vRs232WriteData(DWRD dData)
// Description : Write to RS232 port
// Parameter : dData: Data to write
// Return    : None
// *********************************************************************
void vRs232NonTransparentMode(void)
{
  //UART_WRITE32(4, 0x2);
  UART_WRITE32(0X2, UART0_STATUS);
}

// *********************************************************************
// Function : DWRD dRs232ReadData(void)
// Description : Read from RS232 port
// Parameter : None
// Return    : RS232 Data
// *********************************************************************
uint32_t dRs232ReadData(void)
{
  uint32_t dRegVal;

  dRegVal = UART_READ32(0x28);
  dRegVal = UART_READ32(0);
  return(dRegVal);
}

// *********************************************************************
// Function : DWRD dRs232Status(void)
// Description : Read RS232 port status
// Parameter : None
// Return    : RS232 Status
// *********************************************************************
uint32_t dRs232Status(void)
{
  return(UART_READ32(0x4));
}


// *********************************************************************
// Function : DWRD dRs232Status(void)
// Description : Read RS232 port status
// Parameter : None
// Return    : RS232 Status
// *********************************************************************
bool fgIsRs232RdAllow(void)
{
  return((dRs232Status() & 0x1) > 0);
}


// *********************************************************************
// Function : DWRD dRs232Status(void)
// Description : Read RS232 port status
// Parameter : None
// Return    : RS232 Status
// *********************************************************************
bool fgIsRs232WrAllow(void)
{
  return((dRs232Status() & 0x2) > 0);
}


// *********************************************************************
// Function : void vRs232Write(DWRD dwValue)
// Description : Out data to RS232 queue
// Parameter : dwValue : Value output to RS232 port
// Return    :
// *********************************************************************
void vRs232Write(uint32_t dwValue)
{
//  OssTakeSem(&_rSemRs232, OSS_SUSPEND);
  while (!fgIsRs232WrAllow())
  {
    ;
  }
  vRs232WriteData(dwValue);
//  OssGiveSem(&_rSemRs232);
  return;
}
//---------------------------------------------------------------------------
// Static functions
//---------------------------------------------------------------------------
// *********************************************************************
// Function : DWRD dRs232ReadDram(DWRD dAddr)
// Description : General Read dram
// Parameter : dAddr: Address of dram,
// Return    : dram value
// *********************************************************************
static uint32_t dRs232ReadDram(uint32_t dAddr)
{
  uint32_t dRegVal;
  
  dRegVal = DRAM_READ32(dAddr);

  return(dRegVal);
}

//---------------------------------------------------------------------------
// Static functions
//---------------------------------------------------------------------------
// *********************************************************************
// Function : DWRD dRs232WriteDram(DWRD dAddr)
// Description : General Read Register
// Parameter : dAddr: Address of register, -IO address
// Return    : None
// *********************************************************************
static uint32_t dRs232WriteDram(uint32_t dAddr, uint32_t dValue)
{
  uint32_t dRegVal;
  
  dRegVal = DRAM_WRITE32(dAddr,dValue);

  return(dRegVal);
}

//---------------------------------------------------------------------------
// Static functions
//---------------------------------------------------------------------------
// *********************************************************************
// Function : DWRD dRs232ReadREG(DWRD dAddr)
// Description : General Read Register
// Parameter : dAddr: Address of register, -IO address
// Return    : None
// *********************************************************************
static uint32_t dRs232ReadREG(uint32_t dAddr)
{
  uint32_t dRegVal;
  dAddr <<= 2;
  dAddr  &= 0x000FFFFFL;    // for accessing DSP C registers (0x47XXX)
  //dRegVal = IO_READ32(CKGEN_BASE, dAddr);
  dRegVal = CKGEN_READ32(dAddr);

  return(dRegVal);
}

// *********************************************************************
// Function : void vRs232WriteREG(DWRD dAddr, DWRD dValue)
// Description : General Write Register
// Parameter : dAddr: Address of register, -IO address
//             dValue: value to write
// Return    : None
// *********************************************************************
static void vRs232WriteREG(uint32_t dAddr, uint32_t dValue)
{
  dAddr <<= 2;
  dAddr  &= 0x000FFFFFL;    // for accessing DSP C registers (0x47XXX)
  //IO_WRITE32(CKGEN_BASE, dAddr, dValue);
  CKGEN_WRITE32(dValue, dAddr);

}

// *********************************************************************
// Function : void _Uart2Isr(UINT16 u2Vector)
// Description : General Write Register
// Parameter : dAddr: Address of register, -IO address
//             dValue: value to write
// Return    : None
// *********************************************************************
void _Uart2Isr(uint32_t  irq)
{
  uint32_t dwRsData;
  uint32_t dwValue;
  uint32_t *pdwAddr;
  uint32_t dwTemp;
//  uint8_t bTmp;
//  bool fgResult=FALSE;  
  
  dwCnt=dwCnt+1;

    // input data increment
    dwRsData = dRs232ReadData();
    
    
    // begin to parse a new command
    if((_dwR232OpCnt == 0))
    {
      _dwR232OpCode = dwRsData;
      // count the number of op
      _dwR232OpCnt = 1;
    }
    else                        // count = 1 or 2
    {
      _dwR232OpCnt++;           // count = 2 or 3
      
      switch (_dwR232OpCode)
      {
        case R232_READREG:      //---------------------------------------
          _dwR232Addr = dwRsData;
          // send header of 1 word
          vRs232Write(0xc0000400);
          dwTemp = dRs232ReadREG(_dwR232Addr);
          vRs232Write(dwTemp);
          _dwR232OpCnt = 0;
          break;
        case R232_WRITEREG:     //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
            _dwR232Addr = dwRsData;
          }
          else                    // count == 3
          {
            dwValue = dwRsData;
            vRs232WriteREG(_dwR232Addr, dwValue);
            _dwR232OpCnt = 0;
          }
          break;
        case R232_WRITEDRAM:     //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
            _dwR232Addr = dwRsData;
          }
          else if (_dwR232OpCnt == 3)
          {
            _dwR232Length = dwRsData;
          }
          else
          {
            dwValue = dwRsData;
            // transmit back what receive to verify
            vRs232Write(dwValue);
            
#if USE_MEMORY_MAPING
            dRs232WriteDram(_dwR232Addr, dwValue);
#else
            
            pdwAddr = (uint32_t *)(_dwR232Addr);
            *pdwAddr = dwValue;
#endif

            _dwR232Addr = _dwR232Addr + 4;
            _dwR232Length--;
            if (_dwR232Length == 0)
            {
              _dwR232OpCnt = 0;
            }
          }
          break;
        case R232_RISC_WRITEDRAM:     //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
            _dwR232Addr = dwRsData;
          }
          else if (_dwR232OpCnt == 3)
          {
            dwValue = dwRsData;
#if USE_MEMORY_MAPING
            dRs232WriteDram(4*_dwR232Addr, dwValue);
#else
            
            pdwAddr = (uint32_t *) (4*_dwR232Addr);
            *pdwAddr = dwValue;
#endif
            _dwR232OpCnt = 0;
          }
          break;
        case R232_RISC_READDRAM:      //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
            _dwR232Addr = dwRsData;  // read dram address
            pdwAddr = (uint32_t *) (4*_dwR232Addr);
#if USE_MEMORY_MAPING
            dwValue = dRs232ReadDram(4*_dwR232Addr);
#else
            dwValue = *pdwAddr;
#endif
            vRs232Write (0xC0000400); // send header of 1 word
            vRs232Write (dwValue);
            _dwR232OpCnt = 0;
          }
          break;
        case R232_READDRAM:      //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
            _dwR232Addr = dwRsData; // read dram address
          }
          else if (_dwR232OpCnt == 3)
          {
            _dwR232Length = dwRsData; // read dram length
            vRs232Write (0xc0000000 + ((_dwR232Length & 0x3fff) << 10));
//            OssGiveSem(&_rSemSuccessiveRead);
          }
          break;
        case RS232_WRITE_CACHE:     //---------------------------------------
          if (_dwR232OpCnt == 2)
          {
             // which cache           
             _dwR232Addr =(uint32_t)(dwRsData); 
          }
          else if (_dwR232OpCnt == 3)
          {
            // length
            _dwR232Length = dwRsData;
          }
          else
          {
            dwValue = dwRsData;
            // transmit back what receive to verify
            vRs232Write (dwValue);
            
#if USE_MEMORY_MAPING
            dRs232WriteDram(_dwR232Addr, dwValue);
#else
            
            pdwAddr = (uint32_t *) (_dwR232Addr);
            *pdwAddr = dwValue;
#endif

            _dwR232Addr = _dwR232Addr + 4;
            _dwR232Length--;
            if (_dwR232Length == 0)
            {
              _dwR232OpCnt = 0;
            }
          }
          break;          

        default:                 // error op code receive, ignore and reset to original state
          _dwR232OpCnt = 0;     // show error flag
        }    // switch
    }   //_dealing with command end

    //Reset
    if (dwRsData == 0x0000ffff && _dwRS232ResetStatus == 0)
    {
      _dwRS232ResetStatus = 1;
    }
    else if (dwRsData == 0x12345678 && _dwRS232ResetStatus == 1)
    {
      _dwRS232ResetStatus = 2;
    }
    else if (dwRsData == 0x34345678 && _dwRS232ResetStatus == 2)
    {
      _dwRS232ResetStatus = 3;
    }
    else if (dwRsData == 0x78781234 && _dwRS232ResetStatus == 3)
    {
      _dwRS232ResetStatus = 4;
    }
    else if (dwRsData == 0x87654321 && _dwRS232ResetStatus == 4)
    {
      _dwRS232ResetStatus = 0;
      vRs232Write (0x11110000); // send ack
      _dwR232OpCnt = 0;
    }
    else
    {
      _dwRS232ResetStatus = 0;
    }
    //_fgRs232Int= FALSE;

// Clear UART interrupt
  //BIM_ClearIrq(irq);

  mt85xx_mask_ack_irq(VECTOR_RS232_1);

}
#endif   //endif UART_DSP_DEBUG

/*
 * HW functions
 */

#if 0
static void _mt85xx_u0_set_trans_mode_on(void)
{
        UART_WRITE32(0xE2, UART0_STATUS);
}
#endif


/*********************************************************************
// Function : void _mt33xx_uart_hwinit(void)
// Description : set uart pinmux
// Parameter : None
//           
// Return    : None
 **********************************************************************/
static void _mt33xx_uart_hwinit(void)
{
   int nport;
   int reg_base_addr;
   int itemp;
   uint32_t u4Value;
  
    // UART0 is set to transparent by boot loader.
    // _mt85xx_u0_set_trans_mode_on();

    /* set buad rate to 115200 ? or leave them as the boot settings */
    /*
    UART_WRITE32(0, UART0_COMMCTRL);
    */

    /*
     * rx data timeout = 15 * 1/27M sec
     * trigger level 26
     * flush rx, tx buffer
     */
    /* UART 0 doesn't have buffer to initiate
    UART_WRITE32(BUFCTRL_INIT, UART0_BUFCTRL);
    */

    /* disable interrupt */
    UART_WRITE32(0, UART0_INT_EN);

    // port1 ~port 6
    for(nport = 1 ; nport < UART_NR ; nport ++ )
    {
		reg_base_addr = _mt33xx_get_uartx_base_addr(nport) ;
#if MT33XX_UART_DEBUG	   
		printk(KERN_INFO "[KERNEL UART]hardware init ,nport = %d ,uart base = %x\r\n",nport,reg_base_addr);	  
#endif
      /* disable interrupt */
		UART_WRITE32(0, reg_base_addr + UART_INT_EN);
	   /* set baud rate  to default value */
		UART_WRITE32(0, reg_base_addr + UART_COMMCTRL);
    }
    /* for backcar */
    	u4Value = IO_READ32(0xFD00040C,0);
	IO_WRITE32(0xFD00040C,0,(u4Value|0x780));
	/*init UART1 ~UART 6 Pinmux */ 
	itemp = UART_PINMUX_REG() & (~(UART1_PINMUX_EN | UART2_PINMUX_EN |UART3_PINMUX_EN|UART4_PINMUX_EN |UART5_PINMUX_EN));
	/*Default enbale uart1/2/3/4/5 ,   uart6 isn't enabled*/
	itemp = UART1_PINMUX_EN | UART2_PINMUX_EN /*|UART3_PINMUX_EN*/|UART4_PINMUX_EN |UART5_PINMUX_EN;
	UART_PINMUX_WRITE(itemp);  
	 
#if MT33XX_UART_DEBUG	
	printk(KERN_INFO "[KERNEL UART]hardware init pinmux= %x\r\n",UART_PINMUX_REG());
 #endif  

}



/*********************************************************************
// Function : void _mt33xx_uart0_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
// Description : set uart0 property such as baud rate ,date length ,stop bit ,parity
// Parameter : uCOMMCTRL :common port control register
//           
// Return    : None
 **********************************************************************/
static void _mt33xx_uart0_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
{

	int cur_reg_val ;
	cur_reg_val = UART_READ32(uCOMMCTRL);
#if MT33XX_UART_DEBUG
   	printk(KERN_INFO "[KERNEL UART]uart0 Set curreg= %x ,uCOMMCTRL=%d,baud=%d\r\n",cur_reg_val,uCOMMCTRL,baud);
#endif
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

    //We currently only process the baud rate configuration now.
    //Otherwise, stop/datalen/bitmask/parity should also be set. comment by cmtu

}
static uint32_t  _u4UART_SourceClk = 32400000;


static void _mt33xx_uartx_set(unsigned int uCOMMCTRL, int baud, int datalen, int stop, int parity)
{

   int temp;
   uint32_t u4Rate_Div = 0;
#if MT33XX_UART_DEBUG  
	printk(KERN_INFO "[KERNEL UART]uartx Set uCOMMCTRL =%x,baud=%d,\r\n",uCOMMCTRL,baud);
#endif 
   u4Rate_Div = (_u4UART_SourceClk*10)/baud;
   u4Rate_Div +=5;
   u4Rate_Div /=10;
   u4Rate_Div -=1;
   
   UART_REG_BITCLR(U_BAUD_RATE_MASK,uCOMMCTRL);
   UART_REG_BITSET(SETBAUD(0XD),uCOMMCTRL);
   UART_REG_BITCLR(U_RATE_DIVISOR_MASK,uCOMMCTRL);

   UART_REG_BITSET(U_SET_RATE_DIVISOR(u4Rate_Div),uCOMMCTRL);
   UART_REG_BITCLR(0x000000FF,uCOMMCTRL);
  
   temp = UART_READ32(uCOMMCTRL);
     

     
#if MT33XX_UART_DEBUG
  	printk(KERN_INFO "[KERNEL UART]uartx Set regCOMMCTRL =%x\r\n",temp);
#endif 

    //We currently only process the baud rate configuration now.
    //Otherwise, stop/datalen/bitmask/parity should also be set. comment by cmtu

}



static void _mt33xx_uart_get(unsigned int uCOMMCTRL, int *p_baud, int *p_datalen, int *p_stop, int *p_parity)
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

unsigned int _mt33xx_u0_trans_mode_on(void)
{
    return UART_READ32(UART0_STATUS) & U0_TRANSPARENT;
}

/*
 * uart member functions
 */

static unsigned int _mt33xx_u0_read_allow(int nport )
{
    return UART_READ32(UART0_STATUS) & U0_RD_ALLOW;
}

static unsigned int _mt33xx_u0_write_allow(int nport)
{
    return UART_READ32(UART0_STATUS) & U0_WR_ALLOW;
}

static void _mt33xx_u0_int_enable(int nport,int enable)
{
    if (enable)
    {
        UART_REG_BITSET(U0_INTALL, UART0_INT_EN);
    }
    else
    {
        UART_REG_BITCLR(U0_INTALL, UART0_INT_EN);
    }
}

static void _mt33xx_u0_empty_int_enable(int nport,int enable)
{
    if (enable)
    {
        UART_REG_BITSET(U0_TBUF, UART0_INT_EN);
    }
    else
    {
        UART_REG_BITCLR(U0_TBUF, UART0_INT_EN);
    }
}

static unsigned int _mt33xx_u0_read_byte(int nport)
{
    return UART_READ32(UART0_DATA_BYTE);
}

static void _mt33xx_u0_write_byte(int nport,unsigned int byte)
{

#if 0 // comment by mtk40505
    if((disable_print & 0x01) ==1)
    {
        return;
    }
#endif 

    UART_WRITE32(byte, UART0_DATA_BYTE);
}

#ifdef CONFIG_CONSOLE_POLL

static unsigned int _mt33xx_u0_read_char(int nport,struct uart_port *port)
{   
   while(!_mt33xx_u0_read_allow(UART_PORT0)) ;
    return _mt33xx_u0_read_byte(UART_PORT0);
}

static void _mt33xx_u0_write_char(int nport,struct uart_port *port, unsigned int byte)
{
  while(!_mt33xx_u0_write_allow(UART_PORT0)) ;
   _mt33xx_u0_write_byte(UART_PORT0,byte);
}
#endif

static void _mt33xx_u0_flush(int nport)
{
    //u0 doesn't have buffer to be flushed
    return;
}

static void _mt33xx_u0_get_top_err(int nport,int *p_parity, int *p_end, int *p_break)
{
    *p_parity = (UART_READ32(UART0_STATUS) & U0_PARITY_ERR) ? 1 : 0;
    *p_end    = (UART_READ32(UART0_STATUS) & U0_END_ERR)    ? 1 : 0;
    *p_break  = 0;//u0 doesn't have break to get
}
//used for async log mode


uint32_t _mt33xx_u0_ReadSwFifo (uint8_t u1Port, uint8_t *szBuf, uint32_t u4Len)
{
    uint32_t u4NumberToRead;
 
    if (u4Len ==0)
    {
        return 0;
    }

    u4NumberToRead = (u4Len > RXFIFO_DATASIZE(u1Port)) ? RXFIFO_DATASIZE(u1Port) : u4Len;
    _arTxSWFIFO_U0.u4TotalRWCnt += u4NumberToRead;

    if ((SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Rp) > u4NumberToRead)
    {
        memcpy (szBuf, &RXFIFO_PR_DATA(u1Port), u4NumberToRead);
    }
    else
    {
        memcpy (szBuf, &RXFIFO_PR_DATA(u1Port), (SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Rp));

        memcpy (szBuf + (SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Rp),
                          &(_arTxSWFIFO_U0.aData[0]), u4NumberToRead - (SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Rp));
    }

    // update read pointer
    _arTxSWFIFO_U0.u4Rp = (_arTxSWFIFO_U0.u4Rp + u4NumberToRead) % SW_FIFO_SIZE;


    return u4NumberToRead;
}

static uint32_t _mt33xx_u0_WriteSwFifo (uint8_t u1Port, const uint8_t *szBuf, uint32_t u4Len)
{
	uint32_t  u4NumberToWrite;

    if (u4Len == 0)
    {
        return 0;
    }

    u4NumberToWrite = (u4Len > TXFIFO_FREESIZE(u1Port)) ? TXFIFO_FREESIZE(u1Port) : u4Len;

    // Verify write size and fifo free size
    if (u4Len > TXFIFO_FREESIZE(u1Port))
    {
        _arTxSWFIFO_U0.u4DiscardCnt += (u4Len - TXFIFO_FREESIZE(u1Port));
    }

    _arTxSWFIFO_U0.u4TotalRWCnt += u4NumberToWrite;


    // Write to fifo
    if ((u4NumberToWrite>0) && (u4NumberToWrite<SW_FIFO_SIZE))
    {
        if ((SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Wp) > u4NumberToWrite)
        {
            memcpy(&(_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Wp]), szBuf, u4NumberToWrite);
        }
        else
        {
            uint32_t u4Length;
            u4Length = SW_FIFO_SIZE - _arTxSWFIFO_U0.u4Wp;
            memcpy(&(_arTxSWFIFO_U0.aData[_arTxSWFIFO_U0.u4Wp]), szBuf, u4Length);
           
            memcpy(&(_arTxSWFIFO_U0.aData[0]), szBuf + u4Length,  u4NumberToWrite - u4Length);
        }
        _arTxSWFIFO_U0.u4Wp = (_arTxSWFIFO_U0.u4Wp + u4NumberToWrite) % SW_FIFO_SIZE;
    }

    return u4NumberToWrite;
}



/*********************************************************
*  from uart 1 ~ uart 6
*
*
***********************************************************/

unsigned int _mt33xx_get_uartx_base_addr(int nport)
{
    if (nport < UART_NR)
		return UartxBaseAddr[nport][1];
    else
    	{
		  printk(KERN_ERR "[KERNEL UART]get_uart_base_addr Invalide !! nport =%x\r\n",nport);
		  return 0xFFFFFFFF;
    	}
		
}


unsigned int _mt33xx_uartx_read_allow(int nport)
{
    int ret_val;
    int data_size ;
	int reg_base_addr;
	
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport) ;

    data_size = UART_READ32(reg_base_addr  + UART_STATUS) & RX_BUF_MASK;

	if (nport == 1)
		data_size = data_size & UART1_RX_BUF_MASK;
	else
		data_size = data_size & RX_BUF_MASK;

    if(data_size > 0 )
	    ret_val = 1;
    else
        ret_val = 0; 
    
    return ret_val;
}

unsigned int _mt33xx_uartx_write_allow(int nport)
{
    int ret_val;
    int data_size ;
	int reg_base_addr;

	reg_base_addr = _mt33xx_get_uartx_base_addr(nport);
    data_size = UART_READ32(reg_base_addr + UART_STATUS);
	
    if (nport == 1)
		data_size = data_size & UART1_TX_BUF_MASK;
	else
		data_size = data_size & TX_BUF_MASK;
	
    
    if (data_size > 0 )
		ret_val = 1;
    else
	{
		ret_val = 0;
	} 

    return ret_val;
}

void _mt33xx_uartx_int_enable(int nport,int enable)
{
	int reg_base_addr;
	
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport) ;

    if (enable)
    {
        UART_REG_BITSET( UART_INTALL, reg_base_addr +UART_INT_EN);
    }
    else
    {
        UART_REG_BITCLR( UART_INTALL, reg_base_addr +UART_INT_EN);
    }
}

void _mt33xx_uartx_empty_int_enable(int nport,int enable)
{
    int reg_base_addr;
	
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport) ;
	
    if (enable)
    {
        UART_REG_BITSET(TBUF_E1, reg_base_addr + UART_INT_EN);
    }
    else
    {
        UART_REG_BITCLR(TBUF_E1, reg_base_addr + UART_INT_EN);
    }
}

unsigned int _mt33xx_uartx_read_byte(int nport)
{
    unsigned int r_uart_data;
	int reg_base_addr;
	
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport);

    r_uart_data = UART_READ32(reg_base_addr + UART_DATA_BYTE);

#if MT33XX_UART_DEBUG
	if((nport == 1) || 
		(nport == 2)
		)
		printk(KERN_INFO "uart read, nport =  %d ,baseaddr : %x,data:%x \n\r",nport, reg_base_addr,r_uart_data);
#endif
    return r_uart_data;
}

void _mt33xx_uartx_write_byte(int nport,unsigned int byte)
{
	int reg_base_addr;
	
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport);
	
    UART_WRITE32(byte, reg_base_addr + UART_DATA_BYTE );
#if MT33XX_UART_DEBUG
	if( (nport == 1) || 
		(nport == 2)
		)
		printk(KERN_INFO "uart write, nport =  %d ,baseaddr : %x,data:%x \n\r",nport, reg_base_addr,byte);
#endif		
}

/* not used


unsigned int _mt33xx_uart_read_char(int nport)
{   
   while(!_mt33xx_uart_read_allow(nport)) ;
    return _mt33xx_uart_read_byte(nport);
}

void _mt33xx_uart_write_char(int nport, unsigned int byte)
{
  
  while(!_mt33xx_uart_write_allow(nport)) ;
   _mt33xx_uart_write_byte(nport,byte);
}
*/


void _mt33xx_uartx_flush(int nport)
{
    /*UART1_FLUSH();*/
    int reg_base_addr;
	reg_base_addr = _mt33xx_get_uartx_base_addr(nport);
	
    UART_REG_BITSET((CLEAR_TBUF | CLEAR_RBUF), reg_base_addr + UART_BUFCTRL);
    return;
}

void _mt33xx_uartx_get_top_err(int nport,int *p_parity, int *p_end, int *p_break)
{
    int reg_addr;
	
	reg_addr = _mt33xx_get_uartx_base_addr(nport) + UART_STATUS ;

	if(nport == 1)
    {
        *p_parity = (UART_READ32(reg_addr) & UART1_PARITY_ERR) ? 1 : 0;
        *p_end    = (UART_READ32(reg_addr) & UART1_END_ERR)    ? 1 : 0;
        *p_break  = (UART_READ32(reg_addr) & UART1_BREAK_ERR)    ? 1 : 0;
	}
	else
	{
        *p_parity = (UART_READ32(reg_addr) & UART_PARITY_ERR) ? 1 : 0;
        *p_end    = (UART_READ32(reg_addr) & UART_END_ERR)    ? 1 : 0;
        *p_break  = (UART_READ32(reg_addr) & UART_BREAK_ERR)    ? 1 : 0;
	}
}
/*end uart x */


// Notes: this function cann't be used in ISR
static uint32_t _mt33xx_uart_write(uint8_t u1Port, const uint8_t* pData, uint32_t u4NumberToWrite)
{
    uint32_t  u4Ret = 0;

    if (u4NumberToWrite == 0)
    {
        return 0;
    }

	u4Ret = _mt33xx_u0_WriteSwFifo( u1Port, pData, u4NumberToWrite);
	
#if 0
    if (!TXFIFO_EMPTY (u1Port))
    {
        _mt85xx_u0_empty_int_enable(true);
    }
#endif
    return u4Ret;
}

/*
 * interrupt handling
 */
static void _mt33xx_uart_rx_chars(struct mt33xx_uart_port *mport)
{
    struct tty_struct *tty = mport->port.state->port.tty;
    int max_count = UART_FIFO_SIZE;
    unsigned int data_byte;
    unsigned int flag;
    int err_parity, err_end, err_break;

    while (max_count-- > 0)
    {
        /* check status */
        if (mport->fn_read_allow(mport->nport))
        {
#if 0      //test uart0 soft irq only
           if(check_softirq_existed(UART1_SOFTIRQ))
           {
               raise_softirq(UART1_SOFTIRQ);
           }
#endif
            /* in mt85xx, process error before read byte */
            mport->fn_get_top_err(mport->nport,&err_parity, &err_end, &err_break);

            /* read the byte */
            data_byte = mport->fn_read_byte(mport->nport);
	
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
//				rt_sigmask(SIGUSR1);
//				sys_kill(0, SIGUSR1);
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
        else
        {
            break;
        }

        if (uart_handle_sysrq_char(&mport->port, data_byte))
        {
            continue;
        }

        //2.6.18. no overrun support, set status and mask to 0
        uart_insert_char(&mport->port, 0, 0, data_byte, flag);

        //2.6.10
        //tty_insert_flip_char(tty, data_byte, flag);
        /*
        if (OVERRUN_HAPPENED)
        {
            tty_insert_flip_char(tty, 0, TTY_OVERRUN);
        }
        */
    }
    tty_flip_buffer_push(tty);
}

//static void _mt33xx_uart_stop_tx(struct uart_port *port);
static void _mt33xx_uart_tx_chars(struct mt33xx_uart_port *mport)
{
    struct uart_port *port = &mport->port;
    struct circ_buf *xmit = &port->state->xmit;
    int count;


    /* deal with x_char first */
    if (port->x_char)
    {
        if(mport->nport == 0)
	    {
			if (UART_READ32(UART0_STATUS) & U0_TRANSPARENT)
			{
				mport->fn_write_byte(mport->nport,port->x_char);
			}
	    }
        else
        {
			mport->fn_write_byte(mport->nport,port->x_char);
        }
        
        port->icount.tx++;
        port->x_char = 0;
        return;
    }
	
#if MT33XX_UART_DEBUG
	if(mport->nport != 0)
		printk(KERN_INFO "[UART]uart_tx_chars\r\n");
#endif

    /* stop tx if circular buffer is empty or this port is stopped */
    if (uart_circ_empty(xmit) || uart_tx_stopped(port))
    {
        _mt33xx_uart_stop_tx(port);
        return;
    }
    if(mport->nport==1)
    	count = (UART_READ32(UART_STATUS)&0x1FFC000)>>14;//port->fifosize - 1;
	else
		count = (UART_READ32(UART_STATUS)&0x1FF000)>>12;//port->fifosize - 1;

    do
    {
        if(mport->nport == 0)
	   {
			if (UART_READ32(UART0_STATUS) & U0_TRANSPARENT)
			{
			    mport->fn_write_byte(mport->nport,xmit->buf[xmit->tail]);
			}
	   }
        else
	   {
               mport->fn_write_byte(mport->nport,xmit->buf[xmit->tail]);
	   }
        
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
        if (uart_circ_empty(xmit))
        {
            break;
        }
        if (!mport->fn_write_allow(mport->nport))
        {
            break;
        }
    } while (--count > 0);



    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
    {
        uart_write_wakeup(port);
    }
    if (uart_circ_empty(xmit))
    {
        _mt33xx_uart_stop_tx(port);
    }
}

static irqreturn_t _m33xx_uart_interrupt(int irq, void *dev_id)
{
    unsigned int uart_int_ident;
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)dev_id;
    int reg_base_addr;
   
    /* take care of SA_SHIRQ and return IRQ_NONE if possible [LDD/279] */
    switch (mport->nport)
    {
    case UART_PORT0:
        uart_int_ident = UART_READ32(UART0_INT_STATUS);
        if (!(uart_int_ident & (unsigned int)(U0_INTALL | U0_TBUF)))
        {
            return IRQ_HANDLED;
        }
#if UART_DSP_DEBUG
        if (!((UART_READ32(UART0_STATUS) & U0_TRANSPARENT)))
        {
            //Non transparent mode: fo dsp debug
            //if (mport->fn_read_allow())
            //if (!mport->fn_write_allow())
            {
                _Uart2Isr((uint32_t)irq);
                return IRQ_HANDLED;
            }
        }
#endif
        break;

    case UART_PORT1:
	case UART_PORT2:	
	case UART_PORT3:		
	case UART_PORT4:	
	case UART_PORT5:		
	//case UART_PORT6:	
	    reg_base_addr = _mt33xx_get_uartx_base_addr(mport->nport) ;
        uart_int_ident = UART_READ32(reg_base_addr + UART_INT_STATUS);
        if (!(uart_int_ident & (unsigned int)(UART_INTALL)))
        {
            //Always return handled now
            return IRQ_HANDLED;
        }
        break;
    default:
        /* no others can do RS232 IRQ, tell kernel I handled that well */
        return IRQ_HANDLED;
    }
    /* rx mode */
    if (mport->fn_read_allow(mport->nport))
    {
#if 0   //test uart0 soft irq only
        if(check_softirq_existed(UART1_SOFTIRQ))
        {
            raise_softirq(UART1_SOFTIRQ);
        }
#endif        
        _mt33xx_uart_rx_chars(mport);
             
    }

    /* tx mode :clear interrupt status*/
	if (mport->nport != 0)
	{
	   reg_base_addr = _mt33xx_get_uartx_base_addr(mport->nport) ;
	   UART_WRITE32(0xff,reg_base_addr + UART_INT_STATUS);
	} 
 
    if (mport->fn_write_allow(mport->nport))
    {
        if (g_UartAsyncMode) 
        {
            uint32_t u4DataToWrite;
            uint32_t u4Idx;
                             
            u4DataToWrite = (TXFIFO_DATASIZE(UART_PORT0)>=1) ? 1 : TXFIFO_DATASIZE(UART_PORT0);
                      
            for (u4Idx=0; u4Idx < u4DataToWrite ; u4Idx++)
            {
                mport->fn_write_byte(mport->nport,TXFIFO_PR_DATA(UART_PORT0));

                if( ++_arTxSWFIFO_U0.u4Rp >= SW_FIFO_SIZE)
                {
                    _arTxSWFIFO_U0.u4Rp = 0;
                }
                //port->icount.tx++;
            }
            
            /* Clear Local UART interrupt */
            UART_WRITE32(0xff,UART0_INT_STATUS);
            
            if (TXFIFO_EMPTY (UART_PORT0))
            {
              //_mt85xx_u0_empty_int_enable(false);
              //_mt85xx_uart_stop_tx(port);
            }
            _arTxSWFIFO_U0.u4IntCnt ++;
        }
        else
        {
#if MT33XX_UART_DEBUG        
		if (mport->nport != 0)
			printk(KERN_INFO "[UART]int  tx chars\r\n");
#endif        
		_mt33xx_uart_tx_chars(mport);
        }

    }

    /* modem mode */
    //_mt85xx_uart_modem_state(mport);

    return IRQ_HANDLED;
}

/*
 * uart ops
 */

static unsigned int _mt33xx_uart_tx_empty(struct uart_port *port)
{
    /*
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;

    return (mport->fn_write_allow() == (TX_BUF_SIZE - 1 )) ? TIOCSER_TEMT : 0;
    */
    return TIOCSER_TEMT; //Set to not supporting now

    /*	This function tests whether the transmitter fifo and shifter
	for the port described by 'port' is empty.  If it is empty,
	this function should return TIOCSER_TEMT, otherwise return 0.
	If the port does not support this operation, then it should
	return TIOCSER_TEMT. */
}

static void _mt33xx_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
/*
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;
    unsigned int regval;
    unsigned int uMS;

    uMS = (mport->nport == 1) ? UART1_MODEM : UART0_MODEM;

    regval = UART_READ32(uMS);

    if (mctrl & TIOCM_RTS)
    {
        regval |= MDM_RTS;
    }
    else
    {
        regval &= ~MDM_RTS;
    }

    if (mctrl & TIOCM_DTR)
    {
        regval |= MDM_DTR;
    }
    else
    {
        regval &= ~MDM_DTR;
    }

    UART_WRITE32(regval, uMS);
*/
/*
	this function sets the modem control lines for port described
	by 'port' to the state described by mctrl.  The relevant bits
	of mctrl are:
		- TIOCM_RTS	RTS signal.
		- TIOCM_DTR	DTR signal.
		- TIOCM_OUT1	OUT1 signal.
		- TIOCM_OUT2	OUT2 signal.
		- TIOCM_LOOP	Set the port into loopback mode.
	If the appropriate bit is set, the signal should be driven
	active.  If the bit is clear, the signal should be driven
	inactive.

	Locking: port->lock taken.
	Interrupts: locally disabled.
	This call must not sleep
*/
    return;
}

static unsigned int _mt33xx_uart_get_mctrl(struct uart_port *port)
{
/*
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;
    unsigned int modem_status;
    unsigned int result = 0;
    unsigned int uMS;

    uMS = (mport->nport == 1) ? UART1_MODEM : UART0_MODEM;

    modem_status = UART_READ32(uMS);
    if (modem_status & MDM_DCD)
    {
        result |= TIOCM_CAR;
    }
    if (modem_status & MDM_DSR)
    {
        result |= TIOCM_DSR;
    }
    if (modem_status & MDM_CTS)
    {
        result |= TIOCM_CTS;
    }

    return result;
*/
    unsigned int result = 0;

    result = TIOCM_CTS | TIOCM_CAR | TIOCM_DSR;
    return result;
    /*
	Returns the current state of modem control inputs.  The state
	of the outputs should not be returned, since the core keeps
	track of their state.  The state information should include:
		- TIOCM_DCD	state of DCD signal
		- TIOCM_CTS	state of CTS signal
		- TIOCM_DSR	state of DSR signal
		- TIOCM_RI	state of RI signal
	The bit is set if the signal is currently driven active.  If
	the port does not support CTS, DCD or DSR, the driver should
	indicate that the signal is permanently active.  If RI is
	not available, the signal should not be indicated as active.

	Locking: port->lock taken.
	Interrupts: locally disabled.
	This call must not sleep    */
}

static void _mt33xx_uart_stop_tx(struct uart_port *port)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    //mport->fn_empty_int_enable(0);
    mport->tx_stop = 1;
/*	Stop transmitting characters.  This might be due to the CTS
	line becoming inactive or the tty layer indicating we want
	to stop transmission due to an XOFF character.

	The driver should stop transmitting characters as soon as
	possible.

	Locking: port->lock taken.
	Interrupts: locally disabled.
	This call must not sleep */
}

static void _mt33xx_uart_start_tx(struct uart_port *port)
{
    //struct circ_buf *xmit = &port->info->xmit;
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    int count;
    unsigned long flags;


    //mport->fn_empty_int_enable(1);
    if (g_UartAsyncMode) //USE ASYNC MODE  for uart 0 only
    {
        struct uart_port *port = &mport->port;
        struct circ_buf *xmit = &port->state->xmit;

        local_irq_save(flags);
        
        /* deal with x_char first */
        if (port->x_char)
        {
            _mt33xx_uartx_write_byte(UART_PORT0, port->x_char);
            port->icount.tx++;
            port->x_char = 0;
            return;
        }
        
        count = port->fifosize - 1;
        do
        {
            _mt33xx_uartx_write_byte(UART_PORT0, xmit->buf[xmit->tail]);
            xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
            port->icount.tx++;
            if (uart_circ_empty(xmit))
            {
                break;
            }
        } while (--count > 0);
        /*
		_mt85xx_uart_write(UART_PORT0, (const uint8_t *)(&(xmit->buf[xmit->tail])), port->fifosize);
       	 xmit->tail = (xmit->tail + port->fifosize) % (UART_XMIT_SIZE);
       	port->icount.tx+= port->fifosize-1;
		*/

        if (!TXFIFO_EMPTY (UART_PORT0))
        {   
            /*_mt85xx_u0_empty_int_enable(true);*/
            uart_write_wakeup(port);
        }
        if (TXFIFO_EMPTY (UART_PORT0))
        {
            /*_mt85xx_uart_stop_tx(port);*/
        }
        local_irq_restore(flags);   
    }
    else
    {
 #if MT33XX_UART_DEBUG   
     if (mport->nport != 0)
		printk(KERN_INFO "[UART]int  tx chars\r\n");
#endif    
    local_irq_save(flags);
	   // if (mport->fn_write_allow(mport->nport))
    {
        _mt33xx_uart_tx_chars(mport);
    }
    fgUartRegisterd = true;
    mport->tx_stop = 0;
    local_irq_restore(flags);
    }

    /*
	Start transmitting characters.

	Locking: port->lock taken.
	Interrupts: locally disabled.
	This call must not sleep    */
}

#if 0  //no used
static void _mt85xx_uart_start_tx_for_printk(struct uart_port *port)
{
    //struct circ_buf *xmit = &port->info->xmit;
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;
    int count;
    unsigned long flags;

  

    if (mport->fn_write_allow())
    {
        _mt85xx_uart_tx_chars(mport);
    }
    fgUartRegisterd = true;
    mport->tx_stop = 0;
 }
#endif

static void _mt33xx_uart_stop_rx(struct uart_port *port)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    mport->rx_stop = 1;
}

/*
static void _mt85xx_uart_start_rx(struct uart_port *port)
{
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;
    mport->rx_stop = 0;
}
*/

static void _mt33xx_uart_enable_ms(struct uart_port *port)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    mport->ms_enable = 1;
}

static void _mt33xx_uart_break_ctl(struct uart_port *port, int break_state)
{
/*
    struct mt85xx_uart_port *mport = (struct mt85xx_uart_port *)port;
    unsigned int regval; 
    unsigned long flags;
    unsigned int uCOMMCTRL;

    uCOMMCTRL = (mport->nport == 0) ? UART0_COMMCTRL : UART1_COMMCTRL;

    spin_lock_irqsave(&port->lock, flags);
    regval = UART_READ32(uCOMMCTRL);
    if (break_state == -1)
    {
        regval |= CONTROL_BREAK;
    }
    else
    {
        regval &= ~CONTROL_BREAK;
    }
    UART_WRITE32(regval, uCOMMCTRL);
    spin_unlock_irqrestore(&port->lock, flags);
*/

    return;

    /*
	Control the transmission of a break signal.  If ctl is
	nonzero, the break signal should be transmitted.  The signal
	should be terminated when another call is made with a zero
	ctl.

	Locking: none.
	Interrupts: caller dependent.
	This call must not sleep    */
}



static int _mt33xx_uart_startup(struct uart_port *port)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    unsigned long flags;
    int retval;
    int reg_base_addr;
    /*
      * FIXME: _mt85xx_uart_hwinit ?
      */

   // printk(KERN_INFO "[Kernel uart] mt33xx uart startup _irq_allocated =%d ,mport->nport = %d ,port->irq = %d\r\n",_irq_allocated[mport->nport],mport->nport,port->irq);

    _irq_allocated[mport->nport] ++;
    
    if (_irq_allocated[mport->nport] == 1)  
    {
        /* allocate irq, two ports share the same interrupt number */
        retval = request_irq(port->irq, _m33xx_uart_interrupt, IRQF_SHARED,
            "MT33xx Serial", port);
        if (retval)
        {
            _irq_allocated[mport->nport] --;
            return retval;
        }
    }
	reg_base_addr = _mt33xx_get_uartx_base_addr(mport->nport) ;
    if(mport->nport == 1)
       UART_WRITE32(0xFFFC100, reg_base_addr + UART_BUFCTRL );
	else
	   UART_WRITE32(0xFFFC100, reg_base_addr + UART_BUFCTRL );
    /* enable interrupt */
    mport->fn_empty_int_enable(mport->nport,1);
    mport->fn_int_enable(mport->nport,1);

    spin_lock_irqsave(&port->lock, flags);

    if (mport->nport == 0)
    {
	    /*enable RS232 interrupt*/
		if (!(__raw_readl(BIM_VIRT + REG_IRQEN) & MASK_IREQEN_RS232))
         {
             __raw_writel(__raw_readl(BIM_VIRT + REG_IRQEN) | MASK_IREQEN_RS232, BIM_VIRT + REG_IRQEN);
         }
    } 
    spin_unlock_irqrestore(&port->lock, flags);

    return 0;

}

static void _mt33xx_uart_shutdown(struct uart_port *port)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;

    /* disable interrupt and disable port */
    mport->fn_int_enable(mport->nport,0);

    /*
      * FIXME: disable BIM IRQ enable bit if all ports are shutdown
      */

    _irq_allocated[mport->nport] --;

    if (!_irq_allocated[mport->nport])
    {
        free_irq(port->irq, port);
    }

    /* reset uart port */
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
static void _mt33xx_uart_set_termios(struct uart_port *port,
    struct ktermios *termios, struct ktermios *old)
{
    struct mt33xx_uart_port *mport = (struct mt33xx_uart_port *)port;
    unsigned long flags;
    int baud;
    int datalen;
    int parity = 0;
    int stopbit = 1;
    unsigned int uCOMMCTRL;

    if(mport->nport == 0)
        uCOMMCTRL = UART0_COMMCTRL;
    else   
	    uCOMMCTRL = _mt33xx_get_uartx_base_addr(mport->nport) + UART_COMMCTRL ;

    /* calculate baud rate */
    baud = (int)uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
#if MT33XX_UART_DEBUG   
    printk(KERN_INFO "uart set_termios,mport->nport = %d,baudrate = %d datalength =%x \r \n",
				mport->nport,baud,termios->c_cflag);
#endif
    

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
    if (termios->c_cflag & CSTOPB)
    {
        stopbit = 2;
    }
    /* parity : default none */
    if (termios->c_cflag & PARENB)
    {
        if (termios->c_cflag & PARODD)
        {
            parity = 1; /* odd */
        }
        else
        {
            parity = 2; /* even */
        }
    }
    /* lock from here */
    spin_lock_irqsave(&port->lock, flags);
        /* update per port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);
	/* read status mask */
	if (termios->c_iflag & INPCK)
	{
	    /* frame error, parity error */
	    port->read_status_mask |= UST_FRAME_ERROR | UST_PARITY_ERROR;
	}
	if (termios->c_iflag & (BRKINT | PARMRK))
	{
	    /* break error */
	    port->read_status_mask |= UST_BREAK_ERROR;
	}
	/* status to ignore */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
	{
	    port->ignore_status_mask |= UST_FRAME_ERROR | UST_PARITY_ERROR;
	}
	if (termios->c_iflag & IGNBRK)
	{
	    port->ignore_status_mask |= UST_BREAK_ERROR;
	    if (termios->c_iflag & IGNPAR)
	    {
	        port->ignore_status_mask |= UST_OVRUN_ERROR;
	    }
	}
	if ((termios->c_cflag & CREAD) == 0)
	{
	    // dummy read
	    port->ignore_status_mask |= UST_DUMMY_READ;
	}

	if (mport->nport == 0)
	      _mt33xx_uart0_set(uCOMMCTRL, baud, datalen, stopbit, parity);
	else
	      _mt33xx_uartx_set(uCOMMCTRL, baud, datalen, stopbit, parity);
      
	/* hw rts/cts ? */

	/*
	        if (termios->c_cflag & CRTSCTS)
	        {
	            UART_REG_BITSET(MDM_HW_RTS | MDM_HW_CTS, uMS);
	        }
	*/
	/* disable modem status interrupt */
	mport->ms_enable = 0;
	if (UART_ENABLE_MS(port, termios->c_cflag))
	{
	    // enable modem status interrupt
	    mport->ms_enable = 1;
	}
    /* unlock here */
    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *_mt33xx_uart_type(struct uart_port *port)
{
    return "MT33XX Serial";
}

static void _mt33xx_uart_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MT85XX_UART_SIZE);
}

static int _mt33xx_uart_request_port(struct uart_port *port)
{
    void *pv_region;

    pv_region =
        request_mem_region(port->mapbase, MT85XX_UART_SIZE,
            "MT33XX Uart IO Mem");
	
    return pv_region != NULL ? 0 : -EBUSY;
}

static void _mt33xx_uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
    {
        port->type = PORT_MT33XX;
        _mt33xx_uart_request_port(port);
    }
}

static int _mt33xx_uart_verify_port(struct uart_port *port,
    struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_MT33XX)
	{
		ret = -EINVAL;
	}
	if (ser->irq != port->irq)
	{
		ret = -EINVAL;
	}
	if (ser->baud_base < 110)
	{
		ret = -EINVAL;
	}
	return ret;
}

static struct uart_ops _mt33xx_uart_ops =
{
    .tx_empty       = _mt33xx_uart_tx_empty,
    .set_mctrl      = _mt33xx_uart_set_mctrl,
    .get_mctrl      = _mt33xx_uart_get_mctrl,
    .stop_tx        = _mt33xx_uart_stop_tx,
    .start_tx       = _mt33xx_uart_start_tx,
    /* .send_xchar */
    .stop_rx        = _mt33xx_uart_stop_rx,
    .enable_ms      = _mt33xx_uart_enable_ms,
    .break_ctl      = _mt33xx_uart_break_ctl,
    .startup        = _mt33xx_uart_startup,
    .shutdown       = _mt33xx_uart_shutdown,
    .set_termios    = _mt33xx_uart_set_termios,
    /* .pm */
    /* .set_wake */
    .type           = _mt33xx_uart_type,
    .release_port   = _mt33xx_uart_release_port,
    .request_port   = _mt33xx_uart_request_port,
    .config_port    = _mt33xx_uart_config_port,
    .verify_port    = _mt33xx_uart_verify_port,
    /* .ioctl */	
#ifdef CONFIG_CONSOLE_POLL
    .poll_put_char  = _mt33xx_u0_write_char,
    .poll_get_char  = _mt33xx_u0_read_char,
#endif
};

static struct mt33xx_uart_port _mt33xx_uart_ports[] =
{
   [0] = {		
        .port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_RS232_1,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 0,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[0].port.lock),
        },
        .nport              = 0,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
        .fn_read_allow      = _mt33xx_u0_read_allow,
        .fn_write_allow     = _mt33xx_u0_write_allow,
        .fn_int_enable      = _mt33xx_u0_int_enable,
        .fn_empty_int_enable    = _mt33xx_u0_empty_int_enable,
        .fn_read_byte       = _mt33xx_u0_read_byte,
        .fn_write_byte      = _mt33xx_u0_write_byte,
        .fn_flush           = _mt33xx_u0_flush,
        .fn_get_top_err     = _mt33xx_u0_get_top_err,
    },
#if UART_NR > 1
    [1] = {
        .port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_1,
            .uartclk        = 3200000 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 1,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[1].port.lock),
        },
        .nport              = 1,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
        .fn_read_allow      = _mt33xx_uartx_read_allow,
        .fn_write_allow     = _mt33xx_uartx_write_allow,
        .fn_int_enable      = _mt33xx_uartx_int_enable,
        .fn_empty_int_enable    = _mt33xx_uartx_empty_int_enable,
        .fn_read_byte       = _mt33xx_uartx_read_byte,
        .fn_write_byte      = _mt33xx_uartx_write_byte,
        .fn_flush           = _mt33xx_uartx_flush,
        .fn_get_top_err     = _mt33xx_uartx_get_top_err,
    },
#endif    

#if UART_NR > 2
	 [2] = {
        .port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_2,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 2,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[2].port.lock),
        },
        .nport              = 2,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
        .fn_read_allow      = _mt33xx_uartx_read_allow,
        .fn_write_allow     = _mt33xx_uartx_write_allow,
        .fn_int_enable      = _mt33xx_uartx_int_enable,
        .fn_empty_int_enable    = _mt33xx_uartx_empty_int_enable,
        .fn_read_byte       = _mt33xx_uartx_read_byte,
        .fn_write_byte      = _mt33xx_uartx_write_byte,
        .fn_flush           = _mt33xx_uartx_flush,
        .fn_get_top_err     = _mt33xx_uartx_get_top_err,
    },
#endif

#if UART_NR > 3
	[3] = {
		.port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_3,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 3,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[3].port.lock),
        },
        .nport              = 3,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
		.fn_read_allow		= _mt33xx_uartx_read_allow,
		.fn_write_allow 	= _mt33xx_uartx_write_allow,
		.fn_int_enable		= _mt33xx_uartx_int_enable,
		.fn_empty_int_enable	= _mt33xx_uartx_empty_int_enable,
		.fn_read_byte		= _mt33xx_uartx_read_byte,
		.fn_write_byte		= _mt33xx_uartx_write_byte,
		.fn_flush			= _mt33xx_uartx_flush,
		.fn_get_top_err 	= _mt33xx_uartx_get_top_err,

	},
#endif
#if UART_NR > 4
	[4] = {
		.port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_4,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 4,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[4].port.lock),
        },
        .nport              = 4,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
		.fn_read_allow		= _mt33xx_uartx_read_allow,
		.fn_write_allow 	= _mt33xx_uartx_write_allow,
		.fn_int_enable		= _mt33xx_uartx_int_enable,
		.fn_empty_int_enable	= _mt33xx_uartx_empty_int_enable,
		.fn_read_byte		= _mt33xx_uartx_read_byte,
		.fn_write_byte		= _mt33xx_uartx_write_byte,
		.fn_flush			= _mt33xx_uartx_flush,
		.fn_get_top_err 	= _mt33xx_uartx_get_top_err,

	},
#endif
#if UART_NR > 5
	[5] = {
		.port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_5,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 5,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[5].port.lock),
        },
        .nport              = 5,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
		.fn_read_allow		= _mt33xx_uartx_read_allow,
		.fn_write_allow 	= _mt33xx_uartx_write_allow,
		.fn_int_enable		= _mt33xx_uartx_int_enable,
		.fn_empty_int_enable	= _mt33xx_uartx_empty_int_enable,
		.fn_read_byte		= _mt33xx_uartx_read_byte,
		.fn_write_byte		= _mt33xx_uartx_write_byte,
		.fn_flush			= _mt33xx_uartx_flush,
		.fn_get_top_err 	= _mt33xx_uartx_get_top_err,

	},
#endif

#if UART_NR > 6
	[6] = {
		.port =
        {
            .membase        = (void*)MT85XX_VA_UART,
            .mapbase        = MT85XX_PA_UART,
            .iotype         = SERIAL_IO_MEM,
            .irq            = VECTOR_UART_6,
            .uartclk        = 921600 * 16,
            .fifosize       = UART_FIFO_SIZE,
            .ops            = &_mt33xx_uart_ops,
            .flags          = ASYNC_BOOT_AUTOCONF,
            .line           = 6,
            .lock           = __SPIN_LOCK_UNLOCKED(_mt33xx_uart_ports[6].port.lock),
        },
        .nport              = 6,
        .old_status         = 0,
        .tx_stop            = 0,
        .rx_stop            = 0,
        .ms_enable          = 0,
		.fn_read_allow		= _mt33xx_uartx_read_allow,
		.fn_write_allow 	= _mt33xx_uartx_write_allow,
		.fn_int_enable		= _mt33xx_uartx_int_enable,
		.fn_empty_int_enable	= _mt33xx_uartx_empty_int_enable,
		.fn_read_byte		= _mt33xx_uartx_read_byte,
		.fn_write_byte		= _mt33xx_uartx_write_byte,
		.fn_flush			= _mt33xx_uartx_flush,
		.fn_get_top_err 	= _mt33xx_uartx_get_top_err,

	}
#endif
};

/*
 * console
 */

#ifdef CONFIG_SERIAL_MT33XX_CONSOLE

#define UART_FIX_CONSOLE_LOG_LOST_PROBLEM 1

static void _mt33xx_uart_console_write(struct console *co, const char *s,
    unsigned int count)
{
    struct mt33xx_uart_port *mport;
    struct uart_port *port;
    struct circ_buf *xmit; 
	int i;
    int c = 0, total = (int)count;
	unsigned long flags;
    char *buf =( char *)s;
   
    if (!_mt33xx_u0_trans_mode_on())
    {
        return;
    }

    if (co->index >= UART_NR)
    {
        return;
    }

    mport = &_mt33xx_uart_ports[co->index];
    port = &mport->port;
    xmit = &port->state->xmit;
    
    if (g_UartAsyncMode) //USE ASYNC MODE
    {
        _mt33xx_uart_write(UART_PORT0, s, count);

        if (!TXFIFO_EMPTY (UART_PORT0))
        {   
            //_mt85xx_u0_empty_int_enable(true);
            uart_write_wakeup(port);
        }
        if (TXFIFO_EMPTY (UART_PORT0))
        {
            //_mt85xx_uart_stop_tx(port);
        }
    }
    else
    {
        //add "printk log" to "printf buffer" 
        //struct circ_buf *xmit = &port->info->xmit;
        //int c=0;
        if ((fgUartRegisterd)&&(co->cflag != 0xFF))
        {
            /*add for test only.*/
            #if 0
            for (i = 0; i < 3; i++)
            {
                while (!mport->fn_write_allow())
                {
                    barrier();
                }
                mport->fn_write_byte('#');
            }
            #endif
            //only for test :return directly..
            //return ;
            
            while (1) {
                c = CIRC_SPACE_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);
                if (total < c)
                    c = total;
//#if UART_FIX_CONSOLE_LOG_LOST_PROBLEM
                if(!g_u4enable_log_droped)
                {
                	if ((c <= 0) && (total>0))
	                {
	                    _mt33xx_uart_start_tx((struct uart_port *)mport);
	                    continue;
	                }
//#else
                }
                else
                {
	                if (c <= 0)
	                    break;
                }
//#endif
                memcpy(xmit->buf + xmit->head, buf, c);
                xmit->head = (xmit->head + c) & (UART_XMIT_SIZE - 1);
                buf += c;
                total -= c;
//#if UART_FIX_CONSOLE_LOG_LOST_PROBLEM
                if(!g_u4enable_log_droped)
                {
					if(total <= 0)
	                {
	                    break;
	                }
            	} 
//#endif
            }

            _mt33xx_uart_start_tx((struct uart_port *)mport);

        }
        else
        {
            //add for test only.
            #if 0
            for (i = 0; i < 3; i++)
            {
                while (!mport->fn_write_allow())
                {
                    barrier();
                }
                mport->fn_write_byte('&');
            }
            #endif
			//for flush circ_buf
            if (fgUartRegisterd)
            {
				local_irq_save(flags);
				while (!uart_circ_empty(xmit))
				{
					if (mport->fn_write_allow(mport->nport))
					{
						_mt33xx_uart_tx_chars(mport);
					}
				}
				mport->tx_stop = 0;
				local_irq_restore(flags);
            }
	
            for (i = 0; i < count; i++)
            {
                while (!mport->fn_write_allow(mport->nport))
                {
                    barrier();
                }
#if 0
                    if(g_UartAsyncMode) //USE ASYNC MODE
                    {
                        _mt33xx_uart_write_byte(UART_PORT0, s[i]);
                    }
                    else
#endif
                    {
                         mport->fn_write_byte(mport->nport,s[i]);
                    }


                if (s[i] == '\n')
                {
                    while (!mport->fn_write_allow(mport->nport))
                    {
                        barrier();
                    }
#if 0
                    if(g_UartAsyncMode) //USE ASYNC MODE
                    {
                        _mt33xx_uart_write_byte(UART_PORT0, "\r");
                    }
                    else
#endif
                    {
                        mport->fn_write_byte(mport->nport,'\r');
                    }

                 }
            }
        }

    }

}

static void __init _mt33xx_uart_console_get_options(struct uart_port *port,
    int *pbaud, int *pparity, int *pbits)
{
    int baud=0, parity=0, stopbit, datalen;
    unsigned int uCOMMCTRL;

   
    uCOMMCTRL = UART0_COMMCTRL;

    _mt33xx_uart_get(uCOMMCTRL, &baud, &datalen, &stopbit, &parity);

    if (pbaud)
    {
        *pbaud = baud;
    }

    if (pparity)
    {
        switch (parity)
        {
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

    if (pbits)
    {
        *pbits = datalen;
    }

  // printk(KERN_INFO "[KERNEL UART]console_get_options baud= %d,datalen = %d,stopbit =%d,parity = %d \n",baud,datalen,stopbit,parity);
}

static int __init _mt33xx_uart_console_setup(struct console *co, char *options)
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
    port = (struct uart_port *)&_mt33xx_uart_ports[co->index];

    if (options)
    {
        uart_parse_options(options, &baud, &parity, &bits, &flow);
    }
    else
    {
        _mt33xx_uart_console_get_options(port, &baud, &parity, &stopbit);
    }

    ret = uart_set_options(port, co, baud, parity, bits, flow);
    printk(KERN_INFO "MT3360 console setup : uart_set_option port(%d) "
        "baud(%d) parity(%c) bits(%d) flow(%c) - ret(%d)\n",
        co->index, baud, parity, bits, flow, ret);
    return ret;
}

static struct uart_driver _mt33xx_uart_reg;

static struct console _mt33xx_uart_console =
{
    .name       = "ttyMT",
    .write      = _mt33xx_uart_console_write,
    .device     = uart_console_device,
    .setup      = _mt33xx_uart_console_setup,
    .flags      = CON_PRINTBUFFER,
    .index      = -1,
    .data       = &_mt33xx_uart_reg,
};

static int __init _mt33xx_uart_console_init(void)
{
    // set to transparent if boot loader doesn't do this
    /*_mt33xx_u0_set_trans_mode_on();*/
    register_console(&_mt33xx_uart_console);

    return 0;
}

console_initcall(_mt33xx_uart_console_init);

static int __init _mt33xx_late_console_init(void)
{
    if (!(_mt33xx_uart_console.flags & CON_ENABLED))
    {
        register_console(&_mt33xx_uart_console);
    }

    return 0;
}

late_initcall(_mt33xx_late_console_init);

#define MT33XX_CONSOLE &_mt33xx_uart_console
#else

#define MT33XX_CONSOLE NULL

#endif /* CONFIG_SERIAL_MT33XX_CONSOLE */

static struct uart_driver _mt33xx_uart_reg =
{
    .owner          = THIS_MODULE,
    .driver_name    = "MT33XX serial",
    .dev_name       = "ttyMT",
    .major          = SERIAL_MT33XX_MAJOR,
    .minor          = SERIAL_MT33XX_MINOR,
    .nr             = UART_NR,
    .cons           = MT33XX_CONSOLE,
};


/*
* probe? remove? suspend? resume? ids?
*/

static int _mt33xx_uart_probe(struct device *_dev)
{
    struct platform_device *dev = to_platform_device(_dev);

    _mt33xx_uart_ports[dev->id].port.dev = _dev;
    uart_add_one_port(&_mt33xx_uart_reg, &_mt33xx_uart_ports[dev->id].port);
    dev_set_drvdata(_dev, &_mt33xx_uart_ports[dev->id]);
    return 0;
}

static int _mt33xx_uart_remove(struct device *_dev)
{
    struct mt33xx_uart_port *mport = dev_get_drvdata(_dev);

    dev_set_drvdata(_dev, NULL);

    if (mport)
    {
        uart_remove_one_port(&_mt33xx_uart_reg, &mport->port);
    }

    return 0;
}

static int _mt33xx_uart_suspend(struct device * _dev, pm_message_t state)
{
#if 0
    struct mt33xx_uart_port *mport = dev_get_drvdata(_dev);

    if (mport && level == SUSPEND_DISABLE)
    {
        uart_suspend_port(&_mt33xx_uart_reg, &mport->port);
    }
#endif
    return 0;
}

static int _mt33xx_uart_resume(struct device *_dev)
{
#if 0
    struct mt33xx_uart_port *mport = dev_get_drvdata(_dev);

    if (mport && (level == RESUME_ENABLE))
    {
        uart_resume_port(&_mt33xx_uart_reg, &mport->port);
    }
#endif
    return 0;
}

static struct device_driver _mt33xx_uart_driver =
{
    .name           = "MT33XX uart",
    .bus            = &platform_bus_type,
    .probe          = _mt33xx_uart_probe,
    .remove         = _mt33xx_uart_remove,
    .suspend        = _mt33xx_uart_suspend,
    .resume         = _mt33xx_uart_resume,
};

//extern void printascii(const char *);

/*
* init, exit and module
*/

static int __init _mt33xx_uart_init(void)
{
    int ret;
    int i;

//    printascii("_mt33xx_uart_init \r\n");

    /* reset hardware */
    _mt33xx_uart_hwinit();  /* all port 115200, no int */

    printk(KERN_INFO "Serial: MT33XX driver $Revision: #1 $\n");
    ret = uart_register_driver(&_mt33xx_uart_reg);
    
    if (ret)
    {
        printk(KERN_ERR "Serial: uart_register_driver failure %d\n", ret);
        goto out;
    }

	for (i = 0; i < UART_NR; i++)
		_irq_allocated[i] = 0 ;

    for (i = 0; i < UART_NR; i++)
    {  
        ret = uart_add_one_port(&_mt33xx_uart_reg, &_mt33xx_uart_ports[i].port);
#if MT33XX_UART_DEBUG		
		printk(KERN_INFO "Serial : add uart port %d , line =%d  ret = %x\n", i,ret,_mt33xx_uart_ports[i].port.line);
#endif
    }

    ret = driver_register(&_mt33xx_uart_driver);
    if (ret == 0)
    {
        goto out;
    }
	else
	{
		printk(KERN_ERR "Serial: driver_register failure %d\n", ret);
	}
	
//unreg:
//    uart_unregister_driver(&_mt33xx_uart_reg);

out:
    return ret;
}

static void __exit _mt33xx_uart_exit(void)
{
    uart_unregister_driver(&_mt33xx_uart_reg);
}

module_init(_mt33xx_uart_init);
module_exit(_mt33xx_uart_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MT33XX serial port driver $Revision: #1 $");
MODULE_LICENSE("GPL");
