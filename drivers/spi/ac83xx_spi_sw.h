
#ifndef AC83XX_SPI_SW_H
#define AC83XX_SPI_SW_H

#include "x_typedef.h"

#define SPI_TX_FIFO_DEPTH 32
#define SPI_RX_FIFO_DEPTH 32

#define SPI_CF0_DEFAULT_VALUE       0x05050505
#define SPI_CF1_DEFAULT_VALUE       0x03ffff32

#define SPI_MOTO_OK          1
#define SPI_MOTO_ERR         2
#define SPI_MOTO_TIMEOUT     3


#define SPI_MOTIO_OPERATION_TIMEOUT  60000
/*
typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned int    UINT32;

typedef char   INT8; 
typedef short  INT16;
typedef int    INT32;



/** \enum SPI_TIME_TYPE
 * \ingroup spi
 *
 * @brief
 * Specify the time intervals used by the SPI interface.
 */
enum SPI_TIME_TYPE
{
    SPI_TIME_SETUP,
    /**<
    * \ingroup spi
    * Setup time.
    */
    SPI_TIME_HOLD,
    /**<
    * \ingroup spi
    * Hold time.
    */
    SPI_TIME_LOW,
    /**<
    * \ingroup spi
    * Low voltage time of the SPI clock.
    */
    SPI_TIME_HIGH,
    /**<
    * \ingroup spi
    * High voltage time of the SPI clock.
    */
    SPI_TIME_IDLE
    /**<
    * \ingroup spi
    * Idle time.
    */
};
typedef enum SPI_TIME_TYPE SPI_TIME_TYPE;

/** \enum SPI_INT_TYPE
 * \ingroup spi
 *
 * @brief
 * SPI interrupt category enum.
 *
 * This enumeration defines the two interrupts which SPI devices can generate.
 */
enum SPI_INT_TYPE
{
    SPI_INT_PAUSE,
    /**<
    * \ingroup spi
    * Pause interrupt.
    */
    SPI_INT_FINISH
    /**<
    * \ingroup spi
    * Finish interrupt.
    */
};
typedef enum SPI_INT_TYPE SPI_INT_TYPE;

/** \enum SPI_DIRECTION_TYPE
 * \ingroup spi
 *
 * @brief
 * SPI direction enum.
 *
 * This enumeration defines whether this SPI operation is
 * transmission of reception.
 */
enum SPI_DIRECTION_TYPE
{
    SPI_TX,
    /**<
    * \ingroup spi
    * Means transmission
    */
    SPI_RX
    /**<
    * \ingroup spi
    * Means reception
    */
};
typedef enum SPI_DIRECTION_TYPE SPI_DIRECTION_TYPE;

/** \enum SPI_MLSB
 * \ingroup spi
 *
 * @brief
 * Specify the MSB or LSB used by the SPI TX/RX operation.
 */
enum SPI_MLSB
{
    SPI_LSB = 0,
    /**<
    * \ingroup spi
    * LSB.
    */
    SPI_MSB
    /**<
    * \ingroup spi
    * MSB.
    */
};
typedef enum SPI_MLSB SPI_MLSB;

/** \enum SPI_ENDIAN
 * \ingroup spi
 *
 * @brief
 * Specify the endian used by the SPI interface.
 */
enum SPI_ENDIAN
{
    SPI_ENDIAN_BIG = 0,
    /**<
    * \ingroup spi
    * Big endian.
    */
    SPI_ENDIAN_LITTLE
    /**<
    * \ingroup spi
    * Little endian.
    */
};
typedef enum SPI_ENDIAN SPI_ENDIAN;

/** \enum SPI_CPOL
 * \ingroup spi
 *
 * @brief
 * Choose the desired clock polarities supported by the SPI interface.
 */
enum SPI_CPOL_
{
    SPI_CPOL_0 = 0,
    /**<
    * \ingroup spi
    * SPI clock polarity 0.
    */
    SPI_CPOL_1
    /**<
    * \ingroup spi
    * SPI clock polarity 1.
    */
};
typedef enum SPI_CPOL_ SPI_CPOL_;

/** \enum SPI_CPHA
 * \ingroup spi
 *
 * @brief
 * Choose the desired clock formats supported by the SPI interface.
 */
enum SPI_CPHA_
{
    SPI_CPHA_0 = 0,
    /**<
    * \ingroup spi
    * SPI clock format 0.
    */
    SPI_CPHA_1
    /**<
    * \ingroup spi
    * SPI clock format 1.
    */
};
typedef enum SPI_CPHA_ SPI_CPHA_;

/** \enum SPI_MODE
 * \ingroup spi
 *
 * @brief
 * Choose the SPI FIFO mode or the SPI DMA mode.
 */
enum SPI_MODE
{
    SPI_MODE_FIFO = 0,
    /**<
    * \ingroup spi
    * SPI FIFO mode.
    */
    SPI_MODE_DMA
    /**<
    * \ingroup spi
    * SPI DMA mode.
    */
};
typedef enum SPI_MODE SPI_MODE;

enum SPI_STATE
{
    SPI_STATE_IDLE2IDLE,
    SPI_STATE_IDLE2PAUSE,
    SPI_STATE_PAUSE2PAUSE,
    SPI_STATE_PAUSE2IDLE    
} ;
typedef enum SPI_STATE SPI_STATE;


enum SPI_BIT_STATUS
{
    SPI_DISABLE,
    SPI_ENABLE
};
typedef enum SPI_BIT_STATUS SPI_BIT_STATUS;

typedef void (* SPI_MOTO_ISR_FUN)(void);

typedef struct _tag_SPI_CONFIG
{
    UINT8 setup_time;
    UINT8 hold_time;
    UINT8 clk_low;
    UINT8 clk_high;
    UINT8 idle_time;
    UINT32 clk_polarity;
    UINT32 clk_fmt;
    UINT32 enable_pause_mode;
    UINT32 enable_deassert_mode;
    SPI_BIT_STATUS enable_pause_int;
    SPI_BIT_STATUS enable_finish_int;
    SPI_ENDIAN tx_endian;
    SPI_ENDIAN rx_endian;
    SPI_MLSB tx_mlsb;
    SPI_MLSB rx_mlsb;
    SPI_MODE tx_mode;
    SPI_MODE rx_mode;
}SPI_USER_CONFIG; 

typedef struct _spi_rw_config
{
    UINT32 size;
    UINT32 rPA;
    UINT32 wPA;
}SPI_RW_CONFIG; 

/* Export function prototype. */

extern INT32 SPI_Moto_HAL_EnableInterrupt(UINT32 en);
extern INT32 SPI_Moto_HAL_RegisterDMAISR(SPI_MOTO_ISR_FUN pISRFun);
extern void SPI_Moto_HAL_DMAISR(UINT16 ui2_vector_id);


UINT32 SPI_SetTimeInterval(SPI_TIME_TYPE const type,  UINT8 const value);
UINT32 SPI_SetDesiredSize(UINT16 const pkg_length, UINT16 const pkg_count);
UINT32 SPI_SetRWAddr(SPI_DIRECTION_TYPE const type, UINT32 addr);
UINT32 SPI_ClearFifo(SPI_DIRECTION_TYPE const direction);
void SPI_PushTxFifo(UINT32 data);
UINT32 SPI_PopFifo(SPI_DIRECTION_TYPE const direction, UINT32 * data);
UINT32 SPI_SetInterrupt(SPI_INT_TYPE const type, SPI_BIT_STATUS const status);
UINT32 SPI_SetEndian(SPI_DIRECTION_TYPE const direction, SPI_ENDIAN const endian);
UINT32 SPI_SetMsb(SPI_DIRECTION_TYPE const type, SPI_MLSB const msb);
UINT32 SPI_SelectMode(SPI_DIRECTION_TYPE const type, SPI_MODE const mode);
void SPI_SetCpol(UINT32 const status);
void SPI_SetCpha(UINT32 const status);
void SPI_SetDeassertMode(UINT32 const status);
void SPI_SetPauseMode(UINT32 const status);
UINT32 SPI_IsInPasuseMode(void);
void SPI_PauseModeResume(UINT32 const status);
void SPI_Activate(void);
UINT32 SPI_IsBusy(void);
void SPI_LISR(void);
UINT32 SPI_Hal_Init(void);
void SPI_Moto_Write_reg(UINT32 reg,UINT32 data);

UINT32 SPI_WaitFinished(void);
void SPI_PushRxFifo(UINT32 const data);
void SPI_Moto_Clear_Int(void);
UINT32 SPI_Moto_Is_Int(void);
UINT32 SPI_Set_User_Config(SPI_USER_CONFIG *userConfig);


#define IOCTL_SPI_SET_USER_CONFIG   CTL_CODE(FILE_DEVICE_UNKNOWN,0x0301,METHOD_BUFFERED,FILE_ANY_ACCESS)
#define IOCTL_SPI_LOOPBACK_MODE     CTL_CODE(FILE_DEVICE_UNKNOWN,0x0303,METHOD_BUFFERED,FILE_ANY_ACCESS)
#define IOCTL_SPI_READ_DATA         CTL_CODE(FILE_DEVICE_UNKNOWN,0x0304,METHOD_BUFFERED,FILE_ANY_ACCESS)
#define IOCTL_SPI_WRITE_DATA        CTL_CODE(FILE_DEVICE_UNKNOWN,0x0305,METHOD_BUFFERED,FILE_ANY_ACCESS)

#define MAX_PACKET_LOOP_CNT     256
#define MAX_PACKET_LENGTH       1024
#define MAX_TRANSCATION_BYTE (MAX_PACKET_LOOP_CNT * MAX_PACKET_LENGTH)

#define SPI_DMA_BUF_SIZE   MAX_TRANSCATION_BYTE //4096


#endif  //SPI_MOTO_SW_H
