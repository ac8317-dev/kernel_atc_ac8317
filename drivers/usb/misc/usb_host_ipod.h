#ifndef _USB_HOST_IPOD_H_
#define _USB_HOST_IPOD_H_

#include <linux/cdev.h>

//#define DEBUG 0
#define DEBUG_LV1 0
#define SUPPORT_ACCESSORYINFO
static int DEBUG;
#define IPOD_ACCESSORY_STR_MAX 40
extern char g_AccessoryName[IPOD_ACCESSORY_STR_MAX];//Assessory Name
extern unsigned char g_AccessoryFirmWareVersion[3];//FirmWareVersion
extern char g_AccessoryManufacturer[IPOD_ACCESSORY_STR_MAX];//Manufacturer
extern char g_AccessoryModelNumber[IPOD_ACCESSORY_STR_MAX];//Model Numer
extern char g_PandoraDNS[IPOD_ACCESSORY_STR_MAX];//DNS String
extern char g_PandoraBundleSeed[IPOD_ACCESSORY_STR_MAX];//Bundle Seed String
extern unsigned char g_PandoraProtocolIndex;//Protocol Index
extern unsigned short g_uSessionID;//Session ID

/* Get a minor range for your devices from the usb maintainer */
#define USB_IPOD_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

/* Structure to hold all of our device specific stuff */

#define to_ipod_dev(d) container_of(d, struct usb_carplay, kref)


typedef enum
{
    REP_SEQ_POS_ONLY=0x00,
    REP_SEQ_POS_START=0x02,
    REP_SEQ_POS_MID=0x03,
    REP_SEQ_POS_END=0x01,
    REQ_SEQ_POS_NO=0x04
}REPORT_SEQ_POS;

typedef enum
{
    HID_REPORT,
    HID_SEQ_POS,
    HID_SOF,
    HID_LEN,
    HID_DATA,
    HID_ZERO
}HID_REPORT_STATUS;

//认证过程
typedef enum
{
    AUTH_NOTHING,
    AUTH_START,
    AUTH_NO_IDPS_START,
    AUTH_NO_IDPS_END,
    AUTH_OK,
    AUTH_NG,
}AUTH_STATUS;

typedef struct _FLAGS {
    unsigned char    Open           : 1; // bits 0
    unsigned char    UnloadPending  : 1; // bits 1
    unsigned char    bThreadActive  : 1; // bits 2
    unsigned char    Reserved       : 5;//bits3-7
} FLAGS;

typedef struct
{
    unsigned int dwEmptyLen;//空的buffer长度
    unsigned char bReportId;//report id
    unsigned long uFrameLen;//单个report帧长度
    unsigned bSeqPos;//seq pos
    unsigned long uPacketLen;//该iAp包长度
    unsigned long uDataLen;//该iAp包数据长度
    unsigned long uCurrDataLen;//当前已分析的iAp包数据长度
    unsigned long uPnDataLen;//当前的iAp分包数据长度
    unsigned long uCurrPnDataLen;//当前已分析的iAp分包数据长度
    bool bIsLargePackage;//是否是大包
    unsigned long uAddedZero;//尾部0的长度
    unsigned long uCurrAddedZero;//当前尾部0的长度
}HID_REPORT_ANALYSE;

typedef struct
{
    unsigned int id;//Report Id
    unsigned int count;//Report Count
}ipod_write_report;

typedef struct
{
    unsigned int id;//Report Id
    unsigned int count;//Report Count
}ipod_read_report;
//最大的Report个数

struct ipod_machinfo{
    int devno;
    int major;
    struct cdev ipod_dev;
    struct class *dev_class;
};

#define USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(d, i) ((unsigned short)((unsigned short)d<<8 | i))

#if 0
struct usb_carplay {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*int_in_urb;		/* the urb to read data with */
	struct urb		*ctrl_in_urb;
	unsigned char       int_in_buffer[64*12];		/* the buffer to receive data */
	unsigned int		int_in_size;		/* the size of the receive buffer */
	unsigned int		int_in_filled;		/* number of bytes in the buffer */
	unsigned int		int_in_copied;		/* already copied to user space */
	unsigned int ucarplaystatus;
	__u8			int_in_endpointAddr;	/* the address of the int in endpoint */
	__u8			bInterval;
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	bool			ongoing_read;		/* a read is going on */
	bool			processed_urb;		/* indicates we haven't processed the urb */
    bool            isRead;
    bool            isWrite;
    int             ReadCount;
    int             app_read_count;
    int             AnalyseCount;
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct mutex		read_mutex;		/* synchronize I/O with disconnect */
	struct mutex		write_mutex;		/* synchronize I/O with disconnect */
	struct mutex		ipod_analyse_mutex;
	struct mutex		ipod_read_mutex;
	struct mutex		ipod_write_mutex;
	struct completion	int_in_completion;	/* to wait for an ongoing read */
	struct completion	ctrl_in_completion;
	struct completion	ctrl_out_completion;
	//struct completion	analyse_completion;	/* to wait read complite to anaylse data */
	struct semaphore	analyse_completion;	/* to wait read complite to anaylse data */
	struct completion	read_completion;
	//struct semaphore	read_completion;
	//struct completion	ipod_auth_completion;
	struct semaphore	ipod_auth_completion;

	IPOD_MODEL_IDS		ipod_model_ids;
	AUDIO_TYPE		ipod_audio_type;
	IDPS_STATE		idps_state;

	unsigned int		ipod_rate;
	unsigned long	sig;
	int			wTotalLength_Des;
	unsigned short		Lenght_HID;

	unsigned long		m_unanalyse_buf_count;
	//unsigned char*		m_analyse_buf;
	unsigned char		m_analyse_buf[HID_READ_BUF_STROTE_SIZE];
	unsigned long		m_unanalyse_tmp_buf_count;
	//unsigned char*		m_analyse_tmp_buf;
	unsigned char		m_analyse_tmp_buf[HID_READ_BUF_STROTE_SIZE];
	unsigned long		m_unread_tmp_buf_count;
	//unsigned char*		m_read_tmp_buf;
	unsigned char		m_read_tmp_buf[IPOD_RX_DATA_LENGTH_MAX];
	unsigned long		m_unread_buf_count;
	//unsigned char*		m_read_buf;
	unsigned char		m_read_buf[HID_READ_BUF_STROTE_SIZE];

	struct task_struct	*ipod_analyse_thread_task;
	struct task_struct	*ipod_read_thread_task;
	struct task_struct	*ipod_auth_thread_task;

	ipod_write_report	m_ipod_write_report[HID_REPORT_NUM_MAX];
	int			m_ipod_write_report_size;
	ipod_read_report	m_ipod_read_report[HID_REPORT_NUM_MAX];
	int			m_ipod_read_report_size;
	ANALYSE_STATE		analyes_state;
	READ_STATE		read_state;
	AUTH_EVENT_STATE	auth_event_state;
	FLAGS			m_flags;
	AUTH_STATUS		bAuthProcess;
	bool			bFisrtAuth;
	int uhid_len; 
    struct ipod_machinfo ipod_info;
};
#endif

//extern ssize_t ipod_write_base(struct usb_carplay *dev, char *buf,size_t count);

//extern ssize_t ipod_read_base(struct usb_carplay *dev,char *buffer,size_t count);

//int init_data(struct usb_interface *interface,
//            const struct usb_device_id *id, struct usb_carplay *dev);


#endif
