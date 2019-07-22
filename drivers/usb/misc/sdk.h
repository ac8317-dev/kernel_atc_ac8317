#ifndef _ipod_sdk_h_
#define _ipod_sdk_h_

#define SUPPORT_IDPS
#define SUPPORT_IP
#define SUPPORT_DIGITAL_AUDIO
#define SUPPORT_LOGO
#define EXITLENFLAG 0xffff
#define  SMALLPACKETMAXSIZE 255//小包最大大小

//#define P {printk("Ipod,function=%s,line = %d\n",__FUNCTION__,__LINE__);}
#define P

static const unsigned short supported_sample_rates[]= {
    0x00, 0x00, 0x7D, 0x00,                                                 // 32000
    0x00, 0x00, 0xAC, 0x44,                                                 // 44100
    0x00, 0x00, 0xBB, 0x80,                                                 // 48000
};

typedef enum
{
    WAIT_TIMEOUT = 1,
    TIMER_KILL_EXIT_EVENT,
    TIMER_KILL_EVENT,
}TIME_KILL_STATE;

typedef enum
{
    hAuthExit =1,	//开始认证事件
    g_hStartAuth,	//结束认证事件
    g_hEndAuth,	//开始不带IDPS认证开始事件
    g_hNoIDPSStart,	//发送不带IDPS的认证开始事件
    g_hNoIDPSEnd,
}AUTH_EVENT_STATE;

typedef enum
{
    TIMER_SET_EXIT_EVENT=1,
    TIMER_SET_EVENT,
}TIME_SET_STATE;

typedef enum
{
    CLOSE_ANALYSE_THREAD=1,
    NEW_ANALYSE_EVENT,
}ANALYSE_STATE;

typedef enum
{
    CLOSE_READ_THREAD=1,
    NEW_READ_EVENT,
    ERROR_STATE,
}READ_STATE;

typedef struct ipod_ioctl_var{
    unsigned char ipod_buf[10];
    int buf_len;
}ipod_ioctl_var;

typedef struct
{
    int use_idps;//是否使用IDPS
    short trans_id;//TransID
}IDPS_STATE;

typedef enum
{
    AUDIO_UNKNOWN=0x00,
    AUDIO_DIGITAL,
    AUDIO_ANALOG,
}AUDIO_TYPE;

//FIDToken value define
typedef enum
{
	IDENTIFYTOKEN=0x0000,
	ACCCAPSTOKEN=0x0001,
	ACCINFOTOKEN=0x0002,	//this will set 7 kinds of AccInfoType
	IPODPREFERENCETOKEN=0x0003,   //set iPod preferences
	EAPROTOCOLTOKEN=0x0004,//required for accessories that communicate with application
	BUNDLESEEDIDPREFTOKEN=0x0005,//required for accessories that communicate with application
	MICROPHONECAPSTOKEN=0x0100,
	ENDTOKEN=0xFFFF
}FIDTokenVal;
//ACCInfoType define
typedef enum
{
	ACCESSORYNAME=0x01,
	ACCESSORYFIRMWAREVERSION=0x04,
	ACCESSORYHARDWAREVERSION=0x05,
	ACCESSORYMANUFACTURER=0x06,
	ACCESSORYMODELNUMBER=0x07,
	ACCESSORYSERIALNUMBER=0x08,
	ACCESSORYMAXPLAYLOADSIZE=0x09,  // 2 bytes
	ACCESSORYSTATUS=0x0B,//32-bit mask
	ACCESSORYRFCERTIFICATION=0x0C,//32 bit RF certification mask
}ACCInfoType;

//iPod Preferences ClassID and SettingID define
typedef enum
{
	VIDEOOUTSETTING=0x00,
	SCREENCONFIGURATION=0x01,
	VIDEOSIGNALFORMAT=0x02,
	LINEOUTUSAGE=0x03,
	VIDEOOUTCONNECTION=0x08,
	CLOSEDCAPTIONING=0x09,
	VIDEOMONITORASPECTRATIO=0x0A,
	SUBTITLES=0x0C,
	VIDEOALTERNATEAUDIOCHANNEL=0x0D,
	PAUSEONPOWERREMOVAL=0x0F,
}PREFERENCESClassIDSettingID;

//end status defined
typedef enum
{
	IDPSSUCCESS=0x00,
	IDPSRETRY=0x01,
	IDPSABANDON=0x02,
}ENUM_IDPSSTATUS;

typedef enum
{
    ACK_SUCCESS                         =0x00,
    ACK_ERR_UNKNOWN_DB_CATEGORY	        =0x01,
    ACK_ERR_CMD_FAILED                  =0x02,
    ACK_ERR_OUT_OF_RESOURCES            =0x03,
    ACK_ERR_BAD_PARAMETER               =0x04,
    ACK_ERR_UNKNOWN_ID                  =0x05,
    ACK_PENDING                         =0x06,
    ACK_ERR_NOT_AUTHENTICATED           =0x07,
    ACK_ERR_BAD_AUTH_VERSION            =0x08,
    ACK_ERR_CERTIFICATE_INVALID         =0x0A,
    ACK_ERR_CERTIFY_PERMISSIONS_INVALID = 0x0B,
    ACK_ERR_INVALI_ACCESSARY_RESISTOR_ID_VALUE = 0x11,
}IPOD_ACK;


typedef enum
{
    GENERAL_LINGO                   = 0x00,
    MICROPHONE_LINGO                = 0x01,
    SIMPLE_REMOTE_LINGO             = 0x02,
    DISPLAY_REMOTE_LINGO            = 0x03,
    EXTENDED_INTERFACE_LINGO        = 0x04,
    RF_TRANSMITTER_LINGO            = 0x05,
    DIGITAL_AUDIO_LINGO             = 0x0A,
    STORAGE_LINGO                   = 0x0C,
} IPOD_LINGO_ID;

/*Lingo:0x04,Cmd:0x0034,0x003A,LogoFormats*/
typedef struct
{
	unsigned char dispPixelFormat;
	unsigned short maxWidth;
	unsigned short maxHeight;
	int size;
}iPodLogoFormat;

typedef struct
{
	unsigned char bLogoPixFormat;
	unsigned short uLogoWidth;
	unsigned short uLogoHeight;
	unsigned char* pLogoData;
	unsigned int uLogoDataLen;
}STRUCT_LOGO_FILE;

typedef	struct{
	unsigned char pixelFormat;//
	unsigned char image_data[51200];//图像数据
	unsigned char *pImage;//图像数据偏移指针
	unsigned short maxWidth;//图像最大宽度
	unsigned short maxHeight;//图像最大高度
	unsigned short width;//图像宽度
	unsigned short height;//图像高度
	unsigned int row_byte;//每行所需要的字节数
	unsigned short telegram_index;//Descriptor telegram index
	unsigned int total_byte;//图像总的字节数
	unsigned short left_len;//图像剩下未传的字节数
	unsigned short this_write_len;//图像这次需要传的字节数
}STRUCT_LOGO;

typedef	struct
{
	unsigned char	device_ver;// device version
	unsigned char	firmware_ver;// firmware version
	unsigned char	protocol_ver[2];// protocol version
	unsigned char	device_id[4];// Device ID

	unsigned char	current_section;// section counter for RetDevAuthenticationInfo
	unsigned char	max_section;// max section number证书的最大页号(基数0)
	unsigned short	certification_length;// Certification data length from cp register
	unsigned short	section_size;//1 section data size include packe payload to section data end(not include checksum)
	unsigned short	last_section_size;// last section data size include packe payload to section data end ( not include checksum )
	unsigned char	*data;// start pointer to return data for RetDevAuthenticationInfo

	unsigned char	challenge_data[22];// challenge data length (first 2-byte) & data(other 20-byte) for level V2
	//unsigned char	challenge_data[20];// challenge data length (first 2-byte) & data(other 20-byte) for level V2
	unsigned char	retry_counter;// authentication retry counter from GetDevAuthenticationSignature

	unsigned short	signature_length;// Signature data length from cp register
	unsigned char	*sig_data;// start pointer to return data for RetDevAuthenticationSignature
	unsigned int	current_sample_rate;//采样率
	unsigned char	m_ReqLingo;//请求版本的Lingo
	unsigned char	logo_ver[2];//logo version
	FIDTokenVal m_eFidtokenvalue;//FID token value
	ENUM_IDPSSTATUS m_eIdpsStatus;//IDPS status
	unsigned char m_IDPSErrorCounter;
	unsigned char m_IDPSSucessflag;
	unsigned short m_tokenID;

	bool bIsAckIdentityDeviceLingoes;//是否接收到ACK Identify Device Lingoes
	bool bIsGetDeviceAuthenticationInfo;//是否接收到Get Device Authentication Info
	bool bIsAckRetDevAuthenticationInfo;//是否接收到ACK Ret Device Authentication Info
	bool bIsAckDevAuthenticationInfo;//是否接收到ACK Device Authentication
	bool bIsGetDevAuthenticationSignature;//是否接收到Get Device Authentication Signature
	bool bIsAckDevAuthenticationStatus;//是否接收到ACK Device Authentication Status
	bool bIsAckRetDevAuthenticationSignature;//是否接收到ACK Ret Device Authentication Signature
	bool bIsAckClearIdentifyErrors;//是否接收到ACK Clear Identify Errors
	bool bIsGetAccSampleRateCaps;//是否接收到Get Acc Sample Rate Caps
	bool bIsNewIpodTrackInfo;//是否接收到 New Ipod Track Info
	bool bIsRetRemoteUIMode;//是否接收到Return Remote UI Mode
	bool bIsAckSetEventNotification;
	bool bIsRetSupportedEventNotification;
	bool bIsAckEnterRemoteUIMode;//是否接收到ACK Enter Remote UI Mode
	bool bIsReturnPlayStatus;//是否接收到Return Play Status
	bool bIsRetiPodModelNum;//是否接收到Ret iPod Model Num
	bool bIsRetDispImgLimits;//是否接收到Ret Display Image Limits
	bool bIsAckSetDispImg;//是否接收到ACK Set Display Image
	bool bIsRetiPodOptionsForLingo;//是否接收到Teturn iPod Options For Lingo
	bool bIsSupportedSetOptionsForLingo;
	bool bIsRetLingoProtocolVer;//是否接收到Return Lingo Protocol Version
	bool bIsRetMaxPacketSize;//是否接收到Return Max Packet Size
	bool bIsAckSetRemoteEventNotification;//是否接收到AckSetRemoteEventNotification
	bool bIsAckStartIDPS;//是否接收到AckStartIDPS
	bool bIsRetFIDTokenValueAcks;//是否接收到Ret FID Token Value Acks
	bool bIsIDPSStatus;//是否接收到IDPS Status
}IPOD_AUTH;

typedef enum
{
    IPOD_MODEL_IDS_NOTHING,//Nothing
    IPOD_3G_WHITE_WHITE_WHEEL=0x00030000,//3G iPod.This is the white iPod with 4 buttons above a white click wheel.
    IPOD_MINI_ORIGINAL_4GB=0x00040000,//iPod mini:original 4GB model
    IPOD_4G_WHITE_GRAY_WHEEL=0x00050000,//4G iPod.This is the white iPod with a gray click wheel
    IPOD_PHOTO=0x00060000,//iPod photo
    IPOD_2G_4GB_6GB_MINI=0x00070000,//2nd generation iPod mini(models M9800-M9807,4GB and 6GB)
    IPOD_5G=0x000B0000,//5G iPod
    IPOD_NANO=0x000C0000,//iPod nano
    IPOD_2G_NANO=0x00100000,//2G iPod nano
    IPHONE=0x00110000,//iPhone
    IPOD_CLASSIC=0x00130000,//iPod classic
    IPOD_CLASSIC_120GB=0x00130100,//iPod classic 120 GB
    IPOD_3G_NANO=0x00140000,//3G iPod nano
    IPOD_TOUCH=0x00150000,//iPod touch
    IPOD_4G_NANO=0x00170000,//4G iPod nano
    IPHONE_3G=0x00180000,//iPhone 3G
    TOUCH_2G=0x00190000,//2G touch
    TOUCH_4G=0x00210000,//4G touch
    IPHONE_3GS=0x001B0000,//iPhone 3GS
    IPOD_5G_NANO=0x001C0000,//5G iPod nano
    IPOD_TOUCH_2G_2009=0x001D0000,//2G touch(2009)
    IPOD_6G_NANO=0x001E0000,//6G iPod nano
    IPAD_IPAD_3G=0x001F0000,//iPad and iPad 3G
    IPHONE_4=0x00200000,//iPhone 4
}IPOD_MODEL_IDS;

//******************Lingo 0x00,General Lingo****************
#define     ACKGENERALLINGO                 0x02
#define     REQREMOTEUIMODE                 0x03
#define     RETURNREMOTEUIMODE              0x04
#define     ENTERREMOTEUIMODE               0x05
#define     REQIPODMODELNUM                 0x0D
#define     RETIPODMODELNUM                 0x0E
#define     REQUESTLINGOPROTOCOLVERSION     0x0F
#define     RETUESTLINGOPROTOCOLVERSION     0x10
#define     REQTRANSPORTMAXPACKETSIZE       0x11
#define     RETURNTRANSPORTMAXPACKETSIZE    0x12
#define     IDENTIFYDEVICELINGOES           0x13
#define     GETDEVAUTHENTICATIONINFO        0x14
#define     RETDEVAUTHENTICATIONINFO        0x15
#define     ACKDEVAUTHENTICATIONINFO        0x16
#define     GETDEVAUTHENTICATIONSIGNATURE   0x17
#define     RETDEVAUTHENTICATIONSIGNATURE   0x18
#define     ACKDEVAUTHENTICATIONSTATUS      0x19
#define     GETIPODOPTIONS                  0x24
#define     RETIPODOPTIONS                  0x25
#define     GETACCESSORYINFO                0x27
#define     RETACCESSORYINFO                0x28
#define     SETIPODPREFERENCE               0x2B
#define     STARTIDPS                       0x38
#define     SETFIDTOKENVALUES               0x39
#define     RETFIDTOKENVALUEACKS            0x3A
#define     ENDIDPS                         0x3B
#define     IDPSSTATUS                      0x3C
#define     OPENDATASESSIONFORPROTOCOL      0x3F
#define     CLOSEDATASESSION                0x40
#define     DEVACK                          0x41
#define     DEVDATATRANSFER                 0x42
#define     IPODDATATRANSFER                0x43
#define     SETEVENTNOTIFICATION            0x49
#define     IPODNOTIFICATION                0x4A
#define     GETIPODOPTIONSFORLINGO          0x4B
#define     RETIPODOPTIONSFORLINGO          0x4C
#define     GETSUPPORTEDEVENTNOTIFICAITON   0x4F
#define     RETSUPPORTEDEVENTNOTIFICAITON   0x51

//******************Lingo 0x03,Display Remote Lingo****************
#define     ACKDISPLAYREMOTELINGO           0x00
#define     SETREMOTEEVENTNOTIFICATION      0x08
//******************Lingo 0x04,Exterented Interface Lingo****************
#define     ACKEXTENDEDINTERFACELINGO       0x01
#define     GETPLAYSTATUS                   0x1C
#define     RETURNPLAYSTATUS                0x1D
#define     GETMONODISPIMGLIMITS            0x33
#define     RETMONODISPIMGLIMITS            0x34
#define     GETCOLORDISPIMGLIMITS           0x39
#define     RETCOLORDISPIMGLIMITS           0x3A
#define     SETDISPIMAGE                    0x32
//******************Lingo 0x0A,Digital Audio Lingo****************
#define     ACCACK                          0x00
#define     ACKDIGITALAUDIOLINGO            0x01
#define     GETACCSMAPLERATECAPS            0x02
#define     RETACCSMAPLERATECAPS            0x03
#define     NEWIPODTRACKINFO                0x04
#define     SETVIDEODELAY                   0x05

typedef enum
{
	AS_IDLE,//idle   0

	AS_TX_START_IDPS,//Start IDPS 1
	AS_WAIT_ACK_START_IDPS,//wait ACK start IDPS 2

	AS_TX_SET_FID_TOKEN_VALUES,//Set FID Token Values 3
	AS_WAIT_RET_FID_TOKEN_VALUES,//wait Ret FIDTOken Value ACKs 4

	AS_TX_END_IDPS,//End IDPS 5
	AS_WAIT_RET_IDPS_STATE,//wait Ret IDPS State 6

	AS_TX_CLEAR_IDENTIFY_ERRORS,//Clear Identify Errors 7
	AS_WAIT_ACK_CLEAR_IDENTIFY_ERRORS,//wait ACK clear identify errors 8

	AS_TX_IDENTIFY_DEVICE_LINGOES,//Identify Device Lingoes 9
	AS_WAIT_ACK_IDENTIFY_DEVICE_LINGOES,//wait ACK Identify Device Lingoes 10
	AS_WAIT_GET_DEV_AUTHENTICATION_INFO,//wait Get Device Authentication Info  11

	AS_TX_RET_DEV_AUTHENTICATION_INFO,//Ret Device Authentication Info 12
	AS_WAIT_ACK_RET_DEV_AUTHENTICATION_INFO,//wait ACK ret dev authentication info 13
	AS_WAIT_ACK_DEV_AUTHENTICATION_INFO,//wait ACK Device Authentication Info 14

	AS_WAIT_GET_DEV_AUTHENTICATION_SIGNATURE,//wait Get Device Authentication Signature 15

	AS_TX_RET_DEV_AUTHENTICATION_SIGNATURE,//Ret Device Authentication Signature 16
	AS_WAIT_ACK_DEV_AUTHENTICATION_STATUS,//wait ACK Device Authentication Status 17

	AS_SET_IPOD_PREFERENCES,//Set Ipod Preferences 18

	AS_REQ_LINGO_PROTOCOL_VER,//request Lingo Protocol Version  19
	AS_WAIT_RET_LINGO_PROTOCOL_VER,//wait return Lingo Protocol Version  20

	AS_REQ_MAX_PACKET_SIZE,//request Transport Max Packet Size 21
	AS_WAIT_RETURN_MAX_PACKET_SIZE,//wait return Transport Max Packet Size 22

	AS_TX_GETIPODOPTIONS,//Get iPod Options 23
	AS_WAIT_RETIPODOPTIONS,//wait ret iPod Options  24

	AS_TX_GETIPODOPTIONSFORLINGO,//Get iPod Options For Lingo 25
	AS_WAIT_RETIPODOPTIONSFORLINGO,//wait ret iPod Options For Lingo 26

	//AS_WAIT_GET_ACC_SAMPLE_RETE_CAPS,//wait Get Acc Sample Rate Caps
	//AS_TX_RET_ACC_SAMPLE_RATE_CAPS,//Ret Acc Sample Rate Caps

	//AS_WAIT_NEW_IPOD_TRACK_INFO,//wait New Ipod Track Info

	//AS_WAIT_ACK_SET_REMOTE_EVENT_NOTIFICATION,//wait Ack Set Remote Event Notification
	//AS_TX_SET_REMOTE_EVENT_NOTIFICATION,//Set Remote Event Notification

	AS_TX_REQ_REMOTE_UI_MODE,//Request Remote UI Mode 27
	AS_WAIT_RET_REMOTE_UI_MODE,//wait Return Remote UI Mode  28

	AS_TX_GET_SUPPORTED_EVENT_NOTIFICATION,//Get Supported event notification 29
	AS_WAIT_RET_SUPPORTED_EVENT_NOTIFICATION,// 30

	AS_TX_SET_EVENT_NOTIFICATION,//SetEventNotification 31
	AS_WAIT_ACK_SET_EVENT_NOTIFICATION,//wait ACK SetEventNotification 32

	AS_TX_ENTER_REMOTE_UI_MODE,//Request Enter Remote UI Mode 33
	AS_WAIT_ACK_ENTER_REMOTE_UI_MODE,//wait ACK Enter Remote UI Mode 34

	AS_TX_GETPLAYSTATUS,//get play status 35
	AS_WAIT_RETURNPLAYSTATUS,//wait return play status 36

	AS_TX_REQ_IPOD_MODEL_NUM,//Request iPod Model Num 37
	AS_WAIT_RET_IPOD_MODEL_NUM,//wait return iPod Model Num 38

	AS_TX_GET_DISP_IMG_LIMITS,//Get Mono Display Image Limits 39
	AS_WAIT_RET_DISP_IMG_LIMITS,//wait return Mono Display Image Limits 40

	AS_TX_SET_DISP_IMG,//Set Display Image 41
	AS_WAIT_ACK_SET_DISP_IMG,//wait ACK Set Display Image 42

	AS_COMPLETE,//认证完成 43
	AS_ABORT,//abort 44

	AS_UNUSED//unused
}AUTH_STATE;


#define HID_HID_DESCRIPTOR_TYPE         0x21
#define HID_REPORT_DESCRIPTOR_TYPE	0x22
#define HID_REPORT_OUTPUT               0x02
#define HID_REPORT_NUM_MAX		15
#define IPOD_RX_DATA_LENGTH_MAX	1070
#define HID_WRITE_BUF_MAX		4096
#define HID_READ_BUF_MAX		10240
//#define HID_READ_BUF_STROTE_SIZE	512000
//#define HID_READ_BUF_STROTE_SIZE	51200
#define HID_READ_BUF_STROTE_SIZE	10240 * 5

#define HIBYTE(w) ((unsigned char)(((int)(w) >> 8)&0x00ff))
#define LOBYTE(w) ((unsigned char)((int)(w)&0x00ff ))

#define HIWORD(w) ((int)(((int)(w) >> 16)&0xFFFF))
#define LOWORD(w) ((int)((int)(w)&0xFFFF))

//定义IOCTL代码
#define FILE_DEVICE_VIDEO 0x00000023
#define MYIOCTLDEVTYPE  FILE_DEVICE_VIDEO

#define METHOD_BUFFERED 0
#define FILE_ANY_ACCESS 0
#define CTL_CODE( DeviceType, Function, Method, Access ) ( ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) )


#define IOCTL_SET_DEVICE_RATE \
    CTL_CODE(MYIOCTLDEVTYPE, 0x100, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define IOCTL_GET_DEVICE_RATE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x101, METHOD_BUFFERED, FILE_ANY_ACCESS)//get rate

#define IOCTL_SET_IDPS_STATE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x102, METHOD_BUFFERED, FILE_ANY_ACCESS)//set IDPS state

#define IOCTL_GET_IDPS_STATE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x103, METHOD_BUFFERED, FILE_ANY_ACCESS)//get IDPS state

#define IOCTL_SET_AUDIO_TYPE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x104, METHOD_BUFFERED, FILE_ANY_ACCESS)//set audio type

#define IOCTL_GET_AUDIO_TYPE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x105, METHOD_BUFFERED, FILE_ANY_ACCESS)//get audio type

#define IOCTL_GET_IPOD_TYPE \
	CTL_CODE(MYIOCTLDEVTYPE, 0x106, METHOD_BUFFERED, FILE_ANY_ACCESS)//get iPod type

#define IOCTL_SET_IPOD_DISCONNECT \
    CTL_CODE(MYIOCTLDEVTYPE, 0x107, METHOD_BUFFERED, FILE_ANY_ACCESS)//get iPod type
#if 0
#ifdef SUPPORT_IP
//开始认证事件
extern HANDLE g_hStartAuth;
//开始不带IDPS认证开始事件
extern HANDLE g_hNoIDPSStart;
//开始不带IDPS认证结束事件
extern HANDLE g_hNoIDPSEnd;
//结束认证事件
extern HANDLE g_hEndAuth;
#endif
//#ifdef	SUPPORT_DIGITAL_AUDIO

//#endif

//定义认证成功函数类型
typedef void (CALLBACK* OnAuthOk)(void);
#endif
#endif
