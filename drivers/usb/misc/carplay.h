#ifndef _CARPLAY_H_
#define _CARPLAY_H_

#include <linux/usb.h>



enum eCarplay
{
	eCarplay_UnKnown = 0,
	eCarplay_NotSupportCarPlay = 1,
	eCarplay_SupportCarPlay,
};


//struct usb_carplay 
//{
//	struct usb_device *udev;
//	struct urb *ctrl_in_urb;
//	struct completion ctrl_in_completion;
//	int uhid_len;
//	int ucarplaystatus; //check if it support carplay
//};

extern int _carplay_status;
#endif
