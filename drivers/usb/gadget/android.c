/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

//#define ENABLE_CARPLAY

#ifdef ENABLE_CARPLAY
#define CARPLAY_NCM_FUNCTION_NAME					"ncm"
//extern struct android_usb_function ncm_function;
extern struct android_usb_function iap2_function;
#endif /* ENABLE_CARPLAY */

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

//#include "f_fs.c"
//#include "f_audio_source.c"
#ifdef ENABLE_CARPLAY
#include "u_ether.c"
#include "f_adb.c"
#else
#include "f_mass_storage.c"
#include "u_serial.c"
//#include "f_acm.c"
#include "f_adb.c"
//#include "f_ncm.c"
#include "f_iap2.c"
//#include "f_mtp.c"
//#include "f_accessory.c"
#define USB_ETH_RNDIS y
//#include "f_rndis.c"
//#include "rndis.c"
#include "u_ether.c"
#include "../../../../autochips/platform/kernel/drivers/inc/x_ver.h"
#endif
#include "../../../../autochips/platform/kernel/drivers/inc/x_ver.h"

#include <linux/cdev.h>
#include <linux/usb.h>
#include "f_external.c"

//#include <linux/x_ver.h>
MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

//static struct ea_usb_string* ea_initstring = NULL;
extern struct ea_usb_string ea_store_string;
static char* pinitstring = NULL;
static int ninitstringlen = 0;
module_param(ninitstringlen,int,0);
module_param(pinitstring, charp, 0);


//#define USB_TEST_MODE_CONFIG
static const char longname[] = "Gadget Android";

#ifdef  USB_TEST_MODE_CONFIG
extern void usb_test_mode_handle(uint8_t cmd,struct device *pdev);
#endif
/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001
#ifdef  USB_TEST_MODE_CONFIG
uint8_t usb_test_mode_cmd = 0;

#define 	USB_TEST_MODE_OPEN 				1
#define 	USB_TEST_MODE_TEST_NAK 			4
#define 	USB_TEST_MODE_EXIT 				8

#endif

enum eUSBConnectStatus
{
	eUSBConnectStatus_Unknown = 0,
	eUSBConnectStatus_NCMCONNECTED = 1,
	eUSBConnectStatus_IAPCONNECTED_NOTCARPLAY,
	eUSBConnectStatus_IAPCONNECTED_CARPLAY,
	//eUSBConnectStatus_EAUNDEFINE,
	eUSBConnectStatus_EASTART,
	eUSBConnectStatus_EASTOP,
	
};

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool carplayConnected;
	bool sw_connected;
	int carplay;
	int ea_status;
	struct work_struct work;
	char ffs_aliases[256];
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_WAKEUP | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0x00, /* 0ma */
};

#include "f_uac2.c"

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
#ifdef ENABLE_CARPLAY
	char *configured[2]   = { "IPOD_STATE=IAP2CONNECTED", NULL };
	char *disconnected[2] = { "IPOD_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *carplayConnected[2]   = { "IPOD_STATE=CARPLAYCONNECTED", NULL };
#else
	char *disconnected[2] = { "IPOD_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char *charplay[2] = { "IPOD_STATE=CARPLAYCONNECTED", NULL };
	char *iapplay[2] = { "IPOD_STATE=IAP2CONNECTED", NULL };
	char *iapplay_notcarplay[2] = {"NCM_CARP=IAPCONNECTED_NOTCARP", NULL};
	
#endif
	char **uevent_envp = NULL;

	char *ea_start[2] = {"IPOD_STATE=EASTART", NULL};
	char *ea_stop[2] = {"IPOD_STATE=EASTOP", NULL};
	char ** uevent_ea_envp = NULL;
	
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		uevent_envp = configured;
	else if (dev->connected != dev->sw_connected)
		uevent_envp = dev->connected ? connected : disconnected;
#ifdef ENABLE_CARPLAY
	if (cdev->config && dev->connected && dev->carplayConnected) {
		uevent_envp = carplayConnected;
	}
#endif
	dev->sw_connected = dev->connected;

	if(dev->carplay == eUSBConnectStatus_NCMCONNECTED  )
	{
		uevent_envp = charplay;
		printk("android_work  1 path:%s\n",kobject_get_path(&dev->dev->kobj, GFP_KERNEL));
		dev->carplay = eUSBConnectStatus_Unknown;
	}
		if(dev->carplay == eUSBConnectStatus_IAPCONNECTED_CARPLAY )
	{
		uevent_envp = iapplay;
		printk("android_work  2 path:%s",kobject_get_path(&dev->dev->kobj, GFP_KERNEL));
		dev->carplay = eUSBConnectStatus_Unknown;
	}
	if(dev->carplay == eUSBConnectStatus_IAPCONNECTED_NOTCARPLAY)
	{
		uevent_envp = iapplay_notcarplay;
		printk("android_work  3 path:%s",kobject_get_path(&dev->dev->kobj, GFP_KERNEL));
		dev->carplay = eUSBConnectStatus_Unknown;
	}
	if(dev->ea_status== eUSBConnectStatus_EASTART)
	{
		uevent_ea_envp = ea_start;
		//printk("android_work  4 path:%s\n",kobject_get_path(&dev->dev->kobj, GFP_KERNEL));
		dev->ea_status = eUSBConnectStatus_Unknown;
	}

	if(dev->ea_status == eUSBConnectStatus_EASTOP)
	{
		uevent_ea_envp = ea_stop;
		//printk("android_work  5 path:%s\n",kobject_get_path(&dev->dev->kobj, GFP_KERNEL));
		dev->ea_status = eUSBConnectStatus_Unknown;
	}

	if(!dev->connected)
	{
		//if(_carplay_dev)
		//	_carplay_dev->ucarplaystatus = eCarplay_UnKnown;
		printk("dev->connected = %d ++ %s ++ \n", dev->connected, __func__ );
	}
	
	spin_unlock_irqrestore(&cdev->lock, flags);
	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}

	if(uevent_ea_envp)
	{
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_ea_envp);
		pr_info("%s: sent uevent %s\n", __func__, uevent_ea_envp[0]);
	}
}


static void android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (WARN_ON(!dev->disable_depth))
		return;

	if (--dev->disable_depth == 0) {
		usb_add_config(cdev, &android_config_driver,
					android_bind_config);
		usb_gadget_connect(cdev->gadget);
	}
}
static void __exit cleanup(void);
static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	printk("++%s+++ cdev = 0x%x, dev->disable_depth = %d\n", __func__, cdev, dev->disable_depth);
	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &android_config_driver);
	}
	printk("[%s] Line:%d.\n", __FUNCTION__, __LINE__);
//cleanup();
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */
#if 0
struct functionfs_config {
	bool opened;
	bool enabled;
	struct ffs_data *data;
};

static int ffs_function_init(struct android_usb_function *f,
			     struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct functionfs_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return functionfs_init();
}

static void ffs_function_cleanup(struct android_usb_function *f)
{
	functionfs_cleanup();
	kfree(f->config);
}

static void ffs_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = true;

	/* Disable the gadget until the function is ready */
	if (!config->opened)
		android_disable(dev);
}

static void ffs_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = f->config;

	config->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!config->opened)
		android_enable(dev);
}

static int ffs_function_bind_config(struct android_usb_function *f,
				    struct usb_configuration *c)
{
	struct functionfs_config *config = f->config;
	return functionfs_bind_config(c->cdev, c, config->data);
}

static ssize_t
ffs_aliases_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = _android_dev;
	int ret;

	mutex_lock(&dev->mutex);
	ret = sprintf(buf, "%s\n", dev->ffs_aliases);
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t
ffs_aliases_store(struct device *pdev, struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct android_dev *dev = _android_dev;
	char buff[256];

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	strlcpy(buff, buf, sizeof(buff));
	strlcpy(dev->ffs_aliases, strim(buff), sizeof(dev->ffs_aliases));

	mutex_unlock(&dev->mutex);

	return size;
}

static DEVICE_ATTR(aliases, S_IRUGO | S_IWUSR, ffs_aliases_show,
					       ffs_aliases_store);
static struct device_attribute *ffs_function_attributes[] = {
	&dev_attr_aliases,
	NULL
};

static struct android_usb_function ffs_function = {
	.name		= "ffs",
	.init		= ffs_function_init,
	.enable		= ffs_function_enable,
	.disable	= ffs_function_disable,
	.cleanup	= ffs_function_cleanup,
	.bind_config	= ffs_function_bind_config,
	.attributes	= ffs_function_attributes,
};

static int functionfs_ready_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;
	int ret = 0;

	mutex_lock(&dev->mutex);

	ret = functionfs_bind(ffs, dev->cdev);
	if (ret)
		goto err;

	config->data = ffs;
	config->opened = true;

	if (config->enabled)
		android_enable(dev);

err:
	mutex_unlock(&dev->mutex);
	return ret;
}

static void functionfs_closed_callback(struct ffs_data *ffs)
{
	struct android_dev *dev = _android_dev;
	struct functionfs_config *config = ffs_function.config;

	mutex_lock(&dev->mutex);

	if (config->enabled)
		android_disable(dev);

	config->opened = false;
	config->data = NULL;

	functionfs_unbind(ffs);

	mutex_unlock(&dev->mutex);
}

static int functionfs_check_dev_callback(const char *dev_name)
{
	return 0;
}

#endif

//#ifndef ENABLE_CARPLAY
struct adb_data {
	bool opened;
	bool enabled;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	//pr_alert("[USB] %s ++ \n", __func__);
	
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	//pr_alert("[USB] %s ++ \n", __func__);
	
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	//pr_alert("[USB] %s ++ \n", __func__);
	
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	//pr_alert("[USB] %s ++ \n", __func__);
	
	data->enabled = true;

	/* Disable the gadget until adbd is ready */
	if (!data->opened)
		android_disable(dev);
}

static void adb_android_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	//pr_alert("[USB] %s ++ \n", __func__);
	
	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!data->opened)
		android_enable(dev);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};


static int audio_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	printk("audio_function_init\n");
	audiohost_setup();
	return 0;
}

static void audio_function_cleanup(struct android_usb_function *f)
{
	printk("enter audio_function_cleanup\n");
	audio_cleanup();
#ifdef GADGET_UAC1
	//gaudio_cleanup();
#endif
}


static int audio_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	printk("++ ++++audio_function_bind_config111 ++\n");
	
	return audio_bind_config(c);
}

static void audio_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	printk("enter audio_function_unbind_config\n");
	return uac2_unbind_config(c);
#ifdef GADGET_UAC1
	//gaudio_cleanup();
#endif
}


static struct android_usb_function audio_function = {
	.name		= "g_audio",
	.init		= audio_function_init,
	.cleanup	= audio_function_cleanup,
	.bind_config	= audio_function_bind_config,
	.unbind_config	= audio_function_unbind_config,
	//.attributes, if need?
};

static int ea_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	printk("ea_function_init\n");
	return ea_setup();
}

static void ea_function_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	ea_cleanup();

	if(dev == NULL)
	{
		printk("ea_function_cleanup NULL\n");
		return ;
	}

	dev->ea_status = eUSBConnectStatus_Unknown;
	
}

static int ea_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	printk("++ ++++ea_function_bind_config111 ++\n");
	
	return ea_bind_config(c);
}

static void ea_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	
}

static struct android_usb_function ea_function = {
	.name		= "external_accessory",
	.init		= ea_function_init,
	.cleanup	= ea_function_cleanup,
	.bind_config	= ea_function_bind_config,
	.unbind_config	= ea_function_unbind_config,
	//.attributes, if need?
};
/************add by zhou maicheng***********************/

static int iap2_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	printk("iap2_function_init\n");
	return iap2_setup();
}

static void iap2_function_cleanup(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	iap2_cleanup();

	if(dev == NULL)
	{
		printk("iap2_function_cleanup NULL\n");
		return ;
	}

	return;
	
}

static int iap2_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	printk("++ ++++iap2_function_bind_config111 ++\n");
	
	return iap2_bind_config(c);
}

static void iap2_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	
}

static struct android_usb_function iap2_function = {
	.name		= "iap2",
	.init		= iap2_function_init,
	.cleanup	= iap2_function_cleanup,
	.bind_config	= iap2_function_bind_config,
	.unbind_config	= iap2_function_unbind_config,
	//.attributes, if need?
};


/**************************************end by zhoumaicheng**************************/

static void adb_ready_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
}
//#endif

#if 0
#define MAX_ACM_INSTANCES 4
struct acm_function_config {
	int instances;
};

static int
acm_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return gserial_setup(cdev->gadget, MAX_ACM_INSTANCES);
}

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
	kfree(f->config);
	f->config = NULL;
}

static int
acm_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	for (i = 0; i < config->instances; i++) {
		ret = acm_bind_config(c, i);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show,
						 acm_instances_store);
static struct device_attribute *acm_function_attributes[] = {
	&dev_attr_instances,
	NULL
};

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};


static int
mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int
mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int
ptp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int
ptp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	/* "Wireless" RNDIS; auto-detected by Windows */
	bool	wceis;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};

#endif

#ifndef ENABLE_CARPLAY
struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};
#endif

#ifdef ENABLE_CARPLAY
/* ncm function config */
struct ncm_function_config {
	u8	ethaddr[ETH_ALEN];
	struct eth_dev *dev;
};

static int ncm_function_init(struct android_usb_function *f, struct usb_composite_dev *c)
{
	printk("++ ncm_function_init ++\n");
	f->config = kzalloc(sizeof(struct ncm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	printk("-- ncm_function_init --\n");
	return 0;
}

static void ncm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ncm_function_bind_config(struct android_usb_function *f,
			struct usb_configuration *c)
{
	struct ncm_function_config *ncm = f->config;
	int ret;
	struct eth_dev *dev;

	printk("++ ncm_function_bind_config ++\n");
	if(!ncm) {
		printk("%s: ncm config is null\n", __func__);
		return -EINVAL;
	}

	printk("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);

#ifdef ENABLE_CARPLAY
	dev = gether_setup(c->cdev->gadget, ncm->ethaddr);
#else
	dev = gether_setup_name(c->cdev->gadget, ncm->ethaddr, "ncm");
#endif
	if(IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		printk("%s: gether setup failed err:%d\n", __func__, ret);
		return ret;
	}
	printk("ncm_function_bind_config | 002\n");
	ncm->dev = dev;
	
	printk("ncm_function_bind_config | 003\n");
	ret = ncm_bind_config(c, ncm->ethaddr);
	printk("ncm_function_bind_config | 004\n");
	if (ret) {
		printk("%s: ncm bind config failed err: %d", __func__, ret);
		gether_cleanup();
		return ret;
	}
	return ret;
}

static void ncm_function_unbind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	struct ncm_function_config *ncm = f->config;
	gether_cleanup();
}

static ssize_t ncm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);
}

static ssize_t ncm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;

	if(sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		(int*)&ncm->ethaddr[0], (int*)&ncm->ethaddr[1], (int*)&ncm->ethaddr[2],
		(int*)&ncm->ethaddr[3], (int*)&ncm->ethaddr[4], (int*)&ncm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ncm_ethaddr, S_IRUGO|S_IWUSR, ncm_ethaddr_show, ncm_ethaddr_store);

static struct device_attribute *ncm_function_attributes[] = {
	&dev_attr_ncm_ethaddr,
	NULL
};

static struct android_usb_function ncm_function = {
	.name = "ncm",
	.init = ncm_function_init,
	.cleanup = ncm_function_cleanup,
	.bind_config = ncm_function_bind_config,
	.unbind_config = ncm_function_unbind_config,
	.attributes = ncm_function_attributes,
};
#endif

#if 0
static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};
#endif

static struct android_usb_function *supported_functions[] = {
	//&ffs_function,
#ifdef ENABLE_CARPLAY
	&ncm_function,
	&iap2_function,
	&adb_function,
	&audio_function,
	&ea_function,
#else
	&adb_function,
    //&ncm_function,
    &audio_function,
     &iap2_function,
    &ea_function,
#endif /* ENABLE_CARPLAY */
	//&acm_function,
	//&mtp_function,
	//&ptp_function,
	//&rndis_function,
	//&mass_storage_function,
	//&accessory_function,
	//&audio_source_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;
	int index = 1;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_usb_function *f;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += sprintf(buff, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	char aliases[256], *a;
	int err;
	int is_ffs;
	int ffs_enabled = 0;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (!name)
			continue;

		is_ffs = 0;
		strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
		a = aliases;

		while (a) {
			char *alias = strsep(&a, ",");
			if (alias && !strcmp(name, alias)) {
				is_ffs = 1;
				break;
			}
		}

		if (is_ffs) {
			if (ffs_enabled)
				continue;
			err = android_enable_function(dev, "ffs");
			if (err)
				pr_err("android_usb: Cannot enable ffs (%d)",
									err);
			else
				ffs_enabled = 1;
			continue;
		}

		err = android_enable_function(dev, name);
		if (err)
			pr_err("android_usb: Cannot enable '%s' (%d)",
							   name, err);
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	int enabled = 0;


	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);

	if (enabled && !dev->enabled) {
		/*
		 * Update values in composite driver's copy of
		 * device descriptor.
		 */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
		android_enable(dev);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}

#ifdef  USB_TEST_MODE_CONFIG
static ssize_t usb_test_mode_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	printk("usb_test_mode_cmd =%d\n", usb_test_mode_cmd);
	return sprintf(buf, "%d\n",usb_test_mode_cmd);   
	
}
static ssize_t usb_test_mode_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{

	usb_test_mode_cmd = *buff;
 	usb_test_mode_handle(usb_test_mode_cmd,pdev);
    	return size;
}
#endif
#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	return strlcpy(buffer, buf, sizeof(buffer));			\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

#ifdef  USB_TEST_MODE_CONFIG
static DEVICE_ATTR(usb_test_mode, S_IRWXO, usb_test_mode_show, usb_test_mode_store);
#endif
static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	#ifdef  USB_TEST_MODE_CONFIG
	&dev_attr_usb_test_mode,
	#endif
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	//pr_alert("[USB] %s ++ \n", __func__);

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	//pr_alert("[USB] %s ++ \n", __func__);

	/*
	 * Start disconnected. Userspace will connect the gadget once
	 * it is done configuring the functions.
	 */
	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strncpy(manufacturer_string, "浙江卡尔特汽车科技有限公司", sizeof(manufacturer_string)-1);
	strncpy(product_string, "车载音视频系统", sizeof(product_string) - 1);
	strncpy(serial_string, "2017120100002", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	printk("[USB] %d.\n", __LINE__);
	cancel_work_sync(&dev->work);
	printk("[USB] %d.\n", __LINE__);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
	.max_speed	= USB_SPEED_HIGH,
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	//pr_alert("[USB] %s ++ \n", __func__);
#if 0
	printk("[%s] ctrlrequest :0x%02x,0x%02x, 0x%02x, 0x%02x, 0x%02x.\n", __FUNCTION__, c->bRequest,
				c->bRequestType,
				le16_to_cpu(c->wIndex),
				le16_to_cpu(c->wValue),
				le16_to_cpu(c->wLength));
#endif
	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	//if (value < 0)
	//	value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		schedule_work(&dev->work);
#ifdef ENABLE_CARPLAY
	} else if (c->bRequest == USB_REQ_SET_INTERFACE &&
						!dev->carplayConnected) {
		 					list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
                                               if (0 == strcmp("ncm", f->name)) {
                                                                 dev->carplayConnected = 1;
                                                                 schedule_work(&dev->work);
                                                                 break;
                                               }
                            }

#endif
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	//pr_alert("[USB] %s ++ \n", __func__);

	composite_disconnect(gadget);
	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	//acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
#ifdef ENABLE_CARPLAY
	dev->carplayConnected = 0;
#endif
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static int android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	printk("[USB] %d.\n", __LINE__);
	while((attr = *attrs++))
		device_remove_file(dev->dev, attr);

	printk("[USB] %d.\n", __LINE__);
	device_destroy(android_class, dev->dev->devt);
}


void SendEANotify(int status)
{
	struct android_dev *dev = _android_dev;
	if(dev == NULL)
	{
		printk("SendEANotify err\n");
		return ;
	}
	
	if(status == 0)
	{
		dev->ea_status = eUSBConnectStatus_EASTOP;
		//dev->carplay = eUSBConnectStatus_EASTOP;
	}
	else if(status == 1)
	{
		dev->ea_status = eUSBConnectStatus_EASTART;
		//dev->carplay = eUSBConnectStatus_EASTART;
	}

	schedule_work(&dev->work);

	printk("SendEANotify end\n");
}
void Sendiap2Notify()
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	unsigned long flags;

	//spin_lock_irqsave(&cdev->lock, flags);
	
	//printk("!!!curent carplay status = %d\n", _carplay_status);
	//if(_carplay_status == eCarplay_NotSupportCarPlay) // this value depends on carplay.c(enum eCayPlay)
	//{
	//	dev->carplay = eUSBConnectStatus_IAPCONNECTED_NOTCARPLAY;
	//}
	//else 
	//{
		dev->carplay = eUSBConnectStatus_IAPCONNECTED_CARPLAY;
	//}
	
	schedule_work(&dev->work);
	//spin_unlock_irqrestore(&cdev->lock, flags);

	printk("Sendiap2Notify end\n");
}

static int __init init(void)
{
	struct android_dev *dev;
	int err;
	int nreal_len = 0;
	//Module version output
#ifndef ENABLE_CARPLAY
	MOD_VERSION_INFO("g_android",0,0,0);	
#endif
	//pr_alert("[USB] %s ++ \n", __func__);

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		class_destroy(android_class);
		kfree(dev);
		return err;
	}

	_android_dev = dev;

	memset(ea_store_string.ea_string, 0, sizeof(ea_store_string.ea_string));
	ea_store_string.nlen = 0;
	printk("pinitstring:%s, ninitstringlen=%d\n", pinitstring, ninitstringlen);
	nreal_len = min(sizeof(ea_store_string.ea_string), ninitstringlen);
	if(nreal_len != 0 && pinitstring != NULL)
	{
		memcpy(ea_store_string.ea_string, pinitstring, nreal_len);
		ea_store_string.nlen = nreal_len;
		printk("ea_usb_string :%s====\n", ea_store_string.ea_string);
	}
	

	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
	return usb_composite_probe(&android_usb_driver, android_bind);
}
module_init(init);

static void __exit cleanup(void)
{

	pr_alert("[USB] %s ++ \n", __func__);
	
	android_destroy_device(_android_dev);
	printk("[USB] %d.\n", __LINE__);
	usb_composite_unregister(&android_usb_driver);
	printk("[USB] %d.\n", __LINE__);
	class_destroy(android_class);
	kfree(_android_dev);
	ninitstringlen = 0;
	pinitstring = NULL;
	_android_dev = NULL;
}
module_exit(cleanup);
