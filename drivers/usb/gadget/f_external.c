/*
 * Gadget Driver for Android iAP2
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

//#define EA_BULK_BUFFER_SIZE           512
#define EA_BULK_BUFFER_SIZE           4096 * 2

static const char ea_shortname[] = "appleEA";
#define CMD_EA_SET_USB_STRING 0x01

struct ea_usb_string
{
	char ea_string[100];
	int nlen;
};

struct ea_usb_string ea_store_string;

static int ea_isCanTransport = 0;
static int ea_isEPEnable = 0; //default false;

struct ea_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	struct usb_request *tx_req;
	int rx_done;
	int tx_done;
};

static struct usb_interface_assoc_descriptor ea_iad_desc  = {
	.bLength =		sizeof ea_iad_desc,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	/* .bFirstInterface =	DYNAMIC, */
	.bInterfaceCount =	1,	/* EA interface count */
	.bFunctionClass =	0xFF, //Vendor-Specific interface
	.bFunctionSubClass =	0xF0,  //MFI Accessary
	.bFunctionProtocol =	0x01,  //External Accessory Nactive Transport
	/* .iFunction =		DYNAMIC */
};

static struct usb_interface_descriptor ea_interfaceS0_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	
	.bAlternateSetting		= 0,
	.bNumEndpoints          = 0,
	.bInterfaceClass        = 0xFF, //Vendor-specific interface
	.bInterfaceSubClass     = 0xF0, //Mfi Accessory
	.bInterfaceProtocol     = 0x01, //External Accessory Nactive Transport
};

static struct usb_interface_descriptor ea_interfaceS1_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	
	.bInterfaceNumber       = 0,
	.bAlternateSetting		= 1,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xF0,
	.bInterfaceProtocol     = 0x01, //External Accessory Nactive Transport
};

static struct usb_endpoint_descriptor ea_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor ea_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor ea_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor ea_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};


static struct usb_descriptor_header *fs_ea_descs[] = {
	(struct usb_descriptor_header *) &ea_iad_desc,
	
	(struct usb_descriptor_header *) &ea_interfaceS0_desc,
	(struct usb_descriptor_header *) &ea_interfaceS1_desc,
	(struct usb_descriptor_header *) &ea_fullspeed_in_desc,
	(struct usb_descriptor_header *) &ea_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_ea_descs[] = {
	(struct usb_descriptor_header *) &ea_iad_desc,
	
	(struct usb_descriptor_header *) &ea_interfaceS0_desc,
	(struct usb_descriptor_header *) &ea_interfaceS1_desc,
	(struct usb_descriptor_header *) &ea_highspeed_in_desc,
	(struct usb_descriptor_header *) &ea_highspeed_out_desc,
	NULL,
};

#define STRING_CTRL_IDX	0
#define STRING_MAC_IDX	1
#define STRING_DATA_IDX	2
#define STRING_IAD_IDX	3

static struct usb_string ea_string_defs[] = {
	[STRING_CTRL_IDX].s = "com.pandora.link.v1", //for pandora
	//[STRING_CTRL_IDX].s = "com.hsae.link", //for ALink
	//[STRING_CTRL_IDX].s = "com.baidu.CarLifeVehicleProtocol", //for carlife
	{  } /* end of list */
};

static struct usb_gadget_strings ea_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		ea_string_defs,
};

static struct usb_gadget_strings *ea_strings[] = {
	&ea_string_table,
	NULL,
};


/* temporary variable used between iap2_open() and iap2_gadget_bind() */
static struct ea_dev *_ea_dev;

static inline struct ea_dev *func_to_ea(struct usb_function *f)
{
	return container_of(f, struct ea_dev, function);
}


static struct usb_request *ea_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void ea_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int ea_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void ea_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static void ea_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct ea_dev *dev = _ea_dev;

	//printk("++++ea_complete_in \n");

	dev->tx_done = 1;
	//if (req->status != 0)
	//	dev->error = 1;

	wake_up(&dev->write_wq);
}

static void ea_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct ea_dev *dev = _ea_dev;

	//printk("++++ea_complete_out \n");

	dev->rx_done = 1;
	if (req->status != 0)
	{
		//dev->error = 1;
		printk("dev->error:%d\n", dev->error);
	}
		
	wake_up(&dev->read_wq);
}

static int ea_create_bulk_endpoints(struct ea_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;

	//ERROR(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	printk("ep in: %s\n", ep->name);
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	printk("ep out: %s\n", ep->name);
	DBG(cdev, "usb_ep_autoconfig for ea ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	req = ea_request_new(dev->ep_out, EA_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = ea_complete_out;
	dev->rx_req = req;

	req = ea_request_new(dev->ep_in, EA_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = ea_complete_in;
	dev->tx_req = req;

	return 0;

fail:
	printk(KERN_ERR "ea_bind() could not allocate requests\n");
	return -1;
}

static ssize_t ea_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct ea_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	//printk("ea_read(%d), dev->online:%d, dev->error:%d\n", count, dev->online, dev->error);
	if (!_ea_dev)
		return -ENODEV;

	if (count > EA_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (ea_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		pr_info("ea_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			ea_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		pr_info("ea_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} 
	else 
	{
		//pr_info("rx %p queue\n", req);
	}

	/* wait for a request to complete */
//	ret = wait_event_interruptible_timeout(dev->read_wq, dev->rx_done, msecs_to_jiffies(3000));
	ret = wait_event_interruptible(dev->read_wq, (dev->rx_done || !ea_isCanTransport));
    //printk("ea_readwait_event_interruptible end:%d | %d\n", ret, req->actual);
	if (ret < 0 || !ea_isCanTransport) {
		printk("R: ea_wait_event_interruptible!!!!!\n");
		if (ret != -ERESTARTSYS)
		{
			printk("R: ea_wait_event_interruptible ERESTARTSYS\n");
			dev->error = 1;
		}
        if (!ea_isCanTransport)
            r = -ENODEV;
        else
		    r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (!dev->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		//pr_info("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
		else
			r = xfer;

	} else
		r = -EIO;

done:
	ea_unlock(&dev->read_excl);
	//pr_info("ea_read returning %d\n", r);
	return r;
}

static ssize_t ea_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct ea_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
	int i;
	//pr_info("ea_write(%d)\n",count);

	if (!_ea_dev)
	{
		printk("ea_write1\n");
		return -ENODEV;
	}

	if (ea_lock(&dev->write_excl))
	{
		printk("ea_write2\n");
		return -EBUSY;
	}

	while (count > 0) {
		if (dev->error) {
			printk("ea_write dev->error\n");
			r = -EIO;
			break;
		}
	
		/* get an idle tx request to use */
		req = dev->tx_req;

		/* we will block until we're online */
			while (!(dev->online || dev->error)) {
				pr_info("ea_write: waiting for online state\n");
				ret = wait_event_interruptible(dev->write_wq,
						(dev->online || dev->error));
				if (ret < 0) {
					ea_unlock(&dev->write_excl);
					return ret;
				}
			}
			if (dev->error) {
				r = -EIO;
				ea_unlock(&dev->write_excl);
				return r;

			}
			
		if (req != 0) {
			if (count > EA_BULK_BUFFER_SIZE)
			{
				xfer = EA_BULK_BUFFER_SIZE;
				printk("ea_write, over number !!\n");
			}
			else
				xfer = count;
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}
	
			req->length = xfer;
			dev->tx_done = 0;
				
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				printk("ea_write: xfer error %d\n", ret);
				dev->error = 1;
				r = -EIO;
				break;
			}
	
			buf += xfer;
			count -= xfer;
	
            ret = wait_event_interruptible(dev->write_wq, (dev->tx_done || !ea_isCanTransport));
            if (ret < 0) {
                dev->error = 1;
                r = ret;
                usb_ep_dequeue(dev->ep_in, req);
            }
			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}
	
	ea_unlock(&dev->write_excl);
	//printk("ea_write returning %d\n", r);
	return r;
}


ssize_t ea_inner_write(const char *buf,  size_t count)
{
	struct ea_dev *dev = _ea_dev;
	struct usb_request *req = 0;
	int r = count;
	int ret;
	pr_info("ea_inner_write\n");

	if (!_ea_dev)
		return -ENODEV;

	if (count > EA_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (ea_lock(&dev->write_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		pr_info("ea_inner_write: waiting for online state\n");
		ret = wait_event_interruptible(dev->write_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			ea_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error) {
		r = -EIO;
		goto done;
	}
	
	/* queue a request */
	req = dev->tx_req;
	
	if (!memcpy(req->buf, buf, count)) {
		r = -EFAULT;
		goto done;
	}
	req->length = count;
	dev->tx_done = 0;

	ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
	if (ret < 0) {
		printk("ea_inner_write: failed to queue req %p (%d),dev->error=1\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_info("===>ea_inner_write: tx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible_timeout(dev->write_wq, dev->tx_done, msecs_to_jiffies(3000));
	if (!ret) {
		printk("W: wait_event_interruptible_timeout!!!!!\n");
		r = -EAGAIN;
		usb_ep_dequeue(dev->ep_in, req);
		goto done;
	}
	
	pr_info("ea_inner_write end\n");

done:
	ea_unlock(&dev->write_excl);
	pr_info("ea_inner_write returning %d\n", r);
	return r;
}

ssize_t ea_inner_read(char *buf, size_t count)
{
	struct ea_dev *dev = _ea_dev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	pr_info("ea_inner_read(%d)\n", count);
	if (!_ea_dev)
		return -ENODEV;

	if (count > EA_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (ea_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		//pr_info("ea_inner_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			ea_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		pr_info("ea_inner_read: failed to queue req %p (%d),dev->error=1\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_info("<===ea_inner_read: rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible_timeout(dev->read_wq, dev->rx_done, msecs_to_jiffies(3000));
	if (!ret) {
		printk("R: ea_inner_read wait_event_interruptible_timeout!!!!!,dev->error=1\n");
		dev->error = 1;
		r= -EAGAIN;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (!dev->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		pr_info("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (!memcpy(buf, req->buf, xfer))
			r = -EFAULT;
		else
			r = xfer;

	} else
		r = -EIO;

done:
	ea_unlock(&dev->read_excl);
	pr_info("ea_inner_read returning %d\n", r);
	return r;
}


static int ea_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "ea_open\n");
	if (!_ea_dev)
		return -ENODEV;

	if (ea_lock(&_ea_dev->open_excl))
		return -EBUSY;

	fp->private_data = _ea_dev;

	/* clear the error latch */
	_ea_dev->error = 0;
	printk(KERN_INFO "ea_open_suc\n");

	return 0;
}

static int ea_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "ea_release\n");
	ea_unlock(&_ea_dev->open_excl);
	return 0;
}

static long ea_ioctl (struct file * fp , unsigned int cmd, unsigned long arg)
{
	if(fp == NULL )
	{
		return -1;
	}
	
	int nstring_len = 0;

	if(cmd == CMD_EA_SET_USB_STRING)
	{
		// set USB string after insmod g_android.ko, please don't set it after usb enum
		 memset(ea_store_string.ea_string, 0, sizeof(ea_store_string.ea_string));
		 ea_store_string.nlen = 0;
		 get_user(ea_store_string.nlen, &(((struct ea_usb_string* __user)arg)->nlen));
		 nstring_len = min(ea_store_string.nlen, sizeof(ea_store_string.ea_string));
		 copy_from_user(ea_store_string.ea_string, ((struct ea_usb_string* __user)arg)->ea_string, nstring_len);
		 printk("set ea_usb_string:%s \n", ea_store_string.ea_string);
	}

	return 0;
}


/* file operations for iAP2 device /dev/iap2 */
static struct file_operations ea_fops = {
	.owner = THIS_MODULE,
	.read = ea_read,
	.write = ea_write,
	.open = ea_open,
	.unlocked_ioctl = ea_ioctl,
	.release = ea_release,
};

static struct miscdevice ea_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = ea_shortname,
	.fops = &ea_fops,
};


static int
ea_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct ea_dev	*dev = func_to_ea(f);
	int			id;
	int			ret;
	printk("enter ea_function_bind\n");
	dev->cdev = cdev;
	//INFO(cdev, "ea_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	ea_interfaceS0_desc.bInterfaceNumber = id;
	ea_interfaceS1_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = ea_create_bulk_endpoints(dev, &ea_fullspeed_in_desc,
			&ea_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		ea_highspeed_in_desc.bEndpointAddress =
			ea_fullspeed_in_desc.bEndpointAddress;
		ea_highspeed_out_desc.bEndpointAddress =
			ea_fullspeed_out_desc.bEndpointAddress;
	}

	//INFO(cdev, "%s speed %s: IN/%s, OUT/%s\n",
	//		gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
	//		f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
ea_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct ea_dev	*dev = func_to_ea(f);
	struct usb_request *req;
	printk("enter ea_function_unbind\n");
	dev->online = 0;
	dev->error = 1;
	ea_isCanTransport = 0;
	printk("12dev->error:%d,online:0\n", dev->error);
	wake_up(&dev->read_wq);

	ea_request_free(dev->rx_req, dev->ep_out);
	ea_request_free(dev->tx_req, dev->ep_in);
	ea_isEPEnable = 0;
}
extern void SendEANotify();
static int ea_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct ea_dev	*dev = func_to_ea(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	printk( "ea_function_set_alt intf: %d alt: %d\n", intf, alt);
	

	if(alt > 1)
	{
		printk("ea_function_set_alt err, alt can not > 1, dev->error=1\n");
		dev->error = 1;
		return -EINVAL;
	}

	if(alt == 0)
	{
		dev->online = 1;
		ea_isCanTransport = 0;
		//stop ea transport
		SendEANotify(alt); 
		printk("dev->online=1\n");
        wake_up(&dev->read_wq);
		return 0;
	}
	
	printk("ea_isEPEnable=%d\n", ea_isEPEnable);
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret)
	{
		printk("config_ep_by_speed, in, ret=%d\n", ret);
		return ret;
	}
	
	if(ea_isEPEnable == 0)
	{
		ret = usb_ep_enable(dev->ep_in);
		if (ret)
		{
			printk("usb_ep_enable ,in,ret=%d\n", ret);
			return ret;	
		}
	}
	
		
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret)
	{
		printk("config_ep_by_speed, out, ret=%d\n", ret);
		return ret;
	}
	
	if(ea_isEPEnable == 0)
	{
		ret = usb_ep_enable(dev->ep_out);
		if (ret) {
			printk("usb_ep_enable, out, ret = %d\n",ret);
			usb_ep_disable(dev->ep_in);
			return ret;
		}
	}	
	
	dev->online = 1;
	ea_isCanTransport = 1;
	ea_isEPEnable = 1;
	SendEANotify(alt);
	printk("11dev->online=1\n");
	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void ea_function_disable(struct usb_function *f)
{
	
	struct ea_dev	*dev = func_to_ea(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	printk("enter ea_function_disable\n");
	DBG(cdev, "ea_function_disable cdev %p\n", cdev);
	dev->online = 0;
	dev->error = 1;
	ea_isCanTransport = 0;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	ea_isEPEnable = 0;
	printk("13dev->error:%d,dev->online=0\n", dev->error);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int ea_bind_config(struct usb_configuration *c)
{
	struct ea_dev *dev = _ea_dev;
	int		status;

	printk(KERN_INFO "ea_bind_config\n");

	/* maybe allocate device-global string IDs */
	if (ea_string_defs[0].id == 0) {

		/* control interface label */
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		ea_string_defs[STRING_CTRL_IDX].id = status;
		ea_interfaceS0_desc.iInterface = status;
		ea_interfaceS1_desc.iInterface = status;
	}

	if(ea_store_string.nlen != 0)
	{
		ea_string_defs[STRING_CTRL_IDX].s = ea_store_string.ea_string;
		//ea_strings[0]->strings->s = ea_store_string.ea_string;
	}

	printk("ea_strings:%s===\n", ea_strings[0]->strings->s);

	dev->cdev = c->cdev;
	dev->function.name = "appleEA";
	dev->function.strings = ea_strings;
	dev->function.descriptors = fs_ea_descs;
	dev->function.hs_descriptors = hs_ea_descs;
	dev->function.bind = ea_function_bind;
	dev->function.unbind = ea_function_unbind;
	dev->function.set_alt = ea_function_set_alt;
	dev->function.disable = ea_function_disable;

	return usb_add_function(c, &dev->function);
}

static int ea_setup(void)
{
	struct ea_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	printk("ea_setup  !\n");

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	_ea_dev = dev;
	
	//memset(ea_store_string.ea_string, 0, sizeof(ea_store_string.ea_string));
	//ea_store_string.nlen = 0;
	
	ret = misc_register(&ea_device);
	if (ret)
		goto err;

	return 0;

err:
	kfree(dev);
	printk(KERN_ERR "ea gadget driver failed to initialize\n");
	return ret;
}

static void ea_cleanup(void)
{
	printk("enter ea_cleanup\n");
	misc_deregister(&ea_device);

	kfree(_ea_dev);
	_ea_dev = NULL;
}

