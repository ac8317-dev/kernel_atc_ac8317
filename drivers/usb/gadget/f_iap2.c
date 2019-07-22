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

#define IAP2_BULK_BUFFER_SIZE           512

int DeviceOpenStatus = 0; //0:close, 1:open

static const char iap2_shortname[] = "iap2";

struct iap2_dev {
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

static struct usb_interface_descriptor iap2_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xF0,
	.bInterfaceProtocol     = 0,
};

static struct usb_endpoint_descriptor iap2_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor iap2_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor iap2_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor iap2_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};


static struct usb_descriptor_header *fs_iap2_descs[] = {
	(struct usb_descriptor_header *) &iap2_interface_desc,
	(struct usb_descriptor_header *) &iap2_fullspeed_in_desc,
	(struct usb_descriptor_header *) &iap2_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_iap2_descs[] = {
	(struct usb_descriptor_header *) &iap2_interface_desc,
	(struct usb_descriptor_header *) &iap2_highspeed_in_desc,
	(struct usb_descriptor_header *) &iap2_highspeed_out_desc,
	NULL,
};

#define STRING_CTRL_IDX	0
#define STRING_MAC_IDX	1
#define STRING_DATA_IDX	2
#define STRING_IAD_IDX	3

static struct usb_string iap_string_defs[] = {
	[STRING_CTRL_IDX].s = "iAP Interface",
	{  } /* end of list */
};

static struct usb_gadget_strings iap_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		iap_string_defs,
};

static struct usb_gadget_strings *iap2_strings[] = {
	&iap_string_table,
	NULL,
};


/* temporary variable used between iap2_open() and iap2_gadget_bind() */
static struct iap2_dev *_iap2_dev;

static inline struct iap2_dev *func_to_iap2(struct usb_function *f)
{
	return container_of(f, struct iap2_dev, function);
}


static struct usb_request *iap2_request_new(struct usb_ep *ep, int buffer_size)
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

static void iap2_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int iap2_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void iap2_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static void iap2_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct iap2_dev *dev = _iap2_dev;

	printk("++++iap2_complete_in \n");

	dev->tx_done = 1;
	//if (req->status != 0)
	//	dev->error = 1;

	wake_up(&dev->write_wq);
}

static void iap2_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct iap2_dev *dev = _iap2_dev;

	printk("++++iap2_complete_out \n");

	dev->rx_done = 1;
	if (req->status != 0 && req->status != -ECONNRESET)
		dev->error = 1;

	wake_up(&dev->read_wq);
}

static int iap2_create_bulk_endpoints(struct iap2_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;

	ERROR(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for iap2 ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	req = iap2_request_new(dev->ep_out, IAP2_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = iap2_complete_out;
	dev->rx_req = req;

	req = iap2_request_new(dev->ep_in, IAP2_BULK_BUFFER_SIZE);
	if (!req)
		goto fail;
	req->complete = iap2_complete_in;
	dev->tx_req = req;

	return 0;

fail:
	printk(KERN_ERR "iap2_bind() could not allocate requests\n");
	return -1;
}

static ssize_t iap2_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct iap2_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	pr_info("iap2_read(%d)\n", count);
	if (!_iap2_dev)
		return -ENODEV;

	if (count > IAP2_BULK_BUFFER_SIZE)
		return -EINVAL;

	if(DeviceOpenStatus == 0)
	{
		  printk("error,iap2 read device now is close, please open it\n");	
		  return -ENODEV;	
	}

	if (iap2_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		pr_info("iap2_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
				(dev->online || dev->error || DeviceOpenStatus == 0));
		if (ret < 0) {
			iap2_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (dev->error || DeviceOpenStatus == 0) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;
	if(DeviceOpenStatus == 0 || dev->error == 1 || dev->online == 0)
		{
				printk("error,iap2 read\n");
				goto done;
		} 
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		pr_info("iap2_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_info("rx %p queue\n", req);
	}

	/* wait for a request to complete */
//	ret = wait_event_interruptible_timeout(dev->read_wq, dev->rx_done, msecs_to_jiffies(3000));
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done || (DeviceOpenStatus == 0));

	if (ret < 0) {
		printk("R: wait_event_interruptible!!!!!\n");
		if (ret != -ERESTARTSYS)
		{
		printk("R: wait_event_interruptible ERESTARTSYS\n");
			dev->error = 1;
		}
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (!dev->error) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0)
			goto requeue_req;

		pr_info("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
		else
			r = xfer;

	} else
		r = -EIO;

done:
	iap2_unlock(&dev->read_excl);
	pr_info("iap2_read returning %d\n", r);
	return r;
}

static ssize_t iap2_write(struct file *fp, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct iap2_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
	int i;
	printk("iap2_write\n");

	if (!_iap2_dev)
	{
		printk("error,iap2_write1\n");
		return -ENODEV;
	}
	if(DeviceOpenStatus == 0)
	{
		  printk("error,iap2 device now is close, please open it\n");	
		  return -ENODEV;	
	}
	
	if (iap2_lock(&dev->write_excl))
	{
		printk("error,iap2_write2\n");
		return -EBUSY;
	}

	while (count > 0) {
		if (dev->error) {
			printk("error,iap2_write dev->error\n");
			r = -EIO;
			break;
		}
	
			/* get an idle tx request to use */
			req = dev->tx_req;

			/* we will block until we're online */
				while (!(dev->online || dev->error)) {
					pr_info("iap2_write: waiting for online state\n");
					ret = wait_event_interruptible(dev->write_wq,
							(dev->online || dev->error || DeviceOpenStatus == 0));
					if (ret < 0) {
						iap2_unlock(&dev->write_excl);
						printk("error,iap2_write -- 3\n");
						return ret;
					}
				}
				if (dev->error || (DeviceOpenStatus == 0)) {
					r = -EIO;
					iap2_unlock(&dev->write_excl);
					printk("error,iap2_write -- 4\n");
					return r;

				}
				
			if (req != 0) {
				if (count > IAP2_BULK_BUFFER_SIZE)
					xfer = IAP2_BULK_BUFFER_SIZE;
				else
					xfer = count;
				if (copy_from_user(req->buf, buf, xfer)) {
					r = -EFAULT;
					break;
				}
	
				req->length = xfer;
				dev->tx_done = 0;
				
				if(DeviceOpenStatus == 0)
					{
							printk("error,iap2 device close\n");
							iap2_unlock(&dev->write_excl);
							r = -EIO;
							return r;
					}
					
				ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
				if (ret < 0) {
					printk("error,iap2_write: xfer error %d\n", ret);
					dev->error = 1;
					r = -EIO;
					break;
				}
	
				buf += xfer;
				count -= xfer;
	
	ret = wait_event_interruptible(dev->write_wq, dev->tx_done || (DeviceOpenStatus == 0));
		if (ret < 0) {
			dev->error = 1;
			r = ret;
			usb_ep_dequeue(dev->ep_in, req);
			printk("error,iap2_write 5\n");
		}
				/* zero this so we don't try to free it on error exit */
				req = 0;
			}
		}
	
		iap2_unlock(&dev->write_excl);
		printk("iap2_write returning %d\n", r);
		//printk("error,iap2_write ret=%d\n", r);
		return r;

}


ssize_t iap2_inner_write(const char *buf,  size_t count)
{
	struct iap2_dev *dev = _iap2_dev;
	struct usb_request *req = 0;
	int r = count;
	int ret;
	pr_info("iap2_inner_write\n");

	if (!_iap2_dev)
		return -ENODEV;

	if (count > IAP2_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (iap2_lock(&dev->write_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		pr_info("iap2_inner_write: waiting for online state\n");
		ret = wait_event_interruptible(dev->write_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			iap2_unlock(&dev->read_excl);
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
		pr_info("iap2_inner_write: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_info("===>iap2_inner_write: tx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible_timeout(dev->write_wq, dev->tx_done, msecs_to_jiffies(3000));
	if (!ret) {
		printk("W: wait_event_interruptible_timeout!!!!!\n");
		r = -EAGAIN;
		usb_ep_dequeue(dev->ep_in, req);
		goto done;
	}
	
	pr_info("iap2_inner_write end\n");

done:
	iap2_unlock(&dev->write_excl);
	pr_info("iap2_inner_write returning %d\n", r);
	return r;
}

ssize_t iap2_inner_read(char *buf, size_t count)
{
	struct iap2_dev *dev = _iap2_dev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	pr_info("iap2_inner_read(%d)\n", count);
	if (!_iap2_dev)
		return -ENODEV;

	if (count > IAP2_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (iap2_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(dev->online || dev->error)) {
		//pr_info("iap2_inner_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
				(dev->online || dev->error));
		if (ret < 0) {
			iap2_unlock(&dev->read_excl);
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
		pr_info("iap2_inner_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		pr_info("<===iap2_inner_read: rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible_timeout(dev->read_wq, dev->rx_done, msecs_to_jiffies(3000));
	if (!ret) {
		printk("R: wait_event_interruptible_timeout!!!!!\n");
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
	iap2_unlock(&dev->read_excl);
	pr_info("iap2_inner_read returning %d\n", r);
	return r;
}


static int iap2_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "iap2_open\n");
	if (!_iap2_dev)
		return -ENODEV;

	if (iap2_lock(&_iap2_dev->open_excl))
		return -EBUSY;

	fp->private_data = _iap2_dev;

	/* clear the error latch */
	_iap2_dev->error = 0;
	DeviceOpenStatus = 1;
	printk(KERN_INFO "iap2_open_suc\n");

	return 0;
}

static int iap2_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "iap2_release\n");
	iap2_unlock(&_iap2_dev->open_excl);
	DeviceOpenStatus = 0;
	return 0;
}

/* file operations for iAP2 device /dev/iap2 */
static struct file_operations iap2_fops = {
	.owner = THIS_MODULE,
	.read = iap2_read,
	.write = iap2_write,
	.open = iap2_open,
	.release = iap2_release,
};

static struct miscdevice iap2_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = iap2_shortname,
	.fops = &iap2_fops,
};


static int
iap2_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct iap2_dev	*dev = func_to_iap2(f);
	int			id;
	int			ret;
	printk("iap2_function_bind\n");

	dev->cdev = cdev;
	INFO(cdev, "iap2_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	iap2_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = iap2_create_bulk_endpoints(dev, &iap2_fullspeed_in_desc,
			&iap2_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		iap2_highspeed_in_desc.bEndpointAddress =
			iap2_fullspeed_in_desc.bEndpointAddress;
		iap2_highspeed_out_desc.bEndpointAddress =
			iap2_fullspeed_out_desc.bEndpointAddress;
	}

	INFO(cdev, "%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
iap2_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct iap2_dev	*dev = func_to_iap2(f);
	struct usb_request *req;
	printk("iap2_function_unbind\n");

	dev->online = 0;
	dev->error = 1;

	wake_up(&dev->read_wq);

	iap2_request_free(dev->rx_req, dev->ep_out);
	iap2_request_free(dev->tx_req, dev->ep_in);
}
extern void Sendiap2Notify();
static int iap2_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct iap2_dev	*dev = func_to_iap2(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;
	printk("iap2_function_set_alt");

	ERROR(cdev, "iap2_function_set_alt intf: %d alt: %d\n", intf, alt);
	
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret)
		return ret;

	ret = usb_ep_enable(dev->ep_in);
	if (ret)
		return ret;

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret)
		return ret;

	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		printk("iap2_function_set_alt ret = %d\n",ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->online = 1;
	Sendiap2Notify();
	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}

static void iap2_function_disable(struct usb_function *f)
{
	struct iap2_dev	*dev = func_to_iap2(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG(cdev, "iap2_function_disable cdev %p\n", cdev);
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int iap2_bind_config(struct usb_configuration *c)
{
	struct iap2_dev *dev = _iap2_dev;
	int		status;

	printk(KERN_INFO "iap2_bind_config\n");

	/* maybe allocate device-global string IDs */
	if (iap_string_defs[0].id == 0) {

		/* control interface label */
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		iap_string_defs[STRING_CTRL_IDX].id = status;
		iap2_interface_desc.iInterface = status;
	}

	dev->cdev = c->cdev;
	dev->function.name = "iap2";
	dev->function.strings = iap2_strings;
	dev->function.descriptors = fs_iap2_descs;
	dev->function.hs_descriptors = hs_iap2_descs;
	dev->function.bind = iap2_function_bind;
	dev->function.unbind = iap2_function_unbind;
	dev->function.set_alt = iap2_function_set_alt;
	dev->function.disable = iap2_function_disable;

	return usb_add_function(c, &dev->function);
}

static int iap2_setup(void)
{
	struct iap2_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	printk("iap2_setup  !\n");

	spin_lock_init(&dev->lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->tx_idle);

	_iap2_dev = dev;

	ret = misc_register(&iap2_device);
	if (ret)
		goto err;

	return 0;

err:
	kfree(dev);
	printk(KERN_ERR "iap2 gadget driver failed to initialize\n");
	return ret;
}

static void iap2_cleanup(void)
{
	misc_deregister(&iap2_device);

	kfree(_iap2_dev);
	_iap2_dev = NULL;
}

