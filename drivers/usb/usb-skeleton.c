/*
 * USB Skeleton driver - 2.2
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * but has been rewritten to be easier to read and use.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>


/* Define these values to match your devices */
#define USB_SKEL_VENDOR_ID	0xfff0
#define USB_SKEL_PRODUCT_ID	0xfff0

/* table of devices that work with this driver */
// 当有一个设备接到集线器时，usb子系统就会检查这个设备的vendor ID和product ID，
// 如果它们的值是0xfff0时，那么子系统就会调用这个skeleton模块作为设备的驱动
static const struct usb_device_id skel_table[] = {
	{ USB_DEVICE(USB_SKEL_VENDOR_ID, USB_SKEL_PRODUCT_ID) },
	{ }					/* Terminating entry */
};

//  将xx_driver_ids结构输出到用户空间，这样模块加载系统在加载模块时，就知道了什么模块对应什么硬件设备
MODULE_DEVICE_TABLE(usb, skel_table);  // 第一个参数 是设备的类型 usb 或者  pci   
                //  MODULE_DEVICE_TABLE（设备类型，设备表）   第二个参数是一个 设备表 表的最后一个元素是空 用于表示结束               

/* Get a minor range for your devices from the usb maintainer */
#define USB_SKEL_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

/* Structure to hold all of our device specific stuff usb-skel */
struct usb_skel {
	struct usb_device	*udev;			/* the usb device for this device   描述 usb 设备的结构体 */
	struct usb_interface	*interface;		/* the interface for this device  一个接口 */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress  用于并发访问控制的信号量*/
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct urb		*bulk_in_urb;		/* the urb to read data with */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data  接收数据的缓冲 */
	size_t			bulk_in_size;		/* the size of the receive buffer 缓冲的大小 */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint  批量输入的端口地址 */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint 批量输出的端口地址 */
	int			errors;			/* the last request tanked */
	bool			ongoing_read;		/* a read is going on */
	bool			processed_urb;		/* indicates we haven't processed the urb */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;  /* 内核的引用计数 */
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct completion	bulk_in_completion;	/* to wait for an ongoing read */
};
#define to_skel_dev(d) container_of(d, struct usb_skel, kref)

static struct usb_driver skel_driver;
static void skel_draw_down(struct usb_skel *dev);

static void skel_delete(struct kref *kref)
{
	struct usb_skel *dev = to_skel_dev(kref);

	usb_free_urb(dev->bulk_in_urb);
	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int skel_open(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&skel_driver, subminor);
	if (!interface) {
		err("%s - error, can't find device for minor %d",
		     __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* lock the device to allow correctly handling errors
	 * in resumption */
	mutex_lock(&dev->io_mutex);

	retval = usb_autopm_get_interface(interface);
	if (retval)
		goto out_err;

	/* save our object in the file's private structure */
	file->private_data = dev;
	mutex_unlock(&dev->io_mutex);

exit:
	return retval;
}

static int skel_release(struct inode *inode, struct file *file)
{
	struct usb_skel *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, skel_delete);
	return 0;
}

static int skel_flush(struct file *file, fl_owner_t id)
{
	struct usb_skel *dev;
	int res;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	skel_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

static void skel_read_bulk_callback(struct urb *urb)
{
	struct usb_skel *dev;

	dev = urb->context;

	spin_lock(&dev->err_lock);
	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("%s - nonzero write bulk status received: %d",
			    __func__, urb->status);

		dev->errors = urb->status;
	} else {
		dev->bulk_in_filled = urb->actual_length;
	}
	dev->ongoing_read = 0;
	spin_unlock(&dev->err_lock);

	complete(&dev->bulk_in_completion);
}

//  //  urb  数据传送的细节  
static int skel_do_read_io(struct usb_skel *dev, size_t count)   
{
	int rv;

	/* prepare a read */
	usb_fill_bulk_urb(dev->bulk_in_urb,     //   装车 并告诉司机 货要送到哪  
			dev->udev,
			usb_rcvbulkpipe(dev->udev,
				dev->bulk_in_endpointAddr),
			dev->bulk_in_buffer,
			min(dev->bulk_in_size, count),
			skel_read_bulk_callback,        //   是一个完成回调函数，而他做的主要事情就是检查数据传输状态和释放urb
			dev);
	/* tell everybody to leave the URB alone */
	spin_lock_irq(&dev->err_lock);
	dev->ongoing_read = 1;
	spin_unlock_irq(&dev->err_lock);

	/* do it */
	rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);  //  开始传输  
	if (rv < 0) {
		err("%s - failed submitting read urb, error %d",
			__func__, rv);
		dev->bulk_in_filled = 0;
		rv = (rv == -ENOMEM) ? rv : -EIO;
		spin_lock_irq(&dev->err_lock);
		dev->ongoing_read = 0;
		spin_unlock_irq(&dev->err_lock);
	}

	return rv;
}


// 如果数据量不大的话  可以不用 卡车来传输数据  系统还提供了一种不用 urb 的方式  

// skel 的读函数 就是采用了这种方式 实现的   使用了usb_bulk_msg来传送数据  
static ssize_t skel_read(struct file *file, char *buffer, size_t count,
			 loff_t *ppos)
{
	struct usb_skel *dev;
	int rv;
	bool ongoing_io;

	dev = file->private_data;

	/* if we cannot read at all, return EOF */
	if (!dev->bulk_in_urb || !count)
		return 0;

	/* no concurrent readers */
	rv = mutex_lock_interruptible(&dev->io_mutex);
	if (rv < 0)
		return rv;

	if (!dev->interface) {		/* disconnect() was called */
		rv = -ENODEV;
		goto exit;
	}

	/* if IO is under way, we must not touch things */
retry:
	spin_lock_irq(&dev->err_lock);
	ongoing_io = dev->ongoing_read;
	spin_unlock_irq(&dev->err_lock);

	if (ongoing_io) {
		/* nonblocking IO shall not wait */
		if (file->f_flags & O_NONBLOCK) {
			rv = -EAGAIN;
			goto exit;
		}
		/*
		 * IO may take forever
		 * hence wait in an interruptible state
		 */
		rv = wait_for_completion_interruptible(&dev->bulk_in_completion);
		if (rv < 0)
			goto exit;
		/*
		 * by waiting we also semiprocessed the urb
		 * we must finish now
		 */
		dev->bulk_in_copied = 0;
		dev->processed_urb = 1;
	}

	if (!dev->processed_urb) {
		/*
		 * the URB hasn't been processed
		 * do it now
		 */
		wait_for_completion(&dev->bulk_in_completion);
		dev->bulk_in_copied = 0;
		dev->processed_urb = 1;
	}

	/* errors must be reported */
	rv = dev->errors;
	if (rv < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		rv = (rv == -EPIPE) ? rv : -EIO;
		/* no data to deliver */
		dev->bulk_in_filled = 0;
		/* report it */
		goto exit;
	}

	/*
	 * if the buffer is filled we may satisfy the read
	 * else we need to start IO
	 */

	if (dev->bulk_in_filled) {
		/* we had read data */
		size_t available = dev->bulk_in_filled - dev->bulk_in_copied;
		size_t chunk = min(available, count);

		if (!available) {
			/*
			 * all data has been used
			 * actual IO needs to be done
			 */
			rv = skel_do_read_io(dev, count);    //  urb  数据传送的细节  
			if (rv < 0)
				goto exit;
			else
				goto retry;
		}
		/*
		 * data is available
		 * chunk tells us how much shall be copied
		 */

		if (copy_to_user(buffer,
				 dev->bulk_in_buffer + dev->bulk_in_copied,
				 chunk))
			rv = -EFAULT;
		else
			rv = chunk;

		dev->bulk_in_copied += chunk;

		/*
		 * if we are asked for more than we have,
		 * we start IO but don't wait
		 */
		if (available < count)
			skel_do_read_io(dev, count - chunk);
	} else {
		/* no data in the buffer */
		rv = skel_do_read_io(dev, count);
		if (rv < 0)
			goto exit;
		else if (!(file->f_flags & O_NONBLOCK))
			goto retry;
		rv = -EAGAIN;
	}
exit:
	mutex_unlock(&dev->io_mutex);
	return rv;
}


// 其作用就是 检查数据的传输状态 和 释放 urb   
static void skel_write_bulk_callback(struct urb *urb)
{
	struct usb_skel *dev;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("%s - nonzero write bulk status received: %d",
			    __func__, urb->status);

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;   //  
		spin_unlock(&dev->err_lock);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}



//  skel 的写函数 选用卡车 urb 的方式来传输数据  
static ssize_t skel_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct usb_skel *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min(count, (size_t)MAX_TRANSFER);

	dev = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/*
	 * limit the number of URBs in flight to stop a user from using up all
	 * RAM
	 */
	if (!(file->f_flags & O_NONBLOCK)) {
		if (down_interruptible(&dev->limit_sem)) {
			retval = -ERESTARTSYS;
			goto exit;
		}
	} else {
		if (down_trylock(&dev->limit_sem)) {
			retval = -EAGAIN;
			goto exit;
		}
	}

	spin_lock_irq(&dev->err_lock);
	retval = dev->errors;
	if (retval < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (!dev->interface) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */ //  完成了urb 系统callbak和我们自己的skel_write_bulk_callback之间的联系
	usb_fill_bulk_urb(urb, dev->udev, //  注意skel_write_bulk_callback是中断方式，所以要注意时间不能太久
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, skel_write_bulk_callback, dev);  //  skel_write_bulk_callback  就是一个 urb 的完成回调函数 
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);
	if (retval) {
		err("%s - failed submitting write urb, error %d", __func__,
		    retval);
		goto error_unanchor;
	}

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);


	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);

exit:
	return retval;
}


  //  这部分对应到我们自定义的 读写 需要我们自己实现  
static const struct file_operations skel_fops = {
	.owner =	THIS_MODULE,
	.read =		skel_read,
	.write =	skel_write,
	.open =		skel_open,
	.release =	skel_release,
	.flush =	skel_flush,
	.llseek =	noop_llseek,
};



/*   */
/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 
 */ //  允许devfs用户访问我们的设备 所提供的接口  
static struct usb_class_driver skel_class = {
	.name =		"skel%d",
	.fops =		&skel_fops,    //  真正完成对设备  IO 操作的 函数   
	.minor_base =	USB_SKEL_MINOR_BASE,  // 次设备号的基准值  
};

static int skel_probe(struct usb_interface *interface,  // 通过接口来划分功能 一般会是 0 号接口的默认的 0号设置
		      const struct usb_device_id *id)  // 设备的  ID  描述符   vendor ID   production ID 
{
	struct usb_skel *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err("Out of memory");
		goto error;
	}
	kref_init(&dev->kref); //初始化 kref 并 设置 kref = 1  
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);
	init_completion(&dev->bulk_in_completion);
	//  usb_get_dev() 来增加对 usb_device 的引用计数  kref 是对模块的引用计数   
	dev->udev = usb_get_dev(interface_to_usbdev(interface));  // 通过 usb_interface 来得到 该接口所在设备的设备描述结构体   
	dev->interface = interface;   //  要得到一个usb_device只要用interface_to_usbdev就够了  但因为要增加对该usb_device的引用计数，我们应该在做一个usb_get_dev的操作，
	                              //  来增加引用计数，并在释放设备时用usb_put_dev来减少引用计数

    /* 在得到 usb_device 之后 我们要对我们自定义的 usb_skel 的各个状态跟资源做初始化
       这部分工作的主要任务是向 usb_skel ???  core  注册该 usb 设备的端点 */

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;     /* desc 用于描述 interface 的一些属性   */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {  /* bNumEndpoints 代表了接口的端点数 */
		endpoint = &iface_desc->endpoint[i].desc;  /* 遍历所有的端点  */

		if (!dev->bulk_in_endpointAddr &&
		    usb_endpoint_is_bulk_in(endpoint)) { /* 检查每一个端点的类型和方向 并注册到 usb_skel 中 */
			/* we found a bulk in endpoint */
			buffer_size = usb_endpoint_maxp(endpoint);
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Could not allocate bulk_in_buffer");
				goto error;
			}
			dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);  // 要运货首先要有车  即创建 urb   // 第一个参数为 0 则表示不是等时包  若为等时包 isoc_packets 
			if (!dev->bulk_in_urb) {
				err("Could not allocate bulk_in_urb");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Could not find both bulk-in and bulk-out endpoints");
		goto error;
	}

    //　接下来的工作是向系统注册一些以后会用到的信息
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);  /* 首先向内核注册一个data 即usb_skel结构的dev 这个data 可以用 usb_get_intfdata() 得到 */
 
	/* we can register the device now, as it is ready */  //  　然后我们向这个interface注册一个skel_class结构   事实上它才是定义 真正完成对设备IO操作的函数
	/* 因为每个 usb 设备可以有几个 interface  每个 interface 定义的 IO 操作可能不一样 所以向系统注册的usb_class_driver 
	   要求注册到 每一个 interface 而不是device  */
	retval = usb_register_dev(interface, &skel_class); /*向这个接口注册 skel_class */  // usb_register_dev注册一次，获取一个次设备号。该次设备号从usb_class_driver -> minor_base开始分配  
	if (retval) {                                     //   也就是说，一个usb_interface对应一个次设备号    该次设备号从usb_class_driver -> minor_base开始分配
		/* something prevented us from registering this driver */
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "USB Skeleton device now attached to USBSkel-%d",
		 interface->minor);
	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, skel_delete);
	return retval;
}

static void skel_disconnect(struct usb_interface *interface)
{
	struct usb_skel *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);  //  注销 interface 的 data  

	/* give back our minor */
	usb_deregister_dev(interface, &skel_class);   // 注销  class driver (交还次设备号)

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, skel_delete);

	dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static void skel_draw_down(struct usb_skel *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->bulk_in_urb);
}

static int skel_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	skel_draw_down(dev);
	return 0;
}

static int skel_resume(struct usb_interface *intf)
{
	return 0;
}

static int skel_pre_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	skel_draw_down(dev);

	return 0;
}

static int skel_post_reset(struct usb_interface *intf)
{
	struct usb_skel *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

//  一个USB设备的功能划分是通过接口来完成的  usb_interface 

static struct usb_driver skel_driver = {
	.name =		"skeleton",   // 用来告诉内核  模块的名字是什么  这个注册之后由系统来使用  和我们关系不大 
	.probe =	skel_probe,   // usb子系统通过设备的production ID和vendor ID  的组合或者
	                          //设备的class、subclass跟protocol的组合来识别设备 并调用该函数 
	.disconnect =	skel_disconnect,  // 当设备被拔出集线器时 会调用  
	.suspend =	skel_suspend,
	.resume =	skel_resume,
	.pre_reset =	skel_pre_reset,
	.post_reset =	skel_post_reset,
	.id_table =	skel_table,     //  用来告诉内核  该模块支持哪些设备  
	.supports_autosuspend = 1,
};

module_usb_driver(skel_driver);   //  注册驱动程序    他要向系统提供几个函数入口，跟驱动的名字  skel_driver  

MODULE_LICENSE("GPL");
