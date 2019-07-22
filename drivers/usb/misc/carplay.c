#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/scatterlist.h>
#include <linux/mutex.h>

#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/usb.h>

#include "carplay.h"


#define 	CARPLAY_SWITCH_CMD		0x10
#define		IAP2_SUPPORT_CMD			0x20
#define		NOTCARPLAT_SWITCH_CMD		0x00
#define     CARLIFE_SWITCH_CMD      0x30

static struct kobject *carplay_kobj = NULL;
struct carplay_dev	*g_dev;

int _carplay_status = eCarplay_UnKnown;

static char _store_buf = 0;

EXPORT_SYMBOL_GPL(_carplay_status);

#define USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(d, i) ((unsigned short)((unsigned short)d<<8 | i))

/*-------------------------------------------------------------------------*/

/* FIXME make these public somewhere; usbdevfs.h? */
struct carplay_param {
	/* inputs */
	unsigned		test_num;	/* 0..(TEST_CASES-1) */
	unsigned		iterations;
	unsigned		length;
	unsigned		vary;
	unsigned		sglen;

	/* outputs */
	struct timeval		duration;
};
#define USBTEST_REQUEST	_IOWR('U', 100, struct carplay_param)

/*-------------------------------------------------------------------------*/

#define	GENERIC		/* let probe() bind using module params */

/* Some devices that can be used for testing will have "real" drivers.
 * Entries for those need to be enabled here by hand, after disabling
 * that "real" driver.
 */
//#define	IBOT2		/* grab iBOT2 webcams */
//#define	KEYSPAN_19Qi	/* grab un-renumerated serial adapter */

/*-------------------------------------------------------------------------*/

struct carplay_info {
	const char		*name;
	u8			ep_in;		/* bulk/intr source */
	u8			ep_out;		/* bulk/intr sink */
	unsigned		autoconf:1;
	unsigned		ctrl_out:1;
	unsigned		iso:1;		/* try iso in/out */
	int			alt;
};

/* this is accessed only through usbfs ioctl calls.
 * one ioctl to issue a test ... one lock per device.
 * tests create other threads if they need them.
 * urbs and buffers are allocated dynamically,
 * and data generated deterministically.
 */
struct carplay_dev {
	struct usb_interface	*intf;
	struct carplay_info	*info;
	int			in_pipe;
	int			out_pipe;
	int			in_iso_pipe;
	int			out_iso_pipe;
	struct usb_endpoint_descriptor	*iso_in, *iso_out;
	struct mutex		lock;

#define TBUF_SIZE	256
	u8			*buf;
};

static struct usb_device *testdev_to_usbdev(struct carplay_dev *test)
{
	return interface_to_usbdev(test->intf);
}

/* set up all urbs so they can be used with either bulk or interrupt */
#define	INTERRUPT_RATE		1	/* msec/transfer */

#define ERROR(tdev, fmt, args...) \
	dev_err(&(tdev)->intf->dev , fmt , ## args)
#define WARNING(tdev, fmt, args...) \
	dev_warn(&(tdev)->intf->dev , fmt , ## args)

#define GUARD_BYTE	0xA5

/*-------------------------------------------------------------------------*/

static int
get_endpoints(struct carplay_dev *dev, struct usb_interface *intf)
{
	int				tmp;
	struct usb_host_interface	*alt;
	struct usb_host_endpoint	*in, *out;
	struct usb_host_endpoint	*iso_in, *iso_out;
	struct usb_device		*udev;

	for (tmp = 0; tmp < intf->num_altsetting; tmp++) {
		unsigned	ep;

		in = out = NULL;
		iso_in = iso_out = NULL;
		alt = intf->altsetting + tmp;

		/* take the first altsetting with in-bulk + out-bulk;
		 * ignore other endpoints and altsettings.
		 */
		for (ep = 0; ep < alt->desc.bNumEndpoints; ep++) {
			struct usb_host_endpoint	*e;

			e = alt->endpoint + ep;
			switch (e->desc.bmAttributes) {
			case USB_ENDPOINT_XFER_BULK:
				break;
			case USB_ENDPOINT_XFER_ISOC:
				if (dev->info->iso)
					goto try_iso;
				/* FALLTHROUGH */
			default:
				continue;
			}
			if (usb_endpoint_dir_in(&e->desc)) {
				if (!in)
					in = e;
			} else {
				if (!out)
					out = e;
			}
			continue;
try_iso:
			if (usb_endpoint_dir_in(&e->desc)) {
				if (!iso_in)
					iso_in = e;
			} else {
				if (!iso_out)
					iso_out = e;
			}
		}
		if ((in && out)  ||  iso_in || iso_out)
			goto found;
	}
	return -EINVAL;

found:
	udev = testdev_to_usbdev(dev);
	if (alt->desc.bAlternateSetting != 0) {
		tmp = usb_set_interface(udev,
				alt->desc.bInterfaceNumber,
				alt->desc.bAlternateSetting);
		if (tmp < 0)
			return tmp;
	}

	if (in) {
		dev->in_pipe = usb_rcvbulkpipe(udev,
			in->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
		dev->out_pipe = usb_sndbulkpipe(udev,
			out->desc.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
	}
	if (iso_in) {
		dev->iso_in = &iso_in->desc;
		dev->in_iso_pipe = usb_rcvisocpipe(udev,
				iso_in->desc.bEndpointAddress
					& USB_ENDPOINT_NUMBER_MASK);
	}

	if (iso_out) {
		dev->iso_out = &iso_out->desc;
		dev->out_iso_pipe = usb_sndisocpipe(udev,
				iso_out->desc.bEndpointAddress
					& USB_ENDPOINT_NUMBER_MASK);
	}
	return 0;
}

/*-------------------------------------------------------------------------*/

/* Support for testing basic non-queued I/O streams.
 *
 * These just package urbs as requests that can be easily canceled.
 * Each urb's data buffer is dynamically allocated; callers can fill
 * them with non-zero test data (or test for it) when appropriate.
 */

static void simple_callback(struct urb *urb)
{
	complete(urb->context);
}

static struct urb *carplay_alloc_urb(
	struct usb_device	*udev,
	int			pipe,
	unsigned long		bytes,
	unsigned		transfer_flags,
	unsigned		offset)
{
	struct urb		*urb;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return urb;
	usb_fill_bulk_urb(urb, udev, pipe, NULL, bytes, simple_callback, NULL);
	urb->interval = (udev->speed == USB_SPEED_HIGH)
			? (INTERRUPT_RATE << 3)
			: INTERRUPT_RATE;
	urb->transfer_flags = transfer_flags;
	if (usb_pipein(pipe))
		urb->transfer_flags |= URB_SHORT_NOT_OK;

	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
		urb->transfer_buffer = usb_alloc_coherent(udev, bytes + offset,
			GFP_KERNEL, &urb->transfer_dma);
	else
		urb->transfer_buffer = kmalloc(bytes + offset, GFP_KERNEL);

	if (!urb->transfer_buffer) {
		usb_free_urb(urb);
		return NULL;
	}

	/* To test unaligned transfers add an offset and fill the
		unused memory with a guard value */
	if (offset) {
		memset(urb->transfer_buffer, GUARD_BYTE, offset);
		urb->transfer_buffer += offset;
		if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
			urb->transfer_dma += offset;
	}

	/* For inbound transfers use guard byte so that test fails if
		data not correctly copied */
	memset(urb->transfer_buffer,
			usb_pipein(urb->pipe) ? GUARD_BYTE : 0,
			bytes);
	return urb;
}

static struct urb *simple_alloc_urb(
	struct usb_device	*udev,
	int			pipe,
	unsigned long		bytes)
{
	return carplay_alloc_urb(udev, pipe, bytes, URB_NO_TRANSFER_DMA_MAP, 0);
}

static unsigned pattern;
static unsigned mod_pattern;
module_param_named(pattern, mod_pattern, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mod_pattern, "i/o pattern (0 == zeroes)");

static inline void simple_fill_buf(struct urb *urb)
{
	unsigned	i;
	u8		*buf = urb->transfer_buffer;
	unsigned	len = urb->transfer_buffer_length;

	switch (pattern) {
	default:
		/* FALLTHROUGH */
	case 0:
		memset(buf, 0, len);
		break;
	case 1:			/* mod63 */
		for (i = 0; i < len; i++)
			*buf++ = (u8) (i % 63);
		break;
	}
}

static inline unsigned long buffer_offset(void *buf)
{
	return (unsigned long)buf & (ARCH_KMALLOC_MINALIGN - 1);
}

static int check_guard_bytes(struct carplay_dev *tdev, struct urb *urb)
{
	u8 *buf = urb->transfer_buffer;
	u8 *guard = buf - buffer_offset(buf);
	unsigned i;

	for (i = 0; guard < buf; i++, guard++) {
		if (*guard != GUARD_BYTE) {
			ERROR(tdev, "guard byte[%d] %d (not %d)\n",
				i, *guard, GUARD_BYTE);
			return -EINVAL;
		}
	}
	return 0;
}

static int simple_check_buf(struct carplay_dev *tdev, struct urb *urb)
{
	unsigned	i;
	u8		expected;
	u8		*buf = urb->transfer_buffer;
	unsigned	len = urb->actual_length;

	int ret = check_guard_bytes(tdev, urb);
	if (ret)
		return ret;

	for (i = 0; i < len; i++, buf++) {
		switch (pattern) {
		/* all-zeroes has no synchronization issues */
		case 0:
			expected = 0;
			break;
		/* mod63 stays in sync with short-terminated transfers,
		 * or otherwise when host and gadget agree on how large
		 * each usb transfer request should be.  resync is done
		 * with set_interface or set_config.
		 */
		case 1:			/* mod63 */
			expected = i % 63;
			break;
		/* always fail unsupported patterns */
		default:
			expected = !*buf;
			break;
		}
		if (*buf == expected)
			continue;
		ERROR(tdev, "buf[%d] = %d (not %d)\n", i, *buf, expected);
		return -EINVAL;
	}
	return 0;
}

static void simple_free_urb(struct urb *urb)
{
	unsigned long offset = buffer_offset(urb->transfer_buffer);

	if (urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)
		usb_free_coherent(
			urb->dev,
			urb->transfer_buffer_length + offset,
			urb->transfer_buffer - offset,
			urb->transfer_dma - offset);
	else
		kfree(urb->transfer_buffer - offset);
	usb_free_urb(urb);
}

static int simple_io(
	struct carplay_dev	*tdev,
	struct urb		*urb,
	int			iterations,
	int			vary,
	int			expected,
	const char		*label
)
{
	struct usb_device	*udev = urb->dev;
	int			max = urb->transfer_buffer_length;
	struct completion	completion;
	int			retval = 0;

	urb->context = &completion;
	while (retval == 0 && iterations-- > 0) {
		init_completion(&completion);
		if (usb_pipeout(urb->pipe)) {
			simple_fill_buf(urb);
			urb->transfer_flags |= URB_ZERO_PACKET;
		}
		retval = usb_submit_urb(urb, GFP_KERNEL);
		if (retval != 0)
			break;

		/* NOTE:  no timeouts; can't be broken out of by interrupt */
		wait_for_completion(&completion);
		retval = urb->status;
		urb->dev = udev;
		if (retval == 0 && usb_pipein(urb->pipe))
			retval = simple_check_buf(tdev, urb);

		if (vary) {
			int	len = urb->transfer_buffer_length;

			len += vary;
			len %= max;
			if (len == 0)
				len = (vary < max) ? vary : max;
			urb->transfer_buffer_length = len;
		}

		/* FIXME if endpoint halted, clear halt (and log) */
	}
	urb->transfer_buffer_length = max;

	if (expected != retval)
		dev_err(&udev->dev,
			"%s failed, iterations left %d, status %d (not %d)\n",
				label, iterations, retval, expected);
	return retval;
}



/*-------------------------------------------------------------------------*/

/* unqueued control message testing
 *
 * there's a nice set of device functional requirements in chapter 9 of the
 * usb 2.0 spec, which we can apply to ANY device, even ones that don't use
 * special test firmware.
 *
 * we know the device is configured (or suspended) by the time it's visible
 * through usbfs.  we can't change that, so we won't test enumeration (which
 * worked 'well enough' to get here, this time), power management (ditto),
 * or remote wakeup (which needs human interaction).
 */

static unsigned realworld = 1;
module_param(realworld, uint, 0);
MODULE_PARM_DESC(realworld, "clear to demand stricter spec compliance");

static int get_altsetting(struct carplay_dev *dev)
{
	struct usb_interface	*iface = dev->intf;
	struct usb_device	*udev = interface_to_usbdev(iface);
	int			retval;

	retval = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			USB_REQ_GET_INTERFACE, USB_DIR_IN|USB_RECIP_INTERFACE,
			0, iface->altsetting[0].desc.bInterfaceNumber,
			dev->buf, 1, USB_CTRL_GET_TIMEOUT);
	switch (retval) {
	case 1:
		return dev->buf[0];
	case 0:
		retval = -ERANGE;
		/* FALLTHROUGH */
	default:
		return retval;
	}
}

static int set_altsetting(struct carplay_dev *dev, int alternate)
{
	struct usb_interface		*iface = dev->intf;
	struct usb_device		*udev;

	if (alternate < 0 || alternate >= 256)
		return -EINVAL;

	udev = interface_to_usbdev(iface);
	return usb_set_interface(udev,
			iface->altsetting[0].desc.bInterfaceNumber,
			alternate);
}

static int is_good_config(struct carplay_dev *tdev, int len)
{
	struct usb_config_descriptor	*config;

	if (len < sizeof *config)
		return 0;
	config = (struct usb_config_descriptor *) tdev->buf;

	switch (config->bDescriptorType) {
	case USB_DT_CONFIG:
	case USB_DT_OTHER_SPEED_CONFIG:
		if (config->bLength != 9) {
			ERROR(tdev, "bogus config descriptor length\n");
			return 0;
		}
		/* this bit 'must be 1' but often isn't */
		if (!realworld && !(config->bmAttributes & 0x80)) {
			ERROR(tdev, "high bit of config attributes not set\n");
			return 0;
		}
		if (config->bmAttributes & 0x1f) {	/* reserved == 0 */
			ERROR(tdev, "reserved config bits set\n");
			return 0;
		}
		break;
	default:
		return 0;
	}

	if (le16_to_cpu(config->wTotalLength) == len)	/* read it all */
		return 1;
	if (le16_to_cpu(config->wTotalLength) >= TBUF_SIZE)	/* max partial read */
		return 1;
	ERROR(tdev, "bogus config descriptor read size\n");
	return 0;
}

static int send_iap2_support(struct carplay_dev *dev)
{
	struct usb_interface	*iface = dev->intf;
	struct usb_device	*udev = interface_to_usbdev(iface);
	int			retval;
	char data[] = {0x0e, 0x00, 0xff, 0x55, 0x02, 0x00, 0xee, 0x10, 0x00, 0x00};
	int len = sizeof(data);
       printk("[carplay]+++  send_switch_cmd_nocarplay+++\n");
	retval = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			0x09,
            USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
            USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(2, 0x0e),
            2,
            data,
            len,
			USB_CTRL_GET_TIMEOUT);
	printk("send_iap2_support, retval=%d\n", retval);
	return retval;  //>=0 success
}

static int send_switch_cmd_nocarplay(struct carplay_dev *dev)
{
	struct usb_interface	*iface = dev->intf;
	struct usb_device	*udev = interface_to_usbdev(iface);
	int			 retval;
	printk("[carplay]+++  send_switch_cmd_nocarplay+++\n");
	/*retval = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			0x51,
			0x40,
			0, 0, dev->buf, 0, USB_CTRL_GET_TIMEOUT);
	printk("send_switch_cmd_nocarplay, retval=%d\n", retval);
	if (retval != 0) {
		dev_err(&iface->dev, "get config --> %d \n", retval);
	}
	if (retval == -110) //?
	{
		printk("send_switch_cmd_nocarplay?????\n");
		return 0;
	}*/

	retval = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			0x51,
			0x40,
			0, 0, NULL, 0, 1000/*USB_CTRL_GET_TIMEOUT*/);
	printk("[carplay]+++  send_switch_cmd_nocarplay+++, ret=%d, dev->buf=0x%x\n", retval, dev->buf);

	return retval;
}

static int send_switch_cmd(struct carplay_dev *dev)
{
	struct usb_interface	*iface = dev->intf;
	struct usb_device	*udev = interface_to_usbdev(iface);
	int			i, alt, retval;
	struct file *fp;
	loff_t pos;
	mm_segment_t old_fs;
	char data[10] = {'1'};
	printk("[carplay]+++  send_switch_cmd+++\n");
	/*retval = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			0x51,
			0x40,
			1, 0, dev->buf, 0, USB_CTRL_GET_TIMEOUT);
	printk("send_switch_cmd, retval=%d\n", retval);
	if (retval != 0) {
		dev_err(&iface->dev, "get config --> %d \n", retval);
	}
	if (retval == -110)
	{
		printk("send_switch_cmd?????\n");
		return 0;
	}*/

	retval = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			0x51,
			0x40,
			1, 0, NULL, 0, 1000/*USB_CTRL_GET_TIMEOUT*/);

	printk("[carplay]+++  send_switch_cmd+++, ret=%d, dev->buf=0x%x\n", retval, dev->buf);

#if 0
	fp = filp_open("/sys/udisk2detect/hnp", O_RDWR, 0666);
	if (IS_ERR(fp)) {
		printk("open sys-udisk2detect-hnp failed\n");
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0x0;
	vfs_write(fp, data, 10, &pos);
	filp_close(fp, NULL);
	set_fs(old_fs);
#endif
	//kobject_uevent(&udev->dev.kobj, KOBJ_ONLINE);
	return retval;
}

static int send_switch_cmd_carlife(struct carplay_dev *dev)
{
	struct usb_interface	*iface = dev->intf;
	struct usb_device	*udev = interface_to_usbdev(iface);
	int			 retval;
	printk("[carplay]+++  send_switch_cmd_carlife+++\n");

	retval = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			0x51,
			0x40,
			0, 0, NULL, 0, 1000/*USB_CTRL_GET_TIMEOUT*/);
	printk("[carplay]+++  send_switch_cmd_carlife+++, ret=%d, dev->buf=0x%x\n", retval, dev->buf);

	return retval;
}


/*-------------------------------------------------------------------------*/

/* use ch9 requests to test whether:
 *   (a) queues work for control, keeping N subtests queued and
 *       active (auto-resubmit) for M loops through the queue.
 *   (b) protocol stalls (control-only) will autorecover.
 *       it's not like bulk/intr; no halt clearing.
 *   (c) short control reads are reported and handled.
 *   (d) queues are always processed in-order
 */

struct ctrl_ctx {
	spinlock_t		lock;
	struct carplay_dev	*dev;
	struct completion	complete;
	unsigned		count;
	unsigned		pending;
	int			status;
	struct urb		**urb;
	struct carplay_param	*param;
	int			last;
};

#define NUM_SUBCASES	15		/* how many test subcases here? */

struct subcase {
	struct usb_ctrlrequest	setup;
	int			number;
	int			expected;
};


#undef NUM_SUBCASES


/*-------------------------------------------------------------------------*/

/* We only have this one interface to user space, through usbfs.
 * User mode code can scan usbfs to find N different devices (maybe on
 * different busses) to use when testing, and allocate one thread per
 * test.  So discovery is simplified, and we have no device naming issues.
 *
 * Don't use these only as stress/load tests.  Use them along with with
 * other USB bus activity:  plugging, unplugging, mousing, mp3 playback,
 * video capture, and so on.  Run different tests at different times, in
 * different sequences.  Nothing here should interact with other devices,
 * except indirectly by consuming USB bandwidth and CPU resources for test
 * threads and request completion.  But the only way to know that for sure
 * is to test when HC queues are in use by many devices.
 *
 * WARNING:  Because usbfs grabs udev->dev.sem before calling this ioctl(),
 * it locks out usbcore in certain code paths.  Notably, if you disconnect
 * the device-under-test, khubd will wait block forever waiting for the
 * ioctl to complete ... so that usb_disconnect() can abort the pending
 * urbs and then call carplay_disconnect().  To abort a test, you're best
 * off just killing the userspace task and waiting for it to exit.
 */

static int
carplay_ioctl(struct usb_interface *intf, unsigned int code, void *buf)
{
	struct carplay_dev	*dev = usb_get_intfdata(intf);
	struct usb_device	*udev = testdev_to_usbdev(dev);
	struct carplay_param	*param = buf;
	int			retval = -EOPNOTSUPP;
	struct urb		*urb;
	struct scatterlist	*sg;
	struct usb_sg_request	req;
	struct timeval		start;
	unsigned		i;

	/* FIXME USBDEVFS_CONNECTINFO doesn't say how fast the device is. */

	pattern = mod_pattern;

	if (code != USBTEST_REQUEST)
		return -EOPNOTSUPP;

	if (param->iterations <= 0)
		return -EINVAL;

	if (mutex_lock_interruptible(&dev->lock))
		return -ERESTARTSYS;

	/* FIXME: What if a system sleep starts while a test is running? */

	/* some devices, like ez-usb default devices, need a non-default
	 * altsetting to have any active endpoints.  some tests change
	 * altsettings; force a default so most tests don't need to check.
	 */
	if (dev->info->alt >= 0) {
		int	res;

		if (intf->altsetting->desc.bInterfaceNumber) {
			mutex_unlock(&dev->lock);
			return -ENODEV;
		}
		res = set_altsetting(dev, dev->info->alt);
		if (res) {
			dev_err(&intf->dev,
					"set altsetting to %d failed, %d\n",
					dev->info->alt, res);
			mutex_unlock(&dev->lock);
			return res;
		}
	}

	/*
	 * Just a bunch of test cases that every HCD is expected to handle.
	 *
	 * Some may need specific firmware, though it'd be good to have
	 * one firmware image to handle all the test cases.
	 *
	 * FIXME add more tests!  cancel requests, verify the data, control
	 * queueing, concurrent read+write threads, and so on.
	 */
	do_gettimeofday(&start);
	switch (param->test_num) {

	case 0:
		dev_info(&intf->dev, "TEST 0:  NOP\n");
		retval = 0;
		break;

	case 1:
		retval = 0;
		dev_info(&intf->dev, "send usb swtich cmd\n");
		retval = send_switch_cmd(dev);
		break;
		
	}
	do_gettimeofday(&param->duration);
	param->duration.tv_sec -= start.tv_sec;
	param->duration.tv_usec -= start.tv_usec;
	if (param->duration.tv_usec < 0) {
		param->duration.tv_usec += 1000 * 1000;
		param->duration.tv_sec -= 1;
	}
	mutex_unlock(&dev->lock);
	return retval;
}

/*-------------------------------------------------------------------------*/

static unsigned force_interrupt;
module_param(force_interrupt, uint, 0);
MODULE_PARM_DESC(force_interrupt, "0 = test default; else interrupt");

#ifdef	GENERIC
static unsigned short vendor;
module_param(vendor, ushort, 0);
MODULE_PARM_DESC(vendor, "vendor code (from usb-if)");

static unsigned short product;
module_param(product, ushort, 0);
MODULE_PARM_DESC(product, "product code (from vendor)");
#endif

static int
carplay_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device	*udev;
	struct carplay_dev	*dev;
	struct carplay_info	*info;
	char			*rtest, *wtest;
	char			*irtest, *iwtest;
	static int		timer = 0;
	
	printk("++ carplay probe ++\n");
	//timer++;
	//if (timer == 1)
	//{
	//	printk("exit carplay_probe\n");
	//	return 0;
	//}
	udev = interface_to_usbdev(intf);

#ifdef	GENERIC
	/* specify devices by module parameters? */
	if (id->match_flags == 0) {
		/* vendor match required, product match optional */
		if (!vendor || le16_to_cpu(udev->descriptor.idVendor) != (u16)vendor)
			return -ENODEV;
		if (product && le16_to_cpu(udev->descriptor.idProduct) != (u16)product)
			return -ENODEV;
		dev_info(&intf->dev, "matched module params, "
					"vend=0x%04x prod=0x%04x\n",
				le16_to_cpu(udev->descriptor.idVendor),
				le16_to_cpu(udev->descriptor.idProduct));
	}
#endif

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	g_dev = dev;
	info = (struct carplay_info *) id->driver_info;
	dev->info = info;
	mutex_init(&dev->lock);

	dev->intf = intf;

	/* cacheline-aligned scratch for i/o */
	dev->buf = kmalloc(TBUF_SIZE, GFP_KERNEL);
	if (dev->buf == NULL) {
		kfree(dev);
		return -ENOMEM;
	}

	/* NOTE this doesn't yet test the handful of difference that are
	 * visible with high speed interrupts:  bigger maxpacket (1K) and
	 * "high bandwidth" modes (up to 3 packets/uframe).
	 */
	rtest = wtest = "";
	irtest = iwtest = "";
	if (force_interrupt || udev->speed == USB_SPEED_LOW) {
		if (info->ep_in) {
			dev->in_pipe = usb_rcvintpipe(udev, info->ep_in);
			rtest = " intr-in";
		}
		if (info->ep_out) {
			dev->out_pipe = usb_sndintpipe(udev, info->ep_out);
			wtest = " intr-out";
		}
	} else {
		if (info->autoconf) {
			int status;

			status = get_endpoints(dev, intf);
			if (status < 0) {
				WARNING(dev, "couldn't get endpoints, %d\n",
						status);
				kfree(dev->buf);
				kfree(dev);
				return status;
			}
			/* may find bulk or ISO pipes */
		} else {
			if (info->ep_in)
				dev->in_pipe = usb_rcvbulkpipe(udev,
							info->ep_in);
			if (info->ep_out)
				dev->out_pipe = usb_sndbulkpipe(udev,
							info->ep_out);
		}
		if (dev->in_pipe)
			rtest = " bulk-in";
		if (dev->out_pipe)
			wtest = " bulk-out";
		if (dev->in_iso_pipe)
			irtest = " iso-in";
		if (dev->out_iso_pipe)
			iwtest = " iso-out";
	}
	usb_set_intfdata(intf, dev);
	dev_info(&intf->dev, "%s\n", info->name);
	dev_info(&intf->dev, "%s {control%s%s%s%s%s} tests%s\n",
			usb_speed_string(udev->speed),
			info->ctrl_out ? " in/out" : "",
			rtest, wtest,
			irtest, iwtest,
			info->alt >= 0 ? " (+alt)" : "");

	/* test usb switch function */
	//send_switch_cmd(dev);
	return 0;
}

static int carplay_suspend(struct usb_interface *intf, pm_message_t message)
{
	return 0;
}

static int carplay_resume(struct usb_interface *intf)
{
	return 0;
}


static void carplay_disconnect(struct usb_interface *intf)
{
	struct carplay_dev	*dev = usb_get_intfdata(intf);
	char *charplay[2] = { "IPOD_STATE=DISCONNECTED", NULL };
	char **uevent_envp = NULL;
	printk("carplay_disconnect\n");
	if(intf->usb_dev != NULL)
	{
		printk("cpy_disconnect:%s\n",kobject_get_path(&intf->usb_dev->kobj, GFP_KERNEL));
		kobject_uevent_env(&intf->usb_dev->kobj, KOBJ_CHANGE, uevent_envp);
	}
	g_dev = NULL;
	usb_set_intfdata(intf, NULL);
	dev_dbg(&intf->dev, "disconnect\n");
	kfree(dev);
}

static ssize_t carplay_cmd_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    	printk("carplay show,0x%x\n", _store_buf);
    	
	return 0;   
}


static ssize_t carplay_cmd_store(struct kobject *kobj, struct kobj_attribute *attr,
             const char *buf, size_t count)
{
	int ret;
	printk("[carplay] carplay store buf = %d \n",*buf);
	_store_buf = *buf;
	if(*buf == CARPLAY_SWITCH_CMD){
		if(g_dev != NULL){
 			ret = send_switch_cmd(g_dev);
			//g_dev = NULL;
			printk("++ret CARPLAY_SWITCH_CMD = %d",ret);
			return ret;
		}
		else{
			printk("[carplay CARPLAY_SWITCH_CMD] No carplay device \n");
			return -1;
		}	
	}
	else if(*buf == IAP2_SUPPORT_CMD)
	{
		if(g_dev != NULL){
 			ret = send_iap2_support(g_dev);
			//g_dev = NULL;
			if(ret >= 0)
			{
				_carplay_status = eCarplay_SupportCarPlay;
			}
			printk("++ret IAP2_SUPPORT_CMD = %d",ret);
			return ret;
		}
		else{
			printk("[carplay IAP2_SUPPORT_CMD] No carplay device \n");
			return -1;
		}	
	}
	else if(*buf == NOTCARPLAT_SWITCH_CMD)
	{
		if(g_dev != NULL){
 			ret = send_switch_cmd_nocarplay(g_dev);
			//g_dev = NULL;
			if(ret >= 0)
			{
				_carplay_status = eCarplay_NotSupportCarPlay;
			}
			printk("++ret nocarplay = %d",ret);
			return ret;
		}
		else{
			printk("[carplay-nocarplay] No carplay device \n");
			return -1;
		}	
	}
	else if(*buf == CARLIFE_SWITCH_CMD)
	{
		if(g_dev != NULL){
 			ret = send_switch_cmd_carlife(g_dev);	
			printk("++ret send carlife cmd = %d",ret);
			return ret;
		}
		else{
			printk("[carlife] No carlife device \n");
			return -1;
		}	
	}
	else{
		printk("[carplay] Invaliable cmd\n");
		return -1;
	}
    	
}


static struct kobj_attribute carplay_cmd_attribute =
    __ATTR(carplay_cmd, 0660, carplay_cmd_show, carplay_cmd_store);


static struct attribute *attrs[] = {
    &carplay_cmd_attribute.attr,
    NULL,   /* need to NULL terminate the list of attributes */
};


static struct attribute_group attr_group = {
    .attrs = attrs,
};
static int __init carplay_init_sysfs(void)
{   
	int retval;
	printk("[carplay]+++ carplay_init_sysfs +++\n");
	carplay_kobj = kobject_create_and_add("carplay", NULL);
	if (!carplay_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(carplay_kobj, &attr_group);
	if (retval)
		kobject_put(carplay_kobj);

	return 0;
}





/* Basic testing only needs a device that can source or sink bulk traffic.
 * Any device can test control transfers (default with GENERIC binding).
 *
 * Several entries work with the default EP0 implementation that's built
 * into EZ-USB chips.  There's a default vendor ID which can be overridden
 * by (very) small config EEPROMS, but otherwise all these devices act
 * identically until firmware is loaded:  only EP0 works.  It turns out
 * to be easy to make other endpoints work, without modifying that EP0
 * behavior.  For now, we expect that kind of firmware.
 */


/* fx2 version of ez-usb */
static struct carplay_info iphone5_info = {
	.name		= "ip5 device",
	.ep_in		= 1,
	.ep_out		= 1,
	.alt		= -1,
};

#define USB_IPOD_VENDOR_ID    0x05ac
#define IPOD_DEVICE(vend) \
    .match_flags = USB_DEVICE_ID_MATCH_VENDOR  |USB_DEVICE_ID_MATCH_INT_CLASS,\
    .idVendor = (vend),\
    .bInterfaceClass = 0x03

static const struct usb_device_id id_table[] = {

	/*-------------------------------------------------------------*/

	/* EZ-USB devices which download firmware to replace (or in our
	 * case augment) the default device implementation.
	 */

	/* generic EZ-USB FX2 controller (or development board) */
	//{ USB_DEVICE(0x05ac, 0x12a8),
	//	.driver_info = (unsigned long) &iphone5_info,
	//},
	{  IPOD_DEVICE(USB_IPOD_VENDOR_ID),
	  .driver_info = (unsigned long) &iphone5_info,
	},


	/*-------------------------------------------------------------*/

	{ }
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver carplay_driver = {
	.name =		"carplay",
	.id_table =	id_table,
	.probe =	carplay_probe,
	.unlocked_ioctl = carplay_ioctl,
	.disconnect =	carplay_disconnect,
	.suspend =	carplay_suspend,
	.resume =	carplay_resume,
};

/*-------------------------------------------------------------------------*/

static int __init carplay_init(void)
{
#ifdef GENERIC
	if (vendor)
		pr_debug("params: vend=0x%04x prod=0x%04x\n", vendor, product);
#endif
	carplay_init_sysfs();
	return usb_register(&carplay_driver);
}
module_init(carplay_init);

static void __exit carplay_exit(void)
{
	usb_deregister(&carplay_driver);
}
module_exit(carplay_exit);

MODULE_DESCRIPTION("USB Core/HCD Testing Driver");
MODULE_LICENSE("GPL");

