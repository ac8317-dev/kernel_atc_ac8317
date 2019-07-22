#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/hcd.h>
#include <linux/usb/quirks.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <asm/byteorder.h>
#include "usb.h"


#define USB_MAXALTSETTING		128	/* Hard limit */
#define USB_MAXENDPOINTS		30	/* Hard limit */

#define USB_MAXCONFIG			8	/* Arbitrary limit */


static inline const char *plural(int n)
{
	return (n == 1 ? "" : "s");
}

static int find_next_descriptor(unsigned char *buffer, int size,
    int dt1, int dt2, int *num_skipped)
{
	struct usb_descriptor_header *h;
	int n = 0;
	unsigned char *buffer0 = buffer;

	/* Find the next descriptor of type dt1 or dt2 */
	while (size > 0) {
		h = (struct usb_descriptor_header *) buffer;
		if (h->bDescriptorType == dt1 || h->bDescriptorType == dt2)
			break;
		buffer += h->bLength;
		size -= h->bLength;
		++n;
	}

	/* Store the number of descriptors skipped and return the
	 * number of bytes skipped */
	if (num_skipped)
		*num_skipped = n;
	return buffer - buffer0;
}

static void usb_parse_ss_endpoint_companion(struct device *ddev, int cfgno,
		int inum, int asnum, struct usb_host_endpoint *ep,
		unsigned char *buffer, int size)
{
	struct usb_ss_ep_comp_descriptor *desc;
	int max_tx;

	/* The SuperSpeed endpoint companion descriptor is supposed to
	 * be the first thing immediately following the endpoint descriptor.
	 */
	desc = (struct usb_ss_ep_comp_descriptor *) buffer;
	if (desc->bDescriptorType != USB_DT_SS_ENDPOINT_COMP ||
			size < USB_DT_SS_EP_COMP_SIZE) {
		dev_warn(ddev, "No SuperSpeed endpoint companion for config %d "
				" interface %d altsetting %d ep %d: "
				"using minimum values\n",
				cfgno, inum, asnum, ep->desc.bEndpointAddress);

		/* Fill in some default values.
		 * Leave bmAttributes as zero, which will mean no streams for
		 * bulk, and isoc won't support multiple bursts of packets.
		 * With bursts of only one packet, and a Mult of 1, the max
		 * amount of data moved per endpoint service interval is one
		 * packet.
		 */
		ep->ss_ep_comp.bLength = USB_DT_SS_EP_COMP_SIZE;
		ep->ss_ep_comp.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;
		if (usb_endpoint_xfer_isoc(&ep->desc) ||
				usb_endpoint_xfer_int(&ep->desc))
			ep->ss_ep_comp.wBytesPerInterval =
					ep->desc.wMaxPacketSize;
		return;
	}

	memcpy(&ep->ss_ep_comp, desc, USB_DT_SS_EP_COMP_SIZE);

	/* Check the various values */
	if (usb_endpoint_xfer_control(&ep->desc) && desc->bMaxBurst != 0) {
		dev_warn(ddev, "Control endpoint with bMaxBurst = %d in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to zero\n", desc->bMaxBurst,
				cfgno, inum, asnum, ep->desc.bEndpointAddress);
		ep->ss_ep_comp.bMaxBurst = 0;
	} else if (desc->bMaxBurst > 15) {
		dev_warn(ddev, "Endpoint with bMaxBurst = %d in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to 15\n", desc->bMaxBurst,
				cfgno, inum, asnum, ep->desc.bEndpointAddress);
		ep->ss_ep_comp.bMaxBurst = 15;
	}

	if ((usb_endpoint_xfer_control(&ep->desc) ||
			usb_endpoint_xfer_int(&ep->desc)) &&
				desc->bmAttributes != 0) {
		dev_warn(ddev, "%s endpoint with bmAttributes = %d in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to zero\n",
				usb_endpoint_xfer_control(&ep->desc) ? "Control" : "Bulk",
				desc->bmAttributes,
				cfgno, inum, asnum, ep->desc.bEndpointAddress);
		ep->ss_ep_comp.bmAttributes = 0;
	} else if (usb_endpoint_xfer_bulk(&ep->desc) &&
			desc->bmAttributes > 16) {
		dev_warn(ddev, "Bulk endpoint with more than 65536 streams in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to max\n",
				cfgno, inum, asnum, ep->desc.bEndpointAddress);
		ep->ss_ep_comp.bmAttributes = 16;
	} else if (usb_endpoint_xfer_isoc(&ep->desc) &&
			desc->bmAttributes > 2) {
		dev_warn(ddev, "Isoc endpoint has Mult of %d in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to 3\n", desc->bmAttributes + 1,
				cfgno, inum, asnum, ep->desc.bEndpointAddress);
		ep->ss_ep_comp.bmAttributes = 2;
	}

	if (usb_endpoint_xfer_isoc(&ep->desc))
		max_tx = (desc->bMaxBurst + 1) * (desc->bmAttributes + 1) *
			usb_endpoint_maxp(&ep->desc);
	else if (usb_endpoint_xfer_int(&ep->desc))
		max_tx = usb_endpoint_maxp(&ep->desc) *
			(desc->bMaxBurst + 1);
	else
		max_tx = 999999;
	if (le16_to_cpu(desc->wBytesPerInterval) > max_tx) {
		dev_warn(ddev, "%s endpoint with wBytesPerInterval of %d in "
				"config %d interface %d altsetting %d ep %d: "
				"setting to %d\n",
				usb_endpoint_xfer_isoc(&ep->desc) ? "Isoc" : "Int",
				le16_to_cpu(desc->wBytesPerInterval),
				cfgno, inum, asnum, ep->desc.bEndpointAddress,
				max_tx);
		ep->ss_ep_comp.wBytesPerInterval = cpu_to_le16(max_tx);
	}
}

static int usb_parse_endpoint(struct device *ddev, int cfgno, int inum,
    int asnum, struct usb_host_interface *ifp, int num_ep,
    unsigned char *buffer, int size)
{
	unsigned char *buffer0 = buffer;
	struct usb_endpoint_descriptor *d;
	struct usb_host_endpoint *endpoint;
	int n, i, j, retval;

	d = (struct usb_endpoint_descriptor *) buffer;  //  根据端口描述符长度去更新buffer和size,并确认端口描述符的长度
	buffer += d->bLength;
	size -= d->bLength;

	if (d->bLength >= USB_DT_ENDPOINT_AUDIO_SIZE)
		n = USB_DT_ENDPOINT_AUDIO_SIZE;
	else if (d->bLength >= USB_DT_ENDPOINT_SIZE)
		n = USB_DT_ENDPOINT_SIZE;
	else {
		dev_warn(ddev, "config %d interface %d altsetting %d has an "
		    "invalid endpoint descriptor of length %d, skipping\n",
		    cfgno, inum, asnum, d->bLength);
		goto skip_to_next_endpoint_or_interface_descriptor;
	}

	i = d->bEndpointAddress & ~USB_ENDPOINT_DIR_MASK;     //   确认端口的地址及端口数量，标准的端口地址是从1到15
	if (i >= 16 || i == 0) {
		dev_warn(ddev, "config %d interface %d altsetting %d has an "
		    "invalid endpoint with address 0x%X, skipping\n",
		    cfgno, inum, asnum, d->bEndpointAddress);
		goto skip_to_next_endpoint_or_interface_descriptor;
	}

	/* Only store as many endpoints as we have room for */
	//将接口描述符中的端口数当成接口下端口数组endpoint的索引，获取用于存放当前endpoint信息的地址，并把当前的端口描述符保存到endpoint下
	if (ifp->desc.bNumEndpoints >= num_ep)  
		goto skip_to_next_endpoint_or_interface_descriptor;

	endpoint = &ifp->endpoint[ifp->desc.bNumEndpoints];
	++ifp->desc.bNumEndpoints;

	memcpy(&endpoint->desc, d, n);
	INIT_LIST_HEAD(&endpoint->urb_list);

	/* Fix up bInterval values outside the legal range. Use 32 ms if no
	 * proper value can be guessed. */
	i = 0;		/* i = min, j = max, n = default */   //   对于中断和等时传输类型的端口，根据usb设备速度设置时间时隔interval
	j = 255;
	if (usb_endpoint_xfer_int(d)) {
		i = 1;
		switch (to_usb_device(ddev)->speed) {
		case USB_SPEED_SUPER:
		case USB_SPEED_HIGH:
			/* Many device manufacturers are using full-speed
			 * bInterval values in high-speed interrupt endpoint
			 * descriptors. Try to fix those and fall back to a
			 * 32 ms default value otherwise. */
			n = fls(d->bInterval*8);
			if (n == 0)
				n = 9;	/* 32 ms = 2^(9-1) uframes */
			j = 16;
			break;
		default:		/* USB_SPEED_FULL or _LOW */
			/* For low-speed, 10 ms is the official minimum.
			 * But some "overclocked" devices might want faster
			 * polling so we'll allow it. */
			n = 32;
			break;
		}
	} else if (usb_endpoint_xfer_isoc(d)) {
		i = 1;
		j = 16;
		switch (to_usb_device(ddev)->speed) {
		case USB_SPEED_HIGH:
			n = 9;		/* 32 ms = 2^(9-1) uframes */
			break;
		default:		/* USB_SPEED_FULL */
			n = 6;		/* 32 ms = 2^(6-1) frames */
			break;
		}
	}
	if (d->bInterval < i || d->bInterval > j) {
		dev_warn(ddev, "config %d interface %d altsetting %d "
		    "endpoint 0x%X has an invalid bInterval %d, "
		    "changing to %d\n",
		    cfgno, inum, asnum,
		    d->bEndpointAddress, d->bInterval, n);
		endpoint->desc.bInterval = n;
	}

	/* Some buggy low-speed devices have Bulk endpoints, which is
	 * explicitly forbidden by the USB spec.  In an attempt to make
	 * them usable, we will try treating them as Interrupt endpoints.
	 */  //  在usb2.0协议中有定义说：在低速时是没有bulk类型的端口的，但实际上有可能存在这种情况，这里的话，我们把这个bulk端口当做interrupt类型端口使用
	if (to_usb_device(ddev)->speed == USB_SPEED_LOW &&
			usb_endpoint_xfer_bulk(d)) {
		dev_warn(ddev, "config %d interface %d altsetting %d "
		    "endpoint 0x%X is Bulk; changing to Interrupt\n",
		    cfgno, inum, asnum, d->bEndpointAddress);
		endpoint->desc.bmAttributes = USB_ENDPOINT_XFER_INT;
		endpoint->desc.bInterval = 1;
		if (usb_endpoint_maxp(&endpoint->desc) > 8)
			endpoint->desc.wMaxPacketSize = cpu_to_le16(8);
	}

	/*
	 * Some buggy high speed devices have bulk endpoints using
	 * maxpacket sizes other than 512.  High speed HCDs may not
	 * be able to handle that particular bug, so let's warn...
	 */
	if (to_usb_device(ddev)->speed == USB_SPEED_HIGH    //  对于运行在高速下的bulk端口，usb2.0定义这类端口最大传输长度为512，如果不为512则发出警告
			&& usb_endpoint_xfer_bulk(d)) {
		unsigned maxp;

		maxp = usb_endpoint_maxp(&endpoint->desc) & 0x07ff;
		if (maxp != 512)
			dev_warn(ddev, "config %d interface %d altsetting %d "
				"bulk endpoint 0x%X has invalid maxpacket %d\n",
				cfgno, inum, asnum, d->bEndpointAddress,
				maxp);
	}

	/* Parse a possible SuperSpeed endpoint companion descriptor */
	if (to_usb_device(ddev)->speed == USB_SPEED_SUPER)   //  如果当前设备是超速usb设备，则还要usb_parse_ss_endpoint_companion取得用于描述高速设备的描述符
		usb_parse_ss_endpoint_companion(ddev, cfgno,
				inum, asnum, endpoint, buffer, size);

	/* Skip over any Class Specific or Vendor Specific descriptors;
	 * find the next endpoint or interface descriptor
	 由这个配置描述符分解过程可以知道，配置描述符里包含下面内容：配置描述符，关联接口描述符或接口描述符，端口描述符对于超速设备还有用于描述超速设备的描述符，
	 这个分解过程验证了：usb设备由一个或多个usb 配置组成，usb配置由一个或多个usb接口组成，usb接口下包含多个usb接口设置，usb接口设置由多个usb端口组成
	 讲到这里usb_enumerate_device中的得要部分已经讲完，接下来回到usb_new_device继续讲解*/
	endpoint->extra = buffer;                    //  继续分解端口和接口描述符
	i = find_next_descriptor(buffer, size, USB_DT_ENDPOINT,
			USB_DT_INTERFACE, &n);
	endpoint->extralen = i;
	retval = buffer - buffer0 + i;
	if (n > 0)
		dev_dbg(ddev, "skipped %d descriptor%s after %s\n",
		    n, plural(n), "endpoint");
	return retval;

skip_to_next_endpoint_or_interface_descriptor:
	i = find_next_descriptor(buffer, size, USB_DT_ENDPOINT,
	    USB_DT_INTERFACE, NULL);
	return buffer - buffer0 + i;
}

void usb_release_interface_cache(struct kref *ref)
{
	struct usb_interface_cache *intfc = ref_to_usb_interface_cache(ref);
	int j;

	for (j = 0; j < intfc->num_altsetting; j++) {
		struct usb_host_interface *alt = &intfc->altsetting[j];

		kfree(alt->endpoint);
		kfree(alt->string);
	}
	kfree(intfc);
}

static int usb_parse_interface(struct device *ddev, int cfgno,
    struct usb_host_config *config, unsigned char *buffer, int size,
    u8 inums[], u8 nalts[])
{
	unsigned char *buffer0 = buffer;
	struct usb_interface_descriptor	*d;
	int inum, asnum;
	struct usb_interface_cache *intfc;
	struct usb_host_interface *alt;
	int i, n;
	int len, retval;
	int num_ep, num_ep_orig;

	d = (struct usb_interface_descriptor *) buffer;   //  更新buffer和size，并判断接口描述符的长度是否正常，如果不正常则查找下一个接口描述符
	buffer += d->bLength;
	size -= d->bLength;

	if (d->bLength < USB_DT_INTERFACE_SIZE)
		goto skip_to_next_interface_descriptor;

	/* Which interface entry is this? */
	intfc = NULL;               //  根据当前接口描述符中的接口号来得到用于存放接口信息的intf_cache
	inum = d->bInterfaceNumber;
	for (i = 0; i < config->desc.bNumInterfaces; ++i) {
		if (inums[i] == inum) {
			intfc = config->intf_cache[i];
			break;
		}
	}
	if (!intfc || intfc->num_altsetting >= nalts[i])
		goto skip_to_next_interface_descriptor;

	/* Check for duplicate altsetting entries 
	把接口描述符中的接口设置值与存放在intf_cache中的接口设置值进行比较，如果找到相同的设置值，
	则表示这个接口设置已经保存在intf_cache里了，则跳过当前接口设置，直接去查找下一个接口；如果找不到相同的，
	则表示这是一个新的接口设置，则通过for 循环中++alt来确定接口设置存放位置
	*/
	asnum = d->bAlternateSetting;
	for ((i = 0, alt = &intfc->altsetting[0]);
	      i < intfc->num_altsetting;
	     (++i, ++alt)) {
		if (alt->desc.bAlternateSetting == asnum) {
			dev_warn(ddev, "Duplicate descriptor for config %d "
			    "interface %d altsetting %d, skipping\n",
			    cfgno, inum, asnum);
			goto skip_to_next_interface_descriptor;
		}
	}

	++intfc->num_altsetting;    //  更新接口设置值，并将接口描述符保存到相应接口下
	memcpy(&alt->desc, d, USB_DT_INTERFACE_SIZE);

	/* Skip over any Class Specific or Vendor Specific descriptors;
	 * find the first endpoint or interface descriptor */
	alt->extra = buffer;  //  在剩下的buffer里找到endpoint或interface描述符首次出现位置，并根据返回值来更新buffer和size
	i = find_next_descriptor(buffer, size, USB_DT_ENDPOINT,
	    USB_DT_INTERFACE, &n);
	alt->extralen = i;
	if (n > 0)
		dev_dbg(ddev, "skipped %d descriptor%s after %s\n",
		    n, plural(n), "interface");
	buffer += i;
	size -= i;

	/* Allocate space for the right(?) number of endpoints */
	num_ep = num_ep_orig = alt->desc.bNumEndpoints;   //  得到当前接口设置下有多少个endpoint端口，并为这些端口申请空间
	alt->desc.bNumEndpoints = 0;		/* Use as a counter */
	if (num_ep > USB_MAXENDPOINTS) {
		dev_warn(ddev, "too many endpoints for config %d interface %d "
		    "altsetting %d: %d, using maximum allowed: %d\n",
		    cfgno, inum, asnum, num_ep, USB_MAXENDPOINTS);
		num_ep = USB_MAXENDPOINTS;
	}

	if (num_ep > 0) {
		/* Can't allocate 0 bytes */
		len = sizeof(struct usb_host_endpoint) * num_ep;
		alt->endpoint = kzalloc(len, GFP_KERNEL);
		if (!alt->endpoint)
			return -ENOMEM;
	}

	/* Parse all the endpoint descriptors
	如果在接口描述符后面紧跟着端口描述符，则通过usb_parse_endpoint对端口描述符进行分解，并将它保存在之前申请的endpoint里
	*/
	n = 0;
	while (size > 0) {
		if (((struct usb_descriptor_header *) buffer)->bDescriptorType
		     == USB_DT_INTERFACE)
			break;
		retval = usb_parse_endpoint(ddev, cfgno, inum, asnum, alt,   //  这个函数比较重要    对端口描述符进行解析  
		    num_ep, buffer, size);
		if (retval < 0)
			return retval;
		++n;

		buffer += retval;
		size -= retval;
	}

	if (n != num_ep_orig)
		dev_warn(ddev, "config %d interface %d altsetting %d has %d "
		    "endpoint descriptor%s, different from the interface "
		    "descriptor's value: %d\n",
		    cfgno, inum, asnum, n, plural(n), num_ep_orig);
	return buffer - buffer0;

skip_to_next_interface_descriptor:
	i = find_next_descriptor(buffer, size, USB_DT_INTERFACE,
	    USB_DT_INTERFACE, NULL);
	return buffer - buffer0 + i;
}

/***********
把获得的配置等信息按照类型进行分类
获取得到的信息里它包含了多种信息：配置，相关联接口，接口，端口等；
这个函数的作用就是将这些信息把它补全到之前申请的usb_host_config结构里，把各类信息放到相应的结构中
*****************/
static int usb_parse_configuration(struct usb_device *dev, int cfgidx,
    struct usb_host_config *config, unsigned char *buffer, int size)
{
	struct device *ddev = &dev->dev;
	unsigned char *buffer0 = buffer;
	int cfgno;
	int nintf, nintf_orig;
	int i, j, n;
	struct usb_interface_cache *intfc;
	unsigned char *buffer2;
	int size2;
	struct usb_descriptor_header *header;
	int len, retval;
	u8 inums[USB_MAXINTERFACES], nalts[USB_MAXINTERFACES];
	unsigned iad_num = 0;

	memcpy(&config->desc, buffer, USB_DT_CONFIG_SIZE); //获取得到的信息最前面就是配置信息，所以在这里根据配置信息长度把配置描述符信息保存到usb_host_config中的desc中
	if (config->desc.bDescriptorType != USB_DT_CONFIG || // 如果获得的信息最前面不是配置描述符信息或描述符信息长度少于标准数据长度，则出错，直接返回
	    config->desc.bLength < USB_DT_CONFIG_SIZE) {
		dev_err(ddev, "invalid descriptor for config index %d: "
		    "type = 0x%X, length = %d\n", cfgidx,
		    config->desc.bDescriptorType, config->desc.bLength);
		return -EINVAL;
	}
	cfgno = config->desc.bConfigurationValue;   //  从配置描述符里得到当前配置值

	buffer += config->desc.bLength;      //  更新buffer地址和size大小，从而可以获得接下来的信息
	size -= config->desc.bLength;

	nintf = nintf_orig = config->desc.bNumInterfaces;  //  取得当前配置里接口个数，如果接口个数大于允许的最大接口数据，发出警告，并重新设置设备接口数
	if (nintf > USB_MAXINTERFACES) {
		dev_warn(ddev, "config %d has too many interfaces: %d, "
		    "using maximum allowed: %d\n",
		    cfgno, nintf, USB_MAXINTERFACES);
		nintf = USB_MAXINTERFACES;
	}

	/* Go through the descriptors, checking their length and counting the
	 * number of altsettings for each interface 
     遍历当前配置信息，统计得到当前配置下每个接口对应的接口设置数
	 其中n用来表示接口数，inums和nalts数组用来分别存放接口号及该接口号对应的接口设置数
	 */
	n = 0;  
	for ((buffer2 = buffer, size2 = size);
	      size2 > 0;
	     (buffer2 += header->bLength, size2 -= header->bLength)) {

		if (size2 < sizeof(struct usb_descriptor_header)) {    //  根据usb的描述符头里的长度对buffer里的数据进行判断，判断是否满足长度需求，
			dev_warn(ddev, "config %d descriptor has %d excess "  //  如果不满足，则跳出当前统计循环
			    "byte%s, ignoring\n",
			    cfgno, size2, plural(size2));
			break;
		}
         //  USB3.0协议引进了一个interrface association descriptor概念，它用来描述一个复合型接口，复合型接口是指由2个或以上接口组成一个功能的接口。
         //   如果存在这类描述符，则它是紧跟着配置描述符的
		header = (struct usb_descriptor_header *) buffer2;
		if ((header->bLength > size2) || (header->bLength < 2)) {
			dev_warn(ddev, "config %d has an invalid descriptor "
			    "of length %d, skipping remainder of the config\n",
			    cfgno, header->bLength);
			break;
		}

		if (header->bDescriptorType == USB_DT_INTERFACE) {//如果配置描述符后紧跟着的是接口描述符,则根据接口描述符对长度和接口号进行判断，判断这个接口描述符是否合法
			struct usb_interface_descriptor *d;
			int inum;

			d = (struct usb_interface_descriptor *) header;
			if (d->bLength < USB_DT_INTERFACE_SIZE) {
				dev_warn(ddev, "config %d has an invalid "
				    "interface descriptor of length %d, "
				    "skipping\n", cfgno, d->bLength);
				continue;
			}

			inum = d->bInterfaceNumber;

			if ((dev->quirks & USB_QUIRK_HONOR_BNUMINTERFACES) &&
			    n >= nintf_orig) {
				dev_warn(ddev, "config %d has more interface "
				    "descriptors, than it declares in "
				    "bNumInterfaces, ignoring interface "
				    "number: %d\n", cfgno, inum);
				continue;
			}

			if (inum >= nintf_orig)
				dev_warn(ddev, "config %d has an invalid "
				    "interface number: %d but max is %d\n",
				    cfgno, inum, nintf_orig - 1);

			/* Have we already encountered this interface?
			 * Count its altsettings */
			for (i = 0; i < n; ++i) {  //  遍历inums数组，判断接口描述符中接口号是否已经存在inums数组里
				if (inums[i] == inum)
					break;
			}
			 /******
			 如果在inums里找到和当前接口号相同项，并且这个接口对应的接口设置数量没有超过255，则增加接口设置数。
		     如果不能在inums里找到与当前接口与相同项，表示这是一个新的接口，如果接口数n没有超过最大接口数32，
			 则将当前接口号保存到inums数组里，并设置对应的接口设置数为1  
			        *********/ 
			if (i < n) {
				if (nalts[i] < 255)
					++nalts[i];
			} else if (n < USB_MAXINTERFACES) {
				inums[n] = inum;
				nalts[n] = 1;
				++n;
			}

		} else if (header->bDescriptorType ==
				USB_DT_INTERFACE_ASSOCIATION) {  //  如果当前描述符是interface association descriptor，则把它保存到配置结构intf_assoc里
			if (iad_num == USB_MAXIADS) {
				dev_warn(ddev, "found more Interface "
					       "Association Descriptors "
					       "than allocated for in "
					       "configuration %d\n", cfgno);
			} else {
				config->intf_assoc[iad_num] =
					(struct usb_interface_assoc_descriptor
					*)header;
				iad_num++;
			}
       //  在当前配置描述下，如果还有其它配置或设备描述符，则发出警告
		} else if (header->bDescriptorType == USB_DT_DEVICE ||
			    header->bDescriptorType == USB_DT_CONFIG)
			dev_warn(ddev, "config %d contains an unexpected "
			    "descriptor of type 0x%X, skipping\n",
			    cfgno, header->bDescriptorType);

	}	/* for ((buffer2 = buffer, size2 = size); ...) */
	size = buffer2 - buffer;   //   重新获取配置等描述符信息长度
	config->desc.wTotalLength = cpu_to_le16(buffer2 - buffer0);

	if (n != nintf)  //  对统计出来的接口数进行判断，如果n与配置描述中的接口数不相等或n等于0，则发生警告
		dev_warn(ddev, "config %d has %d interface%s, different from "
		    "the descriptor's value: %d\n",
		    cfgno, n, plural(n), nintf_orig);
	else if (n == 0)
		dev_warn(ddev, "config %d has no interfaces?\n", cfgno);
	config->desc.bNumInterfaces = nintf = n;

	/* Check for missing interface numbers */
	for (i = 0; i < nintf; ++i) {             //  对inums里的接口号进行检查，查看对就的接口号是否存在
		for (j = 0; j < nintf; ++j) {
			if (inums[j] == i)
				break;
		}
		if (j >= nintf)
			dev_warn(ddev, "config %d has no interface number "
			    "%d\n", cfgno, i);
	}

	/* Allocate the usb_interface_caches and altsetting arrays */
	for (i = 0; i < nintf; ++i) {   //  为当前接口和接口下的接口设置申请内存空间，并把它存放在配置里的intf_cache结构里，
		j = nalts[i];
		if (j > USB_MAXALTSETTING) {
			dev_warn(ddev, "too many alternate settings for "
			    "config %d interface %d: %d, "
			    "using maximum allowed: %d\n",
			    cfgno, inums[i], j, USB_MAXALTSETTING);
			nalts[i] = j = USB_MAXALTSETTING;
		}

		len = sizeof(*intfc) + sizeof(struct usb_host_interface) * j;   //  intf_cache本身 是一个指针数组，它用来存放接口和接口设置这个内存空间的地址
		config->intf_cache[i] = intfc = kzalloc(len, GFP_KERNEL);
		if (!intfc)
			return -ENOMEM;
		kref_init(&intfc->ref);
	}

	/* FIXME: parse the BOS descriptor */

	/* Skip over any Class Specific or Vendor Specific descriptors;
	 * find the first interface descriptor
	通过find_next_descriptor函数从buffer里查找类型为接口的描述符，返回值i表示buffer与接口描述符描述符之间数据长度，
	n用来表示buffer与接口描述符之间需要跳过的其它描述符个数。buffer最开始的地方有可能不是接口描述符，
	所以要去寻找第一次出现接口描述符类型的位置，这个位置可以由返回值来确定，即接口描述符开始地址=buffer+返回值i
	 */
	config->extra = buffer;
	i = find_next_descriptor(buffer, size, USB_DT_INTERFACE,
	    USB_DT_INTERFACE, &n);
	config->extralen = i;
	if (n > 0)
		dev_dbg(ddev, "skipped %d descriptor%s after %s\n",
		    n, plural(n), "configuration");
	buffer += i;  //  如果在接口描述符前面还存在其它数据，则更新buffer和size,以确保接下来buffer就为接口描述符首地址
	size -= i;

	/* Parse all the interface/altsetting descriptors 
	   接下来由usb_parse_interface函数对buffer里的接口和端口等信息进行分解  */
	while (size > 0) {
		retval = usb_parse_interface(ddev, cfgno, config,
		    buffer, size, inums, nalts);
		if (retval < 0)
			return retval;

		buffer += retval;
		size -= retval;
	}

	/* Check for missing altsettings */
	for (i = 0; i < nintf; ++i) {
		intfc = config->intf_cache[i];
		for (j = 0; j < intfc->num_altsetting; ++j) {
			for (n = 0; n < intfc->num_altsetting; ++n) {
				if (intfc->altsetting[n].desc.
				    bAlternateSetting == j)
					break;
			}
			if (n >= intfc->num_altsetting)
				dev_warn(ddev, "config %d interface %d has no "
				    "altsetting %d\n", cfgno, inums[i], j);
		}
	}

	return 0;
}

/* hub-only!! ... and only exported for reset/reinit path.
 * otherwise used internally on disconnect/destroy path
 */
void usb_destroy_configuration(struct usb_device *dev)
{
	int c, i;

	if (!dev->config)
		return;

	if (dev->rawdescriptors) {
		for (i = 0; i < dev->descriptor.bNumConfigurations; i++)
			kfree(dev->rawdescriptors[i]);

		kfree(dev->rawdescriptors);
		dev->rawdescriptors = NULL;
	}

	for (c = 0; c < dev->descriptor.bNumConfigurations; c++) {
		struct usb_host_config *cf = &dev->config[c];

		kfree(cf->string);
		for (i = 0; i < cf->desc.bNumInterfaces; i++) {
			if (cf->intf_cache[i])
				kref_put(&cf->intf_cache[i]->ref,
					  usb_release_interface_cache);
		}
	}
	kfree(dev->config);
	dev->config = NULL;
}


/*
 * Get the USB config descriptors, cache and parse'em
 *
 * hub-only!! ... and only in reset path, or usb_new_device()
 * (used by real hubs and virtual root hubs)
 *
 * NOTE: if this is a WUSB device and is not authorized, we skip the
 *       whole thing. A non-authorized USB device has no
 *       configurations.
 这个函数主要分成二部分，第一部分是向usb device侧获取设备的配置，接口，端口信息，这部分由第661-739行完成，
 首先，它为这些信息申请存放空间，然后像之前获取设备描述符一样，先发送一个9 个字节的请求，用来获取配置，接口，端口等描述符的总长度，
 最后根据得到的总长度去得到完整 的设备配置，接口，端口信息；第二部分是把获取到的这个信息按照类别分别进行分类，并存到相应的结构中
 */
int usb_get_configuration(struct usb_device *dev)
{
	struct device *ddev = &dev->dev;
	int ncfg = dev->descriptor.bNumConfigurations;
	int result = 0;
	unsigned int cfgno, length;
	unsigned char *bigbuffer;
	struct usb_config_descriptor *desc;

	cfgno = 0;
	if (dev->authorized == 0)	/* Not really an error */
		goto out_not_authorized;  //   如果usb设备没有得到授权，则直接退出
	result = -ENOMEM;
	if (ncfg > USB_MAXCONFIG) {         //   如果usb设备的配置个数大于最大允许配置数8，则发出警告，并把设备的配置个数改成最大允许配置数8
		dev_warn(ddev, "too many configurations: %d, "
		    "using maximum allowed: %d\n", ncfg, USB_MAXCONFIG);
		dev->descriptor.bNumConfigurations = ncfg = USB_MAXCONFIG;
	}

	if (ncfg < 1) {    //   如果usb设备没有配置，则不允许，直接返回错误信息
		dev_err(ddev, "no configurations\n");
		return -EINVAL;
	}

	length = ncfg * sizeof(struct usb_host_config);  //   根据配置数，申请usb配置结构usb_host_config,并为用于存放配置结构的指针数据申请空间
	dev->config = kzalloc(length, GFP_KERNEL);
	if (!dev->config)
		goto err2;

	length = ncfg * sizeof(char *);
	dev->rawdescriptors = kzalloc(length, GFP_KERNEL);  //   申请用于暂时存放配置描述符的结构空间
	if (!dev->rawdescriptors)
		goto err2;

	desc = kmalloc(USB_DT_CONFIG_SIZE, GFP_KERNEL);
	if (!desc)
		goto err2;

	result = 0;
	for (; cfgno < ncfg; cfgno++) {                       //    依次获取usb设备的配置信息
		/* We grab just the first descriptor so we know how long
		 * the whole configuration is 
		 由于接口，端口，产家专有等描述符信息是直接跟在配置描述符后面的，配置描述符进里有一项wTotalLength，
		 专门用来表示配置，接口，端口，产品和产家专有等描述符的总长度，所以它先获取配置描述符信息，从而得到总长度；
		 wTotallLength在配置描述符里的第3-4个字节位置，所以在获取配置描述时，只要得到的数据大于等于4个字节就可以，但如果少于4字节，就表示出错
		 */
		result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno,
		    desc, USB_DT_CONFIG_SIZE);
		if (result < 0) {
			dev_err(ddev, "unable to read config index %d "
			    "descriptor/%s: %d\n", cfgno, "start", result);
			dev_err(ddev, "chopping to %d config(s)\n", cfgno);
			dev->descriptor.bNumConfigurations = cfgno;
			break;
		} else if (result < 4) {
			dev_err(ddev, "config index %d descriptor too short "
			    "(expected %i, got %i)\n", cfgno,
			    USB_DT_CONFIG_SIZE, result);
			result = -EINVAL;
			goto err;
		}
		length = max((int) le16_to_cpu(desc->wTotalLength),   //   得到wTotallLength
		    USB_DT_CONFIG_SIZE);

		/* Now that we know the length, get the whole thing */
		bigbuffer = kmalloc(length, GFP_KERNEL);  //  根据得到的wTotallLength，申请用于存放这些信息的内存空间
		if (!bigbuffer) {
			result = -ENOMEM;
			goto err;
		}
		//  只是向设备发送一个获取配置信息的请求，相对比较简单，所以就不深入讲解，
		result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno,  //根据得到的wTotallLength，去获取完整的配置，接口，端口等描述符信息,把这些信息存放到bigbuffer里
		    bigbuffer, length);
		if (result < 0) {
			dev_err(ddev, "unable to read config index %d "
			    "descriptor/%s\n", cfgno, "all");
			kfree(bigbuffer);
			goto err;
		}
		if (result < length) {
			dev_warn(ddev, "config index %d descriptor too short "
			    "(expected %i, got %i)\n", cfgno, length, result);
			length = result;
		}

		dev->rawdescriptors[cfgno] = bigbuffer;   //  把用于存放配置等信息的地址保存在之前申请的rawdescriptors里

		result = usb_parse_configuration(dev, cfgno,  //  通过usb_parse_configuration函数，把获得的配置等信息按照类型进行分类
		    &dev->config[cfgno], bigbuffer, length);  // 这里的usb_get_descriptor，只是向设备发送一个获取配置信息的请求，相对比较简单，所以就不深入讲解，
		if (result < 0) {                             // 而usb_parse_configuration函数却比较复杂，这里会对它进一步讲解
			++cfgno;
			goto err;
		}
	}
	result = 0;

err:
	kfree(desc);
out_not_authorized:
	dev->descriptor.bNumConfigurations = cfgno;
err2:
	if (result == -ENOMEM)
		dev_err(ddev, "out of memory\n");
	return result;
}

void usb_release_bos_descriptor(struct usb_device *dev)
{
	if (dev->bos) {
		kfree(dev->bos->desc);
		kfree(dev->bos);
		dev->bos = NULL;
	}
}

/* Get BOS descriptor set */
int usb_get_bos_descriptor(struct usb_device *dev)
{
	struct device *ddev = &dev->dev;
	struct usb_bos_descriptor *bos;
	struct usb_dev_cap_header *cap;
	unsigned char *buffer;
	int length, total_len, num, i;
	int ret;

	bos = kzalloc(sizeof(struct usb_bos_descriptor), GFP_KERNEL);
	if (!bos)
		return -ENOMEM;

	/* Get BOS descriptor */
	ret = usb_get_descriptor(dev, USB_DT_BOS, 0, bos, USB_DT_BOS_SIZE);
	if (ret < USB_DT_BOS_SIZE) {
		dev_err(ddev, "unable to get BOS descriptor\n");
		if (ret >= 0)
			ret = -ENOMSG;
		kfree(bos);
		return ret;
	}

	length = bos->bLength;
	total_len = le16_to_cpu(bos->wTotalLength);
	num = bos->bNumDeviceCaps;
	kfree(bos);
	if (total_len < length)
		return -EINVAL;

	dev->bos = kzalloc(sizeof(struct usb_host_bos), GFP_KERNEL);
	if (!dev->bos)
		return -ENOMEM;

	/* Now let's get the whole BOS descriptor set */
	buffer = kzalloc(total_len, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto err;
	}
	dev->bos->desc = (struct usb_bos_descriptor *)buffer;

	ret = usb_get_descriptor(dev, USB_DT_BOS, 0, buffer, total_len);
	if (ret < total_len) {
		dev_err(ddev, "unable to get BOS descriptor set\n");
		if (ret >= 0)
			ret = -ENOMSG;
		goto err;
	}
	total_len -= length;

	for (i = 0; i < num; i++) {
		buffer += length;
		cap = (struct usb_dev_cap_header *)buffer;
		length = cap->bLength;

		if (total_len < length)
			break;
		total_len -= length;

		if (cap->bDescriptorType != USB_DT_DEVICE_CAPABILITY) {
			dev_warn(ddev, "descriptor type invalid, skip\n");
			continue;
		}

		switch (cap->bDevCapabilityType) {
		case USB_CAP_TYPE_WIRELESS_USB:
			/* Wireless USB cap descriptor is handled by wusb */
			break;
		case USB_CAP_TYPE_EXT:
			dev->bos->ext_cap =
				(struct usb_ext_cap_descriptor *)buffer;
			break;
		case USB_SS_CAP_TYPE:
			dev->bos->ss_cap =
				(struct usb_ss_cap_descriptor *)buffer;
			break;
		case CONTAINER_ID_TYPE:
			dev->bos->ss_id =
				(struct usb_ss_container_id_descriptor *)buffer;
			break;
		default:
			break;
		}
	}

	return 0;

err:
	usb_release_bos_descriptor(dev);
	return ret;
}
