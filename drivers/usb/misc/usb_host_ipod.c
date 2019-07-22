
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include "sdk.h"
#include "usb_host_ipod.h"
#include "usb.h"


static void ipod_draw_down(struct usb_carplay *dev);

static void ipod_delete(struct kref *kref);

static int ipod_init(struct usb_carplay *dev);

static int ipod_deinit(struct usb_carplay *dev);

static int ipod_open(struct inode *inode, struct file *file);

static int ipod_release(struct inode *inode, struct file *file);

static int ipod_close(struct usb_carplay *dev);

static int ipod_flush(struct file *file, fl_owner_t id);


//static int ipod_ioctl(struct inode *indeo, struct file *file,unsigned int cmd, unsigned long arg);
static long ipod_ioctl(struct file *file,unsigned int cmd, unsigned long arg);

//static void ipod_read_int_callback(struct urb *urb);

//static int ipod_do_read_io(struct usb_ipod *dev, size_t count);

static ssize_t ipod_read(struct file *file, char *buffer, size_t count,loff_t *ppos);

ssize_t ipod_write_base(struct usb_carplay *dev, char *buf,size_t count);

//ssize_t ipod_read_base(struct usb_carplay*dev,char *buffer,size_t count);

//static void ipod_write_ctrl_callback(struct urb *urb);

static ssize_t ipod_write(struct file *file, const char *user_buffer,size_t count, loff_t *ppos);

static int ipod_read_thread(void *data);

static void ipod_disconnect(struct usb_interface *interface);

//static int ipod_analyse_thread(void *data);

//static int ipod_analyse_audio_cmd(struct usb_ipod *dev,unsigned char* pBuf,unsigned long uBufLen);

//static ssize_t ipod_read_data(struct usb_carplay *dev,char *buffer,size_t count);

static ssize_t ipod_write_data(struct usb_carplay *dev,struct urb *urb,char *buffer,size_t count,size_t add_zero_length,unsigned int report_id,int seq_ops);

static int calc_write_report_id(struct usb_carplay *dev,size_t bufferLen,unsigned int *byteReportId,size_t *dwWriteLen,size_t *dwAddZero);

static int calc_write_seq_pos(struct usb_carplay *dev,size_t dwWriteNum,size_t dwLenLeft,unsigned int *byteSeqPos);

static int get_and_analyse_report_desp(struct usb_carplay *dev);

//static unsigned char cal_check_parity(const unsigned char* buf,int bufLen);

//static int tx_acc_ack(struct usb_ipod *dev,unsigned char bCmd,unsigned char bStatus,unsigned int uTransID);

//static int tx_dev_samplerate_to_ipod(struct usb_ipod *dev,unsigned int uTransID);

//static int rx_new_ipod_track_info(struct usb_ipod *dev,unsigned char *buf,int bufLen,unsigned int uTransID);

//static int rx_accessory_info(struct usb_ipod *dev,unsigned char * buf,int bufLen);

//static int analyse_accessory_info_cmd(struct usb_ipod *dev,unsigned char *pBuf,unsigned long uBufLen);
extern struct usb_driver cpy_driver;


static int ipod_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_carplay*dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	ipod_draw_down(dev);
	return 0;
}

static int ipod_resume(struct usb_interface *intf)
{
	return 0;
}

static int ipod_pre_reset(struct usb_interface *intf)
{
	struct usb_carplay *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	ipod_draw_down(dev);

	return 0;
}

static int ipod_post_reset(struct usb_interface *intf)
{
	struct usb_carplay *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static int ipod_open(struct inode *inode, struct file *file)
{
    struct usb_carplay *dev;
    struct usb_interface *interface;
    int subminor;
    int retval = 0;
    DEBUG = 1;
    if(DEBUG)printk(">>>>iPodhid:APD_Open +\n");
    subminor = iminor(inode);

    interface = usb_find_interface(&cpy_driver, subminor);
    if (!interface) {
        err("%s - error, can't find device for minor %d",
                    __func__, subminor);
        printk("%s - error, can't find device for minor %d",
                    __func__, subminor);
        retval = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(interface);
    if (!dev) {
    	printk("+++++%s+++++, dev = NULL\n", __func__);
        retval = -ENODEV;
        goto exit;
    }
//#ifdef SUPPORT_IP
//    if(dev->bAuthProcess!=AUTH_OK && dev->bAuthProcess!=AUTH_NG){
//        printk(">>>>iPodhid:APD_Open NG1\n");//认证过程中调用ipod open
//        retval = -EFAULT;
//        goto exit;
//    }
    //if(ipod_init(dev)<0)
    //    goto exit;
//#endif
    if(dev->m_flags.UnloadPending || !dev->m_flags.bThreadActive){
        printk(">>>>iPodhid:APD_Open not ready\n");
        retval = -EFAULT;
        goto exit;
    }
    if(dev->m_flags.Open < 0){
        if(DEBUG)printk(">>>>>>>>>memset to 0\n");
        memset(dev->m_read_buf,0,HID_READ_BUF_STROTE_SIZE);
        memset(dev->m_analyse_buf,0,HID_READ_BUF_STROTE_SIZE);
        memset(dev->m_read_tmp_buf,0,IPOD_RX_DATA_LENGTH_MAX);
        memset(dev->m_analyse_tmp_buf,0,HID_READ_BUF_STROTE_SIZE+1);
        dev->m_unread_buf_count = 0;
        dev->m_unanalyse_buf_count = 0;
    }
    dev->m_flags.Open ++;

    /* increment our usage count for the device */
    kref_get(&dev->kref);

    /* lock the device to allow correctly handling errors
     * in resumption */
    mutex_lock(&dev->io_mutex);

    if (!dev->open_count++) {
        retval = usb_autopm_get_interface(interface);
        if (retval) {
            dev->open_count--;
            mutex_unlock(&dev->io_mutex);
            kref_put(&dev->kref, ipod_delete);
            goto exit;
        }
    } /* else { //uncomment this block if you want exclusive open
         retval = -EBUSY;
         dev->open_count--;
         mutex_unlock(&dev->io_mutex);
         kref_put(&dev->kref, ipod_delete);
         goto exit;
         } */
    /* prevent the device from being autosuspended */

    /* save our object in the file's private structure */
    file->private_data = dev;
    mutex_unlock(&dev->io_mutex);
    printk(">>>>iPodhid:APD_Open -\n");

exit:
    return retval;
}



static void ipod_draw_down(struct usb_carplay *dev)
{
	//int time;
	//time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	//if (!time)
	//	usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->int_in_urb);
	usb_kill_urb(dev->ctrl_in_urb);
}

static int ipod_release(struct inode *inode, struct file *file)
{
    struct usb_carplay *dev;
    DEBUG = 1;
    if(DEBUG)printk(">>>>iPodhid:APD_Close +\n");
    dev = (struct usb_carplay *)file->private_data;
    if (dev == NULL)
      return -ENODEV;
    dev->m_flags.Open --;
    ipod_close(dev);
    //mutex_lock(&dev->io_mutex);
    /* allow the device to be autosuspended */
    //if (!--dev->open_count && dev->interface)
    //  usb_autopm_put_interface(dev->interface);
    //mutex_unlock(&dev->io_mutex);
    printk(">>>>iPodhid:APD_Close -\n");

    //ipod_deinit(dev);
    /* decrement the count on our device */
    return 0;
}


static int ipod_flush(struct file *file, fl_owner_t id)
{
	struct usb_carplay *dev;
	dev = (struct usb_carplay *)file->private_data;
	if (dev == NULL)
		return -ENODEV;
    complete(&dev->read_completion);
    return 0;
}

static int calc_write_report_id(struct usb_carplay *dev,size_t bufferLen,unsigned int *byteReportId,size_t *dwWriteLen,size_t *dwAddZero)
{
    bool bIsHaveReportId=0;//是否有合适的report id
    int i;
    *dwAddZero=0;//补充零的个数
    for (i=0;i<dev->m_ipod_write_report_size;i++)
    {
        if ((bufferLen+2)<=dev->m_ipod_write_report[i].count)
        {
            *byteReportId=dev->m_ipod_write_report[i].id;
            *dwWriteLen=bufferLen;
            bIsHaveReportId=1;//是否有合适的report id
            *dwAddZero=dev->m_ipod_write_report[i].count-(*dwWriteLen+2);//补充零的个数
            break;
        }
    }
    if (bIsHaveReportId==0)//是否有合适的report id
    {
        *byteReportId=dev->m_ipod_write_report[dev->m_ipod_write_report_size-1].id;
        *dwWriteLen=dev->m_ipod_write_report[dev->m_ipod_write_report_size-1].count-2;
        *dwAddZero=0;//补充零的个数
    }
    return 0;
}


static int calc_write_seq_pos(struct usb_carplay *dev,size_t dwWriteNum,size_t dwLenLeft,unsigned int *byteSeqPos)
{
    if (dwWriteNum==0&&dwLenLeft==0)
    {
        *byteSeqPos=REP_SEQ_POS_ONLY;//只有一个包
    }
    else if(dwWriteNum==0)
    {
        *byteSeqPos=REP_SEQ_POS_START;//起始包
    }
    else if (dwLenLeft==0)
    {
        *byteSeqPos=REP_SEQ_POS_END;//结束包
    }
    else
    {
        *byteSeqPos=REP_SEQ_POS_MID;//中间包
    }
    //if(DEBUG)printk("calc_write_seq_pos byteSeqPos = %d\n",*byteSeqPos);
    return 0;
}

static void ipod_read_ctrl_callback(struct urb *urb)
{
	struct usb_carplay*dev;
	DEBUG = 1;
    if(DEBUG)printk("ipod_read_ctrl_callback + \n");
	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("%s - nonzero write int status received: %d",
			    __func__, urb->status);
        dev->int_in_filled = 0;
		//dev->errors = urb->status;
	} else {
		dev->int_in_filled = urb->actual_length;
	}

	complete(&dev->ctrl_in_completion);
    if(DEBUG)printk("ipod_read_ctrl_callback - \n");
}

static int get_and_analyse_report_desp(struct usb_carplay*dev)
{
	printk("+++enter %s+++++\n", __func__);
    char tempPuchar[le16_to_cpu(dev->udev->actconfig->desc.wTotalLength)];
    char* byteReportDescri;
    struct usb_ctrlrequest control_header;
    //Anylyse HID Report Descriptor
    int dwAnalysed=0;//已经解析的字节数
    unsigned char bSize=0;
    unsigned int uReportId=0;
    unsigned uReportCount=0;
    int retval,i;
    DEBUG = 1;
    if(DEBUG)printk(">>>>iPodhid:>APD_GetReport\n");

    //get description length
    if(DEBUG)printk("IOCTL_GET_HIDREPORT_LENGTH\n");
    dev->wTotalLength_Des = le16_to_cpu(dev->udev->actconfig->desc.wTotalLength);
    if(DEBUG)printk(">>>>iPodhid:>APD_GetReport dev->wTotalLength_Des = %d\n",dev->wTotalLength_Des);
    memset(&control_header,0,sizeof(struct usb_ctrlrequest));
    control_header.bRequestType = USB_DIR_IN; //USB_REQUEST_DEVICE_TO_HOST;
    control_header.bRequest = USB_REQ_GET_DESCRIPTOR; //USB_REQUEST_GET_DESCRIPTOR; 0x06
    control_header.wValue   = USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(USB_DT_CONFIG, (dev->udev->actconfig->desc.bConfigurationValue-1));//0x0200;
    control_header.wIndex   = 0;
    control_header.wLength  = dev->wTotalLength_Des;
    /* initialize the urb properly */
    if(DEBUG)printk(">>>>iPodhid:>APD_GetReport: usb_fill_control_urb+\n");
    usb_fill_control_urb(dev->ctrl_in_urb,
                dev->udev,
                usb_rcvctrlpipe(dev->udev, 0),
                (unsigned char *)&control_header,
                tempPuchar,
                dev->wTotalLength_Des,
                ipod_read_ctrl_callback, dev);

    /* send the data out the ctrl port */
//    dev->ctrl_in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    retval = usb_submit_urb(dev->ctrl_in_urb, GFP_KERNEL);
    if (retval < 0) {
        err("%s - failed submitting read ctrl_in_urb, error %d", __func__,retval);
        goto error;
    }
    if(DEBUG)printk(">>>>iPodhid:>APD_GetReport: usb_fill_control_urb-\n");
    P;
    wait_for_completion(&dev->ctrl_in_completion);
    P;
    //wait_for_completion_timeout(&dev->ctrl_in_completion);
    if(DEBUG)printk("dev->Lenght_HID= %d\n",dev->Lenght_HID);
    for(i=0; i< dev->wTotalLength_Des-9; i++)
    {
        //if(DEBUG)printk("tempPuchar[%d]= 0x%x ",i,tempPuchar[i]);
        //if(i%4==0)printk("\n");
        if((tempPuchar[i]==0x09)&&(tempPuchar[i+1]==HID_HID_DESCRIPTOR_TYPE)) //
        {
            //if(DEBUG)printk("-->tempPuchar[%d]= 0x%x\n",i,tempPuchar[i]);
            dev->Lenght_HID = tempPuchar[i+8]*0xff + tempPuchar[i+7]; // 得到HID report描述符总长度
            break;
        }
    }
    if(DEBUG)printk("dev->Lenght_HID= %d\n",dev->Lenght_HID);
    if(dev->Lenght_HID == 0){
        retval = -1;
        goto error;
    }
    //get report id and count
    if(DEBUG)printk("IOCTL_GET_DEVICE_HIDREPORT\n");

    byteReportDescri = (char*)kmalloc(dev->Lenght_HID,GFP_KERNEL);
    memset(&control_header,0,sizeof(struct usb_ctrlrequest));
    //control_header.bRequestType = USB_DIR_IN;//0x81
    control_header.bRequestType = 0x81;
    control_header.bRequest = USB_REQ_GET_DESCRIPTOR;//USB_REQUEST_GET_DESCRIPTOR; 0x06
    control_header.wValue   = USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(HID_REPORT_DESCRIPTOR_TYPE,0);
    control_header.wIndex   = HID_REPORT_OUTPUT;
    control_header.wLength  = dev->Lenght_HID;
    /* initialize the urb properly */
    usb_fill_control_urb(dev->ctrl_in_urb,
                dev->udev,
                usb_rcvctrlpipe(dev->udev, 0),
                (unsigned char *)&control_header,
                byteReportDescri,
                dev->Lenght_HID,
                ipod_read_ctrl_callback, dev);

    /* send the data out the int port */
    //dev->ctrl_in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    retval = usb_submit_urb(dev->ctrl_in_urb, GFP_KERNEL);
    if (retval < 0) {
        err("%s - failed submitting read ctrl_in_urb, error %d", __func__,retval);
        kfree(byteReportDescri);
        byteReportDescri=NULL;
        goto error;
    }
    P;
    wait_for_completion(&dev->ctrl_in_completion);
    P;
    //wait_for_completion_timeout(&dev->ctrl_in_completion);
    if(1){
        for(i=0;i<dev->Lenght_HID;i++){
            printk("byteReportDescri[%d] = 0x%x\t ",i,byteReportDescri[i]);
            if(i%4==0)printk("\n");
        }
        printk("\n");
    }
    //Usage Page
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0x04)//0000 01nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=((byteReportDescri[dwAnalysed]&0x03)+1);
    //Usage
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0x08)//0000 10nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=((byteReportDescri[dwAnalysed]&0x03)+1);
    //Collection
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0xA0)//1010 00nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=2;
    //Report Size
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0x74)//0111 01nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=((byteReportDescri[dwAnalysed]&0x03)+1);
    //Logical Maximum
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0x24)//0010 01nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=((byteReportDescri[dwAnalysed]&0x03)+1);
    //Logical Minimum
    if ((byteReportDescri[dwAnalysed]&0xFC)!=0x14)//0001 01nn
    {
        kfree(byteReportDescri);
        return -1;
    }
    dwAnalysed+=((byteReportDescri[dwAnalysed]&0x03)+1);


    while(dwAnalysed<((int)(dev->Lenght_HID)-1))
    {
        //Usage
        if ((byteReportDescri[dwAnalysed]&0xFC)!=0x08)//0000 10nn
        {
            kfree(byteReportDescri);
            return -1;
        }
        bSize=byteReportDescri[dwAnalysed]&0x03;
        dwAnalysed+=(bSize+1);
        //Report ID
        if ((byteReportDescri[dwAnalysed]&0xFC)!=0x84)//1000 01nn
        {
            kfree(byteReportDescri);
            return -1;
        }
        bSize=byteReportDescri[dwAnalysed]&0x03;
        if (bSize==1)
        {
            uReportId=byteReportDescri[dwAnalysed+1];
        }
        dwAnalysed+=(bSize+1);
        //Report Count
        if ((byteReportDescri[dwAnalysed]&0xFC)!=0x94)//1001 01nn
        {
            kfree(byteReportDescri);
            return -1;
        }
        bSize=byteReportDescri[dwAnalysed]&0x03;
        if (bSize==1)
        {
            uReportCount=byteReportDescri[dwAnalysed+1]+1;
        }
        else if (bSize==2)
        {
            uReportCount=((byteReportDescri[dwAnalysed+2]<<8)|(byteReportDescri[dwAnalysed+1]))+1;
        }
        dwAnalysed+=(bSize+1);
        //Input or Output
        if ((byteReportDescri[dwAnalysed]&0xFC)==0x80)//1000 00nn(Input)
        {
            //read report,input
            dev->m_ipod_read_report[dev->m_ipod_read_report_size].id=uReportId;
            dev->m_ipod_read_report[dev->m_ipod_read_report_size].count=uReportCount;
            if(DEBUG)printk("read report:size = %d, id=0x%x,Count=%d\n",
                            dev->m_ipod_read_report_size,
                            dev->m_ipod_read_report[dev->m_ipod_read_report_size].id,
                            dev->m_ipod_read_report[dev->m_ipod_read_report_size].count);
            //HID report descriptor size
            dev->m_ipod_read_report_size++;
        }
        else if ((byteReportDescri[dwAnalysed]&0xFC)==0x90)//1001 00nn(Output)
        {
            //write report,output
            dev->m_ipod_write_report[dev->m_ipod_write_report_size].id=uReportId;
            dev->m_ipod_write_report[dev->m_ipod_write_report_size].count=uReportCount;
            if(DEBUG)printk("write report:size = %d, id=0x%x,Count=%d\n",
                        dev->m_ipod_write_report_size,
                        dev->m_ipod_write_report[dev->m_ipod_write_report_size].id,
                        dev->m_ipod_write_report[dev->m_ipod_write_report_size].count);
            //HID report descriptor size
            dev->m_ipod_write_report_size++;
        }
        else
        {
            kfree(byteReportDescri);
            return -1;
        }

        bSize=byteReportDescri[dwAnalysed]&0x03;
        dwAnalysed+=(bSize+1);
    }

    //End Collection
    if ((byteReportDescri[dwAnalysed]!=0xC0))
    {
        kfree(byteReportDescri);
        return -1;
    }

    kfree(byteReportDescri);
    //RETAILMSG(OUT_MSG, (TEXT(">>>>iPodhid:<APD_GetReport:1\r\n")));
    return retval;
error:
    return retval;

}


static ssize_t ipod_read(struct file *file, char *buffer, size_t count,
            loff_t *ppos)
{
	struct usb_carplay*dev;
    ssize_t  read_count = 0;
    char read_tmp[count];

    DEBUG = 1;
    dev = (struct usb_carplay *)file->private_data;
    /* if we cannot read at all, return EOF */
   // printk("+++%s+++, count=%d\n", __func__,count);
    if (!count)
      return 0;
    if(dev == NULL )
    {
        if(DEBUG)printk(">>>>iPodhid:APD_Read NG1\n");
        msleep(100);
        return -1;
    }

    read_count = mutex_lock_interruptible(&dev->read_mutex);
    if (read_count< 0)
    {
    	printk("Error, +++%s+++,read_cout = %d\n",  __func__,read_count);
    	goto exit;
    }

    if (!dev->interface) 
    {	
    	/* disconnect() was called */
        read_count = -ENODEV;
        printk("read_cout1 = %d\n", read_count);
        goto exit;
    }
    dev->isRead = 1;

	//mutex_lock(&dev->ipod_read_mutex);
	read_count = (dev->m_unread_buf_count > count) ? count : dev->m_unread_buf_count;

	//printk("read!!!data, +++%s+++,read_cout = %d, dev->m_unread_buf_count = %d\n",  __func__,read_count, dev->m_unread_buf_count);

	memcpy(read_tmp, dev->m_read_buf, read_count);

	if(dev->m_unread_buf_count > read_count)
	{
		dev->m_unread_buf_count -= read_count;
		memcpy(dev->m_read_buf, dev->m_read_buf + read_count, dev->m_unread_buf_count);
	}
	else
	{
		memset(dev->m_read_buf, 0, HID_READ_BUF_STROTE_SIZE);
		dev->m_unread_buf_count = 0;
	}

	if(read_count > 0)
      if (copy_to_user(buffer,read_tmp,read_count)){
          read_count = -EFAULT;
          printk("read_count3=%d\n", read_count);
          goto exit;
      }
exit:
    mutex_unlock(&dev->read_mutex);
    dev->isRead = 0;
#if 0
    if(read_count){
        int i,j;
        j = min(read_count,15);
        printk("Read Buf  :\n");
        for(i=0;i<j;i++)
          printk("0x%x ",read_tmp[i]);
        printk("\n");
    }
    printk(">>>>iPodhid:APD_Read - %d\n",read_count);
#endif
    return read_count;
    
}

static ssize_t ipod_write(struct file *file, const char *user_buffer,size_t count, loff_t *ppos){
    struct usb_carplay *dev;
    int retval = 0;
    size_t writesize = count;
   // size_t writesize = min(count, (size_t)MAX_TRANSFER);
   // char buf[writesize];
    char* pbuf = NULL;
    pbuf = (char*)kmalloc(writesize + 2,GFP_KERNEL);
	
    if(pbuf == NULL)
    {
        printk("error, malloc fail\n");
	return -1;
     }
    memset(pbuf, 0, writesize + 2);
    printk("writesize:%d, count=%d\n", writesize, count);
    //memset(buf, 0, sizeof(buf));
    dev = (struct usb_carplay *)file->private_data;
    if(dev == NULL /**|| dev->bAuthProcess!=AUTH_OK*/){
        if(DEBUG)printk(">>>>iPodhid:APD_Write NG1\n");
        msleep(100);
        return -1;
    }

    //printk(">>>>iPodhid:APD_Write +\n");
    /*
     * limit the number of URBs in flight to stop a user from using up all
     * RAM
     */
    /*
     * if (!(file->f_flags & O_NONBLOCK)) {
     *   if (down_interruptible(&dev->limit_sem)) {
     *       retval = -ERESTARTSYS;
     *       goto exit;
     *   }
     * } else {
     *   if (down_trylock(&dev->limit_sem)) {
     *       retval = -EAGAIN;
     *       goto exit;
     *   }
     *}
     */
    
    if (copy_from_user(pbuf, user_buffer, writesize)) {
        retval = -EFAULT;
        goto exit;
    }
    retval = ipod_write_base(dev,pbuf,writesize);
exit:
    //up(&dev->limit_sem);
	
    if(pbuf != NULL)
	kfree(pbuf);
    printk(">>>>iPodhid:APD_Write -\n");
    return retval;
}

ssize_t ipod_write_base(struct usb_carplay *dev, char *buf,size_t count)
{
    int retval = 0;
    struct urb *urb = NULL;
    int dwLenLeft=count;//剩下需要发送的字节数
    int dwNeedWriteLen=0;//此次需要写的字节数
    int dwWriteOkLen=0;//已经写入的字节数
    int dwWriteNum=0;//写的次数
    int dwAddZero=0;//补充零的个数
    unsigned int byteReportId=0x00;//report id
    unsigned int byteSeqPos=REP_SEQ_POS_ONLY;//序列的位置
    DEBUG = 0;
	//printk("+++%s+++, write count = %d\n", __func__, count);
    /* verify that we actually have some data to write */
    if (count == 0 || buf == NULL /**|| count > HID_WRITE_BUF_MAX*/){
        retval = -EFAULT;
        goto exit;
    }

    //spin_lock_irq(&dev->err_lock);
    retval = dev->errors;
    if (retval < 0) {
        /* any error is reported once */
        dev->errors = 0;
        /* to preserve notifications about reset */
        retval = (retval == -EPIPE) ? retval : -EIO;
    }
    //spin_unlock_irq(&dev->err_lock);
    if (retval < 0)
      goto error;

    /* create a urb, and a buffer for it, and copy the data to the urb */
    //urb = usb_alloc_urb(0, GFP_KERNEL);
    //if (!urb) {
    //    retval = -ENOMEM;
    //    goto error;
    //}

    /* this lock makes sure we don't submit URBs to gone devices */
    if (!dev->interface) {		/* disconnect() was called */
        //mutex_unlock(&dev->write_mutex);
        retval = -ENODEV;
        goto error;
    }
    mutex_lock(&dev->write_mutex);
    dev->isWrite = true;
    //write cmd to ipod with usb
    while (dwLenLeft>0)
    {
        //msleep(1);
        //计算write report id
        calc_write_report_id(dev,
                    dwLenLeft,
                    &byteReportId,
                    &dwNeedWriteLen,
                    &dwAddZero);
        if(DEBUG)printk("dwLenLeft=%d,byteReportId=0x%x,dwNeedWriteLen=%d,dwAddZero=%d\n",dwLenLeft,byteReportId,dwNeedWriteLen,dwAddZero);
        //计算剩下的数据长度
        dwLenLeft=dwLenLeft-dwNeedWriteLen;
        //计算序列的位置
        calc_write_seq_pos(dev,
                    dwWriteNum,
                    dwLenLeft,
                    &byteSeqPos);
        //写一条HID数据
        if (ipod_write_data(dev,
                        urb,
                        buf+dwWriteOkLen,
                        dwNeedWriteLen,
                        dwAddZero,
                        byteReportId,
                        byteSeqPos) < 0)
        {
            break;
        }
        dwWriteOkLen+=dwNeedWriteLen;//已经写入的字节数
        dwWriteNum++;//写的次数
    }
    retval=dwWriteOkLen;
    /*
     * release our reference to this urb, the USB core will eventually free
     * it entirely
     */
    dev->isWrite = false;
    mutex_unlock(&dev->write_mutex);
    //usb_free_urb(urb);
    return retval;
error:
    if (urb) {
        usb_free_urb(urb);
    }
exit:
    return retval;
}
EXPORT_SYMBOL(ipod_write_base);

static ssize_t ipod_write_data(struct usb_carplay *dev,struct urb *urb,char *buffer,size_t count,size_t add_zero_length,unsigned int report_id,int seq_ops){
    size_t bRc = 0;
    int retval = 0;
    int dwTempBufLen=count+add_zero_length+2;
    DEBUG = 0;
    if(DEBUG)printk("ipod write data\n");
    if (dev->m_flags.bThreadActive==0)
    {
        printk("Ipod disconnect return ipod_write_data\n");
        return -1;
    }
    if (buffer==NULL||count<=0)
    {
        printk(">>>>iPodhid:Invalid parameter\n");
        bRc = -1;
    }
    else
    {
        unsigned char  pbTempBuf[dwTempBufLen];
        memset(pbTempBuf,0,dwTempBufLen);
        pbTempBuf[0]=report_id;//HID report ID
        pbTempBuf[1]=seq_ops;//report seq postion
        memcpy(pbTempBuf+2,buffer,count);
        if (pbTempBuf==NULL||dwTempBufLen<=0)
        {
            bRc=-1;
        }
        else
        {
        	if(0)
        	{
        		if(DEBUG){
                int i;
                printk("ipod write: ");
                for(i=0;i<dwTempBufLen;i++)
                  printk(" 0x%x",pbTempBuf[i]);
                printk("\n");
            }
        	}

           // printk("++%s+++, write one HID start\n",__func__);
#if 1
            retval =usb_control_msg(dev->udev,
                        usb_sndctrlpipe(dev->udev, 0),
                        0x09,
                        USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                        USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(2,report_id),
                        HID_REPORT_OUTPUT,
                        pbTempBuf,
                        dwTempBufLen,
                        0);
            if (retval != dwTempBufLen) {
                if (retval >= 0) {
                    printk("ctrl_out, wlen %d (expected %d)\n",
                                retval, dwTempBufLen);
                    retval = -EBADMSG;
                }else{
                    printk("%s usb_control_msg error : %d \n",__func__,retval);
                }
                bRc = retval;
            }
#else
            struct usb_ctrlrequest control_header;
            control_header.bRequestType=USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
            control_header.bRequest = 0x09;//USB_REQ_SET_CONFIGURATION
            control_header.wValue   = USB_DESCRIPTOR_MAKE_TYPE_AND_INDEX(2,report_id);
            control_header.wIndex = HID_REPORT_OUTPUT;
            control_header.wLength  = (unsigned short)dwTempBufLen;

            /* initialize the urb properly */
            usb_fill_control_urb(urb,
                        dev->udev,
                        usb_sndctrlpipe(dev->udev, 0),
                        (unsigned char *)&control_header,
                        pbTempBuf,
                        dwTempBufLen,
                        ipod_write_ctrl_callback, dev);

            /* send the data out the int port */
            //usb_anchor_urb(urb, &dev->submitted);
            retval = usb_submit_urb(urb, GFP_KERNEL);
            if (retval < 0) {
                err("%s - failed submitting write urb, error %d", __func__,retval);
                //usb_unanchor_urb(urb);
                bRc = -1;
            }
            P;
            wait_for_completion(&dev->ctrl_out_completion);
            P;
            //wait_for_completion_timeout(&dev->ctrl_out_completion);
#endif
        }
    }
    if(DEBUG)printk("ipod write data end\n");
    return bRc;
}

static const struct file_operations ipod_fops = {
    .owner =	THIS_MODULE,
    .read =		ipod_read,
    .write =	ipod_write,
    .open =		ipod_open,
    //.ioctl =	ipod_ioctl,
    .unlocked_ioctl =	ipod_ioctl,
    .release =	ipod_release,
    .flush =	ipod_flush,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver ipod_class = {
    .name =		"ipod%d",
    .fops =		&ipod_fops,
    .minor_base =	USB_IPOD_MINOR_BASE,
};

static void ipod_delete(struct kref *kref)
{
	struct usb_carplay *dev = to_ipod_dev(kref);
    //ipod_deinit(dev);
    mutex_lock(&dev->io_mutex);
    printk("ipod delete\n");
    ipod_draw_down(dev);
	usb_free_urb(dev->int_in_urb);
	usb_free_urb(dev->ctrl_in_urb);
	usb_put_dev(dev->udev);
#if 0
    cdev_del (&dev->ipod_info.ipod_dev);
    device_destroy(dev->ipod_info.dev_class, MKDEV(dev->ipod_info.major, 0));
    class_destroy(dev->ipod_info.dev_class);
    unregister_chrdev_region (MKDEV(dev->ipod_info.major,0), 1);
#endif
    mutex_unlock(&dev->io_mutex);
    msleep(1000);
	kfree(dev);
    dev = NULL;
}


static int ipod_read_thread(void* data)
{
	struct usb_carplay *dev = (struct usb_carplay*)data;
    size_t need_len=dev->m_ipod_read_report[dev->m_ipod_read_report_size-1].count;
    int rv = 0,i,read_buf_count;
    int read_error_count = 0;

	int policy;
	//pthread_attr_t attr;
    //rv = pthread_attr_getschedpolicy (attr, &policy);
    //assert (rv == 0);
	//printk("rv = %d\n", rv);
    //switch (policy)
    //{
    //    case SCHED_FIFO:
    //        printf ("policy = SCHED_FIFO\n");
    //        break;
    //    case SCHED_RR:
    //        printf ("policy = SCHED_RR");
    //        break;
    //    case SCHED_OTHER:
    //        printf ("policy = SCHED_OTHER\n");
    //        break;
    //    default:
    //        printf ("policy = UNKNOWN\n");
    //        break; 
    //}

    struct sched_param param = {.sched_priority = 1};
    rv = sched_setscheduler(current, SCHED_RR, &param);
    printk("rv1 = %d\n", rv);

    //rv = sched_setscheduler(0, SCHED_RR, &param);
    
    
    DEBUG = 0; 
    if(DEBUG)printk(">>>>iPodhid Read Thread size = %d ,need_Len = %d\n\n",dev->m_ipod_read_report_size,need_len);
    while (1)
	{
		//printk("======ipod_read_thread=====\n");
		//printk("++%s++,dev->m_flags.bThreadActive=%d \n", __func__, dev->m_flags.bThreadActive);
		if (dev->m_flags.bThreadActive==0)//断开连接时，关闭读线程
		{
			goto back;
		}
        rv = usb_interrupt_msg(dev->udev,
                    usb_rcvintpipe(dev->udev,dev->int_in_endpointAddr),
                    dev->int_in_buffer,
                    need_len,
                    &read_buf_count,
                    0
                    );

        if(rv < 0){
            msleep(100);
            read_error_count ++;
            printk("usb_interrupt_msg error :%d  read_error_count = %d\n",rv,read_error_count);
            if(read_error_count > 20){
                usb_set_configuration(dev->udev, -1);
                usb_remove_device(dev->udev);
                goto back;
            }
            continue;
        }
        read_error_count = 0;
        if(0)printk("ipod_read_thread: read_buf_count = %d\n",read_buf_count);
        //if(dev->bAuthProcess==AUTH_OK){
        if(0){
            int j;
            //j = min(read_buf_count,30);
            j = read_buf_count;
            printk("-------->read buf: ");
            for(i=0;i<j;i++){
                printk("0x%x ",dev->int_in_buffer[i]);
                if(j%30 ==0)
                    printk("\n");
            }
            printk("\n");
        }
       // mutex_lock(&dev->ipod_analyse_mutex);
       // memcpy(dev->m_analyse_buf+dev->m_unanalyse_buf_count,dev->int_in_buffer,read_buf_count);

		int FrameLen = 0;
		
		for (i=0;i < dev->m_ipod_read_report_size;i++)
		{
			if (dev->int_in_buffer[0] == dev->m_ipod_read_report[i].id)
			{
				FrameLen = dev->m_ipod_read_report[i].count;
				break;
			}
		}
		//printk("++%s++, FrameLen = %d, read_buf_count=%d\n", __func__, FrameLen, read_buf_count);
		if(FrameLen == read_buf_count)
		{
			//FrameLen = dev->m_ipod_read_report[i].count - 2;
			 rv = mutex_lock_interruptible(&dev->read_mutex);
   		 if (rv < 0)
    	{
    		printk("Error, +++%s+++,mutex_lock_interruptible = %d\n",  __func__,rv);
    		goto back;
   		 }

			//printk("read_buf_count = %d\n", read_buf_count);
			FrameLen = read_buf_count - 2;
			if(dev->m_unread_buf_count + FrameLen > HID_READ_BUF_STROTE_SIZE)
			{
				//printk("++%s++, unread_buf_read too much!!!!!!!!!!\n", __func__);
				memset(dev->m_read_buf, 0, HID_READ_BUF_STROTE_SIZE);
				dev->m_unread_buf_count = 0;
			}

			memcpy(dev->m_read_buf + dev->m_unread_buf_count , dev->int_in_buffer + 2, FrameLen);

			dev->m_unread_buf_count += FrameLen;	
			mutex_unlock(&dev->read_mutex);
		}

        //离开互斥体
        //mutex_unlock(&dev->ipod_analyse_mutex);
       // printk("read_thread, dev->m_unanalyse_buf_count=%d, dev->analyes_state=%d\n", dev->m_unanalyse_buf_count, dev->analyes_state);
    }
back:
    printk(">>>>iPodhid:APD:Exit Read Thread\r\n");
    return 0;
}

static int ipod_init(struct usb_carplay *dev)
{
    int retval = 0;
    DEBUG = 1;
    if(DEBUG)printk(">>>>iPodhid:>APD_Init(%p)\n",dev);
    //printk("++%s+++, dev = 0x%x\n", __func__, dev);
    mutex_lock(&dev->io_mutex);

    printk("_____mutex_lock_____\n");
    dev->isWrite = false;
    dev->ReadCount = 0;
    dev->app_read_count = 10;
    //此处判断如果不是第一个iPod设备，则不再继续初始化
    //   if(wcscmp(dev->DevName,_T("APD1:")) != 0)
    //   {
    //	   if(DEBUG)printk(">>>>iPodhid:APD_Init not APD1\n");
    //	   return 0;
    //   }
    printk("HID_REPORT_NUM_MAX---------\n");
    memset(dev->m_ipod_write_report,0,HID_REPORT_NUM_MAX * sizeof(ipod_write_report));
    memset(dev->m_ipod_read_report,0,HID_REPORT_NUM_MAX * sizeof(ipod_read_report));
    printk("-----------HID_REPORT_NUM_MAX End \n");
    dev->m_ipod_write_report_size = 0;
    dev->m_ipod_read_report_size = 0;
    //Get and Analyse Report Desp
    if (dev->m_ipod_write_report_size<=0
                ||dev->m_ipod_read_report_size<=0)
    {
        //需重新计算report
        if (get_and_analyse_report_desp(dev)==-1)
        {
            if(DEBUG)printk(">>>>iPodhid:APD_Open NG2\n");
            retval = -1;
            goto exit;
        }
    }

    dev->m_unread_buf_count=0;
    dev->m_flags.bThreadActive=1;
    //创建数据接收线程
    dev->ipod_read_thread_task = kthread_run(ipod_read_thread, dev,"ipod-read-thread");
    if (IS_ERR(dev->ipod_read_thread_task))
    {
        printk(">>>>iPodhid:Failed creating ipod read thread!\n");
        retval = -1;
        goto exit;
    }
exit:
    if (retval == -1)
    {
        //清除已初始化的变量
        if (dev->ipod_read_thread_task)
        {
            kthread_stop(dev->ipod_read_thread_task);
        }
#if 0
        if (dev->m_read_buf!=NULL)
        {
            kfree(dev->m_read_buf);
            dev->m_read_buf=NULL;
        }
        if (dev->m_analyse_buf!=NULL)
        {
            kfree(dev->m_analyse_buf);
            dev->m_analyse_buf=NULL;
        }
        if (dev->m_read_tmp_buf!=NULL)
        {
            kfree(dev->m_read_tmp_buf);
            dev->m_read_tmp_buf=NULL;
        }
        if (dev->m_analyse_tmp_buf!=NULL)
        {
            kfree(dev->m_analyse_tmp_buf);
            dev->m_analyse_tmp_buf=NULL;
        }
#endif
    }
    mutex_unlock(&dev->io_mutex);
    printk(">>>>iPodhid:<APD_Init:%d>\n",retval);
    return retval;
}


int init_data(struct usb_interface *interface,
            const struct usb_device_id *id, struct usb_carplay *dev)
{
	//struct usb_carplay *dev;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    unsigned int buffer_size;
    int i;
    int retval = -ENOMEM;
    iface_desc = interface->cur_altsetting;
    printk("enter ++%s++\n", __func__);
    if(interface == NULL || id == NULL || dev == NULL)
    {
    	printk("++++%s+++++Line = %d, NULL\n", __func__, __LINE__ );
    	return retval;
    }

	printk("+++++%s+++, num_altsetting = %d\n", __func__, interface->num_altsetting);
    if(0){
        printk("iface_desc->desc-> bLength= 0x%x\n",iface_desc->desc.bLength);
        printk("iface_desc->desc-> bDescriptorType= 0x%x\n",iface_desc->desc.bDescriptorType);
        printk("iface_desc->desc-> bAlternateSetting= 0x%x\n",iface_desc->desc.bAlternateSetting);
        printk("iface_desc->desc-> bNumEndpoints= 0x%x\n",iface_desc->desc.bNumEndpoints);
        printk("iface_desc->desc-> iInterface= 0x%x\n",iface_desc->desc.iInterface);
        printk("iface_desc->desc->bInterfaceNumber = 0x%x\n",iface_desc->desc.bInterfaceNumber);
        printk("iface_desc->desc->bInterfaceClass; = 0x%x\n",iface_desc->desc.bInterfaceClass);
        printk("iface_desc->desc->bInterfaceSubClass = 0x%x\n",iface_desc->desc.bInterfaceSubClass);
        printk("iface_desc->desc->bInterfaceProtocol = 0x%x\n",iface_desc->desc.bInterfaceProtocol);
        printk("---------------");
    }
    if(iface_desc->desc.bNumEndpoints <= 0){
      return -ENODEV;
    }
    if(0){
        printk("iface_desc->desc-> bLength= 0x%x\n",iface_desc->desc.bLength);
        printk("iface_desc->desc-> bDescriptorType= 0x%x\n",iface_desc->desc.bDescriptorType);
        printk("iface_desc->desc-> bAlternateSetting= 0x%x\n",iface_desc->desc.bAlternateSetting);
        printk("iface_desc->desc-> bNumEndpoints= 0x%x\n",iface_desc->desc.bNumEndpoints);
        printk("iface_desc->desc-> iInterface= 0x%x\n",iface_desc->desc.iInterface);
        printk("iface_desc->desc->bInterfaceNumber = 0x%x\n",iface_desc->desc.bInterfaceNumber);
        printk("iface_desc->desc->bInterfaceClass; = 0x%x\n",iface_desc->desc.bInterfaceClass);
        printk("iface_desc->desc->bInterfaceSubClass = 0x%x\n",iface_desc->desc.bInterfaceSubClass);
        printk("iface_desc->desc->bInterfaceProtocol = 0x%x\n",iface_desc->desc.bInterfaceProtocol);
    }
    if(DEBUG)printk("ipod_probe\n");
    /* allocate memory for our device state and initialize it */
    //dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        err("Out of memory");
        goto error;
    }
    kref_init(&dev->kref);
    sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
    mutex_init(&dev->io_mutex);
    mutex_init(&dev->read_mutex);
    mutex_init(&dev->write_mutex);
    spin_lock_init(&dev->err_lock);
    //init_usb_anchor(&dev->submitted);
    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    if(1){
    printk("dev->udev->config->desc->bLength = 0x%x\n",dev->udev->config->desc.bLength);
    printk("dev->udev->config->desc->bDescriptorType = 0x%x\n",dev->udev->config->desc.bDescriptorType);
    printk("dev->udev->config->desc->wTotalLength = 0x%x\n",dev->udev->config->desc.wTotalLength);
    printk("dev->udev->config->desc->bNumInterfaces = 0x%x\n",dev->udev->config->desc.bNumInterfaces);
    printk("dev->udev->config->desc->bConfigurationValue = 0x%x\n",dev->udev->config->desc.bConfigurationValue);
    printk("dev->udev->config->desc->iConfiguration = 0x%x\n",dev->udev->config->desc.iConfiguration);
    printk("dev->udev->config->desc->bmAttributes = 0x%x\n",dev->udev->config->desc.bmAttributes);
    printk("dev->udev->config->desc->bMaxPower = 0x%x\n",dev->udev->config->desc.bMaxPower);

    printk("dev->udev->actconfig->desc->bLength = 0x%x\n",dev->udev->actconfig->desc.bLength);
    printk("dev->udev->actconfig->desc->bDescriptorType = 0x%x\n",dev->udev->actconfig->desc.bDescriptorType);
    printk("dev->udev->actconfig->desc->wTotalLength = 0x%x\n",dev->udev->actconfig->desc.wTotalLength);
    printk("dev->udev->actconfig->desc->bNumInterfaces = 0x%x\n",dev->udev->actconfig->desc.bNumInterfaces);
    printk("dev->udev->actconfig->desc->bConfigurationValue = 0x%x\n",dev->udev->actconfig->desc.bConfigurationValue);
    printk("dev->udev->actconfig->desc->iConfiguration = 0x%x\n",dev->udev->actconfig->desc.iConfiguration);
    printk("dev->udev->actconfig->desc->bmAttributes = 0x%x\n",dev->udev->actconfig->desc.bmAttributes);
    printk("dev->udev->actconfig->desc->bMaxPower = 0x%x\n",dev->udev->actconfig->desc.bMaxPower);
    }
    dev->interface = interface;
    //创建数据分析互斥体
    //mutex_init(&dev->ipod_analyse_mutex);
    //创建数据读互斥体
    mutex_init(&dev->ipod_read_mutex);
    init_completion(&dev->int_in_completion);
   // init_completion(&dev->ctrl_in_completion);
    init_completion(&dev->ctrl_out_completion);
    //创建分析数据事件
    //init_completion(&dev->analyse_completion);
    sema_init(&dev->analyse_completion,0);
    //创建接收数据事件
    //sema_init(&dev->read_completion,0);
    init_completion(&dev->read_completion);

    sema_init(&dev->ipod_auth_completion,0);
    //init_completion(&dev->ipod_auth_completion);
    /* set up the endpoint information */
    /* use only the first int-in and int-out endpoints */
    if(DEBUG)printk("iface_desc->desc.bNumEndpoints = %d\n",iface_desc->desc.bNumEndpoints);
    for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
        endpoint = &iface_desc->endpoint[i].desc;
        if(DEBUG)printk("endpoint[i] = %d\n", i);
        if(usb_endpoint_is_int_in(endpoint)) {
            /* we found a int in endpoint */
            if(DEBUG)printk("found interrupt in on endpoint %d ,bInterva = %d ,buffer_size = %d\n", i,endpoint->bInterval,le16_to_cpu(endpoint->wMaxPacketSize));
            buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
            dev->int_in_size = buffer_size;
            dev->int_in_endpointAddr = endpoint->bEndpointAddress;
            dev->bInterval = endpoint->bInterval;
            dev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
            if (!dev->int_in_urb) {
                err("Could not allocate int_in_urb");
                goto error;
            }
        }
    }
    if (!(dev->int_in_endpointAddr)) {
        err("Could not find int-in endpoints");
        goto error;
    }
   // dev->ctrl_in_urb = usb_alloc_urb(0, GFP_KERNEL);
   // if (!dev->ctrl_in_urb) {
   //     err("Could not allocate ctrl_in_urb");
   //     goto error;
   // }

	if(ipod_init(dev)<0)
	{
	   retval = -1;
	   goto error;
	}

if(0)
{
	//#ifdef SUPPORT_IDPS
    
    //创建认证判断线程
   // dev->ipod_auth_thread_task = kthread_run(AuthThreadProc,dev,"AuthThreadProc");
   // if (IS_ERR(dev->ipod_auth_thread_task))
   // {
   //     printk(">>>>iPodhid:Failed creating ipod Auto thread!\n");
   //     retval = -1;
   //     goto error;
   // }
	//#endif
}

    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    /* we can register the device now, as it is ready */
    retval = usb_register_dev(interface, &ipod_class);
    if (retval) {
    /* something prevented us from registering this driver */
        err("Not able to get a minor for this device.");
        usb_set_intfdata(interface, NULL);
        goto error;
    }
    /* let the user know what node this device is now attached to */

    dev_info(&interface->dev,
                "USB ipod device now attached to USBipod-%d\n",
                interface->minor);
#if 0
    retval = alloc_chrdev_region(&dev->ipod_info.devno, 0,1,"ipod");
    if(retval){
        printk(KERN_ERR "alloc ipod region failed\n");
        goto error;
    }
    dev->ipod_info.major = MAJOR(dev->ipod_info.devno);
    cdev_init(&dev->ipod_info.ipod_dev,&ipod_fops);
    dev->ipod_info.ipod_dev.owner = THIS_MODULE;
    retval = cdev_add(&dev->ipod_info.ipod_dev,MKDEV(dev->ipod_info.major,0),1);
    if(retval){
        printk(KERN_ERR"add ipod dev failed\n");
        cdev_del(&dev->ipod_info.ipod_dev);
    }
    dev->ipod_info.dev_class = class_create(THIS_MODULE,"ipod class");
    if(IS_ERR(dev->ipod_info.dev_class)){
        goto error;
    }
    device_create(dev->ipod_info.dev_class,NULL,MKDEV(dev->ipod_info.major,0),NULL,"ipod");
#endif
    return 0;

error:
    if (dev){
        /* this frees allocated memory */
        kref_put(&dev->kref, ipod_delete);
    }
    printk("ipod probe fail\n");
    return retval;
}

static int ipod_deinit(struct usb_carplay*dev)
{
	//DEBUG = 1;
    if(DEBUG)printk(">>>>Enter iPodhid:>APD_Deinit\n");

    if (dev->m_flags.Open > 0){
        ipod_close(dev);
    }

    dev->m_flags.bThreadActive=0;
    memset(dev->m_ipod_write_report,0,HID_REPORT_NUM_MAX * sizeof(ipod_write_report));
    dev->m_ipod_write_report_size=0;
    memset(dev->m_ipod_read_report,0,HID_REPORT_NUM_MAX * sizeof(ipod_read_report));
    dev->m_ipod_read_report_size=0;

    dev->read_state = CLOSE_READ_THREAD;
    complete(&dev->read_completion);
    //up(&dev->read_completion);

    //关闭读线程
    //if(dev->ipod_read_thread_task)
    //kthread_stop(dev->ipod_read_thread_task);

    dev->analyes_state = CLOSE_ANALYSE_THREAD;
    //complete(&dev->analyse_completion);
    up(&dev->analyse_completion);

    dev->auth_event_state = hAuthExit;
    up(&dev->ipod_auth_completion);

    //关闭分析线程
    //if(dev->ipod_analyse_thread_task)
    //kthread_stop(dev->ipod_analyse_thread_task);
    complete(&dev->ctrl_out_completion);
    complete(&dev->ctrl_in_completion);
    complete(&dev->int_in_completion);
    dev->m_flags.Open = 0;
    if(DEBUG)printk(">>>>Exit iPodhid:>APD_Deinit\n");
    return 0;
}

static int ipod_close(struct usb_carplay*dev)
{
	//DEBUG = 1;
    if(DEBUG)printk(">>>>iPodhid:>APD_Close\n");
    dev->read_state = CLOSE_READ_THREAD;
    if(dev->isRead){
        complete(&dev->read_completion);
    }
    //up(&dev->read_completion);
    return 0;
}

static void ipod_disconnect(struct usb_interface *interface)
{
	//DEBUG = 1;
	struct usb_carplay *dev;
	int minor = interface->minor;
    if(DEBUG)printk("ipod_disconnect\n");
	dev = usb_get_intfdata(interface);
    dev->bAuthProcess = AUTH_NOTHING;
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &ipod_class);

	/* prevent more I/O from starting */
    ipod_deinit(dev);

	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);


	//usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
    kref_put(&dev->kref, ipod_delete);

	dev_info(&interface->dev, "USB ipod #%d now disconnected\n", minor);
}

static long ipod_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
{
	
    struct usb_carplay *dev;
    int retval = -EFAULT;
    int bRc = retval;
    dev = (struct usb_carplay *)file->private_data;
    //DEBUG = 1;
    if(dev == NULL ){
        if(DEBUG)printk(">>>>iPodhid:APD_Ioctl NG1\n");
        msleep(100);
        return -1;
    }
    switch(cmd)
    {
    	case IOCTL_SET_DEVICE_RATE:
    	{
    		unsigned int rate;
            if(DEBUG)printk(">>>>iPodhid::IOCTL_SET_DEVICE_RATE\n");
            if (copy_from_user(&rate, (void __user *)arg,sizeof(unsigned int)))
              return -EFAULT;

			unsigned char cTemp[6];
			memset(cTemp, 0, sizeof(cTemp));
			memcpy(cTemp, &rate, 4);

			int i = 0;
			bool isSupportRate = false;
			for(i = 0; i < sizeof( supported_sample_rates ); i += 4 )
			{
				if( (cTemp[0] == supported_sample_rates[0 + i] ) &&
					(cTemp[1] == supported_sample_rates[1 + i] ) &&
					(cTemp[2] == supported_sample_rates[2 + i] ) &&
					(cTemp[3] == supported_sample_rates[3 + i] ) )
				{
					isSupportRate = true;
					rate =  (cTemp[0]<<24);	
					rate |= (cTemp[1]<<16);
					rate |= (cTemp[2]<<8);
					rate |= cTemp[3];
					break;
				}
			}

			if(!isSupportRate)
			{
				printk("Warning!!!!! not support this rate, rate= %d\n", rate);
				return -1;
			}

			if(dev->ipod_rate != rate && rate > 0)
			{
				mutex_lock(&dev->write_mutex);
				retval = usb_control_msg(dev->udev,
                    usb_sndctrlpipe(dev->udev, 0),
                    0x01,
                    0x22,
                    (0x00|(0x01<<8)),
                    0x81,
                    cTemp,
                    3,
                    0);
			        if (retval != 3) 
			        {
			            if (retval >= 0) 
			            {
			                printk("ctrl_out, wlen %d (expected 3)\n",
			                            retval);
			                retval = -EBADMSG;
			            }
			            else
			            {
			                printk("%s usb_control_msg error : %d \n",__func__,retval);
			            }
			        }
			    mutex_unlock(&dev->write_mutex);

            	dev->ipod_rate = rate;
			}
                
            bRc = retval;
    	}
    	break;
    	case IOCTL_GET_DEVICE_RATE:
    	{
    		int rate;
            if(DEBUG)printk(">>>>iPodhid:IOCTL_GET_DEVICE_RATE\n");
            rate = dev->ipod_rate?dev->ipod_rate:44100;
            if (copy_to_user((void __user *)arg,&rate,sizeof(unsigned int)))
            {
            	return -EFAULT;
            }
            else
            {
            	bRc = sizeof(unsigned int);
            }
    	}
    	break;
    	default:
    	break;
    }
    return bRc;
}




