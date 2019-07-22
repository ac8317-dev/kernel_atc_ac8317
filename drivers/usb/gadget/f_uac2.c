/*
 * f_uac2.c -- USB Audio Class 2.0 Function
 *
 * Copyright (C) 2011
 *    Yadwinder Singh (yadi.brar01@gmail.com)
 *    Jaswinder Singh (jaswinder.singh@linaro.org)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/usb/audio.h>
#include <linux/usb/audio-v2.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/kthread.h>

/* Playback(USB-IN) Default Stereo - Fl/Fr */
static int p_chmask = 0x3;
module_param(p_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(p_chmask, "Playback Channel Mask");

/* Playback Default 48 KHz */
static int p_srate = 44100;//48000;
module_param(p_srate, uint, S_IRUGO);
MODULE_PARM_DESC(p_srate, "Playback Sampling Rate");

/* Playback Default 16bits/sample */
static int p_ssize = 2;
module_param(p_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(p_ssize, "Playback Sample Size(bytes)");

/* Capture(USB-OUT) Default Stereo - Fl/Fr */
static int c_chmask = 0x3;
module_param(c_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(c_chmask, "Capture Channel Mask");

/* Capture Default 64 KHz */
//static int c_srate = 64000;
static int c_srate = 44100;//48000;  //change 10.20
module_param(c_srate, uint, S_IRUGO);
MODULE_PARM_DESC(c_srate, "Capture Sampling Rate");

/* Capture Default 16bits/sample */
static int c_ssize = 2;
module_param(c_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(c_ssize, "Capture Sample Size(bytes)");

#define DMA_ADDR_INVALID	(~(dma_addr_t)0)

#define ALT_SET(x, a)	do {(x) &= ~0xff; (x) |= (a); } while (0)
#define ALT_GET(x)	((x) & 0xff)
#define INTF_SET(x, i)	do {(x) &= 0xff; (x) |= ((i) << 8); } while (0)
#define INTF_GET(x)	((x >> 8) & 0xff)

/* Keep everyone on toes */
#define USB_XFERS	2

static char *fn_play_1 =  "/dev/snd/pcmC0D0p";

/*
 * The driver implements a simple UAC_2 topology.
 * USB-OUT -> IT_1 -> OT_3 -> ALSA_Capture
 * ALSA_Playback -> IT_2 -> OT_4 -> USB-IN
 * Capture and Playback sampling rates are independently
 *  controlled by two clock sources :
 *    CLK_5 := c_srate, and CLK_6 := p_srate
 */
/*#define USB_OUT_IT_ID	1
#define IO_IN_IT_ID	2
#define IO_OUT_OT_ID	3
#define USB_IN_OT_ID	4*/
#define USB_OUT_CLK_ID	5
#define USB_IN_CLK_ID	6 
/**
	Mic->USB_OUT_IT_ID1(IT1:mic)->USB_IN_OT_ID(OT7:USB Stream)->ALSA_Capture
	ALSA_Playback -> IO_IN_IT_ID2(IT2:USB Stream)->IO_OUT_OT_ID6(OT6: Speaker)->USB->IN
*/
#define USB_OUT_IT_ID	1
#define IO_IN_IT_ID		2
#define IO_OUT_OT_ID	6
#define USB_IN_OT_ID	7
//#define USB_OUT_CLK_ID	5
//#define USB_IN_CLK_ID	6
#define USB_CLK_SOURCE_ID 0x0B
#define USB_CLK_SLECTOR_ID 0x0C

#define CONTROL_ABSENT	0
#define CONTROL_RDONLY	1
#define CONTROL_RDWR	3

#define CLK_FREQ_CTRL	0
#define CLK_VLD_CTRL	2

#define COPY_CTRL	0
#define CONN_CTRL	2
#define OVRLD_CTRL	4
#define CLSTR_CTRL	6
#define UNFLW_CTRL	8
#define OVFLW_CTRL	10

#define CONFIG_BLACKFIN

//---------------------
unsigned char tmpBuf[512];
int nWriteIndex ;
int nReadIndex;
unsigned char* pTempBuf ;
bool isfulldata = false;
bool isStartRecord = false;
bool isStartPlay = false;
//static int audio_buf_size = 44100;//48000;//48000; //48000;//5120;//48000;
static int  audio_buf_size =5120;
#define NOAUDIO_DATA -2

static struct f_audio_buf *static_play_buf = NULL;
static int play_buf_index = 0;

//-----------------------

static  int uac2_pcm_open(struct snd_pcm_substream *substream);
static  int uac2_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params);
static int uac2_pcm_hw_free(struct snd_pcm_substream *substream);

static del_audio_cdev();

const char *uac2_name = "snd_uac2";

struct uac2_req {
	struct uac2_rtd_params *pp; /* parent param */
	struct usb_request *req;
};

struct uac2_rtd_params {
	bool ep_enabled; /* if the ep is enabled */
	/* Size of the ring buffer */
	size_t dma_bytes;
	unsigned char *dma_area;

	struct snd_pcm_substream *ss;

	/* Ring buffer */
	ssize_t hw_ptr;

	void *rbuf;

	size_t period_size;

	unsigned max_psize;
	struct uac2_req ureq[USB_XFERS];

	spinlock_t lock;
};

//
struct gaudio_snd_dev {
	//struct gaudio			*card;
	struct file			*filp;
	struct snd_pcm_substream	*substream;
	int				access;
	int				format;
	int				channels;
	int				rate;
};

static struct gaudio_snd_dev g_playsnd;

//add----
struct f_audio_buf {
	u8 *buf;
	u8 isPlayfalse;
	int actual;
	struct list_head list;
};

static struct f_audio_buf *f_audio_buffer_alloc(int buf_size)
{
	struct f_audio_buf *copy_buf;

	copy_buf = kzalloc(sizeof *copy_buf, GFP_ATOMIC);
	if (!copy_buf)
		return ERR_PTR(-ENOMEM);
	copy_buf->buf = kzalloc(buf_size, GFP_ATOMIC);
	if (!copy_buf->buf) {
		kfree(copy_buf);
		return ERR_PTR(-ENOMEM);
	}
  INIT_LIST_HEAD(&copy_buf->list);
	return copy_buf;
}

static void f_audio_buffer_free(struct f_audio_buf *audio_buf)
{
	kfree(audio_buf->buf);
	kfree(audio_buf);
}
//add ---

struct snd_uac2_chip {
	struct platform_device pdev;
	struct platform_driver pdrv;

	struct uac2_rtd_params p_prm;
	struct uac2_rtd_params c_prm;

	struct snd_card *card;
	struct snd_pcm *pcm;
//----
	spinlock_t			lock;
	struct mutex        mutex_lock;
	struct f_audio_buf *copy_buf;
	struct work_struct playback_work;
	struct list_head play_queue;
	
    unsigned  int cnt_copy_buf;
};

//add audio char device
static dev_t usbaudio_devid;
static struct class *usbaudio_class;
struct usbaudio_dev
{
	struct audio_dev * data;
	struct cdev dev;
};
static struct usbaudio_dev usbaudio_cdev;
#define MTK_UABAUDIO_NAME "usbaudiohost"

#define MAX_USBAUDIO_DEVS 1
static ssize_t usbaudio_read (struct file *, char __user *, size_t, loff_t *);
static int usbaudio_open (struct inode *, struct file *);

static int usbaudio_release (struct inode *, struct file *);

static long usbaudio_ioctl(struct file *, unsigned int, unsigned long);

static const struct file_operations usbaudio_fops = {
	.owner = THIS_MODULE,
	.read = usbaudio_read,
	.unlocked_ioctl = usbaudio_ioctl,
	.open = usbaudio_open,
	.release = usbaudio_release,
};

//----------

#define BUFF_SIZE_MAX	(PAGE_SIZE * 16)
#define PRD_SIZE_MAX	PAGE_SIZE
#define MIN_PERIODS	4

static struct snd_pcm_hardware uac2_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		 | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		 | SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_max = BUFF_SIZE_MAX / PRD_SIZE_MAX,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_max = PRD_SIZE_MAX,
	.periods_min = MIN_PERIODS,
};

struct audio_dev {
	/* Currently active {Interface[15:8] | AltSettings[7:0]} */
	__u16 ac_alt, as_out_alt, as_in_alt;

	struct usb_ep *in_ep, *out_ep;
	struct usb_function func;

	/* The ALSA Sound Card it represents on the USB-Client side */
	struct snd_uac2_chip uac2;
};

static struct audio_dev *agdev_g;

static inline
struct audio_dev *func_to_agdev(struct usb_function *f)
{
	return container_of(f, struct audio_dev, func);
}

static inline
struct audio_dev *uac2_to_agdev(struct snd_uac2_chip *u)
{
	return container_of(u, struct audio_dev, uac2);
}

static inline
struct snd_uac2_chip *pdev_to_uac2(struct platform_device *p)
{
	return container_of(p, struct snd_uac2_chip, pdev);
}

static inline
struct snd_uac2_chip *prm_to_uac2(struct uac2_rtd_params *r)
{
	struct snd_uac2_chip *uac2 = container_of(r,
					struct snd_uac2_chip, c_prm);

	if (&uac2->c_prm != r)
		uac2 = container_of(r, struct snd_uac2_chip, p_prm);

	return uac2;
}

static inline
uint num_channels(uint chanmask)
{
	uint num = 0;

	while (chanmask) {
		num += (chanmask & 1);
		chanmask >>= 1;
	}

	return num;
}


/*static void open_snd_dev(snd_uac2_chip *puac2)
{
	if(puac2 == NULL)
	{
		return;
	}

	struct snd_pcm_file *pcm_file;

	struct file * filep = filp_open(fn_play_1, O_WRONLY, 0);

	if (IS_ERR(filep)) {
		int ret = PTR_ERR(filep);
		printk("unable to open sound control device file:%s\n", fn_play_1);
		//ERROR(card, "unable to open sound control device file: %s\n",
		//		fn_cntl);
		filep = NULL;
		return ret;
	}

	pcm_file = filep->private_data;

	pcm_file->substream;

	printk("start open\n");
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		printk("open \n");
		filep = filp_open(fn_play_1, O_WRONLY, 0);
		printk("open 1\n");
		if (IS_ERR(filep)) 
		{
			int ret = PTR_ERR(filep);
			printk("unable to open sound control device file:%s\n", fn_play_1);
		//ERROR(card, "unable to open sound control device file: %s\n",
		//		fn_cntl);
			filep = NULL;
			spin_lock_irqsave(&prm->lock, flags);
			return ret;
		}

		printk("open fs\n");
		pcm_file = filep->private_data;
		printk("open fs1\n");
		substream = pcm_file->substream;
		printk("open fs2\n");
		frames = bytes_to_frames(substream->runtime, req->actual);

		printk("start write pcm\n");
		result = snd_pcm_lib_write(substream, (void*)tmpBuf, frames);
		printk("snd_pcm_lib_write!!!!!!result=%d, req->actual=%d\n", result, req->actual);  //added info
		set_fs(old_fs);

		spin_lock_irqsave(&prm->lock, flags);
		if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);
		printk("Playback-----\n");

		struct file * filep = filp_open("/data/audiolog", O_CREAT|O_RDWR|O_APPEND, S_IRUSR);
		printk("end audio\n");

		if (IS_ERR(filep)) 
		{
			int ret = PTR_ERR(filep);
			printk("unable to open sound control device file:%s\n", fn_play_1);
			//ERROR(card, "unable to open sound control device file: %s\n",
			//		fn_cntl);
			filep = NULL;
			return ret;
		}
		loff_t pos = 0x0;
		//printk("end audio1,nLen=%d\n", nLen);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		vfs_write(filep, tmpBuf, nLen, &pos);
		//printk("end audio2\n");
		filp_close(filep, NULL);
		set_fs(old_fs);
	
}*/

static void
agdev_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned pending;
	unsigned long flags;
	bool update_alsa = false;
	unsigned char *src, *dst;
	int status = req->status;
	struct uac2_req *ur = req->context;
	struct snd_pcm_substream *substream;
	struct uac2_rtd_params *prm = ur->pp;
	struct snd_uac2_chip *uac2 = prm_to_uac2(prm);
	struct snd_pcm_file *pcm_file;
	struct f_audio_buf *copy_buf = uac2->copy_buf;
	unsigned long flags_audio;

	struct file * filep;

	mm_segment_t old_fs;
	ssize_t result;
	snd_pcm_sframes_t frames;
	int i = 0;
	int nLen = req->actual;
	int nLeftLen = 0;
	struct snd_uac2_chip *audio = &agdev_g->uac2;
	
	//printk("enter agdev_iso_complate, prm->ep_enable=%d,status=%d \n", prm->ep_enabled, status);

	/* i/f shutting down */
	if (!prm->ep_enabled)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
			__func__, status, req->actual, req->length);
			
	substream = prm->ss;

	//printk("substream=%d, length = %d, max = %d\n", substream, nLen, req->length);

	
	/* Do nothing if ALSA isn't active */
	if (!substream)
		goto exit;

	//spin_lock_irqsave(&prm->lock, flags);

	//printk("req->buf[0]=0x%x, req->buf[1]=0x%x \n", 
	//	   tmpBuf[0], tmpBuf[1]);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if(uac2->cnt_copy_buf < 3 )
	{
		if(req->actual == 0)
		{
			goto exit;
		}

		if(copy_buf == NULL)
		{
			copy_buf = f_audio_buffer_alloc(audio_buf_size);
			if (IS_ERR(copy_buf))
			{
				printk("error, buff error\n");
				goto exit;
			}
		}
		
		if( audio_buf_size - copy_buf->actual < req->actual)
		{
			//buff full
			spin_lock_irqsave(&audio->lock, flags_audio);
			list_add_tail(&copy_buf->list, &uac2->play_queue);
			uac2->cnt_copy_buf++;
			spin_unlock_irqrestore(&audio->lock, flags_audio);
			//printk("add full , copy_buf->actual:%d, req->actual:%d\n", copy_buf->actual, req->actual);
			//schedule_work(&uac2->playback_work);
			copy_buf = f_audio_buffer_alloc(audio_buf_size);
			if (IS_ERR(copy_buf))
				goto exit;
		}

		memcpy(copy_buf->buf + copy_buf->actual, req->buf, req->actual);
		copy_buf->actual += req->actual;
		uac2->copy_buf = copy_buf;
  }
 
		if (usb_ep_queue(ep, req, GFP_ATOMIC))
			dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);
		//printk("end iso\n");
		return;
		
		/*struct snd_pcm_file *pcm_file;
		
		//printk("start audio\n");
		if(nWriteIndex + nLen > 5120)
		{
			printk("write full start\n");
			nLeftLen = nLen - 5120 + nWriteIndex;
			memcpy(pTempBuf, (char*)req->buf, 5120 - nWriteIndex);
			memcpy(tmpBuf , (char*)req->buf + 5120 - nWriteIndex, nLeftLen);
			pTempBuf = tmpBuf + nLeftLen;
			nWriteIndex = nLeftLen;
			isfulldata = true;
			printk("write full \n");
		}
		else
		{
			printk("----1, nLen=%d\n", nLen);
			memcpy(pTempBuf, (char*)req->buf, nLen);
			nWriteIndex = nWriteIndex + nLen;
			pTempBuf = tmpBuf + nWriteIndex;
			printk("----2, nWriteIndex=%d\n", nWriteIndex);
		}
		//printk("[0]=0x%x, [1]=0x%x\n", tmpBuf[0], tmpBuf[1]);
		
		//spin_unlock_irqrestore(&prm->lock, flags);
		if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);
		printk("end iso\n");
		return;*/
		/*uac2_pcm_open(substream);
		uac2_pcm_hw_params(substream, &uac2_pcm_hardware);
		printk("prm->dma_area=%d, prm->hw_ptr=%d\n", prm->dma_area, prm->hw_ptr);
		src = prm->dma_area + prm->hw_ptr;
		req->actual = req->length;
		dst = req->buf;*/
		
		
	 }
	else {
		uac2_pcm_open(substream);
		uac2_pcm_hw_params(substream, &uac2_pcm_hardware);
		printk("prm->dma_area1=%d, prm->hw_ptr1=%d\n", prm->dma_area, prm->hw_ptr);
		dst = prm->dma_area + prm->hw_ptr;
		src = req->buf;
		printk("capture-----\n");
	}

	printk("----prm->hw_ptr = %d, prm->period_size = %d\n", prm->hw_ptr, prm->period_size);
	pending = prm->hw_ptr % prm->period_size;
	pending += req->actual;
	if (pending >= prm->period_size)
	{
		printk("update_alsa = true\n");
		update_alsa = true;
	}
		
	prm->hw_ptr = (prm->hw_ptr + req->actual) % prm->dma_bytes;

	printk("set prm->hw_ptr\n");

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Pack USB load in ALSA ring buffer */
	memcpy(dst, src, req->actual);

	printk("agdev_iso_complete----Pack USB load in ALSA ring buffer\n");
	//frames = bytes_to_frames(substream->runtime, req->actual);
	//old_fs = get_fs();
	//set_fs(KERNEL_DS);
	//result = snd_pcm_lib_write(substream, dst, frames);
	//printk("snd_pcm_lib_write!!!!!!result=%d, req->actual=%d\n", result, req->actual);  //added info
	//set_fs(old_fs);
	
exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);

	if (update_alsa)
	{
		snd_pcm_period_elapsed(substream);
	}
	//if(substream)
	//	uac2_pcm_hw_free(substream);	

	return;
}

static int
uac2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct audio_dev *agdev = uac2_to_agdev(uac2);
	struct uac2_rtd_params *prm;
	unsigned long flags;
	struct usb_ep *ep;
	int err = 0;

	printk("enter uac2_pcm_trigger\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ep = agdev->in_ep;
		prm = &uac2->p_prm;
	} else {
		ep = agdev->out_ep;
		prm = &uac2->c_prm;
	}

	spin_lock_irqsave(&prm->lock, flags);

	/* Reset */
	prm->hw_ptr = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		prm->ss = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		prm->ss = NULL;
		break;
	default:
		err = -EINVAL;
	}

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Clear buffer after Play stops */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && !prm->ss)
		memset(prm->rbuf, 0, prm->max_psize * USB_XFERS);

	printk("enter uac2_pcm_trigger---\n");

	return err;
}

static snd_pcm_uframes_t uac2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;

	printk("enter uac2_pcm_pointer\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	return bytes_to_frames(substream->runtime, prm->hw_ptr);
}

static int uac2_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;
	int err;

	printk("enter uac2_pcm_hw_params\n");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	err = snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
	if (err >= 0) {
		prm->dma_bytes = substream->runtime->dma_bytes;
		prm->dma_area = substream->runtime->dma_area;
		prm->period_size = params_period_bytes(hw_params);
	}

	printk("enter uac2_pcm_hw_params---\n");

	return err;
}

static int uac2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct uac2_rtd_params *prm;

	printk("enter uac2_pcm_hw_free\n");
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac2->p_prm;
	else
		prm = &uac2->c_prm;

	prm->dma_area = NULL;
	prm->dma_bytes = 0;
	prm->period_size = 0;

	return snd_pcm_lib_free_pages(substream);
}

static int uac2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_uac2_chip *uac2 = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	printk("enter uac2_pcm_open\n");
	runtime->hw = uac2_pcm_hardware;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		spin_lock_init(&uac2->p_prm.lock);
		runtime->hw.rate_min = p_srate;
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE; /* ! p_ssize ! */
		runtime->hw.channels_min = num_channels(p_chmask);
		runtime->hw.period_bytes_min = 2 * uac2->p_prm.max_psize
						/ runtime->hw.periods_min;
	} else {
		spin_lock_init(&uac2->c_prm.lock);
		runtime->hw.rate_min = c_srate;
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE; /* ! c_ssize ! */
		runtime->hw.channels_min = num_channels(c_chmask);
		runtime->hw.period_bytes_min = 2 * uac2->c_prm.max_psize
						/ runtime->hw.periods_min;
	}

	runtime->hw.rate_max = runtime->hw.rate_min;
	runtime->hw.channels_max = runtime->hw.channels_min;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	printk("enter uac2_pcm_open----\n");
	return 0;
}

/* ALSA cries without these function pointers */
static int uac2_pcm_null(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_ops uac2_pcm_ops = {
	.open = uac2_pcm_open,
	.close = uac2_pcm_null,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = uac2_pcm_hw_params,
	.hw_free = uac2_pcm_hw_free,
	.trigger = uac2_pcm_trigger,
	.pointer = uac2_pcm_pointer,
	.prepare = uac2_pcm_null,
};

static int gaudio_open_snd_dev(struct gaudio_snd_dev *snd);
static int playback_default_hw_params(struct gaudio_snd_dev *snd);

static int /**__devinit*/ snd_uac2_probe(struct platform_device *pdev)
{
	struct snd_uac2_chip *uac2 = pdev_to_uac2(pdev);
	struct snd_card *card;
	struct snd_pcm *pcm;
	int err;

	printk("enter snd_uac2_probe\n");

	/* Choose any slot, with no id */
	err = snd_card_create(-1, NULL, THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	uac2->card = card;

	/*
	 * Create first PCM device
	 * Create a substream only for non-zero channel streams
	 */
	err = snd_pcm_new(uac2->card, "UAC2 PCM", 0,
			       p_chmask ? 1 : 0, c_chmask ? 1 : 0, &pcm);
	if (err < 0)
		goto snd_fail;

	strcpy(pcm->name, "UAC2 PCM");
	pcm->private_data = uac2;

	uac2->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &uac2_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &uac2_pcm_ops);

	strcpy(card->driver, "UAC2_Gadget");
	strcpy(card->shortname, "UAC2_Gadget");
	sprintf(card->longname, "UAC2_Gadget %i", pdev->id);

	snd_card_set_dev(card, &pdev->dev);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
		snd_dma_continuous_data(GFP_KERNEL), 0, BUFF_SIZE_MAX);

	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(pdev, card);
		return 0;
	}

	//gaudio_open_snd_dev(&g_playsnd);
	
snd_fail:
	snd_card_free(card);

	uac2->pcm = NULL;
	uac2->card = NULL;

	return err;
}

static int /**__devexit*/ snd_uac2_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (card)
		return snd_card_free(card);

	return 0;
}

void snd_uac2_release(struct device * dev)
{
	printk("snd_uac2_release====\n");
}

static int alsa_uac2_init(struct audio_dev *agdev)
{
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	int err;

	uac2->pdrv.probe = snd_uac2_probe;
	uac2->pdrv.remove = snd_uac2_remove;
	uac2->pdrv.driver.name = uac2_name;

	uac2->pdev.id = 0;
	uac2->pdev.name = uac2_name;
	uac2->pdev.dev.release = snd_uac2_release;

	printk("enter alsa_uac2_init\n");

	/* Register snd_uac2 driver */
	err = platform_driver_register(&uac2->pdrv);
	if (err)
		return err;

	/* Register snd_uac2 device */
	err = platform_device_register(&uac2->pdev);
	if (err)
		platform_driver_unregister(&uac2->pdrv);

	printk("enter alsa_uac2_init---\n");
	
	return err;
}

static void alsa_uac2_exit(struct audio_dev *agdev)
{
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	platform_driver_unregister(&uac2->pdrv);
	platform_device_unregister(&uac2->pdev);
}


/* --------- USB Function Interface ------------- */

enum {
	STR_ASSOC,
	STR_IF_CTRL,
	STR_CLKSRC_IN,
	STR_CLKSRC_OUT,
	//STR_CLKSRC,
	//STR_CLKSLECTOR,
	STR_USB_IT,
	STR_IO_IT,
	STR_USB_OT,
	STR_IO_OT,
	STR_AS_OUT_ALT0,
	STR_AS_OUT_ALT1,
	STR_AS_IN_ALT0,
	STR_AS_IN_ALT1,
};

static const char ifassoc[] = "Source/Sink";
//static const char ifctrl[] = "Topology Control";
static const char ifctrl[] = "My Audio";//"Pioneer Audio";
static char clksrc_in[8];
static char clksrc_out[8];
static const char usb_it[] = "USBH Out";
static const char io_it[] = "USBD Out";
static const char usb_ot[] = "USBH In";
static const char io_ot[] = "USBD In";
//static const char out_alt0[] = "Playback Inactive";
static const char out_alt0[] = "Audio-Out Interface";
static const char out_alt1[] = "Playback Active";
//static const char in_alt0[] = "Capture Inactive";
static const char in_alt0[] = "Audio-In Interface";
static const char in_alt1[] = "Capture Active";

static struct usb_string strings_fn[] = {
	[STR_ASSOC].s = ifassoc,
	[STR_IF_CTRL].s = ifctrl,
	[STR_CLKSRC_IN].s = clksrc_in,
	[STR_CLKSRC_OUT].s = clksrc_out,
	//[STR_CLKSRC].s = clksrc_in,
	//[STR_CLKSLECTOR].s = clksrc_out,
	[STR_USB_IT].s = usb_it,
	[STR_IO_IT].s = io_it,
	[STR_USB_OT].s = usb_ot,
	[STR_IO_OT].s = io_ot,
	[STR_AS_OUT_ALT0].s = out_alt0,
	[STR_AS_OUT_ALT1].s = out_alt1,
	[STR_AS_IN_ALT0].s = in_alt0,
	[STR_AS_IN_ALT1].s = in_alt1,
	{ },
};

static struct usb_gadget_strings str_fn = {
	.language = 0x0409,	/* en-us */
	.strings = strings_fn,
};

static struct usb_gadget_strings *fn_strings[] = {
	&str_fn,
	NULL,
};

static struct usb_qualifier_descriptor devqual_desc = {
	.bLength = sizeof devqual_desc,
	.bDescriptorType = USB_DT_DEVICE_QUALIFIER,

	.bcdUSB = cpu_to_le16(0x200),
	.bDeviceClass = USB_CLASS_MISC,
	.bDeviceSubClass = 0x02,
	.bDeviceProtocol = 0x01,
	.bNumConfigurations = 1,
	.bRESERVED = 0,
};

static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	//.bFirstInterface = 0,
	//.bInterfaceCount = 3,
	.bFirstInterface = 1,
	.bInterfaceCount = 3,
	.bFunctionClass = USB_CLASS_AUDIO,
	//.bFunctionSubClass = UAC2_FUNCTION_SUBCLASS_UNDEFINED,
	.bFunctionSubClass = 0x00,//10.20 0x01,
	.bFunctionProtocol = UAC_VERSION_2,
};

/* Audio Control Interface */
static struct usb_interface_descriptor std_ac_if_desc = {
	.bLength = sizeof std_ac_if_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOCONTROL,
	.bInterfaceProtocol = UAC_VERSION_2,
	.iInterface = 0x06,
};

//skypine ipod audio
/*Clock Source*/
struct uac_clock_source_descriptor clk_src_desc = 
{
	.bLength = sizeof clk_src_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC2_CLOCK_SOURCE,
	.bClockID = USB_CLK_SOURCE_ID,//USB_IN_CLK_ID, //0x0B
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_PROG,//UAC_CLOCK_SOURCE_TYPE_INT_FIXED,
	.bmControls = 0x07,//(CONTROL_RDONLY << CLK_FREQ_CTRL),
	.bAssocTerminal = 0,
};

struct uac_clock_selector_descriptor_1 {
	__u8 bLength;
	__u8 bDescriptorType;
	__u8 bDescriptorSubtype;
	__u8 bClockID;
	__u8 bNrInPins;
	__u8 baCSourceID[1];
	__u8 bmControls;
	__u8 iClockSelector;
	/* bmControls, bAssocTerminal and iClockSource omitted */
} __attribute__((packed));

/*Clock selector*/
struct uac_clock_selector_descriptor_1 clk_selector_desc = {
	.bLength = sizeof clk_selector_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC2_CLOCK_SELECTOR,
	.bClockID = USB_CLK_SLECTOR_ID, //0x0C
	.bNrInPins = 0x01,
	.baCSourceID[0] = USB_CLK_SOURCE_ID, //0x0B
	.bmControls = 0x01,
	.iClockSelector = 0x00,
};

/* Clock source for IN traffic */
struct uac_clock_source_descriptor in_clk_src_desc = {
	.bLength = sizeof in_clk_src_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC2_CLOCK_SOURCE,
	.bClockID = USB_IN_CLK_ID,
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_FIXED,
	.bmControls = (CONTROL_RDONLY << CLK_FREQ_CTRL),
	.bAssocTerminal = 0,
};

/* Clock source for OUT traffic */
struct uac_clock_source_descriptor out_clk_src_desc = {
	.bLength = sizeof out_clk_src_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC2_CLOCK_SOURCE,
	.bClockID = USB_OUT_CLK_ID,
	.bmAttributes = UAC_CLOCK_SOURCE_TYPE_INT_FIXED,
	.bmControls = (CONTROL_RDONLY << CLK_FREQ_CTRL),
	.bAssocTerminal = 0,
};

/* Input Terminal for USB_OUT */
/**struct uac2_input_terminal_descriptor usb_out_it_desc = {
	.bLength = sizeof usb_out_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	.bTerminalID = USB_OUT_IT_ID,
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	.bCSourceID = USB_OUT_CLK_ID,
	.iChannelNames = 0,
	.bmControls = (CONTROL_RDWR << COPY_CTRL),
};*/
struct uac2_input_terminal_descriptor usb_out_it_desc = {
	.bLength = sizeof usb_out_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	.bTerminalID = USB_OUT_IT_ID, //0x01
	.wTerminalType = cpu_to_le16(UAC_INPUT_TERMINAL_MICROPHONE),
	.bAssocTerminal = 0,
	.bCSourceID = USB_CLK_SLECTOR_ID, //0x0C
	.iChannelNames = 0x02,
	.bmChannelConfig = 0x00,
	.bmControls = 0x00,//(CONTROL_RDWR << COPY_CTRL),
};

/* Input Terminal for I/O-In */
/*struct uac2_input_terminal_descriptor io_in_it_desc = {
	.bLength = sizeof io_in_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	.bTerminalID = IO_IN_IT_ID,
	.wTerminalType = cpu_to_le16(UAC_INPUT_TERMINAL_UNDEFINED),
	.bAssocTerminal = 0,
	.bCSourceID = USB_IN_CLK_ID,
	.iChannelNames = 0,
	.bmControls = (CONTROL_RDWR << COPY_CTRL),
};*/
struct uac2_input_terminal_descriptor io_in_it_desc = {
	.bLength = sizeof io_in_it_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_INPUT_TERMINAL,
	.bTerminalID = IO_IN_IT_ID, //0x02
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	.bCSourceID = USB_CLK_SLECTOR_ID, //0x0C
	.iChannelNames = 0x02,
	.bmChannelConfig = 0x00,
	.bmControls = 0x00,//(CONTROL_RDWR << COPY_CTRL),
};

/* Ouput Terminal for USB_IN */
/**struct uac2_output_terminal_descriptor usb_in_ot_desc = {
	.bLength = sizeof usb_in_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	.bTerminalID = USB_IN_OT_ID,
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	.bSourceID = IO_IN_IT_ID,
	.bCSourceID = USB_IN_CLK_ID,
	.bmControls = (CONTROL_RDWR << COPY_CTRL),
};*/
struct uac2_output_terminal_descriptor usb_in_ot_desc = {
	.bLength = sizeof usb_in_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	.bTerminalID = USB_IN_OT_ID, //0x07
	.wTerminalType = cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 0,
	.bSourceID = USB_OUT_IT_ID, //0x01
	.bCSourceID = USB_CLK_SLECTOR_ID, //0x0C
	.bmControls = 0x00,//(CONTROL_RDWR << COPY_CTRL),
};


/* Ouput Terminal for I/O-Out */
/**struct uac2_output_terminal_descriptor io_out_ot_desc = {
	.bLength = sizeof io_out_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	.bTerminalID = IO_OUT_OT_ID,
	.wTerminalType = cpu_to_le16(UAC_OUTPUT_TERMINAL_UNDEFINED),
	.bAssocTerminal = 0,
	.bSourceID = USB_OUT_IT_ID,
	.bCSourceID = USB_OUT_CLK_ID,
	.bmControls = (CONTROL_RDWR << COPY_CTRL),
};*/
struct uac2_output_terminal_descriptor io_out_ot_desc = {
	.bLength = sizeof io_out_ot_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_OUTPUT_TERMINAL,
	.bTerminalID = IO_OUT_OT_ID, //0x06
	.wTerminalType = cpu_to_le16(UAC_OUTPUT_TERMINAL_SPEAKER),
	.bAssocTerminal = 0,
	.bSourceID = IO_IN_IT_ID, //0x02
	.bCSourceID = USB_CLK_SLECTOR_ID, //0x0C
	.bmControls = 0x00,//(CONTROL_RDWR << COPY_CTRL),
};

/**struct uac2_ac_header_descriptor ac_hdr_desc = {
	.bLength = sizeof ac_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_MS_HEADER,
	.bcdADC = cpu_to_le16(0x200),
	.bCategory = UAC2_FUNCTION_IO_BOX,
	.wTotalLength = sizeof in_clk_src_desc + sizeof out_clk_src_desc
			 + sizeof usb_out_it_desc + sizeof io_in_it_desc
			+ sizeof usb_in_ot_desc + sizeof io_out_ot_desc,
	.bmControls = 0,
};*/
struct uac2_ac_header_descriptor ac_hdr_desc = {
	.bLength = sizeof ac_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_MS_HEADER,
	.bcdADC = cpu_to_le16(0x200),
	.bCategory = UAC2_FUNCTION_HEADSET, //0x04
	.wTotalLength = sizeof ac_hdr_desc + sizeof clk_src_desc + sizeof clk_selector_desc
			 + sizeof usb_out_it_desc + sizeof io_in_it_desc
			+ sizeof usb_in_ot_desc + sizeof io_out_ot_desc,
	.bmControls = 0,
};

/* Audio Streaming OUT Interface - Alt0 */
static struct usb_interface_descriptor std_as_out_if0_desc = {
	.bLength = sizeof std_as_out_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
	.iInterface = 0x07,
};

/* Audio Streaming OUT Interface - Alt1 */
static struct usb_interface_descriptor std_as_out_if1_desc = {
	.bLength = sizeof std_as_out_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
	.iInterface = 0x07,
};

/* Audio Stream OUT Intface Desc */
struct uac2_as_header_descriptor as_out_hdr_desc = {
	.bLength = sizeof as_out_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_AS_GENERAL,
	.bTerminalLink = IO_IN_IT_ID, //0x02
	.bmControls = 0,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bmFormats = cpu_to_le32(UAC_FORMAT_TYPE_I_PCM),
	.bNrChannels = 0x02,
	.bmChannelConfig = 0x00,
	.iChannelNames = 0,
};

/* Audio USB_OUT Format */
struct uac2_format_type_i_descriptor as_out_fmt1_desc = {
	.bLength = sizeof as_out_fmt1_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_FORMAT_TYPE,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bSubslotSize = 0x02,		
	.bBitResolution = 0x10, //16
};

/**adptive*/
struct usb_endpoint_descriptor fs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = 0x03, //USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.wMaxPacketSize = 0x400,
	.bInterval = 4,//1,
};



struct usb_endpoint_descriptor hs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = 0x03, //USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.wMaxPacketSize = 0x400,
	.bInterval = 4,
};


/* STD AS ISO OUT Endpoint */
/**struct usb_endpoint_descriptor fs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.bInterval = 1,
};



struct usb_endpoint_descriptor hs_epout_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.bInterval = 4,
};*/


/* CS AS ISO OUT Endpoint */
static struct uac2_iso_endpoint_descriptor as_iso_out_desc = {
	.bLength = sizeof as_iso_out_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0,
};



/* Audio Streaming IN Interface - Alt0 */
static struct usb_interface_descriptor std_as_in_if0_desc = {
	.bLength = sizeof std_as_in_if0_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
	.iInterface = 0x08,
};

/* Audio Streaming IN Interface - Alt1 */
static struct usb_interface_descriptor std_as_in_if1_desc = {
	.bLength = sizeof std_as_in_if1_desc,
	.bDescriptorType = USB_DT_INTERFACE,

	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = UAC_VERSION_2,
	.iInterface = 0x08,
};

/* Audio Stream IN Intface Desc */
struct uac2_as_header_descriptor as_in_hdr_desc = {
	.bLength = sizeof as_in_hdr_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,

	.bDescriptorSubtype = UAC_AS_GENERAL,
	.bTerminalLink = USB_IN_OT_ID, //0x07
	.bmControls = 0,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bmFormats = cpu_to_le32(UAC_FORMAT_TYPE_I_PCM),
	.bNrChannels = 0x02,
	.bmChannelConfig = 0x00,
	.iChannelNames = 0,
};

/* Audio USB_IN Format */
struct uac2_format_type_i_descriptor as_in_fmt1_desc = {
	.bLength = sizeof as_in_fmt1_desc,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_FORMAT_TYPE,
	.bFormatType = UAC_FORMAT_TYPE_I,
	.bSubslotSize = 0x02,		/* {1,2,3,4} */
	.bBitResolution = 0x10, //(16)
};

/**SYNC in endpoint*/
struct usb_endpoint_descriptor fs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x84, //USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.wMaxPacketSize = 0x400,
	.bInterval = 4,//1,
};

struct usb_endpoint_descriptor hs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = 0x84, //USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ADAPTIVE,
	//.wMaxPacketSize = 0x400,
	.bInterval = 4,
};


/* STD AS ISO IN Endpoint */
/*
struct usb_endpoint_descriptor fs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.bInterval = 1,
};

struct usb_endpoint_descriptor hs_epin_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bmAttributes = USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.bInterval = 4,
};*/

/* CS AS ISO IN Endpoint */
static struct uac2_iso_endpoint_descriptor as_iso_in_desc = {
	.bLength = sizeof as_iso_in_desc,
	.bDescriptorType = USB_DT_CS_ENDPOINT,

	.bDescriptorSubtype = UAC_EP_GENERAL,
	.bmAttributes = 0,
	.bmControls = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0,
};

/**
static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&in_clk_src_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&in_clk_src_desc,
	(struct usb_descriptor_header *)&out_clk_src_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};
*/

static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&clk_src_desc,
	(struct usb_descriptor_header *)&clk_selector_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,

	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&fs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&std_ac_if_desc,

	(struct usb_descriptor_header *)&ac_hdr_desc,
	(struct usb_descriptor_header *)&clk_src_desc,
	(struct usb_descriptor_header *)&clk_selector_desc,
	(struct usb_descriptor_header *)&usb_out_it_desc,
	(struct usb_descriptor_header *)&io_in_it_desc,
	(struct usb_descriptor_header *)&io_out_ot_desc,
	(struct usb_descriptor_header *)&usb_in_ot_desc,
	

	(struct usb_descriptor_header *)&std_as_out_if0_desc,
	(struct usb_descriptor_header *)&std_as_out_if1_desc,
	
	(struct usb_descriptor_header *)&as_out_hdr_desc,
	(struct usb_descriptor_header *)&as_out_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epout_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,

	(struct usb_descriptor_header *)&std_as_in_if0_desc,
	(struct usb_descriptor_header *)&std_as_in_if1_desc,

	(struct usb_descriptor_header *)&as_in_hdr_desc,
	(struct usb_descriptor_header *)&as_in_fmt1_desc,
	(struct usb_descriptor_header *)&hs_epin_desc,
	(struct usb_descriptor_header *)&as_iso_in_desc,
	NULL,
};


struct cntrl_cur_lay3 {
	__u32	dCUR;
};

struct cntrl_range_lay3 {
	__u16	wNumSubRanges;
	__u32	dMIN;
	__u32	dMAX;
	__u32	dRES;
} __packed;

struct cntrl_range_lay3_ex {
	__u32	dMIN;
	__u32	dMAX;
	__u32	dRES;
};

static inline void
free_ep(struct uac2_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac2_chip *uac2 = prm_to_uac2(prm);
	int i;

	prm->ep_enabled = false;

	for (i = 0; i < USB_XFERS; i++) {
		if (prm->ureq[i].req) {
			usb_ep_dequeue(ep, prm->ureq[i].req);
			usb_ep_free_request(ep, prm->ureq[i].req);
			prm->ureq[i].req = NULL;
		}
	}
        return;

	if (usb_ep_disable(ep))
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
}

static 
int thread_func(void* arg)
{
	//return 0;
	mm_segment_t old_fs;
	while(1)
	{
		if(nWriteIndex == 0 )
		{
			//printk("---nReadIndex=%d, continue\n", nWriteIndex);
			msleep(0);
			continue;
		}
		else
		{
			
			if(isStartRecord == false )
			{
				if(nReadIndex < 5120)
				{
					if(tmpBuf[nReadIndex] == 0)
					{
						nReadIndex ++;
					}
					else
					{
						isStartRecord = true;
					}
				}
				else
				{
					nReadIndex = 0;
				}
				continue;
			}
		}

		if(isStartRecord)
		{
			if(!isStartPlay)
			{
				return 0;
			}
		}
		struct file * filep = filp_open("/data/audiolog.wave", O_CREAT|O_RDWR|O_APPEND, S_IRUSR);
		printk("end audio\n");

		if (IS_ERR(filep)) 
		{
			int ret = PTR_ERR(filep);
			printk("unable to open sound control device file:%s\n", fn_play_1);
			//ERROR(card, "unable to open sound control device file: %s\n",
			//		fn_cntl);
			filep = NULL;
			return ret;
		}
		loff_t pos = 0x0;
		printk("end audio1,nReadIndex=%d\n", nReadIndex);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(isfulldata)
		{
			vfs_write(filep, tmpBuf + nReadIndex, 5120 - nReadIndex, &pos);
			nReadIndex = 0;
			isfulldata = false;
		}
		else
		{
			if(nReadIndex + 256 <= 5210)
			{
				vfs_write(filep, tmpBuf + nReadIndex, 256, &pos);
				nReadIndex += 256;
			}
			else
			{
				if(nReadIndex >= 5120)
				{
					nReadIndex = 0;
				}
				else
				{
					vfs_write(filep, tmpBuf + nReadIndex, 5120 - nReadIndex, &pos);
					nReadIndex = 0;
				}
			}
		}
		
		printk("end audio2\n");
		filp_close(filep, NULL);
		set_fs(old_fs);
		msleep(0);
	}
}

/**
 * Some ALSA internal helper functions
 */
static int snd_interval_refine_set(struct snd_interval *i, unsigned int val)
{
	struct snd_interval t;
	t.empty = 0;
	t.min = t.max = val;
	t.openmin = t.openmax = 0;
	t.integer = 1;
	return snd_interval_refine(i, &t);
}


static int _snd_pcm_hw_param_set(struct snd_pcm_hw_params *params,
				 snd_pcm_hw_param_t var, unsigned int val,
				 int dir)
{
	int changed;
	if (hw_is_mask(var)) {
		struct snd_mask *m = hw_param_mask(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_mask_none(m);
		} else {
			if (dir > 0)
				val++;
			else if (dir < 0)
				val--;
			changed = snd_mask_refine_set(
					hw_param_mask(params, var), val);
		}
	} else if (hw_is_interval(var)) {
		struct snd_interval *i = hw_param_interval(params, var);
		if (val == 0 && dir < 0) {
			changed = -EINVAL;
			snd_interval_none(i);
		} else if (dir == 0)
			changed = snd_interval_refine_set(i, val);
		else {
			struct snd_interval t;
			t.openmin = 1;
			t.openmax = 1;
			t.empty = 0;
			t.integer = 0;
			if (dir < 0) {
				t.min = val - 1;
				t.max = val;
			} else {
				t.min = val;
				t.max = val+1;
			}
			changed = snd_interval_refine(i, &t);
		}
	} else
		return -EINVAL;
	if (changed) {
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	}
	return changed;
}

/**
 * Set default hardware params
 */
static int playback_default_hw_params(struct gaudio_snd_dev *snd)
{
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_hw_params *params;
	snd_pcm_sframes_t result;

       /*
	* SNDRV_PCM_ACCESS_RW_INTERLEAVED,
	* SNDRV_PCM_FORMAT_S16_LE
	* CHANNELS: 2
	* RATE: 48000
	*/
	snd->access = SNDRV_PCM_ACCESS_RW_INTERLEAVED;
	snd->format = SNDRV_PCM_FORMAT_S16_LE;
	snd->channels = 2;
	snd->rate = 48000;//48000;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	_snd_pcm_hw_params_any(params);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_ACCESS,
			snd->access, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_FORMAT,
			snd->format, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_CHANNELS,
			snd->channels, 0);
	_snd_pcm_hw_param_set(params, SNDRV_PCM_HW_PARAM_RATE,
			snd->rate, 0);

	snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_DROP, NULL);
	snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_HW_PARAMS, params);

	result = snd_pcm_kernel_ioctl(substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	if (result < 0) {
		printk(
			"Preparing sound card failed: %d\n", (int)result);
		kfree(params);
		return result;
	}

	/* Store the hardware parameters */
	snd->access = params_access(params);
	snd->format = params_format(params);
	snd->channels = params_channels(params);
	snd->rate = params_rate(params);

	kfree(params);

	printk(
		"Hardware params: access %x, format %x, channels %d, rate %d\n",
		snd->access, snd->format, snd->channels, snd->rate);

	return 0;
}

/**
 * Playback audio buffer data by ALSA PCM device
 */
static size_t u_audio_playback(struct gaudio_snd_dev *snd, void *buf, size_t count)
{
	//struct gaudio_snd_dev	*snd = &card->playback;
	struct snd_pcm_substream *substream = snd->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	mm_segment_t old_fs;
	ssize_t result;
	snd_pcm_sframes_t frames;

	//printk("enter u_audio_playback\n");
try_again:
	if (runtime->status->state == SNDRV_PCM_STATE_XRUN ||
		runtime->status->state == SNDRV_PCM_STATE_SUSPENDED) {
		result = snd_pcm_kernel_ioctl(substream,
				SNDRV_PCM_IOCTL_PREPARE, NULL);
		if (result < 0) {
			//ERROR(card, "Preparing sound card failed: %d\n",
			//		(int)result);
			printk("Preparing sound card failed: %d\n",
					(int)result);
			return result;
		}
	}

	//printk("enter u_audio_playback1\n");

	frames = bytes_to_frames(runtime, count);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	result = snd_pcm_lib_write(snd->substream, buf, frames);
	//INFO(card, "snd_pcm_lib_write!!!!!!\n");  //added info
	//printk("snd_pcm_lib_write!!!!!!, count = %d\n", count);
	//printk("++++%s++++, frames = %d\n", __func__, frames);
	if (result != frames) {
		//ERROR(card, "Playback error: %d\n", (int)result);
		printk("Playback error: %d\n", (int)result);
		set_fs(old_fs);
		goto try_again;
	}
	set_fs(old_fs);

	return 0;
}


static void f_audio_playback_work(struct work_struct *data)
{
	struct snd_uac2_chip *audio = container_of(data, struct snd_uac2_chip,
					playback_work);
	struct f_audio_buf *play_buf;

	printk("enter f_audio_playback_work\n");
	spin_lock_irq(&audio->lock);
	if (list_empty_careful(&audio->play_queue)) {
		spin_unlock_irq(&audio->lock);
		return;
	}
	play_buf = list_first_entry(&audio->play_queue,
			struct f_audio_buf, list);
	list_del_init(&play_buf->list);
	spin_unlock_irq(&audio->lock);

	u_audio_playback(&g_playsnd, play_buf->buf, play_buf->actual);
	f_audio_buffer_free(play_buf);
}


/**
 * Open ALSA PCM and control device files
 * Initial the PCM or control device
 */
static int gaudio_open_snd_dev(struct gaudio_snd_dev *snd)
{
	struct snd_pcm_file *pcm_file;
	//struct gaudio_snd_dev *snd;

	//if (!card)
	//	return -ENODEV;

	/* Open control device */
	

	/* Open PCM playback device and setup substream */
	//snd = &card->playback;
	snd->filp = filp_open(fn_play_1, O_WRONLY, 0);
	if (IS_ERR(snd->filp)) {
		//ERROR(card, "No such PCM playback device: %s\n", fn_play_1);
		printk("No such PCM playback device: %s\n", fn_play_1);
		snd->filp = NULL;
	}
	pcm_file = snd->filp->private_data;
	snd->substream = pcm_file->substream;
	//snd->card = card;
	playback_default_hw_params(snd);

	/* Open PCM capture device and setup substream */
	
	return 0;
}

static long usbaudio_ioctl(struct file * filp, unsigned int iParm , unsigned long lparm)
{
	//printk("read isStartPlay: %d\n", isStartPlay);
	return (long)isStartPlay;
}

static ssize_t usbaudio_read (struct file * filp, char __user * buff, size_t count, loff_t * offset)
{
	int ret = 0;
	unsigned long flags;
	if(buff == NULL || count <= 0)
	{
		printk("usbaudio_read err\n");
		//mutex_unlock(&audio->mutex_lock);
		return ret;
	}
	
	//printk("usbaudio_read==\n");
	if(agdev_g == NULL)
	{
		printk("read err\n");
		//mutex_unlock(&audio->mutex_lock);
		return ret;
	}

	
	struct snd_uac2_chip *audio = &agdev_g->uac2;
	//static struct f_audio_buf *play_buf = NULL;
	//static int play_buf_index = 0;
	spin_lock_irqsave(&audio->lock, flags);
	//if (mutex_lock_interruptible(&audio->mutex_lock))
	//	{
	//			printk("error lock error\n");
	//			return -ERESTARTSYS;
	//	}
		
	
	if (list_empty_careful(&audio->play_queue) && (static_play_buf == NULL)) {
		spin_unlock_irqrestore(&audio->lock, flags);
		//printk("read empty\n");
		if(isStartPlay == false)
		{
			//printk("-----NO DATA\n");
			ret = 1;//NOAUDIO_DATA;
		}
		//printk("read empty-----\n");
		//mutex_unlock(&audio->mutex_lock);
		return ret;
	}

	if(static_play_buf == NULL)
	{
		static_play_buf = list_first_entry(&audio->play_queue,
					struct f_audio_buf, list);
		//if(list_is_singular(&audio->play_queue))
		//printk("one list member ===, real:%d, 0x%x\n", static_play_buf->actual, static_play_buf);
		list_del_init(&static_play_buf->list);
		if(audio->cnt_copy_buf > 0)
		  audio->cnt_copy_buf--;
		play_buf_index = 0;
	}

	spin_unlock_irqrestore(&audio->lock, flags);

	//printk("play_buf:%d\n", play_buf);
	//printk("count:%d\n", count);
	if (mutex_lock_interruptible(&audio->mutex_lock))
		return -ERESTARTSYS;
	
	if(static_play_buf->actual == 0)
	{
		printk("play_buf->actual = 0\n");
		mutex_unlock(&audio->mutex_lock);
		return ret;
	}	
	if(static_play_buf->isPlayfalse == 1)
	{
		printk("_________play false\n");
	}
	//printk("play_buf_index:%d, count=%d, play_buf->actual=%d, play_buf->buf=0x%x\n", play_buf_index, count, static_play_buf->actual, static_play_buf->buf);
	if(play_buf_index + count < static_play_buf->actual)
	{
		//printk("play_buf_index:%d, count=%d, play_buf->actual=%d, play_buf->buf=0x%x\n", play_buf_index, count, static_play_buf->actual, static_play_buf->buf);
		copy_to_user(buff, static_play_buf->buf + play_buf_index, count);
		play_buf_index += count;
		ret  = count;
		//printk("ret: %d===\n", ret);
	}
	else 
	{
		//printk("!!play_buf_index:%d, count=%d, play_buf->actual=%d, play_buf->buf=0x%x\n", play_buf_index, count, static_play_buf->actual, static_play_buf->buf);
		int ncpynumber = static_play_buf->actual - play_buf_index;
		copy_to_user(buff, static_play_buf->buf + play_buf_index, ncpynumber);
		f_audio_buffer_free(static_play_buf);
		static_play_buf = NULL;
		ret  = ncpynumber;
		//printk("one queue ok, ret=%d\n", ret);
	}
	
	//printk("count:%d,ret=%d\n", count, ret);
	mutex_unlock(&audio->mutex_lock);
	return ret;
	
	#if 0
	struct snd_uac2_chip *audio = &agdev_g->uac2;
	//static struct f_audio_buf *play_buf = NULL;
	//static int play_buf_index = 0;
	spin_lock_irq(&audio->lock);
	if (list_empty_careful(&audio->play_queue) && static_play_buf == NULL) {
		spin_unlock_irq(&audio->lock);
		//printk("read empty\n");
		if(isStartPlay == false)
		{
			//printk("-----NO DATA\n");
			ret = 1;//NOAUDIO_DATA;
		}
		return ret;
	}

	if(static_play_buf == NULL)
	{
		static_play_buf = list_first_entry(&audio->play_queue,
					struct f_audio_buf, list);
		//if(list_is_singular(&audio->play_queue))
		//	printk("one list member ===\n");
		list_del_init(&static_play_buf->list);
		play_buf_index = 0;
	}

	spin_unlock_irq(&audio->lock);

	//printk("play_buf:%d\n", play_buf);
	//printk("count:%d\n", count);
	if (mutex_lock_interruptible(&audio->mutex_lock))
		return -ERESTARTSYS;
	
	if(static_play_buf->actual == 0)
	{
//		printk("play_buf->actual = 0\n");
		mutex_unlock(&audio->mutex_lock);
		return ret;
	}	
	if(static_play_buf->isPlayfalse == 1)
	{
		printk("_________play false\n");
	}
	if(play_buf_index + count < static_play_buf->actual)
	{
		//printk("play_buf_index:%d, count=%d, play_buf->actual=%d, play_buf->buf=0x%x\n", play_buf_index, count, play_buf->actual, play_buf->buf);
		copy_to_user(buff, static_play_buf->buf + play_buf_index, count);
		play_buf_index += count;
		ret  = count;
	}
	else 
	{
		//printk("!!play_buf_index:%d, count=%d, play_buf->actual=%d, play_buf->buf=0x%x\n", play_buf_index, count, play_buf->actual, play_buf->buf);
		int ncpynumber = static_play_buf->actual - play_buf_index;
		copy_to_user(buff, static_play_buf->buf + play_buf_index, ncpynumber);
		f_audio_buffer_free(static_play_buf);
		static_play_buf = NULL;
		ret  = ncpynumber;
		//printk("one queue ok\n");
	}
	
	printk("count:%d,ret=%d\n", count, ret);
	mutex_unlock(&audio->mutex_lock);
	return ret;
	#endif
}

static int usbaudio_release (struct inode * pinode, struct file * pfile)
{
	printk("usbaudio_release====\n");
	if(static_play_buf != NULL)
	{
		f_audio_buffer_free(static_play_buf);
		static_play_buf = NULL;
		play_buf_index = 0;
	}
	
	return 1;
}

static int usbaudio_open (struct inode * inode, struct file *flip)
{
	struct usbaudio_dev* pdev;
	
	//pdev = container_of(inode->i_cdev, struct usbaudio_dev, dev);
	//flip->private_data = pdev;
	//flip->private_data = agdev_g;
	struct snd_uac2_chip *audio = &agdev_g->uac2;
	struct f_audio_buf *play_buf = NULL;
	while (!list_empty_careful(&audio->play_queue))
	{
			play_buf = list_first_entry(&audio->play_queue,
			struct f_audio_buf, list);
			if(play_buf)
				{
						 list_del_init(&play_buf->list);
						f_audio_buffer_free(play_buf);
				}
				
			printk("!!!!remove play_queue\n");
	}
	audio->cnt_copy_buf = 0;
	printk("open ok\n");
	return 0;
}
static struct miscdevice usbaudiohost_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usbaudiohost",
	.fops = &usbaudio_fops,
};

static int add_audio_cdev(struct audio_dev *agdev)
{
	int retval = -1;
	int usbaudio_major;
	struct class *my_class;
	//if(agdev == NULL)
	//{
	//	printk("add_audio_cdev, err\n");
	//	return -1;
	//}

	retval = misc_register(&usbaudiohost_device);
	printk("retval = %d\n",retval);
	return retval;

	retval = register_chrdev(0, "usbaudiohost", &usbaudio_fops);
	//if(retval)
	{
		printk("fail register:%d\n", retval);
	}
	usbaudio_major = retval;
	my_class = class_create(THIS_MODULE, "usbaudioclass");
	retval = device_create(my_class, NULL, MKDEV(usbaudio_major, 0), NULL, "usbaudiohost");

	return retval;
	

	usbaudio_class = class_create(THIS_MODULE, "usbaudioclass");
	if (IS_ERR(usbaudio_class)) {
		retval = PTR_ERR(usbaudio_class);
		printk(KERN_ERR "usbaudio_class: class_create failed\n");
		goto err_out1;
	}


	retval = alloc_chrdev_region(&usbaudio_devid, 0, MAX_USBAUDIO_DEVS, MTK_UABAUDIO_NAME);

	if (retval) {
		printk(KERN_ERR "usbaudio: alloc_chrdev_region failed\n");
		goto err_out2;
	}

	usbaudio_major = MAJOR(usbaudio_devid);
	cdev_init(&usbaudio_cdev.dev, &usbaudio_fops);
	usbaudio_cdev.dev.owner = THIS_MODULE;
	retval = kobject_set_name(&usbaudio_cdev.dev.kobj, MTK_UABAUDIO_NAME);
	if (retval)
		goto err_out3;

	retval = cdev_add(&usbaudio_cdev.dev, MKDEV(usbaudio_major, 0), MAX_USBAUDIO_DEVS);
	if (retval) {
		printk("add error\n");
		kobject_put(&usbaudio_cdev.dev.kobj);
		goto err_out4;
	}
	usbaudio_cdev.data = agdev;
	struct device* pdev = device_create(usbaudio_class, NULL, MKDEV(usbaudio_major, 0), NULL, MTK_UABAUDIO_NAME);
	printk("+++++++++++++++++usb audio  OK, retval=%d, pdev = %d, max=%d, min=%d\n", retval, pdev,usbaudio_major, MINOR(usbaudio_devid));
	return retval;
err_out4:
	cdev_del(&usbaudio_cdev.dev);
err_out3:
	unregister_chrdev_region(MKDEV(usbaudio_major, 0), MAX_USBAUDIO_DEVS);
err_out2:
	class_destroy(usbaudio_class);
err_out1:

	printk("---usbaudio module OK\n");
	
	return retval;
}

static int audiohost_setup()
{
	//return 0;
	return add_audio_cdev(agdev_g); //add cdev
}

static audio_cleanup()
{
	misc_deregister(&usbaudiohost_device);
	if(agdev_g != NULL)
	{
		kfree(agdev_g);
		agdev_g = NULL;
	}
	
	//cdev_del(&usbaudio_cdev);
	//unregister_chrdev_region(MKDEV(usbaudio_devid, 0), MAX_USBAUDIO_DEVS);
	//class_destroy(usbaudio_class);
}

static int /**__init*/
afunc_bind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_composite_dev *cdev = cfg->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct uac2_rtd_params *prm;
	int ret;
	struct task_struct* my_thread = NULL;
	
	printk("afunc_bind enter\n");
	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	std_ac_if_desc.bInterfaceNumber = ret;
	ALT_SET(agdev->ac_alt, 0);
	INTF_SET(agdev->ac_alt, ret);

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	std_as_out_if0_desc.bInterfaceNumber = ret;
	std_as_out_if1_desc.bInterfaceNumber = ret;
	ALT_SET(agdev->as_out_alt, 0);
	INTF_SET(agdev->as_out_alt, ret);

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	std_as_in_if0_desc.bInterfaceNumber = ret;
	std_as_in_if1_desc.bInterfaceNumber = ret;
	ALT_SET(agdev->as_in_alt, 0);
	INTF_SET(agdev->as_in_alt, ret);

	agdev->out_ep = usb_ep_autoconfig(gadget, &fs_epout_desc);
	if (!agdev->out_ep)
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
	agdev->out_ep->driver_data = agdev;

	agdev->in_ep = usb_ep_autoconfig(gadget, &fs_epin_desc);
	if (!agdev->in_ep)
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
	agdev->in_ep->driver_data = agdev;

	hs_epout_desc.bEndpointAddress = fs_epout_desc.bEndpointAddress;
	hs_epout_desc.wMaxPacketSize = fs_epout_desc.wMaxPacketSize;
	hs_epin_desc.bEndpointAddress = fs_epin_desc.bEndpointAddress;
	hs_epin_desc.wMaxPacketSize = fs_epin_desc.wMaxPacketSize;

	fn->descriptors = usb_copy_descriptors(fs_audio_desc);
	if (gadget_is_dualspeed(gadget))
	{
		printk("-----dualspeed\n");
		fn->hs_descriptors = usb_copy_descriptors(hs_audio_desc);
	}
	printk("hs_epout_desc.bEndpointAddress:0x%x, hs_epout_desc.wMaxPacketSize:0x%x \
			hs_epin_desc.bEndpointAddress:0x%x, hs_epin_desc.wMaxPacketSize:0x%x\n",
			hs_epout_desc.bEndpointAddress, hs_epout_desc.wMaxPacketSize,
			hs_epin_desc.bEndpointAddress, hs_epin_desc.wMaxPacketSize);

	prm = &agdev->uac2.c_prm;
	prm->max_psize = hs_epout_desc.wMaxPacketSize;
	prm->rbuf = kzalloc(prm->max_psize * USB_XFERS, GFP_KERNEL);
	if (!prm->rbuf) {
		prm->max_psize = 0;
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
	}

	prm = &agdev->uac2.p_prm;
	prm->max_psize = hs_epin_desc.wMaxPacketSize;
	prm->rbuf = kzalloc(prm->max_psize * USB_XFERS, GFP_KERNEL);
	if (!prm->rbuf) {
		prm->max_psize = 0;
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
	}

//add----
	INIT_LIST_HEAD(&uac2->play_queue);
	spin_lock_init(&uac2->lock);
	mutex_init(&uac2->mutex_lock);
	uac2->copy_buf = NULL;
	uac2->cnt_copy_buf =0;
	//gaudio_open_snd_dev(&g_playsnd);
	//INIT_WORK(&uac2->playback_work, f_audio_playback_work);
	
	memset(tmpBuf, 0, sizeof(tmpBuf));
	nWriteIndex = 0;
	nReadIndex = 0;
	pTempBuf = tmpBuf;
	//my_thread = kthread_run(thread_func, NULL, "uac2_thread");

	if(IS_ERR(my_thread))
	{
		ret = PTR_ERR(my_thread);
		printk("error %d create uac2_thread\n", ret);
	}
	ret = alsa_uac2_init(agdev);
	//add_audio_cdev(agdev);
	//gaudio_open_snd_dev(&g_playsnd);
	//INIT_WORK(&uac2->playback_work, f_audio_playback_work);
	return ret;
}

static void
afunc_unbind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct usb_composite_dev *cdev = cfg->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct uac2_rtd_params *prm;

	alsa_uac2_exit(agdev);

	prm = &agdev->uac2.p_prm;
	kfree(prm->rbuf);

	prm = &agdev->uac2.c_prm;
	kfree(prm->rbuf);

	if (gadget_is_dualspeed(gadget))
		usb_free_descriptors(fn->hs_descriptors);
	usb_free_descriptors(fn->descriptors);

	if (agdev->in_ep)
		agdev->in_ep->driver_data = NULL;
	if (agdev->out_ep)
		agdev->out_ep->driver_data = NULL;
}

static int
afunc_set_alt(struct usb_function *fn, unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_gadget *gadget = cdev->gadget;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac2_rtd_params *prm;
	int i;
	int nret = -1;
	//unsigned long flags;
	
	printk("afunc_set_alt:intf=%d, alt=%d\n",intf, alt);
	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (intf == INTF_GET(agdev->ac_alt)) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			dev_err(&uac2->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}
		return 0;
	}
	
	if (intf == INTF_GET(agdev->as_out_alt)) {
		//printk("afunc_set_alt , out-----\n");
		ep = agdev->out_ep;
		prm = &uac2->c_prm;
		prm->ss = uac2->pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		nret = config_ep_by_speed(gadget, fn, ep);
		//printk("out, gadget->speed=%d, bret=%d\n", gadget->speed, nret);
		ALT_SET(agdev->as_out_alt, alt);
	} else if (intf == INTF_GET(agdev->as_in_alt)) {
		//printk("afunc_set_alt , in-----\n");
		ep = agdev->in_ep;
		prm = &uac2->p_prm;
		//prm->ss = uac2->pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
		nret = config_ep_by_speed(gadget, fn, ep);
		//printk("in, gadget->speed=%d, bret=%d\n", gadget->speed, nret);
		ALT_SET(agdev->as_in_alt, alt);
	} else {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (alt == 0) {
		free_ep(prm, ep);
		//printk("---alt = 0, free_ep\n");
		if(intf == 2)
		{
			isStartPlay = false;
			struct f_audio_buf *copy_buf = uac2->copy_buf;
			if (copy_buf) {
				copy_buf->isPlayfalse = 1;
				//printk("----copy_buf->actual=%d\n", copy_buf->actual);
				list_add_tail(&copy_buf->list,
						&uac2->play_queue);
				uac2->copy_buf = NULL;
				//schedule_work(&uac2->playback_work); //play
			}
		}
		return 0;
	}

	if(intf == 3)
	{
		free_ep(prm, ep);
		//printk("---intf = 3, free_ep\n");
		return 0;
	}

	isStartPlay = true;
	prm->ep_enabled = true;
	usb_ep_enable(ep);

	for (i = 0; i < USB_XFERS; i++) {
		if (prm->ureq[i].req) {
			if (usb_ep_queue(ep, prm->ureq[i].req, GFP_ATOMIC))
				dev_err(&uac2->pdev.dev, "%d Error!\n",
					__LINE__);
			continue;
		}

		req = usb_ep_alloc_request(ep, GFP_ATOMIC);
		if (req == NULL) {
			dev_err(&uac2->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}

		prm->ureq[i].req = req;
		prm->ureq[i].pp = prm;

		req->zero = 0;
		req->dma = DMA_ADDR_INVALID;
		req->context = &prm->ureq[i];
		req->length = prm->max_psize;
		req->complete =	agdev_iso_complete;
		req->buf = prm->rbuf + i * req->length;

		//printk("req->length:%d, i = %d\n", req->length, i);

		if (usb_ep_queue(ep, req, GFP_ATOMIC))
			dev_err(&uac2->pdev.dev, "%d Error!\n", __LINE__);
	}

	return 0;
}

static int
afunc_get_alt(struct usb_function *fn, unsigned intf)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	if (intf == INTF_GET(agdev->ac_alt))
		return ALT_GET(agdev->ac_alt);
	else if (intf == INTF_GET(agdev->as_out_alt))
		return ALT_GET(agdev->as_out_alt);
	else if (intf == INTF_GET(agdev->as_in_alt))
		return ALT_GET(agdev->as_in_alt);
	else
		dev_err(&uac2->pdev.dev,
			"%s:%d Invalid Interface %d!\n",
			__func__, __LINE__, intf);

	return -EINVAL;
}

static void
afunc_disable(struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;

	free_ep(&uac2->p_prm, agdev->in_ep);
	ALT_SET(agdev->as_in_alt, 0);

	free_ep(&uac2->c_prm, agdev->out_ep);
	ALT_SET(agdev->as_out_alt, 0);
}

static int
in_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	int value = -EOPNOTSUPP;
	
	printk("in_rq_cur, control_selector:0x%x\n", control_selector);
	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		struct cntrl_cur_lay3 c;

	printk("in_rq_cur, ----entity_id=%d\n", entity_id);
	c.dCUR = p_srate; //lqyan
	//c.dCUR = c_srate;
		/**
		-----
		if (entity_id == USB_IN_CLK_ID)
			c.dCUR = p_srate;
		else if (entity_id == USB_OUT_CLK_ID)
			c.dCUR = c_srate;
		*/

		value = min_t(unsigned, w_length, sizeof c);
		memcpy(req->buf, &c, value);
	} else if (control_selector == UAC2_CS_CONTROL_CLOCK_VALID) {
		*(u8 *)req->buf = 1;
		value = min_t(unsigned, w_length, 1);
	} else {
		dev_err(&uac2->pdev.dev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

static int
in_rq_range(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_request *req = fn->config->cdev->req;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_index = le16_to_cpu(cr->wIndex);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 entity_id = (w_index >> 8) & 0xff;
	u8 control_selector = w_value >> 8;
	struct cntrl_range_lay3 r;
	__u16 wNumSubRanges = 1;  //change 6 to 2, set sample frequency is 44.1K and 48K, if it is too much , the sample will be change from apple
	struct cntrl_range_lay3_ex r_range[wNumSubRanges-1];
	int value = -EOPNOTSUPP;
	int i  = 0;
	int iCount = 0;
	
	printk("in_rq_range, control_selector:0x%x, w_length:0x%x\n", control_selector, w_length);
	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ) {
		printk("in_rq_range , !!!entity_id=%d\n",entity_id);
		//r.dMIN = p_srate;  //lqyan
		//r.dMIN = c_srate;
		/**
		----
		if (entity_id == USB_IN_CLK_ID)
			r.dMIN = p_srate;
		else if (entity_id == USB_OUT_CLK_ID)
			r.dMIN = c_srate;
		else
			return -EOPNOTSUPP;

			r.dMAX = r.dMIN;
			r.dRES = 0;
			r.wNumSubRanges = 1;
		*/
		/*
			host to device
			0xa1 02 00 01  01 0b 0e 00

			device to host
			0x07 00 40 1f   00 00 40 1f
			  00 00 00 00   00 00 

			host to device 
			0xa1 02 00 01  01 0b 56 00

			device to host
			0x07 00 40 1f   00 00 40 1f
			  00 00 00 00   00 00 11 2b
			  00 00 11 2b   00 00 00 00
			  00 00 80 3e   00 00 80 3e
			  00 00 00 00   00 00 22 56
			  00 00 22 56   00 00 00 00
			  00 00 00 7d   00 00 00 7d
			  00 00 00 00   00 00 44 ac 
			  00 00 44 ac   00 00 00 00
			  00 00 00 00   00 00 80 bb
			  00 00 80 bb   00 00 00 00 
			  00 00
			  (0x2b11:11025, 0x3e80:16000, 0x5622:22050, 0x7d00:32000
			   0xac44:44100, 0xbb80:48000)
		*/
		if(w_length >= sizeof(r) ) //0x56=86 (wNumSubRanges=7) , wNumSubRanges=2 (0x1a = 26)
		{
			/*r.dMIN = 8000; //p_srate;
			r.dMAX = r.dMIN;
			r.dRES = 0;*/
			r.dMIN = 44100;
			r.dMAX = r.dMIN;
			r.dRES = 0;
			r.wNumSubRanges = wNumSubRanges; //(sample frequency count is 7/2, behind , it has 6/1 samples )
		}

		value = min_t(unsigned, w_length, sizeof r);
		memcpy(req->buf, &r, value);

		//  w_length = 0x56 (86)
		iCount = (w_length - value) / sizeof(struct cntrl_range_lay3_ex); //iCount should be =6 or 1
		if(iCount == wNumSubRanges - 1 && iCount != 0)
		{
			printk("--add value--, iCount:%d\n", iCount);
			r_range[0].dMIN= 48000;
			r_range[0].dMAX = r_range[0].dMIN;
			r_range[0].dRES = 0;
			
			
			/*r_range[0].dMIN= 11025;
			r_range[0].dMAX = r_range[0].dMIN;
			r_range[0].dRES = 0;
			//memcpy(req->buf + sizeof(r), &r_range[0] , 12);

			r_range[1].dMIN= 16000;
			r_range[1].dMAX = r_range[1].dMIN;
			r_range[1].dRES = 0;
			//memcpy(req->buf + sizeof(r) + 12, &r_range[1] , 12);
		
			r_range[2].dMIN= 22050;
			r_range[2].dMAX = r_range[2].dMIN;
			r_range[2].dRES = 0;
			//memcpy(req->buf + sizeof(r) + 24, &r_range[2] , 12);

			r_range[3].dMIN= 32000;
			r_range[3].dMAX = r_range[3].dMIN;
			r_range[3].dRES = 0;
			//memcpy(req->buf + sizeof(r) + 36, &r_range[3]  , 12);

			r_range[4].dMIN= 44100;
			r_range[4].dMAX = r_range[4].dMIN;
			r_range[4].dRES = 0;
			//memcpy(req->buf + sizeof(r) + 48, &r_range[4] , 12);

			r_range[5].dMIN= 48000;
			r_range[5].dMAX = r_range[5].dMIN;
			r_range[5].dRES = 0;*/
			//memcpy(req->buf + sizeof(r) + 60, &r_range[5] , 12);
			for(i = 0; i < iCount; i ++)
			{
				memcpy(req->buf + sizeof( struct cntrl_range_lay3) + i * sizeof(struct cntrl_range_lay3_ex), 
						&r_range[i], sizeof(struct cntrl_range_lay3_ex));
			}

			value += iCount * sizeof(struct cntrl_range_lay3_ex);
		}

	} else {
		dev_err(&uac2->pdev.dev,
			"%s:%d control_selector=%d TODO!\n",
			__func__, __LINE__, control_selector);
	}

	return value;
}

static int
ac_rq_in(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	if (cr->bRequest == UAC2_CS_CUR)
		return in_rq_cur(fn, cr);
	else if (cr->bRequest == UAC2_CS_RANGE)
		return in_rq_range(fn, cr);
	else
		return -EOPNOTSUPP;
}

static int
out_rq_cur(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	u16 w_length = le16_to_cpu(cr->wLength);
	u16 w_value = le16_to_cpu(cr->wValue);
	u8 control_selector = w_value >> 8;

	if (control_selector == UAC2_CS_CONTROL_SAM_FREQ)
	{
		printk("out_rq_cur:w_length=%d\n", w_length);
		return w_length;
	}
	return -EOPNOTSUPP;
}

static int
setup_rq_inf(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	u16 w_index = le16_to_cpu(cr->wIndex);
	u8 intf = w_index & 0xff;

	if (intf != INTF_GET(agdev->ac_alt)) {
		dev_err(&uac2->pdev.dev,
			"%s:%d Error!\n", __func__, __LINE__);
		return -EOPNOTSUPP;
	}

	if (cr->bRequestType & USB_DIR_IN)
		return ac_rq_in(fn, cr);
	else if (cr->bRequest == UAC2_CS_CUR)
		return out_rq_cur(fn, cr);

	return -EOPNOTSUPP;
}

static int
afunc_setup(struct usb_function *fn, const struct usb_ctrlrequest *cr)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac2_chip *uac2 = &agdev->uac2;
	struct usb_request *req = cdev->req;
	u16 w_length = le16_to_cpu(cr->wLength);
	int value = -EOPNOTSUPP;

	printk("-----afunc_setup, bRequest = 0x%x, cr->bRequestType=0x%x, wValue = 0x%x, wIndex = 0x%x, w_length=0x%x\n", 
			cr->bRequest, cr->bRequestType,  cr->wValue, cr->wIndex , w_length   );
	/* Only Class specific requests are supposed to reach here */
	if ((cr->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS)
	{
		printk("----afunc_setup: != USB_TYPE_CLASS\n");
		return -EOPNOTSUPP;
	}
	
	if ((cr->bRequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE)
	{
		printk("----afunc_setup: == USB_RECIP_INTERFACE\n");
		value = setup_rq_inf(fn, cr);
	}	
	else
	{
		dev_err(&uac2->pdev.dev, "%s:%d Error!\n", __func__, __LINE__);
		printk("----afunc_setup: %s:%d Error!\n", __func__, __LINE__);
	}
		

	if (value >= 0) {
		req->length = value;
		req->zero = value < w_length;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			dev_err(&uac2->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
			req->status = 0;
		}
	}

	return value;
}

static int audio_bind_config(struct usb_configuration *cfg)
{
	int id, res;
	printk("audio_bind_config222222\n");
	agdev_g = kzalloc(sizeof *agdev_g, GFP_KERNEL);
	if (agdev_g == NULL) {
		printk(KERN_ERR "Unable to allocate audio gadget\n");
		return -ENOMEM;
	}

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_ASSOC].id = id;
	iad_desc.iFunction = id,

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_IF_CTRL].id = id;
	std_ac_if_desc.iInterface = id,

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_CLKSRC_IN].id = id;
	//in_clk_src_desc.iClockSource = id, //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	//strings_fn[STR_CLKSRC_OUT].id = id; //10.20
	//out_clk_src_desc.iClockSource = id; //10.20

	//strings_fn[STR_CLKSRC].id = id;
	//clk_src_desc.iClockSource = id;  //10.20
	
	//strings_fn[STR_CLKSLECTOR].id = id;
	//clk_selector_desc.iClockSelector = id; //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_USB_IT].id = id;
	//usb_out_it_desc.iTerminal = id, //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_IO_IT].id = id;
	//io_in_it_desc.iTerminal = id; //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_USB_OT].id = id;
	//usb_in_ot_desc.iTerminal = id; //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_IO_OT].id = id;
	//io_out_ot_desc.iTerminal = id; //10.20

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_AS_OUT_ALT0].id = id;
	std_as_out_if0_desc.iInterface = id;

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_AS_OUT_ALT1].id = id;
	std_as_out_if1_desc.iInterface = id;

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_AS_IN_ALT0].id = id;
	std_as_in_if0_desc.iInterface = id;

	id = usb_string_id(cfg->cdev);
	if (id < 0)
		return id;

	strings_fn[STR_AS_IN_ALT1].id = id;
	std_as_in_if1_desc.iInterface = id;

	printk("cfg->superspeed:%d, cfg->highspeed:%d, cfg->fullspeed:%d\n", 
			cfg->superspeed, cfg->highspeed, cfg->fullspeed);
	printk("audio_bind_config-----1\n");
	agdev_g->func.name = "uac2_func";
	agdev_g->func.strings = fn_strings;
	agdev_g->func.bind = afunc_bind;
	agdev_g->func.unbind = afunc_unbind;
	agdev_g->func.set_alt = afunc_set_alt;
	agdev_g->func.get_alt = afunc_get_alt;
	agdev_g->func.disable = afunc_disable;
	agdev_g->func.setup = afunc_setup;

	/* Initialize the configurable parameters */
	usb_out_it_desc.bNrChannels = num_channels(c_chmask);
	usb_out_it_desc.bmChannelConfig = cpu_to_le32(0);//10.20 cpu_to_le32(c_chmask);
	io_in_it_desc.bNrChannels = num_channels(p_chmask);
	io_in_it_desc.bmChannelConfig = cpu_to_le32(0); //10.20 cpu_to_le32(p_chmask);
	as_out_hdr_desc.bNrChannels = num_channels(c_chmask);
	as_out_hdr_desc.bmChannelConfig = cpu_to_le32(0); //10.20 cpu_to_le32(c_chmask);
	as_in_hdr_desc.bNrChannels = num_channels(p_chmask);
	as_in_hdr_desc.bmChannelConfig = cpu_to_le32(0); //10.20 cpu_to_le32(p_chmask);
	as_out_fmt1_desc.bSubslotSize = c_ssize;
	as_out_fmt1_desc.bBitResolution = c_ssize * 8;
	as_in_fmt1_desc.bSubslotSize = p_ssize;
	as_in_fmt1_desc.bBitResolution = p_ssize * 8;

	snprintf(clksrc_in, sizeof(clksrc_in), "%uHz", p_srate);
	snprintf(clksrc_out, sizeof(clksrc_out), "%uHz", c_srate);
	
	/*snprintf(clksrc_in, sizeof(clksrc_in), "%uHz", as_in_fmt1_desc.bSubslotSize);
	snprintf(clksrc_out, sizeof(clksrc_out), "%uHz", as_in_fmt1_desc.bBitResolution);*/
	printk("clksrc_in:%d, clksrc_out = %d\n", as_in_fmt1_desc.bSubslotSize, as_in_fmt1_desc.bBitResolution);

	//add_audio_cdev(agdev_g); //add cdev

	res = usb_add_function(cfg, &agdev_g->func);

	
	
	printk("audio_bind_config-----2\n");
	if (res < 0)
		kfree(agdev_g);

	return res;
}

static void
uac2_unbind_config(struct usb_configuration *cfg)
{
	if(agdev_g != NULL)
	{
		kfree(agdev_g);
		agdev_g = NULL;
	}
}
