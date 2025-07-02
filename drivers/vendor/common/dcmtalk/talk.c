/*
 * 特别声明：本技术材料受《中华人民共和国著作权法》、《计算机软件保护条例》等法律、法规、行政
 *  规章以及有关国际条约的保护，赞同科技股份有限公司享有知识产权、保留一切权利并视其为技术秘密。未经本公司书
 *  面许可，任何人不得擅自（包括但不限于：以非法的方式复制、传播、展示、镜像、上载、下载）使用，
 *  不得向第三方泄露、透露、披露。否则，本公司将依法追究侵权者的法律责任。特此声明！
 *
 *  Special Declaration: These technical material reserved as the technical secrets by AGREE
 *  TECHNOLOGY have been protected by the "Copyright Law" "ordinances on Protection of Computer
 *  Software" and other relevant administrative regulations and international treaties. Without
 *  the written permission of the Company, no person may use (including but not limited to the
 *  illegal copy, distribute, display, image, upload, and download) and disclose the above
 *  technical documents to any third party. Otherwise, any infringer shall afford the legal
 *  liability to the company.
 *
 *  Copyright 1993-2023 Agree Tech. All rights reserved.
 *
 */

/**
 * Copyright (C), 1993-2023, 赞同科技股份有限公司
 * FileName: talk.c
 * Author:   ouyangyunsheng
 * Reviewer: ouyangyunsheng
 * Date:     2023/5/9 下午4:38
 * Description: DCM Talk
 * Since:    3.2
 * History:
 * <author>          <time>          <version>          <desc>
 * 作者姓名           修改时间           版本号              描述
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <stdbool.h>
#include <linux/delay.h>
#include "dcm_notifier.h"
#include "talk_msg.h"

// 握手协议
#define TALK_CMD_REQ		0x11
#define	TALK_ACK_REQ		0xa1
#define TALK_CMD_READY		0x12
#define TALK_ACK_READY		0xa2

// 两次握手成功的最大间隔时间:单位为毫秒
// 注：超过这个时间，dock/pad会认为usb通讯断开
#define TALK_INTERVAL_MAX_SECOND	10

/* Define these values to match your devices */
#define USB_DCM_VENDOR_ID	0x1a01	/* agree */
#define USB_DCM_PRODUCT_ID	0x0104	/* Dock Multifunction Composite Gadget */
#define USB_TALK_INTERFACE_NUM   0

#define TALK_MAX_BUF_LEN		(512)

/* Structure to hold all of our device specific stuff */
struct usb_talk {
	struct usb_device		*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct usb_anchor 		submitted;
	struct urb				*ep_in_urb;		/* the urb to read data with */
	unsigned char			*ep_in_buffer;	/* the buffer to receive data */
	size_t					ep_in_size;		/* the size of the receive buffer */
	size_t					ep_in_filled;
	__u8					ep_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8					ep_out_endpointAddr;	/* the address of the bulk out endpoint */
	struct mutex			io_mutex;
	int 					errors;
	spinlock_t				err_lock;
	bool					disconnected;
	wait_queue_head_t		ep_in_wait;
	bool					read_complete;

	/* kthread */
	struct task_struct *	kthread;

	struct timer_list 		talk_timer;
	struct work_struct		timer_work;

	bool					run;
	struct mutex			state_lock;

	bool 					lock_inited;
};

static struct usb_talk talk_device = {
	.disconnected = true,
	.run = false,
	.lock_inited = false,
};

static void send_data_callback(struct urb *urb)
{
	struct usb_talk *dev;
	unsigned long flags;

	dev = urb->context;

	if (urb->status) {
		if (!(urb->status == -ENOENT || urb->status == -ECONNRESET || urb->status == -ESHUTDOWN)) {
			printk(KERN_WARNING "<talk>nonzero write status received: %d\n", urb->status);
		}
		printk(KERN_WARNING "<talk>urb OUT error: %d\n", urb->status);
		spin_lock_irqsave(&dev->err_lock, flags);
		dev->errors = urb->status;
		spin_unlock_irqrestore(&dev->err_lock, flags);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	return ;
}

static int send_data_to_device(struct usb_talk *dev, unsigned char *buf, int len)
{
	struct urb *urb = NULL;
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	char *pbuf = NULL;
	size_t count = (size_t)len;
	int ret = 0;

	interface = dev->interface->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;

	/* verify that we actually have some data to write */
	if (count == 0) {
		goto exit;
	}

	if (count > TALK_MAX_BUF_LEN) {
		printk(KERN_WARNING "<transfer>write count[%d] is out of range[0-%d]\n", (int)count, TALK_MAX_BUF_LEN);
		ret = -EFAULT;
		goto exit;
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		printk(KERN_WARNING "<talk>usb alloc urb fail!\n");
		ret = -ENOMEM;
		goto exit;
	}

	pbuf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL,
									&urb->transfer_dma);
	if (!pbuf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	memcpy(pbuf, buf, count);

	mutex_lock(&dev->io_mutex);
	if (dev->disconnected) {
		mutex_unlock(&dev->io_mutex);
		ret = -ENODEV;
		goto release_urb;
	}

	usb_fill_bulk_urb(urb,
					dev->udev,
					usb_sndbulkpipe(dev->udev, dev->ep_out_endpointAddr),
					pbuf,
					count,
					send_data_callback,
					dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	mutex_unlock(&dev->io_mutex);
	if (ret) {
		printk(KERN_WARNING "<talk>submit urb fail!<%d>\n", ret);
		ret = -EIO;
		goto release_anchor;
	}

	usb_free_urb(urb);

	return count;

release_anchor:
	usb_unanchor_urb(urb);
release_urb:
	if (urb) {
		usb_free_coherent(dev->udev, count, pbuf, urb->transfer_dma);
		usb_free_urb(urb);
	}
exit:
	return ret;
}

static void receive_data_callback(struct urb *urb)
{
	struct usb_talk *dev;
	unsigned long flags;

	dev = urb->context;

	spin_lock_irqsave(&dev->err_lock, flags);
	if (urb->status) {
		if (!(urb->status == -ENOENT || urb->status == -ECONNRESET || urb->status == -ESHUTDOWN)) {
			printk(KERN_WARNING "<talk>nonzero read status received:%d\n", urb->status);
		}
		printk(KERN_WARNING "<talk>urb IN error: %d\n", urb->status);
		dev->errors = urb->status;
	} else {
		dev->ep_in_filled = urb->actual_length;
	}

	dev->read_complete = true;
	spin_unlock_irqrestore(&dev->err_lock, flags);
	wake_up_interruptible(&dev->ep_in_wait);
}

static int receive_data_from_device(struct usb_talk *dev, unsigned char *buf, int *len)
{
	unsigned long flags;
	int ret = 0;
	int rx_len = 0;

	ret = mutex_lock_interruptible(&dev->io_mutex);
	if (ret < 0) {
		printk(KERN_WARNING "<talk>mutex lock fail!<%d>\n", ret);
		return ret;
	}

	if (dev->disconnected) {
		printk(KERN_WARNING "<talk>dev is disconnected!\n");
		ret = -ENODEV;
		goto exit;
	}

	if (!dev->ep_in_urb) {
		printk(KERN_WARNING "<talk>cannot receive, skip!\n");
		ret = -EIO;
		goto exit;
	}

	if (!dev->read_complete) {
		printk(KERN_WARNING "<talk>device is busy\n");
		usb_kill_urb(dev->ep_in_urb);
		spin_lock_irqsave(&dev->err_lock, flags);
		dev->read_complete = true;
		spin_unlock_irqrestore(&dev->err_lock, flags);
		ret = -EBUSY;
		goto exit;
	}

	spin_lock_irqsave(&dev->err_lock, flags);

	ret = dev->errors;
	if (ret < 0) {
		dev->errors = 0;
		spin_unlock_irqrestore(&dev->err_lock, flags);
		printk(KERN_WARNING "<talk>dev error:%d\n", ret);
		ret = -EIO;
		goto exit;
	}

	dev->read_complete = false;
	spin_unlock_irqrestore(&dev->err_lock, flags);

	usb_fill_bulk_urb(dev->ep_in_urb, dev->udev,
					usb_rcvbulkpipe(dev->udev, dev->ep_in_endpointAddr),
					dev->ep_in_buffer,
					dev->ep_in_size,
					receive_data_callback,
					dev);

	ret = usb_submit_urb(dev->ep_in_urb, GFP_KERNEL);
	if (ret) {
		spin_lock_irqsave(&dev->err_lock, flags);
		dev->read_complete = true;
		spin_unlock_irqrestore(&dev->err_lock, flags);
		printk(KERN_WARNING "<talk>failed submitting read urb!<%d>\n", ret);
		ret = -EIO;
		goto exit;
	}

	/* wait for read finish */
	ret = wait_event_interruptible_timeout(dev->ep_in_wait, dev->read_complete, msecs_to_jiffies(1000));
	if (ret <= 0) {
		printk(KERN_WARNING "<talk>wait read complete fail!<%d>\n", ret);
		usb_kill_urb(dev->ep_in_urb);
		ret = -EIO;
		goto exit;
	}

	rx_len = dev->ep_in_filled;
	memcpy(buf, dev->ep_in_buffer, rx_len);
	*len = rx_len;

exit:
	mutex_unlock(&dev->io_mutex);
	return ret;
}

static void work_handler(struct work_struct *work)
{
	struct usb_talk *ptalk = (struct usb_talk *)container_of(work, struct usb_talk, timer_work);
	unsigned char vid[HW_VID_LEN] = { 0x00 };
	unsigned char pid[HW_PID_LEN] = { 0x00 };
	unsigned char sn[HW_SN_LEN] = { 0x00 };
	//printk(KERN_INFO "<dcm> work handler ...\n");

	mutex_lock(&ptalk->state_lock);
	ptalk->run = false;

	dcm_info_set_paired_vid(vid);
	dcm_info_set_paired_pid(pid);
	dcm_info_set_paired_sn(sn);

	mutex_unlock(&ptalk->state_lock);

	notify_dock_call_chain(EVENT_OF_DOCK_OFFLINE);

	return ;
}

static void talk_timer_handler(struct timer_list *timer)
{
	struct usb_talk *ptalk = (struct usb_talk *)container_of(timer, struct usb_talk, talk_timer);

	//printk(KERN_INFO "<dcm> talk timer handler ...\n");

	if (ptalk) {
		//printk(KERN_INFO "<dcm> schedule talk work ...\n");
		schedule_work(&ptalk->timer_work);
	}

	mod_timer(timer, jiffies + msecs_to_jiffies(TALK_INTERVAL_MAX_SECOND*1000));

	return ;
}

static int talk_kthread(void *data)
{
	struct usb_talk *ptalk = (struct usb_talk *)data;
	struct work_struct *pwork = &(ptalk->timer_work);
	struct timer_list *ptimer = &(ptalk->talk_timer);
	unsigned char rx_buf[TALK_MAX_BUF_LEN] = { 0x00 };
	int rx_len = 0;
	talk_frame_type *tx_frame;
	talk_frame_type *rx_frame;
	unsigned char local_vid[HW_VID_LEN+1] = { 0x00 };
	unsigned char local_pid[HW_PID_LEN+1] = { 0x00 };
	unsigned char local_sn[HW_SN_LEN+1] = { 0x00 };
	unsigned char paired_vid[HW_VID_LEN+1] = { 0x00 };
	unsigned char paired_pid[HW_PID_LEN+1] = { 0x00 };
	unsigned char paired_sn[HW_SN_LEN+1] = { 0x00 };
	int ret = 0;
	bool first_talk = true;

	printk(KERN_INFO "<talk>kthread start ...\n");

	mutex_init(&ptalk->state_lock);
	ptalk->run = false;
	ptalk->read_complete = true;

	INIT_WORK(pwork, work_handler);
	timer_setup(ptimer, talk_timer_handler, 0);
	ptimer->expires = jiffies + msecs_to_jiffies(TALK_INTERVAL_MAX_SECOND*1000);
	add_timer(ptimer);

	// wait for dcm info ready
	msleep(1000);
	memcpy(local_vid, HW_VID, strlen(HW_VID));
	memcpy(local_pid, HW_PID, strlen(HW_PID));
	dcm_info_get_local_sn(local_sn);

	while(!kthread_should_stop()) {
		/* first time talk: do not wait
		 * others:			wait 3 seconds
		 */
		if (first_talk) {
			first_talk = false;
		} else {
			if (msleep_interruptible(3000)) {
				printk(KERN_INFO "<talk>sleep interrupted!\n");
				continue ;
			}
		}

		/* send req cmd */
		//printk(KERN_INFO "<talk>send req cmd ...\n");
		tx_frame = talk_frame_create(TALK_TYPE_BASETALK, TALK_CMD_REQ, local_vid, local_pid, local_sn, NULL, 0);
		if (!tx_frame) {
			printk(KERN_WARNING "<talk>frame create fail!\n");
			continue ;
		}
		ret = send_data_to_device(ptalk, (unsigned char *)tx_frame, sizeof(talk_frame_type));
		if (ret < 0) {
			printk(KERN_WARNING "<talk>send req cmd to usb fail!<%d>\n", ret);
			continue ;
		}
		talk_frame_destory(tx_frame);

		/* receive req ack */
		//printk(KERN_INFO "<talk>receive req ack ...\n");
		ret = receive_data_from_device(ptalk, rx_buf, &rx_len);
		if (ret < 0) {
			printk(KERN_WARNING "<talk>receive data from usb fail!<%d>\n", ret);
			continue ;
		}

		//printk(KERN_INFO "<talk>receive %d data\n", rx_len);

		if (rx_len < sizeof(talk_frame_type)) {
			printk(KERN_WARNING "<talk>length of receive data is invalid:len=%d\n", rx_len);
			continue ;
		}

		rx_frame = (talk_frame_type *)rx_buf;
		if (talk_frame_check(rx_frame, TALK_TYPE_BASETALK, TALK_CMD_REQ)) {
			printk(KERN_WARNING "<talk>frame check fail\n");
			continue ;
		}

		/* send ready cmd */
		//printk(KERN_INFO "<talk>send ready cmd ...\n");
		tx_frame = talk_frame_create(TALK_TYPE_BASETALK, TALK_CMD_READY, local_vid, local_pid, local_sn, NULL, 0);
		if (!tx_frame) {
			printk(KERN_WARNING "<talk>frame create fail!\n");
			continue ;
		}
		ret = send_data_to_device(ptalk, (unsigned char *)tx_frame, sizeof(talk_frame_type));
		if (ret < 0) {
			printk(KERN_WARNING "<talk>send ready cmd to usb fail!<%d>\n", ret);
			continue ;
		}
		talk_frame_destory(tx_frame);

		/* receive ready ack */
		//printk(KERN_INFO "<talk>receive ready ack ...\n");
		ret = receive_data_from_device(ptalk, rx_buf, &rx_len);
		if (ret < 0) {
			printk(KERN_WARNING "<talk>receive data from usb fail!<%d>\n", ret);
			continue ;
		}

		//printk(KERN_INFO "<talk>receive %d data\n", rx_len);

		if (rx_len < sizeof(talk_frame_type)) {
			printk(KERN_WARNING "<talk>length of receive data is invalid:len=%d\n", rx_len);
			continue ;
		}

		rx_frame = (talk_frame_type *)rx_buf;
		if (talk_frame_check(rx_frame, TALK_TYPE_BASETALK, TALK_CMD_READY)) {
			printk(KERN_WARNING "<talk>frame check fail\n");
			continue ;
		}

		/* talk success */

		/* modify timer */
		mod_timer(ptimer, jiffies + msecs_to_jiffies(TALK_INTERVAL_MAX_SECOND*1000));

		if (!ptalk->run) {
			/* save paired dock information */
			mutex_lock(&(ptalk->state_lock));
			dcm_info_set_paired_vid(rx_frame->msg.hwid.vid);
			dcm_info_set_paired_pid(rx_frame->msg.hwid.pid);
			dcm_info_set_paired_sn(rx_frame->msg.hwid.sn);

			ptalk->run = true;
			mutex_unlock(&(ptalk->state_lock));

			/* print hwid info */
			dcm_info_get_paired_vid(paired_vid);
			dcm_info_get_paired_pid(paired_pid);
			dcm_info_get_paired_sn(paired_sn);
			printk(KERN_INFO "<talk>paired hwid --> %s:%s sn:%s", paired_vid, paired_pid, paired_sn);

			/* send notify */
			notify_dock_call_chain(EVENT_OF_DOCK_ONLINE);
		}

	}

	/* exit */
	printk(KERN_INFO "<talk>kthread stop ...\n");

	/* stop timer */
	del_timer_sync(ptimer);
	cancel_work_sync(pwork);

	/* stop talk */
	notify_dock_call_chain(EVENT_OF_DOCK_OFFLINE);

	printk(KERN_INFO "<talk>kthread exited\n");

	return 0;
}

static int talk_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	struct usb_talk *dev = &talk_device;
	struct usb_endpoint_descriptor *ep_in, *ep_out;
	int retval;

	if (!(dev->lock_inited)) {
		mutex_init(&dev->io_mutex);
		dev->lock_inited = true;
	}

	mutex_lock(&dev->io_mutex);
	dev->disconnected = false;

	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);
	init_waitqueue_head(&dev->ep_in_wait);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = usb_get_intf(interface);

	/* set up the endpoint information */
	/* use only the first INT-in and INT-out endpoints */
	retval = usb_find_common_endpoints(interface->cur_altsetting,
			&ep_in, &ep_out, NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
			"Could not find both INT-in and INT-out endpoints\n");
		goto exit1;
	}

	dev->ep_in_size = TALK_MAX_BUF_LEN;
	dev->ep_in_endpointAddr = ep_in->bEndpointAddress;
	dev->ep_in_buffer = kmalloc(dev->ep_in_size, GFP_KERNEL);
	if (!dev->ep_in_buffer) {
		printk(KERN_WARNING "<talk>kmalloc fail!\n");
		retval = -ENOMEM;
		goto exit1;
	}
	dev->ep_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->ep_in_urb) {
		printk(KERN_WARNING "<talk>usb alloc urb fail!\n");
		retval = -ENOMEM;
		goto exit2;
	}

	dev->ep_out_endpointAddr = ep_out->bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	dev->kthread = kthread_run(talk_kthread, (void *)dev, "talk_kthread");

	mutex_unlock(&dev->io_mutex);

	printk(KERN_INFO "<talk>device attached\n");

	return 0;

exit2:
	kfree(dev->ep_in_buffer);
exit1:
	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);

	mutex_unlock(&dev->io_mutex);

	return retval;
}

static void talk_disconnect(struct usb_interface *interface)
{
	struct usb_talk *dev;

	dev = usb_get_intfdata(interface);

	kthread_stop(dev->kthread);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->disconnected = true;

	usb_set_intfdata(interface, NULL);

	usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->ep_in_urb);
	usb_free_urb(dev->ep_in_urb);
	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);

	kfree(dev->ep_in_buffer);

	mutex_unlock(&dev->io_mutex);

	printk(KERN_INFO "<talk>disconnected\n");

	return ;
}

static void talk_draw_down(struct usb_talk *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 500);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
	usb_kill_urb(dev->ep_in_urb);
}

static int talk_pre_reset(struct usb_interface *intf)
{
	struct usb_talk *dev = usb_get_intfdata(intf);

	printk(KERN_INFO "<talk>pre reset...\n");

	kthread_stop(dev->kthread);

	mutex_lock(&dev->io_mutex);
	talk_draw_down(dev);

	return 0;
}

static int talk_post_reset(struct usb_interface *intf)
{
	struct usb_talk *dev = usb_get_intfdata(intf);

	printk(KERN_INFO "<talk>post reset...\n");

	/* we are sure no URBs are active - no locking needed */
	mutex_unlock(&dev->io_mutex);

	wake_up_process(dev->kthread);

	return 0;
}

static int talk_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_talk *dev = usb_get_intfdata(intf);

	printk(KERN_INFO "<talk>suspend...\n");

	if (!dev)
		return 0;

	kthread_stop(dev->kthread);

	talk_draw_down(dev);

	printk(KERN_INFO "<talk>suspend ok\n");

	return 0;
}

static int talk_resume(struct usb_interface *intf)
{
	struct usb_talk *dev = usb_get_intfdata(intf);

	printk(KERN_INFO "<talk>resume...\n");

	wake_up_process(dev->kthread);

	printk(KERN_INFO "<talk>resume ok\n");

	return 0;
}

/* table of devices that work with this driver */
static const struct usb_device_id talk_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(USB_DCM_VENDOR_ID, USB_DCM_PRODUCT_ID, USB_TALK_INTERFACE_NUM) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, talk_table);

static struct usb_driver talk_driver = {
	.name 		=	"talk",
	.probe 		=	talk_probe,
	.disconnect =	talk_disconnect,
	.pre_reset 	=	talk_pre_reset,
	.post_reset =	talk_post_reset,
	.suspend 	=	talk_suspend,
	.resume 	=	talk_resume,
	.id_table 	=	talk_table,
};

module_usb_driver(talk_driver);

MODULE_LICENSE("GPL v2");
