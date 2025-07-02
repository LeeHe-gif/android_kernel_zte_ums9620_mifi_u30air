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
 * FileName: dcm.c
 * Author:   ouyangyunsheng
 * Reviewer: ouyangyunsheng
 * Date:     2023/5/9 下午4:38
 * Description: Dock Communication Model
 * Since:    3.2
 * History:
 * <author>          <time>          <version>          <desc>
 * 作者姓名           修改时间           版本号              描述
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <stdbool.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
//#include <linux/soc/rockchip/rk_vendor_storage.h>
//#include <linux/wakelock.h>
#include "dcm_notifier.h"
#include "dcm_info.h"

#define DCM_VERSION		"V1.0"

#define DEVICE_LOCAL_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_local_attr_##_name = __ATTR(_name, _mode, _show, _store)

#define DEVICE_PAIRED_ATTR(_name, _mode, _show, _store) \
	struct device_attribute dev_paired_attr_##_name = __ATTR(_name, _mode, _show, _store)

struct dcm_device {
	/* work state */
	bool				run;

	struct mutex		lock;

	struct miscdevice 	misc;

	struct delayed_work	vendor_work;

	struct work_struct 	notifier_work;

	/* sys */
	struct kobject 		*local_kobj;
	struct kobject 		*paired_kobj;

	//struct wake_lock 	wake;
};

static struct dcm_device *dcm_handle = NULL;

static void vendor_work_handler(struct work_struct *work)
{
	unsigned char sn_buf[HW_SN_LEN+1] = { 0x00 };

	// TODO
#if 0
	ret = rk_vendor_read(SN_ID, sn_buf, sizeof(sn_buf));
	if (ret <= 0) {
		printk(KERN_WARNING "<dcm>read sn from vendor fail\n");
		return ;
	}
#endif
	printk(KERN_INFO "<dcm>local hwid --> %s:%s sn:%s\n", HW_VID, HW_PID, sn_buf);
	dcm_info_set_local_sn(sn_buf);

	return ;
}

/*-------------------------------------------------------------------------*/
/*                            DCM notifier                                 */

static int notify_call(struct notifier_block *nb, unsigned long event, void *v);

static struct notifier_block report_notifier = {
	.notifier_call = notify_call,
};

static void notifier_work_handler(struct work_struct *work)
{
	struct dcm_device *pdcm = dcm_handle;
	enum kobject_action action;
	static bool last_state = false;

	if (!pdcm) {
		printk(KERN_WARNING "dcm handle is not inited!\n");
		return ;
	}

	if (last_state == pdcm->run) {
		return ;
	}

	last_state = pdcm->run;

	action = (pdcm->run) ? KOBJ_ONLINE : KOBJ_OFFLINE;

	// 发送uevent消息
	printk(KERN_INFO "<dcm>send uevent:%s ...\n", (KOBJ_ONLINE == action) ? "ONLINE" : "OFFLINE");

	if (kobject_uevent(&(pdcm->misc.this_device->kobj), action) < 0) {
		printk(KERN_WARNING "<dcm>send uevent fail!\n");
	}

	return ;
}

// 函数回调
static int notify_call(struct notifier_block *nb, unsigned long event, void *v)
{
	struct dcm_device *pdcm = dcm_handle;

	if (!pdcm) {
		printk(KERN_WARNING "dcm handle is not inited!\n");
		return NOTIFY_DONE;
	}

	switch(event) {
	case EVENT_OF_DOCK_ONLINE:
		pdcm->run = true;
		break ;

	case EVENT_OF_DOCK_OFFLINE:
		pdcm->run = false;
		break ;

	default:
		break ;
	}

	schedule_work(&(pdcm->notifier_work));

	return NOTIFY_DONE;
}

/*-------------------------------------------------------------------------*/
/*                            DCM file operation                           */

static int dcm_open(struct inode *inode, struct file *fd)
{
	struct dcm_device *pdcm = dcm_handle;

	mutex_lock(&pdcm->lock);
	fd->private_data = pdcm;

	return 0;
}

static int dcm_release(struct inode *inode, struct file *fd)
{
	struct dcm_device *pdcm = dcm_handle;

	fd->private_data = NULL;
	mutex_unlock(&pdcm->lock);

	return 0;
}

static ssize_t dcm_read(struct file *file, char __user *buffer,
			size_t count, loff_t *ptr)
{
	struct dcm_device *pdcm = file->private_data;
	unsigned char value = 0x00;
	int len = 0;

	printk(KERN_INFO "<dcm>read ...\n");

	if (!count) {
		printk(KERN_INFO "<dcm>read 0 data, skip!\n");
		return 0;
	}

	if (!access_ok(buffer, count)) {
		printk(KERN_WARNING "<dcm>no read access!\n");
		return -EFAULT;
	}

	value = (pdcm->run) ? 0x01 : 0x00;
	len = 1;

	/* copy to user space */
	printk(KERN_INFO "<dcm>copy data to user space ...\n");
	if (0 != copy_to_user(buffer, &value, len)) {
		printk(KERN_INFO "<dcm>copy data to user space fail!\n");
		return -EFAULT;
	}

	count = len;

	return count;
}

static const struct file_operations dcm_fops = {
	.owner		= THIS_MODULE,
	.open		= dcm_open,
	.release	= dcm_release,
	.read		= dcm_read,
};

static ssize_t dcm_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcm_device *pdcm = dcm_handle;
	return sprintf(buf, "%s\n", pdcm->run ? "1" : "0");
}

static DEVICE_ATTR(state, S_IRUGO, dcm_state_show, NULL);

static ssize_t dcm_local_vid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", HW_VID);
}

static DEVICE_LOCAL_ATTR(vid, S_IRUGO, dcm_local_vid_show, NULL);

static ssize_t dcm_local_pid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", HW_PID);
}

static DEVICE_LOCAL_ATTR(pid, S_IRUGO, dcm_local_pid_show, NULL);

static ssize_t dcm_local_sn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char sn_buf[HW_SN_LEN+1] = { 0x00 };

	dcm_info_get_local_sn(sn_buf);

	return sprintf(buf, "%s\n", sn_buf);
}

static DEVICE_LOCAL_ATTR(sn, S_IRUGO, dcm_local_sn_show, NULL);


static ssize_t dcm_paired_vid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char vid_buf[HW_VID_LEN+1] = { 0x00 };

	dcm_info_get_paired_vid(vid_buf);

	return sprintf(buf, "%s\n", vid_buf);
}

static DEVICE_PAIRED_ATTR(vid, S_IRUGO, dcm_paired_vid_show, NULL);

static ssize_t dcm_paired_pid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char pid_buf[HW_PID_LEN+1] = { 0x00 };

	dcm_info_get_paired_pid(pid_buf);

	return sprintf(buf, "%s\n", pid_buf);
}

static DEVICE_PAIRED_ATTR(pid, S_IRUGO, dcm_paired_pid_show, NULL);

static ssize_t dcm_paired_sn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char sn_buf[HW_SN_LEN+1] = { 0x00 };

	dcm_info_get_paired_sn(sn_buf);

	return sprintf(buf, "%s\n", sn_buf);
}

static DEVICE_PAIRED_ATTR(sn, S_IRUGO, dcm_paired_sn_show, NULL);

static struct attribute *dcm_attributes[] = {
	&dev_attr_state.attr,
	NULL
};

static struct attribute *dcm_local_attributes[] = {
	&dev_local_attr_vid.attr,
	&dev_local_attr_pid.attr,
	&dev_local_attr_sn.attr,
	NULL
};

static struct attribute *dcm_paired_attributes[] = {
	&dev_paired_attr_vid.attr,
	&dev_paired_attr_pid.attr,
	&dev_paired_attr_sn.attr,
	NULL
};

static struct attribute_group dcm_attribute_group = {
	.attrs  = dcm_attributes,
};

static struct attribute_group dcm_local_attribute_group = {
	.attrs  = dcm_local_attributes,
};

static struct attribute_group dcm_paired_attribute_group = {
	.attrs  = dcm_paired_attributes,
};

static int __init dcm_core_init(void)
{
	struct dcm_device *pdcm;
	int ret = 0;

	printk(KERN_INFO "<dcm>core init:version %s\n", DCM_VERSION);
	pdcm = kzalloc(sizeof(struct dcm_device), GFP_KERNEL);
	if (!pdcm) {
		printk(KERN_WARNING "<dcm>kzalloc fail!\n");
		return -ENOMEM;
	}

	mutex_init(&pdcm->lock);
	pdcm->run = false;

	pdcm->misc.minor = MISC_DYNAMIC_MINOR;
	pdcm->misc.name = "dcm";
	pdcm->misc.fops = &dcm_fops;
	ret = misc_register(&pdcm->misc);
	if (ret < 0) {
		printk(KERN_WARNING "<dcm>misc register fail!<%d>\n", ret);
		ret = -EINVAL;
		goto release_lock;
	}

	/* create sysfs file */
	if (sysfs_create_group(&(pdcm->misc.this_device->kobj), &dcm_attribute_group)) {
		printk(KERN_ERR "<dcm>sysfs_create_group FAIL!\n");
		ret = -EINVAL;
		goto release_misc;
	}

	pdcm->local_kobj = kobject_create_and_add("local", &(pdcm->misc.this_device->kobj));
	if (NULL == pdcm->local_kobj) {
		printk(KERN_WARNING "<dcm>create local kobject fail!\n");
		ret = -EINVAL;
		goto release_state_group;
	}

	if (sysfs_create_group(pdcm->local_kobj, &dcm_local_attribute_group)) {
		printk(KERN_ERR "<dcm>sysfs create local group FAIL!\n");
		ret = -EINVAL;
		goto release_local_kobject;
	}

	pdcm->paired_kobj = kobject_create_and_add("paired", &(pdcm->misc.this_device->kobj));
	if (NULL == pdcm->paired_kobj) {
		printk(KERN_WARNING "<dcm>create paired kobject fail!\n");
		ret = -EINVAL;
		goto release_local_group;
	}

	if (sysfs_create_group(pdcm->paired_kobj, &dcm_paired_attribute_group)) {
		printk(KERN_ERR "<dcm>sysfs create paired group FAIL!\n");
		ret = -EINVAL;
		goto release_paired_kobject;
	}

	INIT_DELAYED_WORK(&pdcm->vendor_work, vendor_work_handler);
	schedule_delayed_work(&pdcm->vendor_work, msecs_to_jiffies(1000));

	dcm_handle = pdcm;

	//wake_lock_init(&pdcm->wake, WAKE_LOCK_SUSPEND, "dcm");

	//wake_lock(&pdcm->wake);

	INIT_WORK(&pdcm->notifier_work, notifier_work_handler);

	register_dock_notifier(&report_notifier);

	return 0;

release_paired_kobject:
	kobject_put(pdcm->paired_kobj);
release_local_group:
	sysfs_remove_group(&(pdcm->misc.this_device->kobj), &dcm_local_attribute_group);
release_local_kobject:
	kobject_put(pdcm->local_kobj);
release_state_group:
	sysfs_remove_group(&(pdcm->misc.this_device->kobj), &dcm_attribute_group);
release_misc:
	misc_deregister(&pdcm->misc);
release_lock:
	mutex_destroy(&pdcm->lock);
	kfree(pdcm);

	return ret;
}

static void __exit dcm_core_exit(void)
{
	struct dcm_device *pdcm = dcm_handle;

	printk(KERN_INFO "<dcm>core exit...\n");

	if (NULL == pdcm) {
		printk(KERN_WARNING "<dcm>dcm handle is NULL!\n");
		return ;
	}

	unregister_dock_notifier(&report_notifier);

	//wake_unlock(&pdcm->wake);

	//wake_lock_destroy(&pdcm->wake);

	cancel_work_sync(&pdcm->notifier_work);

	cancel_delayed_work(&pdcm->vendor_work);

	// 注销sysfs设备
	sysfs_remove_group(&(pdcm->misc.this_device->kobj), &dcm_attribute_group);
	misc_deregister(&pdcm->misc);
	mutex_destroy(&pdcm->lock);
	kfree(pdcm);

	dcm_handle = NULL;

	return ;
}

module_init(dcm_core_init);
module_exit(dcm_core_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("ouyang<ouyangyunsheng@agree.com>");
MODULE_DESCRIPTION("dcm core driver");

