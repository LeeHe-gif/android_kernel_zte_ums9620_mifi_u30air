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
 * FileName: dcm_info.c
 * Author:   ouyangyunsheng
 * Reviewer: ouyangyunsheng
 * Date:     2023/5/9 下午4:38
 * Description: DCM Information
 * Since:    3.2
 * History:
 * <author>          <time>          <version>          <desc>
 * 作者姓名           修改时间           版本号              描述
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include "dcm_info.h"

typedef struct _dcm_info_type {
	hwid_type   local;
	hwid_type   paired;
} dcm_info_type;

static dcm_info_type global_dcm_info = {
	.local = {
		.vid = HW_VID,
		.pid = HW_PID,
		.sn = { 0x00 },
	},
	.paired = {
		.vid = { 0x00 },
		.pid = { 0x00 },
		.sn = { 0x00 }
	}
};


/*****************************************************************/
/*----------------------------- local ---------------------------*/

void dcm_info_set_local_vid(unsigned char buf[HW_VID_LEN])
{
	unsigned char *p = global_dcm_info.local.vid;

	memcpy(p, buf, HW_VID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_local_vid);

void dcm_info_get_local_vid(unsigned char buf[HW_VID_LEN])
{
	unsigned char *p = global_dcm_info.local.vid;

	memcpy(buf, p, HW_VID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_local_vid);

void dcm_info_set_local_pid(unsigned char buf[HW_PID_LEN])
{
	unsigned char *p = global_dcm_info.local.pid;

	memcpy(p, buf, HW_PID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_local_pid);

void dcm_info_get_local_pid(unsigned char buf[HW_PID_LEN])
{
	unsigned char *p = global_dcm_info.local.pid;

	memcpy(buf, p, HW_PID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_local_pid);

void dcm_info_set_local_sn(unsigned char buf[HW_SN_LEN])
{
	unsigned char *p = global_dcm_info.local.sn;

	memcpy(p, buf, HW_SN_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_local_sn);

void dcm_info_get_local_sn(unsigned char buf[HW_SN_LEN])
{
	unsigned char *p = global_dcm_info.local.sn;

	memcpy(buf, p, HW_SN_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_local_sn);

/*****************************************************************/
/*----------------------------- paired ---------------------------*/

void dcm_info_set_paired_vid(unsigned char buf[HW_VID_LEN])
{
	unsigned char *p = global_dcm_info.paired.vid;

	memcpy(p, buf, HW_VID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_paired_vid);

void dcm_info_get_paired_vid(unsigned char buf[HW_VID_LEN])
{
	unsigned char *p = global_dcm_info.paired.vid;

	memcpy(buf, p, HW_VID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_paired_vid);

void dcm_info_set_paired_pid(unsigned char buf[HW_PID_LEN])
{
	unsigned char *p = global_dcm_info.paired.pid;

	memcpy(p, buf, HW_PID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_paired_pid);

void dcm_info_get_paired_pid(unsigned char buf[HW_PID_LEN])
{
	unsigned char *p = global_dcm_info.paired.pid;

	memcpy(buf, p, HW_PID_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_paired_pid);

void dcm_info_set_paired_sn(unsigned char buf[HW_SN_LEN])
{
	unsigned char *p = global_dcm_info.paired.sn;

	memcpy(p, buf, HW_SN_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_set_paired_sn);

void dcm_info_get_paired_sn(unsigned char buf[HW_SN_LEN])
{
	unsigned char *p = global_dcm_info.paired.sn;

	memcpy(buf, p, HW_SN_LEN);

	return ;
}
EXPORT_SYMBOL(dcm_info_get_paired_sn);
