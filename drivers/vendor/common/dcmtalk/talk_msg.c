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
 * FileName: talk_msg.c
 * Author:   ouyangyunsheng
 * Reviewer: ouyangyunsheng
 * Date:     2023/5/9 下午4:38
 * Description: DCM Talk Message
 * Since:    3.2
 * History:
 * <author>          <time>          <version>          <desc>
 * 作者姓名           修改时间           版本号              描述
 */

#include "talk_msg.h"
#include "linux/crc16.h"
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/slab.h>

typedef struct _talk_white_type {
	unsigned char vid[HW_VID_LEN];
	unsigned char pid[HW_PID_LEN];
} talk_white_type;

#define TALK_VID_DOCK	"AGREE-TECH"
#define TALK_PID_DOCK	"BeeDock"

#define TALK_WHILE_TABLE_NUM	16

static talk_white_type white_table[TALK_WHILE_TABLE_NUM] = {
	{ TALK_VID_DOCK, TALK_PID_DOCK },
	{ { 0 }, { 0 } },
};

static int check_white_table(unsigned char vid[HW_VID_LEN], unsigned char pid[HW_PID_LEN])
{
	talk_white_type *p = white_table;
	unsigned char vid_buf[HW_VID_LEN+1] = { 0x00 };
	unsigned char pid_buf[HW_PID_LEN+1] = { 0x00 };
	int i = 0;

	memcpy(vid_buf, vid, HW_VID_LEN);
	memcpy(pid_buf, pid, HW_PID_LEN);

	for (i = 0 ; i < TALK_WHILE_TABLE_NUM ; i++) {
		if((0 == p[i].vid[0]) || (0 == p[i].pid[0])) {
			printk(KERN_WARNING "<talk>white table end\n");
			break ;
		}

		if ((0 == strcmp(p[i].vid, vid_buf)) && (0 == strcmp(p[i].pid, pid_buf))) {
			return TALK_ERR_OK;
		}
	}

	return TALK_ERR_HWID_INVALID;
}

talk_frame_type *talk_frame_create(unsigned char type,
									unsigned char cmd,
									unsigned char vid[HW_VID_LEN],
									unsigned char pid[HW_PID_LEN],
									unsigned char sn[HW_SN_LEN],
									unsigned char *msg_data,
									unsigned char msg_len) {
	talk_frame_type *_frame;
	u16 _crc16 = 0xffff;

	_frame = kmalloc(sizeof(talk_frame_type), GFP_ATOMIC);

	if (_frame) {
		_frame->head = TALK_HEAD;
		_frame->tail = TALK_TAIL;

		if (vid) {
			memcpy(_frame->msg.hwid.vid, vid, HW_VID_LEN);
		}

		if (pid) {
			memcpy(_frame->msg.hwid.pid, pid, HW_PID_LEN);
		}

		if (sn) {
			memcpy(_frame->msg.hwid.sn, sn, HW_SN_LEN);
		}

		_frame->msg.type = type;
		_frame->msg.cmd = cmd;
		_frame->msg.len = (msg_len < sizeof(_frame->msg.data)) ?  msg_len : sizeof(_frame->msg.data);
		if (msg_data) {
			memcpy(_frame->msg.data, msg_data, _frame->msg.len);
		}

		// CRC16
		_crc16 = crc16(_crc16, (unsigned char *)&_frame->msg, sizeof(_frame->msg));
		// high first
		_frame->crc[0] = (_crc16 >> 8) & 0xff;
		_frame->crc[1] = (_crc16 >> 0) & 0xff;
	}

	return _frame;
}

void talk_frame_destory(talk_frame_type *frame)
{
	if (frame) {
		kfree(frame);
	}

	return ;
}

int talk_frame_check(talk_frame_type *frame, unsigned char type, unsigned char cmd)
{
	u16 crc_code[2] = { 0x00 };
	unsigned char vid[HW_VID_LEN+1] = { 0x00 };
	unsigned char pid[HW_PID_LEN+1] = { 0x00 };
	unsigned char sn[HW_SN_LEN+1] = { 0x00 };

	if (!frame) {
		printk(KERN_WARNING "<talk>frame is null!\n");
		return TALK_ERR_FAIL;
	}

	if (TALK_HEAD != frame->head) {
		printk(KERN_WARNING "<talk>frame head is invalid:0x%.2x\n", frame->head);
		return TALK_ERR_HEAD_INVALID;
	}

	if (TALK_TAIL != frame->tail) {
		printk(KERN_WARNING "<talk>frame tail is invalid:0x%.2x\n", frame->tail);
		return TALK_ERR_TAIL_INVALID;
	}

	crc_code[0] = (frame->crc[0] << 8) & 0xff00;
	crc_code[0] |= (frame->crc[1] << 0) & 0x00ff;
	crc_code[1] = crc16(0xffff, (unsigned char *)&frame->msg, sizeof(frame->msg));
	if (crc_code[0] != crc_code[1]) {
		printk(KERN_WARNING "<talk>crc is invalid:0x%.4x 0x%.4x\n", crc_code[0], crc_code[1]);
		return TALK_ERR_CRC_INVALID;
	}

	memcpy(vid, frame->msg.hwid.vid, HW_VID_LEN);
	memcpy(pid, frame->msg.hwid.pid, HW_PID_LEN);
	memcpy(sn, frame->msg.hwid.sn, HW_SN_LEN);
	//printk(KERN_INFO "<talk>paired hwid --> %s:%s sn:%s\n", vid, pid, sn);

	if (check_white_table(frame->msg.hwid.vid, frame->msg.hwid.pid)) {
		printk(KERN_WARNING "<talk>hwid is invalid\n");
		return TALK_ERR_HWID_INVALID;
	}

	if (type != frame->msg.type) {
		printk(KERN_INFO "<talk>type is invalid:0x%.2x\n", frame->msg.type);
		return TALK_ERR_TYPE_INVALID;
	}

	if (TALK_TYPE_BASETALK == frame->msg.type) {
		if (((cmd & 0x0f) + 0xa0) != frame->msg.cmd) {
			printk(KERN_INFO "<talk>cmd is invalid:0x%.2x\n", frame->msg.cmd);
			return TALK_ERR_CMD_INVALID;
		}
	} else {
		// TODO
		printk(KERN_INFO "<talk>type not support yet\n");
		return TALK_ERR_FAIL;
	}

	return TALK_ERR_OK;
}
