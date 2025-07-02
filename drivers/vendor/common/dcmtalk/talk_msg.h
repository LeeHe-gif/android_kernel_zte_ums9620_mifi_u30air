#ifndef __TALK_MSG_H__
#define __TALK_MSG_H__

#include "dcm_info.h"

#define TALK_HEAD					0x6E
#define TALK_TAIL					0xE6

// types
#define TALK_TYPE_BASETALK			0x01

// cmd
#define TALK_CMD_REQ				0x11
#define	TALK_ACK_REQ				0xa1
#define TALK_CMD_READY				0x12
#define TALK_ACK_READY				0xa2

// error code
#define TALK_ERR_OK					0x00
#define TALK_ERR_FAIL				0x01
#define TALK_ERR_HEAD_INVALID		0x02
#define TALK_ERR_TAIL_INVALID		0x03
#define TALK_ERR_CRC_INVALID		0x04
#define TALK_ERR_HWID_INVALID		0x05
#define TALK_ERR_TYPE_INVALID		0x06
#define TALK_ERR_CMD_INVALID		0x07
#define TALK_ERR_LEN_INVALID		0x08
#define TALK_ERR_DATA_INVALID		0x09

typedef struct _talk_msg_type {
	hwid_type	hwid;				//hw id
	unsigned char type;				//op type
	unsigned char cmd;				//op cmd
	unsigned char len;				//data len
	unsigned char data[64];			//frame data
}talk_msg_type;

typedef struct _talk_frame_type {
	unsigned char head;				//head
	talk_msg_type msg;				//msg
	unsigned char crc[2];			//crc16
	unsigned char tail;				//tail
}talk_frame_type;

talk_frame_type *talk_frame_create(unsigned char type,
									unsigned char cmd,
									unsigned char vid[HW_VID_LEN],
									unsigned char pid[HW_PID_LEN],
									unsigned char sn[HW_SN_LEN],
									unsigned char *msg_data,
									unsigned char msg_len);
void talk_frame_destory(talk_frame_type *frame);
int talk_frame_check(talk_frame_type *frame, unsigned char type, unsigned char cmd);

#endif // __TALK_MSG_H__
