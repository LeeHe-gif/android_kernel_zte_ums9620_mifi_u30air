#ifndef __DCM_INFO_H__
#define __DCM_INFO_H__

#define HW_VID						"AGREE-TECH"
#define HW_PID						"BeePad Pro"

#define HW_VID_LEN					(16)
#define HW_PID_LEN					(16)
#define HW_SN_LEN					(12)

typedef struct _hwid_type {
	unsigned char vid[HW_VID_LEN];
	unsigned char pid[HW_PID_LEN];
	unsigned char sn[HW_SN_LEN];
} hwid_type;


void dcm_info_set_local_vid(unsigned char buf[HW_VID_LEN]);
void dcm_info_get_local_vid(unsigned char buf[HW_VID_LEN]);

void dcm_info_set_local_pid(unsigned char buf[HW_PID_LEN]);
void dcm_info_get_local_pid(unsigned char buf[HW_PID_LEN]);

void dcm_info_set_local_sn(unsigned char buf[HW_SN_LEN]);
void dcm_info_get_local_sn(unsigned char buf[HW_SN_LEN]);


void dcm_info_set_paired_vid(unsigned char buf[HW_VID_LEN]);
void dcm_info_get_paired_vid(unsigned char buf[HW_VID_LEN]);

void dcm_info_set_paired_pid(unsigned char buf[HW_PID_LEN]);
void dcm_info_get_paired_pid(unsigned char buf[HW_PID_LEN]);

void dcm_info_set_paired_sn(unsigned char buf[HW_SN_LEN]);
void dcm_info_get_paired_sn(unsigned char buf[HW_SN_LEN]);

#endif // __DCM_INFO_H__
