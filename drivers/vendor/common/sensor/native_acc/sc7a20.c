/******************** (C) COPYRIGHT 2016 Silan *********************************
 *
 * File Name          : sc7a20.c
 * Authors            : dongjianqing@silan.com.cn
 * Version            : V.1.0.0
 * Date               : 20/09/2016
 * Description        : SC7A20 accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, SILSN SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 20/09/2016
 First Release

 ******************************************************************************/

#include	<linux/module.h>
#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include	<linux/slab.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include	<linux/earlysuspend.h>
#endif
#ifdef CONFIG_ARCH_SC8810
#include	<mach/eic.h>
#endif
#include <sensor_common.h>

#include	"sc7a20.h"

#undef CONFIG_ARCH_SC8810

#define	INTERRUPT_MANAGEMENT 1

#define	G_MAX		2000	/* Maximum polled-device-reported g value */

/*
#define	SHIFT_ADJ_2G		4
#define	SHIFT_ADJ_4G		3
#define	SHIFT_ADJ_8G		2
#define	SHIFT_ADJ_16G		1
*/

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

#define	HIGH_RESOLUTION		0x88

#define	AXISDATA_REG		0x28
#define WHOAMI_SC7A20_ACC	0x11	/*      Expctd content for WAI  */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*      WhoAmI register         */
#define	TEMP_CFG_REG		0x1F	/*      temper sens control reg */
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*      control reg 1           */
#define	CTRL_REG2		0x21	/*      control reg 2           */
#define	CTRL_REG3		0x22	/*      control reg 3           */
#define	CTRL_REG4		0x23	/*      control reg 4           */
#define	CTRL_REG5		0x24	/*      control reg 5           */
#define	CTRL_REG6		0x25	/*      control reg 6           */

#define	FIFO_CTRL_REG		0x2E	/*      FiFo control reg        */

#define	INT_CFG1		0x30	/*      interrupt 1 config      */
#define	INT_SRC1		0x31	/*      interrupt 1 source      */
#define	INT_THS1		0x32	/*      interrupt 1 threshold   */
#define	INT_DUR1		0x33	/*      interrupt 1 duration    */

#define	INT_CFG2		0x34	/*      interrupt 2 config      */
#define	INT_SRC2		0x35	/*      interrupt 2 source      */
#define	INT_THS2		0x36	/*      interrupt 2 threshold   */
#define	INT_DUR2		0x37	/*      interrupt 2 duration    */

#define	TT_CFG			0x38	/*      tap config              */
#define	TT_SRC			0x39	/*      tap source              */
#define	TT_THS			0x3A	/*      tap threshold           */
#define	TT_LIM			0x3B	/*      tap time limit          */
#define	TT_TLAT			0x3C	/*      tap time latency        */
#define	TT_TW			0x3D	/*      tap time window         */
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1

#define SC7A20_ACC_PM_OFF		0x00
#define SC7A20_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10	/* 1Hz output data rate */
#define ODR10		0x20	/* 10Hz output data rate */
#define ODR25		0x30	/* 25Hz output data rate */
#define ODR50		0x40	/* 50Hz output data rate */
#define ODR100		0x50	/* 100Hz output data rate */
#define ODR200		0x60	/* 200Hz output data rate */
#define ODR400		0x70	/* 400Hz output data rate */
#define ODR1250		0x90	/* 1250Hz output data rate */

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */

/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01

#define	FUZZ			32
#define	FLAT			32
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x00

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8
#define	RES_INT_CFG2		9
#define	RES_INT_THS2		10
#define	RES_INT_DUR2		11

#define	RES_TT_CFG		12
#define	RES_TT_THS		13
#define	RES_TT_LIM		14
#define	RES_TT_TLAT		15
#define	RES_TT_TW		16

#define	RES_TEMP_CFG_REG	17
#define	RES_REFERENCE_REG	18
#define	RES_FIFO_CTRL_REG	19

#define	RESUME_ENTRIES		20
#define DEVICE_INFO             "ST, SC7A20"
#define DEVICE_INFO_LEN         32
#define SC7A30_XOUT_L			0x28
#define SC7A30_XOUT_H			0x29
#define SC7A30_YOUT_L			0x2A
#define SC7A30_YOUT_H			0x2B
#define SC7A30_ZOUT_L			0x2C
#define SC7A30_ZOUT_H			0x2D
#define SC7A30_MODE			    0x20
#define SC7A30_MODE1			0x21
#define SC7A30_MODE2			0x22
#define SC7A30_MODE3			0x23
#define SC7A30_BOOT			    0x24
#define SC7A30_STATUS			0x27

#define CALIBRATION_NUM		      20
#define AXIS_X_Y_RANGE_LIMIT		200
#define AXIS_X_Y_AVG_LIMIT			400
#define AXIS_Z_RANGE		        200
#define AXIS_Z_DFT_G		        1000
#define GOTO_CALI		        100
#define FAILTO_CALI		            101

#define SC7A30_RANGE			2000000

#define Z_OFF_DEFAULT  (8)	/*(110) */
#define XY_THR_N      (-15)	/*(-240) */
#define XY_THR_P      (15)	/*(240) */
#define Z_THR_MIN_N   (-50)	/*(-780) */
#define Z_THR_MIN_P   (50)	/*(780) */
#define Z_THR_MAX_N   (-78)	/*(-1200) */
#define Z_THR_MAX_P   (78)	/*(1200) */
#define SUM_DOT       (50)
#define THRESHOLD_VAL (8)	/*(40) */

#define SC7A30_PRECISION        12
#define SC7A30_BOUNDARY		(0x1 << (SC7A30_PRECISION - 1))
#define SC7A30_GRAVITY_STEP	(SC7A30_RANGE / SC7A30_BOUNDARY)
#define SC7A20_ACC_NAME     "sc7a20"

/* end RESUME STATE INDICES */

/*#define GSENSOR_GINT1_GPI 0*/
/*#define GSENSOR_GINT2_GPI 1*/
#ifdef CONFIG_ARCH_SC8810
extern int sprd_3rdparty_gpio_gint1_irq;
extern int sprd_3rdparty_gpio_gint2_irq;
#define GSENSOR_GINT1_GPI  EIC_ID_0	/* sprd_3rdparty_gpio_gint1_irq */
#define GSENSOR_GINT2_GPI  EIC_ID_1	/*sprd_3rdparty_gpio_gint2_irq */

extern void sprd_free_eic_irq(int irq);
#endif

// Determine silan_sc7a20_version
#define SILAN_SC7A20_VERSION			1
#define SC7A20_VERSION_ADDR          (0x70)
#define SC7A20_VERSION_VAL           (0x26)

#define SILAN_SC7A20_DATA_UPDATE
#define SILAN_SC7A20_FILTER

#ifdef SILAN_SC7A20_FILTER
typedef struct FilterChannelTag {
	s16 sample_l;
	s16 sample_h;
	s16 flag_l;
	s16 flag_h;
} FilterChannel;

typedef struct Silan_core_channel_s {
	s16 filter_param_l;
	s16 filter_param_h;
	s16 filter_threhold;
	FilterChannel sl_channel[3];
} Silan_core_channel;

Silan_core_channel core_channel;

static s16 filter_average(s16 preAve, s16 sample, s16 Filter_num, s16 *flag)
{
	if (*flag == 0) {
		preAve = sample;
		*flag = 1;
	}
	return preAve + (sample - preAve) / Filter_num;
}

static s16 silan_filter_process(FilterChannel *fac, s16 sample)
{
	if (fac == NULL) {
		return 0;
	}

	fac->sample_l = filter_average(fac->sample_l, sample, core_channel.filter_param_l, &fac->flag_l);
	fac->sample_h = filter_average(fac->sample_h, sample, core_channel.filter_param_h, &fac->flag_h);
	if (abs(fac->sample_l - fac->sample_h) > core_channel.filter_threhold) {
		fac->sample_h = fac->sample_l;
	}

	return fac->sample_h;
}
#endif

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} sc7a20_acc_odr_table[] = {
	{
	1, ODR1250}, {
	3, ODR400}, {
	5, ODR200}, {
	10, ODR100}, {
	20, ODR50}, {
	40, ODR25}, {
	100, ODR10}, {
1000, ODR1},};

struct hrtimer sc7a20_timer;
bool bsc7a20_enable = false;

struct SC7A30_acc {
	int x;
	int y;
	int z;
};

/*	direction settings	*/
static const int coordinate_trans[8][3][3] = {
	/* x_after, y_after, z_after */
	{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
	{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
	{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},
	{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},
	{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},
	{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}},
	{{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}},
	{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}},

};

struct sc7a20_acc_data {
	struct i2c_client *client;
	struct sc7a20_platform_data *pdata;

	struct mutex lock;
	/*struct delayed_work input_work; */
	struct work_struct input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	unsigned char sc7a20_placement;
};

struct sc7a20_acc_data *sc7a20_i2c_acc = NULL;

struct sc7a30_data_s {
	struct i2c_client *client;
	struct input_polled_dev *pollDev;
	struct mutex interval_mutex;

	struct delayed_work dwork;	/*allen */
	int time_of_cali;
	atomic_t enable;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
	volatile int suspend_indator;
#endif
} sc7a30_data;

/*------------------------------------add by david-----------*/
/*#define I2C_BUS_NUM_STATIC_ALLOC*/

struct Cali_Data {
	/* mis p and n */
	unsigned char xpmis;	/* x axis positive mismatch to write */
	unsigned char xnmis;	/* x axis negtive mismatch to write */
	unsigned char ypmis;
	unsigned char ynmis;
	unsigned char zpmis;
	unsigned char znmis;
	/* off p and n */
	unsigned char xpoff;	/* x axis positive offset to write */
	unsigned char xnoff;	/* x axis negtive offset to write */
	unsigned char ypoff;
	unsigned char ynoff;
	unsigned char zpoff;
	unsigned char znoff;
	/* mid mis and off */
	unsigned char xmmis;	/* x axis middle mismatch to write */
	unsigned char ymmis;	/* y axis middle mismatch to write */
	unsigned char zmmis;	/* z axis middle mismatch to write */
	unsigned char xmoff;	/* x axis middle offset to write */
	unsigned char ymoff;	/* y axis middle offset to write */
	unsigned char zmoff;	/* z axis middle offset to write */
	/* output p and n */
	signed int xpoutput;	/* x axis output of positive mismatch */
	signed int xnoutput;	/* x axis output of negtive mismatch */
	signed int ypoutput;
	signed int ynoutput;
	signed int zpoutput;
	signed int znoutput;
	/* output */
	signed int xfoutput;	/* x axis the best or the temporary output */
	signed int yfoutput;	/* y axis the best or the temporary output */
	signed int zfoutput;	/* z axis the best or the temporary output */
	/* final and temp flag */
	unsigned char xfinalf;	/* x axis final flag:if 1,calibration finished */
	unsigned char yfinalf;	/* y axis final flag:if 1,calibration finished */
	unsigned char zfinalf;	/* z axis final flag:if 1,calibration finished */
	unsigned char xtempf;	/* x axis temp flag:if 1,the step calibration finished */
	unsigned char ytempf;	/* y axis temp flag:if 1,the step calibration finished */
	unsigned char ztempf;	/* z axis temp flag:if 1,the step calibration finished */

	unsigned char xaddmis;	/* x axis mismtach register address */
	unsigned char yaddmis;	/* y axis mismtach register address */
	unsigned char zaddmis;	/* z axis mismtach register address */
	unsigned char xaddoff;	/* x axis offset register address */
	unsigned char yaddoff;	/* y axis offset register address */
	unsigned char zaddoff;	/* z axis offset register address */

	unsigned char (*MisDataSpaceConvert)(unsigned char continuous);	/* mismatch space convert function pointer */
	unsigned char (*OffDataSpaceConvert)(unsigned char continuous);	/* offset space convert function pointer */

};

static int ic_type = 0; //0:SC7A20;  1: SC7A20E

struct sc7a20_acc_data *sc7a20_acc_misc_data;
struct i2c_client *sc7a20_i2c_client;

/* static struct workqueue_struct *sc7a30_workqueue = NULL; */

static int sc7a20_acc_i2c_read(struct sc7a20_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = 0,	/* acc->client->flags & I2C_M_TEN, */
		 .len = 1,
		 .buf = buf,},
		{
		 .addr = acc->client->addr,
		 .flags = I2C_M_RD,	/* (acc->client->flags & I2C_M_TEN) | I2C_M_RD, */
		 .len = len,
		 .buf = buf,},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int sc7a20_acc_i2c_write(struct sc7a20_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = 0,	/* acc->client->flags & I2C_M_TEN, */
		 .len = len + 1,
		 .buf = buf,
		 },
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	/* pr_info("sc7a20_acc_i2c_write buf[0]=%x,buf[1]=%x,err=%d\n",buf[0],buf[1],err);//david */
	return err;
}

/* -------------------------------------- */

/*unsigned char Read_Reg(unsigned char reg)
{

	i2c_smbus_read_byte_data(sc7a20_i2c_acc, reg);
	return reg;
}*/
static char Read_Reg(unsigned char reg)
{

	char buf[2];

	buf[0] = reg;
	sc7a20_acc_i2c_read(sc7a20_i2c_acc, buf, 1);
	return buf[0];
}

/*static void Write_Input(char addr, char thedata)
{
	int result;
	char buffer[3]={0};
	buffer[0]= thedata;
	result = i2c_smbus_write_byte_data(sc7a20_i2c_acc, addr, thedata);
}*/

static void Write_Input(char addr, char thedata)
{

	char buf[2];

	buf[0] = addr;
	buf[1] = thedata;

	sc7a20_acc_i2c_write(sc7a20_i2c_acc, buf, 1);
	/* result = sensor_write_reg(sc7a20_i2c_acc, addr, thedata); */
	/* result = i2c_smbus_write_byte_data(sc7a20_i2c_acc, addr, thedata); */

}

static void Read_Output_3axis(unsigned char *acc_buf)
{
	char buffer[3] = { 0 };
	int index = 0;

	while (1) {
		msleep(20);
		*buffer = SC7A30_STATUS;
		buffer[0] = Read_Reg(0x27);
		if ((buffer[0] & 0x08) != 0) {
			break;
		}
		index++;
		if (index > 40)
			break;
	}
	/* 6 register data be read out   */

	buffer[0] = Read_Reg(SC7A30_XOUT_L);
	acc_buf[0] = buffer[0];
	buffer[0] = Read_Reg(SC7A30_XOUT_H);
	acc_buf[1] = buffer[0];

	buffer[0] = Read_Reg(SC7A30_YOUT_L);
	acc_buf[2] = buffer[0];
	buffer[0] = Read_Reg(SC7A30_YOUT_H);
	acc_buf[3] = buffer[0];

	buffer[0] = Read_Reg(SC7A30_ZOUT_L);
	acc_buf[4] = buffer[0];
	buffer[0] = Read_Reg(SC7A30_ZOUT_H);
	acc_buf[5] = buffer[0];
}

static void tilt_3axis_mtp(signed int x, signed int y, signed int z)
{
	char buffer[6] = { 0 };
	unsigned char buffer0[6] = { 0 };
	unsigned char buffer1[6] = { 0 };
	signed char mtpread[3] = { 0 };
	signed int xoutp, youtp, zoutp;
	signed int xoutpt, youtpt, zoutpt;
	signed char xtilt, ytilt, ztilt;

	xoutp = youtp = zoutp = 0;
	xoutpt = youtpt = zoutpt = 0;
	xtilt = ytilt = ztilt = 0;
	Read_Output_3axis(buffer0);
	Read_Output_3axis(buffer1);

	xoutpt = ((signed int)((buffer1[1] << 8) | buffer1[0])) >> 4;
	youtpt = ((signed int)((buffer1[3] << 8) | buffer1[2])) >> 4;
	zoutpt = ((signed int)((buffer1[5] << 8) | buffer1[4])) >> 4;

	xoutp = xoutpt - x * 16;
	youtp = youtpt - y * 16;
	zoutp = zoutpt - z * 16;

	buffer[0] = Read_Reg(0x10);
	mtpread[0] = (signed char)buffer[0];

	buffer[0] = Read_Reg(0x11);
	mtpread[1] = (signed char)buffer[0];

	buffer[0] = Read_Reg(0x12);
	mtpread[2] = (signed char)buffer[0];

	/* calculate the new tilt mtp value  */
	xtilt = (signed char)(xoutp / 8) + mtpread[0];
	ytilt = (signed char)(youtp / 8) + mtpread[1];
	ztilt = (signed char)(zoutp / 8) + mtpread[2];

	/* write the new into mtp */
	Write_Input(0x10, xtilt);
	Write_Input(0x11, ytilt);
	Write_Input(0x12, ztilt);
}

#if 0
static char IsNearZero(unsigned int Pos, unsigned int Neg, unsigned char error)
{
	if (abs(Pos) < error)
		return 1;
	if (abs(Neg) < error)
		return 2;
	return 0;
}
#endif
static unsigned char forword_MisDataSpaceConvert(unsigned char continuous)
{
	if (continuous >= 128)
		return continuous - 128;
	else
		return 255 - continuous;
}

static unsigned char reverse_MisDataSpaceConvert(unsigned char continuous)
{
	if (continuous >= 128)
		return continuous;
	else
		return 127 - continuous;
}

static unsigned char reverse_OffDataSpaceConvert(unsigned char continuous)
{
	return 127 - continuous;
}

static unsigned char forword_OffDataSpaceConvert(unsigned char continuous)
{
	return continuous;
}

/* set finalflag xfinalf-yfinalf-zfinalf upto the relative output */
static void check_output_set_finalflag(struct Cali_Data *pcalidata, unsigned char err)
{

	if (abs(pcalidata->xfoutput) < err) {
		/* pr_info("line:%d Xcali finish!Final=%d\n",__LINE__,pcalidata->xfoutput); */
		pcalidata->xfinalf = 1;
	}
	if (abs(pcalidata->yfoutput) < err) {
		/* pr_info("line:%d Xcali finish!Final=%d\n",__LINE__,pcalidata->yfoutput); */
		pcalidata->yfinalf = 1;
	}
	if (abs(pcalidata->zfoutput) < err) {
		/* pr_info("line:%d Xcali finish!Final=%d\n",__LINE__,pcalidata->zfoutput); */
		pcalidata->zfinalf = 1;
	}

}

static void check_finalflag_set_tempflag(struct Cali_Data *pcalidata)
{
	if (pcalidata->xfinalf) {
		pcalidata->xtempf = 1;
	}
	if (pcalidata->yfinalf) {
		pcalidata->ytempf = 1;
	}
	if (pcalidata->zfinalf) {
		pcalidata->ztempf = 1;
	}
}

static unsigned char check_flag_is_return(struct Cali_Data *pcalidata)
{
	if ((pcalidata->xfinalf) && (pcalidata->yfinalf) && (pcalidata->zfinalf)) {
		return 1;
	} else
		return 0;
}

/* updata middle mismatch register with the new mismatch */
static void updata_midmis_address(struct Cali_Data *pcalidata)
{
	if (pcalidata->xtempf == 0) {
		pcalidata->xmmis =
		    (unsigned char)(((unsigned int)(pcalidata->xpmis) + (unsigned int)(pcalidata->xnmis)) / 2);
		pcalidata->MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(pcalidata->xaddmis, (*(pcalidata->MisDataSpaceConvert)) (pcalidata->xmmis));
	}
	if (pcalidata->ytempf == 0) {
		pcalidata->ymmis =
		    (unsigned char)(((unsigned int)(pcalidata->ypmis) + (unsigned int)(pcalidata->ynmis)) / 2);
		pcalidata->MisDataSpaceConvert = forword_MisDataSpaceConvert;
		Write_Input(pcalidata->yaddmis, (*(pcalidata->MisDataSpaceConvert)) (pcalidata->ymmis));
	}
	if (pcalidata->ztempf == 0) {
		pcalidata->zmmis =
		    (unsigned char)(((unsigned int)(pcalidata->zpmis) + (unsigned int)(pcalidata->znmis)) / 2);
		pcalidata->MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(pcalidata->zaddmis, (*(pcalidata->MisDataSpaceConvert)) (pcalidata->zmmis));
	}
}

/* updata middle offset register with the new offset */
static void updata_midoff_address(struct Cali_Data *pcalidata)
{
	if (pcalidata->xtempf == 0) {
		pcalidata->xmoff =
		    (unsigned char)(((unsigned int)(pcalidata->xpoff) + (unsigned int)(pcalidata->xnoff)) / 2);
		pcalidata->OffDataSpaceConvert = reverse_OffDataSpaceConvert;
		Write_Input(pcalidata->xaddoff, (*(pcalidata->OffDataSpaceConvert)) (pcalidata->xmoff));
	}
	if (pcalidata->ytempf == 0) {
		pcalidata->ymoff =
		    (unsigned char)(((unsigned int)(pcalidata->ypoff) + (unsigned int)(pcalidata->ynoff)) / 2);
		pcalidata->OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(pcalidata->yaddoff, (*(pcalidata->OffDataSpaceConvert)) (pcalidata->ymoff));
	}
	if (pcalidata->ztempf == 0) {
		pcalidata->zmoff =
		    (unsigned char)(((unsigned int)(pcalidata->zpoff) + (unsigned int)(pcalidata->znoff)) / 2);
		pcalidata->OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(pcalidata->zaddoff, (*(pcalidata->OffDataSpaceConvert)) (pcalidata->zmoff));
	}
}

static void updata_mmis_pnfoutput_set_tempflag(struct Cali_Data *pcalidata,
					       unsigned char *buf, signed int xrel, signed int yrel, signed int zrel)
{
	/* output 2 struct data */
	pcalidata->xfoutput = (signed int)((signed char)buf[1]) - xrel;
	pcalidata->yfoutput = (signed int)((signed char)buf[3]) - yrel;
	pcalidata->zfoutput = (signed int)((signed char)buf[5]) - zrel;

	if (abs(pcalidata->xfoutput) < 25)
		pcalidata->xtempf = 1;
	if (abs(pcalidata->yfoutput) < 25)
		pcalidata->ytempf = 1;
	if (abs(pcalidata->zfoutput) < 25)
		pcalidata->ztempf = 1;

	if (pcalidata->xtempf == 0) {
		if (pcalidata->xfoutput > 0) {
			pcalidata->xpoutput = pcalidata->xfoutput;
			pcalidata->xpmis = pcalidata->xmmis;
		} else {
			pcalidata->xnoutput = pcalidata->xfoutput;
			pcalidata->xnmis = pcalidata->xmmis;
		}
	}

	if (pcalidata->ytempf == 0) {
		if (pcalidata->yfoutput > 0) {
			pcalidata->ypoutput = pcalidata->yfoutput;
			pcalidata->ypmis = pcalidata->ymmis;
		} else {
			pcalidata->ynoutput = pcalidata->yfoutput;
			pcalidata->ynmis = pcalidata->ymmis;
		}
	}

	if (pcalidata->ztempf == 0) {
		if (pcalidata->zfoutput > 0) {
			pcalidata->zpoutput = pcalidata->zfoutput;
			pcalidata->zpmis = pcalidata->zmmis;
		} else {
			pcalidata->znoutput = pcalidata->zfoutput;
			pcalidata->znmis = pcalidata->zmmis;
		}
	}
}

static void updata_moff_pnfoutput_set_tempflag(struct Cali_Data *pcalidata,
					       unsigned char *buf, signed int xrel, signed int yrel, signed int zrel)
{
	/* output 2 struct data */
	pcalidata->xfoutput = (signed int)((signed char)buf[1]) - xrel;
	pcalidata->yfoutput = (signed int)((signed char)buf[3]) - yrel;
	pcalidata->zfoutput = (signed int)((signed char)buf[5]) - zrel;

	if (abs(pcalidata->xfoutput) < 3)
		pcalidata->xtempf = 1;
	if (abs(pcalidata->yfoutput) < 3)
		pcalidata->ytempf = 1;
	if (abs(pcalidata->zfoutput) < 3)
		pcalidata->ztempf = 1;

	if (pcalidata->xtempf == 0) {
		if (pcalidata->xfoutput > 0) {
			pcalidata->xpoutput = pcalidata->xfoutput;
			pcalidata->xpoff = pcalidata->xmoff;
		} else {
			pcalidata->xnoutput = pcalidata->xfoutput;
			pcalidata->xnoff = pcalidata->xmoff;
		}
	}

	if (pcalidata->ytempf == 0) {
		if (pcalidata->yfoutput > 0) {
			pcalidata->ypoutput = pcalidata->yfoutput;
			pcalidata->ypoff = pcalidata->ymoff;
		} else {
			pcalidata->ynoutput = pcalidata->yfoutput;
			pcalidata->ynoff = pcalidata->ymoff;
		}
	}

	if (pcalidata->ztempf == 0) {
		if (pcalidata->zfoutput > 0) {
			pcalidata->zpoutput = pcalidata->zfoutput;
			pcalidata->zpoff = pcalidata->zmoff;
		} else {
			pcalidata->znoutput = pcalidata->zfoutput;
			pcalidata->znoff = pcalidata->zmoff;
		}
	}
}

int auto_calibration_instant(signed int x, signed int y, signed int z)
{

	unsigned char count = 0, cyclecount = 0;
	unsigned char acc_buf[6];

	struct Cali_Data calidata = { 0 };

	calidata.xaddmis = 0x40;
	calidata.yaddmis = 0x41;
	calidata.zaddmis = 0x42;
	calidata.xaddoff = 0x47;
	calidata.yaddoff = 0x48;
	calidata.zaddoff = 0x49;
#ifdef PRINT
	printf("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (UINT) calidata.xfinalf, (UINT) calidata.xtempf,
	       (UINT) calidata.yfinalf, (UINT) calidata.ytempf, (UINT) calidata.zfinalf, (UINT) calidata.ztempf);
#endif

	Read_Output_3axis(acc_buf);
	calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
	calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
	calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
	check_output_set_finalflag(&calidata, 2);
	if (check_flag_is_return(&calidata)) {
		pr_info("step1:=file=%s,line=%d calibration ok\n", __FILE__, __LINE__);
		return 1;
	}
#ifdef PRINT
	printf("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (UINT) calidata.xfinalf, (UINT) calidata.xtempf,
	       (UINT) calidata.yfinalf, (UINT) calidata.ytempf, (UINT) calidata.zfinalf, (UINT) calidata.ztempf);
#endif

	if (calidata.xfinalf == 0) {
		Write_Input(calidata.xaddoff, 0x3f);	/* cali mis under off=0x3f */
		Write_Input(0x10, 0);	/* tilt clear */
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.xaddmis, (*(calidata.MisDataSpaceConvert)) (255));	/* x mis to max */
	}

	if (calidata.yfinalf == 0) {
		Write_Input(calidata.yaddoff, 0x3f);	/* cali mis under off=0x3f */
		Write_Input(0x11, 0);	/* tilt clear */
		calidata.MisDataSpaceConvert = forword_MisDataSpaceConvert;
		Write_Input(calidata.yaddmis, (*(calidata.MisDataSpaceConvert)) (255));	/* y mis to max */
	}

	if (calidata.zfinalf == 0) {
		Write_Input(calidata.zaddoff, 0x3f);	/* cali mis under off=0x3f */
		Write_Input(0x12, 0);	/* tilt clear */
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.zaddmis, (*(calidata.MisDataSpaceConvert)) (255));	/* z mis to max */
	}

	Read_Output_3axis(acc_buf);
	calidata.xpoutput = calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
	calidata.ypoutput = calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
	calidata.zpoutput = calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
	if ((calidata.xpoutput < -25) || (calidata.ypoutput < -25) || (calidata.zpoutput < -25)) {
		pr_info("step2:=file=%s,line=%d  calibration failed\n", __FILE__, __LINE__);
		pr_info("calidata.xpoutput = 0x%4x  calidata.ypoutput = 0x%4x  calidata.zpoutput = 0x%4x\n",
		       calidata.xpoutput, calidata.ypoutput, calidata.zpoutput);
		return 0;
	}

	if (calidata.xfinalf == 0) {
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.xaddmis, (*(calidata.MisDataSpaceConvert)) (0));
	}

	if (calidata.yfinalf == 0) {
		calidata.MisDataSpaceConvert = forword_MisDataSpaceConvert;
		Write_Input(calidata.yaddmis, (*(calidata.MisDataSpaceConvert)) (0));
	}

	if (calidata.zfinalf == 0) {
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.zaddmis, (*(calidata.MisDataSpaceConvert)) (0));
	}

	Read_Output_3axis(acc_buf);
	calidata.xnoutput = calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
	calidata.ynoutput = calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
	calidata.znoutput = calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
	if ((calidata.xnoutput > 25) || (calidata.ynoutput > 25) || (calidata.znoutput > 25)) {
		pr_info("step2:=file=%s,line=%d  calibration failed\n", __FILE__, __LINE__);
		pr_info("calidata.xnoutput = 0x%4x  calidata.ynoutput = 0x%4x  calidata.znoutput = 0x%4x\n",
		       calidata.xnoutput, calidata.ynoutput, calidata.znoutput);
		return 0;
	}

	if (abs(calidata.xpoutput) <= abs(calidata.xnoutput)) {
		calidata.xfoutput = calidata.xpoutput;
		calidata.xmmis = 255;
	} else {
		calidata.xfoutput = calidata.xnoutput;
		calidata.xmmis = 0;
	}

	if (abs(calidata.ypoutput) <= abs(calidata.ynoutput)) {
		calidata.yfoutput = calidata.ypoutput;
		calidata.ymmis = 255;
	} else {
		calidata.yfoutput = calidata.ynoutput;
		calidata.ymmis = 0;
	}

	if (abs(calidata.zpoutput) <= abs(calidata.znoutput)) {
		calidata.zfoutput = calidata.zpoutput;
		calidata.zmmis = 255;
	} else {
		calidata.zfoutput = calidata.znoutput;
		calidata.zmmis = 0;
	}

	if (calidata.xfinalf == 0) {
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.xaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.xmmis));
	}

	if (calidata.yfinalf == 0) {
		calidata.MisDataSpaceConvert = forword_MisDataSpaceConvert;
		Write_Input(calidata.yaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.ymmis));
	}

	if (calidata.zfinalf == 0) {
		calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
		Write_Input(calidata.zaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.zmmis));
	}

	check_output_set_finalflag(&calidata, 2);
	/* if the smaller output <25,the step3 mis is finished */
	if (abs(calidata.xfoutput) < 25)
		calidata.xtempf = 1;
	if (abs(calidata.yfoutput) < 25)
		calidata.ytempf = 1;
	if (abs(calidata.zfoutput) < 25)
		calidata.ztempf = 1;

	calidata.xpmis = calidata.ypmis = calidata.zpmis = 255;
	calidata.xnmis = calidata.ynmis = calidata.znmis = 0;
	check_finalflag_set_tempflag(&calidata);
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	cyclecount = 0;
	while (1) {
		if (++cyclecount > 20)
			break;	/* if some errs happened,the cyclecount exceeded */

		if ((calidata.xtempf) && (calidata.ytempf) && (calidata.ztempf))
			break;
		updata_midmis_address(&calidata);
		Read_Output_3axis(acc_buf);
		calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
		calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
		calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
#ifdef PRINT
		pr_info
		    ("xp%4d=%4d,xm%4d=%4d,xn%4d=%4d,      yp%4d=%4d,ym%4d=%4d,yn%4d=%4d,      zp%4d=%4d,zm%4d=%4d,zn%4d=%4d\n\r",
		     calidata.xpoutput, (unsigned int)calidata.xpmis, calidata.xfoutput, (unsigned int)calidata.xmmis,
		     calidata.xnoutput, (unsigned int)calidata.xnmis, calidata.ypoutput, (unsigned int)calidata.ypmis,
		     calidata.yfoutput, (unsigned int)calidata.ymmis, calidata.ynoutput, (unsigned int)calidata.ynmis,
		     calidata.zpoutput, (unsigned int)calidata.zpmis, calidata.zfoutput, (unsigned int)calidata.zmmis,
		     calidata.znoutput, (unsigned int)calidata.znmis);
#endif
		updata_mmis_pnfoutput_set_tempflag(&calidata, acc_buf, x, y, z);
		check_output_set_finalflag(&calidata, 2);
		if (check_flag_is_return(&calidata))
			return 1;
	}
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif

	calidata.xtempf = calidata.ytempf = calidata.ztempf = 1;
	if ((calidata.xmmis > 0) && (calidata.xmmis < 255))
		calidata.xtempf = 0;
	if ((calidata.ymmis > 0) && (calidata.ymmis < 255))
		calidata.ytempf = 0;
	if ((calidata.zmmis > 0) && (calidata.zmmis < 255))
		calidata.ztempf = 0;
	calidata.xpmis = calidata.xnmis = calidata.xmmis;
	calidata.ypmis = calidata.ynmis = calidata.ymmis;
	calidata.zpmis = calidata.znmis = calidata.zmmis;
	for (count = 0; count < 3; count++) {
		if (calidata.xtempf == 0) {
			calidata.xpmis = calidata.xmmis + count - 1;
			if ((calidata.xpmis > calidata.xmmis) && (calidata.xpmis == 128))
				calidata.xpmis = calidata.xmmis + count - 1 + 1;
			if ((calidata.xpmis < calidata.xmmis) && (calidata.xpmis == 127))
				calidata.xpmis = calidata.xmmis + count - 1 - 1;
			calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
			Write_Input(calidata.xaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.xpmis));
		}

		if (calidata.ytempf == 0) {
			calidata.ypmis = calidata.ymmis + count - 1;
			if ((calidata.ypmis > calidata.ymmis) && (calidata.ypmis == 128))
				calidata.ypmis = calidata.ymmis + count - 1 + 1;
			if ((calidata.ypmis < calidata.ymmis) && (calidata.ypmis == 127))
				calidata.ypmis = calidata.ymmis + count - 1 - 1;
			calidata.MisDataSpaceConvert = forword_MisDataSpaceConvert;
			Write_Input(calidata.yaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.ypmis));
		}

		if (calidata.ztempf == 0) {
			calidata.zpmis = calidata.zmmis + count - 1;
			if ((calidata.zpmis > calidata.zmmis) && (calidata.zpmis == 128))
				calidata.zpmis = calidata.zmmis + count - 1 + 1;
			if ((calidata.zpmis < calidata.zmmis) && (calidata.zpmis == 127))
				calidata.zpmis = calidata.zmmis + count - 1 - 1;
			calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
			Write_Input(calidata.zaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.zpmis));
		}

		Read_Output_3axis(acc_buf);
		if (abs((signed int)((signed char)acc_buf[1]) - x) < abs(calidata.xfoutput)) {
			calidata.xnmis = calidata.xpmis;
			calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
		}

		if (abs((signed int)((signed char)acc_buf[3]) - y) < abs(calidata.yfoutput)) {
			calidata.ynmis = calidata.ypmis;
			calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
		}

		if (abs((signed int)((signed char)acc_buf[5]) - z) < abs(calidata.zfoutput)) {
			calidata.znmis = calidata.zpmis;
			calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
		}

		if (calidata.xtempf == 0) {
			calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
			Write_Input(calidata.xaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.xnmis));
		}

		if (calidata.ytempf == 0) {
			calidata.MisDataSpaceConvert = forword_MisDataSpaceConvert;
			Write_Input(calidata.yaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.ynmis));
		}

		if (calidata.ztempf == 0) {
			calidata.MisDataSpaceConvert = reverse_MisDataSpaceConvert;
			Write_Input(calidata.zaddmis, (*(calidata.MisDataSpaceConvert)) (calidata.znmis));
		}
#ifdef PRINT
		pr_info("L%4d:xf=%4d,xmis=%4d,yf=%4d,ymis=%4d,zf=%4d,zmis=%4d\n\r", __LINE__,
		       (signed int)((signed char)acc_buf[1]) - x, (unsigned int)calidata.xpmis,
		       (signed int)((signed char)acc_buf[3]) - y, (unsigned int)calidata.ypmis,
		       (signed int)((signed char)acc_buf[5]) - z, (unsigned int)calidata.zpmis);
#endif

	}

	calidata.xpoff = calidata.ypoff = calidata.zpoff = 0x7f;
	calidata.xnoff = calidata.ynoff = calidata.znoff = 0;
	calidata.xtempf = calidata.ytempf = calidata.ztempf = 0;
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	check_finalflag_set_tempflag(&calidata);
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	/* offset max */
	if (calidata.xtempf == 0) {
		calidata.OffDataSpaceConvert = reverse_OffDataSpaceConvert;
		Write_Input(calidata.xaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.xpoff));	/* x off to max */
	}

	if (calidata.ytempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.yaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.xpoff));	/* y off to max */
	}

	if (calidata.ztempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.zaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.xpoff));	/* z off to max */
	}

	Read_Output_3axis(acc_buf);
	calidata.xpoutput = calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
	calidata.ypoutput = calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
	calidata.zpoutput = calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	check_output_set_finalflag(&calidata, 2);
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif

	if (calidata.xtempf == 0) {
		calidata.OffDataSpaceConvert = reverse_OffDataSpaceConvert;
		Write_Input(calidata.xaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.xnoff));	/* x off to min */
	}

	if (calidata.ytempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.yaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.ynoff));	/* y off to min */
	}

	if (calidata.ztempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.zaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.znoff));	/* z off to min */
	}

	Read_Output_3axis(acc_buf);
	calidata.xnoutput = calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
	calidata.ynoutput = calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
	calidata.znoutput = calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	check_output_set_finalflag(&calidata, 2);
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	if (abs(calidata.xpoutput) <= abs(calidata.xnoutput)) {
		calidata.xfoutput = calidata.xpoutput;
		calidata.xmoff = calidata.xpoff;
	} else {
		calidata.xfoutput = calidata.xnoutput;
		calidata.xmoff = calidata.xnoff;
	}

	if (abs(calidata.ypoutput) <= abs(calidata.ynoutput)) {
		calidata.yfoutput = calidata.ypoutput;
		calidata.ymoff = calidata.ypoff;
	} else {
		calidata.yfoutput = calidata.ynoutput;
		calidata.ymoff = calidata.ynoff;
	}

	if (abs(calidata.zpoutput) <= abs(calidata.znoutput)) {
		calidata.zfoutput = calidata.zpoutput;
		calidata.zmoff = calidata.zpoff;
	} else {
		calidata.zfoutput = calidata.znoutput;
		calidata.zmoff = calidata.znoff;
	}

	if (calidata.xtempf == 0) {
		calidata.OffDataSpaceConvert = reverse_OffDataSpaceConvert;
		Write_Input(calidata.xaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.xmoff));
	}

	if (calidata.ytempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.yaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.ymoff));
	}

	if (calidata.ztempf == 0) {
		calidata.OffDataSpaceConvert = forword_OffDataSpaceConvert;
		Write_Input(calidata.zaddoff, (*(calidata.OffDataSpaceConvert)) (calidata.zmoff));
	}

	if ((calidata.xpoutput > 0 && calidata.xnoutput > 0) || (calidata.xpoutput < 0 && calidata.xnoutput < 0)) {
		calidata.xfinalf = 1;
	}

	if ((calidata.ypoutput > 0 && calidata.ynoutput > 0) || (calidata.ypoutput < 0 && calidata.ynoutput < 0)) {
		calidata.yfinalf = 1;
	}

	if ((calidata.zpoutput > 0 && calidata.znoutput > 0) || (calidata.zpoutput < 0 && calidata.znoutput < 0)) {
		calidata.zfinalf = 1;
	}

	check_finalflag_set_tempflag(&calidata);
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif
	cyclecount = 0;
	while (1) {
		if (++cyclecount > 20)
			break;

		if ((calidata.xtempf) && (calidata.ytempf) && (calidata.ztempf))
			break;
		updata_midoff_address(&calidata);
		Read_Output_3axis(acc_buf);
		calidata.xfoutput = (signed int)((signed char)acc_buf[1]) - x;
		calidata.yfoutput = (signed int)((signed char)acc_buf[3]) - y;
		calidata.zfoutput = (signed int)((signed char)acc_buf[5]) - z;
#ifdef PRINT
		pr_info
		    ("xp%4d=%4d,xm%4d=%4d,xn%4d=%4d,      yp%4d=%4d,ym%4d=%4d,yn%4d=%4d,      zp%4d=%4d,zm%4d=%4d,zn%4d=%4d\n\r",
		     calidata.xpoutput, (unsigned int)calidata.xpoff, calidata.xfoutput, (unsigned int)calidata.xmoff,
		     calidata.xnoutput, (unsigned int)calidata.xnoff, calidata.ypoutput, (unsigned int)calidata.ypoff,
		     calidata.yfoutput, (unsigned int)calidata.ymoff, calidata.ynoutput, (unsigned int)calidata.ynoff,
		     calidata.zpoutput, (unsigned int)calidata.zpoff, calidata.zfoutput, (unsigned int)calidata.zmoff,
		     calidata.znoutput, (unsigned int)calidata.znoff);
#endif
		updata_moff_pnfoutput_set_tempflag(&calidata, acc_buf, x, y, z);
		check_output_set_finalflag(&calidata, 2);
		if (check_flag_is_return(&calidata))
			return 1;
	}
#ifdef PRINT
	pr_info("L%4d:xff=%4d,xtf=%4d,yff=%4d,ytf=%4d,zff=%4d,ztf=%4d\n\r", __LINE__,
	       (unsigned int)calidata.xfinalf, (unsigned int)calidata.xtempf,
	       (unsigned int)calidata.yfinalf, (unsigned int)calidata.ytempf,
	       (unsigned int)calidata.zfinalf, (unsigned int)calidata.ztempf);
#endif

	return 1;
}

int auto_calibration_instant_mtp(signed int x, signed int y, signed int z)
{
	unsigned char readbuf[3] = { 0 };
	unsigned char buffer[6] = { 0 };
	char reg_13;
	/* unsigned char tbuffer[6] = {0};       */
	signed int xoutp, youtp, zoutp;
	unsigned char xfinalf, yfinalf, zfinalf;

	xoutp = youtp = zoutp = 0;
	xfinalf = yfinalf = zfinalf = 0;
	reg_13 = 0;
	Write_Input(0x1e, 0x05);
	if (auto_calibration_instant(x, y, z) == 0)
		return 0;

	/* msleep(20); */
	tilt_3axis_mtp(x, y, z);
	Read_Output_3axis(buffer);
	xoutp = (signed int)((signed char)buffer[1]) - x;
	youtp = (signed int)((signed char)buffer[3]) - y;
	zoutp = (signed int)((signed char)buffer[5]) - z;

	if (abs(xoutp) < 2) {
		xfinalf = 1;
	}
	if (abs(youtp) < 2) {
		yfinalf = 1;
	}
	if (abs(zoutp) < 2) {
		zfinalf = 1;
	}

	/* *tbuffer = 0x10; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[0] = Read_Reg(0x10);
	/* *tbuffer = 0x40; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[1] = Read_Reg(0x40);
	/* *tbuffer = 0x47; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[2] = Read_Reg(0x47);
	pr_info("L%4d:xtilt=%4d,xmis=%4d,xoff=%4d\n\r", __LINE__,
	       (unsigned int)readbuf[0], (unsigned int)readbuf[1], (unsigned int)readbuf[2]);

	/* *tbuffer = 0x11; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[0] = Read_Reg(0x11);
	/* *tbuffer = 0x41; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[1] = Read_Reg(0x41);
	/* *tbuffer = 0x48; */
	/* sensor_rx_data(sc7a30_client, tbuffer,1); */
	readbuf[2] = Read_Reg(0x48);
	pr_info("L%4d:ytilt=%4d,ymis=%4d,yoff=%4d\n\r", __LINE__,
	       (unsigned int)readbuf[0], (unsigned int)readbuf[1], (unsigned int)readbuf[2]);

	readbuf[0] = Read_Reg(0x12);

	readbuf[1] = Read_Reg(0x42);

	readbuf[2] = Read_Reg(0x49);
	pr_info("L%4d:ztilt=%4d,zmis=%4d,zoff=%4d\n\r", __LINE__,
	       (unsigned int)readbuf[0], (unsigned int)readbuf[1], (unsigned int)readbuf[2]);

	if (xfinalf && yfinalf && zfinalf) {
		Write_Input(0x13, 0x01);	/* allen MTP  */
		reg_13 = Read_Reg(0x13);
		pr_info("line %d  reg_13 = %x\n", __LINE__, reg_13);
		Write_Input(0x1e, 0x15);
		mdelay(300);
		reg_13 = Read_Reg(0x13);
		pr_info("line %d  reg_13 = %x\n", __LINE__, reg_13);
		Write_Input(0x1e, 05);

		pr_info("run calibration finished\n");

		return 1;
	} else
		return 0;
}

/* ------------------------------------add by david---end-------- */

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */

static int sc7a20_acc_hw_init(struct sc7a20_acc_data *acc)
{
	int err = -1;
	u8 buf[7];
	if(ic_type == 1){
		buf[0] = 0x58;
		buf[1] = 0x02;
		err = sc7a20_acc_i2c_write(acc, buf, 1);
		if (err < 0){
			pr_info("%s: hw init sc7a20_acc_i2c_write error\n", SC7A20_ACC_DEV_NAME);
		}
		buf[0] = 0x1f;
		buf[1] = 0x00;
		err = sc7a20_acc_i2c_write(acc, buf, 1);
		if (err < 0){
			pr_info("%s: hw init sc7a20_acc_i2c_write error\n", SC7A20_ACC_DEV_NAME);
		}
	}else{
		buf[0] = 0x1f;
		buf[1] = 0x40;
		err = sc7a20_acc_i2c_write(acc, buf, 1);
		if (err < 0){
			pr_info("%s: hw init sc7a20_acc_i2c_write error\n", SC7A20_ACC_DEV_NAME);
		}
	}
	/* int i=0; */
	pr_info("%s: hw init start\n", SC7A20_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = sc7a20_acc_i2c_read(acc, buf, 1);
	pr_info("err sc7a20_hw_init = %x\n", err);
	pr_info("who_am_i = %x\n", buf[0]);

/*
	buf[0] = WHO_AM_I;
	err = sc7a20_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_SC7A20_ACC) {
		err = -1;
		goto error_unknown_device;
	}
*/
	pr_info("%s A\n", __func__);

	buf[0] = 0x20;
	buf[1] = 0x77;

	err = sc7a20_acc_i2c_write(acc, buf, 1);

	if (err < 0)
		goto error1;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", SC7A20_ACC_DEV_NAME);
	return 0;

/*error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		 "available/working?\n");
	goto error1;*/
/*error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_SC7A20_ACC, buf[0]);*/
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0], buf[1], err);
	return err;
}

static void sc7a20_acc_device_power_off(struct sc7a20_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, SC7A20_ACC_PM_OFF };

	err = sc7a20_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if (acc->irq1 != 0)
			disable_irq_nosync(acc->irq1);
		if (acc->irq2 != 0)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if (acc->irq1 != 0)
			disable_irq_nosync(acc->irq1);
		if (acc->irq2 != 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}

static int sc7a20_acc_device_power_on(struct sc7a20_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(&acc->client->dev, "power_on failed: %d\n", err);
			return err;
		}
	}

	if (!acc->hw_initialized) {
		err = sc7a20_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			sc7a20_acc_device_power_off(acc);
			return err;
		}
	}
	return 0;
}

static irqreturn_t sc7a20_acc_isr1(int irq, void *dev)
{
	struct sc7a20_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	pr_info("%s: isr1 queued\n", SC7A20_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t sc7a20_acc_isr2(int irq, void *dev)
{
	struct sc7a20_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);

	pr_info("%s: isr2 queued\n", SC7A20_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void sc7a20_acc_irq1_work_func(struct work_struct *work)
{

	/*struct sc7a20_acc_data *acc =
	   container_of(work, struct sc7a20_acc_data, irq1_work);
	 */
	/* TODO  add interrupt service procedure.
	   ie:sc7a20_acc_get_int1_source(acc); */

	/*  */
	pr_info("%s: IRQ1 triggered\n", SC7A20_ACC_DEV_NAME);
}

static void sc7a20_acc_irq2_work_func(struct work_struct *work)
{

	/*struct sc7a20_acc_data *acc =
	   container_of(work, struct sc7a20_acc_data, irq2_work);
	 */
	/* TODO  add interrupt service procedure.
	   ie:sc7a20_acc_get_tap_source(acc); */

	/*  */

	pr_info("%s: IRQ2 triggered\n", SC7A20_ACC_DEV_NAME);
}

int sc7a20_acc_update_g_range(struct sc7a20_acc_data *acc, u8 new_g_range)
{
	int err;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = SC7A20_ACC_FS_MASK | HIGH_RESOLUTION;

	pr_info("%s\n", __func__);

	switch (new_g_range) {
	case SC7A20_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case SC7A20_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case SC7A20_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case SC7A20_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n", new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = sc7a20_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = sc7a20_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;

		pr_info("%s sensitivity %d g-range %d\n", __func__, sensitivity, new_g_range);
	}
	return 0;

error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n", buf[0], buf[1], err);
	return err;
}

int sc7a20_acc_update_odr(struct sc7a20_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(sc7a20_acc_odr_table) - 1; i >= 0; i--) {
		if (sc7a20_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = sc7a20_acc_odr_table[i].mask;

	config[1] |= SC7A20_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = sc7a20_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return 0;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n", config[0], config[1], err);

	return err;
}

/* */
static int sc7a20_acc_register_write(struct sc7a20_acc_data *acc, u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

	if (atomic_read(&acc->enabled)) {
		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = sc7a20_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}
	return err;
}

static int sc7a20_acc_register_read(struct sc7a20_acc_data *acc, u8 *buf, u8 reg_address)
{

	int err = -1;

	buf[0] = (reg_address);
	err = sc7a20_acc_i2c_read(acc, buf, 1);
	return err;
}

static int sc7a20_acc_register_update(struct sc7a20_acc_data *acc, u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;

	err = sc7a20_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = sc7a20_acc_register_write(acc, buf, reg_address, updated_val);
	}
	return err;
}

static int sensor_convert_data(char high_byte, char low_byte, s16 off)
{
	s64 result;

	result = (((s16) ((high_byte << 8) + low_byte)));

	return (int)result;
}

void find_max_min(s16 *array, int len, s16 *max, s16 *min)
{
	s16 temp_max = *array;
	s16 temp_min = *array;

	int i = 0;

	for (i = 0; i < len; i++) {
		if (*(array + i) > temp_max)
			temp_max = *(array + i);

		if (*(array + i) < temp_min)
			temp_min = *(array + i);
	}

	*max = temp_max;
	*min = temp_min;
}

#define SC7A30_DATA_LENGTH       6
static int sc7a20_acc_get_acceleration_data(struct sc7a20_acc_data *acc, int *xyz)
{
	int ret = -1;

	char buffer1[6] = { 0 };

	static s16 axis_x_off = 0;	/* jq 2017-08-07 */
	static s16 axis_y_off = 0;	/* jq 2017-08-07         */
	static s16 axis_z_off = 0;	/* jq 2017-08-07 */
	static u16 index = 0;
	static s16 x_value[SUM_DOT] = { 0 };
	static s16 y_value[SUM_DOT] = { 0 };
	static s16 z_value[SUM_DOT] = { 0 };

	int flag_x = 0;
	int flag_y = 0;
	int flag_z = 0;

	int i = 0;

#ifdef SILAN_SC7A20_DATA_UPDATE
	static s16 rand_count = 0;
#endif

	s16 x_min = 0;
	s16 x_max = 0;
	s16 y_min = 0;
	s16 y_max = 0;
	s16 z_min = 0;
	s16 z_max = 0;

	s32 temp_x_value = 0;
	s32 temp_y_value = 0;
	s32 temp_z_value = 0;

	s16 buffer3[6] = { 0 };

	*buffer1 = 0xa8;
	ret = sc7a20_acc_i2c_read(acc, buffer1, 6);

	if (index < SUM_DOT) {
		buffer3[0] = (s16) (s8) buffer1[0];
		buffer3[1] = (s16) (s8) buffer1[1];
		buffer3[2] = (s16) (s8) buffer1[2];
		buffer3[3] = (s16) (s8) buffer1[3];
		buffer3[4] = (s16) (s8) buffer1[4];
		buffer3[5] = (s16) (s8) buffer1[5];

		if (buffer3[1] > XY_THR_N && buffer3[1] < XY_THR_P)
			flag_x = 1;
		else
			flag_x = 0;

		if (buffer3[3] > XY_THR_N && buffer3[3] < XY_THR_P)
			flag_y = 1;
		else
			flag_y = 0;

		if ((buffer3[5] > Z_THR_MAX_N && buffer3[5] < Z_THR_MIN_N)
		    || (buffer3[5] > Z_THR_MIN_P && buffer3[5] < Z_THR_MAX_P))
			flag_z = 1;
		else
			flag_z = 0;

		if (flag_x == 1 && flag_y == 1 && flag_z == 1) {
			x_value[index] = buffer3[1];
			y_value[index] = buffer3[3];
			z_value[index] = buffer3[5];
			index = index + 1;
		} else
			index = 0;
	}

	if (index == SUM_DOT) {
		find_max_min(x_value, SUM_DOT, &x_max, &x_min);
		pr_info("aaaaa %s:x_max=%d\n  x_min=%d\n ", __func__, x_max, x_min);
		find_max_min(y_value, SUM_DOT, &y_max, &y_min);
		pr_info("aaaaa %s:y_max=%d\n  y_min=%d\n ", __func__, y_max, y_min);
		find_max_min(z_value, SUM_DOT, &z_max, &z_min);
		pr_info("aaaaa %s:z_max=%d\n  z_min=%d\n", __func__, z_max, z_min);

		if (((x_max - x_min) < THRESHOLD_VAL) && ((y_max - y_min) < THRESHOLD_VAL)
		    && ((z_max - z_min) < THRESHOLD_VAL)) {
			temp_x_value = 0;
			for (i = 0; i < SUM_DOT; i++) {
				temp_x_value += x_value[i];
			}

			temp_x_value = temp_x_value / SUM_DOT;
			axis_x_off = 0 - (s16) temp_x_value;
			temp_y_value = 0;

			for (i = 0; i < SUM_DOT; i++) {
				temp_y_value += y_value[i];
			}
			temp_y_value = temp_y_value / SUM_DOT;
			axis_y_off = 0 - (s16) temp_y_value;

			temp_z_value = 0;
			for (i = 0; i < SUM_DOT; i++) {
				temp_z_value += z_value[i];
			}
			temp_z_value = temp_z_value / SUM_DOT;

			if (temp_z_value > Z_THR_MAX_N && temp_z_value < Z_THR_MIN_N)
				axis_z_off = -64 - (s16) temp_z_value;
			else
				axis_z_off = 64 - (s16) temp_z_value;
			pr_info("aaaaa %s:axis_x_off=%d\n  axis_y_off=%d\n  axis_z_off=%d\n", __func__, axis_x_off,
					axis_y_off, axis_z_off);
		}
		/* index = 0; */
		index += 1;

	}

	buffer1[1] += axis_x_off;
	buffer1[3] += axis_y_off;
	buffer1[5] += axis_z_off;

	if(ic_type == 0){
		xyz[0] = sensor_convert_data(buffer1[1], buffer1[0], 0);	/* buffer[1]:high bit  */
		xyz[1] = sensor_convert_data(buffer1[3], buffer1[2], 0);	/* buffer[1]:high bit  */
		xyz[2] = sensor_convert_data(buffer1[5], buffer1[4], 0);	/* buffer[1]:high bit  */
		// pr_info("%s sc7a20 x=%d, y=%d, z=%d\n", SC7A20_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	}else{
		xyz[1] = sensor_convert_data(buffer1[1], buffer1[0], 0);	/* buffer[1]:high bit  */
		xyz[2] = sensor_convert_data(buffer1[3], buffer1[2], 0);	/* buffer[1]:high bit  */
		xyz[0] = sensor_convert_data(buffer1[5], buffer1[4], 0);	/* buffer[1]:high bit  */
		// pr_info("%s sc7a20e11 x=%d, y=%d, z=%d\n", SC7A20_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	}

#ifdef SILAN_SC7A20_FILTER
	xyz[0] = silan_filter_process(&core_channel.sl_channel[0], xyz[0]);
	xyz[1] = silan_filter_process(&core_channel.sl_channel[1], xyz[1]);
	xyz[2] = silan_filter_process(&core_channel.sl_channel[2], xyz[2]);
#endif

#ifdef SILAN_SC7A20_DATA_UPDATE
		rand_count = (rand_count + 1) % 2;
		xyz[0] += rand_count;
		xyz[1] += rand_count;
		xyz[2] += rand_count;
#endif

	return ret;
}

static void sc7a20_acc_report_values(struct sc7a20_acc_data *acc, int *xyz)
{
	s16 acc_x = 0, acc_y = 0, acc_z = 0;
	int ii;
	int placement_no = (int)acc->sc7a20_placement;

	for (ii = 0; ii < 3; ii++) {
		acc_x += xyz[ii] * coordinate_trans[placement_no][0][ii];
		acc_y += xyz[ii] * coordinate_trans[placement_no][1][ii];
		acc_z += xyz[ii] * coordinate_trans[placement_no][2][ii];
	}

	input_report_abs(acc->input_dev, ABS_X, acc_x);
	input_report_abs(acc->input_dev, ABS_Y, acc_y);
	input_report_abs(acc->input_dev, ABS_Z, acc_z);

	input_sync(acc->input_dev);
}

static int sc7a20_acc_enable(struct sc7a20_acc_data *acc)
{
	int err;

	pr_info("sc7a20: -- %s -- !\n", __func__);
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = sc7a20_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}

		if (acc->hw_initialized) {
			if (acc->irq1 != 0)
				enable_irq(acc->irq1);
			if (acc->irq2 != 0)
				enable_irq(acc->irq2);
			pr_info("%s: power on: irq enabled\n", SC7A20_ACC_DEV_NAME);
		}

		/* schedule_delayed_work(&acc->input_work, */
		/* msecs_to_jiffies(acc->pdata-> */
		/* poll_interval)); */
		bsc7a20_enable = true;
		hrtimer_start(&sc7a20_timer,
			      ktime_set(acc->pdata->poll_interval / 1000, (acc->pdata->poll_interval % 1000) * 1000000),
			      HRTIMER_MODE_REL);

	}
	pr_info("sc7a20: -- %s -- success!\n", __func__);
	return 0;
}

static int sc7a20_acc_disable(struct sc7a20_acc_data *acc)
{
	pr_info("sc7a20: -- %s --\n", __func__);
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		/* cancel_delayed_work_sync(&acc->input_work); */
		sc7a20_acc_device_power_off(acc);
	}
	bsc7a20_enable = false;
	hrtimer_cancel(&sc7a20_timer);

	return 0;
}

static int sc7a20_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = sc7a20_acc_misc_data;

	return 0;
}

static long sc7a20_acc_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	u8 mask;
	u8 reg_address;
	u8 bit_values;
	int err;
	/* int interval; */
	u32 interval;
	char state = 0;
	int xyz[3] = { 0 };
	struct sc7a20_acc_data *acc = file->private_data;

	switch (cmd) {
	case SC7A20_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case SC7A20_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		interval = interval / 1000000;
		if (interval < 0 || interval > 1000)
			return -EINVAL;

		/* acc->pdata->poll_interval = max(interval, */
		/* acc->pdata->min_interval); */

		if (interval > acc->pdata->min_interval)
			acc->pdata->poll_interval = interval;
		else
			acc->pdata->poll_interval = acc->pdata->min_interval;

		err = sc7a20_acc_update_odr(acc, acc->pdata->poll_interval);
		/* TODO: if update fails poll is still set */
		if (err < 0)
			return err;
		break;

	case SC7A20_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&state, argp, sizeof(char)))
			return -EFAULT;
		if (state > 1)
			return -EINVAL;
		if (state)
			err = sc7a20_acc_enable(acc);
		else
			err = sc7a20_acc_disable(acc);
		if (err)
			return err;
		break;

	case SC7A20_ACC_IOCTL_GET_ENABLE:
		state = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &state, sizeof(char)))
			return -EINVAL;
		break;

	case SC7A20_ACC_IOCTL_SET_G_RANGE:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		bit_values = buf[0];
		err = sc7a20_acc_update_g_range(acc, bit_values);
		if (err < 0)
			return err;
		break;

#ifdef INTERRUPT_MANAGEMENT
	case SC7A20_ACC_IOCTL_SET_CTRL_REG3:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = CTRL_REG3;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG3] = ((mask & bit_values) | (~mask & acc->resume_state[RES_CTRL_REG3]));
		break;

	case SC7A20_ACC_IOCTL_SET_CTRL_REG6:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = CTRL_REG6;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG6] = ((mask & bit_values) | (~mask & acc->resume_state[RES_CTRL_REG6]));
		break;

	case SC7A20_ACC_IOCTL_SET_DURATION1:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_DUR1;
		mask = 0x7F;
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_DUR1] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_DUR1]));
		break;

	case SC7A20_ACC_IOCTL_SET_THRESHOLD1:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_THS1;
		mask = 0x7F;
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_THS1] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_THS1]));
		break;

	case SC7A20_ACC_IOCTL_SET_CONFIG1:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = INT_CFG1;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_CFG1] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_CFG1]));
		break;

	case SC7A20_ACC_IOCTL_GET_SOURCE1:
		err = sc7a20_acc_register_read(acc, buf, INT_SRC1);
		if (err < 0)
			return err;

		pr_info("INT1_SRC content: %d , 0x%x\n", buf[0], buf[0]);

		if (copy_to_user(argp, buf, 1))
			return -EINVAL;
		break;

	case SC7A20_ACC_IOCTL_SET_DURATION2:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_DUR2;
		mask = 0x7F;
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_DUR2] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_DUR2]));
		break;

	case SC7A20_ACC_IOCTL_SET_THRESHOLD2:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_THS2;
		mask = 0x7F;
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_THS2] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_THS2]));
		break;

	case SC7A20_ACC_IOCTL_SET_CONFIG2:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = INT_CFG2;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_CFG2] = ((mask & bit_values) | (~mask & acc->resume_state[RES_INT_CFG2]));
		break;

	case SC7A20_ACC_IOCTL_GET_SOURCE2:
		err = sc7a20_acc_register_read(acc, buf, INT_SRC2);
		if (err < 0)
			return err;

		pr_info("INT2_SRC content: %d , 0x%x\n", buf[0], buf[0]);

		if (copy_to_user(argp, buf, 1))
			return -EINVAL;
		break;

	case SC7A20_ACC_IOCTL_GET_TAP_SOURCE:
		err = sc7a20_acc_register_read(acc, buf, TT_SRC);
		if (err < 0)
			return err;
		pr_info("TT_SRC content: %d , 0x%x\n", buf[0], buf[0]);

		if (copy_to_user(argp, buf, 1)) {
			pr_err("%s:%s error in copy_to_user\n", SC7A20_ACC_DEV_NAME, __func__);
			return -EINVAL;
		}
		break;
	case SC7A20_ACC_IOCTL_GET_COOR_XYZ:
		err = sc7a20_acc_get_acceleration_data(acc, xyz);
		if (err < 0)
			return err;

		if (copy_to_user(argp, xyz, sizeof(xyz))) {
			pr_err("%s %d error in copy_to_user\n", __func__, __LINE__);
			return -EINVAL;
		}
		break;
	case SC7A20_ACC_IOCTL_SET_TAP_CFG:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_CFG;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_CFG] = ((mask & bit_values) | (~mask & acc->resume_state[RES_TT_CFG]));
		break;

	case SC7A20_ACC_IOCTL_SET_TAP_TLIM:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_LIM;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_LIM] = ((mask & bit_values) | (~mask & acc->resume_state[RES_TT_LIM]));
		break;

	case SC7A20_ACC_IOCTL_SET_TAP_THS:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_THS;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_THS] = ((mask & bit_values) | (~mask & acc->resume_state[RES_TT_THS]));
		break;

	case SC7A20_ACC_IOCTL_SET_TAP_TLAT:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_TLAT;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_TLAT] = ((mask & bit_values) | (~mask & acc->resume_state[RES_TT_TLAT]));
		break;

	case SC7A20_ACC_IOCTL_SET_TAP_TW:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_TW;
		mask = buf[1];
		bit_values = buf[0];
		err = sc7a20_acc_register_update(acc, (u8 *) arg, reg_address, mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_TW] = ((mask & bit_values) | (~mask & acc->resume_state[RES_TT_TW]));
		break;

#endif /* INTERRUPT_MANAGEMENT */
	case SC7A20_ACC_IOCTL_GET_CHIP_ID:
		{
			u8 devid = 0;
			u8 devinfo[DEVICE_INFO_LEN] = { 0 };

			err = sc7a20_acc_register_read(acc, &devid, WHO_AM_I);
			if (err < 0) {
				/* pr_info("__func__, error read register WHO_AM_I\n", __func__); */
				return -EAGAIN;
			}
			sprintf(devinfo, "%s, %#x", DEVICE_INFO, devid);

			if (copy_to_user(argp, devinfo, sizeof(devinfo))) {
				pr_info("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
				return -EINVAL;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations sc7a20_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = sc7a20_acc_misc_open,
	.unlocked_ioctl = sc7a20_acc_misc_ioctl,
};

static struct miscdevice sc7a20_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sc7a20_acc",
	.fops = &sc7a20_acc_misc_fops,
};

static void sc7a20_acc_input_work_func(struct work_struct *work)
{
	struct sc7a20_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	/* acc = container_of((struct delayed_work *)work, */
	/* struct sc7a20_acc_data, input_work); */

	acc = container_of((struct work_struct *)work, struct sc7a20_acc_data, input_work);

	mutex_lock(&acc->lock);

	if (bsc7a20_enable)
		hrtimer_start(&sc7a20_timer,
			      ktime_set(acc->pdata->poll_interval / 1000, (acc->pdata->poll_interval % 1000) * 1000000),
			      HRTIMER_MODE_REL);

	err = sc7a20_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		sc7a20_acc_report_values(acc, xyz);
	/* schedule_delayed_work(&acc->input_work, */
	/* msecs_to_jiffies(acc->pdata->poll_interval)); */
	mutex_unlock(&acc->lock);
}

#ifdef SC7A20_OPEN_ENABLE
int sc7a20_acc_input_open(struct input_dev *input)
{
	struct sc7a20_acc_data *acc = input_get_drvdata(input);

	return sc7a20_acc_enable(acc);
}

void sc7a20_acc_input_close(struct input_dev *dev)
{
	struct sc7a20_acc_data *acc = input_get_drvdata(dev);

	sc7a20_acc_disable(acc);
}
#endif

static int sc7a20_acc_validate_pdata(struct sc7a20_acc_data *acc)
{
	/* acc->pdata->poll_interval = max(acc->pdata->poll_interval, */
	/* acc->pdata->min_interval); */

	if (acc->pdata->poll_interval > acc->pdata->min_interval)
		acc->pdata->poll_interval = acc->pdata->poll_interval;
	else
		acc->pdata->poll_interval = acc->pdata->min_interval;

	if (acc->pdata->axis_map_x > 2 || acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x, acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 || acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x, acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int sc7a20_acc_input_init(struct sc7a20_acc_data *acc)
{
	int err;
	/* Polling rx data when the interrupt is not used. */

	INIT_WORK(&acc->input_work, sc7a20_acc_input_work_func);

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef SC7A20_ACC_OPEN_ENABLE
	acc->input_dev->open = sc7a20_acc_input_open;
	acc->input_dev->close = sc7a20_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*      next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*      next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -SC7A30_RANGE, SC7A30_RANGE, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -SC7A30_RANGE, SC7A30_RANGE, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -SC7A30_RANGE, SC7A30_RANGE, 0, 0);
	/*      next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*      next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	acc->input_dev->name = "accelerometer";
	/* acc->input_dev->name = "gsensor"; */

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev, "unable to register input polled device %s\n", acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void sc7a20_acc_input_cleanup(struct sc7a20_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static ssize_t sc7a20_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc7a20_acc_data *sc7a20_acc = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&sc7a20_acc->enabled));
}

static ssize_t sc7a20_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc7a20_acc_data *sc7a20_acc = i2c_get_clientdata(client);
	int err;
	unsigned long enable;

	err = kstrtoul(buf, 10, &enable);
	if (err) {
		pr_info("%s kstrtoul failed, error=%d\n", __func__, err);
		return err;
	}

	pr_info("%s called  %lu\n", __func__, enable);
	if (enable)
		err = sc7a20_acc_enable(sc7a20_acc);
	else
		err = sc7a20_acc_disable(sc7a20_acc);

	if (err < 0) {
		atomic_set(&sc7a20_acc->enabled, 0);
		pr_info("%s called  error\n", __func__);
		return err;
	}
	return count;
}

static ssize_t sc7a20_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc7a20_acc_data *sc7a20_acc = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", sc7a20_acc->pdata->poll_interval);
}

static ssize_t sc7a20_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc7a20_acc_data *sc7a20_acc = i2c_get_clientdata(client);
	int err;
	unsigned long delay;

	err = kstrtoul(buf, 10, &delay);
	if (err) {
		pr_info("%s kstrtoul failed, error=%d\n", __func__, err);
		return err;
	}
	/* pr_info(" called 1st %d\n",  delay); */
	/* sc7a20_acc->pdata->poll_interval = max(delay, */
	/* sc7a20_acc->pdata->min_interval); */

	if (delay > sc7a20_acc->pdata->min_interval)
		sc7a20_acc->pdata->poll_interval = delay;
	else
		sc7a20_acc->pdata->poll_interval = sc7a20_acc->pdata->min_interval;

/* pr_info("%s called 2nd %d\n", __func__, sc7a20_acc->pdata->poll_interval); */
	err = sc7a20_acc_update_odr(sc7a20_acc, sc7a20_acc->pdata->poll_interval);
	/* TODO: if update fails poll is still set */
	if (err < 0) {
		pr_info("%s called 3th %d\n", __func__, sc7a20_acc->pdata->poll_interval);
		err = sc7a20_acc_update_odr(sc7a20_acc, sc7a20_acc->pdata->poll_interval);
		if (err < 0) {
			pr_info("%s called error\n", __func__);
			return err;
		}
	}
	return count;
}

static ssize_t sc7a20_identify_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* struct i2c_client *client = to_i2c_client(dev); */
/* struct sc7a20_acc_data *sc7a20_acc = i2c_get_clientdata(client); */

	return 0;
}

static ssize_t SC7A30_3_axis_Calibration(struct device *dev, struct device_attribute *attr, const char *buf,
					 size_t count)
{
	/* unsigned temp[3]; */

	Write_Input(0x20, 0x77);
	Write_Input(0x1e, 0x05);
	mdelay(100);
	auto_calibration_instant_mtp(0, 0, -64);

	Write_Input(0x1e, 0x15);
	mdelay(300);
	/* Write_Input(0x1e, 0);         */
	/* mdelay(5);            */
	/* Write_Input(0x20, 0x47); */
	pr_info("%s,run calibration finished\n", __func__);

	mdelay(5);
	return 1;
}

static ssize_t SC7A30_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int address, value;
	int ret = 0;

	/* int result = 0; */
	ret = sscanf(buf, "0x%x=0x%x", &address, &value);
	if (!ret) {
		pr_info("%s fail to store reg!\n", __func__);
		return -EINVAL;
	}


	/* result = sensor_write_reg(sc7a30_client, address,value); */

	/* if(result) */
	/* pr_info("%s:fail to write sensor_register\n",__func__); */
	Write_Input(address, value);

	return count;
}

static ssize_t SC7A30_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* size_t count = 0; */
	unsigned int count = 0;
	size_t buf_len = 0;
	/* u8 reg[0x5b]; */
	char i;
	char buffer[3] = { 0 };

	i = 0x0f;
	buffer[0] = i;
	/* count = sensor_rx_data(sc7a30_client, &buffer[0], 1);  */
	count = Read_Reg(buffer[0]);
	buf_len = sprintf(buf, "0x%x: 0x%x\n", i, count);
	for (i = 0x10; i < 0x5a; i++) {
		/* buffer[0] = i;                */
		/* sensor_rx_data(sc7a30_client, &buffer[0], 1); */
		buffer[0] = Read_Reg(i);
		buf_len += sprintf(&buf[buf_len], "0x%x: 0x%x\n", i, buffer[0]);
	}
	return buf_len;
}

static ssize_t sc7a20_chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("%s called\n", __func__);

	return snprintf(buf, 64, "%s", SC7A20_ACC_NAME);
}

static DEVICE_ATTR(enable, 0664, sc7a20_enable_show, sc7a20_enable_store);
static DEVICE_ATTR(delay, 0664, sc7a20_delay_show, sc7a20_delay_store);
static DEVICE_ATTR(identify, 0444, sc7a20_identify_show, NULL);
static DEVICE_ATTR(reg, 0664,	/* S_IRUGO|S_IWUSR|S_IWGRP, */
		   SC7A30_register_show, SC7A30_register_store);
static DEVICE_ATTR(calibration_run, 0664,	/* S_IWUSR|S_IWGRP, //calibration_run */
		   NULL, SC7A30_3_axis_Calibration);
static DEVICE_ATTR(chip_info, 0444, sc7a20_chip_info_show, NULL);

static struct attribute *sc7a20_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_identify.attr,
	&dev_attr_reg.attr,
	&dev_attr_calibration_run.attr,
	&dev_attr_chip_info.attr,
	NULL
};

static struct attribute_group sc7a20_attribute_group = {
	.attrs = sc7a20_attributes
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sc7a20_early_suspend(struct early_suspend *es);
static void sc7a20_early_resume(struct early_suspend *es);
#endif

#ifdef CONFIG_OF
static struct sc7a20_platform_data *sc7a20_acc_parse_dt(struct device *dev)
{
	struct sc7a20_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pr_info("%s begin!", __func__);
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct sc7a20_platform_data");
		return NULL;
	}
	ret = of_property_read_u32(np, "poll_interval", &pdata->poll_interval);
	if (ret) {
		dev_err(dev, "fail to get poll_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "min_interval", &pdata->min_interval);
	if (ret) {
		dev_err(dev, "fail to get min_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "g_range", &pdata->g_range);
	if (ret) {
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x);
	if (ret) {
		dev_err(dev, "fail to get axis_map_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y);
	if (ret) {
		dev_err(dev, "fail to get axis_map_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z);
	if (ret) {
		dev_err(dev, "fail to get axis_map_z\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_x", &pdata->negate_x);
	if (ret) {
		dev_err(dev, "fail to get negate_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_y", &pdata->negate_y);
	if (ret) {
		dev_err(dev, "fail to get negate_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_z", &pdata->negate_z);
	if (ret) {
		dev_err(dev, "fail to get negate_z\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "direction", &pdata->direction);
	if (ret) {
		dev_err(dev, "fail to get direction\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static enum hrtimer_restart sc7a20_timer_func(struct hrtimer *timer)
{
	// pr_info("###### %s\n", __func__);
	schedule_work(&sc7a20_acc_misc_data->input_work);
	return HRTIMER_NORESTART;
}

static int sc7a20_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np;
	struct sc7a20_acc_data *acc;
	struct sc7a20_platform_data *pdata = client->dev.platform_data;
	u32 real_address;

	int err = -1;
	int tempvalue;
	int ver_value;
	u8 chip_id;
	u8 ver_id;

#ifdef SILAN_SC7A20_FILTER
	int j = 0;

	/* configure default filter param */
	core_channel.filter_param_l  = 2;
	core_channel.filter_param_h  = 32;
	core_channel.filter_threhold = 33;   //4G scale: 33; 2G scale: 65

	for (j = 0; j < 3; j++) {
		core_channel.sl_channel[j].sample_l = 0;
		core_channel.sl_channel[j].sample_h = 0;
		core_channel.sl_channel[j].flag_l = 0;
		core_channel.sl_channel[j].flag_h = 0;
	}
#endif

	pr_info("%s: probe start.\n", SC7A20_ACC_DEV_NAME);
	if (sensor_markinfo.ACC_have_registered) {
		pr_err("acc sensor have been register\n");
		return -ENODEV;
	}

#ifdef CONFIG_OF
	np = client->dev.of_node;
	if (np && !pdata) {
		pdata = sc7a20_acc_parse_dt(&client->dev);
		if (pdata) {
			client->dev.platform_data = pdata;
		}
		if (!pdata) {
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	pr_info("sc7a30: -- %s -- start ! A\n", __func__);
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	pr_info("sc7a30: -- %s -- start !B\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */
	pr_info("sc7a30: -- %s -- start !C\n", __func__);
	acc = kzalloc(sizeof(struct sc7a20_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for module data: %d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);
	pr_info("sc7a30: -- %s -- start D!\n", __func__);

	err = of_property_read_u32(np, "real_addr", &real_address);
	if (err) {
		dev_err(&client->dev, "fail to get real_reg,use default address value!\n");
	}

	client->addr = real_address;
	acc->client = client;
	sc7a20_i2c_client = client;

	i2c_set_clientdata(client, acc);
	sc7a20_i2c_acc = i2c_get_clientdata(client);

#ifdef CONFIG_ARCH_SC8810
	acc->irq1 = sprd_alloc_eic_irq(GSENSOR_GINT1_GPI);
	acc->irq2 = sprd_alloc_eic_irq(GSENSOR_GINT2_GPI);
#else
	acc->irq1 = 0;
	acc->irq2 = 0;
#endif

	if (acc->irq1 != 0) {
		pr_info("%s request irq1\n", __func__);
		err = request_irq(acc->irq1, sc7a20_acc_isr1, IRQF_TRIGGER_RISING, "sc7a20_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_mutexunlockfreedata;
		}
		disable_irq_nosync(acc->irq1);

		INIT_WORK(&acc->irq1_work, sc7a20_acc_irq1_work_func);
		acc->irq1_work_queue = create_singlethread_workqueue("sc7a20_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue1: %d\n", err);
			goto err_free_irq1;
		}
	}

	if (acc->irq2 != 0) {
		err = request_irq(acc->irq2, sc7a20_acc_isr2, IRQF_TRIGGER_RISING, "sc7a20_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq2);

/*Create workqueue for IRQ.*/
		INIT_WORK(&acc->irq2_work, sc7a20_acc_irq2_work_func);
		acc->irq2_work_queue = create_singlethread_workqueue("sc7a20_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue2: %d\n", err);
			goto err_free_irq2;
		}
	}

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for pdata: %d\n", err);
		goto err_destoyworkqueue2;
	}
	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = sc7a20_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (i2c_smbus_read_byte(client) < 0) {
		pr_err("i2c_smbus_read_byte error!!\n");
		goto err2;
	} else {
		pr_info("%s Device detected!\n", SC7A20_ACC_DEV_NAME);
	}

	err = sc7a20_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	chip_id = (u8)(tempvalue & 0x00FF);
	pr_info("%s chip id: 0x%x !\n", __func__, chip_id);

	if ((tempvalue & 0x00FF) == SC7A20_WHO_AM_I_WIA_ID) {
		pr_info("%s I2C driver registered!\n", SC7A20_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		pr_info("%s I2C driver not registered! Device unknown 0x%x\n", __func__, chip_id);
		goto err_destoyworkqueue2;
	}

#ifdef SILAN_SC7A20_VERSION
	ver_value = i2c_smbus_read_word_data(client, SC7A20_VERSION_ADDR);
	ver_id = (u8)(ver_value & 0x00FF);
	pr_info("SC7A20 VERSION ID: 0x%x\n", ver_id);
	if ((ver_value & 0x00FF) == SC7A20_VERSION_VAL) {
		ic_type = 1;
		pr_info( "%s Version ID : 0x%x!\n",SC7A20_ACC_DEV_NAME, SC7A20_VERSION_VAL);
	} else {
		ic_type = 0;
	}
#endif

	i2c_set_clientdata(client, acc);

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	acc->sc7a20_placement = acc->pdata->direction;

	err = sc7a20_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&acc->enabled, 1);

	err = sc7a20_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = sc7a20_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}
	pr_info("sc7a30: -- %s -- start !H\n", __func__);
	err = sc7a20_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}
	err = sysfs_create_group(&acc->input_dev->dev.kobj, &sc7a20_attribute_group);
	if (err < 0) {
		pr_info("%s: create group fail!\n", __func__);
		goto err_input_cleanup;
	}
	sc7a20_acc_misc_data = acc;

	hrtimer_init(&sc7a20_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sc7a20_timer.function = sc7a20_timer_func;

	err = misc_register(&sc7a20_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "misc SC7A20_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}
	pr_info("sc7a30: -- %s -- start !I\n", __func__);
	sc7a20_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
	acc->early_suspend.suspend = sc7a20_early_suspend;
	acc->early_suspend.resume = sc7a20_early_resume;
	acc->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&acc->early_suspend);
#endif

	mutex_unlock(&acc->lock);

	sensor_markinfo.ACC_have_registered = true;
	dev_info(&client->dev, "###%s###\n", __func__);
	pr_info("sc7a30: -- %s -- success !\n", __func__);
	return 0;
err_input_cleanup:
	sc7a20_acc_input_cleanup(acc);
err_power_off:
	sc7a20_acc_device_power_off(acc);
err2:
	/* pr_info("**********enter err2\n"); */
	/* if (acc->pdata->exit) */
	/* acc->pdata->exit(); */
exit_kfree_pdata:
	pr_info("%s**********kfree acc pdata\n", __func__);
	kfree(acc->pdata);
err_destoyworkqueue2:
	if (acc->irq2_work_queue)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq2:
	if (acc->irq2) {
		free_irq(acc->irq2, acc);
	}
err_destoyworkqueue1:
	if (acc->irq1_work_queue)
		destroy_workqueue(acc->irq1_work_queue);
err_free_irq1:
	if (acc->irq1) {
		free_irq(acc->irq1, acc);
	}
err_mutexunlockfreedata:
#ifdef CONFIG_ARCH_SC8810
	sprd_free_eic_irq(acc->irq1);
	sprd_free_eic_irq(acc->irq2);
#endif
	mutex_unlock(&acc->lock);
	kfree(acc);
	acc = NULL;
	i2c_set_clientdata(client, NULL);
	sc7a20_acc_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", SC7A20_ACC_DEV_NAME);
exit_alloc_platform_data_failed:
	return err;
}

static int sc7a20_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct sc7a20_acc_data *acc = i2c_get_clientdata(client);

	if (acc != NULL) {
		if (acc->irq1) {
			free_irq(acc->irq1, acc);

		}
		if (acc->irq2) {
			free_irq(acc->irq2, acc);

		}
#ifdef CONFIG_ARCH_SC8810
		if (acc->irq1 != 0)
			sprd_free_eic_irq(acc->irq1);
		if (acc->irq2 != 0)
			sprd_free_eic_irq(acc->irq2);
#else
		/* if (acc->irq1 != 0) sprd_alloc_gpio_irq(acc->irq1); */
		/* if (acc->irq2 != 0) sprd_alloc_gpio_irq(acc->irq2); */
#endif

		if (acc->irq1_work_queue)
			destroy_workqueue(acc->irq1_work_queue);
		if (acc->irq2_work_queue)
			destroy_workqueue(acc->irq2_work_queue);
		misc_deregister(&sc7a20_acc_misc_device);
		sc7a20_acc_input_cleanup(acc);
		sc7a20_acc_device_power_off(acc);
		if (acc->pdata->exit)
			acc->pdata->exit();
		kfree(acc->pdata);
		kfree(acc);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sc7a20_early_suspend(struct early_suspend *es)
{
	sc7a20_acc_suspend(sc7a20_i2c_client, (pm_message_t) {
			   .event = 0});
}

static void sc7a20_early_resume(struct early_suspend *es)
{
	sc7a20_acc_resume(sc7a20_i2c_client);
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id sc7a20_acc_id[]
= { {SC7A20_ACC_DEV_NAME, 0}, {}, };

MODULE_DEVICE_TABLE(i2c, sc7a20_acc_id);

static const struct of_device_id sc7a20_acc_of_match[] = {
	{.compatible = "SC,sc7a20_acc",},
	{}
};

MODULE_DEVICE_TABLE(of, sc7a20_acc_of_match);

static struct i2c_driver sc7a20_acc_driver = {
	.driver = {
		   .name = SC7A20_ACC_I2C_NAME,
		   .of_match_table = sc7a20_acc_of_match,
		   },
	.probe = sc7a20_acc_probe,
	.remove = sc7a20_acc_remove,
	/* .resume = sc7a20_acc_resume, */
	/* .suspend = sc7a20_acc_suspend, */
	.id_table = sc7a20_acc_id,
};

static int __init sc7a20_acc_init(void)
{
	return i2c_add_driver(&sc7a20_acc_driver);
}

static void __exit sc7a20_acc_exit(void)
{
	pr_info("%s accelerometer driver exit\n", SC7A20_ACC_DEV_NAME);

#ifdef I2C_BUS_NUM_STATIC_ALLOC
	/* i2c_unregister_device(this_client); */
#endif

	i2c_del_driver(&sc7a20_acc_driver);
}

module_init(sc7a20_acc_init);
module_exit(sc7a20_acc_exit);

MODULE_DESCRIPTION("sc7a30 accelerometer misc driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
