/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     ov16b10_mipi_sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ov16b10z6530v_mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "ov16b10_camera_sensor"
#define LOG_1 LOG_INF("OV16B10,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2304*1728@30fps,102Mbps/lane; capture 16M@30fps,1500Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV16B10_SENSOR_ID,

	.checksum_value = 0xf7375923,	/*checksum value for Camera Auto Test */

	.pre = {
				.pclk = 90007200,	/* record different mode's pclk */
				.linelength = 810,	/* record different mode's linelength */
				.framelength = 3704,	/* record different mode's framelength */
				.startx = 0,	/* record different mode's startx of grabwindow */
				.starty = 0,	/* record different mode's starty of grabwindow */
				.grabwindow_width = 2304,	/* record different mode's width of grabwindow */
				.grabwindow_height = 1728,	/* record different mode's height of grabwindow */
				/*  by different scenario    */
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				/*     following for GetDefaultFramerateByScenario()    */
				.mipi_pixel_rate = 432000000,
				.max_framerate = 300,
			},
	.cap = {
				.pclk = 90007200,
				.linelength = 810,
				.framelength = 3704,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 4608,
				.grabwindow_height = 3456,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 600000000,
				.max_framerate = 300,
				},
	.normal_video = {
				.pclk = 90007200,
				.linelength = 810,
				.framelength = 3704,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 2304,
				.grabwindow_height = 1728,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 432000000,
				.max_framerate = 300,
				},
	.hs_video = {
				.pclk = 90000000,
				.linelength = 720,
				.framelength = 1042,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 1168,
				.grabwindow_height = 876,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 432000000,
				.max_framerate = 1200,
				},
	#if 0
	.hs_video = {
				.pclk = 90000000,
				.linelength = 630,
				.framelength = 1190,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 1280,
				.grabwindow_height = 720,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 432000000,
				.max_framerate = 1200,
				},
	#endif
	.slim_video = {
				.pclk = 90007200,
				.linelength = 810,
				.framelength = 3704,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 2304,
				.grabwindow_height = 1728,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 432000000,
				.max_framerate = 300,
				},
	.custom1 = {
				.pclk = 90007200,
				.linelength = 810,
				.framelength = 3704,
				.startx = 0,
				.starty = 0,
				.grabwindow_width = 4096,
				.grabwindow_height = 3072,
				.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
				.mipi_pixel_rate = 600000000,
				.max_framerate = 300,
				},

	.margin = 16,		/* sensor framelength & shutter margin */
	.min_shutter = 1,	/* min shutter */
	.max_frame_length = 0x7fff,	/* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	/* shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,
	/* sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 6,	/* support sensor mode num */

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 0,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_2MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x6c, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,	/* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,
	/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	/* current scenario id */
	.ihdr_en = 0,		/* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x6c,	/* record current sensor's i2c write id */
};



/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728},
	{4608, 3456, 0, 0, 4608, 3456, 4608, 3456, 0, 0, 4608, 3456, 0, 0, 4608, 3456},	/* capture */
	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728},	/* video */
	{4704, 3536, 0, 0, 4704, 3536, 1176, 884, 3, 4, 1170, 876, 0, 0, 1168, 876},
	/*{4608, 3456, 0, 0, 4608, 3456, 1280, 720, 0, 0, 1280, 720, 0, 0, 1280, 720},*//* hight speed video */
	{4608, 3456, 0, 0, 4608, 3456, 2304, 1728, 0, 0, 2304, 1728, 0, 0, 2304, 1728},	/* slim video */
	{4608, 3456, 0, 0, 4608, 3456, 4096, 3072, 0, 0, 4096, 3072, 0, 0, 4096, 3072},/*custom1*/
};

static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x35, 0x0280, 0x0001,
	 0x00, 0x2f, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000},
	/* Capture mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1220, 0x0da0, 0x01, 0x00, 0x0000, 0x0000,
	 0x01, 0x2b, 0x168, 0x6c0, 0x03, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1070, 0x0C30, 0x01, 0x00, 0x0000, 0x0000,
	 0x02, 0x2f, 0x0000, 0x0000, 0x03, 0x00, 0x0000, 0x0000}
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 32,
	.i4PitchY = 32,
	.i4PairNum = 16,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,		/* need check xb.pang */
	.i4BlockNumX = 144,
	.i4BlockNumY = 108,
	.i4PosR = {
	    {5, 2}, {13, 2}, {21, 2}, {29, 2}, {1, 10}, {9, 10}, {17, 10}, {25, 10},
		{5, 18}, {13, 18}, {21, 18}, {29, 18}, {1, 26}, {9, 26}, {17, 26}, {25, 26},},
	.i4PosL = {
	    {6, 2}, {14, 2}, {22, 2}, {30, 2}, {2, 10}, {10, 10}, {18, 10}, {26, 10},
		{6, 18}, {14, 18}, {22, 18}, {30, 18}, {2, 26}, {10, 26}, {18, 26}, {26, 26},},
	.iMirrorFlip = 0,
};

#if 0
static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX = 16,
	.i4PitchY = 16,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 8,		/* need check xb.pang */
	.i4BlockNumX = 288,
	.i4BlockNumY = 216,
	.i4PosR = {
	    {5, 2}, {13, 2}, {1, 10}, {9, 10},},
	.i4PosL = {
	    {6, 2}, {14, 2}, {2, 10}, {10, 10},},
	.iMirrorFlip = 0,
};
#endif



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, */
	/* or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);

}				/*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x300B) << 8) | read_cmos_sensor(0x300C));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min_framelength_en = %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
	    (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}				/*    set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	LOG_INF("write_shutter, shutter = %d\n", shutter);

	if (imgsensor.sensor_mode == IMGSENSOR_MODE_HIGH_SPEED_VIDEO) {
		if (shutter > imgsensor.frame_length - imgsensor_info.margin) {
			shutter = imgsensor.frame_length - imgsensor_info.margin;
		}
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3502, shutter & 0xFF);
		write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);
		LOG_INF("mtk_test highspeed video write_shutter , shutter =%d, framelength =%d\n", shutter,
		imgsensor.frame_length);
		return;
	}

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter =
	    (shutter >
	     (imgsensor_info.max_frame_length -
	      imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
					 imgsensor_info.margin) : shutter;

	/* Framelength should be an even number */
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}

	/* Update Shutter */
	/*mtk_test close shutter*/
	write_cmos_sensor(0x3502, shutter & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);

	LOG_INF("[A5142] << write_shutter , shutter =%d, framelength =%d\n", shutter,
		imgsensor.frame_length);
}


/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{

	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter set_shutter_frame_length! shutter =%d\n", shutter);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	}
	write_cmos_sensor(0x3502, shutter & 0xFF);
	write_cmos_sensor(0x3501, (shutter >> 8) & 0xFF);

	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
	reg_gain = reg_gain & 0xFFFF;

	return (kal_uint16) reg_gain;
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X    */
	/* [4:9] = M meams M X         */
	/* Total gain = M + N /16 X   */

	/*  */
	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	reg_gain = gain >> 6;
	write_cmos_sensor(0x3508, reg_gain);
	reg_gain = (gain & 0x3F) << 2;
	write_cmos_sensor(0x3509, reg_gain);

	return gain;
}				/*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	/* not support HDR */
	/* LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain); */
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x3820, ((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
		write_cmos_sensor(0x3821, ((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}

}
#endif

/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}				/*    night_mode    */

static void sensor_init(void)
{
	LOG_INF("E\n");
write_cmos_sensor(0x0100, 0x00);

write_cmos_sensor(0x0103, 0x01);
		write_cmos_sensor(0x0102, 0x01);
		write_cmos_sensor(0x0304, 0x5a);
		write_cmos_sensor(0x030b, 0x05);
		write_cmos_sensor(0x030c, 0x01);
		write_cmos_sensor(0x030d, 0x68);
		write_cmos_sensor(0x030e, 0x00);
		write_cmos_sensor(0x030f, 0x05);
		write_cmos_sensor(0x0311, 0x01);
		write_cmos_sensor(0x0324, 0x00);
		write_cmos_sensor(0x0314, 0x05);
		write_cmos_sensor(0x0315, 0x01);
		write_cmos_sensor(0x0316, 0x68);
		write_cmos_sensor(0x0318, 0x00);
		write_cmos_sensor(0x031c, 0x0b);
		write_cmos_sensor(0x031d, 0x01);
		write_cmos_sensor(0x0320, 0x15);
		write_cmos_sensor(0x3010, 0x01);
		write_cmos_sensor(0x3012, 0x41);
		write_cmos_sensor(0x3016, 0xb4);
		write_cmos_sensor(0x3017, 0xf0);
		write_cmos_sensor(0x3018, 0xf0);
		write_cmos_sensor(0x3019, 0xd2);
		write_cmos_sensor(0x301b, 0x1e);
		write_cmos_sensor(0x301e, 0x98);
		write_cmos_sensor(0x3022, 0xd0);
		write_cmos_sensor(0x3025, 0x03);
		write_cmos_sensor(0x3026, 0x10);
		write_cmos_sensor(0x3027, 0x08);
		write_cmos_sensor(0x3028, 0xc3);
		write_cmos_sensor(0x3032, 0xc1);
		write_cmos_sensor(0x3103, 0x0e);
		write_cmos_sensor(0x3106, 0x00);
		write_cmos_sensor(0x3400, 0x04);
		write_cmos_sensor(0x3410, 0x00);
		write_cmos_sensor(0x3408, 0x03);
		write_cmos_sensor(0x3412, 0x00);
		write_cmos_sensor(0x3413, 0x00);
		write_cmos_sensor(0x3414, 0x00);
		write_cmos_sensor(0x3415, 0x00);
		write_cmos_sensor(0x340c, 0xff);
		write_cmos_sensor(0x3421, 0x08);
		write_cmos_sensor(0x3423, 0x11);
		write_cmos_sensor(0x3424, 0x40);
		write_cmos_sensor(0x3425, 0x10);
		write_cmos_sensor(0x3426, 0x11);
		write_cmos_sensor(0x3501, 0x10);
		write_cmos_sensor(0x3502, 0x22);
		write_cmos_sensor(0x3504, 0x08);
		write_cmos_sensor(0x3508, 0x01);
		write_cmos_sensor(0x3509, 0x00);
		write_cmos_sensor(0x350a, 0x01);
		write_cmos_sensor(0x350b, 0x00);
		write_cmos_sensor(0x350c, 0x00);
		write_cmos_sensor(0x3548, 0x01);
		write_cmos_sensor(0x3549, 0x00);
		write_cmos_sensor(0x354a, 0x01);
		write_cmos_sensor(0x354b, 0x00);
		write_cmos_sensor(0x354c, 0x00);
		write_cmos_sensor(0x3603, 0x0f);
		write_cmos_sensor(0x3608, 0x51);
		write_cmos_sensor(0x3619, 0xf9);
		write_cmos_sensor(0x361a, 0x0f);
		write_cmos_sensor(0x361c, 0xf9);
		write_cmos_sensor(0x361d, 0x0f);
		write_cmos_sensor(0x3623, 0x44);
		write_cmos_sensor(0x3624, 0x88);
		write_cmos_sensor(0x3625, 0x86);
		write_cmos_sensor(0x3626, 0xf9);
		write_cmos_sensor(0x3627, 0x33);
		write_cmos_sensor(0x3628, 0x77);
		write_cmos_sensor(0x3629, 0x0b);
		write_cmos_sensor(0x362b, 0x03);
		write_cmos_sensor(0x362d, 0x03);
		write_cmos_sensor(0x3633, 0x34);
		write_cmos_sensor(0x3634, 0x30);
		write_cmos_sensor(0x3635, 0x30);
		write_cmos_sensor(0x3636, 0x30);
		write_cmos_sensor(0x3637, 0xcc);
		write_cmos_sensor(0x3638, 0xcc);
		write_cmos_sensor(0x3653, 0x00);
		write_cmos_sensor(0x3639, 0xce);
		write_cmos_sensor(0x363a, 0xcc);
		write_cmos_sensor(0x3644, 0x40);
		write_cmos_sensor(0x361e, 0x87);
		write_cmos_sensor(0x3664, 0x47);
		write_cmos_sensor(0x3631, 0xf2);
		write_cmos_sensor(0x3632, 0xf2);
		write_cmos_sensor(0x360b, 0xdc);
		write_cmos_sensor(0x360a, 0xf1);
		write_cmos_sensor(0x363e, 0x28);
		write_cmos_sensor(0x363d, 0x14);
		write_cmos_sensor(0x363c, 0x0a);
		write_cmos_sensor(0x363b, 0x05);
		write_cmos_sensor(0x3700, 0x38);
		write_cmos_sensor(0x3701, 0x1b);
		write_cmos_sensor(0x3702, 0x48);
		write_cmos_sensor(0x3703, 0x32);
		write_cmos_sensor(0x3704, 0x1b);
		write_cmos_sensor(0x3705, 0x00);
		write_cmos_sensor(0x3707, 0x06);
		write_cmos_sensor(0x3708, 0x2d);
		write_cmos_sensor(0x3709, 0xa0);
		write_cmos_sensor(0x370a, 0x00);
		write_cmos_sensor(0x370c, 0x09);
		write_cmos_sensor(0x3711, 0x40);
		write_cmos_sensor(0x3712, 0x00);
		write_cmos_sensor(0x3716, 0x44);
		write_cmos_sensor(0x3717, 0x02);
		write_cmos_sensor(0x3718, 0x14);
		write_cmos_sensor(0x3719, 0x11);
		write_cmos_sensor(0x371a, 0x1c);
		write_cmos_sensor(0x371b, 0xa0);
		write_cmos_sensor(0x371c, 0x03);
		write_cmos_sensor(0x371d, 0x1b);
		write_cmos_sensor(0x371e, 0x14);
		write_cmos_sensor(0x371f, 0x09);
		write_cmos_sensor(0x3721, 0x10);
		write_cmos_sensor(0x3722, 0x04);
		write_cmos_sensor(0x3723, 0x00);
		write_cmos_sensor(0x3724, 0x06);
		write_cmos_sensor(0x3725, 0x32);
		write_cmos_sensor(0x3727, 0x23);
		write_cmos_sensor(0x3728, 0x11);
		write_cmos_sensor(0x3730, 0x05);
		write_cmos_sensor(0x3731, 0x05);
		write_cmos_sensor(0x3732, 0x05);
		write_cmos_sensor(0x3733, 0x05);
		write_cmos_sensor(0x3734, 0x05);
		write_cmos_sensor(0x3735, 0x05);
		write_cmos_sensor(0x3736, 0x08);
		write_cmos_sensor(0x3737, 0x02);
		write_cmos_sensor(0x3738, 0x08);
		write_cmos_sensor(0x3739, 0x02);
		write_cmos_sensor(0x373a, 0x08);
		write_cmos_sensor(0x373b, 0x04);
		write_cmos_sensor(0x373c, 0x08);
		write_cmos_sensor(0x373d, 0x08);
		write_cmos_sensor(0x373e, 0x08);
		write_cmos_sensor(0x373f, 0x1b);
		write_cmos_sensor(0x3740, 0x05);
		write_cmos_sensor(0x3741, 0x03);
		write_cmos_sensor(0x3742, 0x05);
		write_cmos_sensor(0x3743, 0x08);
		write_cmos_sensor(0x3744, 0x18);
		write_cmos_sensor(0x3745, 0x08);
		write_cmos_sensor(0x3746, 0x18);
		write_cmos_sensor(0x3747, 0x3c);
		write_cmos_sensor(0x3748, 0x00);
		write_cmos_sensor(0x3749, 0xb4);
		write_cmos_sensor(0x374a, 0x0f);
		write_cmos_sensor(0x374b, 0x27);
		write_cmos_sensor(0x374d, 0x17);
		write_cmos_sensor(0x374e, 0x2f);
		write_cmos_sensor(0x374f, 0x07);
		write_cmos_sensor(0x3751, 0x05);
		write_cmos_sensor(0x3752, 0x00);
		write_cmos_sensor(0x3753, 0x00);
		write_cmos_sensor(0x375b, 0x00);
		write_cmos_sensor(0x375c, 0x00);
		write_cmos_sensor(0x375d, 0x00);
		write_cmos_sensor(0x3760, 0x08);
		write_cmos_sensor(0x3761, 0x10);
		write_cmos_sensor(0x3762, 0x08);
		write_cmos_sensor(0x3763, 0x08);
		write_cmos_sensor(0x3764, 0x08);
		write_cmos_sensor(0x3765, 0x10);
		write_cmos_sensor(0x3766, 0x18);
		write_cmos_sensor(0x3767, 0x28);
		write_cmos_sensor(0x3768, 0x00);
		write_cmos_sensor(0x3769, 0x08);
		write_cmos_sensor(0x376a, 0x10);
		write_cmos_sensor(0x376b, 0x00);
		write_cmos_sensor(0x3770, 0x24);
		write_cmos_sensor(0x3773, 0x30);
		write_cmos_sensor(0x3774, 0x3f);
		write_cmos_sensor(0x3775, 0x32);
		write_cmos_sensor(0x3776, 0x09);
		write_cmos_sensor(0x3777, 0x03);
		write_cmos_sensor(0x3778, 0x3e);
		write_cmos_sensor(0x3779, 0x15);
		write_cmos_sensor(0x377a, 0x0f);
		write_cmos_sensor(0x377b, 0x33);
		write_cmos_sensor(0x377c, 0x0b);
		write_cmos_sensor(0x377d, 0x03);
		write_cmos_sensor(0x377e, 0x05);
		write_cmos_sensor(0x377f, 0x03);
		write_cmos_sensor(0x379b, 0x0c);
		write_cmos_sensor(0x379c, 0x0c);
		write_cmos_sensor(0x379d, 0x0c);
		write_cmos_sensor(0x379e, 0x24);
		write_cmos_sensor(0x379f, 0x24);
		write_cmos_sensor(0x37a0, 0x09);
		write_cmos_sensor(0x37a1, 0x09);
		write_cmos_sensor(0x37a2, 0x03);
		write_cmos_sensor(0x37a3, 0x09);
		write_cmos_sensor(0x37a4, 0x03);
		write_cmos_sensor(0x37a5, 0x35);
		write_cmos_sensor(0x37a6, 0x0c);
		write_cmos_sensor(0x37a7, 0x06);
		write_cmos_sensor(0x37a8, 0x06);
		write_cmos_sensor(0x37a9, 0x06);
		write_cmos_sensor(0x37aa, 0x06);
		write_cmos_sensor(0x37ab, 0x0a);
		write_cmos_sensor(0x37ac, 0x0f);
		write_cmos_sensor(0x37ad, 0x0f);
		write_cmos_sensor(0x37ae, 0x0f);
		write_cmos_sensor(0x37b0, 0x3f);
		write_cmos_sensor(0x37b1, 0x3f);
		write_cmos_sensor(0x37b2, 0x08);
		write_cmos_sensor(0x37b3, 0x48);
		write_cmos_sensor(0x37b4, 0x48);
		write_cmos_sensor(0x37b5, 0x10);
		write_cmos_sensor(0x37b6, 0x00);
		write_cmos_sensor(0x37b7, 0x00);
		write_cmos_sensor(0x37b8, 0x00);
		write_cmos_sensor(0x37b9, 0x00);
		write_cmos_sensor(0x37ba, 0x00);
		write_cmos_sensor(0x37bc, 0x00);
		write_cmos_sensor(0x37bd, 0x00);
		write_cmos_sensor(0x37be, 0x00);
		write_cmos_sensor(0x37bf, 0x00);
		write_cmos_sensor(0x37c0, 0x05);
		write_cmos_sensor(0x37c1, 0x02);
		write_cmos_sensor(0x37c2, 0x03);
		write_cmos_sensor(0x37c3, 0x05);
		write_cmos_sensor(0x37c4, 0x03);
		write_cmos_sensor(0x37c5, 0x05);
		write_cmos_sensor(0x37c8, 0x03);
		write_cmos_sensor(0x37c9, 0x03);
		write_cmos_sensor(0x37cc, 0xf0);
		write_cmos_sensor(0x37cd, 0x02);
		write_cmos_sensor(0x37ce, 0x08);
		write_cmos_sensor(0x37cf, 0x02);
		write_cmos_sensor(0x37d0, 0x08);
		write_cmos_sensor(0x37d1, 0x02);
		write_cmos_sensor(0x37d2, 0x08);
		write_cmos_sensor(0x37d3, 0x02);
		write_cmos_sensor(0x37d4, 0x03);
		write_cmos_sensor(0x37d5, 0x0c);
		write_cmos_sensor(0x37d6, 0x17);
		write_cmos_sensor(0x37d7, 0x24);
		write_cmos_sensor(0x37d8, 0x05);
		write_cmos_sensor(0x37d9, 0x14);
		write_cmos_sensor(0x37da, 0x00);
		write_cmos_sensor(0x37de, 0x02);
		write_cmos_sensor(0x37df, 0x00);
		write_cmos_sensor(0x37e0, 0x18);
		write_cmos_sensor(0x37e1, 0x03);
		write_cmos_sensor(0x37e2, 0x05);
		write_cmos_sensor(0x37e3, 0x98);
		write_cmos_sensor(0x37e4, 0x08);
		write_cmos_sensor(0x37e5, 0x18);
		write_cmos_sensor(0x37e6, 0x08);
		write_cmos_sensor(0x37e7, 0x18);
		write_cmos_sensor(0x37e8, 0x08);
		write_cmos_sensor(0x37e9, 0x1d);
		write_cmos_sensor(0x37eb, 0x05);
		write_cmos_sensor(0x37ec, 0x05);
		write_cmos_sensor(0x37ed, 0x08);
		write_cmos_sensor(0x37ee, 0x02);
		write_cmos_sensor(0x37f0, 0x08);
		write_cmos_sensor(0x37f1, 0x02);
		write_cmos_sensor(0x37f2, 0x08);
		write_cmos_sensor(0x37f3, 0x02);
		write_cmos_sensor(0x37f6, 0x17);
		write_cmos_sensor(0x37f7, 0x02);
		write_cmos_sensor(0x37f8, 0x2d);
		write_cmos_sensor(0x37f9, 0x02);
		write_cmos_sensor(0x37fa, 0x02);
		write_cmos_sensor(0x37fb, 0x02);
		write_cmos_sensor(0x37fc, 0x00);
		write_cmos_sensor(0x37fd, 0x00);
		write_cmos_sensor(0x37fe, 0x00);
		write_cmos_sensor(0x37ff, 0x00);
		write_cmos_sensor(0x3802, 0x00);
		write_cmos_sensor(0x3803, 0x00);
		write_cmos_sensor(0x3806, 0x0c);
		write_cmos_sensor(0x3807, 0x43);
		write_cmos_sensor(0x3808, 0x12);
		write_cmos_sensor(0x3809, 0x40);
		write_cmos_sensor(0x380a, 0x0d);
		write_cmos_sensor(0x380b, 0xb0);
		write_cmos_sensor(0x380c, 0x02);
		write_cmos_sensor(0x380d, 0xd0);
		write_cmos_sensor(0x380e, 0x10);
		write_cmos_sensor(0x380f, 0x46);
		write_cmos_sensor(0x3810, 0x00);
		write_cmos_sensor(0x3811, 0x0f);
		write_cmos_sensor(0x3812, 0x00);
		write_cmos_sensor(0x3813, 0x00);
		write_cmos_sensor(0x3814, 0x11);
		write_cmos_sensor(0x3815, 0x11);
		write_cmos_sensor(0x3816, 0x01);
		write_cmos_sensor(0x3817, 0x10);
		write_cmos_sensor(0x3818, 0x01);
		write_cmos_sensor(0x3819, 0x00);
		write_cmos_sensor(0x3820, 0x00);
		write_cmos_sensor(0x3821, 0x04);
		write_cmos_sensor(0x3822, 0x00);
		write_cmos_sensor(0x3823, 0x01);
		write_cmos_sensor(0x382a, 0x01);
		write_cmos_sensor(0x383d, 0xec);
		write_cmos_sensor(0x383e, 0x0d);
		write_cmos_sensor(0x3f01, 0x12);
		write_cmos_sensor(0x3d85, 0x0b);
		write_cmos_sensor(0x3d8c, 0x77);
		write_cmos_sensor(0x3d8d, 0xa0);
		write_cmos_sensor(0x4009, 0x02);
		write_cmos_sensor(0x4010, 0x28);
		write_cmos_sensor(0x4011, 0x01);
		write_cmos_sensor(0x4012, 0x1d);
		write_cmos_sensor(0x4015, 0x00);
		write_cmos_sensor(0x4016, 0x1f);
		write_cmos_sensor(0x4017, 0x08);
		write_cmos_sensor(0x4018, 0x0f);
		write_cmos_sensor(0x401a, 0x40);
		write_cmos_sensor(0x401e, 0x01);
		write_cmos_sensor(0x401f, 0x79);
		write_cmos_sensor(0x4028, 0x00);
		write_cmos_sensor(0x4056, 0x25);
		write_cmos_sensor(0x4057, 0x20);
		write_cmos_sensor(0x4020, 0x04);
		write_cmos_sensor(0x4021, 0x00);
		write_cmos_sensor(0x4022, 0x04);
		write_cmos_sensor(0x4023, 0x00);
		write_cmos_sensor(0x4024, 0x04);
		write_cmos_sensor(0x4025, 0x00);
		write_cmos_sensor(0x4026, 0x04);
		write_cmos_sensor(0x4027, 0x00);
		write_cmos_sensor(0x4502, 0x88);
		write_cmos_sensor(0x4504, 0x02);
		write_cmos_sensor(0x4510, 0x08);
		write_cmos_sensor(0x4640, 0x00);
		write_cmos_sensor(0x4641, 0x24);
		write_cmos_sensor(0x4643, 0x08);
		write_cmos_sensor(0x4645, 0x04);
		write_cmos_sensor(0x4800, 0x80);
		write_cmos_sensor(0x4809, 0x2b);
		write_cmos_sensor(0x480e, 0x06);
		write_cmos_sensor(0x4813, 0x98);
		write_cmos_sensor(0x4837, 0x07);
		write_cmos_sensor(0x4855, 0x4c);
		write_cmos_sensor(0x4a00, 0x10);
		write_cmos_sensor(0x4d00, 0x04);
		write_cmos_sensor(0x4d01, 0xb1);
		write_cmos_sensor(0x4d02, 0xbc);
		write_cmos_sensor(0x4d03, 0x5d);
		write_cmos_sensor(0x4d04, 0x72);
		write_cmos_sensor(0x4d05, 0x48);
		write_cmos_sensor(0x5000, 0x0b);
		write_cmos_sensor(0x5001, 0x4b);
		write_cmos_sensor(0x5002, 0x15);
		write_cmos_sensor(0x5004, 0x00);
		write_cmos_sensor(0x501e, 0x00);
		write_cmos_sensor(0x5005, 0x40);
		write_cmos_sensor(0x501c, 0x00);
		write_cmos_sensor(0x501d, 0x00);
		write_cmos_sensor(0x5038, 0x00);
		write_cmos_sensor(0x5081, 0x80);
		write_cmos_sensor(0x5180, 0x00);
		write_cmos_sensor(0x5181, 0x10);
		write_cmos_sensor(0x5182, 0x03);
		write_cmos_sensor(0x5183, 0xcc);
		write_cmos_sensor(0x5200, 0x6f);
		write_cmos_sensor(0x5820, 0xc5);
		write_cmos_sensor(0x5860, 0x00);
		write_cmos_sensor(0x5ac2, 0x61);
		write_cmos_sensor(0x5acc, 0x00);
		write_cmos_sensor(0x5acd, 0x00);
		write_cmos_sensor(0x5ace, 0x00);
		write_cmos_sensor(0x5acf, 0x08);
		write_cmos_sensor(0x5aff, 0x10);
		write_cmos_sensor(0x5b01, 0x08);
		write_cmos_sensor(0x5b02, 0x12);
		write_cmos_sensor(0x5b03, 0x40);
		write_cmos_sensor(0x5b04, 0x0d);
		write_cmos_sensor(0x5b05, 0xc0);
		write_cmos_sensor(0x5b10, 0x00);
		write_cmos_sensor(0x5b11, 0x10);
		write_cmos_sensor(0x5b12, 0x00);
		write_cmos_sensor(0x5b13, 0x08);
		write_cmos_sensor(0x5b14, 0x12);
		write_cmos_sensor(0x5b15, 0x40);
		write_cmos_sensor(0x5b16, 0x0d);
		write_cmos_sensor(0x5b17, 0xb0);
		write_cmos_sensor(0x5aeb, 0x10);
		write_cmos_sensor(0x5aec, 0x00);
		write_cmos_sensor(0x5ac4, 0x00);
		write_cmos_sensor(0x5ac5, 0x6f);
		write_cmos_sensor(0x5ac6, 0x00);
		write_cmos_sensor(0x5ac7, 0x94);
		write_cmos_sensor(0x5ae9, 0x11);
		write_cmos_sensor(0x5aed, 0x02);
		write_cmos_sensor(0x5aee, 0x00);
		write_cmos_sensor(0x5aef, 0x00);
		write_cmos_sensor(0x5af2, 0x10);
		write_cmos_sensor(0x5af3, 0x10);
		write_cmos_sensor(0x5af4, 0x02);
		write_cmos_sensor(0x5af5, 0x0a);
		write_cmos_sensor(0x5af6, 0x06);
		write_cmos_sensor(0x5af7, 0x0e);
		write_cmos_sensor(0x5af8, 0x02);
		write_cmos_sensor(0x5af9, 0x02);
		write_cmos_sensor(0x5afa, 0x0a);
		write_cmos_sensor(0x5afb, 0x0a);
		write_cmos_sensor(0x5b06, 0x03);
		write_cmos_sensor(0x5b07, 0x03);
		write_cmos_sensor(0x5b08, 0x03);
		write_cmos_sensor(0x5b09, 0x03);
		write_cmos_sensor(0x5d02, 0x80);
		write_cmos_sensor(0x5d03, 0x44);
		write_cmos_sensor(0x5d05, 0xfc);
		write_cmos_sensor(0x5d06, 0x0b);
		write_cmos_sensor(0x5d07, 0x08);
		write_cmos_sensor(0x5d08, 0x10);
		write_cmos_sensor(0x5d09, 0x10);
		write_cmos_sensor(0x5d0a, 0x02);
		write_cmos_sensor(0x5d0b, 0x0a);
		write_cmos_sensor(0x5d0c, 0x06);
		write_cmos_sensor(0x5d0d, 0x0e);
		write_cmos_sensor(0x5d0e, 0x02);
		write_cmos_sensor(0x5d0f, 0x02);
		write_cmos_sensor(0x5d10, 0x0a);
		write_cmos_sensor(0x5d11, 0x0a);
		write_cmos_sensor(0x5d34, 0x00);
		write_cmos_sensor(0x5d35, 0x10);
		write_cmos_sensor(0x5d36, 0x00);
		write_cmos_sensor(0x5d37, 0x08);
		write_cmos_sensor(0x5d38, 0x12);
		write_cmos_sensor(0x5d39, 0x40);
		write_cmos_sensor(0x5d3a, 0x0d);
		write_cmos_sensor(0x5d3b, 0xc0);
		write_cmos_sensor(0x5d4a, 0x01);
		write_cmos_sensor(0x5d46, 0x00);
		write_cmos_sensor(0x5d47, 0x00);
		write_cmos_sensor(0x5d48, 0x00);
		write_cmos_sensor(0x5d49, 0x08);
		write_cmos_sensor(0x5d1e, 0x04);
		write_cmos_sensor(0x5d1f, 0x04);
		write_cmos_sensor(0x5d20, 0x04);
		write_cmos_sensor(0x5d27, 0x64);
		write_cmos_sensor(0x5d28, 0xc8);
		write_cmos_sensor(0x5d29, 0x96);
		write_cmos_sensor(0x5d2a, 0xff);
		write_cmos_sensor(0x5d2b, 0xc8);
		write_cmos_sensor(0x5d2c, 0xff);
		write_cmos_sensor(0x5d2d, 0x04);
		write_cmos_sensor(0x40c2, 0x06);
		write_cmos_sensor(0x40c3, 0x06);
		write_cmos_sensor(0x4505, 0x06);
		write_cmos_sensor(0x4506, 0x06);
		write_cmos_sensor(0x5042, 0x06);
		write_cmos_sensor(0x5043, 0x18);
		write_cmos_sensor(0x504c, 0x06);
		write_cmos_sensor(0x504d, 0x18);
		write_cmos_sensor(0x504a, 0x06);
		write_cmos_sensor(0x504b, 0x18);
		write_cmos_sensor(0x3221, 0x0a);
		write_cmos_sensor(0x3d96, 0x0a);
		write_cmos_sensor(0x460a, 0x0a);
		write_cmos_sensor(0x464a, 0x0a);
		write_cmos_sensor(0x5044, 0x0a);
		write_cmos_sensor(0x5046, 0x0a);
		write_cmos_sensor(0x5048, 0x0a);
		write_cmos_sensor(0x504e, 0x0a);
		write_cmos_sensor(0x5050, 0x0a);
		write_cmos_sensor(0x5d15, 0x10);
		write_cmos_sensor(0x5d16, 0x10);
		write_cmos_sensor(0x5d17, 0x10);
		write_cmos_sensor(0x5d18, 0x10);
		write_cmos_sensor(0x5d1a, 0x10);
		write_cmos_sensor(0x5d1b, 0x10);
		write_cmos_sensor(0x5d1c, 0x10);
		write_cmos_sensor(0x5d1d, 0x10);
		write_cmos_sensor(0x0300, 0xfa);
		write_cmos_sensor(0x3002, 0x00);
		write_cmos_sensor(0x381a, 0x00);
		write_cmos_sensor(0x381b, 0x00);
		write_cmos_sensor(0x381c, 0x00);
		write_cmos_sensor(0x381d, 0x00);
		write_cmos_sensor(0x381e, 0x00);


write_cmos_sensor(0x0100, 0x00);

}				/*    MIPI_sensor_Init  */


static void preview_setting(void)
{
	LOG_INF("test123 E\n");
	mDELAY(10);

	write_cmos_sensor(0x0304, 0x5a);
	write_cmos_sensor(0x4837, 0x0e);
	write_cmos_sensor(0x3664, 0x00);
	write_cmos_sensor(0x3709, 0xb2);
	write_cmos_sensor(0x3711, 0x40);
	write_cmos_sensor(0x3808, 0x09);
	write_cmos_sensor(0x3809, 0x00);
	write_cmos_sensor(0x380a, 0x06);
	write_cmos_sensor(0x380b, 0xc0);
	write_cmos_sensor(0x380c, 0x03);
	write_cmos_sensor(0x380d, 0x2a);
	write_cmos_sensor(0x380e, 0x0e);
	write_cmos_sensor(0x380f, 0x78);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x17);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x21);
	write_cmos_sensor(0x3821, 0x24);
	write_cmos_sensor(0x3822, 0x00);
	write_cmos_sensor(0x4016, 0x0f);
	write_cmos_sensor(0x4017, 0x04);
	write_cmos_sensor(0x4018, 0x07);
	write_cmos_sensor(0x4057, 0x10);
	write_cmos_sensor(0x4504, 0x12);
	write_cmos_sensor(0x4510, 0x18);
	write_cmos_sensor(0x480e, 0x02);
	write_cmos_sensor(0x4813, 0x90);
	write_cmos_sensor(0x5000, 0x0b);
	write_cmos_sensor(0x5001, 0x4f);
	write_cmos_sensor(0x5002, 0x15);
	write_cmos_sensor(0x5005, 0x00);
	write_cmos_sensor(0x5820, 0xc5);
	write_cmos_sensor(0x5860, 0x08);
	write_cmos_sensor(0x5ac2, 0x29);
	write_cmos_sensor(0x5acc, 0x00);
	write_cmos_sensor(0x5acd, 0x00);
	write_cmos_sensor(0x5ace, 0x00);
	write_cmos_sensor(0x5acf, 0x10);
	write_cmos_sensor(0x5aff, 0x08);
	write_cmos_sensor(0x5b01, 0x04);
	write_cmos_sensor(0x5b02, 0x09);
	write_cmos_sensor(0x5b03, 0x20);
	write_cmos_sensor(0x5b04, 0x06);
	write_cmos_sensor(0x5b05, 0xe0);
	write_cmos_sensor(0x5b10, 0x00);
	write_cmos_sensor(0x5b11, 0x18);
	write_cmos_sensor(0x5b12, 0x00);
	write_cmos_sensor(0x5b13, 0x04);
	write_cmos_sensor(0x5b14, 0x09);
	write_cmos_sensor(0x5b15, 0x00);
	write_cmos_sensor(0x5b16, 0x06);
	write_cmos_sensor(0x5b17, 0xc0);
	write_cmos_sensor(0x5ac5, 0xde);
	write_cmos_sensor(0x5ac6, 0x01);
	write_cmos_sensor(0x5ac7, 0x28);
	write_cmos_sensor(0x5ae9, 0x13);
	write_cmos_sensor(0x5af2, 0x08);
	write_cmos_sensor(0x5af3, 0x08);
	write_cmos_sensor(0x5af4, 0x00);
	write_cmos_sensor(0x5af5, 0x04);
	write_cmos_sensor(0x5af6, 0x02);
	write_cmos_sensor(0x5af7, 0x06);
	write_cmos_sensor(0x5af8, 0x00);
	write_cmos_sensor(0x5af9, 0x00);
	write_cmos_sensor(0x5afa, 0x04);
	write_cmos_sensor(0x5afb, 0x04);
	write_cmos_sensor(0x5d00, 0x43);
	write_cmos_sensor(0x5d01, 0x00);
	write_cmos_sensor(0x5d08, 0x08);
	write_cmos_sensor(0x5d09, 0x08);
	write_cmos_sensor(0x5d0a, 0x00);
	write_cmos_sensor(0x5d0b, 0x04);
	write_cmos_sensor(0x5d0c, 0x02);
	write_cmos_sensor(0x5d0d, 0x06);
	write_cmos_sensor(0x5d0e, 0x00);
	write_cmos_sensor(0x5d0f, 0x00);
	write_cmos_sensor(0x5d10, 0x04);
	write_cmos_sensor(0x5d11, 0x04);
	write_cmos_sensor(0x5d35, 0x08);
	write_cmos_sensor(0x5d37, 0x04);
	write_cmos_sensor(0x5d38, 0x09);
	write_cmos_sensor(0x5d39, 0x20);
	write_cmos_sensor(0x5d3a, 0x06);
	write_cmos_sensor(0x5d3b, 0xe0);
	write_cmos_sensor(0x5d48, 0x00);
	write_cmos_sensor(0x5d49, 0x10);
	write_cmos_sensor(0x3002, 0x00);
	write_cmos_sensor(0x381a, 0x00);
	write_cmos_sensor(0x381b, 0x00);
	write_cmos_sensor(0x381c, 0x00);
	write_cmos_sensor(0x381d, 0x00);
	write_cmos_sensor(0x381e, 0x00);
}				/*    preview_setting  */


static void capture_setting(kal_uint16 currefps)
{

LOG_INF(" E\n");

write_cmos_sensor(0x480e, 0x02);
write_cmos_sensor(0x0100, 0x00);
write_cmos_sensor(0x0304, 0x7d);
write_cmos_sensor(0x4837, 0x0a);
write_cmos_sensor(0x3622, 0x44);
write_cmos_sensor(0x3664, 0x47);
write_cmos_sensor(0x3706, 0x44);
write_cmos_sensor(0x3709, 0xa0);
write_cmos_sensor(0x370b, 0xa4);
write_cmos_sensor(0x3711, 0x40);
write_cmos_sensor(0x3808, 0x12);
write_cmos_sensor(0x3809, 0x00);
write_cmos_sensor(0x380a, 0x0d);
write_cmos_sensor(0x380b, 0x80);
write_cmos_sensor(0x380c, 0x03);
write_cmos_sensor(0x380d, 0x2a);
write_cmos_sensor(0x380e, 0x0e);
write_cmos_sensor(0x380f, 0x78);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x2f);
write_cmos_sensor(0x3814, 0x11);
write_cmos_sensor(0x3815, 0x11);
write_cmos_sensor(0x3820, 0x00);
write_cmos_sensor(0x3821, 0x04);
write_cmos_sensor(0x3822, 0x00);
write_cmos_sensor(0x4016, 0x1f);
write_cmos_sensor(0x4017, 0x08);
write_cmos_sensor(0x4018, 0x0f);
write_cmos_sensor(0x4057, 0x20);
write_cmos_sensor(0x4504, 0x02);
write_cmos_sensor(0x4510, 0x08);
write_cmos_sensor(0x4813, 0x98);
write_cmos_sensor(0x5000, 0x0b);
write_cmos_sensor(0x5001, 0x4b);
write_cmos_sensor(0x5002, 0x15);
write_cmos_sensor(0x5005, 0x40);
write_cmos_sensor(0x5820, 0xc5);
write_cmos_sensor(0x5860, 0x00);
write_cmos_sensor(0x5ac2, 0x61);
write_cmos_sensor(0x5acc, 0x00);
write_cmos_sensor(0x5acd, 0x00);
write_cmos_sensor(0x5ace, 0x00);
write_cmos_sensor(0x5acf, 0x20);
write_cmos_sensor(0x5aff, 0x10);
write_cmos_sensor(0x5b01, 0x08);
write_cmos_sensor(0x5b02, 0x12);
write_cmos_sensor(0x5b03, 0x40);
write_cmos_sensor(0x5b04, 0x0d);
write_cmos_sensor(0x5b05, 0xc0);
write_cmos_sensor(0x5b10, 0x00);
write_cmos_sensor(0x5b11, 0x30);
write_cmos_sensor(0x5b12, 0x00);
write_cmos_sensor(0x5b13, 0x08);
write_cmos_sensor(0x5b14, 0x12);
write_cmos_sensor(0x5b15, 0x00);
write_cmos_sensor(0x5b16, 0x0d);
write_cmos_sensor(0x5b17, 0x80);
write_cmos_sensor(0x5ac5, 0x6f);
write_cmos_sensor(0x5ac6, 0x00);
write_cmos_sensor(0x5ac7, 0x94);
write_cmos_sensor(0x5ae9, 0x11);
write_cmos_sensor(0x5af2, 0x10);
write_cmos_sensor(0x5af3, 0x10);
write_cmos_sensor(0x5af4, 0x02);
write_cmos_sensor(0x5af5, 0x0a);
write_cmos_sensor(0x5af6, 0x06);
write_cmos_sensor(0x5af7, 0x0e);
write_cmos_sensor(0x5af8, 0x02);
write_cmos_sensor(0x5af9, 0x02);
write_cmos_sensor(0x5afa, 0x0a);
write_cmos_sensor(0x5afb, 0x0a);
write_cmos_sensor(0x5d00, 0xff);
write_cmos_sensor(0x5d01, 0x07);
write_cmos_sensor(0x5d08, 0x10);
write_cmos_sensor(0x5d09, 0x10);
write_cmos_sensor(0x5d0a, 0x02);
write_cmos_sensor(0x5d0b, 0x0a);
write_cmos_sensor(0x5d0c, 0x06);
write_cmos_sensor(0x5d0d, 0x0e);
write_cmos_sensor(0x5d0e, 0x02);
write_cmos_sensor(0x5d0f, 0x02);
write_cmos_sensor(0x5d10, 0x0a);
write_cmos_sensor(0x5d11, 0x0a);
write_cmos_sensor(0x5d35, 0x10);
write_cmos_sensor(0x5d37, 0x08);
write_cmos_sensor(0x5d38, 0x12);
write_cmos_sensor(0x5d39, 0x40);
write_cmos_sensor(0x5d3a, 0x0d);
write_cmos_sensor(0x5d3b, 0xc0);
write_cmos_sensor(0x5d48, 0x00);
write_cmos_sensor(0x5d49, 0x20);
write_cmos_sensor(0x4641, 0x24);
write_cmos_sensor(0x3501, 0x0e);
write_cmos_sensor(0x3502, 0x68);
write_cmos_sensor(0x3508, 0x01);
write_cmos_sensor(0x3509, 0x00);
write_cmos_sensor(0x480e, 0x06);
write_cmos_sensor(0x3002, 0x00);
write_cmos_sensor(0x381a, 0x00);
write_cmos_sensor(0x381b, 0x00);
write_cmos_sensor(0x381c, 0x00);
write_cmos_sensor(0x381d, 0x00);
write_cmos_sensor(0x381e, 0x00);


}				/*    capture_setting  */

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);

	preview_setting();
}				/*    preview_setting  */

static void hs_video_setting(void)
{
	LOG_INF("12345E\n");
	#if 0
	write_cmos_sensor(0x480e, 0x02);
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0304, 0x5a);
	write_cmos_sensor(0x4837, 0x0e);
	write_cmos_sensor(0x3622, 0x44);
	write_cmos_sensor(0x3664, 0x00);
	write_cmos_sensor(0x3706, 0x3a);
	write_cmos_sensor(0x3709, 0x6e);
	write_cmos_sensor(0x370b, 0x9a);
	write_cmos_sensor(0x3711, 0x40);
	write_cmos_sensor(0x3808, 0x05);
	write_cmos_sensor(0x3809, 0x00);
	write_cmos_sensor(0x380a, 0x02);
	write_cmos_sensor(0x380b, 0xd0);
	write_cmos_sensor(0x380c, 0x02);
	write_cmos_sensor(0x380d, 0x76);
	write_cmos_sensor(0x380e, 0x04);
	write_cmos_sensor(0x380f, 0xa6);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0xd7);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x21);
	write_cmos_sensor(0x3821, 0x24);
	write_cmos_sensor(0x3822, 0x00);
	write_cmos_sensor(0x4016, 0x0f);
	write_cmos_sensor(0x4017, 0x04);
	write_cmos_sensor(0x4018, 0x07);
	write_cmos_sensor(0x4057, 0x10);
	write_cmos_sensor(0x4504, 0x12);
	write_cmos_sensor(0x4510, 0x18);
	write_cmos_sensor(0x4813, 0x90);
	write_cmos_sensor(0x5000, 0x0b);
	write_cmos_sensor(0x5001, 0x4f);
	write_cmos_sensor(0x5002, 0x15);
	write_cmos_sensor(0x5005, 0x00);
	write_cmos_sensor(0x5820, 0xc5);
	write_cmos_sensor(0x5860, 0x08);
	write_cmos_sensor(0x5ac2, 0x29);
	write_cmos_sensor(0x5acc, 0x00);
	write_cmos_sensor(0x5acd, 0x00);
	write_cmos_sensor(0x5ace, 0x01);
	write_cmos_sensor(0x5acf, 0x54);
	write_cmos_sensor(0x5aff, 0x08);
	write_cmos_sensor(0x5b01, 0x04);
	write_cmos_sensor(0x5b02, 0x09);
	write_cmos_sensor(0x5b03, 0x20);
	write_cmos_sensor(0x5b04, 0x06);
	write_cmos_sensor(0x5b05, 0xe0);
	write_cmos_sensor(0x5b10, 0x00);
	write_cmos_sensor(0x5b11, 0xd8);
	write_cmos_sensor(0x5b12, 0x00);
	write_cmos_sensor(0x5b13, 0x04);
	write_cmos_sensor(0x5b14, 0x07);
	write_cmos_sensor(0x5b15, 0x80);
	write_cmos_sensor(0x5b16, 0x04);
	write_cmos_sensor(0x5b17, 0x38);
	write_cmos_sensor(0x5ac5, 0xde);
	write_cmos_sensor(0x5ac6, 0x01);
	write_cmos_sensor(0x5ac7, 0x28);
	write_cmos_sensor(0x5ae9, 0x13);
	write_cmos_sensor(0x5af2, 0x08);
	write_cmos_sensor(0x5af3, 0x08);
	write_cmos_sensor(0x5af4, 0x00);
	write_cmos_sensor(0x5af5, 0x04);
	write_cmos_sensor(0x5af6, 0x02);
	write_cmos_sensor(0x5af7, 0x06);
	write_cmos_sensor(0x5af8, 0x00);
	write_cmos_sensor(0x5af9, 0x00);
	write_cmos_sensor(0x5afa, 0x04);
	write_cmos_sensor(0x5afb, 0x04);
	write_cmos_sensor(0x5d00, 0x43);
	write_cmos_sensor(0x5d01, 0x00);
	write_cmos_sensor(0x5d08, 0x08);
	write_cmos_sensor(0x5d09, 0x08);
	write_cmos_sensor(0x5d0a, 0x00);
	write_cmos_sensor(0x5d0b, 0x04);
	write_cmos_sensor(0x5d0c, 0x02);
	write_cmos_sensor(0x5d0d, 0x06);
	write_cmos_sensor(0x5d0e, 0x00);
	write_cmos_sensor(0x5d0f, 0x00);
	write_cmos_sensor(0x5d10, 0x04);
	write_cmos_sensor(0x5d11, 0x04);
	write_cmos_sensor(0x5d35, 0x08);
	write_cmos_sensor(0x5d37, 0x04);
	write_cmos_sensor(0x5d38, 0x09);
	write_cmos_sensor(0x5d39, 0x20);
	write_cmos_sensor(0x5d3a, 0x06);
	write_cmos_sensor(0x5d3b, 0xe0);
	write_cmos_sensor(0x5d48, 0x01);
	write_cmos_sensor(0x5d49, 0x54);
	write_cmos_sensor(0x4641, 0x24);
	write_cmos_sensor(0x3501, 0x04);
	write_cmos_sensor(0x3502, 0x90);
	write_cmos_sensor(0x3508, 0x04);
	write_cmos_sensor(0x3509, 0x00);
	write_cmos_sensor(0x3002, 0x00);
	write_cmos_sensor(0x381a, 0x00);
	write_cmos_sensor(0x381b, 0x00);
	write_cmos_sensor(0x381c, 0x00);
	write_cmos_sensor(0x381d, 0x00);
	write_cmos_sensor(0x381e, 0x00);
	#endif

	write_cmos_sensor(0x0304, 0x5a);
write_cmos_sensor(0x4837, 0x0e);
write_cmos_sensor(0x3622, 0x44);
write_cmos_sensor(0x3664, 0x00);
write_cmos_sensor(0x3706, 0x44);
write_cmos_sensor(0x3709, 0xa0);
write_cmos_sensor(0x370b, 0xa4);
write_cmos_sensor(0x3711, 0x30);
write_cmos_sensor(0x3808, 0x04);
write_cmos_sensor(0x3809, 0x90);
write_cmos_sensor(0x380a, 0x03);
write_cmos_sensor(0x380b, 0x6c);
write_cmos_sensor(0x380c, 0x02);
write_cmos_sensor(0x380d, 0xd0);
write_cmos_sensor(0x380e, 0x04);
write_cmos_sensor(0x380f, 0x12);
write_cmos_sensor(0x3810, 0x00);
write_cmos_sensor(0x3811, 0x03);
write_cmos_sensor(0x3814, 0x31);
write_cmos_sensor(0x3815, 0x35);
write_cmos_sensor(0x3820, 0x01);
write_cmos_sensor(0x3821, 0x04);
write_cmos_sensor(0x3822, 0x01);
write_cmos_sensor(0x4016, 0x07);
write_cmos_sensor(0x4017, 0x02);
write_cmos_sensor(0x4018, 0x03);
write_cmos_sensor(0x4057, 0x10);
write_cmos_sensor(0x4504, 0x12);
write_cmos_sensor(0x4510, 0x18);
write_cmos_sensor(0x4813, 0x90);
write_cmos_sensor(0x5000, 0x21);
write_cmos_sensor(0x5001, 0x46);
write_cmos_sensor(0x5002, 0x11);
write_cmos_sensor(0x5005, 0x00);
write_cmos_sensor(0x5820, 0xc1);
write_cmos_sensor(0x5860, 0x08);
write_cmos_sensor(0x5ac2, 0x21);
write_cmos_sensor(0x5acc, 0x00);
write_cmos_sensor(0x5acd, 0x00);
write_cmos_sensor(0x5ace, 0x00);
write_cmos_sensor(0x5acf, 0x20);
write_cmos_sensor(0x5aff, 0x10);
write_cmos_sensor(0x5b01, 0x08);
write_cmos_sensor(0x5b02, 0x12);
write_cmos_sensor(0x5b03, 0x40);
write_cmos_sensor(0x5b04, 0x0d);
write_cmos_sensor(0x5b05, 0xc0);
write_cmos_sensor(0x5b10, 0x00);
write_cmos_sensor(0x5b11, 0x30);
write_cmos_sensor(0x5b12, 0x00);
write_cmos_sensor(0x5b13, 0x08);
write_cmos_sensor(0x5b14, 0x12);
write_cmos_sensor(0x5b15, 0x00);
write_cmos_sensor(0x5b16, 0x0d);
write_cmos_sensor(0x5b17, 0x80);
write_cmos_sensor(0x5ac5, 0x6f);
write_cmos_sensor(0x5ac6, 0x00);
write_cmos_sensor(0x5ac7, 0x94);
write_cmos_sensor(0x5ae9, 0x11);
write_cmos_sensor(0x5af2, 0x10);
write_cmos_sensor(0x5af3, 0x10);
write_cmos_sensor(0x5af4, 0x02);
write_cmos_sensor(0x5af5, 0x0a);
write_cmos_sensor(0x5af6, 0x06);
write_cmos_sensor(0x5af7, 0x0e);
write_cmos_sensor(0x5af8, 0x02);
write_cmos_sensor(0x5af9, 0x02);
write_cmos_sensor(0x5afa, 0x0a);
write_cmos_sensor(0x5afb, 0x0a);
write_cmos_sensor(0x5d00, 0xff);
write_cmos_sensor(0x5d01, 0x07);
write_cmos_sensor(0x5d08, 0x10);
write_cmos_sensor(0x5d09, 0x10);
write_cmos_sensor(0x5d0a, 0x02);
write_cmos_sensor(0x5d0b, 0x0a);
write_cmos_sensor(0x5d0c, 0x06);
write_cmos_sensor(0x5d0d, 0x0e);
write_cmos_sensor(0x5d0e, 0x02);
write_cmos_sensor(0x5d0f, 0x02);
write_cmos_sensor(0x5d10, 0x0a);
write_cmos_sensor(0x5d11, 0x0a);
write_cmos_sensor(0x5d35, 0x10);
write_cmos_sensor(0x5d37, 0x08);
write_cmos_sensor(0x5d38, 0x12);
write_cmos_sensor(0x5d39, 0x40);
write_cmos_sensor(0x5d3a, 0x0d);
write_cmos_sensor(0x5d3b, 0xc0);
write_cmos_sensor(0x5d48, 0x00);
write_cmos_sensor(0x5d49, 0x20);
write_cmos_sensor(0x3501, 0x04);
write_cmos_sensor(0x3502, 0x00);
write_cmos_sensor(0x3508, 0x01);
write_cmos_sensor(0x3509, 0x00);



}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}


static void custom1_setting(void)
{

	LOG_INF("E\n");

	write_cmos_sensor(0x480e, 0x02);
write_cmos_sensor(0x0100, 0x00);
write_cmos_sensor(0x0304, 0x7d);
write_cmos_sensor(0x4837, 0x0a);
write_cmos_sensor(0x3622, 0x44);
write_cmos_sensor(0x3664, 0x47);
write_cmos_sensor(0x3706, 0x44);
write_cmos_sensor(0x3709, 0xa0);
write_cmos_sensor(0x370b, 0xa4);
write_cmos_sensor(0x3711, 0x40);
write_cmos_sensor(0x3808, 0x10);
write_cmos_sensor(0x3809, 0x00);
write_cmos_sensor(0x380a, 0x0c);
write_cmos_sensor(0x380b, 0x00);
write_cmos_sensor(0x380c, 0x03);
write_cmos_sensor(0x380d, 0x2a);
write_cmos_sensor(0x380e, 0x0e);
write_cmos_sensor(0x380f, 0x78);
write_cmos_sensor(0x3810, 0x01);
write_cmos_sensor(0x3811, 0x2f);
write_cmos_sensor(0x3814, 0x11);
write_cmos_sensor(0x3815, 0x11);
write_cmos_sensor(0x3820, 0x00);
write_cmos_sensor(0x3821, 0x04);
write_cmos_sensor(0x3822, 0x00);
write_cmos_sensor(0x4016, 0x1f);
write_cmos_sensor(0x4017, 0x08);
write_cmos_sensor(0x4018, 0x0f);
write_cmos_sensor(0x4057, 0x20);
write_cmos_sensor(0x4504, 0x02);
write_cmos_sensor(0x4510, 0x08);
write_cmos_sensor(0x4813, 0x98);
write_cmos_sensor(0x5000, 0x0b);
write_cmos_sensor(0x5001, 0x4b);
write_cmos_sensor(0x5002, 0x15);
write_cmos_sensor(0x5005, 0x40);
write_cmos_sensor(0x5820, 0xc5);
write_cmos_sensor(0x5860, 0x00);
write_cmos_sensor(0x5ac2, 0x61);
write_cmos_sensor(0x5acc, 0x00);
write_cmos_sensor(0x5acd, 0x00);
write_cmos_sensor(0x5ace, 0x00);
write_cmos_sensor(0x5acf, 0xe0);
write_cmos_sensor(0x5aff, 0x10);
write_cmos_sensor(0x5b01, 0x08);
write_cmos_sensor(0x5b02, 0x12);
write_cmos_sensor(0x5b03, 0x40);
write_cmos_sensor(0x5b04, 0x0d);
write_cmos_sensor(0x5b05, 0xc0);
write_cmos_sensor(0x5b10, 0x01);
write_cmos_sensor(0x5b11, 0x30);
write_cmos_sensor(0x5b12, 0x00);
write_cmos_sensor(0x5b13, 0x08);
write_cmos_sensor(0x5b14, 0x10);
write_cmos_sensor(0x5b15, 0x00);
write_cmos_sensor(0x5b16, 0x0c);
write_cmos_sensor(0x5b17, 0x00);
write_cmos_sensor(0x5ac5, 0x6f);
write_cmos_sensor(0x5ac6, 0x00);
write_cmos_sensor(0x5ac7, 0x94);
write_cmos_sensor(0x5ae9, 0x11);
write_cmos_sensor(0x5af2, 0x10);
write_cmos_sensor(0x5af3, 0x10);
write_cmos_sensor(0x5af4, 0x02);
write_cmos_sensor(0x5af5, 0x0a);
write_cmos_sensor(0x5af6, 0x06);
write_cmos_sensor(0x5af7, 0x0e);
write_cmos_sensor(0x5af8, 0x02);
write_cmos_sensor(0x5af9, 0x02);
write_cmos_sensor(0x5afa, 0x0a);
write_cmos_sensor(0x5afb, 0x0a);
write_cmos_sensor(0x5d00, 0xff);
write_cmos_sensor(0x5d01, 0x07);
write_cmos_sensor(0x5d08, 0x10);
write_cmos_sensor(0x5d09, 0x10);
write_cmos_sensor(0x5d0a, 0x02);
write_cmos_sensor(0x5d0b, 0x0a);
write_cmos_sensor(0x5d0c, 0x06);
write_cmos_sensor(0x5d0d, 0x0e);
write_cmos_sensor(0x5d0e, 0x02);
write_cmos_sensor(0x5d0f, 0x02);
write_cmos_sensor(0x5d10, 0x0a);
write_cmos_sensor(0x5d11, 0x0a);
write_cmos_sensor(0x5d35, 0x10);
write_cmos_sensor(0x5d37, 0x08);
write_cmos_sensor(0x5d38, 0x12);
write_cmos_sensor(0x5d39, 0x40);
write_cmos_sensor(0x5d3a, 0x0d);
write_cmos_sensor(0x5d3b, 0xc0);
write_cmos_sensor(0x5d48, 0x00);
write_cmos_sensor(0x5d49, 0xe0);
write_cmos_sensor(0x4641, 0x24);
write_cmos_sensor(0x3501, 0x0e);
write_cmos_sensor(0x3502, 0x68);
write_cmos_sensor(0x3508, 0x01);
write_cmos_sensor(0x3509, 0x00);
write_cmos_sensor(0x480e, 0x06);
write_cmos_sensor(0x3002, 0x80);
write_cmos_sensor(0x381a, 0x0e);
write_cmos_sensor(0x381b, 0x70);
write_cmos_sensor(0x381c, 0x01);
write_cmos_sensor(0x381d, 0x95);
write_cmos_sensor(0x381e, 0x02);

}
/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		LOG_INF("Read sensor addr, addr: 0x%x\n", imgsensor.i2c_write_id);
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;
	LOG_2;

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		LOG_INF("Read sensor i2c addr id: 0x%x\n", imgsensor.i2c_write_id);
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}				/*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function */

	return ERROR_NONE;
}				/*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}				/*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/* set_mirror_flip(sensor_config_data->SensorImageMirror); */

	return ERROR_NONE;
}				/*    slim_video     */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();

	return ERROR_NONE;
}   /*  Custom1   */




static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;
	return ERROR_NONE;
}				/*    get_resolution    */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/* sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10;  not use */
	/* sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10;  not use */
	/* imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;  not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;	/* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		    imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}				/*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{				/*  */
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
		    imgsensor_info.normal_video.pclk / framerate * 10 /
		    imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.normal_video.framelength) ? (frame_length -
								 imgsensor_info.normal_video.
								 framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length =
		    imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.cap.framelength) ? (frame_length -
							imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
		    imgsensor_info.hs_video.pclk / framerate * 10 /
		    imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.hs_video.framelength) ? (frame_length -
							     imgsensor_info.hs_video.
							     framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
		    imgsensor_info.slim_video.pclk / framerate * 10 /
		    imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.slim_video.framelength) ? (frame_length -
							       imgsensor_info.slim_video.
							       framelength) : 0;
		imgsensor.frame_length =
		    imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length =
		    imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.custom1.framelength) ? (frame_length -
							imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frame_length =
		    imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		    (frame_length >
		     imgsensor_info.pre.framelength) ? (frame_length -
							imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id,
						    MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	/* 0x503D[8]: 1 enable,  0 disable */
	/* 0x503D[1:0]; 00 Color bar, 01 Random Data, 10 Square */
	if (enable)
		write_cmos_sensor(0x503D, 0x80);
	else
		write_cmos_sensor(0x503D, 0x00);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0100, 0x01);
	else
		write_cmos_sensor(0x0100, 0x00);

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	SENSOR_VC_INFO_STRUCT *pvcinfo;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SET_PD_BLOCK_INFO_T *PDAFinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *(feature_data),
						  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL) * feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		/* For pointer address in user space is 32-bit, need use compat_ptr to */
		/* convert it to 64-bit in kernel space */
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
			       sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", *feature_data_32,
			*(feature_data_32 + 1), *(feature_data_32 + 2));
		ihdr_write_shutter_gain(*feature_data_32, *(feature_data_32 + 1),
					*(feature_data_32 + 2));
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		{
			kal_uint32 rate;

			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				rate = imgsensor_info.cap.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				rate = imgsensor_info.normal_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				rate = imgsensor_info.hs_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				rate = imgsensor_info.slim_video.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				rate = imgsensor_info.custom1.mipi_pixel_rate;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				rate = imgsensor_info.pre.mipi_pixel_rate;
				break;
			}
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = rate;
		}
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("mtk_test SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
			       sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("mtk_test SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/* PDAF capacity enable or not, ov13855 only full size support PDAF */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;	/* video & capture use same setting */
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
			       sizeof(SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


UINT32 OV16B10_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    OV16B10MIPISensorInit    */
