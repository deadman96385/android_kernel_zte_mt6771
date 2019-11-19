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
 *     HI556mipi_Sensor.c
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
#include "hi556_mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "HI556_camera_sensor"
#define LOG_1 LOG_INF("HI556,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,420Mbps/lane; capture 5M@15fps,420Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)	pr_err(PFX "[ %s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI556_SENSOR_ID,

	.checksum_value = 0xf7375923,	/*checksum value for Camera Auto Test */

	.pre = {
		.pclk = 173100000,	/* record different mode's pclk */
		.linelength = 2816,	/* record different mode's linelength */
		.framelength = 2049,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 1296,	/* record different mode's width of grabwindow */
		.grabwindow_height = 972,	/* record different mode's height of grabwindow */
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		/*     following for GetDefaultFramerateByScenario()    */
		.mipi_pixel_rate = 88000000,
		.max_framerate = 300,
		},
	.cap = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 176000000,
		.max_framerate = 300,
		},
	.normal_video = {
		.pclk = 160000000,	/* record different mode's pclk */
		.linelength = 2816,	/* record different mode's linelength */
		.framelength = 2083,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 1296,	/* record different mode's width of grabwindow */
		.grabwindow_height = 972,	/* record different mode's height of grabwindow */
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		/*     following for GetDefaultFramerateByScenario()    */
		.mipi_pixel_rate = 88000000,
		.max_framerate = 300,
		},
	.hs_video = {
		.pclk = 160000000,	/* record different mode's pclk */
		.linelength = 2816,	/* record different mode's linelength */
		.framelength = 2083,	/* record different mode's framelength */
		.startx = 0,	/* record different mode's startx of grabwindow */
		.starty = 0,	/* record different mode's starty of grabwindow */
		.grabwindow_width = 1296,	/* record different mode's width of grabwindow */
		.grabwindow_height = 972,	/* record different mode's height of grabwindow */
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		/*     following for GetDefaultFramerateByScenario()    */
		.mipi_pixel_rate = 88000000,
		.max_framerate = 300,
		},
	.slim_video = {
		.pclk = 84000000,
		.linelength = 3728,
		.framelength = 748,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 84000000,
		.max_framerate = 300,
		},
	.custom1 = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,	/* unit , ns */
		.mipi_pixel_rate = 176000000,
		.max_framerate = 300,
		},
	.margin = 4,		/* sensor framelength & shutter margin */
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
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,	/* sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,	/* sensor output first pixel color */
	.mclk = 24,		/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_2_LANE,	/* mipi lane num */
	.i2c_addr_table = {0x40, 0xff},
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
	.i2c_write_id = 0x40,	/* record current sensor's i2c write id */
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972},	/* Preview */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944},	/* capture */
	{2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972},	/* video */
	{2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972},	/* hight speed video */
	{2592, 1944, 0, 0, 2592, 1944, 1296, 972, 0, 0, 1296, 972, 0, 0, 1296, 972},	/* slim video */
	{2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0, 2592, 1944},	/* custom1 */
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}



static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF(" dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, */
	/* or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0006, imgsensor.frame_length);
	write_cmos_sensor(0x0074, imgsensor.line_length & 0x00FFFF);

}				/*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0F16) << 8) | read_cmos_sensor(0x0F17));
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF(" framerate = %d, min_framelength_en = %d\n", framerate, min_framelength_en);

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

	LOG_INF(" write shutter");
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
		imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0006, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);
	LOG_INF("Exit! shutter =0x%x, framelength =0x%x\n", shutter, imgsensor.frame_length);
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
			write_cmos_sensor(0x0006, imgsensor.frame_length);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0006, imgsensor.frame_length);
	}
	write_cmos_sensor(0x0074, shutter & 0x00FFFF);

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
	reg_gain = (((gain/64-1) << 4)&0xF0)|(((gain%64)/4)&0x0F);

	write_cmos_sensor_8(0x0077, reg_gain);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

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

	/* @@ global setting */
write_cmos_sensor(0x0a00, 0x0000);

mDELAY(10);

write_cmos_sensor(0x0e00, 0x0102);
write_cmos_sensor(0x0e02, 0x0102);
write_cmos_sensor(0x0e0c, 0x0100);
write_cmos_sensor(0x2000, 0x4031);
write_cmos_sensor(0x2002, 0x8400);
write_cmos_sensor(0x2004, 0x12b0);
write_cmos_sensor(0x2006, 0xe292);
write_cmos_sensor(0x2008, 0x12b0);
write_cmos_sensor(0x200a, 0xe2b0);
write_cmos_sensor(0x200c, 0x12b0);
write_cmos_sensor(0x200e, 0xfce4);
write_cmos_sensor(0x2010, 0x40b2);
write_cmos_sensor(0x2012, 0x0250);
write_cmos_sensor(0x2014, 0x8070);
write_cmos_sensor(0x2016, 0x40b2);
write_cmos_sensor(0x2018, 0x01b8);
write_cmos_sensor(0x201a, 0x8072);
write_cmos_sensor(0x201c, 0x93d2);
write_cmos_sensor(0x201e, 0x00bd);
write_cmos_sensor(0x2020, 0x248b);
write_cmos_sensor(0x2022, 0x0900);
write_cmos_sensor(0x2024, 0x7312);
write_cmos_sensor(0x2026, 0x43d2);
write_cmos_sensor(0x2028, 0x00bd);
write_cmos_sensor(0x202a, 0x12b0);
write_cmos_sensor(0x202c, 0xe60e);
write_cmos_sensor(0x202e, 0x12b0);
write_cmos_sensor(0x2030, 0xe64c);
write_cmos_sensor(0x2032, 0x12b0);
write_cmos_sensor(0x2034, 0xeb28);
write_cmos_sensor(0x2036, 0x12b0);
write_cmos_sensor(0x2038, 0xe662);
write_cmos_sensor(0x203a, 0x12b0);
write_cmos_sensor(0x203c, 0xe878);
write_cmos_sensor(0x203e, 0x12b0);
write_cmos_sensor(0x2040, 0xfc3a);
write_cmos_sensor(0x2042, 0x12b0);
write_cmos_sensor(0x2044, 0xfc62);
write_cmos_sensor(0x2046, 0x12b0);
write_cmos_sensor(0x2048, 0xfc7a);
write_cmos_sensor(0x204a, 0x12b0);
write_cmos_sensor(0x204c, 0xe2ca);
write_cmos_sensor(0x204e, 0x12b0);
write_cmos_sensor(0x2050, 0xe67c);
write_cmos_sensor(0x2052, 0x12b0);
write_cmos_sensor(0x2054, 0xe7dc);
write_cmos_sensor(0x2056, 0xb3a2);
write_cmos_sensor(0x2058, 0x0a84);
write_cmos_sensor(0x205a, 0x2402);
write_cmos_sensor(0x205c, 0x12b0);
write_cmos_sensor(0x205e, 0xe9c8);
write_cmos_sensor(0x2060, 0x4382);
write_cmos_sensor(0x2062, 0x7322);
write_cmos_sensor(0x2064, 0x4392);
write_cmos_sensor(0x2066, 0x7326);
write_cmos_sensor(0x2068, 0x12b0);
write_cmos_sensor(0x206a, 0xf946);
write_cmos_sensor(0x206c, 0x12b0);
write_cmos_sensor(0x206e, 0xe438);
write_cmos_sensor(0x2070, 0x12b0);
write_cmos_sensor(0x2072, 0xe4a0);
write_cmos_sensor(0x2074, 0x12b0);
write_cmos_sensor(0x2076, 0xe4ee);
write_cmos_sensor(0x2078, 0x12b0);
write_cmos_sensor(0x207a, 0xe534);
write_cmos_sensor(0x207c, 0x12b0);
write_cmos_sensor(0x207e, 0xe586);
write_cmos_sensor(0x2080, 0x12b0);
write_cmos_sensor(0x2082, 0xe6ca);
write_cmos_sensor(0x2084, 0x12b0);
write_cmos_sensor(0x2086, 0xe6f0);
write_cmos_sensor(0x2088, 0x12b0);
write_cmos_sensor(0x208a, 0xe722);
write_cmos_sensor(0x208c, 0x4392);
write_cmos_sensor(0x208e, 0x731c);
write_cmos_sensor(0x2090, 0x12b0);
write_cmos_sensor(0x2092, 0xea52);
write_cmos_sensor(0x2094, 0x12b0);
write_cmos_sensor(0x2096, 0xfc8e);
write_cmos_sensor(0x2098, 0x12b0);
write_cmos_sensor(0x209a, 0xe5a2);
write_cmos_sensor(0x209c, 0x93c2);
write_cmos_sensor(0x209e, 0x8294);
write_cmos_sensor(0x20a0, 0x2009);
write_cmos_sensor(0x20a2, 0x0b00);
write_cmos_sensor(0x20a4, 0x7302);
write_cmos_sensor(0x20a6, 0x0258);
write_cmos_sensor(0x20a8, 0x4382);
write_cmos_sensor(0x20aa, 0x7902);
write_cmos_sensor(0x20ac, 0x0900);
write_cmos_sensor(0x20ae, 0x7308);
write_cmos_sensor(0x20b0, 0x12b0);
write_cmos_sensor(0x20b2, 0xf946);
write_cmos_sensor(0x20b4, 0x12b0);
write_cmos_sensor(0x20b6, 0xe342);
write_cmos_sensor(0x20b8, 0x12b0);
write_cmos_sensor(0x20ba, 0xe370);
write_cmos_sensor(0x20bc, 0x12b0);
write_cmos_sensor(0x20be, 0xfd10);
write_cmos_sensor(0x20c0, 0x4292);
write_cmos_sensor(0x20c2, 0x8292);
write_cmos_sensor(0x20c4, 0x7114);
write_cmos_sensor(0x20c6, 0x421f);
write_cmos_sensor(0x20c8, 0x7316);
write_cmos_sensor(0x20ca, 0xc312);
write_cmos_sensor(0x20cc, 0x100f);
write_cmos_sensor(0x20ce, 0x821f);
write_cmos_sensor(0x20d0, 0x8298);
write_cmos_sensor(0x20d2, 0x4f82);
write_cmos_sensor(0x20d4, 0x7334);
write_cmos_sensor(0x20d6, 0x0f00);
write_cmos_sensor(0x20d8, 0x7302);
write_cmos_sensor(0x20da, 0x12b0);
write_cmos_sensor(0x20dc, 0xe6b8);
write_cmos_sensor(0x20de, 0x43b2);
write_cmos_sensor(0x20e0, 0x7a06);
write_cmos_sensor(0x20e2, 0x12b0);
write_cmos_sensor(0x20e4, 0xfcb0);
write_cmos_sensor(0x20e6, 0x9392);
write_cmos_sensor(0x20e8, 0x7114);
write_cmos_sensor(0x20ea, 0x2011);
write_cmos_sensor(0x20ec, 0x0b00);
write_cmos_sensor(0x20ee, 0x7302);
write_cmos_sensor(0x20f0, 0x0258);
write_cmos_sensor(0x20f2, 0x4382);
write_cmos_sensor(0x20f4, 0x7902);
write_cmos_sensor(0x20f6, 0x0800);
write_cmos_sensor(0x20f8, 0x7118);
write_cmos_sensor(0x20fa, 0x12b0);
write_cmos_sensor(0x20fc, 0xeb4c);
write_cmos_sensor(0x20fe, 0x0900);
write_cmos_sensor(0x2100, 0x7112);
write_cmos_sensor(0x2102, 0x12b0);
write_cmos_sensor(0x2104, 0xe3e6);
write_cmos_sensor(0x2106, 0x0b00);
write_cmos_sensor(0x2108, 0x7302);
write_cmos_sensor(0x210a, 0x0036);
write_cmos_sensor(0x210c, 0x3fec);
write_cmos_sensor(0x210e, 0x0b00);
write_cmos_sensor(0x2110, 0x7302);
write_cmos_sensor(0x2112, 0x0036);
write_cmos_sensor(0x2114, 0x4392);
write_cmos_sensor(0x2116, 0x7902);
write_cmos_sensor(0x2118, 0x4292);
write_cmos_sensor(0x211a, 0x7100);
write_cmos_sensor(0x211c, 0x82be);
write_cmos_sensor(0x211e, 0x12b0);
write_cmos_sensor(0x2120, 0xe74a);
write_cmos_sensor(0x2122, 0x12b0);
write_cmos_sensor(0x2124, 0xea8e);
write_cmos_sensor(0x2126, 0x12b0);
write_cmos_sensor(0x2128, 0xead4);
write_cmos_sensor(0x212a, 0x12b0);
write_cmos_sensor(0x212c, 0xe3e6);
write_cmos_sensor(0x212e, 0x930f);
write_cmos_sensor(0x2130, 0x27da);
write_cmos_sensor(0x2132, 0x12b0);
write_cmos_sensor(0x2134, 0xfcc8);
write_cmos_sensor(0x2136, 0x3fb0);
write_cmos_sensor(0x2138, 0x12b0);
write_cmos_sensor(0x213a, 0xe5be);
write_cmos_sensor(0x213c, 0x12b0);
write_cmos_sensor(0x213e, 0xe5e6);
write_cmos_sensor(0x2140, 0x3f70);
write_cmos_sensor(0x2142, 0x4030);
write_cmos_sensor(0x2144, 0xf770);
write_cmos_sensor(0x2146, 0x12b0);
write_cmos_sensor(0x2148, 0xee16);
write_cmos_sensor(0x214a, 0x12b0);
write_cmos_sensor(0x214c, 0xee5c);
write_cmos_sensor(0x214e, 0x12b0);
write_cmos_sensor(0x2150, 0xeeac);
write_cmos_sensor(0x2152, 0x12b0);
write_cmos_sensor(0x2154, 0xeeea);
write_cmos_sensor(0x2156, 0x12b0);
write_cmos_sensor(0x2158, 0xef06);
write_cmos_sensor(0x215a, 0x12b0);
write_cmos_sensor(0x215c, 0xef28);
write_cmos_sensor(0x215e, 0x12b0);
write_cmos_sensor(0x2160, 0xef98);
write_cmos_sensor(0x2162, 0x12b0);
write_cmos_sensor(0x2164, 0xfb2a);
write_cmos_sensor(0x2166, 0x12b0);
write_cmos_sensor(0x2168, 0xfa60);
write_cmos_sensor(0x216a, 0x12b0);
write_cmos_sensor(0x216c, 0xfb4c);
write_cmos_sensor(0x216e, 0x12b0);
write_cmos_sensor(0x2170, 0xf294);
write_cmos_sensor(0x2172, 0xd392);
write_cmos_sensor(0x2174, 0x7102);
write_cmos_sensor(0x2176, 0x4130);
write_cmos_sensor(0x2178, 0x120b);
write_cmos_sensor(0x217a, 0x120a);
write_cmos_sensor(0x217c, 0x1209);
write_cmos_sensor(0x217e, 0x1208);
write_cmos_sensor(0x2180, 0x1207);
write_cmos_sensor(0x2182, 0x4f0c);
write_cmos_sensor(0x2184, 0x4e08);
write_cmos_sensor(0x2186, 0x4d47);
write_cmos_sensor(0x2188, 0x425f);
write_cmos_sensor(0x218a, 0x00de);
write_cmos_sensor(0x218c, 0x4f4e);
write_cmos_sensor(0x218e, 0x430f);
write_cmos_sensor(0x2190, 0x421a);
write_cmos_sensor(0x2192, 0x8308);
write_cmos_sensor(0x2194, 0x421b);
write_cmos_sensor(0x2196, 0x830a);
write_cmos_sensor(0x2198, 0x8e0a);
write_cmos_sensor(0x219a, 0x7f0b);
write_cmos_sensor(0x219c, 0x4a0e);
write_cmos_sensor(0x219e, 0x4b0f);
write_cmos_sensor(0x21a0, 0x4c3a);
write_cmos_sensor(0x21a2, 0x4c3b);
write_cmos_sensor(0x21a4, 0x4a0c);
write_cmos_sensor(0x21a6, 0x4b0d);
write_cmos_sensor(0x21a8, 0x8e0c);
write_cmos_sensor(0x21aa, 0x7f0d);
write_cmos_sensor(0x21ac, 0x284a);
write_cmos_sensor(0x21ae, 0x425f);
write_cmos_sensor(0x21b0, 0x00de);
write_cmos_sensor(0x21b2, 0x4f4a);
write_cmos_sensor(0x21b4, 0x430b);
write_cmos_sensor(0x21b6, 0xb0f2);
write_cmos_sensor(0x21b8, 0x0010);
write_cmos_sensor(0x21ba, 0x00dd);
write_cmos_sensor(0x21bc, 0x2040);
write_cmos_sensor(0x21be, 0x4349);
write_cmos_sensor(0x21c0, 0x421e);
write_cmos_sensor(0x21c2, 0x7560);
write_cmos_sensor(0x21c4, 0x421f);
write_cmos_sensor(0x21c6, 0x7578);
write_cmos_sensor(0x21c8, 0x12b0);
write_cmos_sensor(0x21ca, 0xefc4);
write_cmos_sensor(0x21cc, 0x4a0c);
write_cmos_sensor(0x21ce, 0x4b0d);
write_cmos_sensor(0x21d0, 0x8e0c);
write_cmos_sensor(0x21d2, 0x7f0d);
write_cmos_sensor(0x21d4, 0x2c01);
write_cmos_sensor(0x21d6, 0x4359);
write_cmos_sensor(0x21d8, 0x434f);
write_cmos_sensor(0x21da, 0x93c2);
write_cmos_sensor(0x21dc, 0x8294);
write_cmos_sensor(0x21de, 0x2001);
write_cmos_sensor(0x21e0, 0x435f);
write_cmos_sensor(0x21e2, 0xf94f);
write_cmos_sensor(0x21e4, 0x434e);
write_cmos_sensor(0x21e6, 0x93c8);
write_cmos_sensor(0x21e8, 0x0000);
write_cmos_sensor(0x21ea, 0x2001);
write_cmos_sensor(0x21ec, 0x435e);
write_cmos_sensor(0x21ee, 0xfe4f);
write_cmos_sensor(0x21f0, 0x4fc8);
write_cmos_sensor(0x21f2, 0x0000);
write_cmos_sensor(0x21f4, 0x9347);
write_cmos_sensor(0x21f6, 0x2013);
write_cmos_sensor(0x21f8, 0x4a82);
write_cmos_sensor(0x21fa, 0x7540);
write_cmos_sensor(0x21fc, 0x4b82);
write_cmos_sensor(0x21fe, 0x7574);
write_cmos_sensor(0x2200, 0x934f);
write_cmos_sensor(0x2202, 0x2428);
write_cmos_sensor(0x2204, 0x93c2);
write_cmos_sensor(0x2206, 0x00cd);
write_cmos_sensor(0x2208, 0x2407);
write_cmos_sensor(0x220a, 0x425f);
write_cmos_sensor(0x220c, 0x806d);
write_cmos_sensor(0x220e, 0x5f82);
write_cmos_sensor(0x2210, 0x7542);
write_cmos_sensor(0x2212, 0x5f82);
write_cmos_sensor(0x2214, 0x7544);
write_cmos_sensor(0x2216, 0x3c1e);
write_cmos_sensor(0x2218, 0x425f);
write_cmos_sensor(0x221a, 0x806c);
write_cmos_sensor(0x221c, 0x3ff8);
write_cmos_sensor(0x221e, 0x4a82);
write_cmos_sensor(0x2220, 0x754a);
write_cmos_sensor(0x2222, 0x4b82);
write_cmos_sensor(0x2224, 0x7576);
write_cmos_sensor(0x2226, 0x934f);
write_cmos_sensor(0x2228, 0x2415);
write_cmos_sensor(0x222a, 0x93c2);
write_cmos_sensor(0x222c, 0x00cd);
write_cmos_sensor(0x222e, 0x2412);
write_cmos_sensor(0x2230, 0x425f);
write_cmos_sensor(0x2232, 0x806d);
write_cmos_sensor(0x2234, 0x5f82);
write_cmos_sensor(0x2236, 0x754c);
write_cmos_sensor(0x2238, 0x5f82);
write_cmos_sensor(0x223a, 0x754e);
write_cmos_sensor(0x223c, 0x3c0b);
write_cmos_sensor(0x223e, 0x435f);
write_cmos_sensor(0x2240, 0x3fd7);
write_cmos_sensor(0x2242, 0x421e);
write_cmos_sensor(0x2244, 0x8308);
write_cmos_sensor(0x2246, 0x421f);
write_cmos_sensor(0x2248, 0x830a);
write_cmos_sensor(0x224a, 0x8a0e);
write_cmos_sensor(0x224c, 0x7b0f);
write_cmos_sensor(0x224e, 0x4e0a);
write_cmos_sensor(0x2250, 0x4f0b);
write_cmos_sensor(0x2252, 0x3fb1);
write_cmos_sensor(0x2254, 0x4137);
write_cmos_sensor(0x2256, 0x4138);
write_cmos_sensor(0x2258, 0x4139);
write_cmos_sensor(0x225a, 0x413a);
write_cmos_sensor(0x225c, 0x413b);
write_cmos_sensor(0x225e, 0x4130);
write_cmos_sensor(0x2260, 0x425f);
write_cmos_sensor(0x2262, 0x00d8);
write_cmos_sensor(0x2264, 0xf37f);
write_cmos_sensor(0x2266, 0x421e);
write_cmos_sensor(0x2268, 0x0086);
write_cmos_sensor(0x226a, 0x12b0);
write_cmos_sensor(0x226c, 0xefc4);
write_cmos_sensor(0x226e, 0x4e82);
write_cmos_sensor(0x2270, 0x8308);
write_cmos_sensor(0x2272, 0x4f82);
write_cmos_sensor(0x2274, 0x830a);
write_cmos_sensor(0x2276, 0x4292);
write_cmos_sensor(0x2278, 0x0088);
write_cmos_sensor(0x227a, 0x7316);
write_cmos_sensor(0x227c, 0x425f);
write_cmos_sensor(0x227e, 0x806f);
write_cmos_sensor(0x2280, 0x4f82);
write_cmos_sensor(0x2282, 0x7542);
write_cmos_sensor(0x2284, 0x93c2);
write_cmos_sensor(0x2286, 0x00cc);
write_cmos_sensor(0x2288, 0x244d);
write_cmos_sensor(0x228a, 0x521f);
write_cmos_sensor(0x228c, 0x828c);
write_cmos_sensor(0x228e, 0x4f82);
write_cmos_sensor(0x2290, 0x7544);
write_cmos_sensor(0x2292, 0x93c2);
write_cmos_sensor(0x2294, 0x00cd);
write_cmos_sensor(0x2296, 0x240d);
write_cmos_sensor(0x2298, 0x425f);
write_cmos_sensor(0x229a, 0x806f);
write_cmos_sensor(0x229c, 0x425e);
write_cmos_sensor(0x229e, 0x806e);
write_cmos_sensor(0x22a0, 0x4f0d);
write_cmos_sensor(0x22a2, 0x5e0d);
write_cmos_sensor(0x22a4, 0x4d82);
write_cmos_sensor(0x22a6, 0x754c);
write_cmos_sensor(0x22a8, 0x521f);
write_cmos_sensor(0x22aa, 0x828e);
write_cmos_sensor(0x22ac, 0x5e0f);
write_cmos_sensor(0x22ae, 0x4f82);
write_cmos_sensor(0x22b0, 0x754e);
write_cmos_sensor(0x22b2, 0x425f);
write_cmos_sensor(0x22b4, 0x00f3);
write_cmos_sensor(0x22b6, 0xf37f);
write_cmos_sensor(0x22b8, 0x403d);
write_cmos_sensor(0x22ba, 0x82ee);
write_cmos_sensor(0x22bc, 0x421e);
write_cmos_sensor(0x22be, 0x00f4);
write_cmos_sensor(0x22c0, 0x12b0);
write_cmos_sensor(0x22c2, 0xefd0);
write_cmos_sensor(0x22c4, 0x93c2);
write_cmos_sensor(0x22c6, 0x00cd);
write_cmos_sensor(0x22c8, 0x2023);
write_cmos_sensor(0x22ca, 0x434d);
write_cmos_sensor(0x22cc, 0x403e);
write_cmos_sensor(0x22ce, 0x8306);
write_cmos_sensor(0x22d0, 0x403f);
write_cmos_sensor(0x22d2, 0x82ee);
write_cmos_sensor(0x22d4, 0x12b0);
write_cmos_sensor(0x22d6, 0xf978);
write_cmos_sensor(0x22d8, 0x93c2);
write_cmos_sensor(0x22da, 0x00cd);
write_cmos_sensor(0x22dc, 0x2011);
write_cmos_sensor(0x22de, 0x4382);
write_cmos_sensor(0x22e0, 0x733e);
write_cmos_sensor(0x22e2, 0x93c2);
write_cmos_sensor(0x22e4, 0x00cd);
write_cmos_sensor(0x22e6, 0x240b);
write_cmos_sensor(0x22e8, 0x421e);
write_cmos_sensor(0x22ea, 0x82ee);
write_cmos_sensor(0x22ec, 0x421f);
write_cmos_sensor(0x22ee, 0x82f0);
write_cmos_sensor(0x22f0, 0x821e);
write_cmos_sensor(0x22f2, 0x82f2);
write_cmos_sensor(0x22f4, 0x721f);
write_cmos_sensor(0x22f6, 0x82f4);
write_cmos_sensor(0x22f8, 0x2c02);
write_cmos_sensor(0x22fa, 0x4392);
write_cmos_sensor(0x22fc, 0x733e);
write_cmos_sensor(0x22fe, 0x4130);
write_cmos_sensor(0x2300, 0x435d);
write_cmos_sensor(0x2302, 0x403e);
write_cmos_sensor(0x2304, 0x8307);
write_cmos_sensor(0x2306, 0x403f);
write_cmos_sensor(0x2308, 0x82f2);
write_cmos_sensor(0x230a, 0x12b0);
write_cmos_sensor(0x230c, 0xf978);
write_cmos_sensor(0x230e, 0x3fe7);
write_cmos_sensor(0x2310, 0x425f);
write_cmos_sensor(0x2312, 0x00f2);
write_cmos_sensor(0x2314, 0xf37f);
write_cmos_sensor(0x2316, 0x403d);
write_cmos_sensor(0x2318, 0x82f2);
write_cmos_sensor(0x231a, 0x421e);
write_cmos_sensor(0x231c, 0x00f0);
write_cmos_sensor(0x231e, 0x12b0);
write_cmos_sensor(0x2320, 0xefd0);
write_cmos_sensor(0x2322, 0x3fd3);
write_cmos_sensor(0x2324, 0x521f);
write_cmos_sensor(0x2326, 0x828e);
write_cmos_sensor(0x2328, 0x3fb2);
write_cmos_sensor(0x232a, 0x421e);
write_cmos_sensor(0x232c, 0x7540);
write_cmos_sensor(0x232e, 0x421f);
write_cmos_sensor(0x2330, 0x7574);
write_cmos_sensor(0x2332, 0x12b0);
write_cmos_sensor(0x2334, 0xefc4);
write_cmos_sensor(0x2336, 0x4e82);
write_cmos_sensor(0x2338, 0x8346);
write_cmos_sensor(0x233a, 0x4f82);
write_cmos_sensor(0x233c, 0x8348);
write_cmos_sensor(0x233e, 0x93c2);
write_cmos_sensor(0x2340, 0x00c1);
write_cmos_sensor(0x2342, 0x2403);
write_cmos_sensor(0x2344, 0x42d2);
write_cmos_sensor(0x2346, 0x834b);
write_cmos_sensor(0x2348, 0x8310);
write_cmos_sensor(0x234a, 0x4130);
write_cmos_sensor(0x234c, 0x4292);
write_cmos_sensor(0x234e, 0x7320);
write_cmos_sensor(0x2350, 0x8344);
write_cmos_sensor(0x2352, 0x93c2);
write_cmos_sensor(0x2354, 0x00c1);
write_cmos_sensor(0x2356, 0x2001);
write_cmos_sensor(0x2358, 0x4130);
write_cmos_sensor(0x235a, 0x12b0);
write_cmos_sensor(0x235c, 0xfb64);
write_cmos_sensor(0x235e, 0x12b0);
write_cmos_sensor(0x2360, 0xfbe8);
write_cmos_sensor(0x2362, 0x3ffa);
write_cmos_sensor(0x2364, 0x120b);
write_cmos_sensor(0x2366, 0x120a);
write_cmos_sensor(0x2368, 0x1209);
write_cmos_sensor(0x236a, 0x1208);
write_cmos_sensor(0x236c, 0x421e);
write_cmos_sensor(0x236e, 0x7314);
write_cmos_sensor(0x2370, 0x421f);
write_cmos_sensor(0x2372, 0x7336);
write_cmos_sensor(0x2374, 0x12b0);
write_cmos_sensor(0x2376, 0xefc4);
write_cmos_sensor(0x2378, 0x4e0c);
write_cmos_sensor(0x237a, 0x4f0d);
write_cmos_sensor(0x237c, 0x4e82);
write_cmos_sensor(0x237e, 0x8338);
write_cmos_sensor(0x2380, 0x4f82);
write_cmos_sensor(0x2382, 0x833a);
write_cmos_sensor(0x2384, 0x421f);
write_cmos_sensor(0x2386, 0x0196);
write_cmos_sensor(0x2388, 0x4f0e);
write_cmos_sensor(0x238a, 0x430f);
write_cmos_sensor(0x238c, 0x4c0a);
write_cmos_sensor(0x238e, 0x4d0b);
write_cmos_sensor(0x2390, 0x8e0a);
write_cmos_sensor(0x2392, 0x7f0b);
write_cmos_sensor(0x2394, 0x4a0e);
write_cmos_sensor(0x2396, 0x4b0f);
write_cmos_sensor(0x2398, 0x803e);
write_cmos_sensor(0x239a, 0x0102);
write_cmos_sensor(0x239c, 0x730f);
write_cmos_sensor(0x239e, 0x281b);
write_cmos_sensor(0x23a0, 0x4039);
write_cmos_sensor(0x23a2, 0x0101);
write_cmos_sensor(0x23a4, 0x8329);
write_cmos_sensor(0x23a6, 0x4982);
write_cmos_sensor(0x23a8, 0x7320);
write_cmos_sensor(0x23aa, 0x421e);
write_cmos_sensor(0x23ac, 0x7540);
write_cmos_sensor(0x23ae, 0x421f);
write_cmos_sensor(0x23b0, 0x7574);
write_cmos_sensor(0x23b2, 0x12b0);
write_cmos_sensor(0x23b4, 0xefc4);
write_cmos_sensor(0x23b6, 0x4e82);
write_cmos_sensor(0x23b8, 0x8332);
write_cmos_sensor(0x23ba, 0x4f82);
write_cmos_sensor(0x23bc, 0x8334);
write_cmos_sensor(0x23be, 0x4348);
write_cmos_sensor(0x23c0, 0x5039);
write_cmos_sensor(0x23c2, 0xfffa);
write_cmos_sensor(0x23c4, 0x490c);
write_cmos_sensor(0x23c6, 0x430d);
write_cmos_sensor(0x23c8, 0x5e0c);
write_cmos_sensor(0x23ca, 0x6f0d);
write_cmos_sensor(0x23cc, 0x8a0c);
write_cmos_sensor(0x23ce, 0x7b0d);
write_cmos_sensor(0x23d0, 0x2804);
write_cmos_sensor(0x23d2, 0x4358);
write_cmos_sensor(0x23d4, 0x3c02);
write_cmos_sensor(0x23d6, 0x4a09);
write_cmos_sensor(0x23d8, 0x3fe5);
write_cmos_sensor(0x23da, 0x48c2);
write_cmos_sensor(0x23dc, 0x834b);
write_cmos_sensor(0x23de, 0x4138);
write_cmos_sensor(0x23e0, 0x4139);
write_cmos_sensor(0x23e2, 0x413a);
write_cmos_sensor(0x23e4, 0x413b);
write_cmos_sensor(0x23e6, 0x4130);
write_cmos_sensor(0x23e8, 0x120b);
write_cmos_sensor(0x23ea, 0x120a);
write_cmos_sensor(0x23ec, 0x93c2);
write_cmos_sensor(0x23ee, 0x8294);
write_cmos_sensor(0x23f0, 0x2021);
write_cmos_sensor(0x23f2, 0x93c2);
write_cmos_sensor(0x23f4, 0x8310);
write_cmos_sensor(0x23f6, 0x2404);
write_cmos_sensor(0x23f8, 0x43b2);
write_cmos_sensor(0x23fa, 0x7574);
write_cmos_sensor(0x23fc, 0x43b2);
write_cmos_sensor(0x23fe, 0x7540);
write_cmos_sensor(0x2400, 0x93c2);
write_cmos_sensor(0x2402, 0x834b);
write_cmos_sensor(0x2404, 0x2417);
write_cmos_sensor(0x2406, 0x421e);
write_cmos_sensor(0x2408, 0x8332);
write_cmos_sensor(0x240a, 0x421f);
write_cmos_sensor(0x240c, 0x8334);
write_cmos_sensor(0x240e, 0x532e);
write_cmos_sensor(0x2410, 0x630f);
write_cmos_sensor(0x2412, 0x4e82);
write_cmos_sensor(0x2414, 0x8332);
write_cmos_sensor(0x2416, 0x4f82);
write_cmos_sensor(0x2418, 0x8334);
write_cmos_sensor(0x241a, 0x421a);
write_cmos_sensor(0x241c, 0x8338);
write_cmos_sensor(0x241e, 0x421b);
write_cmos_sensor(0x2420, 0x833a);
write_cmos_sensor(0x2422, 0x4a0c);
write_cmos_sensor(0x2424, 0x4b0d);
write_cmos_sensor(0x2426, 0x8e0c);
write_cmos_sensor(0x2428, 0x7f0d);
write_cmos_sensor(0x242a, 0x2c04);
write_cmos_sensor(0x242c, 0x4a82);
write_cmos_sensor(0x242e, 0x8332);
write_cmos_sensor(0x2430, 0x4b82);
write_cmos_sensor(0x2432, 0x8334);
write_cmos_sensor(0x2434, 0x413a);
write_cmos_sensor(0x2436, 0x413b);
write_cmos_sensor(0x2438, 0x4130);
write_cmos_sensor(0x243a, 0x93c2);
write_cmos_sensor(0x243c, 0x00c3);
write_cmos_sensor(0x243e, 0x240e);
write_cmos_sensor(0x2440, 0x93c2);
write_cmos_sensor(0x2442, 0x00c1);
write_cmos_sensor(0x2444, 0x200b);
write_cmos_sensor(0x2446, 0x43d2);
write_cmos_sensor(0x2448, 0x834a);
write_cmos_sensor(0x244a, 0x93d2);
write_cmos_sensor(0x244c, 0x00bf);
write_cmos_sensor(0x244e, 0x2403);
write_cmos_sensor(0x2450, 0x43c2);
write_cmos_sensor(0x2452, 0x834c);
write_cmos_sensor(0x2454, 0x4130);
write_cmos_sensor(0x2456, 0x43d2);
write_cmos_sensor(0x2458, 0x834c);
write_cmos_sensor(0x245a, 0x4130);
write_cmos_sensor(0x245c, 0x43c2);
write_cmos_sensor(0x245e, 0x834a);
write_cmos_sensor(0x2460, 0x3ff4);
write_cmos_sensor(0x2462, 0x93c2);
write_cmos_sensor(0x2464, 0x834c);
write_cmos_sensor(0x2466, 0x2408);
write_cmos_sensor(0x2468, 0xb3e2);
write_cmos_sensor(0x246a, 0x00c2);
write_cmos_sensor(0x246c, 0x2403);
write_cmos_sensor(0x246e, 0x43d2);
write_cmos_sensor(0x2470, 0x00c8);
write_cmos_sensor(0x2472, 0x4130);
write_cmos_sensor(0x2474, 0x43c2);
write_cmos_sensor(0x2476, 0x00c8);
write_cmos_sensor(0x2478, 0x4130);
write_cmos_sensor(0x247a, 0x93c2);
write_cmos_sensor(0x247c, 0x834a);
write_cmos_sensor(0x247e, 0x2406);
write_cmos_sensor(0x2480, 0x403f);
write_cmos_sensor(0x2482, 0x00c3);
write_cmos_sensor(0x2484, 0x4fe2);
write_cmos_sensor(0x2486, 0x8330);
write_cmos_sensor(0x2488, 0xc3df);
write_cmos_sensor(0x248a, 0x0000);
write_cmos_sensor(0x248c, 0x4130);
write_cmos_sensor(0x248e, 0x93c2);
write_cmos_sensor(0x2490, 0x834a);
write_cmos_sensor(0x2492, 0x2403);
write_cmos_sensor(0x2494, 0x42d2);
write_cmos_sensor(0x2496, 0x8330);
write_cmos_sensor(0x2498, 0x00c3);
write_cmos_sensor(0x249a, 0x421e);
write_cmos_sensor(0x249c, 0x7314);
write_cmos_sensor(0x249e, 0x421f);
write_cmos_sensor(0x24a0, 0x7336);
write_cmos_sensor(0x24a2, 0x12b0);
write_cmos_sensor(0x24a4, 0xefc4);
write_cmos_sensor(0x24a6, 0x4e82);
write_cmos_sensor(0x24a8, 0x831c);
write_cmos_sensor(0x24aa, 0x4f82);
write_cmos_sensor(0x24ac, 0x831e);
write_cmos_sensor(0x24ae, 0x4130);
write_cmos_sensor(0x24b0, 0x93c2);
write_cmos_sensor(0x24b2, 0x834c);
write_cmos_sensor(0x24b4, 0x2408);
write_cmos_sensor(0x24b6, 0xb3e2);
write_cmos_sensor(0x24b8, 0x00c2);
write_cmos_sensor(0x24ba, 0x2403);
write_cmos_sensor(0x24bc, 0x43c2);
write_cmos_sensor(0x24be, 0x00c8);
write_cmos_sensor(0x24c0, 0x4130);
write_cmos_sensor(0x24c2, 0x43d2);
write_cmos_sensor(0x24c4, 0x00c8);
write_cmos_sensor(0x24c6, 0x4130);
write_cmos_sensor(0x24c8, 0x93c2);
write_cmos_sensor(0x24ca, 0x834c);
write_cmos_sensor(0x24cc, 0x2408);
write_cmos_sensor(0x24ce, 0xb3e2);
write_cmos_sensor(0x24d0, 0x00c2);
write_cmos_sensor(0x24d2, 0x2403);
write_cmos_sensor(0x24d4, 0x43d2);
write_cmos_sensor(0x24d6, 0x00c8);
write_cmos_sensor(0x24d8, 0x3c02);
write_cmos_sensor(0x24da, 0x43c2);
write_cmos_sensor(0x24dc, 0x00c8);
write_cmos_sensor(0x24de, 0x43c2);
write_cmos_sensor(0x24e0, 0x8294);
write_cmos_sensor(0x24e2, 0x4130);
write_cmos_sensor(0x24e4, 0x43c2);
write_cmos_sensor(0x24e6, 0x834b);
write_cmos_sensor(0x24e8, 0x4130);
write_cmos_sensor(0x24ea, 0x403e);
write_cmos_sensor(0x24ec, 0x00c2);
write_cmos_sensor(0x24ee, 0x421f);
write_cmos_sensor(0x24f0, 0x7312);
write_cmos_sensor(0x24f2, 0xf07f);
write_cmos_sensor(0x24f4, 0x000c);
write_cmos_sensor(0x24f6, 0x5f4f);
write_cmos_sensor(0x24f8, 0x5f4f);
write_cmos_sensor(0x24fa, 0xdfce);
write_cmos_sensor(0x24fc, 0x0000);
write_cmos_sensor(0x24fe, 0xf0fe);
write_cmos_sensor(0x2500, 0x000f);
write_cmos_sensor(0x2502, 0x0000);
write_cmos_sensor(0x2504, 0x4130);
write_cmos_sensor(0x2506, 0x4f82);
write_cmos_sensor(0x2508, 0x7334);
write_cmos_sensor(0x250a, 0x0f00);
write_cmos_sensor(0x250c, 0x7300);
write_cmos_sensor(0x250e, 0x4130);
write_cmos_sensor(0x2510, 0x421f);
write_cmos_sensor(0x2512, 0x0196);
write_cmos_sensor(0x2514, 0x503f);
write_cmos_sensor(0x2516, 0x0006);
write_cmos_sensor(0x2518, 0x4f82);
write_cmos_sensor(0x251a, 0x832e);
write_cmos_sensor(0x251c, 0x93c2);
write_cmos_sensor(0x251e, 0x00c1);
write_cmos_sensor(0x2520, 0x242d);
write_cmos_sensor(0x2522, 0x425e);
write_cmos_sensor(0x2524, 0x00c2);
write_cmos_sensor(0x2526, 0xc35e);
write_cmos_sensor(0x2528, 0x425f);
write_cmos_sensor(0x252a, 0x8310);
write_cmos_sensor(0x252c, 0xdf4e);
write_cmos_sensor(0x252e, 0x4ec2);
write_cmos_sensor(0x2530, 0x00c2);
write_cmos_sensor(0x2532, 0x934f);
write_cmos_sensor(0x2534, 0x2006);
write_cmos_sensor(0x2536, 0x9382);
write_cmos_sensor(0x2538, 0x730a);
write_cmos_sensor(0x253a, 0x27fd);
write_cmos_sensor(0x253c, 0xd3d2);
write_cmos_sensor(0x253e, 0x00c2);
write_cmos_sensor(0x2540, 0x3c1f);
write_cmos_sensor(0x2542, 0x12b0);
write_cmos_sensor(0x2544, 0xfd86);
write_cmos_sensor(0x2546, 0x12b0);
write_cmos_sensor(0x2548, 0xfdbc);
write_cmos_sensor(0x254a, 0x421f);
write_cmos_sensor(0x254c, 0x8342);
write_cmos_sensor(0x254e, 0x933f);
write_cmos_sensor(0x2550, 0x380a);
write_cmos_sensor(0x2552, 0x932f);
write_cmos_sensor(0x2554, 0x3803);
write_cmos_sensor(0x2556, 0x12b0);
write_cmos_sensor(0x2558, 0xff36);
write_cmos_sensor(0x255a, 0x3ff0);
write_cmos_sensor(0x255c, 0x12b0);
write_cmos_sensor(0x255e, 0xff52);
write_cmos_sensor(0x2560, 0x12b0);
write_cmos_sensor(0x2562, 0xff9c);
write_cmos_sensor(0x2564, 0x3feb);
write_cmos_sensor(0x2566, 0x12b0);
write_cmos_sensor(0x2568, 0xfe06);
write_cmos_sensor(0x256a, 0x12b0);
write_cmos_sensor(0x256c, 0xfe36);
write_cmos_sensor(0x256e, 0x12b0);
write_cmos_sensor(0x2570, 0xfe68);
write_cmos_sensor(0x2572, 0x12b0);
write_cmos_sensor(0x2574, 0xfec0);
write_cmos_sensor(0x2576, 0x12b0);
write_cmos_sensor(0x2578, 0xfefe);
write_cmos_sensor(0x257a, 0x3fe0);
write_cmos_sensor(0x257c, 0x0900);
write_cmos_sensor(0x257e, 0x7328);
write_cmos_sensor(0x2580, 0x12b0);
write_cmos_sensor(0x2582, 0xffc2);
write_cmos_sensor(0x2584, 0x4130);
write_cmos_sensor(0x2586, 0x421e);
write_cmos_sensor(0x2588, 0x7314);
write_cmos_sensor(0x258a, 0x421f);
write_cmos_sensor(0x258c, 0x7336);
write_cmos_sensor(0x258e, 0x12b0);
write_cmos_sensor(0x2590, 0xefc4);
write_cmos_sensor(0x2592, 0x4e82);
write_cmos_sensor(0x2594, 0x832a);
write_cmos_sensor(0x2596, 0x4f82);
write_cmos_sensor(0x2598, 0x832c);
write_cmos_sensor(0x259a, 0xb3e2);
write_cmos_sensor(0x259c, 0x00c2);
write_cmos_sensor(0x259e, 0x2407);
write_cmos_sensor(0x25a0, 0x9382);
write_cmos_sensor(0x25a2, 0x7318);
write_cmos_sensor(0x25a4, 0x27fd);
write_cmos_sensor(0x25a6, 0x9392);
write_cmos_sensor(0x25a8, 0x7318);
write_cmos_sensor(0x25aa, 0x27fd);
write_cmos_sensor(0x25ac, 0x4130);
write_cmos_sensor(0x25ae, 0x9392);
write_cmos_sensor(0x25b0, 0x7318);
write_cmos_sensor(0x25b2, 0x27fd);
write_cmos_sensor(0x25b4, 0x9382);
write_cmos_sensor(0x25b6, 0x7318);
write_cmos_sensor(0x25b8, 0x27fd);
write_cmos_sensor(0x25ba, 0x4130);
write_cmos_sensor(0x25bc, 0x120b);
write_cmos_sensor(0x25be, 0x120a);
write_cmos_sensor(0x25c0, 0x421e);
write_cmos_sensor(0x25c2, 0x7300);
write_cmos_sensor(0x25c4, 0x421f);
write_cmos_sensor(0x25c6, 0x733c);
write_cmos_sensor(0x25c8, 0x12b0);
write_cmos_sensor(0x25ca, 0xefc4);
write_cmos_sensor(0x25cc, 0x4e82);
write_cmos_sensor(0x25ce, 0x8324);
write_cmos_sensor(0x25d0, 0x4f82);
write_cmos_sensor(0x25d2, 0x8326);
write_cmos_sensor(0x25d4, 0x421c);
write_cmos_sensor(0x25d6, 0x8344);
write_cmos_sensor(0x25d8, 0x430d);
write_cmos_sensor(0x25da, 0x421a);
write_cmos_sensor(0x25dc, 0x831c);
write_cmos_sensor(0x25de, 0x421b);
write_cmos_sensor(0x25e0, 0x831e);
write_cmos_sensor(0x25e2, 0x8c0a);
write_cmos_sensor(0x25e4, 0x7d0b);
write_cmos_sensor(0x25e6, 0x8a0e);
write_cmos_sensor(0x25e8, 0x7b0f);
write_cmos_sensor(0x25ea, 0x2c04);
write_cmos_sensor(0x25ec, 0x4292);
write_cmos_sensor(0x25ee, 0x8324);
write_cmos_sensor(0x25f0, 0x8342);
write_cmos_sensor(0x25f2, 0x3c06);
write_cmos_sensor(0x25f4, 0x421b);
write_cmos_sensor(0x25f6, 0x8324);
write_cmos_sensor(0x25f8, 0x821b);
write_cmos_sensor(0x25fa, 0x831c);
write_cmos_sensor(0x25fc, 0x4b82);
write_cmos_sensor(0x25fe, 0x8342);
write_cmos_sensor(0x2600, 0x413a);
write_cmos_sensor(0x2602, 0x413b);
write_cmos_sensor(0x2604, 0x4130);
write_cmos_sensor(0x2606, 0x4382);
write_cmos_sensor(0x2608, 0x8336);
write_cmos_sensor(0x260a, 0x50b2);
write_cmos_sensor(0x260c, 0xfffd);
write_cmos_sensor(0x260e, 0x8342);
write_cmos_sensor(0x2610, 0x4382);
write_cmos_sensor(0x2612, 0x7336);
write_cmos_sensor(0x2614, 0x43a2);
write_cmos_sensor(0x2616, 0x7314);
write_cmos_sensor(0x2618, 0x403e);
write_cmos_sensor(0x261a, 0x7542);
write_cmos_sensor(0x261c, 0x4ea2);
write_cmos_sensor(0x261e, 0x833e);
write_cmos_sensor(0x2620, 0x403f);
write_cmos_sensor(0x2622, 0x7544);
write_cmos_sensor(0x2624, 0x4fa2);
write_cmos_sensor(0x2626, 0x8340);
write_cmos_sensor(0x2628, 0x429e);
write_cmos_sensor(0x262a, 0x8320);
write_cmos_sensor(0x262c, 0x0000);
write_cmos_sensor(0x262e, 0x429f);
write_cmos_sensor(0x2630, 0x8322);
write_cmos_sensor(0x2632, 0x0000);
write_cmos_sensor(0x2634, 0x4130);
write_cmos_sensor(0x2636, 0x421e);
write_cmos_sensor(0x2638, 0x8324);
write_cmos_sensor(0x263a, 0x421f);
write_cmos_sensor(0x263c, 0x8326);
write_cmos_sensor(0x263e, 0x821e);
write_cmos_sensor(0x2640, 0x8346);
write_cmos_sensor(0x2642, 0x721f);
write_cmos_sensor(0x2644, 0x8348);
write_cmos_sensor(0x2646, 0x2c0a);
write_cmos_sensor(0x2648, 0x4382);
write_cmos_sensor(0x264a, 0x7574);
write_cmos_sensor(0x264c, 0x4382);
write_cmos_sensor(0x264e, 0x7540);
write_cmos_sensor(0x2650, 0x421f);
write_cmos_sensor(0x2652, 0x831c);
write_cmos_sensor(0x2654, 0x821f);
write_cmos_sensor(0x2656, 0x8346);
write_cmos_sensor(0x2658, 0x4f82);
write_cmos_sensor(0x265a, 0x8336);
write_cmos_sensor(0x265c, 0x12b0);
write_cmos_sensor(0x265e, 0xfcea);
write_cmos_sensor(0x2660, 0x0b00);
write_cmos_sensor(0x2662, 0x7302);
write_cmos_sensor(0x2664, 0x0578);
write_cmos_sensor(0x2666, 0x4130);
write_cmos_sensor(0x2668, 0x4392);
write_cmos_sensor(0x266a, 0x7318);
write_cmos_sensor(0x266c, 0x4292);
write_cmos_sensor(0x266e, 0x833e);
write_cmos_sensor(0x2670, 0x7542);
write_cmos_sensor(0x2672, 0x4292);
write_cmos_sensor(0x2674, 0x8340);
write_cmos_sensor(0x2676, 0x7544);
write_cmos_sensor(0x2678, 0x421e);
write_cmos_sensor(0x267a, 0x8342);
write_cmos_sensor(0x267c, 0x4e0f);
write_cmos_sensor(0x267e, 0x5f0f);
write_cmos_sensor(0x2680, 0x7f0f);
write_cmos_sensor(0x2682, 0xe33f);
write_cmos_sensor(0x2684, 0x521e);
write_cmos_sensor(0x2686, 0x8332);
write_cmos_sensor(0x2688, 0x621f);
write_cmos_sensor(0x268a, 0x8334);
write_cmos_sensor(0x268c, 0x4e82);
write_cmos_sensor(0x268e, 0x8332);
write_cmos_sensor(0x2690, 0x4f82);
write_cmos_sensor(0x2692, 0x8334);
write_cmos_sensor(0x2694, 0x421d);
write_cmos_sensor(0x2696, 0x8336);
write_cmos_sensor(0x2698, 0x522d);
write_cmos_sensor(0x269a, 0x4d0c);
write_cmos_sensor(0x269c, 0x4c0d);
write_cmos_sensor(0x269e, 0x5d0d);
write_cmos_sensor(0x26a0, 0x7d0d);
write_cmos_sensor(0x26a2, 0xe33d);
write_cmos_sensor(0x26a4, 0x8c0e);
write_cmos_sensor(0x26a6, 0x7d0f);
write_cmos_sensor(0x26a8, 0x2c04);
write_cmos_sensor(0x26aa, 0x4c82);
write_cmos_sensor(0x26ac, 0x8332);
write_cmos_sensor(0x26ae, 0x4d82);
write_cmos_sensor(0x26b0, 0x8334);
write_cmos_sensor(0x26b2, 0x4292);
write_cmos_sensor(0x26b4, 0x8334);
write_cmos_sensor(0x26b6, 0x7574);
write_cmos_sensor(0x26b8, 0x4292);
write_cmos_sensor(0x26ba, 0x8332);
write_cmos_sensor(0x26bc, 0x7540);
write_cmos_sensor(0x26be, 0x4130);
write_cmos_sensor(0x26c0, 0x421e);
write_cmos_sensor(0x26c2, 0x8342);
write_cmos_sensor(0x26c4, 0x4e0f);
write_cmos_sensor(0x26c6, 0x5f0f);
write_cmos_sensor(0x26c8, 0x7f0f);
write_cmos_sensor(0x26ca, 0xe33f);
write_cmos_sensor(0x26cc, 0x521e);
write_cmos_sensor(0x26ce, 0x832a);
write_cmos_sensor(0x26d0, 0x621f);
write_cmos_sensor(0x26d2, 0x832c);
write_cmos_sensor(0x26d4, 0x4e82);
write_cmos_sensor(0x26d6, 0x832a);
write_cmos_sensor(0x26d8, 0x4f82);
write_cmos_sensor(0x26da, 0x832c);
write_cmos_sensor(0x26dc, 0x421c);
write_cmos_sensor(0x26de, 0x832e);
write_cmos_sensor(0x26e0, 0x430d);
write_cmos_sensor(0x26e2, 0x8c0e);
write_cmos_sensor(0x26e4, 0x7d0f);
write_cmos_sensor(0x26e6, 0x2c04);
write_cmos_sensor(0x26e8, 0x4c82);
write_cmos_sensor(0x26ea, 0x832a);
write_cmos_sensor(0x26ec, 0x4d82);
write_cmos_sensor(0x26ee, 0x832c);
write_cmos_sensor(0x26f0, 0x4292);
write_cmos_sensor(0x26f2, 0x832c);
write_cmos_sensor(0x26f4, 0x7336);
write_cmos_sensor(0x26f6, 0x4292);
write_cmos_sensor(0x26f8, 0x832a);
write_cmos_sensor(0x26fa, 0x7314);
write_cmos_sensor(0x26fc, 0x4130);
write_cmos_sensor(0x26fe, 0x4292);
write_cmos_sensor(0x2700, 0x7320);
write_cmos_sensor(0x2702, 0x8328);
write_cmos_sensor(0x2704, 0x421e);
write_cmos_sensor(0x2706, 0x8336);
write_cmos_sensor(0x2708, 0x4e0f);
write_cmos_sensor(0x270a, 0x821f);
write_cmos_sensor(0x270c, 0x8342);
write_cmos_sensor(0x270e, 0x4f82);
write_cmos_sensor(0x2710, 0x833c);
write_cmos_sensor(0x2712, 0x421d);
write_cmos_sensor(0x2714, 0x8328);
write_cmos_sensor(0x2716, 0x8f0d);
write_cmos_sensor(0x2718, 0x4d82);
write_cmos_sensor(0x271a, 0x8328);
write_cmos_sensor(0x271c, 0x922d);
write_cmos_sensor(0x271e, 0x3402);
write_cmos_sensor(0x2720, 0x42a2);
write_cmos_sensor(0x2722, 0x8328);
write_cmos_sensor(0x2724, 0x4292);
write_cmos_sensor(0x2726, 0x8328);
write_cmos_sensor(0x2728, 0x7320);
write_cmos_sensor(0x272a, 0x931e);
write_cmos_sensor(0x272c, 0x3803);
write_cmos_sensor(0x272e, 0x4e0f);
write_cmos_sensor(0x2730, 0x12b0);
write_cmos_sensor(0x2732, 0xfd06);
write_cmos_sensor(0x2734, 0x4130);
write_cmos_sensor(0x2736, 0x0b00);
write_cmos_sensor(0x2738, 0x7302);
write_cmos_sensor(0x273a, 0x0578);
write_cmos_sensor(0x273c, 0x4392);
write_cmos_sensor(0x273e, 0x7318);
write_cmos_sensor(0x2740, 0x4292);
write_cmos_sensor(0x2742, 0x8334);
write_cmos_sensor(0x2744, 0x7574);
write_cmos_sensor(0x2746, 0x4292);
write_cmos_sensor(0x2748, 0x8332);
write_cmos_sensor(0x274a, 0x7540);
write_cmos_sensor(0x274c, 0x12b0);
write_cmos_sensor(0x274e, 0xfcea);
write_cmos_sensor(0x2750, 0x4130);
write_cmos_sensor(0x2752, 0x120b);
write_cmos_sensor(0x2754, 0x120a);
write_cmos_sensor(0x2756, 0x403a);
write_cmos_sensor(0x2758, 0x7314);
write_cmos_sensor(0x275a, 0x403b);
write_cmos_sensor(0x275c, 0x7336);
write_cmos_sensor(0x275e, 0x4a2e);
write_cmos_sensor(0x2760, 0x4b2f);
write_cmos_sensor(0x2762, 0x12b0);
write_cmos_sensor(0x2764, 0xefc4);
write_cmos_sensor(0x2766, 0x4e0c);
write_cmos_sensor(0x2768, 0x4f0d);
write_cmos_sensor(0x276a, 0x421e);
write_cmos_sensor(0x276c, 0x8342);
write_cmos_sensor(0x276e, 0x4e0f);
write_cmos_sensor(0x2770, 0x5f0f);
write_cmos_sensor(0x2772, 0x7f0f);
write_cmos_sensor(0x2774, 0xe33f);
write_cmos_sensor(0x2776, 0x5e0c);
write_cmos_sensor(0x2778, 0x6f0d);
write_cmos_sensor(0x277a, 0x4d8b);
write_cmos_sensor(0x277c, 0x0000);
write_cmos_sensor(0x277e, 0x4c8a);
write_cmos_sensor(0x2780, 0x0000);
write_cmos_sensor(0x2782, 0x421c);
write_cmos_sensor(0x2784, 0x8332);
write_cmos_sensor(0x2786, 0x421d);
write_cmos_sensor(0x2788, 0x8334);
write_cmos_sensor(0x278a, 0x5e0c);
write_cmos_sensor(0x278c, 0x6f0d);
write_cmos_sensor(0x278e, 0x4d82);
write_cmos_sensor(0x2790, 0x7574);
write_cmos_sensor(0x2792, 0x4c82);
write_cmos_sensor(0x2794, 0x7540);
write_cmos_sensor(0x2796, 0x413a);
write_cmos_sensor(0x2798, 0x413b);
write_cmos_sensor(0x279a, 0x4130);
write_cmos_sensor(0x279c, 0x421e);
write_cmos_sensor(0x279e, 0x7300);
write_cmos_sensor(0x27a0, 0x421f);
write_cmos_sensor(0x27a2, 0x733c);
write_cmos_sensor(0x27a4, 0x12b0);
write_cmos_sensor(0x27a6, 0xefc4);
write_cmos_sensor(0x27a8, 0x803e);
write_cmos_sensor(0x27aa, 0x0003);
write_cmos_sensor(0x27ac, 0x730f);
write_cmos_sensor(0x27ae, 0x2ff6);
write_cmos_sensor(0x27b0, 0x12b0);
write_cmos_sensor(0x27b2, 0xfcea);
write_cmos_sensor(0x27b4, 0x0b00);
write_cmos_sensor(0x27b6, 0x7300);
write_cmos_sensor(0x27b8, 0x0002);
write_cmos_sensor(0x27ba, 0x0b00);
write_cmos_sensor(0x27bc, 0x7302);
write_cmos_sensor(0x27be, 0x0320);
write_cmos_sensor(0x27c0, 0x4130);
write_cmos_sensor(0x27c2, 0x4292);
write_cmos_sensor(0x27c4, 0x7542);
write_cmos_sensor(0x27c6, 0x8320);
write_cmos_sensor(0x27c8, 0x4292);
write_cmos_sensor(0x27ca, 0x7544);
write_cmos_sensor(0x27cc, 0x8322);
write_cmos_sensor(0x27ce, 0x4130);
write_cmos_sensor(0x27fe, 0xe000);
write_cmos_sensor(0x3000, 0x60f8);
write_cmos_sensor(0x3002, 0x187f);
write_cmos_sensor(0x3004, 0x7060);
write_cmos_sensor(0x3006, 0x0114);
write_cmos_sensor(0x3008, 0x60b0);
write_cmos_sensor(0x300a, 0x1473);
write_cmos_sensor(0x300c, 0x0013);
write_cmos_sensor(0x300e, 0x140f);
write_cmos_sensor(0x3010, 0x0040);
write_cmos_sensor(0x3012, 0x100f);
write_cmos_sensor(0x3014, 0x60f8);
write_cmos_sensor(0x3016, 0x187f);
write_cmos_sensor(0x3018, 0x7060);
write_cmos_sensor(0x301a, 0x0114);
write_cmos_sensor(0x301c, 0x60b0);
write_cmos_sensor(0x301e, 0x1473);
write_cmos_sensor(0x3020, 0x0013);
write_cmos_sensor(0x3022, 0x140f);
write_cmos_sensor(0x3024, 0x0040);
write_cmos_sensor(0x3026, 0x000f);
write_cmos_sensor(0x0b00, 0x0000);
write_cmos_sensor(0x0b02, 0x0045);
write_cmos_sensor(0x0b04, 0xb405);
write_cmos_sensor(0x0b06, 0xc403);
write_cmos_sensor(0x0b08, 0x0081);
write_cmos_sensor(0x0b0a, 0x8252);
write_cmos_sensor(0x0b0c, 0xf814);
write_cmos_sensor(0x0b0e, 0xc618);
write_cmos_sensor(0x0b10, 0xa828);
write_cmos_sensor(0x0b12, 0x004c);
write_cmos_sensor(0x0b14, 0x4068);
write_cmos_sensor(0x0b16, 0x0000);
write_cmos_sensor(0x0f30, 0x6e25);
write_cmos_sensor(0x0f32, 0x7067);
write_cmos_sensor(0x0954, 0x0009);
write_cmos_sensor(0x0956, 0x1100);
write_cmos_sensor(0x0958, 0xcc80);
write_cmos_sensor(0x095a, 0x0000);
write_cmos_sensor(0x0c00, 0x1110);
write_cmos_sensor(0x0c02, 0x0011);
write_cmos_sensor(0x0c04, 0x0000);
write_cmos_sensor(0x0c06, 0x0200);
write_cmos_sensor(0x0c10, 0x0040);
write_cmos_sensor(0x0c12, 0x0040);
write_cmos_sensor(0x0c14, 0x0040);
write_cmos_sensor(0x0c16, 0x0040);
write_cmos_sensor(0x0a10, 0x4000);
write_cmos_sensor(0x3068, 0xffff);
write_cmos_sensor(0x306a, 0xffff);
write_cmos_sensor(0x006c, 0x0300);
write_cmos_sensor(0x005e, 0x0200);
write_cmos_sensor(0x000e, 0x0100);
write_cmos_sensor(0x0e0a, 0x0001);
write_cmos_sensor(0x004a, 0x0100);
write_cmos_sensor(0x004c, 0x0000);
write_cmos_sensor(0x000c, 0x0022);
write_cmos_sensor(0x0008, 0x0b00);
write_cmos_sensor(0x005a, 0x0202);
write_cmos_sensor(0x0012, 0x000e);
write_cmos_sensor(0x0018, 0x0a31);
write_cmos_sensor(0x0022, 0x0008);
write_cmos_sensor(0x0028, 0x0017);
write_cmos_sensor(0x0024, 0x0028);
write_cmos_sensor(0x002a, 0x002d);
write_cmos_sensor(0x0026, 0x0030);
write_cmos_sensor(0x002c, 0x07c7);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0030, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x0823);
write_cmos_sensor(0x0116, 0x07b6);
write_cmos_sensor(0x0a22, 0x0000);
write_cmos_sensor(0x0a12, 0x0a20);
write_cmos_sensor(0x0a14, 0x0798);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0074, 0x080e);
write_cmos_sensor(0x0070, 0x0407);
write_cmos_sensor(0x0002, 0x0000);
write_cmos_sensor(0x0a02, 0x0000);
write_cmos_sensor(0x0a24, 0x0100);
write_cmos_sensor(0x0046, 0x0000);
write_cmos_sensor(0x0076, 0x0000);
write_cmos_sensor(0x0060, 0x0000);
write_cmos_sensor(0x0062, 0x0530);
write_cmos_sensor(0x0064, 0x0500);
write_cmos_sensor(0x0066, 0x0530);
write_cmos_sensor(0x0068, 0x0500);
write_cmos_sensor(0x0122, 0x0300);
write_cmos_sensor(0x015a, 0xff08);
write_cmos_sensor(0x0804, 0x0200);
write_cmos_sensor(0x005c, 0x0100);
write_cmos_sensor(0x0a1a, 0x0800);


write_cmos_sensor(0x0a00, 0x0000);

}				/*    MIPI_sensor_Init  */


static void preview_setting(void)
{
	LOG_INF(" preview_setting E!");

	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8259);
write_cmos_sensor(0x0f30, 0x6e25);
write_cmos_sensor(0x0f32, 0x7167);
write_cmos_sensor(0x004a, 0x0100);
write_cmos_sensor(0x004c, 0x0000);
write_cmos_sensor(0x000c, 0x0122);
write_cmos_sensor(0x0008, 0x0b00);
write_cmos_sensor(0x005a, 0x0404);
write_cmos_sensor(0x0012, 0x000c);
write_cmos_sensor(0x0018, 0x0a33);
write_cmos_sensor(0x0022, 0x0008);
write_cmos_sensor(0x0028, 0x0017);
write_cmos_sensor(0x0024, 0x0022);
write_cmos_sensor(0x002a, 0x002b);
write_cmos_sensor(0x0026, 0x0030);
write_cmos_sensor(0x002c, 0x07c7);
write_cmos_sensor(0x002e, 0x3311);
write_cmos_sensor(0x0030, 0x3311);
write_cmos_sensor(0x0032, 0x3311);
write_cmos_sensor(0x0006, 0x0823);
write_cmos_sensor(0x0a22, 0x0000);
write_cmos_sensor(0x0a12, 0x0510);
write_cmos_sensor(0x0a14, 0x03cc);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0074, 0x0821);
write_cmos_sensor(0x0070, 0x0411);
write_cmos_sensor(0x0804, 0x0208);
write_cmos_sensor(0x0a04, 0x016a);
write_cmos_sensor(0x090c, 0x09c0);
write_cmos_sensor(0x090e, 0x0010);
write_cmos_sensor(0x0040, 0x0000);
write_cmos_sensor(0x0042, 0x0100);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0126, 0x00f9);
write_cmos_sensor(0x0902, 0x4319);
write_cmos_sensor(0x0914, 0xc106);
write_cmos_sensor(0x0916, 0x040e);
write_cmos_sensor(0x0918, 0x0304);
write_cmos_sensor(0x091a, 0x0708);
write_cmos_sensor(0x091c, 0x0e06);
write_cmos_sensor(0x091e, 0x0300);
write_cmos_sensor(0x0958, 0xbb80);
}				/*    preview_setting  */


static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("123456 E! currefps:%d\n", currefps);

write_cmos_sensor(0x0a00, 0x0000);
write_cmos_sensor(0x0b0a, 0x8252);
write_cmos_sensor(0x0f30, 0x6e25);
write_cmos_sensor(0x0f32, 0x7067);
write_cmos_sensor(0x004a, 0x0100);
write_cmos_sensor(0x004c, 0x0000);
write_cmos_sensor(0x000c, 0x0022);
write_cmos_sensor(0x0008, 0x0b00);
write_cmos_sensor(0x005a, 0x0202);
write_cmos_sensor(0x0012, 0x000e);
write_cmos_sensor(0x0018, 0x0a31);
write_cmos_sensor(0x0022, 0x0008);
write_cmos_sensor(0x0028, 0x0017);
write_cmos_sensor(0x0024, 0x0028);
write_cmos_sensor(0x002a, 0x002d);
write_cmos_sensor(0x0026, 0x0030);
write_cmos_sensor(0x002c, 0x07c7);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0030, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x0823);
write_cmos_sensor(0x0116, 0x07b6);
write_cmos_sensor(0x0a22, 0x0000);
write_cmos_sensor(0x0a12, 0x0a20);
write_cmos_sensor(0x0a14, 0x0798);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0074, 0x0821);
write_cmos_sensor(0x0070, 0x0411);
write_cmos_sensor(0x0804, 0x0200);
write_cmos_sensor(0x0a04, 0x014a);
write_cmos_sensor(0x090c, 0x0fdc);
write_cmos_sensor(0x090e, 0x002d);
write_cmos_sensor(0x0040, 0x0000);
write_cmos_sensor(0x0042, 0x0100);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0126, 0x00f9);
write_cmos_sensor(0x0902, 0x4319);
write_cmos_sensor(0x0914, 0xc10a);
write_cmos_sensor(0x0916, 0x071f);
write_cmos_sensor(0x0918, 0x0408);
write_cmos_sensor(0x091a, 0x0c0d);
write_cmos_sensor(0x091c, 0x0f09);
write_cmos_sensor(0x091e, 0x0a00);
write_cmos_sensor(0x0958, 0xbb80);

}				/*    capture_setting  */


static void custom1_setting()
{
	LOG_INF("custom1_setting");

write_cmos_sensor(0x0a00, 0x0000);
write_cmos_sensor(0x0b0a, 0x8252);
write_cmos_sensor(0x0f30, 0x6e25);
write_cmos_sensor(0x0f32, 0x7067);
write_cmos_sensor(0x004a, 0x0100);
write_cmos_sensor(0x004c, 0x0000);
write_cmos_sensor(0x000c, 0x0022);
write_cmos_sensor(0x0008, 0x0b00);
write_cmos_sensor(0x005a, 0x0202);
write_cmos_sensor(0x0012, 0x000e);
write_cmos_sensor(0x0018, 0x0a31);
write_cmos_sensor(0x0022, 0x0008);
write_cmos_sensor(0x0028, 0x0017);
write_cmos_sensor(0x0024, 0x0028);
write_cmos_sensor(0x002a, 0x002d);
write_cmos_sensor(0x0026, 0x0030);
write_cmos_sensor(0x002c, 0x07c7);
write_cmos_sensor(0x002e, 0x1111);
write_cmos_sensor(0x0030, 0x1111);
write_cmos_sensor(0x0032, 0x1111);
write_cmos_sensor(0x0006, 0x0823);
write_cmos_sensor(0x0116, 0x07b6);
write_cmos_sensor(0x0a22, 0x0000);
write_cmos_sensor(0x0a12, 0x0a20);
write_cmos_sensor(0x0a14, 0x0798);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0074, 0x0821);
write_cmos_sensor(0x0070, 0x0411);
write_cmos_sensor(0x0804, 0x0200);
write_cmos_sensor(0x0a04, 0x014a);
write_cmos_sensor(0x090c, 0x0fdc);
write_cmos_sensor(0x090e, 0x002d);
write_cmos_sensor(0x0040, 0x0001);
write_cmos_sensor(0x0042, 0x0101);
write_cmos_sensor(0x003e, 0x0000);
write_cmos_sensor(0x0126, 0x00f9);
write_cmos_sensor(0x0902, 0x4319);
write_cmos_sensor(0x0914, 0xc10a);
write_cmos_sensor(0x0916, 0x071f);
write_cmos_sensor(0x0918, 0x0408);
write_cmos_sensor(0x091a, 0x0c0d);
write_cmos_sensor(0x091c, 0x0f09);
write_cmos_sensor(0x091e, 0x0a00);
write_cmos_sensor(0x0958, 0xbb80);

}				/*    capture_setting  */


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);

    /********************************************************
       *
       *   1296x972 30fps 2 lane MIPI 420Mbps/lane
       *
       ********************************************************/
	preview_setting();
}				/*    preview_setting  */


static void video_1080p_setting(void)
{
	LOG_INF(" 1080p");
	preview_setting();

}				/*    preview_setting  */

static void video_720p_setting(void)
{
	LOG_INF(" 720p");
	preview_setting();

	LOG_INF("Exit!");
}				/*    preview_setting  */


static void hs_video_setting(void)
{
	LOG_INF("E\n");

	video_1080p_setting();
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");

	video_720p_setting();
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
			LOG_INF("mtk_test: oh Read sensor id fail, id: 0x%x\n", *sensor_id);
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
		LOG_INF("mtk_test: Read sensor i2c addr id: 0x%x\n", imgsensor.i2c_write_id);
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("mtk_test: i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("mtk_test: Read sensor id fail, id: 0x%x\n", sensor_id);
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
	LOG_INF(" preview E\n");

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
	LOG_INF(" capture E\n");

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
	LOG_INF(" video E\n");

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
	LOG_INF(" hE\n");

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
	LOG_INF(" slimE\n");

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
	LOG_INF(" get res E\n");
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
	LOG_INF(" get info scenario_id = %d\n", scenario_id);


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
	LOG_INF(" control scenario_id = %d\n", scenario_id);
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
	if (0) {
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
		}
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
	if (0) {
	/* 0x503D[1:0]; 00 Color bar, 01 Random Data, 10 Square */
	if (enable)
		write_cmos_sensor(0x503D, 0x80);
	else
		write_cmos_sensor(0x503D, 0x00);
		}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor(0x0a00, 0x0100);
	else {
		write_cmos_sensor(0x0a00, 0x0000);
		mdelay(30);
	}

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

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
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


UINT32 HI556_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}				/*    HI556MIPISensorInit    */
