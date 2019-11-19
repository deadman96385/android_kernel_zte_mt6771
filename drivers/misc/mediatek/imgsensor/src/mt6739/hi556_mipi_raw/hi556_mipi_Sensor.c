/*****************************************************************************
 *
 * Filename:
 * ---------
 *   Hi556_mipi_raw_Sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
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
#include <asm/atomic.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi556_mipi_Sensor.h"

#define LOG_INF LOG_INF_NEW
#define PFX "hi556_camera_sensor"
#define LOG_INF_NEW(format, args...)    pr_debug(PFX "[%s] " format, __func__, ##args)
#define LOG_INF_LOD(format, args...)  xlog_printk(ANDROID_LOG_INFO, PFX, "[%s] " format, __func__, ##args)
#define LOG_1 LOG_INF("hi556,MIPI 2LANE\n")
#define SENSORDB LOG_INF
static DEFINE_SPINLOCK(imgsensor_drv_lock);
kal_uint16 hi556_eeprom_data_awb[8] = { 0 };
static imgsensor_info_struct imgsensor_info = {
	.sensor_id = HI556_SENSOR_ID,
	.checksum_value = 0x55e2a82f,
	.pre = {
		.pclk = 160000000,
		.linelength = 2816,
		.framelength = 2049,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1296,
		.grabwindow_height = 972,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
#ifdef NONCONTINUEMODE
	.cap = {
		.pclk = 176000000,
		.linelength  = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2592,
		.grabwindow_height = 1944,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
#else
	.cap = {
		.pclk = 176000000,
		.linelength  = 2816,
		.framelength = 2049,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
#endif
	.cap1 = {
		.pclk = 176000000,
		.linelength  = 2816,
		.framelength = 4166,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 150,
	},
	.normal_video = {
		.pclk = 176000000,
		.linelength  = 2816,
		.framelength = 2083,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2592,
		.grabwindow_height = 1944,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 176000000,
		.linelength = 2816,
		.framelength = 520,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 640,
		.grabwindow_height = 480,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 19,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 176000000,
		.linelength  = 2816,
		.framelength = 2049,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1296,
		.grabwindow_height = 972,
		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 14,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 176000000,
		.linelength = 4592,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2096,
		.grabwindow_height = 1552,

		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom2 = {
		.pclk = 176000000,
		.linelength = 4592,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2096,
		.grabwindow_height = 1552,

		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom3 = {
		.pclk = 176000000,
		.linelength = 4592,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2096,
		.grabwindow_height = 1552,

		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom4 = {
		.pclk = 176000000,
		.linelength = 4592,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2096,
		.grabwindow_height = 1552,

		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.custom5 = {
		.pclk = 440000000,
		.linelength = 4592,
		.framelength = 3188,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2096,
		.grabwindow_height = 1552,

		/*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/*   following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},

	.margin = 4,
	.min_shutter = 4,
	.max_frame_length = 0x7FFF,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x50, 0xff},
	.i2c_speed = 300,
};

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x200,
	.gain = 0x200,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 0,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = KAL_FALSE,
	.i2c_write_id = 0x50,
};


/* Sensor output window information */

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944,   0,    0, 2592, 1944, 1296,  972,   0, 0, 1296,  972,   0, 0, 1296,  972},
	{ 2592, 1944,   0,    0, 2592, 1944, 2592, 1944,   0, 0, 2592, 1944,   0, 0, 2592, 1944},
	{ 2592, 1944,   0,    0, 2592, 1944, 2592, 1944,   0, 0, 2592, 1944,   0, 0, 2592, 1944},
	{ 2592, 1944,   0,    0, 2592, 1944, 1296,  972,   0, 0,  640,  480,   0, 0,  640,  480},
	{ 2592, 1944,   0,    0, 2592, 1944, 1296,  972,   0, 0, 1296,  972,   0, 0, 1296,  972},
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(imgsensor_info.i2c_speed);
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte << 8) & 0xFF00) | ((get_byte >> 8) & 0x00FF);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	kdSetI2CSpeed(imgsensor_info.i2c_speed);
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel,
	or you can set dummy by imgsensor.frame_length
	and imgsensor.line_length
	*/
	write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0008, imgsensor.line_length & 0xFFFF);
} /*  set_dummy  */


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable(%d)\n", framerate, min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
	frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en) {
		imgsensor.min_frame_length = imgsensor.frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
} /*  set_max_framerate  */


/*************************************************************************
* FUNCTION
* set_shutter
*
* DESCRIPTION
* This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
* iShutter : exposured lines
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin) {
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	} else {
		imgsensor.frame_length = imgsensor.min_frame_length;
	}
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	}
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
	(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305) {
			set_max_framerate(296, 0);
		} else if (realtime_fps >= 147 && realtime_fps <= 150) {
			set_max_framerate(146, 0);
		} else {
			write_cmos_sensor(0x0046, 0x0100);
			write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		write_cmos_sensor(0x0046, 0x0100);
		write_cmos_sensor(0x0006, imgsensor.frame_length & 0xFFFF);
	}
	write_cmos_sensor(0X0074, shutter & 0xFFFF);
	write_cmos_sensor(0x0046, 0x0000);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain / 4 - 16;
	return (kal_uint16)reg_gain;
}
/*************************************************************************
* FUNCTION
* set_gain
*
* DESCRIPTION
* This function is to set global gain to sensor.
*
* PARAMETERS
* iGain : sensor global gain(base: 0x40)
*
* RETURNS
* the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	LOG_INF("set_gain %d\n", gain);
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN) {
			gain = BASEGAIN;
		} else if (gain > 16 * BASEGAIN) {
			gain = 16 * BASEGAIN;
		}
	}
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0076, (reg_gain & 0xFFFF));
	write_cmos_sensor(0x0046, 0x0000);
	return gain;
} /*  set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin) {
			imgsensor.frame_length = le + imgsensor_info.margin;
		} else {
			imgsensor.frame_length = imgsensor.min_frame_length;
		}
		if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
			imgsensor.frame_length = imgsensor_info.max_frame_length;
		}
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter) {
			le = imgsensor_info.min_shutter;
		}
		if (se < imgsensor_info.min_shutter) {
			se = imgsensor_info.min_shutter;
		}
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3512, (se << 4) & 0xFF);
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
/*************************************************************************
* FUNCTION
* night_mode
*
* DESCRIPTION
* This function night mode of sensor.
*
* PARAMETERS
* bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
} /*  night_mode  */

static void sensor_init(void)
{
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0e00, 0x0102);
	write_cmos_sensor(0x0e02, 0x0102);
	write_cmos_sensor(0x0e0c, 0x0100);
	write_cmos_sensor(0x2000, 0x7400);
	write_cmos_sensor(0x2002, 0x001c);
	write_cmos_sensor(0x2004, 0x0242);
	write_cmos_sensor(0x2006, 0x0942);
	write_cmos_sensor(0x2008, 0x7007);
	write_cmos_sensor(0x200a, 0x0fd9);
	write_cmos_sensor(0x200c, 0x0259);
	write_cmos_sensor(0x200e, 0x7008);
	write_cmos_sensor(0x2010, 0x160e);
	write_cmos_sensor(0x2012, 0x0047);
	write_cmos_sensor(0x2014, 0x2118);
	write_cmos_sensor(0x2016, 0x0041);
	write_cmos_sensor(0x2018, 0x00d8);
	write_cmos_sensor(0x201a, 0x0145);
	write_cmos_sensor(0x201c, 0x0006);
	write_cmos_sensor(0x201e, 0x0181);
	write_cmos_sensor(0x2020, 0x13cc);
	write_cmos_sensor(0x2022, 0x2057);
	write_cmos_sensor(0x2024, 0x7001);
	write_cmos_sensor(0x2026, 0x0fca);
	write_cmos_sensor(0x2028, 0x00cb);
	write_cmos_sensor(0x202a, 0x009f);
	write_cmos_sensor(0x202c, 0x7002);
	write_cmos_sensor(0x202e, 0x13cc);
	write_cmos_sensor(0x2030, 0x019b);
	write_cmos_sensor(0x2032, 0x014d);
	write_cmos_sensor(0x2034, 0x2987);
	write_cmos_sensor(0x2036, 0x2766);
	write_cmos_sensor(0x2038, 0x0020);
	write_cmos_sensor(0x203a, 0x2060);
	write_cmos_sensor(0x203c, 0x0e5d);
	write_cmos_sensor(0x203e, 0x181d);
	write_cmos_sensor(0x2040, 0x2066);
	write_cmos_sensor(0x2042, 0x20c4);
	write_cmos_sensor(0x2044, 0x5000);
	write_cmos_sensor(0x2046, 0x0005);
	write_cmos_sensor(0x2048, 0x0000);
	write_cmos_sensor(0x204a, 0x01db);
	write_cmos_sensor(0x204c, 0x025a);
	write_cmos_sensor(0x204e, 0x00c0);
	write_cmos_sensor(0x2050, 0x0005);
	write_cmos_sensor(0x2052, 0x0006);
	write_cmos_sensor(0x2054, 0x0ad9);
	write_cmos_sensor(0x2056, 0x0259);
	write_cmos_sensor(0x2058, 0x0618);
	write_cmos_sensor(0x205a, 0x0258);
	write_cmos_sensor(0x205c, 0x2266);
	write_cmos_sensor(0x205e, 0x20c8);
	write_cmos_sensor(0x2060, 0x2060);
	write_cmos_sensor(0x2062, 0x707b);
	write_cmos_sensor(0x2064, 0x0fdd);
	write_cmos_sensor(0x2066, 0x81b8);
	write_cmos_sensor(0x2068, 0x5040);
	write_cmos_sensor(0x206a, 0x0020);
	write_cmos_sensor(0x206c, 0x5060);
	write_cmos_sensor(0x206e, 0x3143);
	write_cmos_sensor(0x2070, 0x5081);
	write_cmos_sensor(0x2072, 0x025c);
	write_cmos_sensor(0x2074, 0x7800);
	write_cmos_sensor(0x2076, 0x7400);
	write_cmos_sensor(0x2078, 0x001c);
	write_cmos_sensor(0x207a, 0x0242);
	write_cmos_sensor(0x207c, 0x0942);
	write_cmos_sensor(0x207e, 0x0bd9);
	write_cmos_sensor(0x2080, 0x0259);
	write_cmos_sensor(0x2082, 0x7008);
	write_cmos_sensor(0x2084, 0x160e);
	write_cmos_sensor(0x2086, 0x0047);
	write_cmos_sensor(0x2088, 0x2118);
	write_cmos_sensor(0x208a, 0x0041);
	write_cmos_sensor(0x208c, 0x00d8);
	write_cmos_sensor(0x208e, 0x0145);
	write_cmos_sensor(0x2090, 0x0006);
	write_cmos_sensor(0x2092, 0x0181);
	write_cmos_sensor(0x2094, 0x13cc);
	write_cmos_sensor(0x2096, 0x2057);
	write_cmos_sensor(0x2098, 0x7001);
	write_cmos_sensor(0x209a, 0x0fca);
	write_cmos_sensor(0x209c, 0x00cb);
	write_cmos_sensor(0x209e, 0x009f);
	write_cmos_sensor(0x20a0, 0x7002);
	write_cmos_sensor(0x20a2, 0x13cc);
	write_cmos_sensor(0x20a4, 0x019b);
	write_cmos_sensor(0x20a6, 0x014d);
	write_cmos_sensor(0x20a8, 0x2987);
	write_cmos_sensor(0x20aa, 0x2766);
	write_cmos_sensor(0x20ac, 0x0020);
	write_cmos_sensor(0x20ae, 0x2060);
	write_cmos_sensor(0x20b0, 0x0e5d);
	write_cmos_sensor(0x20b2, 0x181d);
	write_cmos_sensor(0x20b4, 0x2066);
	write_cmos_sensor(0x20b6, 0x20c4);
	write_cmos_sensor(0x20b8, 0x50a0);
	write_cmos_sensor(0x20ba, 0x0005);
	write_cmos_sensor(0x20bc, 0x0000);
	write_cmos_sensor(0x20be, 0x01db);
	write_cmos_sensor(0x20c0, 0x025a);
	write_cmos_sensor(0x20c2, 0x00c0);
	write_cmos_sensor(0x20c4, 0x0005);
	write_cmos_sensor(0x20c6, 0x0006);
	write_cmos_sensor(0x20c8, 0x0ad9);
	write_cmos_sensor(0x20ca, 0x0259);
	write_cmos_sensor(0x20cc, 0x0618);
	write_cmos_sensor(0x20ce, 0x0258);
	write_cmos_sensor(0x20d0, 0x2266);
	write_cmos_sensor(0x20d2, 0x20c8);
	write_cmos_sensor(0x20d4, 0x2060);
	write_cmos_sensor(0x20d6, 0x707b);
	write_cmos_sensor(0x20d8, 0x0fdd);
	write_cmos_sensor(0x20da, 0x86b8);
	write_cmos_sensor(0x20dc, 0x50e0);
	write_cmos_sensor(0x20de, 0x0020);
	write_cmos_sensor(0x20e0, 0x5100);
	write_cmos_sensor(0x20e2, 0x3143);
	write_cmos_sensor(0x20e4, 0x5121);
	write_cmos_sensor(0x20e6, 0x7800);
	write_cmos_sensor(0x20e8, 0x3140);
	write_cmos_sensor(0x20ea, 0x01c4);
	write_cmos_sensor(0x20ec, 0x01c1);
	write_cmos_sensor(0x20ee, 0x01c0);
	write_cmos_sensor(0x20f0, 0x01c4);
	write_cmos_sensor(0x20f2, 0x2700);
	write_cmos_sensor(0x20f4, 0x3d40);
	write_cmos_sensor(0x20f6, 0x7800);
	write_cmos_sensor(0x20f8, 0xffff);
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
	write_cmos_sensor(0x3068, 0xf800);
	write_cmos_sensor(0x306a, 0xf876);
	write_cmos_sensor(0x006c, 0x0000);
	write_cmos_sensor(0x005e, 0x0200);
	write_cmos_sensor(0x000e, 0x0200);
	write_cmos_sensor(0x0e0a, 0x0001);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0100);
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
	write_cmos_sensor(0x0006, 0x0801);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0a20);
	write_cmos_sensor(0x0a14, 0x0798);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0074, 0x07ff);
	write_cmos_sensor(0x0070, 0x0411);
	write_cmos_sensor(0x0002, 0x0000);
	write_cmos_sensor(0x0a02, 0x0100);
	write_cmos_sensor(0x0a24, 0x0100);
	write_cmos_sensor(0x0076, 0x0000);
	write_cmos_sensor(0x0060, 0x0000);
	write_cmos_sensor(0x0062, 0x0530);
	write_cmos_sensor(0x0064, 0x0500);
	write_cmos_sensor(0x0066, 0x0530);
	write_cmos_sensor(0x0068, 0x0500);
	write_cmos_sensor(0x0122, 0x0300);
	write_cmos_sensor(0x015a, 0xff08);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x005c, 0x0102);
	write_cmos_sensor(0x0a1a, 0x0800);
	write_cmos_sensor(0x0a00, 0x0100);
} /*  sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8259);
	write_cmos_sensor(0x0f30, 0x6425);
	write_cmos_sensor(0x0f32, 0x7167);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0000);
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
	write_cmos_sensor(0x0006, 0x0801);
	write_cmos_sensor(0x0a22, 0x0000);
	write_cmos_sensor(0x0a12, 0x0510);
	write_cmos_sensor(0x0a14, 0x03cc);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090e, 0x0010);
	write_cmos_sensor(0x090c, 0x09c0);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc106);
	write_cmos_sensor(0x0916, 0x040e);
	write_cmos_sensor(0x0918, 0x0304);
	write_cmos_sensor(0x091a, 0x0709);
	write_cmos_sensor(0x091c, 0x0e06);
	write_cmos_sensor(0x091e, 0x0300);
	write_cmos_sensor(0x0a00, 0x0100);
} /*  preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	if (currefps == 300) {
		write_cmos_sensor(0x0a00, 0x0000);
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x6e25);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0000);
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
		write_cmos_sensor(0x0006, 0x0801);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
		write_cmos_sensor(0x0a00, 0x0100);
	} else if (currefps == 150) {
		write_cmos_sensor(0x0a00, 0x0000);
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x6e25);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0000);
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
		write_cmos_sensor(0x0006, 0x1046);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
		write_cmos_sensor(0x0a00, 0x0100);
	} else {
		write_cmos_sensor(0x0a00, 0x0000);
		write_cmos_sensor(0x0b0a, 0x8252);
		write_cmos_sensor(0x0f30, 0x6e25);
		write_cmos_sensor(0x0f32, 0x7067);
		write_cmos_sensor(0x004a, 0x0100);
		write_cmos_sensor(0x004c, 0x0000);
		write_cmos_sensor(0x004e, 0x0000);
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
		write_cmos_sensor(0x0006, 0x0801);
		write_cmos_sensor(0x0a22, 0x0000);
		write_cmos_sensor(0x0a12, 0x0a20);
		write_cmos_sensor(0x0a14, 0x0798);
		write_cmos_sensor(0x003e, 0x0000);
		write_cmos_sensor(0x0804, 0x0200);
		write_cmos_sensor(0x0a04, 0x014a);
		write_cmos_sensor(0x090c, 0x0fdc);
		write_cmos_sensor(0x090e, 0x002d);
		write_cmos_sensor(0x0902, 0x4319);
		write_cmos_sensor(0x0914, 0xc10a);
		write_cmos_sensor(0x0916, 0x071f);
		write_cmos_sensor(0x0918, 0x0408);
		write_cmos_sensor(0x091a, 0x0c0d);
		write_cmos_sensor(0x091c, 0x0f09);
		write_cmos_sensor(0x091e, 0x0a00);
		write_cmos_sensor(0x0a00, 0x0100);
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	capture_setting(currefps);
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	write_cmos_sensor(0x0a00, 0x0000);
	write_cmos_sensor(0x0b0a, 0x8252);
	write_cmos_sensor(0x0f30, 0x6e25);
	write_cmos_sensor(0x0f32, 0x7267);
	write_cmos_sensor(0x004a, 0x0100);
	write_cmos_sensor(0x004c, 0x0000);
	write_cmos_sensor(0x004e, 0x0000);
	write_cmos_sensor(0x000c, 0x0022);
	write_cmos_sensor(0x0008, 0x0b00);
	write_cmos_sensor(0x005a, 0x0208);
	write_cmos_sensor(0x0012, 0x0018);
	write_cmos_sensor(0x0018, 0x0a27);
	write_cmos_sensor(0x0022, 0x0008);
	write_cmos_sensor(0x0028, 0x0017);
	write_cmos_sensor(0x0024, 0x002e);
	write_cmos_sensor(0x002a, 0x0033);
	write_cmos_sensor(0x0026, 0x003c);
	write_cmos_sensor(0x002c, 0x07bb);
	write_cmos_sensor(0x002e, 0x1111);
	write_cmos_sensor(0x0030, 0x1111);
	write_cmos_sensor(0x0032, 0x7711);
	write_cmos_sensor(0x0006, 0x0208);
	write_cmos_sensor(0x0a22, 0x0100);
	write_cmos_sensor(0x0a12, 0x0280);
	write_cmos_sensor(0x0a14, 0x01e0);
	write_cmos_sensor(0x003e, 0x0000);
	write_cmos_sensor(0x0804, 0x0200);
	write_cmos_sensor(0x0a04, 0x016a);
	write_cmos_sensor(0x090c, 0x0270);
	write_cmos_sensor(0x090e, 0x000c);
	write_cmos_sensor(0x0902, 0x4319);
	write_cmos_sensor(0x0914, 0xc103);
	write_cmos_sensor(0x0916, 0x0207);
	write_cmos_sensor(0x0918, 0x0302);
	write_cmos_sensor(0x091a, 0x0406);
	write_cmos_sensor(0x091c, 0x0903);
	write_cmos_sensor(0x091e, 0x0300);
	write_cmos_sensor(0x0a00, 0x0100);
}


static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}


static kal_uint32 return_sensor_id(void)
{
	return read_cmos_sensor(0x0f16);
}

/*************************************************************************
* FUNCTION
* get_imgsensor_id
*
* DESCRIPTION
* This function get the sensor ID
*
* PARAMETERS
* *sensorID : return the sensor ID
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
#ifdef CONFIG_MTK_CAM_CAL
#endif
				LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n",
				imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, id: 0x%x, id: 0x%x, imgsensor_info.sensor_id:0x%x.\n",
			imgsensor.i2c_write_id, *sensor_id, imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* open
*
* DESCRIPTION
* This function initialize the registers of CMOS sensor
*
* PARAMETERS
* None
*
* RETURNS
* None
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
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id) {
			break;
		}
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	/* initail sequence write in  */
	sensor_init();
	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
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
} /*  open  */



/*************************************************************************
* FUNCTION
* close
*
* DESCRIPTION
*
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");
	/*No Need to implement this function*/
	return ERROR_NONE;
} /*  close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
* This function start the sensor preview.
*
* PARAMETERS
* *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
* None
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
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	mdelay(10);
#ifdef FANPENGTAO
	int i = 0;

	for (i = 0; i < 10; i++) {
		LOG_INF("delay time = %d, the frame no = %d\n", i * 10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
#endif
	return ERROR_NONE;
} /*  preview   */

/*************************************************************************
* FUNCTION
* capture
*
* DESCRIPTION
* This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
* None
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
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n", imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n", imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n",
		imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	mdelay(10);
#if 0
	if (imgsensor.test_pattern == KAL_TRUE) {
		write_cmos_sensor(0x0a04, 0x0141);
		write_cmos_sensor(0x0200, 0x0001);
		write_cmos_sensor(0x0206, 0x000a);
		write_cmos_sensor(0x0208, 0x0a0a);
		write_cmos_sensor(0x020a, 0x000a);
		write_cmos_sensor(0x020c, 0x0a0a);
	}
#endif
	return ERROR_NONE;
} /* capture() */

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
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	return ERROR_NONE;
} /*  normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}    /*    hs_video   */

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
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* Custom1
*
* DESCRIPTION
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	preview_setting();
	return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}   /*  Custom4   */
static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}   /*  Custom5   */
static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth   = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight  = imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth  = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight   = imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;
	sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;
	sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;
	sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
	return ERROR_NONE;
} /*  get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
							MSDK_SENSOR_INFO_STRUCT *sensor_info,
							MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */
	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
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
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;
#ifdef FPTPDAFSUPPORT
	sensor_info->PDAF_Support = 1;
#else
	sensor_info->PDAF_Support = 0;
#endif
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
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
		imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
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
}    /*    get_info  */


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
		Custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		Custom4(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		Custom5(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
} /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	if (framerate == 0) {
		return ERROR_NONE;
	}
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 296;
	} else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE)) {
		imgsensor.current_fps = 146;
	} else {
		imgsensor.current_fps = framerate;
	}
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) {
		imgsensor.autoflicker_en = KAL_TRUE;
	} else {
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
		(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0) {
			return ERROR_NONE;
		}
		frame_length = imgsensor_info.normal_video.pclk / framerate *
		10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
		(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (framerate == 300) {
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ?
			(frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
		(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
		(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ?
		(frame_length - imgsensor_info.custom1.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate *
		10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ?
		(frame_length - imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate *
		10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ?
		(frame_length - imgsensor_info.custom3.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate *
		10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ?
		(frame_length - imgsensor_info.custom4.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate *
		10 / imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ?
		(frame_length - imgsensor_info.custom5.framelength) : 0;
		if (imgsensor.dummy_line < 0) {
			imgsensor.dummy_line = 0;
		}
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate *
		10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
		(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
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
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = imgsensor_info.custom4.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = imgsensor_info.custom5.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable) {
		write_cmos_sensor(0x0a04, 0x0141);
		write_cmos_sensor(0x0200, 0x0001);
		write_cmos_sensor(0x0206, 0x000a);
		write_cmos_sensor(0x0208, 0x0a0a);
		write_cmos_sensor(0x020a, 0x000a);
		write_cmos_sensor(0x020c, 0x0a0a);
		LOG_INF(">>crc enable>> reg: %d\n", enable);
	} else {
		write_cmos_sensor(0x0a04, 0x0140);
		write_cmos_sensor(0x0200, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
								UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
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
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) *
		(feature_data),
		(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL) * feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
		(UINT16)*feature_data, (UINT16)*(feature_data + 1),
		(UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
		(UINT16)*(feature_data + 1),
		(UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}
	return ERROR_NONE;
}    /*    feature_control()  */

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
	if (pfFunc != NULL) {
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
} /*  HI556_MIPI_RAW_SensorInit */
