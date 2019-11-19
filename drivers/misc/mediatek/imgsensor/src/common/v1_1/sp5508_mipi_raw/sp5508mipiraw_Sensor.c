/*****************************************************************************
 *
 * Filename:
 * ---------
 *     SP5508mipi_Sensor.c
 *
 * Project:
 * --------
 *         ALPS
 *
 * Description:
 * ------------
 *         Source code of Sensor driver
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
/*#include <asm/system.h> */

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "sp5508mipiraw_Sensor.h"
/****************************Modify Following Strings for Debug****************************/
#define PFX "SP5508_camera_sensor"
#define LOG_1 LOG_INF("SP5508,MIPI 2LANE\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

	static imgsensor_info_struct imgsensor_info = {
	.sensor_id = SP5508_SENSOR_ID,

	.checksum_value = 0xf7375923,        /*checksum value for Camera Auto Test */

	.pre = {
		.pclk = 93600000,              /*record different mode's pclk */
		.linelength = 1562,             /*record different mode's linelength */
		.framelength = 1994,            /*record different mode's framelength */
		.startx = 0,                    /*record different mode's startx of grabwindow */
		.starty = 0,                    /*record different mode's starty of grabwindow */
		.grabwindow_width = 2592,        /*record different mode's width of grabwindow */
		.grabwindow_height = 1944,        /*record different mode's height of grabwindow */
		/*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		/*     following for GetDefaultFramerateByScenario()    */
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 93600000,
		.linelength = 1562,
		.framelength = 1994,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 93600000,
		.linelength = 1562,
		.framelength = 1994,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 93600000,
		.linelength = 1562,
		.framelength = 1994,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 93600000,
		.linelength = 1562,
		.framelength = 1994,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		.max_framerate = 300,
	},
		.slim_video = {
		.pclk = 93600000,
		.linelength = 1562,
		.framelength = 1994,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 23,/*unit , ns */
		.max_framerate = 300,
	},
	.margin = 12,            /*sensor framelength & shutter margin */
	.min_shutter = 4,        /*min shutter */
	.max_frame_length = 0x7fff, /*max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0, /*shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2 */
	.ae_sensor_gain_delay_frame = 0,
	/*sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2 */
	.ae_ispGain_delay_frame = 2,/*isp gain delay frame for AE cycle */
	.ihdr_support = 0,      /*1, support; 0,not support */
	.ihdr_le_firstline = 0,  /*1,le first ; 0, se first */
	.sensor_mode_num = 5,      /*support sensor mode num */

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA, /*mclk driving current */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,/*sensor_interface_type */
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,/*sensor output first pixel color  */
	.mclk = 24,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz  */
	.mipi_lane_num = SENSOR_MIPI_2_LANE,/*mipi lane num */
	.i2c_addr_table = {0x78, 0xff}, /*5409H---0x78 /5409---0x7a  zhanghua */
	.i2c_speed = 200,
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,                                /*mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/*IMGSENSOR_MODE enum value,record current sensor mode,such as:
		INIT, Preview, Capture, Video,High Speed Video, Slim Video */
	.shutter = 0x3D0,                                        /*current shutter */
	.gain = 0x100,                                           /*current gain */
	.dummy_pixel = 0,                                        /*current dummypixel */
	.dummy_line = 0,                                        /*current dummyline */
	.current_fps = 300,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	/*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.test_pattern = KAL_FALSE,
	/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id */
	.ihdr_en = 0, /*sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x78,/*record current sensor's i2c write id 5409H---0x78 /5409/5409V---0x7a   zhanghua */
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2608, 1960, 8, 8, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0000, 0000, 2592, 1944},/*Pre 2112*1558*/
	{ 2608, 1960, 8, 8, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0000, 0000, 2592, 1944},/*cap 4206*3128*/
	{ 2608, 1960, 8, 8, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0000, 0000, 2592, 1944},/* video */
	{ 2608, 1960, 8, 8, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0000, 0000, 2592, 1944},/*HighSpeedVideo*/
	{ 2608, 1960, 8, 8, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 0000, 0000, 2592, 1944},/* slim video*/
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	/* guoyongming disable Add this func to set i2c speed by each sensor */
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF)};
	/* guoyongming disable Add this func to set i2c speed by each sensor */
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}
static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/*you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel,
	or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x05, (imgsensor.dummy_line >> 8) & 0xFF);
	write_cmos_sensor(0x06, imgsensor.dummy_line & 0xFF);
	write_cmos_sensor(0x01, 0x01);
}        /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0xfd, 0x00);
	return ((read_cmos_sensor(0x02) << 8) | read_cmos_sensor(0x03));
}
static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
		(frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	/*dummy_line = frame_length - imgsensor.min_frame_length; */
	/*if (dummy_line < 0) */
	/*imgsensor.dummy_line = 0; */
	/*else */
	/*imgsensor.dummy_line = dummy_line; */
	/*imgsensor.frame_length = frame_length + imgsensor.dummy_line; */
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);

	set_dummy();
}        /*        set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	/* Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.frame_length = (imgsensor.frame_length>>1)<<1;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
		(imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	shutter = (shutter>>1)<<1;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor(0x0e, imgsensor.frame_length >> 8);
			write_cmos_sensor(0x0f, imgsensor.frame_length & 0xFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor(0x0e, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0f, imgsensor.frame_length & 0xFF);
	}

	LOG_INF("huazai_shutter = %d == %x\n", shutter, shutter);
	/* Update Shutter */
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x03, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x04, shutter  & 0xFF);
	write_cmos_sensor(0x01, 0x01);
	LOG_INF("huazai_read 0x03 = 0x%x, 0x04 == 0x%x\n", read_cmos_sensor(0x03), read_cmos_sensor(0x04));
	/*LOG_INF("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length); */
} /*write_shutter  */


/*************************************************************************
 * FUNCTION
 *        set_shutter
 *
 * DESCRIPTION
 *        This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *        iShutter : exposured lines
 *
 * RETURNS
 *        None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	if (shutter < 8)
		shutter = 8;
	else if (shutter > 0xffff)
		shutter = 0xffff;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}

/*************************************************************************
 * FUNCTION
 *        set_gain
 *
 * DESCRIPTION
 *        This function is to set global gain to sensor.
 *
 * PARAMETERS
 *        iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *        the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	if (gain >= BASEGAIN && gain <= 16 * BASEGAIN) {
		reg_gain = 0x10 * gain/BASEGAIN ;        /*change mtk gain base to aptina gain base */

		if (reg_gain <= 0x10) {
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, 0x10);/*0x23 */
			write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP5508MIPI reg_gain =%d, SP5508MIPI Gain = %d", reg_gain, gain);
		} else if (reg_gain >= 0xf8) {
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, 0xf8);
			write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP5508MIPI reg_gain =%d, SP5508MIPI Gain = %d",  reg_gain, gain);
		} else {
			write_cmos_sensor(0xfd, 0x01);
			write_cmos_sensor(0x24, (kal_uint8)reg_gain);
			write_cmos_sensor(0x01, 0x01);
			LOG_INF("SP5508MIPI reg_gain =%d, SP5508MIPI Gain = %d",  reg_gain, gain);
		}
	} else
		LOG_INF("error gain setting");

	return gain;
}        /*        set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	/*not support HDR */
	/*LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain); */
}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
	/*  LOG_INF("image_mirror = %d\n", image_mirror); */

	/********************************************************
	 *

	 *   ISP and Sensor flip or mirror register bit should be the same!!
	 *
	 ********************************************************/
#if 0
	switch (image_mirror) {
	case IMAGE_NORMAL:

		break;
	case IMAGE_H_MIRROR:

		break;
	case IMAGE_V_MIRROR:

		break;
	case IMAGE_HV_MIRROR:

		break;
	default:
		/*          LOG_INF("Error image_mirror setting\n"); */
	}
#endif
}
#endif

/*************************************************************************
 * FUNCTION
 *        night_mode
 *
 * DESCRIPTION
 *        This function night mode of sensor.
 *
 * PARAMETERS
 *        bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *        None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
	/*No Need to implement this function*/
}        /*        night_mode        */

static void sensor_init(void)
{
	write_cmos_sensor(0xfd, 0x00);
	write_cmos_sensor(0x20, 0x00);
	mdelay(5);
	write_cmos_sensor(0xfd, 0x00);
	write_cmos_sensor(0x2e, 0x24);/*pll clk 468M,dac clk 234M,row clk 58.5M, col clk 93.6M */
	write_cmos_sensor(0x2f, 0x01);
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x03, 0x06);/* 30ms */
	write_cmos_sensor(0x04, 0x36);
	write_cmos_sensor(0x06, 0x01);
	write_cmos_sensor(0x0d, 0x00);/* fixed frame rate enable */
	write_cmos_sensor(0x24, 0xff);/* analog gain [7~0] */
	write_cmos_sensor(0x39, 0x0a);
	write_cmos_sensor(0x31, 0x00);/* bit[4]=1 bypassdsp */
	write_cmos_sensor(0x01, 0x01);
	write_cmos_sensor(0x11, 0x60);/* rst_num1 */
	write_cmos_sensor(0x33, 0x90);/* rst_num2 */
	write_cmos_sensor(0x12, 0x03);
	write_cmos_sensor(0x13, 0xc0);/*scn_num_8lsb */
	write_cmos_sensor(0x45, 0x2e);/*scnt_dds_8lsb */
	write_cmos_sensor(0x16, 0xea);
	write_cmos_sensor(0x19, 0xf7); /*icomp1 6u, icomp2 2.35u */
	write_cmos_sensor(0x1a, 0x5f); /*psw */
	write_cmos_sensor(0x1c, 0x0c);
	write_cmos_sensor(0x1d, 0x06);
	write_cmos_sensor(0x1e, 0x09); /*pcp rst on 3.35v,sig_pbias_boost off */
	write_cmos_sensor(0x20, 0x07); /*ncp -0.95v */
	write_cmos_sensor(0x2a, 0x0b); /*adc range 0.7v */
	write_cmos_sensor(0x2c, 0x10); /*bit[4]=1 output high 8bit */
	write_cmos_sensor(0x25, 0x0d); /*sunspot off */
	write_cmos_sensor(0x26, 0x0d); /*pcp tx off,pcp rowsel on 3.35v */
	write_cmos_sensor(0x27, 0x08); /*even/odd mode */
	write_cmos_sensor(0x29, 0x01); /*pldo off */
	write_cmos_sensor(0x55, 0x14);
	write_cmos_sensor(0x56, 0x00);
	write_cmos_sensor(0x57, 0x1d);
	write_cmos_sensor(0x59, 0x00);
	write_cmos_sensor(0x5a, 0x04);
	write_cmos_sensor(0x50, 0x10);
	write_cmos_sensor(0x53, 0x0e);
	write_cmos_sensor(0x6b, 0x10);
	write_cmos_sensor(0x5c, 0x20);
	write_cmos_sensor(0x5d, 0x00); /*p12 */
	write_cmos_sensor(0x5e, 0x06); /*p13   */
	write_cmos_sensor(0x66, 0x38); /*sc1 */
	write_cmos_sensor(0x68, 0x30); /*sc2 */
	write_cmos_sensor(0x71, 0x45); /*rst d0 */
	write_cmos_sensor(0x72, 0x05); /*rst d1  */
	write_cmos_sensor(0x73, 0x44); /*sig d0 */
	write_cmos_sensor(0x81, 0x22); /*p49 */
	write_cmos_sensor(0x8a, 0x66); /*pbias 3.86u */
	write_cmos_sensor(0x8b, 0x66);
	write_cmos_sensor(0xc0, 0x02); /*blc_rpc 0.0625 */
	write_cmos_sensor(0xc1, 0x02);
	write_cmos_sensor(0xc2, 0x02);
	write_cmos_sensor(0xc3, 0x02);
	write_cmos_sensor(0xc4, 0x82);/*blc_exp   */
	write_cmos_sensor(0xc5, 0x82);
	write_cmos_sensor(0xc6, 0x82);
	write_cmos_sensor(0xc7, 0x82);
	write_cmos_sensor(0xfb, 0x43); /*blc on   */
	write_cmos_sensor(0xf0, 0x33);/* offset */
	write_cmos_sensor(0xf1, 0x33);
	write_cmos_sensor(0xf2, 0x33);
	write_cmos_sensor(0xf3, 0x33);
	write_cmos_sensor(0xb1, 0xad); /*mipi_12_sw en,shutdowna=1 */
	write_cmos_sensor(0xb6, 0x42); /*remove hot spot right side     */
	write_cmos_sensor(0xa1, 0x04); /*tx_speed  */
	write_cmos_sensor(0x3f, 0x02); /*vertical flip  */
	write_cmos_sensor(0xa0, 0x01); /*mipi buf en */
	write_cmos_sensor(0xfd, 0x02);
	write_cmos_sensor(0x34, 0xc8);
	write_cmos_sensor(0x60, 0x99);
	write_cmos_sensor(0x93, 0x03);
	write_cmos_sensor(0x18, 0xe0);
	write_cmos_sensor(0x19, 0xa0);
	write_cmos_sensor(0xfd, 0x04);
	write_cmos_sensor(0x31, 0x4b);
	write_cmos_sensor(0x32, 0x4b);
	write_cmos_sensor(0xfd, 0x03);
	write_cmos_sensor(0xc0, 0x01);
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0x8e, 0x0a); /*mipi output size  */
	write_cmos_sensor(0x8f, 0x20);
	write_cmos_sensor(0x90, 0x07);
	write_cmos_sensor(0x91, 0x98);
	write_cmos_sensor(0xfd, 0x02);
	write_cmos_sensor(0xa0, 0x00);
	write_cmos_sensor(0xa1, 0x08);
	write_cmos_sensor(0xa2, 0x07);
	write_cmos_sensor(0xa3, 0x98);
	write_cmos_sensor(0xa4, 0x00);
	write_cmos_sensor(0xa5, 0x08);
	write_cmos_sensor(0xa6, 0x0a);
	write_cmos_sensor(0xa7, 0x20);
	write_cmos_sensor(0xfd, 0x01);
}    /*    MIPI_sensor_Init  */
#if 0
static void preview_setting(void)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0xa0, 0x01);
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0xa0, 0x01);
}    /*    capture_setting  */

static void normal_video_setting(kal_uint16 currefps)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0xa0, 0x01);
}    /*    preview_setting  */

static void video_1080p_setting(void)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0xa0, 0x01);
}    /*    preview_setting  */

static void video_720p_setting(void)
{
	write_cmos_sensor(0xfd, 0x01);
	write_cmos_sensor(0xa0, 0x01);
}    /*    preview_setting  */

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
#endif
/*************************************************************************
 * FUNCTION
 *        get_imgsensor_id
 *
 * DESCRIPTION
 *        This function get the sensor ID
 *
 * PARAMETERS
 *        *sensorID : return the sensor ID
 *
 * RETURNS
 *        None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	LOG_1;
	LOG_INF("PRW 2608*1960@30fps,420Mbps/lane; VID 2608*1960@30fps,420Mbps/lane; CAP 5M@15fps,420Mbps/lane\n");
	/*sensor have two i2c address 0x78 & 0x7a, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("superpix i2c write id:0x%x,sensorid:0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("superpix Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
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
 *        open
 *
 * DESCRIPTION
 *        This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *        None
 *
 * RETURNS
 *        None
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

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("superpix i2c write id:0x%x,sensorid:0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("superpix Read sensor id fail,id:0x%x\n", imgsensor.i2c_write_id, sensor_id);
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
}        /*        open  */

/*************************************************************************
 * FUNCTION
 *        close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *        None
 *
 * RETURNS
 *        None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}        /*        close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *        This function start the sensor preview.
 *
 * PARAMETERS
 *        *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *        None
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
	/*imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	sensor_init();
	/*preview_setting();*/
	/*set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}        /*        preview   */

/*************************************************************************
 * FUNCTION
 *        capture
 *
 * DESCRIPTION
 *        This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *        None
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
	/*PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",
			imgsensor.current_fps, imgsensor_info.cap1.max_framerate / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	sensor_init();
	/*capture_setting(imgsensor.current_fps);*/
	/*set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}        /* capture() */
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
	/*imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	sensor_init();
	/*normal_video_setting(imgsensor.current_fps);*/
	/*set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}        /*        normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	sensor_init();
	/*hs_video_setting();*/
	/*set_mirror_flip(sensor_config_data->SensorImageMirror); */
	return ERROR_NONE;
}        /*        hs_video   */

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
	sensor_init();
	/*slim_video_setting();*/
	/*set_mirror_flip(sensor_config_data->SensorImageMirror); */

	return ERROR_NONE;
}        /*        slim_video         */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth         = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight         = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth         = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight         = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}        /*        get_resolution        */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
							MSDK_SENSOR_INFO_STRUCT *sensor_info,
							MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);


	/*sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; not use */
	/*sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; not use */
	/*imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate;  not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
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

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
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
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;        /* 0 is default 1x */
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
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}        /*        get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}        /* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
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
	if (enable) /*enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
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
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate *
			10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
			(frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
			(frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
			(frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /*coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfd, 0x01);
		write_cmos_sensor(0xb6, 0x01);
	} else {
		write_cmos_sensor(0xfd, 0x01);
		write_cmos_sensor(0xb6, 0x00);
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
	/*unsigned long long *feature_return_para = (unsigned long long *) feature_para; */

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id); */
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
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
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /*for factory mode auto testing */
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
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

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
			(UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		ihdr_write_shutter_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
	default:
		break;
	}

	return ERROR_NONE;
}  /*        feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 SP5508_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
	      *pfFunc = &sensor_func;
	return ERROR_NONE;
}        /*        SP2509MIPISensorInit        */
