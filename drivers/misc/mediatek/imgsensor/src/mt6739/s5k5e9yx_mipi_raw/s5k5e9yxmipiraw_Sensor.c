/* *****************************************************************************
 *
 * Filename:
 * ---------
 *   s5k5e9yxmipi_Sensor.c
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
#include <linux/atomic.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k5e9yxmipiraw_Sensor.h"
/****************************Modify following Strings for debug****************************/
#define PFX "s5k5e9yx_camera_sensor"
#define LOG_1 LOG_INF("s5k5e9yx,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps;video 1280*960;capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(fmt, args...) pr_debug(PFX "[%s] " fmt, __func__, ##args)
static DEFINE_SPINLOCK(imgsensor_drv_lock);


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K5E9YX_SENSOR_ID,

	.checksum_value = 0x2ae69154,

	.pre = {
		.pclk = 190000000,        /* record different mode's pclk */
		.linelength  = 3112,        /* record different mode's linelength */
		.framelength = 2034,      /* record different mode's framelength */
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1304,    /* record different mode's width of grabwindow */
		.grabwindow_height = 980,   /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 190000000,        /* record different mode's pclk */
		.linelength  = 3112,        /* record different mode's linelength */
		.framelength = 2034,      /* record different mode's framelength */
		.startx = 0,          /* record different mode's startx of grabwindow */
		.starty = 0,          /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2608,    /* record different mode's width of grabwindow */
		.grabwindow_height = 1960,    /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 190000000,        /* record different mode's pclk */
		.linelength  = 3112,        /* record different mode's linelength */
		.framelength = 2034,      /* record different mode's framelength */
		.startx = 0,
		.starty = 0,          /* record different mode's starty of grabwindow */
		.grabwindow_width  = 2608,    /* record different mode's width of grabwindow */
		.grabwindow_height = 1960,    /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.hs_video = {/*NULL*/
		.pclk = 560000000,        /* record different mode's pclk */
		.linelength  = 4832,        /* record different mode's linelength */
		.framelength = 965,     /* record different mode's framelength */
		.startx = 0,          /* record different mode's startx of grabwindow */
		.starty = 0,          /* record different mode's starty of grabwindow */
		.grabwindow_width  = 1280,    /* record different mode's width of grabwindow */
		.grabwindow_height = 720,   /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario()  */
		.max_framerate = 1200,
	},
	.slim_video = {/*NULL*/
		.pclk = 560000000,        /* record different mode's pclk */
		.linelength  = 5120,        /* record different mode's linelength */
		.framelength = 3644,      /* record different mode's framelength */
		.startx = 0,          /* record different mode's startx of grabwindow */
		.starty = 0,          /* record different mode's starty of grabwindow */
		.grabwindow_width  = 1920,    /* record different mode's width of grabwindow */
		.grabwindow_height = 1080,    /* record different mode's height of grabwindow */
		/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario */
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario()  */
		.max_framerate = 300,
	},
	.margin = 6,
	.min_shutter = 5,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 7,

	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,

	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x5a, 0xff},
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 0,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x5a,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2608, 1960,    0,  0, 2608, 1960, 1304,  980, 0000, 0000, 1304,  980,    0,  0, 1304,  980},
	{ 2608, 1960,    0,  0, 2608, 1960, 2608, 1960, 0000, 0000, 2608, 1960,    0,  0, 2608, 1960},
	{ 2608, 1960,    0,    0, 2608, 1960, 2608, 1960, 0000, 0000, 2608, 1960,    0,  0, 2608, 1960},
	{ 2592, 1944,    24, 260, 2560, 1440,  640,  480, 0000, 0000,  640,  480,    0,  0,  640,  480},
	{ 2592, 1944,    24,  20, 2560, 1920, 1280,  720, 0000, 0000, 1280,  720,    0,  0, 1280,  720},
};


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
	(char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
	(char)(para >> 8), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n",
	imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);
} /*  set_dummy  */

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}


static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable? %d\n",
	framerate, min_framelength_en);
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


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

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
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	}
	write_cmos_sensor_8(0x0202, shutter >> 8);
	write_cmos_sensor_8(0x0203, shutter & 0xFF);
	LOG_INF("shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
} /*  write_shutter  */



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

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	write_shutter(shutter);
} /*  set_shutter */

#if 0
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to
	get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode,
	thus extra exposure lines must be updated here. */
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;
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
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	}
	write_cmos_sensor_8(0x0202, shutter >> 8);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	LOG_INF("Add for N3D! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}

#endif

/*
static kal_uint16 gain2reg(const kal_uint16 gain)
{
	return gain>>1;
}
*/
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

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X  */
	/* [4:9] = M meams M X     */
	/* Total gain = M + N /16 X   */
	reg_gain = gain >> 1;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor_8(0x0204, reg_gain >> 8);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
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
		write_cmos_sensor(0x3508, (se << 4) & 0xFF);
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}



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
		write_cmos_sensor_8(0x0101, 0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_8(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_8(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8(0x0101, 0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}
}

static void check_stremoff(void)
{
	unsigned int i = 0, framecnt = 0;

	for (i = 0; i < 100; i++) {
		framecnt = read_cmos_sensor(0x0005);
		if (framecnt == 0xFF) {
			break;
		}
	}
	LOG_INF(" Stream Off Fail!\n");
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
/*
static void night_mode(kal_bool enable)
{

}night_mode */

static void sensor_init(void)
{
	LOG_INF("E\n");
	write_cmos_sensor_8(0x0100, 0x00);
	write_cmos_sensor_8(0x3B45, 0x01);
	write_cmos_sensor_8(0x0B05, 0x01);
	write_cmos_sensor_8(0x392F, 0x01);
	write_cmos_sensor_8(0x3930, 0x00);
	write_cmos_sensor_8(0x3924, 0x7F);
	write_cmos_sensor_8(0x3925, 0xFD);
	write_cmos_sensor_8(0x3C08, 0xFF);
	write_cmos_sensor_8(0x3C09, 0xFF);
	write_cmos_sensor_8(0x3C31, 0xFF);
	write_cmos_sensor_8(0x3C32, 0xFF);
	write_cmos_sensor_8(0x3290, 0x10);
	write_cmos_sensor_8(0x3200, 0x01);
	write_cmos_sensor_8(0x3074, 0x06);
	write_cmos_sensor_8(0x3075, 0x2F);
	write_cmos_sensor_8(0x308A, 0x20);
	write_cmos_sensor_8(0x308B, 0x08);
	write_cmos_sensor_8(0x308C, 0x0B);
	write_cmos_sensor_8(0x3081, 0x07);
	write_cmos_sensor_8(0x307B, 0x85);
	write_cmos_sensor_8(0x307A, 0x0A);
	write_cmos_sensor_8(0x3079, 0x0A);
	write_cmos_sensor_8(0x306E, 0x71);
	write_cmos_sensor_8(0x306F, 0x28);
	write_cmos_sensor_8(0x301F, 0x20);
	write_cmos_sensor_8(0x3012, 0x4E);
	write_cmos_sensor_8(0x306B, 0x9A);
	write_cmos_sensor_8(0x3091, 0x16);
	write_cmos_sensor_8(0x30C4, 0x06);
	write_cmos_sensor_8(0x306A, 0x79);
	write_cmos_sensor_8(0x30B0, 0xFF);
	write_cmos_sensor_8(0x306D, 0x08);
	write_cmos_sensor_8(0x3084, 0x16);
	write_cmos_sensor_8(0x3070, 0x0F);
	write_cmos_sensor_8(0x30C2, 0x05);
	write_cmos_sensor_8(0x3069, 0x87);
	write_cmos_sensor_8(0x3C0F, 0x00);
	write_cmos_sensor_8(0x0A02, 0x3F);
	write_cmos_sensor_8(0x3083, 0x14);
	write_cmos_sensor_8(0x3080, 0x08);
	write_cmos_sensor_8(0x3C34, 0xEA);
	write_cmos_sensor_8(0x3C35, 0x5C);
} /*  sensor_init  */

static void preview_setting(void)
{
	write_cmos_sensor_8(0x0100, 0x00);
	mdelay(3);
	check_stremoff();
	write_cmos_sensor_8(0x0136, 0x18);
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0x5F);
	write_cmos_sensor_8(0x030D, 0x04);
	write_cmos_sensor_8(0x030E, 0x00);
	write_cmos_sensor_8(0x030F, 0x92);
	write_cmos_sensor_8(0x3C1F, 0x00);
	write_cmos_sensor_8(0x3C17, 0x00);
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x01);
	write_cmos_sensor_8(0x0820, 0x03);
	write_cmos_sensor_8(0x0821, 0x6C);
	write_cmos_sensor_8(0x0822, 0x00);
	write_cmos_sensor_8(0x0823, 0x00);
	write_cmos_sensor_8(0x3929, 0x0F);
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x0A);
	write_cmos_sensor_8(0x0349, 0x2F);
	write_cmos_sensor_8(0x034A, 0x07);
	write_cmos_sensor_8(0x034B, 0xA7);
	write_cmos_sensor_8(0x034C, 0x05);
	write_cmos_sensor_8(0x034D, 0x18);
	write_cmos_sensor_8(0x034E, 0x03);
	write_cmos_sensor_8(0x034F, 0xD4);
	write_cmos_sensor_8(0x0900, 0x01);
	write_cmos_sensor_8(0x0901, 0x22);
	write_cmos_sensor_8(0x0381, 0x01);
	write_cmos_sensor_8(0x0383, 0x01);
	write_cmos_sensor_8(0x0385, 0x01);
	write_cmos_sensor_8(0x0387, 0x03);
	write_cmos_sensor_8(0x0101, 0x00);
	write_cmos_sensor_8(0x0340, 0x07);
	write_cmos_sensor_8(0x0341, 0xF2);
	write_cmos_sensor_8(0x0342, 0x0C);
	write_cmos_sensor_8(0x0343, 0x28);
	write_cmos_sensor_8(0x0200, 0x0B);
	write_cmos_sensor_8(0x0201, 0x9C);
	write_cmos_sensor_8(0x0202, 0x00);
	write_cmos_sensor_8(0x0203, 0x02);
	write_cmos_sensor_8(0x30B8, 0x2A);
	write_cmos_sensor_8(0x30BA, 0x2E);
	write_cmos_sensor_8(0x3931, 0x02);
	write_cmos_sensor_8(0x0B05, 0x01);
	write_cmos_sensor_8(0x0100, 0x01);
} /*  preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_cmos_sensor_8(0x0100, 0x00);
	mDELAY(3);
	check_stremoff();
	write_cmos_sensor_8(0x0136, 0x18);
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0x5F);
	write_cmos_sensor_8(0x030D, 0x04);
	write_cmos_sensor_8(0x030E, 0x00);
	write_cmos_sensor_8(0x030F, 0x92);
	write_cmos_sensor_8(0x3C1F, 0x00);
	write_cmos_sensor_8(0x3C17, 0x00);
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x01);
	write_cmos_sensor_8(0x0820, 0x03);
	write_cmos_sensor_8(0x0821, 0x6C);
	write_cmos_sensor_8(0x0822, 0x00);
	write_cmos_sensor_8(0x0823, 0x00);
	write_cmos_sensor_8(0x3929, 0x0F);
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x0A);
	write_cmos_sensor_8(0x0349, 0x2F);
	write_cmos_sensor_8(0x034A, 0x07);
	write_cmos_sensor_8(0x034B, 0xA7);
	write_cmos_sensor_8(0x034C, 0x0A);
	write_cmos_sensor_8(0x034D, 0x30);
	write_cmos_sensor_8(0x034E, 0x07);
	write_cmos_sensor_8(0x034F, 0xA8);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x00);
	write_cmos_sensor_8(0x0381, 0x01);
	write_cmos_sensor_8(0x0383, 0x01);
	write_cmos_sensor_8(0x0385, 0x01);
	write_cmos_sensor_8(0x0387, 0x01);
	write_cmos_sensor_8(0x0101, 0x00);
	write_cmos_sensor_8(0x0340, 0x07);
	write_cmos_sensor_8(0x0341, 0xF2);
	write_cmos_sensor_8(0x0342, 0x0C);
	write_cmos_sensor_8(0x0343, 0x28);
	write_cmos_sensor_8(0x0200, 0x0B);
	write_cmos_sensor_8(0x0201, 0x9C);
	write_cmos_sensor_8(0x0202, 0x00);
	write_cmos_sensor_8(0x0203, 0x02);
	write_cmos_sensor_8(0x30B8, 0x2E);
	write_cmos_sensor_8(0x30BA, 0x36);
	write_cmos_sensor_8(0x3931, 0x02);
	write_cmos_sensor_8(0x0B05, 0x01);
	write_cmos_sensor_8(0x0100, 0x01);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
	write_cmos_sensor_8(0x0100, 0x00);
	mDELAY(3);
	check_stremoff();
	write_cmos_sensor_8(0x0136, 0x18);
	write_cmos_sensor_8(0x0137, 0x00);
	write_cmos_sensor_8(0x0305, 0x04);
	write_cmos_sensor_8(0x0306, 0x00);
	write_cmos_sensor_8(0x0307, 0x5F);
	write_cmos_sensor_8(0x030D, 0x04);
	write_cmos_sensor_8(0x030E, 0x00);
	write_cmos_sensor_8(0x030F, 0x92);
	write_cmos_sensor_8(0x3C1F, 0x00);
	write_cmos_sensor_8(0x3C17, 0x00);
	write_cmos_sensor_8(0x0112, 0x0A);
	write_cmos_sensor_8(0x0113, 0x0A);
	write_cmos_sensor_8(0x0114, 0x01);
	write_cmos_sensor_8(0x0820, 0x03);
	write_cmos_sensor_8(0x0821, 0x6C);
	write_cmos_sensor_8(0x0822, 0x00);
	write_cmos_sensor_8(0x0823, 0x00);
	write_cmos_sensor_8(0x3929, 0x0F);
	write_cmos_sensor_8(0x0344, 0x00);
	write_cmos_sensor_8(0x0345, 0x00);
	write_cmos_sensor_8(0x0346, 0x00);
	write_cmos_sensor_8(0x0347, 0x00);
	write_cmos_sensor_8(0x0348, 0x0A);
	write_cmos_sensor_8(0x0349, 0x2F);
	write_cmos_sensor_8(0x034A, 0x07);
	write_cmos_sensor_8(0x034B, 0xA7);
	write_cmos_sensor_8(0x034C, 0x0A);
	write_cmos_sensor_8(0x034D, 0x30);
	write_cmos_sensor_8(0x034E, 0x07);
	write_cmos_sensor_8(0x034F, 0xA8);
	write_cmos_sensor_8(0x0900, 0x00);
	write_cmos_sensor_8(0x0901, 0x00);
	write_cmos_sensor_8(0x0381, 0x01);
	write_cmos_sensor_8(0x0383, 0x01);
	write_cmos_sensor_8(0x0385, 0x01);
	write_cmos_sensor_8(0x0387, 0x01);
	write_cmos_sensor_8(0x0101, 0x00);
	write_cmos_sensor_8(0x0340, 0x07);
	write_cmos_sensor_8(0x0341, 0xF2);
	write_cmos_sensor_8(0x0342, 0x0C);
	write_cmos_sensor_8(0x0343, 0x28);
	write_cmos_sensor_8(0x0200, 0x0B);
	write_cmos_sensor_8(0x0201, 0x9C);
	write_cmos_sensor_8(0x0202, 0x00);
	write_cmos_sensor_8(0x0203, 0x02);
	write_cmos_sensor_8(0x30B8, 0x2E);
	write_cmos_sensor_8(0x30BA, 0x36);
	write_cmos_sensor_8(0x3931, 0x02);
	write_cmos_sensor_8(0x0B05, 0x01);
	write_cmos_sensor_8(0x0100, 0x01);
}
static void hs_video_setting(void)
{
}

static void slim_video_setting(void)
{
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
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail,i2c_write_id 0x%x id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
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
	LOG_2;
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
			LOG_INF("Read sensor id fail,i2c_write_id 0x%x id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
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
	set_mirror_flip(imgsensor.mirror);
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
	/* imgsensor.current_fps = 240; */
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
	/* PIP capture:15fps */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {    /* PIP capture: 24fps */
		imgsensor.pclk = imgsensor_info.cap2.pclk;
		imgsensor.line_length = imgsensor_info.cap2.linelength;
		imgsensor.frame_length = imgsensor_info.cap2.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
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
	set_mirror_flip(imgsensor.mirror);
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
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
} /*  hs_video   */

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
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
} /*  slim_video   */



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
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
} /*  get_info  */


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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
} /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n", framerate);
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
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
							? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0) {
			return ERROR_NONE;
		}
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
						imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
						? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength)
				? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else if (imgsensor.current_fps == imgsensor_info.cap2.max_framerate) {
			frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength)
			? (frame_length - imgsensor_info.cap2.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
				LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate, imgsensor_info.cap.max_framerate / 10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
					? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate *
		10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate *
		10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
			(frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
		break;
	default:    /* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate *
		10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
				(frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		/* set_dummy(); */
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
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable) {
		write_cmos_sensor_8(0x0601, 0x02);
	} else {
		write_cmos_sensor_8(0x0601, 0x00);
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
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
	(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d", feature_id);
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
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		if ((sensor_reg_data->RegData >> 8) > 0) {
			write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		} else {
			write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		}
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
		set_auto_flicker_mode((BOOL) (*feature_data_16), *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) (*feature_data),
									*(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM) (*feature_data),
		(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
	/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%llu\n", *feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		/* LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16); */
		LOG_INF("Warning! Not Support IHDR Feature");
		spin_lock(&imgsensor_drv_lock);
		/* imgsensor.ihdr_en = (BOOL)*feature_data_16; */
		imgsensor.ihdr_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%llu\n", *feature_data);
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
						 sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
		(UINT16)*feature_data, (UINT16)*(feature_data + 1),
		(UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
		(UINT16)*(feature_data + 1), (UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}
	return ERROR_NONE;
} /*  feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K5E9YX_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL) {
		*pfFunc = &sensor_func;
	}
	return ERROR_NONE;
} /*  S5K5E9YX_MIPI_RAW_SensorInit*/
