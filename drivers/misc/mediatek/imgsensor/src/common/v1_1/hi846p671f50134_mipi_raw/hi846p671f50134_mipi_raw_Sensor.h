/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 HI846 mipi_Sensor.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _HI846MIPI_SENSOR_H
#define _HI846MIPI_SENSOR_H


enum IMGSENSOR_MODE {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;
	kal_uint32 linelength;
	kal_uint32 framelength;

	kal_uint8 startx;
	kal_uint8 starty;

	kal_uint16 grabwindow_width;
	kal_uint16 grabwindow_height;

	/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
	kal_uint8 mipi_data_lp2hs_settle_dc;

	/*	 following for GetDefaultFramerateByScenario()	*/
	kal_uint16 max_framerate;
	kal_uint32 mipi_pixel_rate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
struct imgsensor_struct {
	kal_uint8 mirror;

	kal_uint8 sensor_mode;

	kal_uint32 shutter;
	kal_uint16 gain;

	kal_uint32 pclk;

	kal_uint32 frame_length;
	kal_uint32 line_length;

	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel;
	kal_uint16 dummy_line;

	kal_uint16 current_fps;
	kal_bool   autoflicker_en;
	kal_bool test_pattern;
	kal_bool enable_secure;	/* run as secure driver or not */
	MSDK_SCENARIO_ID_ENUM current_scenario_id;
	kal_uint8  ihdr_en;

	kal_uint8 i2c_write_id;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
struct imgsensor_info_struct {
	kal_uint16 sensor_id;
	#ifdef VENDOR_EDIT
	/*Zhenagjiang.zhu@camera.drv 2017/10/18,modify for different module*/
	kal_uint16 module_id;
	#endif
	kal_uint32 checksum_value;
	struct imgsensor_mode_struct pre;	/* preview scenario relative information */
	struct imgsensor_mode_struct cap;	/* capture scenario relative information */
	struct imgsensor_mode_struct cap1;	/* capture for PIP 15fps relative information*/
	struct imgsensor_mode_struct normal_video;	/* normal video  scenario relative information */
	struct imgsensor_mode_struct hs_video;	/* high speed video scenario relative information */
	struct imgsensor_mode_struct slim_video;	/* slim video for VT scenario relative information */

	kal_uint8  ae_shut_delay_frame;
	kal_uint8  ae_sensor_gain_delay_frame;
	kal_uint8  ae_ispGain_delay_frame;
	kal_uint8  ihdr_support;
	kal_uint8  ihdr_le_firstline;
	kal_uint8  sensor_mode_num;

	kal_uint8  cap_delay_frame;
	kal_uint8  pre_delay_frame;
	kal_uint8  video_delay_frame;
	kal_uint8  hs_video_delay_frame;
	kal_uint8  slim_video_delay_frame;

	kal_uint8  margin;
	kal_uint32 min_shutter;
	kal_uint32 max_frame_length;

	kal_uint8  isp_driving_current;
	kal_uint8  sensor_interface_type;
	kal_uint8  mipi_sensor_type;
	kal_uint8  mipi_settle_delay_mode;
	kal_uint8  sensor_output_dataformat;
	kal_uint8  mclk;

	kal_uint8  mipi_lane_num;
	kal_uint8  i2c_addr_table[5];
	kal_uint32  i2c_speed;
};


extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);


#endif
