#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4h8mipi_Sensor.h"
#define S5K4H8_DEBUG
#ifdef S5K4H8_DEBUG
#define PFX "s5k4h8mipi_otp"
#define LOG_INF(format, args...)  pr_debug(PFX "%s: " format, __func__, ##args)
#else
#define SENSORDB(fmt, arg...)
#endif

#define s5k4h8MIPI_WRITE_ID   0x5a
#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)
#define TRULY_ID           0x02
#define LARGAN_LENS        0x01
#define DONGWOON           0x01
#define TDK_VCM        0x01
#define VALID_OTP          0x01
#define WB_OFFSET          0x1C
#define GAIN_DEFAULT       0x0100
#define Golden_RG   0x286
#define Golden_BG   0x258

USHORT Current_RG;
USHORT Current_BG;


static kal_uint32 r_ratio;
static kal_uint32 b_ratio;

kal_uint16 s5k4h8_read_cmos_sensor(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(400);
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, s5k4h8MIPI_WRITE_ID);
	return get_byte;
}

void s5k4h8_write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF),
	(char)(para >> 8), (char)(para & 0xFF)};

	kdSetI2CSpeed(400);
	iWriteRegI2C(pusendcmd, 4, s5k4h8MIPI_WRITE_ID);
}

void s5k4h8_write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	kdSetI2CSpeed(400);
	iWriteRegI2C(pusendcmd, 3, s5k4h8MIPI_WRITE_ID);
}

/*************************************************************************************************
 * Function    :  start_read_otp
 * Description :  before read otp , set the reading block setting
 * Parameters  :  void
 * Return      :  void
 **************************************************************************************************/
void start_read_otp(void)
{
	LOG_INF(" start_read_otp\n");
	s5k4h8_write_cmos_sensor_8(0x0100, 0x01);
	s5k4h8_write_cmos_sensor_8(0x0A02, 0x0F);
	s5k4h8_write_cmos_sensor_8(0x0A00, 0x01);
	Sleep(20);
}

/*************************************************************************************************
 * Function    :  stop_read_otp
 * Description :  after read otp , stop and reset otp block setting
 **************************************************************************************************/
void stop_read_otp(void)
{
	s5k4h8_write_cmos_sensor_8(0x0A00, 0x00);
}


/*************************************************************************************************
 * Function    :  get_otp_flag
 * Description :  get otp WRITTEN_FLAG
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE], if 1 , this type has valid otp data, otherwise, invalid otp data
 **************************************************************************************************/
BYTE get_otp_flag(BYTE zone)
{
	BYTE flag = 0;

	start_read_otp();
	if (zone == 1) {
		flag = s5k4h8_read_cmos_sensor(0x0A04);
	}
	if (zone == 2) {
		flag = s5k4h8_read_cmos_sensor(0x0A1A);
	}
	LOG_INF("get_otp_flag: flag=%d\n", flag);
	if ((flag & 0xFF) == 0x00) {
		return 0;
	} else if ((flag & 0xFF) == 0x01) {
		return 1;
	} else if ((flag & 0xFF) == 0x03) {
		return 2;
	} else {
		return 0;
	}
}

/*************************************************************************************************
 * Function    :  get_otp_date
 * Description :  get otp date value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 **************************************************************************************************/
#if 0
bool get_otp_date(BYTE zone)
{
	BYTE year  = 0;
	BYTE month = 0;
	BYTE day   = 0;

	start_read_otp();
	year  = s5k4h8_read_cmos_sensor(0x0A06 + (zone * WB_OFFSET));
	month = s5k4h8_read_cmos_sensor(0x0A07 + (zone * WB_OFFSET));
	day   = s5k4h8_read_cmos_sensor(0x0A08 + (zone * WB_OFFSET));
	stop_read_otp();
	LOG_INF("OTP date=%02d.%02d.%02d", year, month, day);
	return 1;
}
#endif


/*************************************************************************************************
 * Function    :  get_otp_module_id
 * Description :  get otp MID value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail
 other value : module ID data , TRULY ID is 0x0002
 **************************************************************************************************/
#if 0
BYTE get_otp_module_id(BYTE zone)
{
	BYTE module_id = 0;

	start_read_otp();
	module_id  = s5k4h8_read_cmos_sensor(0x0A05 + (zone * WB_OFFSET));
	stop_read_otp();
	LOG_INF("OTP_Module ID: 0x%02x.\n", module_id);
	return module_id;
}
#endif


/*************************************************************************************************
 * Function    :  get_otp_lens_id
 * Description :  get otp LENS_ID value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail
 other value : LENS ID data
 **************************************************************************************************/
#if 0
BYTE get_otp_lens_id(BYTE zone)
{
	BYTE lens_id = 0;

	start_read_otp();
	lens_id  = s5k4h8_read_cmos_sensor(0x0A09 + (zone * WB_OFFSET));
	stop_read_otp();
	LOG_INF("OTP_Lens ID: 0x%02x.\n", lens_id);
	return lens_id;
}
#endif


/*************************************************************************************************
 * Function    :  get_otp_vcm_id
 * Description :  get otp VCM_ID value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail
 other value : VCM ID data
 **************************************************************************************************/
#if 0
BYTE get_otp_vcm_id(BYTE zone)
{
	BYTE vcm_id = 0;

	start_read_otp();
	vcm_id = s5k4h8_read_cmos_sensor(0x0A0A + (zone * WB_OFFSET));
	stop_read_otp();
	LOG_INF("OTP_VCM ID: 0x%02x.\n", vcm_id);
	return vcm_id;
}
#endif


/*************************************************************************************************
 * Function    :  get_otp_driver_id
 * Description :  get otp driver id value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail
 other value : driver ID data
 **************************************************************************************************/
#if 0
BYTE get_otp_driver_id(BYTE zone)
{
	BYTE driver_id = 0;

	start_read_otp();
	driver_id = s5k4h8_read_cmos_sensor(0x0A0B + (zone * WB_OFFSET));
	stop_read_otp();
	LOG_INF("OTP_Driver ID: 0x%02x.\n", driver_id);
	return driver_id;
}
#endif

/*************************************************************************************************
 * Function    :  get_light_id
 * Description :  get otp environment light temperature value
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail
 other value : driver ID data
BIT0:D65(6500K) EN
BIT1:D50(5100K) EN
BIT2:CWF(4000K) EN
BIT3:A Light(2800K) EN
 **************************************************************************************************/
#if 0
BYTE get_light_id(BYTE zone)
{
	BYTE light_id = 0;

	start_read_otp();
	light_id = s5k4h8_read_cmos_sensor(0x0A0D);
	stop_read_otp();
	LOG_INF("OTP_Light ID: 0x%02x.\n", light_id);
	return light_id;
}
#endif

/*************************************************************************************************
 * Function    :  otp_lenc_update
 * Description :  Update lens correction
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [bool] 0 : OTP data fail
1 : otp_lenc update success
 **************************************************************************************************/
bool otp_lenc_update(USHORT zone)
{
	LOG_INF("otp_lenc_update: flag=%d\n", zone);
	s5k4h8_write_cmos_sensor(0x0B00, 0x0180);
	s5k4h8_write_cmos_sensor(0x6028, 0x2000);
	s5k4h8_write_cmos_sensor(0x602A, 0x0c40);
	s5k4h8_write_cmos_sensor(0x6F12, 0x0140);
	Sleep(10);
	s5k4h8_write_cmos_sensor(0x0100, 0x0100);
	return 1;
}

/*************************************************************************************************
 * Function    :  wb_gain_set
 * Description :  Set WB ratio to register gain setting  512x
 * Parameters  :  [int] r_ratio : R ratio data compared with golden module R
b_ratio : B ratio data compared with golden module B
 * Return      :  [bool] 0 : set wb fail
1 : WB set success
 **************************************************************************************************/

bool wb_gain_set(void)
{
	USHORT R_GAIN;
	USHORT B_GAIN;
	USHORT Gr_GAIN;
	USHORT Gb_GAIN;
	USHORT G_GAIN;
	kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;

	if (!r_ratio || !b_ratio) {
		return 0;
	}
	if (r_ratio >= 512) {
		if (b_ratio >= 512) {
			R_GAIN = (USHORT)(GAIN_DEFAULT * r_ratio / 512);
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT)(GAIN_DEFAULT * b_ratio / 512);
		} else {
			R_GAIN =  (USHORT)(GAIN_DEFAULT * r_ratio / b_ratio);
			G_GAIN = (USHORT)(GAIN_DEFAULT * 512 / b_ratio);
			B_GAIN = GAIN_DEFAULT;
		}
	} else {
		if (b_ratio >= 512) {
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT)(GAIN_DEFAULT * 512 / r_ratio);
			B_GAIN =  (USHORT)(GAIN_DEFAULT * b_ratio / r_ratio);
		} else {
			Gr_GAIN = (USHORT)(GAIN_DEFAULT * 512 / r_ratio);
			Gb_GAIN = (USHORT)(GAIN_DEFAULT * 512 / b_ratio);
			if (Gr_GAIN >= Gb_GAIN) {
				R_GAIN = GAIN_DEFAULT;
				G_GAIN = (USHORT)(GAIN_DEFAULT * 512 / r_ratio);
				B_GAIN =  (USHORT)(GAIN_DEFAULT * b_ratio / r_ratio);
			} else {
				R_GAIN =  (USHORT)(GAIN_DEFAULT * r_ratio  / b_ratio);
				G_GAIN = (USHORT)(GAIN_DEFAULT * 512 / b_ratio);
				B_GAIN = GAIN_DEFAULT;
			}
		}
	}
	R_GainH = (R_GAIN & 0xff00) >> 8;
	R_GainL = (R_GAIN & 0x00ff);
	B_GainH = (B_GAIN & 0xff00) >> 8;
	B_GainL = (B_GAIN & 0x00ff);
	G_GainH = (G_GAIN & 0xff00) >> 8;
	G_GainL = (G_GAIN & 0x00ff);
	LOG_INF("QYC_OTP_golden_rg=%d,golden_bg=%d\n", Golden_RG, Golden_BG);
	LOG_INF("QYC_OTP_current_rg=%d,current_bg=%d\n", Current_RG, Current_BG);
	LOG_INF("QYC_OTP_r_ratio=%d,b_ratio=%d\n", r_ratio, b_ratio);
	s5k4h8_write_cmos_sensor(0x6028, 0x4000);
	s5k4h8_write_cmos_sensor(0x602A, 0x3058);
	s5k4h8_write_cmos_sensor(0x6F12, 0x0001);
	s5k4h8_write_cmos_sensor_8(0x020e, G_GainH);
	s5k4h8_write_cmos_sensor_8(0x020f, G_GainL);
	s5k4h8_write_cmos_sensor_8(0x0210, R_GainH);
	s5k4h8_write_cmos_sensor_8(0x0211, R_GainL);
	s5k4h8_write_cmos_sensor_8(0x0212, B_GainH);
	s5k4h8_write_cmos_sensor_8(0x0213, B_GainL);
	s5k4h8_write_cmos_sensor_8(0x0214, G_GainH);
	s5k4h8_write_cmos_sensor_8(0x0215, G_GainL);
	LOG_INF("QYC_OTP WB Update Finished!\n");
	return 1;
}

/*************************************************************************************************
 * Function    :  get_otp_wb
 * Description :  Get WB data
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
 **************************************************************************************************/
bool get_otp_wb(int zone)
{
	BYTE temph = 0;
	BYTE templ = 0;

	LOG_INF("get_otp_wb: zone=%d\n", zone);
#if 0
	for (i = 0x0A04; i <= 0x0A43; i++) {
		val = s5k4h8_read_cmos_sensor(i);
		LOG_INF("s5k4h8 get_otp_wb: 0x%x=0x%x\n", i, val);
	}
#endif
	if (zone == 1) {
		templ = s5k4h8_read_cmos_sensor(0x0A0C);
		temph = s5k4h8_read_cmos_sensor(0x0A0D);
		Current_RG = (USHORT)((temph << 8) | templ);
		templ = s5k4h8_read_cmos_sensor(0x0A0E);
		temph = s5k4h8_read_cmos_sensor(0x0A0F);
		Current_BG = (USHORT)((temph << 8) | templ);
	}
	if (zone == 2) {
		templ = s5k4h8_read_cmos_sensor(0x0A22);
		temph = s5k4h8_read_cmos_sensor(0x0A23);
		Current_RG = (USHORT)((temph << 8) | templ);
		templ = s5k4h8_read_cmos_sensor(0x0A24);
		temph = s5k4h8_read_cmos_sensor(0x0A25);
		Current_BG = (USHORT)((temph << 8) | templ);
	}
	LOG_INF("Current_RG=0x%x, Current_BG=0x%x\n", Current_RG, Current_BG);
	stop_read_otp();
	return 1;
}


/*************************************************************************************************
 * Function    :  otp_wb_update
 * Description :  Update WB correction
 * Return      :  [bool] 0 : OTP data fail
1 : otp_WB update success
 **************************************************************************************************/
bool otp_wb_update(BYTE zone)
{
	if (!get_otp_wb(zone)) {
		return 0;
	}
	r_ratio = 512 * Golden_RG / Current_RG;
	b_ratio = 512 * Golden_BG / Current_BG;
	wb_gain_set();
	LOG_INF("otp_wb_update finished!\n");
	return 1;
}

/*************************************************************************************************
 * Function    :  otp_update_s5k4h8()
 * Description :  update otp data from otp , it otp data is valid,
 it include get ID and WB update function
 * Return      :  [bool] 0 : update fail
1 : update success
 **************************************************************************************************/
bool otp_update_s5k4h8(void)
{
	BYTE zone = 0x00;
	BYTE FLG = 0x00;
	int i;

	LOG_INF("otp_update_s5k4h8 begain!\n");
	for (i = 0; i < 3; i++) {
		FLG = get_otp_flag(zone);
		if (FLG == VALID_OTP) {
			break;
		}
		zone++;
	}
	if (i == 3) {
		LOG_INF("wgs_Warning: No OTP Data or OTP data is invalid!!");
		return 0;
	}
	/*
		MID = get_otp_module_id(zone);
		LENS_ID = get_otp_lens_id(zone);
		VCM_ID =  get_otp_vcm_id(zone);
		get_otp_date(zone);
		get_otp_driver_id(zone);
			get_light_id(zone);
		if (MID != TRULY_ID)
		{
				return 0;
		}*/
	otp_wb_update(zone);
	otp_lenc_update(zone);
	LOG_INF("otp_update_s5k4h8 end!\n");
	return 1;
}
