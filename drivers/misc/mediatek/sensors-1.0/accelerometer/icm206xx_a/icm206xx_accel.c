/*
 * ICM206XX sensor driver
 * Copyright (C) 2016 Invensense, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/ioctl.h>
#include <linux/proc_fs.h>
#include "hwmsensor.h"
#include "cust_acc.h"
#include "accel.h"
#include "icm206xx_register_20608D.h"
#include "icm206xx_share.h"
#include "icm206xx_accel.h"
#include "hwmsen_helper.h"

#define ICM206XX_ACCEL_DEV_NAME	"ICM206XX_ACCEL"

/*------for acc calibration------------------start---*/
#define ICM206XX_CALI_LEN       (10)
#define ICM206XX_CALI_TOLERANCE (1000)
static struct proc_dir_entry *acc_calibrate_proc_file = NULL;
u64 set_delay_ns = 0;
/*------for acc calibration------------------end----*/

struct icm206xx_accel_i2c_data {
	struct i2c_client *client;
	struct acc_hw hw;
	struct hwmsen_convert cvt;

	/*misc */
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;

	/*data */
	s16 cali_sw[ICM206XX_AXIS_NUM + 1];
	s16 data[ICM206XX_AXIS_NUM + 1];

	u8 offset[ICM206XX_AXIS_NUM + 1];
	bool flush;
};

static int icm206xx_accel_init_flag = -1;

static int icm206xx_accel_i2c_suspend(struct device *dev);
static int icm206xx_accel_i2c_resume(struct device *dev);
struct i2c_client *icm206xx_accel_i2c_client = NULL;
static struct icm206xx_accel_i2c_data *obj_i2c_data;

#ifdef ICM206XX_SELFTEST
static char selftestRes[8] = { 0 };
#endif
bool power_acc = false;

static int g_icm206xx_accel_sensitivity = ICM206XX_ACCEL_DEFAULT_SENSITIVITY;	/* +/-4G as Default */

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops icm26xx_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(icm206xx_accel_i2c_suspend, icm206xx_accel_i2c_resume)
};
#endif
static int icm206xx_accel_local_init(void);
static int icm206xx_accel_remove(void);
static struct acc_init_info icm206xx_accel_init_info = {
	.name = ICM206XX_ACCEL_DEV_NAME,
	.init = icm206xx_accel_local_init,
	.uninit = icm206xx_accel_remove,
};

/*=======================================================================================*/
/* Vendor Specific Functions Section                                                     */
/*=======================================================================================*/

static int icm206xx_accel_SetFullScale(struct i2c_client *client, u8 accel_fsr)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	if (accel_fsr < ICM206XX_ACCEL_RANGE_2G || accel_fsr > ICM206XX_ACCEL_RANGE_16G) {
		return ICM206XX_ERR_INVALID_PARAM;
	}

	res = icm206xx_share_i2c_read_register(ICM206XX_REG_ACCEL_CFG, databuf, 1);
	if (res < 0) {
		ACC_PR_ERR("read fsr register err!\n");
		return ICM206XX_ERR_I2C;
	}

	databuf[0] &= ~ICM206XX_BIT_ACCEL_FSR;	/* clear FSR bit */
	databuf[0] |= accel_fsr << ICM206XX_ACCEL_FS_SEL;

	g_icm206xx_accel_sensitivity = ICM206XX_ACCEL_MAX_SENSITIVITY >> accel_fsr;

	res = icm206xx_share_i2c_write_register(ICM206XX_REG_ACCEL_CFG, databuf, 1);
	if (res < 0) {
		ACC_PR_ERR("write fsr register err!\n");
		return ICM206XX_ERR_I2C;
	}

	return ICM206XX_SUCCESS;
}

static int icm206xx_accel_SetFilter(struct i2c_client *client, u8 accel_filter)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	if (accel_filter < ICM206XX_ACCEL_DLPF_0 || accel_filter > ICM206XX_ACCEL_DLPF_7) {
		return ICM206XX_ERR_INVALID_PARAM;
	}

	res = icm206xx_share_i2c_read_register(ICM206XX_REG_ACCEL_CFG2, databuf, 1);
	if (res < 0) {
		ACC_PR_ERR("read filter register err!\n");
		return ICM206XX_ERR_I2C;
	}

	databuf[0] &= ~ICM206XX_BIT_ACCEL_FILTER;	/* clear Filter bit */
	databuf[0] |= accel_filter;

	res = icm206xx_share_i2c_write_register(ICM206XX_REG_ACCEL_CFG2, databuf, 1);
	res = 0;		/* Taly!!! : Remove this line on actual platform */
	/* i2c write return fail on MTK evb, but value is written properly */
	if (res < 0) {
		ACC_PR_ERR("write filter register err!\n");
		return ICM206XX_ERR_I2C;
	}

	return ICM206XX_SUCCESS;
}

static int icm206xx_accel_ReadSensorDataDirect(struct i2c_client *client, s16 data[ICM206XX_AXIS_NUM])
{
	char databuf[6];
	int i;
	int res = 0;

	if (client == NULL)
		return ICM206XX_ERR_INVALID_PARAM;

	res = icm206xx_share_i2c_read_register(ICM206XX_REG_ACCEL_XH, databuf, ICM206XX_DATA_LEN);
	if (res < 0) {
		ACC_PR_ERR("read accelerometer data error\n");
		return ICM206XX_ERR_I2C;
	}

	for (i = 0; i < ICM206XX_AXIS_NUM; i++)
		data[i] = ((s16) ((databuf[i * 2] << 8) | (databuf[i * 2 + 1])));	/* convert 8-bit to 16-bit */

	return ICM206XX_SUCCESS;
}

static int icm206xx_accel_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	char databuf[6];
	int data[3];
	int res = 0;
	int i = 0;
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);

	if (client == NULL)
		return ICM206XX_ERR_INVALID_PARAM;

	res = icm206xx_share_i2c_read_register(ICM206XX_REG_ACCEL_XH, databuf, ICM206XX_DATA_LEN);
	if (res < 0) {
		ACC_PR_ERR("read accelerometer data error\n");
		return ICM206XX_ERR_I2C;
	}

	/* convert 8-bit to 16-bit */
	/* This part must be done completely before any translation (Orientation Translation, Unit Translation) */
	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		obj->data[i] = ((s16) ((databuf[i * 2] << 8) | (databuf[i * 2 + 1])));

		/* Add Cali */
		obj->data[i] += obj->cali_sw[i];
	}

	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		/* Orientation Translation Lower (sensor) --> Upper (Device) */
		data[i] = obj->cvt.sign[i] * obj->data[obj->cvt.map[i]];

		/* Unit Translation */
		data[i] = data[i] * GRAVITY_EARTH_1000 / g_icm206xx_accel_sensitivity;
	}

	snprintf(buf, PAGE_SIZE, "%04x %04x %04x", data[ICM206XX_AXIS_X], data[ICM206XX_AXIS_Y], data[ICM206XX_AXIS_Z]);

	if (atomic_read(&obj->trace)) {
		ACC_LOG("Accelerometer Data - %04x %04x %04x\n",
				data[ICM206XX_AXIS_X], data[ICM206XX_AXIS_Y], data[ICM206XX_AXIS_Z]);
	}

	return ICM206XX_SUCCESS;
}


static int icm206xx_accel_ReadRawData(struct i2c_client *client, char *buf)
{
	int res = 0;
	s16 data[ICM206XX_AXIS_NUM] = { 0, 0, 0 };

	res = icm206xx_accel_ReadSensorDataDirect(client, data);
	if (res < 0) {
		ACC_PR_ERR("read accelerometer raw data  error\n");
		return ICM206XX_ERR_I2C;
	}

	/* Sensor Raw Data direct read from sensor register     */
	/* No orientation translation, No unit Translation      */
	snprintf(buf, PAGE_SIZE, "%04x %04x %04x", data[ICM206XX_AXIS_X], data[ICM206XX_AXIS_Y], data[ICM206XX_AXIS_Z]);

	return ICM206XX_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int icm206xx_accel_ResetCalibration(struct i2c_client *client)
{
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));

	return ICM206XX_SUCCESS;
}

static int icm206xx_accel_ReadCalibration(struct i2c_client *client, struct SENSOR_DATA *sensor_data)
{
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);
	int i;
	int res = 0;
	int dat[ICM206XX_AXIS_NUM];

	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		dat[i] = obj->cvt.sign[i] * obj->cali_sw[obj->cvt.map[i]];
	}

	/* lower layer (sensor) unit Translation into Upper layer (HAL) unit */
	sensor_data->x = dat[ICM206XX_AXIS_X] * GRAVITY_EARTH_1000 / g_icm206xx_accel_sensitivity;
	sensor_data->y = dat[ICM206XX_AXIS_Y] * GRAVITY_EARTH_1000 / g_icm206xx_accel_sensitivity;
	sensor_data->z = dat[ICM206XX_AXIS_Z] * GRAVITY_EARTH_1000 / g_icm206xx_accel_sensitivity;

	if (atomic_read(&obj->trace)) {
		ACC_LOG("Accel ReadCalibration:[sensor_data:%5d %5d %5d][cali_sw:%5d %5d %5d]\n",
			sensor_data->x, sensor_data->y, sensor_data->z,
			obj->cali_sw[ICM206XX_AXIS_X], obj->cali_sw[ICM206XX_AXIS_Y], obj->cali_sw[ICM206XX_AXIS_Z]);
	}

	return res;
}

static int icm206xx_accel_WriteCalibration(struct i2c_client *client, struct SENSOR_DATA *sensor_data)
{
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);
	int cali[ICM206XX_AXIS_NUM];
	int res = 0;
	int i;

	/* Upper layer (HAL) unit Translation into lower layer (sensor) unit */
	cali[ICM206XX_AXIS_X] = sensor_data->x * g_icm206xx_accel_sensitivity / GRAVITY_EARTH_1000;
	cali[ICM206XX_AXIS_Y] = sensor_data->y * g_icm206xx_accel_sensitivity / GRAVITY_EARTH_1000;
	cali[ICM206XX_AXIS_Z] = sensor_data->z * g_icm206xx_accel_sensitivity / GRAVITY_EARTH_1000;

	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		/* Orientation Translation */
		obj->cali_sw[obj->cvt.map[i]] += obj->cvt.sign[i] * cali[i];
	}

	if (atomic_read(&obj->trace)) {
		ACC_LOG("Accel writeCalibration:[sensor_data:%5d %5d %5d][cali_sw:%5d %5d %5d]\n",
			sensor_data->x, sensor_data->y, sensor_data->z,
			obj->cali_sw[ICM206XX_AXIS_X], obj->cali_sw[ICM206XX_AXIS_Y], obj->cali_sw[ICM206XX_AXIS_Z]);
	}

	return res;
}

/*----------------------------------------------------------------------------*/
#ifdef ICM206XX_SELFTEST
static int icm206xx_accel_InitSelfTest(struct i2c_client *client)
{
	/* -------------------------------------------------------------------------------- */
	/* Configuration Setting on chip to do selftest */
	/* -------------------------------------------------------------------------------- */

	int res = 0;

	/* softreset */
	res = icm206xx_share_ChipSoftReset();
	if (res != ICM206XX_SUCCESS)
		return res;

	/* setpowermode(true) --> exit sleep */
	res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, true);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* fsr : ICM206XX_ACCEL_RANGE_2G */
	res = icm206xx_accel_SetFullScale(client, ICM206XX_ACCEL_RANGE_2G);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* filter : ICM206XX_ACCEL_DLPF_2 */
	res = icm206xx_accel_SetFilter(client, ICM206XX_ACCEL_DLPF_2);
	res = 0;
	if (res != ICM206XX_SUCCESS)
		return res;

	/* odr : 1000hz (1kHz) */
	res = icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, 1000000, true);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* Set enable sensor */
	res = icm206xx_share_EnableSensor(ICM206XX_SENSOR_TYPE_ACC, true);
	if (res != ICM206XX_SUCCESS)
		return res;

	return res;
}

static int icm206xx_accel_CalcAvgWithSamples(struct i2c_client *client, int avg[3], int count)
{
	int res = 0;
	int i, nAxis;
	s16 sensor_data[ICM206XX_AXIS_NUM];
	s32 sum[ICM206XX_AXIS_NUM] = { 0, 0, 0 };

	for (i = 0; i < count; i++) {
		res = icm206xx_accel_ReadSensorDataDirect(client, sensor_data);
		if (res) {
			ACC_PR_ERR("read data fail: %d\n", res);
			return ICM206XX_ERR_STATUS;
		}

		for (nAxis = 0; nAxis < ICM206XX_AXIS_NUM; nAxis++)
			sum[nAxis] += sensor_data[nAxis];
		/* Data register updated @1khz */
		mdelay(1);
	}

	for (nAxis = 0; nAxis < ICM206XX_AXIS_NUM; nAxis++)
		avg[nAxis] = (int)(sum[nAxis] / count) * ICM206XX_ST_PRECISION;

	return res;
}

static bool icm206xx_accel_DoSelfTest(struct i2c_client *client)
{
	int res = 0;
	int i;
	int acc_ST_on[ICM206XX_AXIS_NUM], acc_ST_off[ICM206XX_AXIS_NUM];
	int acc_ST_response[ICM206XX_AXIS_NUM];
	u8 st_code[ICM206XX_AXIS_NUM];	/* index of otp_lookup_tbl */
	u16 st_otp[ICM206XX_AXIS_NUM];
	bool otp_value_has_zero = false;
	bool test_result = true;
	u8 databuf[2] = { 0 };

	/* -------------------------------------------------------------------------------- */
	/* Acquire OTP value from OTP Lookup Table */
	/* -------------------------------------------------------------------------------- */

	/* read st_code */
	res = icm206xx_share_i2c_read_register(ICM206XX_REG_SELF_TEST_X_ACC, &(st_code[0]), 3);
	if (res) {
		ACC_PR_ERR("Read data fail: %d\n", res);
		return false;
	}

	ACC_LOG("st_code: %02x, %02x, %02x\n", st_code[0], st_code[1], st_code[2]);

	/* lookup OTP value with st_code */
	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		if (st_code[i] != 0)
			st_otp[i] = st_otp_lookup_tbl[st_code[i] - 1];
		else {
			st_otp[i] = 0;
			otp_value_has_zero = true;
		}
	}

	/* -------------------------------------------------------------------------------- */
	/* Read sensor data and calculate average values from it */
	/* -------------------------------------------------------------------------------- */

	/* Read 200 Samples with selftest off */
	res = icm206xx_accel_CalcAvgWithSamples(client, acc_ST_off, ICM206XX_ST_SAMPLE_COUNT);
	if (res) {
		ACC_PR_ERR("Read data fail: %d\n", res);
		return false;
	}

	/* set selftest on */
	res = icm206xx_share_i2c_read_register(ICM206XX_REG_ACCEL_CFG, databuf, 1);
	databuf[0] |= ICM206XX_BIT_ACCEL_SELFTEST;
	res = icm206xx_share_i2c_write_register(ICM206XX_REG_ACCEL_CFG, databuf, 1);

	/* Wait 20ms for oscillations to stabilize */
	mdelay(20);

	/* Read 200 Samples with selftest on */
	res = icm206xx_accel_CalcAvgWithSamples(client, acc_ST_on, ICM206XX_ST_SAMPLE_COUNT);
	if (res) {
		ACC_PR_ERR("Read data fail: %d\n", res);
		return false;
	}

	/* -------------------------------------------------------------------------------- */
	/* Compare calculated value with INVN OTP value to judge Success or Fail */
	/* -------------------------------------------------------------------------------- */

	for (i = 0; i < ICM206XX_AXIS_NUM; i++) {
		acc_ST_response[i] = abs(acc_ST_on[i] - acc_ST_off[i]);

		if (otp_value_has_zero == false) {
			ACC_LOG("Selftest result A : [i:%d][%d][%d][%d]\n",
				i,
				acc_ST_response[i],
				(st_otp[i] * ICM206XX_ST_PRECISION * ICM206XX_ST_ACCEL_DELTA_MIN / 100),
				(st_otp[i] * ICM206XX_ST_PRECISION * ICM206XX_ST_ACCEL_DELTA_MAX / 100));

			if ((acc_ST_response[i] <
				(st_otp[i] * ICM206XX_ST_PRECISION * ICM206XX_ST_ACCEL_DELTA_MIN / 100)) ||
				(acc_ST_response[i] >
				(st_otp[i] * ICM206XX_ST_PRECISION * ICM206XX_ST_ACCEL_DELTA_MAX / 100)))
				test_result = false;
		} else {
			ACC_LOG("Selftest result B: [i:%d][%d][%d][%d]\n",
				i,
				acc_ST_response[i],
				(ICM206XX_ST_ACCEL_AL_MIN * 16384 / 1000 * ICM206XX_ST_PRECISION),
				(ICM206XX_ST_ACCEL_AL_MAX * 16384 / 1000 * ICM206XX_ST_PRECISION));

			if ((acc_ST_response[i] < (ICM206XX_ST_ACCEL_AL_MIN * 16384 / 1000 * ICM206XX_ST_PRECISION)) ||
			    (acc_ST_response[i] > (ICM206XX_ST_ACCEL_AL_MAX * 16384 / 1000 * ICM206XX_ST_PRECISION)))
				test_result = false;
		}

		ACC_LOG("Selftest result:%d [i:%d][acc_ST_response:%d][acc_ST_on:%d][acc_ST_off:%d]\n",
			test_result, i, acc_ST_response[i], acc_ST_on[i], acc_ST_off[i]);

	}

	return test_result;
}
#endif
/*----------------------------------------------------------------------------*/
static int icm206xx_accel_init_client(struct i2c_client *client, bool enable)
{
	int res = 0;

	/* Exit sleep mode */
	res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, true);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* Set fsr ICM206XX_ACCEL_RANGE_4G as default */
	res = icm206xx_accel_SetFullScale(client, ICM206XX_ACCEL_RANGE_4G);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* Set filter ICM206XX_ACCEL_DLPF_1 as default */
	res = icm206xx_accel_SetFilter(client, ICM206XX_ACCEL_DLPF_1);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* Set 5ms(500hz) sample rate */
	res = icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, 2000000, false);
	if (res != ICM206XX_SUCCESS)
		return res;

	/* Disable sensor - standby mode for accelerometer */
	/*res = icm206xx_share_EnableSensor(ICM206XX_SENSOR_TYPE_ACC, enable);
	if (res != ICM206XX_SUCCESS)
		return res;*/

	/* Set power mode - sleep or normal */
	res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, enable);
	if (res != ICM206XX_SUCCESS) {
		return res;
	}

	ACC_LOG("icm206xx_accel_init_client OK!\n");

	return ICM206XX_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/* For  driver get cust info
struct acc_hw *get_cust_acc(void)
{
	return &accel_cust;
}
*/
/* accelerometer power control function */
static void icm206xx_accel_power(struct acc_hw *hw, unsigned int on)
{
/* Usually, it do nothing in the power function. Because the power of sensor is always on. */
}

/*=======================================================================================*/
/* Driver Attribute Section */
/*=======================================================================================*/

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[ICM206XX_BUFSIZE];

	icm206xx_share_ReadChipInfo(strbuf, ICM206XX_BUFSIZE);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm206xx_accel_i2c_client;
	char strbuf[ICM206XX_BUFSIZE];

	if (client == NULL) {
		ACC_PR_ERR("i2c client is null!!\n");
		return 0;
	}

	icm206xx_accel_ReadSensorData(client, strbuf, ICM206XX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (kstrtoint(buf, 16, &trace) == 0)
		atomic_set(&obj->trace, trace);
	else
		ACC_PR_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
			obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);

	return len;
}

static ssize_t show_chip_orientation(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct acc_hw *obj = &obj_i2c_data->hw;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	ACC_LOG("[%s] default direction: %d\n", __func__, obj->direction);

	len = snprintf(buf, PAGE_SIZE, "default direction = %d\n", obj->direction);

	return len;
}

static ssize_t store_chip_orientation(struct device_driver *ddri, const char *buf, size_t tCount)
{
	int nDirection = 0;
	int res = 0;
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = kstrtoint(buf, 10, &nDirection);
	if (res == 0) {
		if (hwmsen_get_convert(nDirection, &obj->cvt)) {
			ACC_PR_ERR("ERR: fail to set direction\n");
		}
	}

	ACC_LOG("[%s] set direction: %d\n", __func__, nDirection);

	return tCount;
}

static ssize_t show_register_dump(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;
	int i;
	u8 databuf[2] = { 0 };
	int res = 0;

	if (obj == NULL) {
		ACC_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	len += snprintf(buf + len, PAGE_SIZE, "Register Dump\n");

	for (i = 0; i < 0x7F; i++) {
		/* Don't read memory r/w register */
		if (i == ICM206XX_REG_MEM_R_W)
			databuf[0] = 0;
		else {
			res = icm206xx_share_i2c_read_register(i, databuf, 1);
			if (res < 0) {
				ACC_PR_ERR("read register err!\n");
				return 0;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE, "0x%02x: 0x%02x\n", i, databuf[0]);
	}

	return len;
}

#ifdef ICM206XX_SELFTEST
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = icm206xx_accel_i2c_client;

	if (client == NULL) {
		ACC_PR_ERR("i2c client is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", selftestRes);
}

static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = icm206xx_accel_i2c_client;
	int num;
	int res = 0;

	/* check parameter values to run selftest */
	res = kstrtoint(buf, 10, &num);
	if (res != 0) {
		ACC_PR_ERR("parse number fail\n");
		return count;
	} else if (num == 0) {
		ACC_PR_ERR("invalid data count\n");
		return count;
	}

	/* run selftest */
	res = icm206xx_accel_InitSelfTest(client);

	if (icm206xx_accel_DoSelfTest(client) == true) {
		strlcpy(selftestRes, "y", 1);
		ACC_LOG("ACCEL SELFTEST : PASS\n");
	} else {
		strlcpy(selftestRes, "n", 1);
		ACC_LOG("ACCEL SELFTEST : FAIL\n");
	}

	/* Taly!!! : Selftest is considered to be called only in factory mode */
	/* In general mode, the condition before selftest will not be recovered and sensor will not be in sleep mode */
	res = icm206xx_accel_init_client(client, true);

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(regdump, S_IRUGO, show_register_dump, NULL);
#ifdef ICM206XX_SELFTEST
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_selftest_value, store_selftest_value);
#endif

static struct driver_attribute *icm206xx_accel_attr_list[] = {
	&driver_attr_chipinfo,	/* chip information */
	&driver_attr_sensordata,	/* dump sensor data */
	&driver_attr_trace,	/* trace log */
	&driver_attr_status,	/* chip status */
	&driver_attr_orientation,	/* chip orientation information */
	&driver_attr_regdump,	/* register dump */
#ifdef ICM206XX_SELFTEST
	&driver_attr_selftest,	/* run selftest when store, report selftest result when show */
#endif
};

/*----------------------------------------------------------------------------*/
static int icm206xx_accel_create_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(ARRAY_SIZE(icm206xx_accel_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		res = driver_create_file(driver, icm206xx_accel_attr_list[idx]);
		if (res != 0) {
			ACC_PR_ERR("driver_create_file (%s) = %d\n", icm206xx_accel_attr_list[idx]->attr.name, res);
			break;
		}
	}
	return res;
}

static int icm206xx_accel_delete_attr(struct device_driver *driver)
{
	int idx;
	int res = 0;
	int num = (int)(ARRAY_SIZE(icm206xx_accel_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, icm206xx_accel_attr_list[idx]);

	return res;
}

#ifdef ACC_OWN_MISCDEVICE
/*=======================================================================================*/
/* Misc - Factory Mode (IOCTL) Device Driver Section					 */
/*=======================================================================================*/

static int icm206xx_accel_open(struct inode *inode, struct file *file)
{
	file->private_data = icm206xx_accel_i2c_client;

	if (file->private_data == NULL) {
		ACC_PR_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

static int icm206xx_accel_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long icm206xx_accel_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct icm206xx_accel_i2c_data *obj = (struct icm206xx_accel_i2c_data *)i2c_get_clientdata(client);
	char strbuf[ICM206XX_BUFSIZE] = { 0 };
	void __user *data;
	long res = 0;
	struct SENSOR_DATA sensor_data;
	static struct GSENSOR_VECTOR3D gsensor_gain;

	if (_IOC_DIR(cmd) & _IOC_READ)
		res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (res) {
		ACC_PR_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

/*	addCmdLog(cmd);
	printCmdLog();*/

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		icm206xx_share_ChipSoftReset();
		icm206xx_accel_init_client(client, true);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		icm206xx_share_ReadChipInfo(strbuf, ICM206XX_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			res = -EINVAL;
			break;
		}

		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		icm206xx_accel_ReadSensorData(client, strbuf, ICM206XX_BUFSIZE);
		if (copy_to_user(data, strbuf, sizeof(strbuf))) {
			res = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_OFFSET:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		icm206xx_accel_ReadOffsetData(client, strbuf);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			res = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		icm206xx_accel_ReadGain(client, &gsensor_gain);
		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
			res = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		icm206xx_accel_ReadRawData(client, strbuf);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			res = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			res = -EFAULT;
			break;
		}

		if (atomic_read(&obj->suspend)) {
			ACC_PR_ERR("Perform calibration in suspend state!!\n");
			res = -EINVAL;
		} else {
			ACC_LOG("accel set cali:[%5d %5d %5d]\n", sensor_data.x, sensor_data.y, sensor_data.z);

			res = icm206xx_accel_WriteCalibration(client, &sensor_data);
		}
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		res = icm206xx_accel_ReadCalibration(client, &sensor_data);
		if (res)
			break;

		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			res = -EFAULT;
			break;
		}

		break;

	case GSENSOR_IOCTL_CLR_CALI:
		res = icm206xx_accel_ResetCalibration(client);
		break;

	default:
		ACC_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		res = -ENOIOCTLCMD;
	}

	return res;
}

#ifdef CONFIG_COMPAT
static long icm206xx_accel_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long res = 0;
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl) {
		ACC_PR_ERR("compat_ion_ioctl file has no f_op or no f_op->unlocked_ioctl.\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_INIT:
	case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
	case COMPAT_GSENSOR_IOCTL_READ_OFFSET:
	case COMPAT_GSENSOR_IOCTL_READ_GAIN:
	case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			ACC_PR_ERR("invalid argument.");
			return -EINVAL;
		}

		res = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
		break;

	default:
		ACC_PR_ERR("%s not supported = 0x%04x", __func__, cmd);
		return -ENOIOCTLCMD;
	}

	return res;
}
#endif

static const struct file_operations icm206xx_accel_fops = {
	.open = icm206xx_accel_open,
	.release = icm206xx_accel_release,
	.unlocked_ioctl = icm206xx_accel_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = icm206xx_accel_compat_ioctl,
#endif
};

static struct miscdevice icm206xx_accel_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &icm206xx_accel_fops,
};
#endif
/*=======================================================================================*/
/* Misc - I2C HAL Support Section							 */
/*=======================================================================================*/

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int icm206xx_accel_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int icm206xx_accel_enable_nodata(int en)
{
	int res = 0;
	u8 databuf[2] = { 0 };

	if (en == 1) {
		power_acc = true;
		res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, power_acc);
		res = icm206xx_share_EnableSensor(ICM206XX_SENSOR_TYPE_ACC, power_acc);
	}
	if (en == 0) {
		power_acc = false;
		res = icm206xx_share_EnableSensor(ICM206XX_SENSOR_TYPE_ACC, power_acc);
		res = icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, 2000000, true);
		res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, power_acc);
	}

	res = icm206xx_share_i2c_read_register(ICM206XX_REG_SAMRT_DIV, databuf, 1);
	ACC_LOG("%s read ODR: 0x%02x\n", __func__, databuf[0]);
	if (res != ICM206XX_SUCCESS) {
		ACC_PR_ERR("%s fail!\n", __func__);
		return res;
	}

	ACC_LOG("icm206xx_accel_enable_nodata OK (%d)!\n", en ? 1 : 0);

	return 0;
}

static int icm206xx_accel_set_delay(u64 ns)
{
	ACC_LOG("%s is called [ns:%lld]\n", __func__, ns);

	icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, ns, false);
	set_delay_ns = ns;

	return 0;
}

static int icm206xx_accel_get_data(int *x, int *y, int *z, int *status)
{
	char buff[ICM206XX_BUFSIZE];
	int res = 0;

	icm206xx_accel_ReadSensorData(obj_i2c_data->client, buff, ICM206XX_BUFSIZE);

	res = sscanf(buff, "%x %x %x", x, y, z);

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
/*=======================================================================================*/
/* Calibration Proc Section			add by zte dgsh 20171212		 */
/*=======================================================================================*/
/*--------------------------------------------------------start-----------------*/
static bool is_acc_calibration_valid(struct SENSOR_DATA *cali_value)
{
	/* cali_value's unit is mg. */
	if ((abs(cali_value->x) > ICM206XX_CALI_TOLERANCE)
		|| (abs(cali_value->y) > ICM206XX_CALI_TOLERANCE)
		|| (abs(cali_value->z) > 2 * ICM206XX_CALI_TOLERANCE)) {
		return false;
	}
	return true;
}
static int acc_calibrate_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = obj_i2c_data;

	return 0;
}

static int calculate_cali_data(struct SENSOR_DATA *cali_data)
{
	char buff[ICM206XX_BUFSIZE];
	int rawacc[ICM206XX_AXIS_NUM] = {0};
	int raw_sum[ICM206XX_AXIS_NUM] = {0};
	int i = 0;
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;
	int res = 0;

	if (power_acc == false) {
		res = icm206xx_accel_enable_nodata(true);
		if (res) {
			ACC_VER("Power on lis2ds12 error %d!\n", res);
		}
		msleep(20);
	}

	/* Reset calibration  */
	icm206xx_accel_ResetCalibration(obj->client);
	/* set odr 1khz. */
	res = icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, 1000000, true);
	if (res != ICM206XX_SUCCESS)
		return res;

	for (i = 0; i < ICM206XX_CALI_LEN; i++) {
		icm206xx_accel_ReadSensorData(obj->client, buff, ICM206XX_BUFSIZE);
		res = sscanf(buff, "%x %x %x",
			&rawacc[ICM206XX_AXIS_X], &rawacc[ICM206XX_AXIS_Y], &rawacc[ICM206XX_AXIS_Z]);
		if (res < 0) {
			ACC_VER("I2C error at %d: ret value=%d", i, res);
			res = -3;
			return res;
		}
/* ACC_LOG("rawacc[x=%d,y=%d,z=%d].\n", rawacc[ICM206XX_AXIS_X], rawacc[ICM206XX_AXIS_Y], rawacc[ICM206XX_AXIS_Z]); */
		raw_sum[ICM206XX_AXIS_X] += rawacc[ICM206XX_AXIS_X];
		raw_sum[ICM206XX_AXIS_Y] += rawacc[ICM206XX_AXIS_Y];
		raw_sum[ICM206XX_AXIS_Z] += rawacc[ICM206XX_AXIS_Z];
		msleep(20);
	}
	res = icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, set_delay_ns, false);
	if (res != ICM206XX_SUCCESS)
		return res;
	/* calc avarage */
	ACC_LOG("sum,x:%d,y:%d,z:%d\n", raw_sum[ICM206XX_AXIS_X], raw_sum[ICM206XX_AXIS_Y], raw_sum[ICM206XX_AXIS_Z]);
	rawacc[ICM206XX_AXIS_X] = raw_sum[ICM206XX_AXIS_X] / ICM206XX_CALI_LEN;
	rawacc[ICM206XX_AXIS_Y] = raw_sum[ICM206XX_AXIS_Y] / ICM206XX_CALI_LEN;
	rawacc[ICM206XX_AXIS_Z] = raw_sum[ICM206XX_AXIS_Z] / ICM206XX_CALI_LEN;
	ACC_LOG("avg,x:%d,y:%d,z:%d\n", rawacc[ICM206XX_AXIS_X], rawacc[ICM206XX_AXIS_Y], rawacc[ICM206XX_AXIS_Z]);

	cali_data->x = 0 - rawacc[ICM206XX_AXIS_X];
	cali_data->y = 0 - rawacc[ICM206XX_AXIS_Y];
	cali_data->z = GRAVITY_EARTH_1000 - rawacc[ICM206XX_AXIS_Z];

	/*check........ */
	if (false == is_acc_calibration_valid(cali_data)) {
		ACC_VER("calibration value invalid[X:%d,Y:%d,Z:%d].",
				cali_data->x, cali_data->y, cali_data->z);
		res = -2;
		return res;
	}

	icm206xx_accel_WriteCalibration(obj->client, cali_data);
	return res;
}

static ssize_t acc_calibrate_read_proc(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int cnt;
	int res = 0;
	char buff[ICM206XX_BUFSIZE];
	struct SENSOR_DATA cali_data = {.x = 0, .y = 0, .z = 0};

	ACC_LOG("%s enter.\n", __func__);
	if (*offset != 0) {
		ACC_LOG("%s,offset!=0 -> return 0\n", __func__);
		return 0;
	}

	res = calculate_cali_data(&cali_data);
	if (res < 0) {
		ACC_LOG("%s, calculate_cali_data error\n", __func__);
		return res;
	}

	cnt = snprintf(buff, PAGE_SIZE, "%d %d %d", cali_data.x, cali_data.y, cali_data.z);
	ACC_LOG("cnt:%d, buffer:%s", cnt, buff);
	if (copy_to_user(buffer, buff, cnt)) {
		ACC_PR_ERR("%s, copy_to_user error\n", __func__);
		return -EINVAL;
	}
	*offset += cnt;
	return cnt;
}

static ssize_t acc_calibrate_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
	struct icm206xx_accel_i2c_data *obj = file->private_data;
	struct SENSOR_DATA cali_data = {.x = 0, .y = 0, .z = 0};
	char buf[16] = {0};
	int res = 0;

	size_t copyLen = 0;

	ACC_LOG("%s: write len = %d\n", __func__, (int)len);
	copyLen = len < 16 ? len : 16;
	if (copy_from_user(buf, user_buf, copyLen)) {
		ACC_LOG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}

	res = sscanf(buf, "%d %d %d", &cali_data.x, &cali_data.y, &cali_data.z);
	ACC_LOG("[x:%d,y:%d,z:%d], copyLen=%d\n", cali_data.x, cali_data.y, cali_data.z, (int)copyLen);

	/*check........ */
	if (false == is_acc_calibration_valid(&cali_data)) {
		ACC_VER("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data.x, cali_data.y, cali_data.z);
		return -EFAULT;
	}

	icm206xx_accel_ResetCalibration(obj->client);
	icm206xx_accel_WriteCalibration(obj->client, &cali_data);
	return len;
}

static const struct file_operations acc_calibrate_proc_fops = {
	.owner      = THIS_MODULE,
	.open       = acc_calibrate_open,
	.read       = acc_calibrate_read_proc,
	.write      = acc_calibrate_write_proc,
};

static void create_acc_calibrate_proc_file(void)
{
	acc_calibrate_proc_file = proc_create("driver/acc_calibration", 0664, NULL, &acc_calibrate_proc_fops);
	ACC_LOG("%s enter.\n", __func__);

	if (acc_calibrate_proc_file == NULL) {
		ACC_VER("create acc_calibrate_proc_file fail!\n");
	}
}
/*acc calibrate proc, add by zte dgsh 20161212  --e-n-d--*/
/*----------------------------------------------------------------------------*/


/*=======================================================================================*/
/* I2C Device Driver Section */
/*=======================================================================================*/

static const struct i2c_device_id icm206xx_accel_i2c_id[] = { {ICM206XX_ACCEL_DEV_NAME, 0}, {} };

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};
#endif


static int icm206xx_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	ACC_LOG("%s is called [ns:%lld]\n", __func__, samplingPeriodNs);
	icm206xx_share_SetSampleRate(ICM206XX_SENSOR_TYPE_ACC, samplingPeriodNs, false);
	set_delay_ns = samplingPeriodNs;
	return 0;
}

static int icm206xx_flush(void)
{
	return acc_flush_report();
}

/************************* For MTK factory mode ************************************/
static int icm206xx_factory_do_self_test(void)
{
	return 0;
}

static int icm206xx_factory_get_cali(int32_t data[3])
{
	struct i2c_client *client = icm206xx_accel_i2c_client;
	struct SENSOR_DATA sensor_data;
	int err = -1;

	err = icm206xx_accel_ReadCalibration(client, &sensor_data);
	if (err) {
		ACC_PR_ERR("Read accel cali fail\n");
		return err;
	}
	data[0] = sensor_data.x;
	data[1] = sensor_data.y;
	data[2] = sensor_data.z;
	return 0;
}

static int icm206xx_factory_set_cali(int32_t data[3])
{
	int err = 0;
	struct SENSOR_DATA sensor_data;

	sensor_data.x = data[0];
	sensor_data.y = data[1];
	sensor_data.z = data[2];
	err = icm206xx_accel_WriteCalibration(icm206xx_accel_i2c_client, &sensor_data);
	if (err) {
		ACC_PR_ERR("icm206xx_accel_WriteCalibration failed!\n");
		return err;
	}
	return 0;
}

static int icm206xx_factory_enable_calibration(void)
{
	struct acc_data offset_data;
	struct SENSOR_DATA cali_data = {.x = 0, .y = 0, .z = 0};
	int    res = 0;

	res = calculate_cali_data(&cali_data);
	if (res < 0) {
		ACC_PR_ERR("%s, calculate_cali_data error\n", __func__);
		return res;
	}

	offset_data.x = cali_data.x;
	offset_data.y = cali_data.y;
	offset_data.z = cali_data.z;
	res = acc_cali_report(&offset_data);
	return res;
}

static int icm206xx_factory_clear_cali(void)
{
	int err = 0;

	err = icm206xx_accel_ResetCalibration(icm206xx_accel_i2c_client);
	if (err) {
		ACC_PR_ERR("icm206xx_accel_factory_clear_cali failed!\n");
		return err;
	}
	return 0;
}

static int icm206xx_factory_get_raw_data(int32_t data[3])
{
	char strbuf[ICM206XX_BUFSIZE] = { 0 };

	icm206xx_accel_ReadRawData(icm206xx_accel_i2c_client, strbuf);
	ACC_LOG("support icm206xx_accel_factory_get_raw_data!\n");
	return 0;
}

static int icm206xx_factory_get_data(int32_t data[3], int *status)
{
	return icm206xx_accel_get_data(&data[0], &data[1], &data[2], status);

}

static int icm206xx_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = icm206xx_accel_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		ACC_PR_ERR("%s enable accel sensor failed!\n", __func__);
		err = -1;
		return err;
	}
	err = icm206xx_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		ACC_PR_ERR("%s enable accel set batch failed!\n", __func__);
		err = -1;
		return err;
	}
	return 0;
}

static struct accel_factory_fops icm206xx_factory_fops = {
	.enable_sensor = icm206xx_factory_enable_sensor,
	.get_data = icm206xx_factory_get_data,
	.get_raw_data = icm206xx_factory_get_raw_data,
	.enable_calibration = icm206xx_factory_enable_calibration,
	.clear_cali = icm206xx_factory_clear_cali,
	.set_cali = icm206xx_factory_set_cali,
	.get_cali = icm206xx_factory_get_cali,
	.do_self_test = icm206xx_factory_do_self_test,
};

static struct accel_factory_public icm206xx_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &icm206xx_factory_fops,
};
#ifdef CONFIG_TRAN_SYSTEM_DEVINFO
extern void app_get_g_sensor_name(char *g_name);
#endif
bool icm206xx_Acc_CheckChipID(struct i2c_client *client)
{
	u8 databuf = 0;
	int res = 0;

	if (client == NULL) {
		return false;
	}

	res = hwmsen_read_byte(client, ICM206XX_REG_WHO_AM_I, &databuf);
	if (res < 0) {
		ACC_PR_ERR("read who_am_i register err!\n");
		return false;
	}
	if (databuf == ICM20608D_WHO_AM_I) {
		/* read bmi160 chip id addr. */
		res = hwmsen_read_byte(client, BMI160_USER_CHIP_ID_ADDR, &databuf);
		if (res < 0) {
			ACC_PR_ERR("read bmi160 who_am_i register err!\n");
			return false;
		}
		if (databuf == SENSOR_CHIP_ID_BMI_C2) {
			/* its bmi160 chip id, check more (bmi160's 0x75 is R/W, icm20600's 0x75 is RO). */
			ACC_LOG("Register[0x00=0x%02x], cant confirm bmi or icm, check more.\n", databuf);
			res = hwmsen_write_byte(client, ICM206XX_REG_WHO_AM_I, 0x00);
			if (res < 0) {
				ACC_PR_ERR("write bmi160 who_am_i register err!\n");
				return false;
			}
			msleep(20);
			res = hwmsen_read_byte(client, ICM206XX_REG_WHO_AM_I, &databuf);
			if (res < 0) {
				ACC_PR_ERR("read who_am_i register err!\n");
				return false;
			}
			if (databuf != ICM20608D_WHO_AM_I) {
				return false;
			}
		}
		ACC_LOG("OK, It's ICM20600 device.\n");
		return true;
	}
	ACC_LOG("It's not ICM20600 device.\n");
	return false;
}

static int gsensor_set_cali(uint8_t *data, uint8_t count)
{
	int32_t *buf = (int32_t *)data;
	int error = 0;
	struct SENSOR_DATA offset = {.x = 0, .y = 0, .z = 0};
	struct icm206xx_accel_i2c_data *obj = obj_i2c_data;

	if (count >= 5) {
		offset.x = buf[3];
		offset.y = buf[4];
		offset.z = buf[5];
	}
	ACC_LOG("gsensor_set_cali %d %d %d\n", offset.x, offset.y, offset.z);

	icm206xx_accel_ResetCalibration(obj->client);
	error = icm206xx_accel_WriteCalibration(obj->client, &offset);
	return error;
}

static int icm206xx_accel_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct icm206xx_accel_i2c_data *obj;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	int res = 0;

	ACC_LOG();
	/*obj = kzalloc(sizeof(*obj), GFP_KERNEL);*/
	obj = devm_kzalloc(&client->dev, sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		res = -ENOMEM;
		goto exit;
	}
	if (!icm206xx_Acc_CheckChipID(client)) {
		res = -ENOMEM;
		goto exit;
	}

	res = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (res < 0) {
		ACC_PR_ERR("get accel dts info fail\n");
		goto exit;
	}

	res = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if (res) {
		ACC_PR_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	icm206xx_accel_i2c_client = new_client;

	/* Soft reset is called only in accelerometer init */
	/* Do not call soft reset in gyro and step_count init */
	res = icm206xx_share_ChipSoftReset();
	if (res != ICM206XX_SUCCESS)
		goto exit;

	res = icm206xx_accel_init_client(new_client, false);
	if (res)
		goto exit;

	/* misc_register() for factory mode, engineer mode and so on */
	/*res = misc_register(&icm206xx_accel_device); */
	res = accel_factory_device_register(&icm206xx_factory_device);
	if (res) {
		ACC_PR_ERR("icm206xx_accel_device acc_factory register failed!\n");
		goto exit;
	}

	/* Crate platform_driver attribute */
	res = icm206xx_accel_create_attr(&(icm206xx_accel_init_info.platform_diver_addr->driver));
	if (res) {
		ACC_PR_ERR("icm206xx_accel create attribute err = %d\n", res);
		goto exit_misc_device_register_failed;
	}

	/*create calibration proc */
	create_acc_calibrate_proc_file();

	/*-----------------------------------------------------------*/
	/* Fill and Register the acc_control_path and acc_data_path */
	/*-----------------------------------------------------------*/

	/* Fill the acc_control_path */
	ctl.is_use_common_factory = false;
	ctl.is_report_input_direct = false;
	ctl.batch = icm206xx_batch;
	ctl.flush = icm206xx_flush;
	ctl.is_support_batch = obj->hw.is_batch_supported;
	ctl.open_report_data = icm206xx_accel_open_report_data;	/* function pointer */
	ctl.enable_nodata = icm206xx_accel_enable_nodata;	/* function pointer */
	ctl.set_delay = icm206xx_accel_set_delay;	/* function pointer */
	ctl.set_cali = gsensor_set_cali;

	/* Register the acc_control_path */
	res = acc_register_control_path(&ctl);
	if (res) {
		ACC_PR_ERR("register accel control path err\n");
		goto exit_create_attr_failed;
	}

	/* Fill the acc_data_path */
	data.get_data = icm206xx_accel_get_data;	/* function pointer */
	data.vender_div = 1000;

	/* Register the acc_data_path */
	res = acc_register_data_path(&data);
	if (res) {
		ACC_PR_ERR("register accel_data_path fail = %d\n", res);
		goto exit_create_attr_failed;
	}

#ifdef CONFIG_TRAN_SYSTEM_DEVINFO
	app_get_g_sensor_name("icm206xx_gsensor");
#endif
	/* Set init_flag = 0 and return */
	icm206xx_accel_init_flag = 0;

	ACC_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	/*misc_deregister(&icm206xx_accel_device); */
	icm206xx_accel_delete_attr(&(icm206xx_accel_init_info.platform_diver_addr->driver));
	remove_proc_entry("driver/acc_calibration", NULL);

exit_misc_device_register_failed:
	accel_factory_device_deregister(&icm206xx_factory_device);
exit:
	ACC_PR_ERR("%s: err = %d\n", __func__, res);
	icm206xx_accel_init_flag = -1;
	icm206xx_accel_i2c_client = NULL;
	new_client = NULL;
	obj = NULL;
	return res;
}

static int icm206xx_accel_i2c_remove(struct i2c_client *client)
{
	int res = 0;

	res = icm206xx_accel_delete_attr(&(icm206xx_accel_init_info.platform_diver_addr->driver));
	if (res)
		ACC_PR_ERR("icm206xx_accel_delete_attr fail: %d\n", res);

	res = accel_factory_device_deregister(&icm206xx_factory_device);
	if (res)
		ACC_PR_ERR("misc_deregister fail: %d\n", res);

	icm206xx_accel_i2c_client = NULL;
	i2c_unregister_device(client);
	accel_factory_device_deregister(&icm206xx_factory_device);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static int icm206xx_accel_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, ICM206XX_ACCEL_DEV_NAME, sizeof(info->type));

	return 0;
}

static int icm206xx_accel_i2c_suspend(struct device *dev)
{
	int res = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);

	if (obj == NULL) {
		ACC_PR_ERR("null pointer!!\n");
		return -EINVAL;
	}
	atomic_set(&obj->suspend, 1);

	res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, false);
	if (res < 0) {
		ACC_PR_ERR("write power control fail!\n");
		return res;
	}

	icm206xx_accel_power(&obj->hw, 0);
	ACC_LOG("icm206xx_accel suspend ok\n");
	return res;
}

static int icm206xx_accel_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct icm206xx_accel_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	if (obj == NULL) {
		ACC_PR_ERR("null pointer!!\n");
		return -EINVAL;
	}

	icm206xx_accel_power(&obj->hw, 1);
	res = icm206xx_share_SetPowerMode(ICM206XX_SENSOR_TYPE_ACC, power_acc);

	if (res) {
		ACC_PR_ERR("initialize client fail!!\n");
		return res;
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}

static struct i2c_driver icm206xx_accel_i2c_driver = {
	.driver = {
			.name = ICM206XX_ACCEL_DEV_NAME,
#ifdef CONFIG_OF
			.of_match_table = accel_of_match,
#endif
#ifdef CONFIG_PM_SLEEP
			.pm = &icm26xx_i2c_pm_ops,
#endif
	},
	.probe = icm206xx_accel_i2c_probe,
	.remove = icm206xx_accel_i2c_remove,
	.detect = icm206xx_accel_i2c_detect,
	.id_table = icm206xx_accel_i2c_id,
};

/*=======================================================================================*/
/* Kernel Module Section								 */
/*=======================================================================================*/

static int icm206xx_accel_remove(void)
{
	i2c_del_driver(&icm206xx_accel_i2c_driver);

	return 0;
}

static int icm206xx_accel_local_init(void)
{
	int res = 0;

	if (i2c_add_driver(&icm206xx_accel_i2c_driver)) {
		ACC_PR_ERR("add driver error\n");
		res = -1;
		return res;
	}

	if (icm206xx_accel_init_flag == -1) {
		res = -1;
		return res;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init icm206xx_accel_init(void)
{
	acc_driver_add(&icm206xx_accel_init_info);

	return 0;
}

static void __exit icm206xx_accel_exit(void)
{

}

/*----------------------------------------------------------------------------*/
module_init(icm206xx_accel_init);
module_exit(icm206xx_accel_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("icm206xx accelerometer driver");
