/* LSM6DSE IMU driver
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

#include "lsm6dse.h"

static struct data_resolution lsm6dse_acc_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 0, 0}, 61},     /*2g   1LSB=61 ug */
	{{ 0, 0}, 488},    /* 16g */
	{{ 0, 0}, 122},    /* 4g */
	{{ 0, 0}, 244},    /* 8g */
};

static struct data_resolution lsm6dse_offset_resolution = {{15, 6}, 64};
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
struct acc_hw lsm6dse_acc_cust_hw;

static struct proc_dir_entry *acc_calibrate_proc_file = NULL;

/*For driver get cust info*/
struct acc_hw *lsm6dse_get_cust_acc_hw(void)
{
	return &lsm6dse_acc_cust_hw;
}

static int lsm6dse_acc_set_resolution(struct lsm6dse_acc *acc_obj)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;
	int res;
	u8  dat, reso;

	res = lsm6dse_i2c_read_block(client, LSM6DSE_REG_CTRL1_XL, &dat, 0x01);

	if (res < 0) {
		ST_ERR("write data format fail!!\n");
		return res;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso = (dat & LSM6DSE_REG_CTRL1_XL_MASK_FS_XL) >> 2;
	if (reso >= 0x3)
		reso = 0x3;

	if (reso < (ARRAY_SIZE(lsm6dse_acc_data_resolution))) {
		acc_obj->reso = &lsm6dse_acc_data_resolution[reso];
		return LSM6DSE_SUCCESS;
	}

	return -EINVAL;
}

static int lsm6dse_acc_read_rawdata(struct lsm6dse_acc *acc_obj, s16 data[LSM6DSE_AXES_NUM])
{

	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;

	u8 buf[LSM6DSE_DATA_LEN] = {0};
	int res = 0;

	if (client == NULL) {
		res = -EINVAL;
	} else {
		if ((lsm6dse_i2c_read_block(client, LSM6DSE_REG_OUTX_L_XL, buf, 0x06)) < 0) {
			ST_ERR("read  G sensor data register err!\n");
			res = -1;
			return res;
		}
		data[LSM6DSE_AXIS_X] = (s16)((buf[LSM6DSE_AXIS_X*2+1] << 8) | (buf[LSM6DSE_AXIS_X*2]));
		data[LSM6DSE_AXIS_Y] = (s16)((buf[LSM6DSE_AXIS_Y*2+1] << 8) | (buf[LSM6DSE_AXIS_Y*2]));
		data[LSM6DSE_AXIS_Z] = (s16)((buf[LSM6DSE_AXIS_Z*2+1] << 8) | (buf[LSM6DSE_AXIS_Z*2]));

		if (atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA) {
			ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LSM6DSE_AXIS_X], data[LSM6DSE_AXIS_Y],
				data[LSM6DSE_AXIS_Z], data[LSM6DSE_AXIS_X], data[LSM6DSE_AXIS_Y], data[LSM6DSE_AXIS_Z]);
		}

#ifdef CONFIG_LSM6DSE_LOWPASS
		if (atomic_read(&acc_obj->filter)) {
			if (atomic_read(&acc_obj->fir_en) && !atomic_read(&acc_obj->suspend)) {
				int idx, firlen = atomic_read(&acc_obj->firlen);

				if (acc_obj->fir.num < firlen) {
					acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_X] = data[LSM6DSE_AXIS_X];
					acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_Y] = data[LSM6DSE_AXIS_Y];
					acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_Z] = data[LSM6DSE_AXIS_Z];
					acc_obj->fir.sum[LSM6DSE_AXIS_X] += data[LSM6DSE_AXIS_X];
					acc_obj->fir.sum[LSM6DSE_AXIS_Y] += data[LSM6DSE_AXIS_Y];
					acc_obj->fir.sum[LSM6DSE_AXIS_Z] += data[LSM6DSE_AXIS_Z];
					if (atomic_read(&acc_obj->trace) & ADX_TRC_FILTER) {
						ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", acc_obj->fir.num,
							acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_X],
							acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_Y],
							acc_obj->fir.raw[acc_obj->fir.num][LSM6DSE_AXIS_Z],
							acc_obj->fir.sum[LSM6DSE_AXIS_X],
							acc_obj->fir.sum[LSM6DSE_AXIS_Y],
							acc_obj->fir.sum[LSM6DSE_AXIS_Z]);
					}
					acc_obj->fir.num++;
					acc_obj->fir.idx++;
				} else {
					idx = acc_obj->fir.idx % firlen;
					acc_obj->fir.sum[LSM6DSE_AXIS_X] -= acc_obj->fir.raw[idx][LSM6DSE_AXIS_X];
					acc_obj->fir.sum[LSM6DSE_AXIS_Y] -= acc_obj->fir.raw[idx][LSM6DSE_AXIS_Y];
					acc_obj->fir.sum[LSM6DSE_AXIS_Z] -= acc_obj->fir.raw[idx][LSM6DSE_AXIS_Z];
					acc_obj->fir.raw[idx][LSM6DSE_AXIS_X] = data[LSM6DSE_AXIS_X];
					acc_obj->fir.raw[idx][LSM6DSE_AXIS_Y] = data[LSM6DSE_AXIS_Y];
					acc_obj->fir.raw[idx][LSM6DSE_AXIS_Z] = data[LSM6DSE_AXIS_Z];
					acc_obj->fir.sum[LSM6DSE_AXIS_X] += data[LSM6DSE_AXIS_X];
					acc_obj->fir.sum[LSM6DSE_AXIS_Y] += data[LSM6DSE_AXIS_Y];
					acc_obj->fir.sum[LSM6DSE_AXIS_Z] += data[LSM6DSE_AXIS_Z];
					acc_obj->fir.idx++;
					data[LSM6DSE_AXIS_X] = acc_obj->fir.sum[LSM6DSE_AXIS_X]/firlen;
					data[LSM6DSE_AXIS_Y] = acc_obj->fir.sum[LSM6DSE_AXIS_Y]/firlen;
					data[LSM6DSE_AXIS_Z] = acc_obj->fir.sum[LSM6DSE_AXIS_Z]/firlen;
					if (atomic_read(&acc_obj->trace) & ADX_TRC_FILTER) {
						ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n",
							idx, acc_obj->fir.raw[idx][LSM6DSE_AXIS_X],
							acc_obj->fir.raw[idx][LSM6DSE_AXIS_Y],
							acc_obj->fir.raw[idx][LSM6DSE_AXIS_Z],
							acc_obj->fir.sum[LSM6DSE_AXIS_X],
							acc_obj->fir.sum[LSM6DSE_AXIS_Y],
							acc_obj->fir.sum[LSM6DSE_AXIS_Z],
							data[LSM6DSE_AXIS_X],
							data[LSM6DSE_AXIS_Y],
							data[LSM6DSE_AXIS_Z]);
					}
				}
			}
		}
#endif
	}

	return res;
}

static int lsm6dse_acc_reset_calibration(struct lsm6dse_acc *acc_obj)
{
	memset(acc_obj->cali_sw, 0x00, sizeof(acc_obj->cali_sw));
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_read_calibration(struct lsm6dse_acc *acc_obj, int dat[LSM6DSE_AXES_NUM])
{
	dat[acc_obj->cvt.map[LSM6DSE_AXIS_X]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_X]*acc_obj->cali_sw[LSM6DSE_AXIS_X];
	dat[acc_obj->cvt.map[LSM6DSE_AXIS_Y]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_Y]*acc_obj->cali_sw[LSM6DSE_AXIS_Y];
	dat[acc_obj->cvt.map[LSM6DSE_AXIS_Z]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_Z]*acc_obj->cali_sw[LSM6DSE_AXIS_Z];

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_write_calibration(struct lsm6dse_acc *acc_obj, int dat[LSM6DSE_AXES_NUM])
{
	int res = 0;
	s16 cali[LSM6DSE_AXES_NUM];

	ST_FUN();

	if (!acc_obj || !dat) {
		ST_ERR("null ptr!!\n");
		return -EINVAL;
	}

	cali[acc_obj->cvt.map[LSM6DSE_AXIS_X]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_X]*acc_obj->cali_sw[LSM6DSE_AXIS_X];
	cali[acc_obj->cvt.map[LSM6DSE_AXIS_Y]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_Y]*acc_obj->cali_sw[LSM6DSE_AXIS_Y];
	cali[acc_obj->cvt.map[LSM6DSE_AXIS_Z]] =
		acc_obj->cvt.sign[LSM6DSE_AXIS_Z]*acc_obj->cali_sw[LSM6DSE_AXIS_Z];

	cali[LSM6DSE_AXIS_X] += dat[LSM6DSE_AXIS_X];
	cali[LSM6DSE_AXIS_Y] += dat[LSM6DSE_AXIS_Y];
	cali[LSM6DSE_AXIS_Z] += dat[LSM6DSE_AXIS_Z];

	acc_obj->cali_sw[LSM6DSE_AXIS_X] +=
		acc_obj->cvt.sign[LSM6DSE_AXIS_X]*cali[acc_obj->cvt.map[LSM6DSE_AXIS_X]];
	acc_obj->cali_sw[LSM6DSE_AXIS_Y] +=
		acc_obj->cvt.sign[LSM6DSE_AXIS_Y]*cali[acc_obj->cvt.map[LSM6DSE_AXIS_Y]];
	acc_obj->cali_sw[LSM6DSE_AXIS_Z] +=
		acc_obj->cvt.sign[LSM6DSE_AXIS_Z]*cali[acc_obj->cvt.map[LSM6DSE_AXIS_Z]];

	return res;
}

/*----------------------------------------------------------------------------*/
/*acc calibrate proc, add by zte dgsh 20170227  --start-- */
static bool is_acc_calibration_valid(int cali_value[LSM6DSE_AXES_NUM])
{
	/* cali_value's unit is mg. */
	if ((abs(cali_value[LSM6DSE_AXIS_X]) > LSM6DSE_CALI_TOLERANCE)
		|| (abs(cali_value[LSM6DSE_AXIS_Y]) > LSM6DSE_CALI_TOLERANCE)
		|| (abs(cali_value[LSM6DSE_AXIS_Z]) > 2 * LSM6DSE_CALI_TOLERANCE)) {
		return false;
	}
	return true;
}

static int calc_acc_calibrate(int cali_data[LSM6DSE_AXES_NUM])
{
	s16 rawacc[LSM6DSE_AXES_NUM] = {0};
	s32 raw_sum[LSM6DSE_AXES_NUM] = {0};
	int i = 0;
	struct lsm6dse_acc *acc_obj = &(obj_i2c_data->lsm6dse_acc_data);
	int res = 0;


	if (acc_obj->lsm6dse_acc_power == false) {
		res = lsm6dse_acc_set_power_mode(acc_obj, true);
		if (res) {
			ST_ERR("Power on lis2ds12 error %d!\n", res);
		}
		msleep(20);
	}
	for (i = 0; i < LSM6DSE_CALI_LEN; i++) {
		res = lsm6dse_acc_read_rawdata(acc_obj, rawacc);
		if (res < 0) {
			ST_ERR("I2C error at %d: ret value=%d", i, res);
			res = -3;
			return res;
		}
		raw_sum[LSM6DSE_AXIS_X] += rawacc[LSM6DSE_AXIS_X];
		raw_sum[LSM6DSE_AXIS_Y] += rawacc[LSM6DSE_AXIS_Y];
		raw_sum[LSM6DSE_AXIS_Z] += rawacc[LSM6DSE_AXIS_Z];
		msleep(20);
	}
	/* calc avarage */
	ST_LOG("sum,x:%d,y:%d,z:%d\n", raw_sum[LSM6DSE_AXIS_X], raw_sum[LSM6DSE_AXIS_Y], raw_sum[LSM6DSE_AXIS_Z]);
	rawacc[LSM6DSE_AXIS_X] = raw_sum[LSM6DSE_AXIS_X] / LSM6DSE_CALI_LEN;
	rawacc[LSM6DSE_AXIS_Y] = raw_sum[LSM6DSE_AXIS_Y] / LSM6DSE_CALI_LEN;
	rawacc[LSM6DSE_AXIS_Z] = raw_sum[LSM6DSE_AXIS_Z] / LSM6DSE_CALI_LEN;
	ST_LOG("avg,x:%d,y:%d,z:%d\n", rawacc[LSM6DSE_AXIS_X], rawacc[LSM6DSE_AXIS_Y], rawacc[LSM6DSE_AXIS_Z]);

	rawacc[LSM6DSE_AXIS_X] = (rawacc[LSM6DSE_AXIS_X] * acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;
	rawacc[LSM6DSE_AXIS_Y] = (rawacc[LSM6DSE_AXIS_Y] * acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;
	rawacc[LSM6DSE_AXIS_Z] = (rawacc[LSM6DSE_AXIS_Z] * acc_obj->reso->sensitivity/1000)*GRAVITY_EARTH_1000/1000;

	cali_data[acc_obj->cvt.map[LSM6DSE_AXIS_X]] = 0 - acc_obj->cvt.sign[LSM6DSE_AXIS_X] * rawacc[LSM6DSE_AXIS_X];
	cali_data[acc_obj->cvt.map[LSM6DSE_AXIS_Y]] = 0 - acc_obj->cvt.sign[LSM6DSE_AXIS_Y] * rawacc[LSM6DSE_AXIS_Y];
	cali_data[acc_obj->cvt.map[LSM6DSE_AXIS_Z]] = GRAVITY_EARTH_1000
			- acc_obj->cvt.sign[LSM6DSE_AXIS_Z] * rawacc[LSM6DSE_AXIS_Z];

	/*check........ */
	if (false == is_acc_calibration_valid(cali_data)) {
		ST_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[LSM6DSE_AXIS_X],
			cali_data[LSM6DSE_AXIS_Y], cali_data[LSM6DSE_AXIS_Z]);
		res = -2;
		return res;
	}

	lsm6dse_acc_reset_calibration(acc_obj);
	lsm6dse_acc_write_calibration(acc_obj, cali_data);
	return res;
}
static ssize_t acc_calibrate_read_proc(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int cnt;
	int res = 0;
	int cali_data[LSM6DSE_AXES_NUM] = {0};
	char buff[20] = {'\0'};

	ST_FUN();
	if (*offset != 0) {
		ST_LOG("%s,offset!=0 -> return 0\n", __func__);
		return 0;
	}
	res = calc_acc_calibrate(cali_data);
	if (res < 0) {
		ST_ERR("%s, calc_acc_calibrate error\n", __func__);
		return res;
	}

	cnt = snprintf(buff, PAGE_SIZE, "%d %d %d", cali_data[LSM6DSE_AXIS_X],
		cali_data[LSM6DSE_AXIS_Y], cali_data[LSM6DSE_AXIS_Z]);
	ST_LOG("cnt:%d, buffer:%s", cnt, buff);
	if (copy_to_user(buffer, buff, cnt)) {
		ST_ERR("%s, copy_to_user error\n", __func__);
		return -EINVAL;
	}
	*offset += cnt;
	return cnt;
}

static ssize_t acc_calibrate_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
	int cali_data[LSM6DSE_AXES_NUM] = {0};
	char buf[16] = {0};
	int res = 0;

	size_t copyLen = 0;

	struct lsm6dse_acc *acc_obj = &(obj_i2c_data->lsm6dse_acc_data);

	ST_LOG("%s: write len = %d\n", __func__, (int)len);
	copyLen = len < 16 ? len : 16;
	if (copy_from_user(buf, user_buf, copyLen)) {
		ST_LOG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}

	res = sscanf(buf, "%d %d %d", &cali_data[LSM6DSE_AXIS_X],
		&cali_data[LSM6DSE_AXIS_Y], &cali_data[LSM6DSE_AXIS_Z]);
	ST_LOG("[x:%d,y:%d,z:%d], copyLen=%d\n",
		cali_data[LSM6DSE_AXIS_X], cali_data[LSM6DSE_AXIS_Y],
		cali_data[LSM6DSE_AXIS_Z], (int)copyLen);

	/*check........ */
	if (is_acc_calibration_valid(cali_data) == false) {
		ST_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[LSM6DSE_AXIS_X],
			cali_data[LSM6DSE_AXIS_Y], cali_data[LSM6DSE_AXIS_Z]);
		return -EFAULT;
	}

	lsm6dse_acc_reset_calibration(acc_obj);
	lsm6dse_acc_write_calibration(acc_obj, cali_data);
	return len;
}

static const struct file_operations acc_calibrate_proc_fops = {
	.owner      = THIS_MODULE,
	.read       = acc_calibrate_read_proc,
	.write      = acc_calibrate_write_proc,
};

static void create_acc_calibrate_proc_file(void)
{
	acc_calibrate_proc_file = proc_create("driver/acc_calibration", 0664, NULL, &acc_calibrate_proc_fops);
	ST_FUN(f);

	if (acc_calibrate_proc_file == NULL) {
		ST_ERR("create acc_calibrate_proc_file fail!\n");
	}
}
/* acc calibrate proc, add by zte dgsh 20170227  --e-n-d-- */
/*----------------------------------------------------------------------------*/

static int lsm6dse_acc_set_full_scale(struct lsm6dse_acc *acc_obj, u8 dataformat)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;
	int res = 0;

	res = lsm6dse_i2c_write_with_mask(client, LSM6DSE_REG_CTRL1_XL, LSM6DSE_REG_CTRL1_XL_MASK_FS_XL, dataformat);
	if (res < 0) {
		ST_ERR("read reg_ctl_reg1 register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return lsm6dse_acc_set_resolution(acc_obj);

}

static int lsm6dse_acc_set_odr(struct lsm6dse_acc *acc_obj, u8 odr)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;
	int res = 0;

#if ST_SENSOR_TILT || ST_SENSOR_STEP_DETECT || ST_SENSOR_STEP_COUNTER || ST_SENSOR_SIGNIFICANT_MOTION
	if (odr < LSM6DSE_REG_CTRL1_XL_ODR_26HZ) {
		if (CONFIG_PEDOMETER_ALWAYS_ON || obj->step_c_enabled
				|| obj->step_d_enabled || obj->significant_enabled || obj->tilt_enabled) {
			odr = LSM6DSE_REG_CTRL1_XL_ODR_26HZ;
		}
	}
#endif

	res = lsm6dse_i2c_write_with_mask(client, LSM6DSE_REG_CTRL1_XL, LSM6DSE_REG_CTRL1_XL_MASK_ODR_XL, odr);
	if (res < 0)
		return LSM6DSE_ERR_I2C;

	return LSM6DSE_SUCCESS;
}

int lsm6dse_acc_set_power_mode(struct lsm6dse_acc *acc_obj, bool state)
{
	int res = 0;

	if (state == acc_obj->lsm6dse_acc_power) {
		ST_LOG("Sensor power status is newest!\n");
		return LSM6DSE_SUCCESS;
	}

	if (state == true) {
		if (acc_obj->odr == 0) {
			acc_obj->odr = LSM6DSE_REG_CTRL1_XL_ODR_104HZ;
		}
		res = lsm6dse_acc_set_odr(acc_obj, acc_obj->odr);
	} else if (state == false) {
		res = lsm6dse_acc_set_odr(acc_obj, LSM6DSE_REG_CTRL1_XL_ODR_0HZ);
	} else {
		ST_ERR("set power state error!\n");
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (res < 0) {
		ST_ERR("set power mode failed!\n");
		return LSM6DSE_ERR_I2C;
	} else if (atomic_read(&acc_obj->trace) & ADX_TRC_INFO) {
		ST_LOG("set power mode ok %d!\n", state);
	}

	acc_obj->lsm6dse_acc_power = state;
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_init(struct lsm6dse_acc *acc_obj, bool enable)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;
	int res = 0;
	u8 buf[2] = {0, 0};

	ST_FUN();

	buf[0] = 0x00;
	res = lsm6dse_i2c_write_block(client, LSM6DSE_REG_CTRL1_XL, buf, 0x01);
	if (res < 0) {
		ST_ERR("lsm6dse_acc_init step 1!\n");
		return res;
	}

	res = lsm6dse_acc_set_full_scale(acc_obj, LSM6DSE_REG_CTRL1_XL_FS_4G); /* pedometer better work at 4G */
	if (res < 0) {
		ST_ERR("lsm6dse_acc_init step 3!\n");
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = acc_obj->reso->sensitivity;

	acc_obj->odr = 0;
	res = lsm6dse_acc_set_power_mode(acc_obj, enable);
	if (res < 0) {
		ST_ERR("lsm6dse_acc_init step 2!\n");
		return res;
	}

#ifdef CONFIG_LSM6DSE_LOWPASS
	memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
#endif
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_read_chip_name(struct lsm6dse_acc *acc_obj, u8 *buf, int bufsize)
{
	snprintf(buf, PAGE_SIZE, "%s", acc_obj->name);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_read_data(struct lsm6dse_acc *acc_obj, u8 *data, int bufsize)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;

	int acc[LSM6DSE_AXES_NUM];
	int acc_temp[LSM6DSE_AXES_NUM];
	int res = 0;

	if (data == NULL) {
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (client == NULL) {
		*data = 0;
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (atomic_read(&acc_obj->suspend)) {
		ST_LOG("sensor in suspend read not data!\n");
		return LSM6DSE_SUCCESS;
	}

	if (acc_obj->lsm6dse_acc_power == false) {
		lsm6dse_acc_set_power_mode(acc_obj, true);
		/*TODO: need turn-on time */
	}

	res = lsm6dse_acc_read_rawdata(acc_obj, acc_obj->data);
	if (res) {
		ST_ERR("I2C error: ret value=%d", res);
		return res;
	}

	acc_temp[LSM6DSE_AXIS_X] = (acc_obj->data[LSM6DSE_AXIS_X]*acc_obj->reso->sensitivity/1000)
			*GRAVITY_EARTH_1000/1000;
	acc_temp[LSM6DSE_AXIS_Y] = (acc_obj->data[LSM6DSE_AXIS_Y]*acc_obj->reso->sensitivity/1000)
			*GRAVITY_EARTH_1000/1000;
	acc_temp[LSM6DSE_AXIS_Z] = (acc_obj->data[LSM6DSE_AXIS_Z]*acc_obj->reso->sensitivity/1000)
			*GRAVITY_EARTH_1000/1000;

	acc_temp[LSM6DSE_AXIS_X] += acc_obj->cali_sw[LSM6DSE_AXIS_X];
	acc_temp[LSM6DSE_AXIS_Y] += acc_obj->cali_sw[LSM6DSE_AXIS_Y];
	acc_temp[LSM6DSE_AXIS_Z] += acc_obj->cali_sw[LSM6DSE_AXIS_Z];

	/*remap coordinate*/
	acc[acc_obj->cvt.map[LSM6DSE_AXIS_X]] = acc_obj->cvt.sign[LSM6DSE_AXIS_X]*acc_temp[LSM6DSE_AXIS_X];
	acc[acc_obj->cvt.map[LSM6DSE_AXIS_Y]] = acc_obj->cvt.sign[LSM6DSE_AXIS_Y]*acc_temp[LSM6DSE_AXIS_Y];
	acc[acc_obj->cvt.map[LSM6DSE_AXIS_Z]] = acc_obj->cvt.sign[LSM6DSE_AXIS_Z]*acc_temp[LSM6DSE_AXIS_Z];

	snprintf(data, PAGE_SIZE, "%04x %04x %04x", acc[LSM6DSE_AXIS_X], acc[LSM6DSE_AXIS_Y], acc[LSM6DSE_AXIS_Z]);
	if (atomic_read(&acc_obj->trace) & ADX_TRC_IOCTL) {
		ST_LOG("gsensor data: %s!\n", data);
		dumpReg(obj);
	}

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_read_rawdata_string(struct lsm6dse_acc *acc_obj, u8 *buf)
{
	struct lsm6dse_data *obj = container_of(acc_obj, struct lsm6dse_data, lsm6dse_acc_data);
	struct i2c_client *client = obj->client;
	int res = 0;

	if (!buf || !client) {
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	res = lsm6dse_acc_read_rawdata(acc_obj, acc_obj->data);
	if (res) {
		ST_ERR("I2C error: ret value=%d", res);
		return res;
	}

	snprintf(buf, PAGE_SIZE, "%04x %04x %04x", acc_obj->data[LSM6DSE_AXIS_X],
			acc_obj->data[LSM6DSE_AXIS_Y], acc_obj->data[LSM6DSE_AXIS_Z]);

	return LSM6DSE_SUCCESS;
}

static ssize_t lsm6dse_attr_acc_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	u8 strbuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return LSM6DSE_SUCCESS;
	}

	lsm6dse_acc_read_chip_name(acc_obj, strbuf, LSM6DSE_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t lsm6dse_attr_acc_show_chipid_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return LSM6DSE_SUCCESS;
	}

	return snprintf(buf, PAGE_SIZE, "0x%x\n", obj->chip_id);
}

static ssize_t lsm6dse_attr_acc_show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	u8 databuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return LSM6DSE_SUCCESS;
	}

	lsm6dse_acc_read_data(acc_obj, databuf, LSM6DSE_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lsm6dse_attr_acc_show_rawdata_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	u8 databuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return LSM6DSE_SUCCESS;
	}

	lsm6dse_acc_read_rawdata_string(acc_obj, databuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lsm6dse_attr_acc_show_cali_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res, len, mul;
	int tmp[LSM6DSE_AXES_NUM];

	len = 0;

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	res = lsm6dse_acc_read_calibration(acc_obj, tmp);
	if (res) {
		return -EINVAL;
	}

	mul = acc_obj->reso->sensitivity/lsm6dse_offset_resolution.sensitivity;
	len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n",
			mul, acc_obj->offset[LSM6DSE_AXIS_X],
			acc_obj->offset[LSM6DSE_AXIS_Y], acc_obj->offset[LSM6DSE_AXIS_Z],
			acc_obj->offset[LSM6DSE_AXIS_X],
			acc_obj->offset[LSM6DSE_AXIS_Y], acc_obj->offset[LSM6DSE_AXIS_Z]);

	len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			acc_obj->cali_sw[LSM6DSE_AXIS_X],
			acc_obj->cali_sw[LSM6DSE_AXIS_Y], acc_obj->cali_sw[LSM6DSE_AXIS_Z]);

	len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			acc_obj->offset[LSM6DSE_AXIS_X]*mul + acc_obj->cali_sw[LSM6DSE_AXIS_X],
			acc_obj->offset[LSM6DSE_AXIS_Y]*mul + acc_obj->cali_sw[LSM6DSE_AXIS_Y],
			acc_obj->offset[LSM6DSE_AXIS_Z]*mul + acc_obj->cali_sw[LSM6DSE_AXIS_Z],
			tmp[LSM6DSE_AXIS_X], tmp[LSM6DSE_AXIS_Y], tmp[LSM6DSE_AXIS_Z]);

	return len;
}

static ssize_t lsm6dse_attr_acc_store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res, x, y, z;
	int dat[LSM6DSE_AXES_NUM];

	res = strncmp(buf, "rst", 3);
	if (!res) {
		res = lsm6dse_acc_reset_calibration(acc_obj);
		if (res) {
			ST_ERR("reset offset err = %d\n", res);
		}
	} else if (sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z) == 3) {
		dat[LSM6DSE_AXIS_X] = x;
		dat[LSM6DSE_AXIS_Y] = y;
		dat[LSM6DSE_AXIS_Z] = z;
		res = lsm6dse_acc_write_calibration(acc_obj, dat);
		if (res) {
			ST_ERR("write calibration err = %d\n", res);
		}
	} else {
		ST_ERR("invalid format\n");
	}

	return count;
}

static ssize_t lsm6dse_attr_acc_show_power_status(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct i2c_client *client = obj->client;
	u8 data;

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	lsm6dse_i2c_read_block(client, LSM6DSE_REG_CTRL1_XL, &data, 0x01);
	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t lsm6dse_attr_acc_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LSM6DSE_LOWPASS
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;

	if (atomic_read(&acc_obj->firlen)) {
		int idx, len = atomic_read(&acc_obj->firlen);

		ST_LOG("len = %2d, idx = %2d\n", acc_obj->fir.num, acc_obj->fir.idx);

		for (idx = 0; idx < len; idx++) {
			ST_LOG("[%5d %5d %5d]\n", acc_obj->fir.raw[idx][LSM6DSE_AXIS_X],
				acc_obj->fir.raw[idx][LSM6DSE_AXIS_Y], acc_obj->fir.raw[idx][LSM6DSE_AXIS_Z]);
		}

		ST_LOG("sum = [%5d %5d %5d]\n", acc_obj->fir.sum[LSM6DSE_AXIS_X],
			acc_obj->fir.sum[LSM6DSE_AXIS_Y], acc_obj->fir.sum[LSM6DSE_AXIS_Z]);
		ST_LOG("avg = [%5d %5d %5d]\n", acc_obj->fir.sum[LSM6DSE_AXIS_X]/len,
			acc_obj->fir.sum[LSM6DSE_AXIS_Y]/len, acc_obj->fir.sum[LSM6DSE_AXIS_Z]/len);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&acc_obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

static ssize_t lsm6dse_attr_acc_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LSM6DSE_LOWPASS
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int firlen;

	if (sscanf(buf, "%d", &firlen) != 1) {
		ST_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		ST_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&acc_obj->firlen, firlen);
		if (firlen == 0) {
			atomic_set(&acc_obj->fir_en, 0);
		} else {
			memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
			atomic_set(&acc_obj->fir_en, 1);
		}
	}
#endif
	return count;
}

static ssize_t lsm6dse_attr_acc_show_trace_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	ssize_t res;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));
	return res;
}

static ssize_t lsm6dse_attr_acc_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int trace;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (sscanf(buf, "0x%x", &trace) == 1) {
		atomic_set(&acc_obj->trace, trace);
	} else {
		ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}

	return count;
}

static ssize_t lsm6dse_attr_acc_show_status_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	ssize_t len = 0;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (acc_obj->lsm6dse_acc_hw) {
		len += snprintf(buf+len, PAGE_SIZE-len,
				"CUST: i2c_num=%d, direction=%d, sensitivity = %d, (power_id=%d, power_vol=%d)\n",
				acc_obj->lsm6dse_acc_hw->i2c_num,
				acc_obj->lsm6dse_acc_hw->direction, acc_obj->reso->sensitivity,
				acc_obj->lsm6dse_acc_hw->power_id, acc_obj->lsm6dse_acc_hw->power_vol);
		dumpReg(obj);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	return len;
}

static ssize_t lsm6dse_attr_acc_show_chipinit_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	ssize_t res;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));
	return res;
}

static ssize_t lsm6dse_attr_acc_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return count;
	}

	lsm6dse_acc_init(acc_obj, false);
	dumpReg(obj);

	return count;
}

static ssize_t lsm6dse_attr_acc_show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res = 0;

	if (obj == NULL) {
		ST_LOG("lsm6dse_acc is null!!\n");
		res = -1;
		return res;
	}

	return snprintf(buf, PAGE_SIZE, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
			acc_obj->lsm6dse_acc_hw->direction, acc_obj->cvt.sign[0], acc_obj->cvt.sign[1],
			acc_obj->cvt.sign[2], acc_obj->cvt.map[0], acc_obj->cvt.map[1], acc_obj->cvt.map[2]);
}

static ssize_t lsm6dse_attr_acc_store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int layout = 0;

	if (obj == NULL) {
		ST_ERR("lsm6dse_acc is null!!\n");
		return count;
	}

	if (sscanf(buf, "%d", &layout) == 1) {
		if (!hwmsen_get_convert(layout, &acc_obj->cvt)) {
			ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(acc_obj->lsm6dse_acc_hw->direction, &acc_obj->cvt)) {
			ST_LOG("invalid layout: %d, restore to %d\n", layout, acc_obj->lsm6dse_acc_hw->direction);
		} else {
			ST_ERR("invalid layout: (%d, %d)\n", layout, acc_obj->lsm6dse_acc_hw->direction);
			hwmsen_get_convert(0, &acc_obj->cvt);
		}
	} else {
		ST_LOG("invalid format = '%s'\n", buf);
	}

	return count;
}

static DRIVER_ATTR(chipinfo,             S_IRUGO, lsm6dse_attr_acc_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lsm6dse_attr_acc_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lsm6dse_attr_acc_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lsm6dse_attr_acc_show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, lsm6dse_attr_acc_show_cali_value,
		lsm6dse_attr_acc_store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, lsm6dse_attr_acc_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lsm6dse_attr_acc_show_firlen_value,
		lsm6dse_attr_acc_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lsm6dse_attr_acc_show_trace_value,
		lsm6dse_attr_acc_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lsm6dse_attr_acc_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lsm6dse_attr_acc_show_chipinit_value,
		lsm6dse_attr_acc_store_chipinit_value);
static DRIVER_ATTR(layout,     S_IRUGO | S_IWUSR, lsm6dse_attr_acc_show_layout_value,
		lsm6dse_attr_acc_store_layout_value);

static struct driver_attribute *lsm6dse_attr_acc_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_chipid,       /*chip id*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_rawdata,      /*dump sensor raw data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,        /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
};

int lsm6dse_acc_create_attr(struct device_driver *driver)
{
	int idx, res = 0;
	int num = (int)(ARRAY_SIZE(lsm6dse_attr_acc_list));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		res = driver_create_file(driver, lsm6dse_attr_acc_list[idx]);
		if (res) {
			ST_ERR("driver_create_file (%s) = %d\n", lsm6dse_attr_acc_list[idx]->attr.name, res);
			break;
		}
	}

	return res;
}

int lsm6dse_acc_delete_attr(struct device_driver *driver)
{
	int idx, res = 0;
	int num = (int)(ARRAY_SIZE(lsm6dse_attr_acc_list));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, lsm6dse_attr_acc_list[idx]);
	}

	return res;
}

/*if use this typ of enable, Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int lsm6dse_acc_open_report_data_intf(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return LSM6DSE_SUCCESS;
}

/* if use this typ of enable, Gsensor only enabled but not report inputEvent to HAL */
static int lsm6dse_acc_enable_nodata_intf(int en)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res = 0;
	bool power = false;

	if (en) {
		power = true;
	} else {
		power = false;
	}

	obj->acc_enabled = en;
	res = lsm6dse_acc_set_power_mode(acc_obj, power);
	if (res != LSM6DSE_SUCCESS) {
		ST_ERR("lsm6dse_acc_set_power_mode fail!\n");
		return res;
	}

	ST_LOG("lsm6dse_acc_enable_nodata_intf OK![%d]\n", en);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_set_delay_intf(u64 ns)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int ms = 0, odr = 0, res;

	ms = (int)ns/1000/1000;
	if (ms <= 5) {
		odr = LSM6DSE_REG_CTRL1_XL_ODR_208HZ;
	} else if (ms <= 10) {
		odr = LSM6DSE_REG_CTRL1_XL_ODR_104HZ;
	} else if (ms <= 20) {
		odr = LSM6DSE_REG_CTRL1_XL_ODR_52HZ;
	} else {
		odr = LSM6DSE_REG_CTRL1_XL_ODR_26HZ;
	}

	acc_obj->odr = odr;
	res = lsm6dse_acc_set_odr(acc_obj, acc_obj->odr);
	if (res != LSM6DSE_SUCCESS) {
		ST_ERR("Set delay parameter error!\n");
	}

#ifdef CONFIG_LSM6DSE_LOWPASS
	if (ms >= 50) {
		atomic_set(&acc_obj->filter, 0);
	} else {
		acc_obj->fir.num = 0;
		acc_obj->fir.idx = 0;
		acc_obj->fir.sum[LSM6DSE_AXIS_X] = 0;
		acc_obj->fir.sum[LSM6DSE_AXIS_Y] = 0;
		acc_obj->fir.sum[LSM6DSE_AXIS_Z] = 0;
		atomic_set(&acc_obj->filter, 1);
	}
#endif
	ST_LOG("lsm6dse_acc_set_delay_intf (%d)\n", ms);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_batch_intf(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return lsm6dse_acc_set_delay_intf((u64)samplingPeriodNs);
}

static int lsm6dse_acc_flush_intf(void)
{
	return acc_flush_report();
}

static int lsm6dse_acc_get_data_intf(int *x, int *y, int *z, int *status)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res = 0;
	u8 buff[LSM6DSE_BUFSIZE];

	lsm6dse_acc_read_data(acc_obj, buff, LSM6DSE_BUFSIZE);

	res = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_acc_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;

	if (file->private_data == NULL) {
		ST_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

static int lsm6dse_acc_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long lsm6dse_acc_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct lsm6dse_data *obj = (struct lsm6dse_data *)file->private_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	struct SENSOR_DATA sensor_data;
	u8 strbuf[LSM6DSE_BUFSIZE];
	void __user *data;
	long res = 0;
	int cali[3];
	struct acc_data offset_data;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		res = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	} else if (_IOC_DIR(cmd) & _IOC_WRITE) {
		res = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if (res) {
		ST_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		lsm6dse_acc_init(acc_obj, false);
		break;
	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		lsm6dse_acc_read_chip_name(acc_obj, strbuf, LSM6DSE_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			res = -EFAULT;
			break;
		}
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		lsm6dse_acc_read_data(acc_obj, strbuf, LSM6DSE_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			res = -EFAULT;
			break;
		}
		break;
	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
			res = -EFAULT;
			break;
		}
		break;
	case GSENSOR_IOCTL_READ_OFFSET:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D))) {
			res = -EFAULT;
			break;
		}
		break;
	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		lsm6dse_acc_read_rawdata_string(acc_obj, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf)+1)) {
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
		if (atomic_read(&acc_obj->suspend)) {
			ST_ERR("Perform calibration in suspend state!!\n");
			res = -EINVAL;
		} else {
			cali[LSM6DSE_AXIS_X] = sensor_data.x * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[LSM6DSE_AXIS_Y] = sensor_data.y * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[LSM6DSE_AXIS_Z] = sensor_data.z * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
			/*res = lsm6dse_acc_write_calibration(acc_obj, cali);*/
		}
		break;
	case GSENSOR_IOCTL_CLR_CALI:
		res = lsm6dse_acc_reset_calibration(acc_obj);
		break;
	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		res = lsm6dse_acc_read_calibration(acc_obj, cali);
		if (res) {
			break;
		}

		sensor_data.x = cali[LSM6DSE_AXIS_X] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
		sensor_data.y = cali[LSM6DSE_AXIS_Y] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
		sensor_data.z = cali[LSM6DSE_AXIS_Z] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			res = -EFAULT;
			break;
		}
		break;
	case GSENSOR_IOCTL_ENABLE_CALI:
		res = calc_acc_calibrate(cali);
		if (res < 0) {
			ST_ERR("%s, calc_acc_calibrate error\n", __func__);
			return res;
		}
		offset_data.x = cali[0];
		offset_data.y = cali[1];
		offset_data.z = cali[2];
		res = acc_cali_report(&offset_data);
		break;
	default:
		ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
		res = -ENOIOCTLCMD;
		break;
	}

	return res;
}

#ifdef CONFIG_COMPAT
static long lsm6dse_acc_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long res = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_INIT:
	case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
	case COMPAT_GSENSOR_IOCTL_READ_GAIN:
	case COMPAT_GSENSOR_IOCTL_READ_OFFSET:
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
	case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		res = filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
		if (res) {
			ST_ERR("unlocked_ioctl: 0x%x failed.", cmd);
			return res;
		}

	break;

	default:
		ST_ERR("unknown IOCTL: %s 0x%x\n", __func__, cmd);
		res = -ENOIOCTLCMD;
	break;
	}

	return res;
}
#endif
static int gsensor_set_cali(uint8_t *data, uint8_t count)
{
	int32_t *buf = (int32_t *)data;
	int error = 0;
	int offset[3] = {0};
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;

	if (count >= 5) {
		offset[LSM6DSE_AXIS_X] = buf[3];
		offset[LSM6DSE_AXIS_Y] = buf[4];
		offset[LSM6DSE_AXIS_Z] = buf[5];
	}
	ST_LOG("gsensor_set_cali %d %d %d\n",
		offset[LSM6DSE_AXIS_X], offset[LSM6DSE_AXIS_Y], offset[LSM6DSE_AXIS_Z]);
	lsm6dse_acc_reset_calibration(acc_obj);
	error = lsm6dse_acc_write_calibration(acc_obj, offset);
	return error;
}

static struct file_operations lsm6dse_acc_fops = {
	.owner    = THIS_MODULE,
	.open     = lsm6dse_acc_open,
	.release  = lsm6dse_acc_release,
	.unlocked_ioctl = lsm6dse_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl  = lsm6dse_acc_compat_ioctl,
#endif
};

static struct miscdevice lsm6dse_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "gsensor",
	.fops  = &lsm6dse_acc_fops,
};

static int lsm6dse_acc_local_init(void)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	struct i2c_client *client = obj->client;
	int res = 0, retry = 0;
	struct acc_control_path ctl = {0};
	struct acc_data_path data = {0};

	/*const u8 *name = "mediatek,lsm6dse_acc";*/
	ST_FUN();
	if (lsm6dse_module_init_flag == -1) {
		ST_LOG("lsm6dse module init fail, acc driver exit directly.\n");
		res = -1;
		return res;
	}
#if 0
	acc_obj->lsm6dse_acc_hw = get_accel_dts_func(name, &lsm6dse_acc_cust_hw);
	if (!acc_obj->lsm6dse_acc_hw) {
		ST_ERR("get lsm6dse dts info failed\n");
	}
#else
	res = get_accel_dts_func(client->dev.of_node, &lsm6dse_acc_cust_hw);
	if (res < 0) {
		ST_ERR("get dts info fail\n");
		return -EFAULT;
	}

	acc_obj->lsm6dse_acc_hw = &lsm6dse_acc_cust_hw;
#endif

	res = hwmsen_get_convert(acc_obj->lsm6dse_acc_hw->direction, &acc_obj->cvt);
	if (res) {
		ST_ERR("invalid direction: %d\n", acc_obj->lsm6dse_acc_hw->direction);
		goto exit;
	}

	atomic_set(&acc_obj->trace, 0);
	atomic_set(&acc_obj->suspend, 0);

#ifdef CONFIG_LSM6DSE_LOWPASS
	if (acc_obj->lsm6dse_acc_hw->firlen > C_MAX_FIR_LENGTH) {
		atomic_set(&acc_obj->firlen, C_MAX_FIR_LENGTH);
	} else {
		atomic_set(&acc_obj->firlen, acc_obj->lsm6dse_acc_hw->firlen);
	}

	if (atomic_read(&acc_obj->firlen) > 0) {
		atomic_set(&acc_obj->fir_en, 1);
	}
#endif

	lsm6dse_acc_reset_calibration(acc_obj);
	for (retry = 0; retry < 3; retry++) {
		res = lsm6dse_acc_init(acc_obj, false);
		if (res) {
			ST_ERR("lsm6dse_acc_device init cilent fail time: %d\n", retry);
			continue;
		} else
			break;
	}

	if (res != 0)
		goto exit_init_failed;

	snprintf(acc_obj->name, PAGE_SIZE, "%s_ACC", obj->name);

	res = misc_register(&lsm6dse_acc_device);
	if (res) {
		ST_ERR("lsm6dse_acc_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	res = lsm6dse_acc_create_attr(&(lsm6dse_acc_init_info.platform_diver_addr->driver));
	if (res) {
		ST_ERR("create attribute err = %d\n", res);
		goto exit_create_attr_failed;
	}
	create_acc_calibrate_proc_file();

	ctl.open_report_data       = lsm6dse_acc_open_report_data_intf;
	ctl.enable_nodata          = lsm6dse_acc_enable_nodata_intf;
	ctl.set_delay              = lsm6dse_acc_set_delay_intf;
	ctl.batch                  = lsm6dse_acc_batch_intf;
	ctl.flush                  = lsm6dse_acc_flush_intf;
	ctl.is_use_common_factory  = false;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch       = acc_obj->lsm6dse_acc_hw->is_batch_supported;
	ctl.set_cali               = gsensor_set_cali;

	res = acc_register_control_path(&ctl);
	if (res) {
		ST_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = lsm6dse_acc_get_data_intf;
	data.vender_div = 1000;
	res = acc_register_data_path(&data);
	if (res) {
		ST_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	ST_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&lsm6dse_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	ST_ERR("%s: err = %d\n", __func__, res);
	res = -1;
	return res;
}

static int lsm6dse_acc_local_remove(void)
{
	ST_FUN();
	misc_deregister(&lsm6dse_acc_device);
	lsm6dse_acc_delete_attr(&(lsm6dse_acc_init_info.platform_diver_addr->driver));

	return LSM6DSE_SUCCESS;
}

struct acc_init_info lsm6dse_acc_init_info = {
	.name   = "lsm6dse_acc",
	.init   = lsm6dse_acc_local_init,
	.uninit = lsm6dse_acc_local_remove,
};

MODULE_DESCRIPTION("STMicroelectronics lsm6dse driver");
MODULE_AUTHOR("Ian Yang, William Zeng");
MODULE_LICENSE("GPL v2");
