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

struct gyro_hw lsm6dse_gyro_cust_hw;

static struct data_resolution lsm6dse_gyro_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 0, 0}, 875},   /*245dps*/
	{{ 0, 0}, 1750},  /*500dps*/
	{{ 0, 0}, 3500},  /*1000dps*/
	{{ 0, 0}, 7000},  /*2000dps  1LSB=70mdps  here  mdps*100*/
};

/*For driver get cust info*/
struct gyro_hw *lsm6dse_get_cust_gyro_hw(void)
{
	return &lsm6dse_gyro_cust_hw;
}

static int lsm6dse_gyro_set_resolution(struct lsm6dse_gyro *gyro_obj)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;

	int res;
	u8  dat, reso;

	res = lsm6dse_i2c_read_block(client, LSM6DSE_REG_CTRL2_G, &dat, 0x01);
	if (res < 0) {
		ST_ERR("write data format fail!!\n");
		return res;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso = (dat & LSM6DSE_REG_CTRL2_G_MASK_FS_G) >> 2;
	if (reso >= 0x3)
		reso = 0x3;

	if (reso < ARRAY_SIZE(lsm6dse_gyro_data_resolution)) {
		gyro_obj->reso = &lsm6dse_gyro_data_resolution[reso];
		return LSM6DSE_SUCCESS;
	} else {
		return -EINVAL;
	}
}

static int lsm6dse_gyro_read_rawdata(struct lsm6dse_gyro *gyro_obj, s16 data[LSM6DSE_AXES_NUM])
{

	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;

	u8 buf[LSM6DSE_DATA_LEN] = {0};
	int res = 0;

	if (client == NULL) {
		res = -EINVAL;
	} else {
		if ((lsm6dse_i2c_read_block(client, LSM6DSE_REG_OUTX_L_G, buf, 0x06)) < 0) {
			ST_ERR("read  G sensor data register err!\n");
			res = -1;
			return res;
		}

		data[LSM6DSE_AXIS_X] = (s16)((buf[LSM6DSE_AXIS_X*2+1] << 8) | (buf[LSM6DSE_AXIS_X*2]));
		data[LSM6DSE_AXIS_Y] = (s16)((buf[LSM6DSE_AXIS_Y*2+1] << 8) | (buf[LSM6DSE_AXIS_Y*2]));
		data[LSM6DSE_AXIS_Z] = (s16)((buf[LSM6DSE_AXIS_Z*2+1] << 8) | (buf[LSM6DSE_AXIS_Z*2]));

		if (atomic_read(&gyro_obj->trace) & ADX_TRC_RAWDATA) {
			ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LSM6DSE_AXIS_X], data[LSM6DSE_AXIS_Y],
			data[LSM6DSE_AXIS_Z], data[LSM6DSE_AXIS_X], data[LSM6DSE_AXIS_Y], data[LSM6DSE_AXIS_Z]);
		}

#ifdef CONFIG_LSM6DSE_LOWPASS
		if (atomic_read(&gyro_obj->filter)) {
			if (atomic_read(&gyro_obj->fir_en) && !atomic_read(&gyro_obj->suspend)) {
				int idx, firlen = atomic_read(&gyro_obj->firlen);

				if (gyro_obj->fir.num < firlen) {
					gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_X] = data[LSM6DSE_AXIS_X];
					gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_Y] = data[LSM6DSE_AXIS_Y];
					gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_Z] = data[LSM6DSE_AXIS_Z];
					gyro_obj->fir.sum[LSM6DSE_AXIS_X] += data[LSM6DSE_AXIS_X];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Y] += data[LSM6DSE_AXIS_Y];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Z] += data[LSM6DSE_AXIS_Z];
					if (atomic_read(&gyro_obj->trace) & ADX_TRC_FILTER) {
						ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", gyro_obj->fir.num,
							gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_X],
							gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_Y],
							gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DSE_AXIS_Z],
							gyro_obj->fir.sum[LSM6DSE_AXIS_X],
							gyro_obj->fir.sum[LSM6DSE_AXIS_Y],
							gyro_obj->fir.sum[LSM6DSE_AXIS_Z]);
					}
					gyro_obj->fir.num++;
					gyro_obj->fir.idx++;
				} else {
					idx = gyro_obj->fir.idx % firlen;
					gyro_obj->fir.sum[LSM6DSE_AXIS_X] -= gyro_obj->fir.raw[idx][LSM6DSE_AXIS_X];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Y] -= gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Y];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Z] -= gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Z];
					gyro_obj->fir.raw[idx][LSM6DSE_AXIS_X] = data[LSM6DSE_AXIS_X];
					gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Y] = data[LSM6DSE_AXIS_Y];
					gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Z] = data[LSM6DSE_AXIS_Z];
					gyro_obj->fir.sum[LSM6DSE_AXIS_X] += data[LSM6DSE_AXIS_X];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Y] += data[LSM6DSE_AXIS_Y];
					gyro_obj->fir.sum[LSM6DSE_AXIS_Z] += data[LSM6DSE_AXIS_Z];
					gyro_obj->fir.idx++;
					data[LSM6DSE_AXIS_X] = gyro_obj->fir.sum[LSM6DSE_AXIS_X]/firlen;
					data[LSM6DSE_AXIS_Y] = gyro_obj->fir.sum[LSM6DSE_AXIS_Y]/firlen;
					data[LSM6DSE_AXIS_Z] = gyro_obj->fir.sum[LSM6DSE_AXIS_Z]/firlen;
					if (atomic_read(&gyro_obj->trace) & ADX_TRC_FILTER) {
						ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n",
							idx, gyro_obj->fir.raw[idx][LSM6DSE_AXIS_X],
							gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Y],
							gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Z],
							gyro_obj->fir.sum[LSM6DSE_AXIS_X],
							gyro_obj->fir.sum[LSM6DSE_AXIS_Y],
							gyro_obj->fir.sum[LSM6DSE_AXIS_Z], data[LSM6DSE_AXIS_X],
							data[LSM6DSE_AXIS_Y], data[LSM6DSE_AXIS_Z]);
					}
				}
			}
		}
#endif
	}

	return res;
}

static int lsm6dse_gyro_reset_calibration(struct lsm6dse_gyro *gyro_obj)
{
	memset(gyro_obj->cali_sw, 0x00, sizeof(gyro_obj->cali_sw));
	return 0;
}

static int lsm6dse_gyro_read_calibration(struct lsm6dse_gyro *gyro_obj, int dat[LSM6DSE_AXES_NUM])
{
	dat[gyro_obj->cvt.map[LSM6DSE_AXIS_X]] = gyro_obj->cvt.sign[LSM6DSE_AXIS_X]*gyro_obj->cali_sw[LSM6DSE_AXIS_X];
	dat[gyro_obj->cvt.map[LSM6DSE_AXIS_Y]] = gyro_obj->cvt.sign[LSM6DSE_AXIS_Y]*gyro_obj->cali_sw[LSM6DSE_AXIS_Y];
	dat[gyro_obj->cvt.map[LSM6DSE_AXIS_Z]] = gyro_obj->cvt.sign[LSM6DSE_AXIS_Z]*gyro_obj->cali_sw[LSM6DSE_AXIS_Z];

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_write_calibration(struct lsm6dse_gyro *gyro_obj, int dat[LSM6DSE_AXES_NUM])
{
	int res = 0;
	s16 cali[LSM6DSE_AXES_NUM];

	ST_FUN();

	if (!gyro_obj || !dat) {
		ST_ERR("null ptr!!\n");
		return -EINVAL;
	}

	cali[gyro_obj->cvt.map[LSM6DSE_AXIS_X]] =
		gyro_obj->cvt.sign[LSM6DSE_AXIS_X]*gyro_obj->cali_sw[LSM6DSE_AXIS_X];
	cali[gyro_obj->cvt.map[LSM6DSE_AXIS_Y]] =
		gyro_obj->cvt.sign[LSM6DSE_AXIS_Y]*gyro_obj->cali_sw[LSM6DSE_AXIS_Y];
	cali[gyro_obj->cvt.map[LSM6DSE_AXIS_Z]] =
		gyro_obj->cvt.sign[LSM6DSE_AXIS_Z]*gyro_obj->cali_sw[LSM6DSE_AXIS_Z];

	cali[LSM6DSE_AXIS_X] += dat[LSM6DSE_AXIS_X];
	cali[LSM6DSE_AXIS_Y] += dat[LSM6DSE_AXIS_Y];
	cali[LSM6DSE_AXIS_Z] += dat[LSM6DSE_AXIS_Z];

	gyro_obj->cali_sw[LSM6DSE_AXIS_X] +=
		gyro_obj->cvt.sign[LSM6DSE_AXIS_X]*cali[gyro_obj->cvt.map[LSM6DSE_AXIS_X]];
	gyro_obj->cali_sw[LSM6DSE_AXIS_Y] +=
		gyro_obj->cvt.sign[LSM6DSE_AXIS_Y]*cali[gyro_obj->cvt.map[LSM6DSE_AXIS_Y]];
	gyro_obj->cali_sw[LSM6DSE_AXIS_Z] +=
		gyro_obj->cvt.sign[LSM6DSE_AXIS_Z]*cali[gyro_obj->cvt.map[LSM6DSE_AXIS_Z]];

	return res;
}

static int lsm6dse_gyro_set_full_scale(struct lsm6dse_gyro *gyro_obj, u8 dataformat)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;
	int res = 0;

	res = lsm6dse_i2c_write_with_mask(client, LSM6DSE_REG_CTRL2_G, LSM6DSE_REG_CTRL2_G_MASK_FS_G, dataformat);
	if (res < 0) {
		ST_ERR("read reg_ctl_reg2 register err!\n");
		return LSM6DSE_ERR_I2C;
	}

	return lsm6dse_gyro_set_resolution(gyro_obj);
}

static int lsm6dse_gyro_set_odr(struct lsm6dse_gyro *gyro_obj, u8 odr)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;
	int res = 0;

	res = lsm6dse_i2c_write_with_mask(client, LSM6DSE_REG_CTRL2_G, LSM6DSE_REG_CTRL2_G_MASK_ODR_G, odr);
	if (res < 0) {
		return LSM6DSE_ERR_I2C;
	}

	return LSM6DSE_SUCCESS;
}

int lsm6dse_gyro_set_power_mode(struct lsm6dse_gyro *gyro_obj, bool state)
{
	int res = 0;

	if (state == gyro_obj->lsm6dse_gyro_power) {
		ST_LOG("Sensor power status is newest!\n");
		return LSM6DSE_SUCCESS;
	}

	if (state == true) {
		if (gyro_obj->odr == 0) {
			gyro_obj->odr = LSM6DSE_REG_CTRL2_G_ODR_104HZ;
		}
		res = lsm6dse_gyro_set_odr(gyro_obj, gyro_obj->odr);
	} else if (state == false) {
		res = lsm6dse_gyro_set_odr(gyro_obj, LSM6DSE_REG_CTRL2_G_ODR_0HZ);
	} else {
		ST_ERR("set power state error!\n");
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (res < 0) {
		ST_ERR("set power mode failed!\n");
		return LSM6DSE_ERR_I2C;
	} else if (atomic_read(&gyro_obj->trace) & ADX_TRC_INFO) {
		ST_LOG("set power mode ok %d!\n", state);
	}

	gyro_obj->lsm6dse_gyro_power = state;
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_init(struct lsm6dse_gyro *gyro_obj, bool enable)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;

	int res = 0;
	u8 buf[2] = {0, 0};

	ST_FUN();

	buf[0] = 0x00;
	res = lsm6dse_i2c_write_block(client, LSM6DSE_REG_CTRL2_G, buf, 0x01);
	if (res < 0) {
		ST_ERR("lsm6dse_gyro_init step 1!\n");
		return res;
	}

	res = lsm6dse_gyro_set_full_scale(gyro_obj, LSM6DSE_REG_CTRL2_G_FS_2000DPS);
	if (res < 0) {
		ST_ERR("lsm6dse_gyro_init step 3!\n");
		return res;
	}

	gyro_obj->odr = 0;
	res = lsm6dse_gyro_set_power_mode(gyro_obj, enable);
	if (res < 0) {
		ST_ERR("lsm6dse_gyro_init step 4!\n");
		return res;
	}


#ifdef CONFIG_LSM6DSE_LOWPASS
	memset(&gyro_obj->fir, 0x00, sizeof(gyro_obj->fir));
#endif
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_read_chip_name(struct lsm6dse_gyro *gyro_obj, u8 *buf, int bufsize)
{
	snprintf(buf, PAGE_SIZE, "%s", gyro_obj->name);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_read_data(struct lsm6dse_gyro *gyro_obj, u8 *data, int bufsize)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;
	int gyro[LSM6DSE_AXES_NUM];
	int gyro_final_data[LSM6DSE_AXES_NUM];
	int res = 0;

	if (data == NULL) {
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (client == NULL) {
		*data = 0;
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	if (atomic_read(&gyro_obj->suspend)) {
		ST_LOG("sensor in suspend read not data!\n");
		return 0;
	}

	if (gyro_obj->lsm6dse_gyro_power == false) {
		lsm6dse_gyro_set_power_mode(gyro_obj, true);
		/*TODO: need turn-on time*/
	}

	res = lsm6dse_gyro_read_rawdata(gyro_obj, gyro_obj->data);
	if (res) {
		ST_ERR("I2C error: ret value=%d", res);
		return res;
	}
	gyro[LSM6DSE_AXIS_X] =
			gyro_obj->data[LSM6DSE_AXIS_X]*gyro_obj->reso->sensitivity/100*131/1000;
	gyro[LSM6DSE_AXIS_Y] =
			gyro_obj->data[LSM6DSE_AXIS_Y]*gyro_obj->reso->sensitivity/100*131/1000;
	gyro[LSM6DSE_AXIS_Z] =
			gyro_obj->data[LSM6DSE_AXIS_Z]*gyro_obj->reso->sensitivity/100*131/1000;

	gyro[LSM6DSE_AXIS_X] += gyro_obj->cali_sw[LSM6DSE_AXIS_X];
	gyro[LSM6DSE_AXIS_Y] += gyro_obj->cali_sw[LSM6DSE_AXIS_Y];
	gyro[LSM6DSE_AXIS_Z] += gyro_obj->cali_sw[LSM6DSE_AXIS_Z];

	/*remap coordinate*/
	gyro_final_data[gyro_obj->cvt.map[LSM6DSE_AXIS_X]] =
			gyro_obj->cvt.sign[LSM6DSE_AXIS_X]*gyro[LSM6DSE_AXIS_X];
	gyro_final_data[gyro_obj->cvt.map[LSM6DSE_AXIS_Y]] =
			gyro_obj->cvt.sign[LSM6DSE_AXIS_Y]*gyro[LSM6DSE_AXIS_Y];
	gyro_final_data[gyro_obj->cvt.map[LSM6DSE_AXIS_Z]] =
			gyro_obj->cvt.sign[LSM6DSE_AXIS_Z]*gyro[LSM6DSE_AXIS_Z];

	snprintf(data, PAGE_SIZE, "%04x %04x %04x",
			gyro_final_data[LSM6DSE_AXIS_X],
			gyro_final_data[LSM6DSE_AXIS_Y],
			gyro_final_data[LSM6DSE_AXIS_Z]);
	if (atomic_read(&gyro_obj->trace) & ADX_TRC_IOCTL) {
		ST_LOG("gyroscope data: %s!\n", data);
		dumpReg(obj);
	}

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_read_rawdata_string(struct lsm6dse_gyro *gyro_obj, u8 *buf)
{
	struct lsm6dse_data *obj = container_of(gyro_obj, struct lsm6dse_data, lsm6dse_gyro_data);
	struct i2c_client *client = obj->client;
	int res = 0;

	if (!buf || !client) {
		return LSM6DSE_ERR_SETUP_FAILURE;
	}

	res = lsm6dse_gyro_read_rawdata(gyro_obj, gyro_obj->data);
	if (res) {
		ST_ERR("I2C error: ret value=%d", res);
		return res;
	}
	snprintf(buf, PAGE_SIZE, "%04x %04x %04x", gyro_obj->data[LSM6DSE_AXIS_X],
			gyro_obj->data[LSM6DSE_AXIS_Y], gyro_obj->data[LSM6DSE_AXIS_Z]);

	return LSM6DSE_SUCCESS;
}

static ssize_t lsm6dse_attr_gyro_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	u8 strbuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	lsm6dse_gyro_read_chip_name(gyro_obj, strbuf, LSM6DSE_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t lsm6dse_attr_gyro_show_chipid_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "0x%x\n", obj->chip_id);
}

static ssize_t lsm6dse_attr_gyro_show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	u8 databuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	lsm6dse_gyro_read_data(gyro_obj, databuf, LSM6DSE_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lsm6dse_attr_gyro_show_rawdata_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	u8 databuf[LSM6DSE_BUFSIZE];

	if (obj->client == NULL) {
		ST_ERR("i2c client is null!!\n");
		return 0;
	}

	lsm6dse_gyro_read_rawdata_string(gyro_obj, databuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}

static ssize_t lsm6dse_attr_gyro_show_power_status(struct device_driver *ddri, char *buf)
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

static ssize_t lsm6dse_attr_gyro_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LSM6DSE_LOWPASS
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;

	if (atomic_read(&gyro_obj->firlen)) {
		int idx, len = atomic_read(&gyro_obj->firlen);

		ST_LOG("len = %2d, idx = %2d\n", gyro_obj->fir.num, gyro_obj->fir.idx);

		for (idx = 0; idx < len; idx++) {
			ST_LOG("[%5d %5d %5d]\n", gyro_obj->fir.raw[idx][LSM6DSE_AXIS_X],
				gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Y], gyro_obj->fir.raw[idx][LSM6DSE_AXIS_Z]);
		}

		ST_LOG("sum = [%5d %5d %5d]\n", gyro_obj->fir.sum[LSM6DSE_AXIS_X], gyro_obj->fir.sum[LSM6DSE_AXIS_Y],
			gyro_obj->fir.sum[LSM6DSE_AXIS_Z]);
		ST_LOG("avg = [%5d %5d %5d]\n",
			gyro_obj->fir.sum[LSM6DSE_AXIS_X]/len, gyro_obj->fir.sum[LSM6DSE_AXIS_Y]/len,
			gyro_obj->fir.sum[LSM6DSE_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gyro_obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

static ssize_t lsm6dse_attr_gyro_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LSM6DSE_LOWPASS
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int firlen;

	if (sscanf(buf, "%d", &firlen) != 1) {
		ST_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		ST_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&gyro_obj->firlen, firlen);
		if (firlen == 0) {
			atomic_set(&gyro_obj->fir_en, 0);
		} else {
			memset(&gyro_obj->fir, 0x00, sizeof(gyro_obj->fir));
			atomic_set(&gyro_obj->fir_en, 1);
		}
	}
#endif
	return count;
}

static ssize_t lsm6dse_attr_gyro_show_trace_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	ssize_t res;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&gyro_obj->trace));
	return res;
}

static ssize_t lsm6dse_attr_gyro_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int trace;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (sscanf(buf, "0x%x", &trace) == 1) {
		atomic_set(&gyro_obj->trace, trace);
	} else {
		ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}

	return count;
}

static ssize_t lsm6dse_attr_gyro_show_status_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	ssize_t len = 0;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (gyro_obj->lsm6dse_gyro_hw) {
		len += snprintf(buf+len, PAGE_SIZE-len,
				"CUST:i2c_num=%d,direction=%d,sensitivity = %d,(power_id=%d,power_vol=%d)\n",
				gyro_obj->lsm6dse_gyro_hw->i2c_num, gyro_obj->lsm6dse_gyro_hw->direction,
				gyro_obj->reso->sensitivity, gyro_obj->lsm6dse_gyro_hw->power_id,
				gyro_obj->lsm6dse_gyro_hw->power_vol);
		dumpReg(obj);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t lsm6dse_attr_gyro_show_chipinit_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	ssize_t res;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&gyro_obj->trace));
	return res;
}

static ssize_t lsm6dse_attr_gyro_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;

	if (obj == NULL) {
		ST_ERR("i2c_data obj is null!!\n");
		return count;
	}

	lsm6dse_gyro_init(gyro_obj, false);
	dumpReg(obj);

	return count;
}

static ssize_t lsm6dse_attr_gyro_show_layout_value(struct device_driver *ddri, char *buf)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int res = 0;

	if (obj == NULL) {
		ST_LOG("lsm6dse_gyro is null!!\n");
		res = -1;
		return res;
	}

	return snprintf(buf, PAGE_SIZE, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
				gyro_obj->lsm6dse_gyro_hw->direction,
				gyro_obj->cvt.sign[0], gyro_obj->cvt.sign[1],
				gyro_obj->cvt.sign[2], gyro_obj->cvt.map[0],
				gyro_obj->cvt.map[1], gyro_obj->cvt.map[2]);
}

static ssize_t lsm6dse_attr_gyro_store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int layout = 0;

	if (obj == NULL) {
		ST_ERR("lsm6dse_gyro is null!!\n");
		return count;
	}

	if (sscanf(buf, "%d", &layout) == 1) {
		if (!hwmsen_get_convert(layout, &gyro_obj->cvt)) {
			ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(gyro_obj->lsm6dse_gyro_hw->direction, &gyro_obj->cvt)) {
			ST_LOG("invalid layout: %d, restore to %d\n", layout, gyro_obj->lsm6dse_gyro_hw->direction);
		} else {
			ST_ERR("invalid layout: (%d, %d)\n", layout, gyro_obj->lsm6dse_gyro_hw->direction);
			hwmsen_get_convert(0, &gyro_obj->cvt);
		}
	} else {
		ST_LOG("invalid format = '%s'\n", buf);
	}

	return count;
}

static DRIVER_ATTR(chipinfo,             S_IRUGO, lsm6dse_attr_gyro_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lsm6dse_attr_gyro_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lsm6dse_attr_gyro_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lsm6dse_attr_gyro_show_sensordata_value,    NULL);
/*static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, lsm6dse_attr_gyro_show_cali_value,
								lsm6dse_attr_gyro_store_cali_value);*/
static DRIVER_ATTR(power,                S_IRUGO, lsm6dse_attr_gyro_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lsm6dse_attr_gyro_show_firlen_value,
								lsm6dse_attr_gyro_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lsm6dse_attr_gyro_show_trace_value,
								lsm6dse_attr_gyro_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lsm6dse_attr_gyro_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lsm6dse_attr_gyro_show_chipinit_value,
								lsm6dse_attr_gyro_store_chipinit_value);
static DRIVER_ATTR(layout,     S_IRUGO | S_IWUSR, lsm6dse_attr_gyro_show_layout_value,
								lsm6dse_attr_gyro_store_layout_value);

static struct driver_attribute *lsm6dse_attr_gyro_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_chipid,       /*chip id*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_rawdata,      /*dump sensor raw data*/
	/*&driver_attr_cali,         show calibration data*/
	&driver_attr_power,        /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
};

int lsm6dse_gyro_create_attr(struct device_driver *driver)
{
	int idx, res = 0;
	int num = (int)(ARRAY_SIZE(lsm6dse_attr_gyro_list));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		res = driver_create_file(driver, lsm6dse_attr_gyro_list[idx]);
		if (res) {
			ST_ERR("driver_create_file (%s) = %d\n", lsm6dse_attr_gyro_list[idx]->attr.name, res);
			break;
		}
	}

	return res;
}

int lsm6dse_gyro_delete_attr(struct device_driver *driver)
{
	int idx, res = 0;
	int num = (int)(ARRAY_SIZE(lsm6dse_attr_gyro_list));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, lsm6dse_attr_gyro_list[idx]);
	}

	return res;
}

/*if use this typ of enable, Gsensor should report inputEvent(x, y, z ,stats, div) to HAL*/
static int lsm6dse_gyro_open_report_data_intf(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true */
	return LSM6DSE_SUCCESS;
}

/*if use this typ of enable, Gsensor only enabled but not report inputEvent to HAL*/
static int lsm6dse_gyro_enable_nodata_intf(int en)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int res = 0;
	bool power = false;

	if (en) {
		power = true;
	} else {
		power = false;
	}

	obj->gyro_enabled = en;
	res = lsm6dse_gyro_set_power_mode(gyro_obj, power);
	if (res != LSM6DSE_SUCCESS) {
		ST_ERR("lsm6dse_gyro_set_power_mode fail!\n");
		return res;
	}

	ST_LOG("lsm6dse_gyro_enable_nodata_intf OK![%d]\n", en);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_set_delay_intf(u64 ns)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	int value = 0;
	int sample_delay = 0;
	int res;

	value = (int)ns/1000/1000;
	if (value <= 5) {
		sample_delay = LSM6DSE_REG_CTRL2_G_ODR_208HZ;
	} else if (value <= 10) {
		sample_delay = LSM6DSE_REG_CTRL2_G_ODR_104HZ;
	} else if (value <= 20) {
		sample_delay = LSM6DSE_REG_CTRL2_G_ODR_52HZ;
	} else {
		sample_delay = LSM6DSE_REG_CTRL2_G_ODR_26HZ;
	}

	gyro_obj->odr = sample_delay;
	res = lsm6dse_gyro_set_odr(gyro_obj, gyro_obj->odr);
	/*0x2C->BW=100Hz*/
	if (res != LSM6DSE_SUCCESS) {
		ST_ERR("Set delay parameter error!\n");
	}
#ifdef CONFIG_LSM6DSE_LOWPASS
	if (value >= 50) {
		atomic_set(&gyro_obj->filter, 0);
	} else {
		gyro_obj->fir.num = 0;
		gyro_obj->fir.idx = 0;
		gyro_obj->fir.sum[LSM6DSE_AXIS_X] = 0;
		gyro_obj->fir.sum[LSM6DSE_AXIS_Y] = 0;
		gyro_obj->fir.sum[LSM6DSE_AXIS_Z] = 0;
		atomic_set(&gyro_obj->filter, 1);
	}
#endif
	/* sampling over than 20hz, delay 100ms for stable(VTS).*/
	if (value <= 50) {
		msleep(100);
	}
	ST_LOG("lsm6dse_gyro_set_delay_intf (%d)\n", value);
	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_batch_intf(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return lsm6dse_gyro_set_delay_intf((u64)samplingPeriodNs);
}

static int lsm6dse_gyro_flush_intf(void)
{
	return gyro_flush_report();
}

static int lsm6dse_gyro_get_data_intf(int *x, int *y, int *z, int *status)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	u8 buff[LSM6DSE_BUFSIZE];
	int res = 0;

	lsm6dse_gyro_read_data(gyro_obj, buff, LSM6DSE_BUFSIZE);
	res = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return LSM6DSE_SUCCESS;
}

static int lsm6dse_gyro_open(struct inode *inode, struct file *file)
{
	file->private_data = obj_i2c_data;

	if (file->private_data == NULL) {
		ST_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int lsm6dse_gyro_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long lsm6dse_gyro_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct lsm6dse_data *obj = (struct lsm6dse_data *)file->private_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	char strbuf[LSM6DSE_BUFSIZE] = {0};
	void __user *data;
	long res = 0;
	int copy_cnt = 0;
	struct SENSOR_DATA sensor_data;
	int cali[3] = {0};
	int smtRes = 0;

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
	case GYROSCOPE_IOCTL_INIT:
		lsm6dse_gyro_init(gyro_obj, false);
	break;
	case GYROSCOPE_IOCTL_SMT_DATA:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		ST_LOG("IOCTL smtRes: %d!\n", smtRes);
		copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));

		if (copy_cnt) {
			res = -EFAULT;
			ST_ERR("copy gyro data to user failed!\n");
		}
		ST_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
	break;
	case GYROSCOPE_IOCTL_READ_SENSORDATA:
		data = (void __user *) arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}

		lsm6dse_gyro_read_data(gyro_obj, strbuf, LSM6DSE_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf)+1)) {
			res = -EFAULT;
			break;
		}
	break;
	case GYROSCOPE_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			res = -EFAULT;
			break;
		}
		cali[LSM6DSE_AXIS_X] = (s64)(sensor_data.x);
		cali[LSM6DSE_AXIS_Y] = (s64)(sensor_data.y);
		cali[LSM6DSE_AXIS_Z] = (s64)(sensor_data.z);
		res = lsm6dse_gyro_write_calibration(gyro_obj, cali);
	break;
	case GYROSCOPE_IOCTL_CLR_CALI:
		res = lsm6dse_gyro_reset_calibration(gyro_obj);
		break;

	case GYROSCOPE_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			res = -EINVAL;
			break;
		}
		res = lsm6dse_gyro_read_calibration(gyro_obj, cali);
		if (res) {
			break;
		}

		sensor_data.x = (s64)(cali[LSM6DSE_AXIS_X]);
		sensor_data.y = (s64)(cali[LSM6DSE_AXIS_Y]);
		sensor_data.z = (s64)(cali[LSM6DSE_AXIS_Z]);

		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			res = -EFAULT;
			break;
		}
		break;
	case GYROSCOPE_IOCTL_ENABLE_CALI:
		ST_LOG("GYROSCOPE_IOCTL_ENABLE_CALI\n");
		break;
	default:
		ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
		res = -ENOIOCTLCMD;
	break;
	}

	return res;
}

#ifdef CONFIG_COMPAT
static long lsm6dse_gyro_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long res = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GYROSCOPE_IOCTL_INIT:
	case COMPAT_GYROSCOPE_IOCTL_SMT_DATA:
	case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
	case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
	case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
	case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
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

static struct file_operations lsm6dse_gyro_fops = {
	.owner          = THIS_MODULE,
	.open           = lsm6dse_gyro_open,
	.release        = lsm6dse_gyro_release,
	.unlocked_ioctl = lsm6dse_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = lsm6dse_gyro_compat_ioctl,
#endif
};

static struct miscdevice lsm6dse_gyro_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "gyroscope",
	.fops  = &lsm6dse_gyro_fops,
};

static int lsm6dse_gyro_local_init(struct platform_device *pdev)
{
	struct lsm6dse_data *obj = obj_i2c_data;
	struct lsm6dse_gyro *gyro_obj = &obj->lsm6dse_gyro_data;
	struct lsm6dse_acc *acc_obj = &obj->lsm6dse_acc_data;
	int res = 0, retry = 0;
	struct gyro_control_path ctl = {0};
	struct gyro_data_path data = {0};
	/*const u8 *name = "mediatek,lsm6dse_gyro";*/

	ST_FUN();
	if (lsm6dse_module_init_flag == -1) {
		ST_LOG("lsm6dse module init fail, gyro driver exit directly.\n");
		res = -1;
		return res;
	}

	gyro_obj->lsm6dse_gyro_hw = &lsm6dse_gyro_cust_hw;
	gyro_obj->lsm6dse_gyro_hw->direction = 6;
	gyro_obj->lsm6dse_gyro_hw->firlen = 3;
	if (acc_obj->lsm6dse_acc_hw != NULL) {
		gyro_obj->lsm6dse_gyro_hw->direction = acc_obj->lsm6dse_acc_hw->direction;
		gyro_obj->lsm6dse_gyro_hw->firlen = acc_obj->lsm6dse_acc_hw->firlen;
	} else {
		ST_ERR("invalid lsm6dse_acc_hw: NULL\n");
	}

	res = hwmsen_get_convert(gyro_obj->lsm6dse_gyro_hw->direction, &gyro_obj->cvt);
	if (res) {
		ST_ERR("invalid direction: %d\n", gyro_obj->lsm6dse_gyro_hw->direction);
		goto exit;
	}
	atomic_set(&gyro_obj->trace, 0);
	atomic_set(&gyro_obj->suspend, 0);

#ifdef CONFIG_LSM6DSE_LOWPASS
	if (gyro_obj->lsm6dse_gyro_hw->firlen > C_MAX_FIR_LENGTH) {
		atomic_set(&gyro_obj->firlen, C_MAX_FIR_LENGTH);
	} else {
		atomic_set(&gyro_obj->firlen, gyro_obj->lsm6dse_gyro_hw->firlen);
	}

	if (atomic_read(&gyro_obj->firlen) > 0) {
		atomic_set(&gyro_obj->fir_en, 1);
	}
#endif

	lsm6dse_gyro_reset_calibration(gyro_obj);

	for (retry = 0; retry < 3; retry++) {
		res = lsm6dse_gyro_init(gyro_obj, false);
		if (res) {
			ST_ERR("lsm6dse_gyro_device init cilent fail time: %d\n", retry);
			continue;
		} else
			break;
	}
	if (res != 0)
		goto exit_init_failed;

	snprintf(gyro_obj->name, PAGE_SIZE, "%s_GYRO", obj->name);

	res = misc_register(&lsm6dse_gyro_device);
	if (res) {
		ST_ERR("lsm6dse_gyro_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	res = lsm6dse_gyro_create_attr(&(lsm6dse_gyro_init_info.platform_diver_addr->driver));
	if (res) {
		ST_ERR("create attribute err = %d\n", res);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data       = lsm6dse_gyro_open_report_data_intf;
	ctl.enable_nodata          = lsm6dse_gyro_enable_nodata_intf;
	ctl.set_delay              = lsm6dse_gyro_set_delay_intf;
	ctl.batch                  = lsm6dse_gyro_batch_intf;
	ctl.flush                  = lsm6dse_gyro_flush_intf;
	ctl.is_use_common_factory  = false;
	ctl.is_report_input_direct = false;

	if (acc_obj->lsm6dse_acc_hw != NULL) {
		ctl.is_support_batch = acc_obj->lsm6dse_acc_hw->is_batch_supported;
	} else
		ctl.is_support_batch = 0;

	res = gyro_register_control_path(&ctl);
	if (res) {
		ST_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data   = lsm6dse_gyro_get_data_intf;
	data.vender_div = DEGREE_TO_RAD;
	res = gyro_register_data_path(&data);
	if (res) {
		ST_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	ST_LOG("%s: OK\n", __func__);
	return LSM6DSE_SUCCESS;

exit_create_attr_failed:
	misc_deregister(&lsm6dse_gyro_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	ST_ERR("%s: err = %d\n", __func__, res);
	res = -1;
	return res;
}

static int lsm6dse_gyro_local_remove(void)
{
	ST_FUN();
	misc_deregister(&lsm6dse_gyro_device);
	lsm6dse_gyro_delete_attr(&(lsm6dse_gyro_init_info.platform_diver_addr->driver));

	return LSM6DSE_SUCCESS;
}

struct gyro_init_info lsm6dse_gyro_init_info = {
	.name   = "lsm6dse_gyro",
	.init   = lsm6dse_gyro_local_init,
	.uninit = lsm6dse_gyro_local_remove,
};

MODULE_DESCRIPTION("STMicroelectronics lsm6dse driver");
MODULE_AUTHOR("Ian Yang, William Zeng");
MODULE_LICENSE("GPL v2");
