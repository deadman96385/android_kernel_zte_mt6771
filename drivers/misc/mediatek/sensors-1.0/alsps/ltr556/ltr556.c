/*
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

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>

#include "cust_alsps.h"
#include "ltr556.h"
#include "alsps.h"

#define GN_MTK_BSP_PS_DYNAMIC_CALI
/*#define DELAYED_PS_CALI*/
/*#define DEMO_BOARD*/

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR556_DEV_NAME			"ltr556"

/*----------------------------------------------------------------------------*/
#define APS_TAG                 "[ALS/PS] "
#define APS_FUN(f)              pr_info(APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)   pr_err(APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   pr_notice(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   pr_err(APS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr556_i2c_id[] = { {LTR556_DEV_NAME, 0}, {} };
static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;

/*----------------------------------------------------------------------------*/
static int ltr556_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ltr556_i2c_remove(struct i2c_client *client);
static int ltr556_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltr556_i2c_suspend(struct device *dev);
static int ltr556_i2c_resume(struct device *dev);

static int ltr556_calibrate_zte(void);

/*static int ps_gainrange;*/
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/*add by shengxia for ztecalips*/
static int ltr556_ps_startup_cross_talk = 0;
static int ltr556_ps_startup_calibration = 0;
static u16 ltr556_ps_cross_talk = 0;
static u16 prox_threshold_hi_param = 0;
static u16 prox_threshold_lo_param = 0;
static int als_value_backup = -1;
static int lux_backup = -1;

/*----------------------------------------------------------------------------*/
typedef enum {
	CMC_BIT_ALS    = 1,
	CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr556_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;
	struct delayed_work cali_ps_work;

	/*misc*/
	u16			als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on;		/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end;		/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t	trace;

#ifdef CONFIG_OF
	struct device_node *irq_node;
	int		irq;
#endif

	/*data*/
	u16			als;
	u16			ps;
	u8			_align;
	u16			als_level_num;
	u16			als_value_num;
	u32			als_level[C_CUST_ALS_LEVEL-1];
	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low;		/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_high;	/*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable;			/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
};

struct PS_CALI_DATA_STRUCT {
	int close;
	int far_away;
	int valid;
};

static struct PS_CALI_DATA_STRUCT ps_cali = {0, 0, 0};
static int intr_flag_value = 0;

static struct ltr556_priv *ltr556_obj = NULL;
static struct i2c_client *ltr556_i2c_client = NULL;

static DEFINE_MUTEX(ltr556_mutex);

static int ltr556_local_init(void);
static int ltr556_remove(void);
static int ltr556_init_flag =  -1;

static int irq_enabled = 0;

static struct alsps_init_info ltr556_init_info = {
		.name = "ltr556",
		.init = ltr556_local_init,
		.uninit = ltr556_remove,
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr556_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr556_i2c_suspend, ltr556_i2c_resume)
};
#endif

static struct i2c_driver ltr556_i2c_driver = {
	.probe      = ltr556_i2c_probe,
	.remove     = ltr556_i2c_remove,
	.detect     = ltr556_i2c_detect,
	.id_table   = ltr556_i2c_id,
	.driver = {
		.name           = LTR556_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif

#ifdef CONFIG_PM_SLEEP
	.pm = &ltr556_pm_ops,
#endif
	},
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr556_dynamic_calibrate(void);
static int dynamic_calibrate = 0;
#endif
/*-----------------------------------------------------------------------------*/

/*
 * #########
 * ## I2C ##
 * #########
 */

static int ltr556_i2c_read_reg(u8 regnum)
{
	u8 buffer[1], reg_value[1];
	int res = 0;

	mutex_lock(&ltr556_mutex);
	buffer[0] = regnum;
	res = i2c_master_send(ltr556_obj->client, buffer, 0x1);
	if (res <= 0) {
	    APS_ERR("read reg send res = %d\n", res);
	    mutex_unlock(&ltr556_mutex);
		return res;
	}
	res = i2c_master_recv(ltr556_obj->client, reg_value, 0x1);
	if (res <= 0) {
		APS_ERR("read reg recv res = %d\n", res);
		mutex_unlock(&ltr556_mutex);
		return res;
	}
	mutex_unlock(&ltr556_mutex);
	return reg_value[0];
}

static int ltr556_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];
	int res = 0;

	mutex_lock(&ltr556_mutex);
	databuf[0] = regnum;
	databuf[1] = value;
	res = i2c_master_send(ltr556_obj->client, databuf, 0x2);
	mutex_unlock(&ltr556_mutex);

	if (res < 0) {
		APS_ERR("write reg send res = %d\n", res);
		return res;
	} else
		return 0;
}

/*----------------------------------------------------------------------------*/
static void ltr556_power(struct alsps_hw *hw, unsigned int on)
{
#ifdef DEMO_BOARD
	static unsigned int power_on = 0;

	if (hw->power_id != POWER_NONE_MACRO) {
		if (power_on == on) {
			APS_LOG("ignore power control: %d\n", on);
		} else if (on) {
			if (!hwPowerOn(hw->power_id, hw->power_vol, "ltr556")) {
				APS_ERR("power on fails!!\n");
			}
		} else {
			if (!hwPowerDown(hw->power_id, "ltr556")) {
				APS_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif
}
/********************************************************************/
/*
 * ###############
 * ## PS CONFIG ##
 * ###############

 */
static int ltr556_ps_set_thres(void)
{
	int res;
	u8 databuf[2];

	struct i2c_client *client = ltr556_obj->client;
	struct ltr556_priv *obj = ltr556_obj;

	APS_FUN();

	APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);

	if (ps_cali.valid == 1) {
		databuf[0] = LTR556_PS_THRES_LOW_0;
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_LOW_1;
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_UP_0;
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_UP_1;
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
	} else {
		databuf[0] = LTR556_PS_THRES_LOW_0;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_LOW_1;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) >> 8) & 0x00FF);

		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_UP_0;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = LTR556_PS_THRES_UP_1;
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
	}

	res = 1;
	return res;

EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

static int ltr556_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata = 0x0;
	int err;
	int res;
	u8 databuf[2];

	APS_LOG("ltr556_ps_enable() ...start!\n");

	regdata = ltr556_i2c_read_reg(LTR556_PS_CONTR);
	APS_LOG("LTR556_PS_CONTR beforeset= %d\n", regdata);
	if (enable != 0) {
		APS_LOG("PS: enable ps only\n");
		regdata = 0x23; /* MODE_PS_Gain16 | 0x03, ps_gain_16 | PS_Saturation_Indicator_Enable */
	} else {
		APS_LOG("PS: disable ps only\n");
		regdata = 0x0;
	}

	err = ltr556_i2c_write_reg(LTR556_PS_CONTR, regdata);
	if (err < 0) {
		APS_ERR("PS: enable ps err: %d en: %d\n", err, enable);
		return err;
	}
	APS_LOG("LTR556_PS_CONTR afterset= %d\n", regdata);
	mdelay(WAKEUP_DELAY);
	/*ltr556_i2c_read_reg(LTR556_PS_CONTR);*/

	if (ltr556_obj->hw->polling_mode_ps == 0 && enable != 0) {
		APS_LOG("eint enable");

		ltr556_calibrate_zte();

		/* ltr556_ps_set_thres();*/

		databuf[0] = LTR556_INTERRUPT;
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		databuf[0] = LTR556_INTERRUPT_PERSIST;
		databuf[1] = 0x10;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
#ifndef DELAYED_PS_CALI
#if 0
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
		err = ltr556_dynamic_calibrate();
		if (err < 0) {
			APS_LOG("ltr556_dynamic_calibrate() failed\n");
		}
#endif
		ltr556_ps_set_thres();
#endif
#else
		cancel_delayed_work(&ltr556_obj->cali_ps_work);
		schedule_delayed_work(&ltr556_obj->cali_ps_work, msecs_to_jiffies(200));
#endif
	} else if (ltr556_obj->hw->polling_mode_ps == 0 && enable == 0) {
		/*ancel_work_sync(&ltr556_obj->eint_work);*/
	}

	if ((irq_enabled == 1) && (enable != 0)) {
		irq_enabled = 2;
		enable_irq(ltr556_obj->irq);
	}
	APS_LOG("ltr556_ps_enable ...OK!\n");

	return err;
EXIT_ERR:
	APS_ERR("error set thres: %d\n", res);
	return res;

}

/********************************************************************/
static int ltr556_ps_read(struct i2c_client *client, u16 *data)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr556_i2c_read_reg(LTR556_PS_DATA_0);
	APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0) {
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}

	psval_hi = ltr556_i2c_read_reg(LTR556_PS_DATA_1);
	APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);
	if (psval_hi < 0) {
		APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	if ((psval_hi & 0x80) == 0x80) {/* PS_Saturation */
		APS_DBG("%s ps saturated! psval_lo=%d psval_hi=%d\n", __func__, psval_lo, psval_hi);
		goto out;
	}

	psdata = ((psval_hi & 7) * 256) + psval_lo;
	*data = psdata;
	APS_DBG("ltr556_ps_read: ps_rawdata = %d\n", psdata);

out:
	final_prox_val = psdata;
	*data = 0;
	return psdata;
}

#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr556_dynamic_calibrate(void)
{
	int i = 0;
	int data;
	int data_total = 0;
	int noise = 0;
	int count = 5;
	int ps_thd_val_low, ps_thd_val_high;
	struct ltr556_priv *obj = ltr556_obj;
	int res = 0;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		/*len = sprintf(buf, "ltr556_obj is null\n");*/
		res = -1;
		return res;
	}

	for (i = 0; i < count; i++) {
		/* wait for ps value be stable*/
		msleep(120);

		data = ltr556_ps_read(ltr556_obj->client, &ltr556_obj->ps);
		if (data < 0) {
			i--;
			continue;
		}

		if (data & 0x8000) {
			break;
		}

		data_total += data;
	}

	noise = data_total / count;
	dynamic_calibrate = noise;

	if (noise < 100) {
		ps_thd_val_high = noise + 100;
		ps_thd_val_low  = noise + 50;
	} else if (noise < 200) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low  = noise + 60;
	} else if (noise < 300) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low  = noise + 60;
	} else if (noise < 400) {
		ps_thd_val_high = noise + 150;
		ps_thd_val_low  = noise + 60;
	} else if (noise < 600) {
		ps_thd_val_high = noise + 180;
		ps_thd_val_low  = noise + 90;
	} else if (noise < 1000) {
		ps_thd_val_high = noise + 300;
		ps_thd_val_low  = noise + 180;
	} else if (noise < 1250) {
		ps_thd_val_high = noise + 400;
		ps_thd_val_low  = noise + 300;
	} else {
		ps_thd_val_high = 1600;
		ps_thd_val_low  = 1400;
		APS_ERR("dynamic calibrate fails!!\n");
	}

	atomic_set(&obj->ps_thd_val_high, ps_thd_val_high);
	atomic_set(&obj->ps_thd_val_low, ps_thd_val_low);

	APS_LOG("%s:noise = %d\n", __func__, noise);
	APS_LOG("%s:obj->ps_thd_val_high = %d\n", __func__, ps_thd_val_high);
	APS_LOG("%s:obj->ps_thd_val_low = %d\n", __func__, ps_thd_val_low);

	return 0;
}
#endif

static int ltr556_enable_ps_withoutWakeLock(struct i2c_client *client, int enable)
{
	int res = 0;
	u8 databuf[2] = {0};

	if (client == NULL) {
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		res = -1;
		return res;
	}
	/*buffer = ltr556_i2c_read_reg(LTR556_PS_CONTR);*/

	if (enable) {
		databuf[0] = LTR556_PS_CONTR;
		databuf[1] = 0x03; /*ps set active mode */
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
			return ltr556_ERR_I2C;
		}
	} else {
		databuf[0] = LTR556_PS_CONTR;
		databuf[1] = 0x0;  /*ps set standby mode */
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
			return ltr556_ERR_I2C;
		}
	}

	return 0;

EXIT_ERR:
	APS_ERR("ltr558_enable_ps_withoutWakeLock fail\n");
	return res;
}

static void ltr556_check_prox_mean(unsigned int prox_mean)
{
	int temp = prox_mean;

	if (temp <= 30) {
		prox_threshold_hi_param = prox_mean * 50 / 10 + 20;
		prox_threshold_lo_param = prox_mean * 30 / 10 + 20;
	} else if (temp > 30 && temp <= 100) {
		prox_threshold_hi_param = prox_mean + 140;
		prox_threshold_lo_param = prox_mean + 80;
	} else if (temp <= 150) {
		prox_threshold_hi_param = prox_mean * 23 / 10;
		prox_threshold_lo_param = prox_mean * 18 / 10;
	} else {
		prox_threshold_hi_param = prox_mean * 22 / 10;
		prox_threshold_lo_param = prox_mean * 18 / 10;

		if (prox_threshold_hi_param > 1000) {
			prox_threshold_hi_param = 999;
			prox_threshold_lo_param = 950;
		}
	}
#if 0
	if (prox_threshold_hi_param < 100) {
		prox_threshold_hi_param = 90;
		prox_threshold_lo_param = 70;
	}
#endif
	if (prox_threshold_hi_param - prox_threshold_lo_param <= 10) {
		prox_threshold_lo_param = prox_threshold_hi_param - 15;
	}

	APS_ERR("cross_talk=%d,hi=%d,lo=%d\n", prox_mean, prox_threshold_hi_param, prox_threshold_lo_param);
}

int ltr556_get_threshold(struct i2c_client *client, int repeat)
{
	int i = 0;
	u16 ps_thredata = 0;
	int ps_thredata_sum = 0;
	int ps_thredata_mean = 0;
	int ps_thredata_max = 0;
	int ps_thredata_min = 0xFFFF;
	int ltr556_ps_original_power;
	struct ltr556_priv *obj = i2c_get_clientdata(client);

	ltr556_ps_original_power = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
	if (ltr556_ps_original_power == 0) {
		ltr556_enable_ps_withoutWakeLock(client, 1);
	}

	ltr556_i2c_write_reg(LTR556_PS_MEAS_RATE, 0x08);
	mdelay(100);
	for (i = 0; i < repeat; i++) {
		ps_thredata = ltr556_ps_read(client, &ps_thredata);
		mdelay(10);
		ps_thredata_max = (ps_thredata > ps_thredata_max) ? ps_thredata : ps_thredata_max;
		ps_thredata_min = (ps_thredata < ps_thredata_min) ? ps_thredata : ps_thredata_min;
		ps_thredata_sum += ps_thredata;
	}
	ps_thredata_mean = ps_thredata_sum / repeat;
	ltr556_i2c_write_reg(LTR556_PS_MEAS_RATE, 0x02);

	ltr556_ps_original_power = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
	if (ltr556_ps_original_power == 0) {
		ltr556_enable_ps_withoutWakeLock(client, 0);
	}

	return ps_thredata_mean;
}

void ltr556_set_threshold(int prox_mean)
{
	ltr556_check_prox_mean(prox_mean);
	atomic_set(&ltr556_obj->ps_thd_val,  prox_mean);

	atomic_set(&ltr556_obj->ps_thd_val_high,  prox_threshold_hi_param);
	atomic_set(&ltr556_obj->ps_thd_val_low,  prox_threshold_lo_param);

	if (ltr556_obj->hw->polling_mode_ps == 0) {
		ltr556_ps_set_thres();
	}
}

int ltr556_calibrate_get_threshold(struct i2c_client *client, int repeat)
{
	int i = 0;
	u16 ps_thredata = 0;
	int ps_thredata_sum = 0;
	int ps_thredata_mean = 0;
	int ps_thredata_max = 0;
	int ps_thredata_min = 0xFFFF;

	ltr556_i2c_write_reg(LTR556_PS_MEAS_RATE, 0x08);
	mdelay(100);
	for (i = 0; i < repeat; i++) {
	ps_thredata = ltr556_ps_read(client, &ps_thredata);
	mdelay(10);
	ps_thredata_max = (ps_thredata > ps_thredata_max) ? ps_thredata : ps_thredata_max;
	ps_thredata_min = (ps_thredata < ps_thredata_min) ? ps_thredata : ps_thredata_min;
	ps_thredata_sum += ps_thredata;
	}
	ps_thredata_mean = ps_thredata_sum / repeat;
	ltr556_i2c_write_reg(LTR556_PS_MEAS_RATE, 0x02);

	return ps_thredata_mean;
}

static int ltr556_calibrate_zte(void)
{
	APS_FUN();

	ltr556_ps_startup_cross_talk = ltr556_calibrate_get_threshold(ltr556_i2c_client, 5);
	APS_DBG("%s, ltr556_ps_cross_talk=%d, ltr556_ps_calibration_incall_cross_talk=%d\n",
			__func__, ltr556_ps_cross_talk, ltr556_ps_startup_cross_talk);

	if (ltr556_ps_cross_talk > 0) {
		if (ltr556_ps_startup_cross_talk > ltr556_ps_cross_talk) {
			if ((ltr556_ps_startup_cross_talk - ltr556_ps_cross_talk) > 200) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		}
	}
	ltr556_set_threshold(ltr556_ps_startup_cross_talk);

	APS_DBG("%s,ltr556_ps_calibration_incall_result=%d\n", __func__, ltr556_ps_startup_cross_talk);
	return 0;
}

/********************************************************************/
/*
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr556_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	regdata = ltr556_i2c_read_reg(LTR556_ALS_CONTR);
	APS_LOG("LTR556_ALS_CONTR beforeset= %d\n", regdata);
	if (enable != 0) {
		APS_LOG("ALS(1): enable als only\n");
		regdata |= 0x01;
	} else {
		APS_LOG("ALS(1): disable als only\n");
		regdata &= 0xfe;
	}

	err = ltr556_i2c_write_reg(LTR556_ALS_CONTR, regdata);
	if (err < 0) {
		APS_ERR("ALS: enable als err: %d en: %d\n", err, enable);
		return err;
	}
	APS_LOG("LTR556_ALS_CONTR afterset= %d\n", regdata);

	mdelay(WAKEUP_DELAY);
	als_value_backup = -1;

	return 0;
}

static int ltr556_als_read(struct i2c_client *client, u16 *data)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int luxdata_int;
	int ratio;

	alsval_ch1_lo = ltr556_i2c_read_reg(LTR556_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr556_i2c_read_reg(LTR556_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = ltr556_i2c_read_reg(LTR556_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr556_i2c_read_reg(LTR556_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

	if ((alsval_ch1 == 0) || (alsval_ch0 == 0)) {
		luxdata_int = 0;
		goto out;
	}

	ratio = (alsval_ch1 * 100) / (alsval_ch0 + alsval_ch1);
	if (ratio < 45) {
		luxdata_int = (((17743 * alsval_ch0) + (11059 * alsval_ch1)) / als_gainrange) / 1000;
	} else if ((ratio < 64) && (ratio >= 45)) {
		luxdata_int = (((42785 * alsval_ch0) - (19548 * alsval_ch1)) / als_gainrange) / 1000;
	} else if ((ratio < 85) && (ratio >= 64)) {
		luxdata_int = (((5926 * alsval_ch0) + (1185 * alsval_ch1)) / als_gainrange) / 1000;
	} else {
		luxdata_int = 0;
	}

out:
	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}
/********************************************************************/
static int ltr556_get_ps_value(struct ltr556_priv *obj, u16 ps)
{
	int val;
	int invalid = 0;
	static int val_temp = 1;

	if (ps > atomic_read(&obj->ps_thd_val_high)) {
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	} else if (ps < atomic_read(&obj->ps_thd_val_low)) {
		val = 5;  /*far away*/
		val_temp = 5;
		intr_flag_value = 0;
	} else
		val = val_temp;

	if (atomic_read(&obj->ps_suspend)) {
		invalid = 1;
	} else if (atomic_read(&obj->ps_deb_on) == 1) {
		unsigned long endt = atomic_read(&obj->ps_deb_end);

		if (time_after(jiffies, endt)) {
			atomic_set(&obj->ps_deb_on, 0);
		}

		if (atomic_read(&obj->ps_deb_on) == 1) {
			invalid = 1;
		}
	} else if (obj->als > 50000) {
		/*invalid = 1;*/
		APS_DBG("ligh too high will result to failt proximiy\n");
		/*return 1;  far away*/
	}

	if (!invalid) {
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}

	val = -1;
	return val;
}

u16 ltr556_als_raw2lux(u16 als)
{
	u16 lux = 0;

#if 0
	if (als < 400) {
		lux = als * 4 / 10;
	} else if (als < 1500) {
		lux = als * 26 / 100 + 58;
	} else {
		lux = als * 2 / 10 + 150;
		/*has defined LIGHT_RANGE = 10240.0f in hwmsen_chip_info.c*/
		if (lux > 10240) {
			lux = 10240;
		}
	}
#endif
	if (als < 100) {
		lux = als * 86 / 100;
	} else {
	   lux = als * 75 / 100;
	}
	if (lux > 10240) {
	    lux = 10240;
	}
	return lux;
}

/********************************************************************/
static int ltr556_get_als_value(struct ltr556_priv *obj, u16 als)
{
	int invalid = 0;
	bool changed = false;
	u16 als_lux = 0;
	int res = 0;

/*
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if (als < obj->hw->als_level[idx]) {
			break;
		}
	}

	if (idx >= obj->als_value_num) {
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}
*/
	if (als >= 0) {
		if (als_value_backup != -1) {
			if ((als < als_value_backup * 7/10) || (als > als_value_backup * 14/10)) {
				als_lux = ltr556_als_raw2lux(als);
				APS_DBG("LTR556-ALS: %05d => %05d\n", als, als_lux);
				changed = true;
			} else {
				als_lux = lux_backup;
				changed = false;
			}
		} else {
		    APS_DBG("LTR556-ALS: ----als_value_backup = -1\n");
			als_lux = ltr556_als_raw2lux(als);
			changed = true;
		}
	} else {
		APS_DBG("LTR556-ALS: als < 0, als = %05d\n", als);
		als_lux = lux_backup;
		changed = false;
	}

	if (atomic_read(&obj->als_deb_on) == 1) {
		unsigned long endt = atomic_read(&obj->als_deb_end);

		if (time_after(jiffies, endt)) {
			atomic_set(&obj->als_deb_on, 0);
		}

		if (atomic_read(&obj->als_deb_on) == 1) {
			invalid = 1;
		}
	}
/*
	if (!invalid) {
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	} else {
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
*/
	if (!invalid) {
		/* APS_DBG("LTR556-ALS: %05d => %05d\n", als, als_lux);*/
		if (changed) {
		    als_value_backup = als;
		    lux_backup = als_lux;
		}
		return als_lux;
	}

	APS_ERR("LTR556-ALS: %05d => %05d (-1)\n", als, als_lux);
	als_value_backup = -1;
	lux_backup = -1;
	res = -1;
	return res;
}
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr556_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n",
		atomic_read(&ltr556_obj->i2c_retry), atomic_read(&ltr556_obj->als_debounce),
		atomic_read(&ltr556_obj->ps_mask), atomic_read(&ltr556_obj->ps_thd_val),
		atomic_read(&ltr556_obj->ps_debounce));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	if (sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb) == 5) {
		atomic_set(&ltr556_obj->i2c_retry, retry);
		atomic_set(&ltr556_obj->als_debounce, als_deb);
		atomic_set(&ltr556_obj->ps_mask, mask);
		atomic_set(&ltr556_obj->ps_thd_val, thres);
		atomic_set(&ltr556_obj->ps_debounce, ps_deb);
	} else {
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr556_obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	if (sscanf(buf, "0x%x", &trace) == 1) {
		atomic_set(&ltr556_obj->trace, trace);
	} else {
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_als(struct device_driver *ddri, char *buf)
{
	int res;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}
	res = ltr556_als_read(ltr556_obj->client, &ltr556_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);

}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}
	res = ltr556_ps_read(ltr556_obj->client, &ltr556_obj->ps);
	return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_reg(struct device_driver *ddri, char *buf)
{
	int i, len = 0;
	int reg[] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c,
		0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x97, 0x98, 0x99, 0x9a, 0x9e};

	for (i = 0; i < 27; i++) {
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n",
		reg[i], ltr556_i2c_read_reg(reg[i]));
	}
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_send(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	} else if (sscanf(buf, "%x %x", &addr, &cmd) != 2) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	/****************************/
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_recv(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	} else if (sscanf(buf, "%x", &addr) != 1) {
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	/****************************/
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	if (ltr556_obj->hw) {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n",
			ltr556_obj->hw->i2c_num, ltr556_obj->hw->power_id, ltr556_obj->hw->power_vol);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n",
		atomic_read(&ltr556_obj->als_suspend), atomic_read(&ltr556_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct ltr556_priv *obj, const char *buf, size_t count, u32 data[], int len)
{
	int idx = 0;
	char *cur = (char *)buf, *end = (char *)(buf+count);

	while (idx < len) {
		while ((cur < end) && IS_SPACE(*cur)) {
			cur++;
		}

		if (sscanf(cur, "%d", &data[idx]) != 1) {
			break;
		}

		idx++;
		while ((cur < end) && !IS_SPACE(*cur)) {
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < ltr556_obj->als_level_num; idx++) {
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr556_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(ltr556_obj->als_level, ltr556_obj->hw->als_level, sizeof(ltr556_obj->als_level));
	} else if (ltr556_obj->als_level_num != read_int_from_buf(ltr556_obj, buf, count,
			ltr556_obj->hw->als_level, ltr556_obj->als_level_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	}

	for (idx = 0; idx < ltr556_obj->als_value_num; idx++) {
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ltr556_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr556_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		return 0;
	} else if (!strcmp(buf, "def")) {
		memcpy(ltr556_obj->als_value, ltr556_obj->hw->als_value, sizeof(ltr556_obj->als_value));
	} else if (ltr556_obj->als_value_num != read_int_from_buf(ltr556_obj, buf, count,
			ltr556_obj->hw->als_value, ltr556_obj->als_value_num)) {
		APS_ERR("invalid format: '%s'\n", buf);
	}
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr556_show_als,		NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr556_show_ps,		NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr556_show_config,	ltr556_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr556_show_alslv,	ltr556_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr556_show_alsval,	ltr556_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ltr556_show_trace,	ltr556_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr556_show_status,	NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, ltr556_show_send,	ltr556_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, ltr556_show_recv,	ltr556_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr556_show_reg,		NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr556_attr_list[] = {
	&driver_attr_als,
	&driver_attr_ps,
	&driver_attr_trace,        /*trace log*/
	&driver_attr_config,
	&driver_attr_alslv,
	&driver_attr_alsval,
	&driver_attr_status,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int ltr556_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(ltr556_attr_list));

	if (driver == NULL) {
		return -EINVAL;
	}

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, ltr556_attr_list[idx]);
		if (err) {
			APS_ERR("driver_create_file (%s) = %d\n", ltr556_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr556_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(ltr556_attr_list));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		driver_remove_file(driver, ltr556_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------interrupt functions--------------------------------*/
#ifndef CUSTOM_KERNEL_SENSORHUB
static int ltr556_check_and_clear_intr(struct i2c_client *client)
{
	int res, intp, intl;
	u8 buffer[2];
	u8 temp;

	APS_FUN();

	/*if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) skip if no interrupt*/
	/*	  return 0; */

	buffer[0] = LTR556_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	temp = buffer[0];
	res = 1;
	intp = 0;
	intl = 0;
	if ((buffer[0] & 0x02) != 0) {
		res = 0;
		intp = 1;
	}
	if ((buffer[0] & 0x08) != 0) {
		res = 0;
		intl = 1;
	}

	if (res == 0) {
		if ((intp == 1) && (intl == 0)) {
			buffer[1] = buffer[0] & 0xfD;
		} else if ((intp == 0) && (intl == 1)) {
			buffer[1] = buffer[0] & 0xf7;
		} else {
			buffer[1] = buffer[0] & 0xf5;
		}
		buffer[0] = LTR556_ALS_PS_STATUS;
		res = i2c_master_send(client, buffer, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		} else {
			res = 0;
		}
	} else
		return 0;

EXIT_ERR:
	APS_ERR("ltr556_check_and_clear_intr fail\n");
	return 1;
}

static int ltr556_check_intr(struct i2c_client *client)
{
	int res, intp, intl;
	u8 buffer[2];

	APS_FUN();

	/* if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) skip if no interrupt*/
	/*    return 0;*/

	buffer[0] = LTR556_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}

	res = 0;
	intp = 0;
	intl = 0;
	if ((buffer[0] & 0x02) != 0) {
		res = 1;	/*PS int*/
		intp = 1;
	}
	if ((buffer[0] & 0x08) != 0) {
		res = 2;	/*ALS int*/
		intl = 1;
	}
	if ((intp == 1) && (intl == 1)) {
		res = 4;	/*ALS & PS int*/
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr556_check_intr fail\n");
	return 0;
}

static int ltr556_clear_intr(struct i2c_client *client)
{
	int res;
	u8 buffer[2];

	APS_FUN();

	buffer[0] = LTR556_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if (res <= 0) {
		goto EXIT_ERR;
	}
	APS_DBG("buffer[0] = %d\n", buffer[0]);
	buffer[1] = buffer[0] & 0xF5;
	buffer[0] = LTR556_ALS_PS_STATUS;

	res = i2c_master_send(client, buffer, 0x2);
	if (res <= 0) {
		goto EXIT_ERR;
	} else {
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr556_clear_intr fail\n");
	return 1;
}
#endif /* CUSTOM_KERNEL_SENSORHUB */

static void ltr556_cali_ps_work(struct work_struct *work)
{
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
	int err = 0;

	err = ltr556_dynamic_calibrate();
	if (err < 0) {
		APS_LOG("ltr556_dynamic_calibrate() failed\n");
	}
#endif
	ltr556_ps_set_thres();
}
/*----------------------------------------------------------------------------*/
static void ltr556_eint_work(struct work_struct *work)
{
	struct ltr556_priv *obj = (struct ltr556_priv *)container_of(work, struct ltr556_priv, eint_work);
	u8 databuf[2];
	int res = 0;
	int err;
	int value = 1;

	err = ltr556_check_intr(obj->client);
	if (err < 0) {
		goto EXIT_INTR;
	} else {
		/*get raw data*/
		obj->ps = ltr556_ps_read(obj->client, &obj->ps);
		if (obj->ps <= 0) {
			pr_err("%s ps(%d)<=0\n", __func__, obj->ps);
			intr_flag_value = 0;
			value = 5;
		} else
			value = ltr556_get_ps_value(obj, obj->ps);

		APS_DBG("ltr556_eint_work: rawdata ps=%d!\n", obj->ps);
		value = ltr556_get_ps_value(obj, obj->ps);
		APS_DBG("intr_flag_value=%d\n", intr_flag_value);
		if (intr_flag_value) {
			databuf[0] = LTR556_PS_THRES_LOW_0;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_LOW_1;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_UP_0;
			databuf[1] = (u8)(0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_UP_1;
			databuf[1] = (u8)((0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
		} else{
#ifndef DELAYED_PS_CALI
#if 0
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
			if (obj->ps > 20 && obj->ps < (dynamic_calibrate - 50)) {
			if (obj->ps < 100) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+100);
				atomic_set(&obj->ps_thd_val_low, obj->ps+50);
			} else if (obj->ps < 200) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
				atomic_set(&obj->ps_thd_val_low, obj->ps+60);
			} else if (obj->ps < 300) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
				atomic_set(&obj->ps_thd_val_low, obj->ps+60);
			} else if (obj->ps < 400) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+150);
				atomic_set(&obj->ps_thd_val_low, obj->ps+60);
			} else if (obj->ps < 600) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+180);
				atomic_set(&obj->ps_thd_val_low, obj->ps+90);
			} else if (obj->ps < 1000) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+300);
				atomic_set(&obj->ps_thd_val_low, obj->ps+180);
			} else if (obj->ps < 1250) {
				atomic_set(&obj->ps_thd_val_high,  obj->ps+400);
				atomic_set(&obj->ps_thd_val_low, obj->ps+300);
			} else {
				atomic_set(&obj->ps_thd_val_high,  1400);
				atomic_set(&obj->ps_thd_val_low, 1000);
			}

			dynamic_calibrate = obj->ps;
		}
#endif
#endif
			databuf[0] = LTR556_PS_THRES_LOW_0;
			databuf[1] = (u8)(0 & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_LOW_1;
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_UP_0;
			databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
			databuf[0] = LTR556_PS_THRES_UP_1;
			databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if (res <= 0) {
				goto EXIT_INTR;
			}
#else
			cancel_delayed_work(&ltr556_obj->cali_ps_work);
			schedule_delayed_work(&ltr556_obj->cali_ps_work, msecs_to_jiffies(2000));
#endif
		}
		/*let up layer to know*/
		res = ps_report_interrupt_data(value);
	}

EXIT_INTR:
	ltr556_clear_intr(obj->client);
#ifdef CONFIG_OF
	enable_irq(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr556_eint_func(void)
{
	struct ltr556_priv *obj = ltr556_obj;

	if (!obj) {
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

#ifdef CONFIG_OF
static irqreturn_t ltr556_eint_handler(int irq, void *desc)
{
	if (irq_enabled == 2) {
		disable_irq_nosync(ltr556_obj->irq);
		ltr556_eint_func();
	}

	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr556_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = { 0, 0 };

	APS_FUN();

	/* gpio setting */
	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}

	/* eint request */
	if (ltr556_obj->irq_node) {
		of_property_read_u32_array(ltr556_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		ltr556_obj->irq = irq_of_parse_and_map(ltr556_obj->irq_node, 0);
		APS_LOG("ltr556_obj->irq = %d\n", ltr556_obj->irq);
		if (!ltr556_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		/*APS_ERR("irq to gpio = %d\n", irq_to_gpio(ltr556_obj->irq));*/
		if (request_irq(ltr556_obj->irq, ltr556_eint_handler, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq_wake(ltr556_obj->irq);
		disable_irq(ltr556_obj->irq);
		irq_enabled = 1;
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
/**********************************************************************************************/

/*-------------------------------MISC device related------------------------------------------*/
static int ltr556_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr556_i2c_client;

	if (!file->private_data) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ltr556_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ltr556_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct ltr556_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user *) arg;
	int dat;
	uint32_t enable;
	int ps_cali;
	int threshold[2];
	int ps_cross[2];

	APS_DBG("cmd= %d\n", cmd);

	switch (cmd) {
	case ALSPS_SET_PS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		err = ltr556_ps_enable(obj->client, enable);
		if (err < 0) {
			APS_ERR("enable ps fail: %d en: %d\n", err, enable);
			goto err_out;
		}
		if (enable)
			set_bit(CMC_BIT_PS, &obj->enable);
		else
			clear_bit(CMC_BIT_PS, &obj->enable);
	break;
	case ALSPS_GET_PS_MODE:
		enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
				goto err_out;
		}
	break;
	case ALSPS_GET_PS_DATA:
		APS_DBG("ALSPS_GET_PS_DATA\n");
		obj->ps = ltr556_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0) {
				goto err_out;
		}

		dat = ltr556_get_ps_value(obj, obj->ps);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_GET_PS_RAW_DATA:
		obj->ps = ltr556_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0) {
			goto err_out;
		}
		dat = obj->ps;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_SET_ALS_MODE:
		if (copy_from_user(&enable, ptr, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
		err = ltr556_als_enable(obj->client, enable);
		if (err < 0) {
			APS_ERR("enable als fail: %d en: %d\n", err, enable);
			goto err_out;
		}
		if (enable)
			set_bit(CMC_BIT_ALS, &obj->enable);
		else
			clear_bit(CMC_BIT_ALS, &obj->enable);
	break;
	case ALSPS_GET_ALS_MODE:
		enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
		if (copy_to_user(ptr, &enable, sizeof(enable))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_GET_ALS_DATA:
		obj->als = ltr556_als_read(obj->client, &obj->als);
		if (obj->als < 0) {
			goto err_out;
		}
		dat = ltr556_get_als_value(obj, obj->als);
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_GET_ALS_RAW_DATA:
		obj->als = ltr556_als_read(obj->client, &obj->als);
		if (obj->als < 0) {
			goto err_out;
		}
		dat = obj->als;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
	break;

/*----------------------------------for factory mode test---------------------------------------*/
	case ALSPS_GET_PS_TEST_RESULT:
		obj->ps = ltr556_ps_read(obj->client, &obj->ps);
		if (obj->ps < 0) {
			goto err_out;
		}
		if (obj->ps > atomic_read(&obj->ps_thd_val_low))
			dat = 1;
		else
			dat = 0;
		if (copy_to_user(ptr, &dat, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_IOCTL_CLR_CALI:
		if (copy_from_user(&dat, ptr, sizeof(dat))) {
			err = -EFAULT;
			goto err_out;
		}
		if (dat == 0)
			obj->ps_cali = 0;
	break;
	case ALSPS_IOCTL_GET_CALI:
		ps_cali = obj->ps_cali;
		if (copy_to_user(ptr, &ps_cali, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_IOCTL_SET_CALI:
		APS_LOG("enter ALSPS_IOCTL_SET_CALI\n");
		if (copy_from_user(&ps_cali, ptr, sizeof(ps_cali))) {
			err = -EFAULT;
			goto err_out;
		}
		obj->ps_cali = ps_cali;
	break;
	case ALSPS_SET_PS_THRESHOLD:
		APS_LOG("enter ALSPS_SET_PS_THRESHOLD\n");
		if (copy_from_user(threshold, ptr, sizeof(threshold))) {
			err = -EFAULT;
			goto err_out;
		}
		atomic_set(&obj->ps_thd_val_high, (threshold[0]+obj->ps_cali));
		atomic_set(&obj->ps_thd_val_low, (threshold[1]+obj->ps_cali));/*need to confirm*/

		ltr556_ps_set_thres();
	break;
	case ALSPS_GET_PS_THRESHOLD_HIGH:
		threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_GET_PS_THRESHOLD_LOW:
		threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
		if (copy_to_user(ptr, &threshold[0], sizeof(threshold[0]))) {
			err = -EFAULT;
			goto err_out;
		}
	break;
	case ALSPS_PS_ENABLE_CALI:
		APS_LOG("enter ALSPS_PS_ENABLE_CALI\n");
		ltr556_ps_cross_talk = ltr556_get_threshold(client, 10);
		ltr556_set_threshold(ltr556_ps_cross_talk);
		ps_cross[0] = ltr556_ps_cross_talk;
		ps_cross[1] = 0;
		ps_cali_report(ps_cross);
	break;
/*------------------------------------------------------------------------------------------*/
	default:
		APS_LOG("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
	break;
	}

err_out:
	return err;
}
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ltr556_fops = {
	.owner = THIS_MODULE,
	.open = ltr556_open,
	.release = ltr556_release,
	.unlocked_ioctl = ltr556_unlocked_ioctl,
};

static struct miscdevice ltr556_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr556_fops,
};

/*--------------------------------------------------------------------------------*/
static int ltr556_init_client(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];

	struct i2c_client *client = ltr556_obj->client;

	struct ltr556_priv *obj = ltr556_obj;

	mdelay(PON_DELAY);

	init_ps_gain = MODE_PS_Gain16;

	APS_LOG("LTR556_PS setgain = %d!\n", init_ps_gain);
	res = ltr556_i2c_write_reg(LTR556_PS_CONTR, init_ps_gain);
	if (res < 0) {
		APS_LOG("ltr556 set ps gain error\n");
		goto EXIT_ERR;
	}

	res = ltr556_i2c_write_reg(LTR556_PS_N_PULSES, 4);
	if (res < 0) {
		APS_LOG("ltr556 set ps pulse error\n");
		goto EXIT_ERR;
	}

	res = ltr556_i2c_write_reg(LTR556_PS_LED, 0x7F);
	if (res < 0) {
		APS_LOG("ltr556 set ps led error\n");
		goto EXIT_ERR;
	}

	/*for interrupt work mode support */
	if (obj->hw->polling_mode_ps == 0) {
		APS_LOG("eint enable");

		databuf[0] = LTR556_INTERRUPT;
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}

		databuf[0] = LTR556_INTERRUPT_PERSIST;
		databuf[1] = 0x10;
		res = i2c_master_send(client, databuf, 0x2);
		if (res <= 0) {
			goto EXIT_ERR;
		}
	}

#if 0
	res = ltr556_ps_enable(client, 1);
	if (res < 0) {
		APS_ERR("enable ps fail: %d\n", res);
		goto EXIT_ERR;
	}
#endif

	/* Enable ALS to Full Range at startup*/
	als_gainrange = ALS_RANGE_1300;/*ALS_RANGE_64K;*/
	init_als_gain = als_gainrange;
	APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

	switch (init_als_gain) {
	case ALS_RANGE_64K:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range1);
		break;

	case ALS_RANGE_32K:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range2);
		break;

	case ALS_RANGE_16K:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range3);
		break;

	case ALS_RANGE_8K:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range4);
		break;

	case ALS_RANGE_1300:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range5);
		break;

	case ALS_RANGE_600:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range6);
		break;

	default:
		res = ltr556_i2c_write_reg(LTR556_ALS_CONTR, MODE_ALS_Range1);
		break;
	}

	/* LTR556_ALS_MEAS_RATE set 0x01 */
	res = ltr556_i2c_write_reg(LTR556_ALS_MEAS_RATE, 0x01);
	if (res < 0) {
		APS_LOG("ltr556 set als rate error\n");
		goto EXIT_ERR;
	}

	res = ltr556_als_enable(client, 1);
	if (res < 0) {
		APS_ERR("enable als fail: %d\n", res);
		goto EXIT_ERR;
	}

	res = ltr556_setup_eint(client) != 0;
	if (res) {
		APS_ERR("setup eint: %d\n", res);
		goto EXIT_ERR;
	}

	res = ltr556_check_and_clear_intr(client);
	if (res) {
		APS_ERR("check/clear intr: %d\n", res);
		goto EXIT_ERR;
	}

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return 1;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int als_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int als_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ltr556_obj als enable value = %d\n", en);

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		res = -1;
		return res;
	}

	mutex_lock(&ltr556_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr556_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr556_obj->enable);
	mutex_unlock(&ltr556_mutex);
#if 0
	if ((en == 0) && test_bit(CMC_BIT_PS, &obj->enable))
		return 0;
#endif
	res = ltr556_als_enable(ltr556_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return res;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	/* Do nothing */
	return 0;
}

static int als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return als_set_delay(samplingPeriodNs);
}

static int als_flush(void)
{
	return als_flush_report();
}

static int als_get_data(int *value, int *status)
{
	int err = 0;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		err = -1;
		return err;
	}

	ltr556_obj->als = ltr556_als_read(ltr556_obj->client, &ltr556_obj->als);
	if (ltr556_obj->als < 0)
		err = -1;
	else {
		*value = ltr556_get_als_value(ltr556_obj, ltr556_obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int ps_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int ps_enable_nodata(int en)
{
	int res = 0;

	APS_LOG("ltr556_obj ps enable value = %d\n", en);

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		res = -1;
		return res;
	}

	mutex_lock(&ltr556_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr556_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr556_obj->enable);
	mutex_unlock(&ltr556_mutex);

	res = ltr556_ps_enable(ltr556_obj->client, en);
	if (res < 0) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return res;
	}
	return 0;
}

static int ps_set_delay(u64 ns)
{
	/* Do nothing */
	return 0;
}

static int ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	return 0;
}

static int ps_flush(void)
{
	return ps_flush_report();
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;

	if (!ltr556_obj) {
		APS_ERR("ltr556_obj is null!!\n");
		err = -1;
		return err;
	}

	ltr556_obj->ps = ltr556_ps_read(ltr556_obj->client, &ltr556_obj->ps);
	if (ltr556_obj->ps < 0)
		err = -1;
	else {
		*value = ltr556_get_ps_value(ltr556_obj, ltr556_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}
/*-----------------------------------------------------------------------------------*/

/*add by shengxia for ztecalips begin */
static ssize_t prox_calibration_value_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int p_data = 0;
	int cnt;
	char buff[32] = {'\0'};

	APS_FUN();

	if (*offset != 0) {
		APS_DBG("%s,return 0\n", __func__);
		return 0;
	}

	p_data = ltr556_get_threshold(ltr556_i2c_client, 1);
	cnt = snprintf(buff, PAGE_SIZE, "%d %d %d %d %d\n",
		ltr556_ps_startup_calibration, ltr556_ps_startup_cross_talk,
		prox_threshold_hi_param, prox_threshold_lo_param, p_data);
	if (copy_to_user(buffer, buff, cnt)) {
		APS_ERR("%s, copy_to_user error\n", __func__);
		return -EINVAL;
	}

	*offset += cnt;
	return cnt;
}

static ssize_t ltr556_read_proc(struct file *file, char __user *buffer, size_t count, loff_t *offset)
{
	int cnt;
	char buff[16] = {'\0'};
	/* ps_calibrating = TRUE; */

	APS_FUN();

	if (*offset != 0) {
		APS_DBG("%s,return 0\n", __func__);
		return 0;
	}
	ltr556_ps_cross_talk = ltr556_get_threshold(ltr556_i2c_client, 10);
	ltr556_set_threshold(ltr556_ps_cross_talk);
	cnt = snprintf(buff, sizeof(ltr556_ps_cross_talk), "%d\n", ltr556_ps_cross_talk);
	if (copy_to_user(buffer, buff, cnt)) {
		APS_ERR("%s, copy_to_user error\n", __func__);
		return -EINVAL;
	}
	/*ps_calibrating = FALSE;*/
	*offset += cnt;
	APS_DBG("%s,ltr556_ps_cross_talk=%d\n", __func__, ltr556_ps_cross_talk);
	return cnt;
}

static ssize_t ltr556_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
	int value = 0;
	char buf[16];
	size_t copyLen = 0;
	int res = 0;

	APS_DBG("%s: write len = %d\n", __func__, (int)len);
	copyLen = len < 16 ? len : 16;
	if (copy_from_user(buf, user_buf, copyLen)) {
		APS_DBG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}

	res = sscanf(buf, "%d", &value);
	APS_DBG("buf=%s, value = %d, copyLen=%d\n", buf, value, (int)copyLen);

	if (value > 0 && value < 2048) {
		ltr556_ps_cross_talk = value;
	}

	ltr556_ps_startup_cross_talk = ltr556_get_threshold(ltr556_i2c_client, 10);
	ltr556_ps_startup_calibration = ltr556_ps_startup_cross_talk;
	APS_DBG("%s, ltr556_ps_cross_talk=%d, ltr556_ps_startup_cross_talk=%d\n",
		__func__, ltr556_ps_cross_talk, ltr556_ps_startup_cross_talk);

	if (ltr556_ps_cross_talk > 0) {
		if (ltr556_ps_startup_cross_talk > ltr556_ps_cross_talk) {
			if ((ltr556_ps_startup_cross_talk - ltr556_ps_cross_talk) > 300) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		} else {
			/*LTR556 ps_data=0 in sunlight*/
			if (ltr556_ps_startup_cross_talk < (ltr556_ps_cross_talk / 2)) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		}
	} else {
		/*  ltr556_ps_cross_talk <=0, change to power on calibration.++++dinggaoshan.20150921 */
		ltr556_ps_cross_talk = ltr556_ps_startup_cross_talk;
		APS_DBG("%s, ltr556_ps_cross_talk=%d", __func__, ltr556_ps_cross_talk);
		/* ----------------------------------end---dinggaoshan.20150921 */
	}

	ltr556_set_threshold(ltr556_ps_startup_cross_talk);

	return len;
}

static struct proc_dir_entry *ltr556_proc_file = NULL;
static struct proc_dir_entry *prox_calibration_value = NULL;

static const struct file_operations ltr556_proc_fops = {
	.owner      = THIS_MODULE,
	/*.open     = ltr556_calibrate_open, */
	.read       = ltr556_read_proc,
	.write      = ltr556_write_proc,
};

static const struct file_operations prox_calibration_value_fops = {
	.owner      = THIS_MODULE,
	.read       = prox_calibration_value_read,
};

#if 0
static struct proc_dir_entry *calibration_inCall = NULL;

static ssize_t calibration_inCall_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int len;

	APS_FUN();

	if (*offset != 0) {
		APS_DBG("%s,return 0\n", __func__);
		return 0;
	}

	ltr556_ps_startup_cross_talk = ltr556_get_threshold(ltr556_i2c_client, 5);
	APS_DBG("%s, ltr556_ps_cross_talk=%d, ltr556_ps_calibration_incall_cross_talk=%d\n",
		__func__, ltr556_ps_cross_talk, ltr556_ps_startup_cross_talk);

	if (ltr556_ps_cross_talk > 0) {
		if (ltr556_ps_startup_cross_talk > ltr556_ps_cross_talk) {
			if ((ltr556_ps_startup_cross_talk - ltr556_ps_cross_talk) > 200) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		}
/*
		else if (ltr556_ps_startup_cross_talk <= ltr556_ps_cross_talk) {
			if ((ltr556_ps_cross_talk - ltr556_ps_startup_cross_talk) > 20) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		}
*/
	}
	ltr556_set_threshold(ltr556_ps_startup_cross_talk);

	len = snprintf(buffer, sizeof(ltr556_ps_startup_cross_talk), "%d\n", ltr556_ps_startup_cross_talk);
	*offset += len;
	APS_DBG("%s,ltr556_ps_cross_talk=%d\n", __func__, ltr556_ps_cross_talk);
	return len;
}

static const struct file_operations calibration_inCall_fops = {
	.owner      = THIS_MODULE,
	.read       = calibration_inCall_read,
};
#endif

static void create_ltr556_proc_file(void)
{
	ltr556_proc_file = proc_create("driver/alsps_threshold", 0664, NULL, &ltr556_proc_fops);
	pr_info("%s\n", __func__);

	if (ltr556_proc_file == NULL) {
		pr_err("create_ltr556_proc_file fail!\n");
	}

	prox_calibration_value = proc_create("driver/prox_calibration_value", 0444, NULL, &prox_calibration_value_fops);
	if (prox_calibration_value == NULL) {
		pr_err("create_ltr556_proc_file prox_calibration_value fail!\n");
	}
#if 0
	calibration_inCall = proc_create("driver/calibration_inCall", 0444, NULL, &calibration_inCall_fops);
	if (calibration_inCall == NULL) {
		pr_err("create_ltr556_proc_file calibration_inCall  fail!\n");
	}
#endif
}
/*add by shengxia for ztecalips end */

static int ltr556_psensor_set_cali(uint8_t *data, uint8_t count)
{

	APS_DBG("%s, set_cali =%d\n", __func__, data[0]);
	if (data[0] > 0 && data[0] < 2048) {
		ltr556_ps_cross_talk = data[0];
	}

	ltr556_ps_startup_cross_talk = ltr556_get_threshold(ltr556_i2c_client, 10);
	ltr556_ps_startup_calibration = ltr556_ps_startup_cross_talk;
	APS_DBG("%s, ltr556_ps_cross_talk=%d, ltr556_ps_startup_cross_talk=%d\n",
		__func__, ltr556_ps_cross_talk, ltr556_ps_startup_cross_talk);

	if (ltr556_ps_cross_talk > 0) {
		if (ltr556_ps_startup_cross_talk > ltr556_ps_cross_talk) {
			if ((ltr556_ps_startup_cross_talk - ltr556_ps_cross_talk) > 300) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		} else {
			/*LTR556 ps_data=0 in sunlight*/
			if (ltr556_ps_startup_cross_talk < (ltr556_ps_cross_talk / 2)) {
				ltr556_ps_startup_cross_talk = ltr556_ps_cross_talk;
			}
		}
	} else {
		/*  ltr556_ps_cross_talk <=0, change to power on calibration.++++dinggaoshan.20150921 */
		ltr556_ps_cross_talk = ltr556_ps_startup_cross_talk;
		APS_DBG("%s, ltr556_ps_cross_talk=%d", __func__, ltr556_ps_cross_talk);
		/* ----------------------------------end---dinggaoshan.20150921 */
	}

	ltr556_set_threshold(ltr556_ps_startup_cross_talk);

	return 0;
}

/*-----------------------------------i2c operations----------------------------------*/
static int ltr556_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr556_priv *obj = NULL;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;

	APS_FUN();

	err = get_alsps_dts_func(client->dev.of_node, hw);
	if (err < 0) {
		APS_ERR("get customization info from dts failed\n");
		return -EFAULT;
	}

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr556_obj = obj;

	obj->hw = hw;
	INIT_WORK(&obj->eint_work, ltr556_eint_work);
	INIT_DELAYED_WORK(&obj->cali_ps_work, ltr556_cali_ps_work);

	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	/*atomic_set(&obj->als_cmd_val, 0xDF);*/
	/*atomic_set(&obj->ps_cmd_val,  0xC1);*/
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	obj->irq_node = client->dev.of_node;

	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
	obj->als_level_num = ARRAY_SIZE(obj->hw->als_level);
	obj->als_value_num = ARRAY_SIZE(obj->hw->als_value);
	obj->als_modulus = (400*100)/(16*150);

	/*-----------------------------value need to be confirmed-----------------------------------------*/

	WARN_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	WARN_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr556_init_client() start...!\n");
	ltr556_i2c_client = client;
	err = ltr556_init_client();
	if (err) {
		goto exit_init_failed;
	}
	APS_LOG("ltr556_init_client() OK!\n");

	err = misc_register(&ltr556_device);
	/*err = alsps_factory_device_register(&ltr556_device);*/
	if (err) {
		APS_ERR("ltr556_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;

	/*------------------------ltr556 attribute file for debug--------------------------------------*/
	/*err = ltr556_create_attr(&(ltr556_init_info.platform_diver_addr->driver)); */
	err = ltr556_create_attr(&(ltr556_i2c_driver.driver));
	if (err) {
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ltr556 attribute file for debug--------------------------------------*/

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.batch = als_batch;
	als_ctl.flush = als_flush;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.batch = ps_batch;
	ps_ctl.flush = ps_flush;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	ps_ctl.is_polling_mode = hw->polling_mode_ps;
	ps_ctl.set_cali = ltr556_psensor_set_cali;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	create_ltr556_proc_file();

	ltr556_init_flag = 0;
	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&ltr556_device);
exit_init_failed:
	kfree(obj);
exit:
	ltr556_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr556_init_flag = -1;
	return err;
}

static int ltr556_i2c_remove(struct i2c_client *client)
{
	int err;

	/*err = ltr579_delete_attr(&(ltr579_init_info.platform_diver_addr->driver));*/
	err = ltr556_delete_attr(&(ltr556_i2c_driver.driver));
	if (err) {
		APS_ERR("ltr556_delete_attr fail: %d\n", err);
	}

	misc_deregister(&ltr556_device);
	/*err = alsps_factory_device_deregister(&ltr556_device);
	if (err) {
		APS_ERR("misc_deregister fail: %d\n", err);
	}*/

	ltr556_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int ltr556_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, LTR556_DEV_NAME, 8);
	return 0;
}

static int ltr556_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr556_priv *obj = i2c_get_clientdata(client);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	atomic_set(&obj->als_suspend, 1);
	err = ltr556_als_enable(obj->client, 0);
	if (err < 0) {
		APS_ERR("disable als: %d\n", err);
		return err;
	}
/*
	atomic_set(&obj->ps_suspend, 1);
	err = ltr556_ps_enable(obj->client, 0);
	if (err < 0) {
		APS_ERR("disable ps:  %d\n", err);
		return err;
	}

	ltr556_power(obj->hw, 0);
*/
	return 0;
}

static int ltr556_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr556_priv *obj = i2c_get_clientdata(client);
	int err;

	APS_FUN();

	if (!obj) {
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	/*ltr556_power(obj->hw, 1);*/

	atomic_set(&obj->als_suspend, 0);
	if (test_bit(CMC_BIT_ALS, &obj->enable)) {
		err = ltr556_als_enable(obj->client, 1);
	    if (err < 0) {
			APS_ERR("enable als fail: %d\n", err);
		}
	}
	/*
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable)) {
		err = ltr556_ps_enable(obj->client, 1);
		if (err < 0) {
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
	*/

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltr556_remove(void)
{
	APS_FUN();

	ltr556_power(hw, 0);
	i2c_del_driver(&ltr556_i2c_driver);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr556_local_init(void)
{
	int res = 0;

	APS_FUN();

	ltr556_power(hw, 1);

	if (i2c_add_driver(&ltr556_i2c_driver)) {
		APS_ERR("add driver error\n");
		res = -1;
		return res;
	}

	if (ltr556_init_flag == -1) {
		res = -1;
	   return res;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr556_init(void)
{
	alsps_driver_add(&ltr556_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr556_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr556_init);
module_exit(ltr556_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-556ALSPS Driver");
MODULE_LICENSE("GPL");

