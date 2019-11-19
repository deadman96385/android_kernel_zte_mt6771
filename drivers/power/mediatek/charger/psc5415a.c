/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros*/
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include "upmu_common.h"
#include "psc5415a.h"
#include "mtk_charger_intf.h"

struct pinctrl *pinctrl1_psc5415a;
struct pinctrl_state *psc_chg_en_low;
struct pinctrl_state *psc_chg_en_high;

int charger_vendor = 0;
const unsigned int VBAT_CVTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000
};

const unsigned int CSTH[] = {
	350000, 647000, 794000, 956000,
	1103000, 1206000, 1397000, 1544000
};
/*for sy charger chip*/
const unsigned int ITERM_CSTH[] = {
	61000,	121000,	182000,	243000,
	304000,	364000,	425000,	486000
};
/*psc5415a REG00 IINLIM[5:0]*/
const unsigned int INPUT_CSTH[] = {
	100000, 500000, 800000, 5000000
};

/* psc5415a REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150,
};

#ifdef CONFIG_OF
#else
#define psc5415a_SLAVE_ADDR_WRITE	0xD4
#define psc5415a_SLAVE_ADDR_Read	0xD5
#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define psc5415a_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define psc5415a_BUSNUM 0
#endif
#endif

struct psc5415a_info {
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	enum charger_type chg_type;
	int irq;
};

static struct i2c_client *new_client;
static const struct i2c_device_id psc5415a_i2c_id[] = { {"psc5415a", 0}, {} };

unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	if (val < array_size)
		return parameter[val];
	pr_info("Can't find the parameter\n");
	return parameter[0];
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size,
					const unsigned int val)
{
	unsigned int i;

	/*pr_debug_ratelimited("array_size = %d\n", array_size);*/

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_info("NO register value match\n");
	/* TODO: ASSERT(0);	// not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number,
					 unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				/* pr_info("sym zzf_%d<=%d, i=%d\n", pList[i], level, i); */
				return pList[i];
			}
		}
		/*pr_info("Can't find closest level\n");*/
		return pList[0];
		/* return 000; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		/*pr_info("Can't find closest level\n");*/
		return pList[number - 1];
		/* return 000; */
	}
}

unsigned char psc5415a_reg[PSC5415A_REG_NUM] = { 0 };
static DEFINE_MUTEX(psc5415a_i2c_access);
static DEFINE_MUTEX(psc5415a_access_lock);

static int psc5415a_read_byte(u8 reg_addr, u8 *rd_buf, int rd_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg[2];
	u8 *w_buf = NULL;
	u8 *r_buf = NULL;

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	w_buf = kzalloc(1, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	r_buf = kzalloc(rd_len, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;

	*w_buf = reg_addr;

	msg[0].addr = new_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = new_client->addr;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	ret = i2c_transfer(adap, msg, 2);

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

int psc5415a_write_byte(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	struct i2c_adapter *adap = new_client->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;
	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;

	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = new_client->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	ret = i2c_transfer(adap, &msg, 1);

	kfree(w_buf);
	return ret;
}

unsigned int psc5415a_read_interface(unsigned char reg_num, unsigned char *val, unsigned char MASK,
				unsigned char SHIFT)
{
	unsigned char psc5415a_reg = 0;
	unsigned int ret = 0;

	ret = psc5415a_read_byte(reg_num, &psc5415a_reg, 1);
	/*pr_debug_ratelimited("[psc5415a_read_interface] Reg[%x]=0x%x\n", reg_num, psc5415a_reg);*/
	psc5415a_reg &= (MASK << SHIFT);
	*val = (psc5415a_reg >> SHIFT);
	/*pr_debug_ratelimited("[psc5415a_read_interface] val=0x%x\n", *val);*/

	return ret;
}

unsigned int psc5415a_config_interface(unsigned char reg_num, unsigned char val, unsigned char MASK,
					unsigned char SHIFT)
{
	unsigned char psc5415a_reg = 0;
	unsigned char psc5415a_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&psc5415a_access_lock);
	ret = psc5415a_read_byte(reg_num, &psc5415a_reg, 1);
	psc5415a_reg_ori = psc5415a_reg;
	psc5415a_reg &= ~(MASK << SHIFT);
	psc5415a_reg |= (val << SHIFT);
	if (reg_num == PSC5415A_CON4)
		psc5415a_reg &= ~(1 << CON4_RESET_SHIFT);

	ret = psc5415a_write_byte(reg_num, &psc5415a_reg, 2);
	mutex_unlock(&psc5415a_access_lock);
//  pr_debug_ratelimited("[psc5415a_config_interface] write Reg[%x]=0x%x from 0x%x\n", reg_num,
//  		psc5415a_reg, psc5415a_reg_ori);
	/* Check */
	/* psc5415a_read_byte(reg_num, &psc5415a_reg, 1); */
	/* printk("[psc5415a_config_interface] Check Reg[%x]=0x%x\n", reg_num, psc5415a_reg); */

	return ret;
}

/* write one register directly */
unsigned int psc5415a_reg_config_interface(unsigned char reg_num, unsigned char val)
{
	unsigned char psc5415a_reg = val;

	return psc5415a_write_byte(reg_num, &psc5415a_reg, 2);
}

void psc5415a_set_tmr_rst(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_TMR_RST_MASK),
				(unsigned char)(CON0_TMR_RST_SHIFT)
				);
}

unsigned int psc5415a_get_otg_status(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_OTG_MASK),
				(unsigned char)(CON0_OTG_SHIFT)
				);
	return val;
}

void psc5415a_set_en_stat(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
}

unsigned int psc5415a_get_chip_status(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_STAT_MASK),
				(unsigned char)(CON0_STAT_SHIFT)
				);
	return val;
}

unsigned int psc5415a_get_boost_status(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_BOOST_MASK),
				(unsigned char)(CON0_BOOST_SHIFT)
				);
	return val;

}

unsigned int psc5415a_get_fault_status(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_FAULT_MASK),
				(unsigned char)(CON0_FAULT_SHIFT)
				);
	return val;
}

void psc5415a_set_input_charging_current(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);
}

unsigned int psc5415a_get_input_charging_current(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);

	return val;
}

void psc5415a_set_v_low(unsigned int val)
{

	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LOW_V_MASK),
				(unsigned char)(CON1_LOW_V_SHIFT)
				);
}

void psc5415a_set_te(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_TE_MASK),
				(unsigned char)(CON1_TE_SHIFT)
				);
}

void psc5415a_set_ce(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_CE_MASK),
				(unsigned char)(CON1_CE_SHIFT)
				);
}

void psc5415a_set_hz_mode(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
}

void psc5415a_set_opa_mode(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_OPA_MODE_MASK),
				(unsigned char)(CON1_OPA_MODE_SHIFT)
				);
}

void psc5415a_set_oreg(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OREG_MASK),
				(unsigned char)(CON2_OREG_SHIFT)
				);
}
void psc5415a_set_otg_pl(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_PL_MASK),
				(unsigned char)(CON2_OTG_PL_SHIFT)
				);
}
void psc5415a_set_otg_en(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_EN_MASK),
				(unsigned char)(CON2_OTG_EN_SHIFT)
				);
}
/*
unsigned int psc5415a_get_vender_code(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_VENDER_CODE_MASK),
				(unsigned char)(CON3_VENDER_CODE_SHIFT)
				);
	return val;
}
*/
unsigned int psc5415a_get_pn(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_PIN_MASK),
				(unsigned char)(CON3_PIN_SHIFT)
				);
	return val;
}

unsigned int psc5415a_get_revision(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_REVISION_MASK),
				(unsigned char)(CON3_REVISION_SHIFT)
				);
	return val;
}

void psc5415a_set_reset(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_RESET_MASK),
				(unsigned char)(CON4_RESET_SHIFT)
				);
}

void psc5415a_set_iocharge(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_CHR_MASK),
				(unsigned char)(CON4_I_CHR_SHIFT)
				);
}

void psc5415a_set_iterm(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_TERM_MASK),
				(unsigned char)(CON4_I_TERM_SHIFT)
				);
}

void psc5415a_set_dis_vreg(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_DIS_VREG_MASK),
				(unsigned char)(CON5_DIS_VREG_SHIFT)
				);
}

void psc5415a_set_io_level(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_IO_LEVEL_MASK),
				(unsigned char)(CON5_IO_LEVEL_SHIFT)
				);
}

unsigned int psc5415a_get_sp_status(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_SP_STATUS_MASK),
				(unsigned char)(CON5_SP_STATUS_SHIFT)
				);
	return val;
}

unsigned int psc5415a_get_en_level(void)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_EN_LEVEL_MASK),
				(unsigned char)(CON5_EN_LEVEL_SHIFT)
				);
	return val;
}

void psc5415a_set_vsp(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_VSP_MASK),
				(unsigned char)(CON5_VSP_SHIFT)
				);
}

void psc5415a_set_i_safe(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_ISAFE_MASK),
				(unsigned char)(CON6_ISAFE_SHIFT)
				);
}

void psc5415a_set_v_safe(unsigned int val)
{
	psc5415a_config_interface((unsigned char)(PSC5415A_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_VSAFE_MASK),
				(unsigned char)(CON6_VSAFE_SHIFT)
				);
}

static int psc5415a_dump_register(struct charger_device *chg_dev)
{
	int i;

	printk("psc5415: ");
	for (i = 0; i < PSC5415A_REG_NUM; i++) {
		psc5415a_read_byte(i, &psc5415a_reg[i], 1);
		printk("[0x%x]=0x%x ", i, psc5415a_reg[i]);
	}
	pr_debug("\n");

	return 0;
}

static int psc5415a_parse_dt(struct psc5415a_info *info, struct device *dev)
{
	struct device_node *np = dev->of_node;

	pr_info("%s\n", __func__);

	if (!np) {
		pr_err("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name", &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		pr_warn("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name", &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "psc5415a";
		pr_warn("%s: no alias name\n", __func__);
	}

	return 0;
}

static int psc5415a_do_event(struct charger_device *chg_dev, unsigned int event, unsigned int args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_info("%s: event = %d\n", __func__, event);

	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

static int psc5415a_enable_charging(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;
	static bool init = false;
	static bool en_last;
	static int set_times = 0;

	if(charger_vendor != 7){
		if (en) {
			psc5415a_set_ce(0);
			psc5415a_set_hz_mode(0);
			psc5415a_set_opa_mode(0);
		} else {
			psc5415a_set_ce(1);
		}
		return status;
	}

	if(init == true) {
		if(en_last == en) {
			set_times++;
			if(set_times < 10) {
				return status;
			}
		}
	}

	if (IS_ERR(pinctrl1_psc5415a))
		pr_info("pinctrl1_psc5415a is error!\n");
	else {
		if(en) {
			pinctrl_select_state(pinctrl1_psc5415a, psc_chg_en_low);
			pr_info("psc5415a_enable_charging: set psc_chg_en_low\n");
		}else{
			pinctrl_select_state(pinctrl1_psc5415a, psc_chg_en_high);
			pr_info("psc5415a_enable_charging: set psc_chg_en_high\n");
		}
	}

	set_times = 0;
	en_last = en;
	init = true;
	return status;
}

static int psc5415a_is_charging_enabled(struct charger_device *chg_dev, bool *en)
{
	unsigned char val = 0;

	psc5415a_read_interface((unsigned char)(PSC5415A_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_CE_MASK),
				(unsigned char)(CON1_CE_SHIFT)
				);
	if (val == 1)
		*en = false;
	else
		*en = true;
	pr_debug("%s val: %d, en: %d\n", __func__, val, *en);

	return *en;
}

static int psc5415a_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;
	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CVTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CVTH, array_size, cv);

	register_value =
	charging_parameter_to_value(VBAT_CVTH, array_size, set_cv_voltage);
	/* pr_info("charging_set_cv_voltage register_value=0x%x %d %d\n",
	 register_value, cv, set_cv_voltage); */
	psc5415a_set_oreg(register_value);

	return status;
}

static int psc5415a_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	int status = 0;
	unsigned int array_size;
	unsigned char reg_value;

	array_size = ARRAY_SIZE(CSTH);
	psc5415a_read_interface(0x1, &reg_value, 0x3, 0x6);	/* IINLIM */
	*ichg = charging_value_to_parameter(CSTH, array_size, reg_value);

	return status;
}

static int psc5415a_set_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (current_value <= 35000) {
		psc5415a_set_io_level(1);
	} else {
		psc5415a_set_io_level(0);
		array_size = ARRAY_SIZE(CSTH);
		set_chr_current = bmt_find_closest_level(CSTH, array_size, current_value);
		register_value = charging_parameter_to_value(CSTH, array_size, set_chr_current);
		psc5415a_set_iocharge(register_value);
	}
//printk("sym %s  current_value=%d,set_chr_current=%d,register_value=%d\n",__func__,current_value,set_chr_current,register_value);
	return status;
}

static int psc5415a_enable_otg(struct charger_device *chg_dev, bool en)
{
	unsigned int status = 0;
	printk("sym %s en:%d\n",__func__,en);
	if(en ==1){
		psc5415a_set_opa_mode(1);
		psc5415a_set_otg_pl(1);
		  psc5415a_set_otg_en(1);
		}
	else
	{
	psc5415a_set_opa_mode(0);
	psc5415a_set_otg_pl(0);
	  psc5415a_set_otg_en(0);
		//  psc5415a_write_byte(0x01,0x30);
		//  psc5415a_write_byte(0x02,0x8e); //reset
	}
	return status;

}
static int psc5415a_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	unsigned int status = 0;
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = psc5415a_get_input_charging_current();
	*aicr = charging_parameter_to_value(INPUT_CSTH, array_size, register_value);

	return status;
}

static int psc5415a_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	unsigned int status = 0;
	unsigned int set_chr_current;
	unsigned int array_size;
	unsigned int register_value;

	if (current_value > 500000) {
		register_value = 0x3;
	} else {
		array_size = ARRAY_SIZE(INPUT_CSTH);
		set_chr_current = bmt_find_closest_level(INPUT_CSTH, array_size, current_value);
		register_value =
	 charging_parameter_to_value(INPUT_CSTH, array_size, set_chr_current);
	}
//printk("sym %s  current_value=%d,set_chr_current=%d,register_value=%d\n",__func__,current_value,set_chr_current,register_value);
	psc5415a_set_input_charging_current(register_value);

	return status;
}

static int psc5415a_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	unsigned int status = 0;
	unsigned int ret_val;

	ret_val = psc5415a_get_chip_status();

	if (ret_val == 0x2)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static int psc5415a_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	psc5415a_set_tmr_rst(1);
	return 0;
}
/*
static int psc5415a_get_vender(struct charger_device *chg_dev,int *vendor)
{
	*vendor = psc5415a_get_vender_code();
	return 0;
}
*/
static int psc5415a_set_eoc_current(struct charger_device *chg_dev, u32 iterm_current)
{
	unsigned int status = 0;
	unsigned int set_iterm_current;
	unsigned int array_size;
	unsigned int register_value;

	if(iterm_current <= 0) {
		psc5415a_set_te(0);
	}else{
		psc5415a_set_te(1);
		array_size = ARRAY_SIZE(ITERM_CSTH);

		set_iterm_current = bmt_find_closest_level(ITERM_CSTH, array_size, iterm_current);
		register_value = charging_parameter_to_value(ITERM_CSTH, array_size, set_iterm_current);
		psc5415a_set_iterm(register_value);
	}
	//printk("%s  iterm_current=%d,set_iterm_current=%d,register_value=%d\n",__func__,iterm_current,set_iterm_current,register_value);
	return status;
}

static struct charger_ops psc5415a_chg_ops = {

	/* Normal charging */
	.dump_registers = psc5415a_dump_register,
	.enable = psc5415a_enable_charging,
	.is_enabled = psc5415a_is_charging_enabled,
	.get_charging_current = psc5415a_get_current,
	.set_charging_current = psc5415a_set_current,
	.get_input_current = psc5415a_get_input_current,
	.set_input_current = psc5415a_set_input_current,
	/*.get_constant_voltage = psc5415a_get_battery_voreg,*/
	.set_constant_voltage = psc5415a_set_cv_voltage,
	.kick_wdt = psc5415a_reset_watch_dog_timer,
	.is_charging_done = psc5415a_get_charging_status,
	.enable_otg = psc5415a_enable_otg,
	.event = psc5415a_do_event,
	/*.get_vender = psc5415a_get_vender,*/
	.set_eoc_current = psc5415a_set_eoc_current,
};

int psc5415a_get_gpio_info(struct psc5415a_info *info)
{
	int ret = 0;

	pinctrl1_psc5415a = devm_pinctrl_get(info->dev);
	if (IS_ERR(pinctrl1_psc5415a)) {
		ret = PTR_ERR(pinctrl1_psc5415a);
		printk("psc5415a_get_gpio_info Cannot find charger pinctrl1_psc5415a!\n");
		return ret;
	}

	psc_chg_en_low = pinctrl_lookup_state(pinctrl1_psc5415a, "psc_chg_en_low");
	if (IS_ERR(psc_chg_en_low)) {
		ret = PTR_ERR(psc_chg_en_low);
		printk("psc5415a_get_gpio_info Cannot find charger pinctrl psc_chg_en_low %d!\n", ret);
	}
	psc_chg_en_high = pinctrl_lookup_state(pinctrl1_psc5415a, "psc_chg_en_high");
	if (IS_ERR(psc_chg_en_high)) {
		ret = PTR_ERR(psc_chg_en_high);
		printk("psc5415a_get_gpio_info Cannot find charger pinctrl psc_chg_en_high!\n");
		return ret;
	}

	return ret;
}

static int psc5415a_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct psc5415a_info *info = NULL;

	pr_err("[psc5415a_driver_probe]\n");
	info = devm_kzalloc(&client->dev, sizeof(struct psc5415a_info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;
	ret = psc5415a_parse_dt(info, &client->dev);

	if (ret < 0)
		return ret;

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
		&client->dev, info, &psc5415a_chg_ops, &info->chg_props);

	if (IS_ERR_OR_NULL(info->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}
        /*
		charger_vendor = psc5415a_get_vender_code();
		pr_err("%s: psc5415a_get_vender_code= %d\n", __func__,charger_vendor);
		charger_vendor = psc5415a_get_vender_code();
		pr_err("%s: psc5415a_get_vender_code= %d\n", __func__,charger_vendor);
        */
	/* psc5415a_hw_init(); //move to charging_hw_xxx.c */
	info->psy = power_supply_get_by_name("charger");

	if (!info->psy) {
		pr_err("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

	psc5415a_reg_config_interface(0x06, 0x7A);	/* ISAFE = 1518mA, VSAFE = 4.4V */
	psc5415a_reg_config_interface(0x06, 0x7A);

	psc5415a_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
	psc5415a_reg_config_interface(0x01, 0xb8);	/* TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
	psc5415a_reg_config_interface(0x05, 0x04);

	psc5415a_reg_config_interface(0x04, 0x79);	/* 121mA */

	psc5415a_dump_register(info->chg_dev);

	psc5415a_get_gpio_info(info);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id psc5415a_of_match[] = {
	{.compatible = "mediatek,swithing_charger"},
//  {.compatible = "halo,psc5415a"},
	{},
};
#else
static struct i2c_board_info i2c_psc5415a __initdata = {
	I2C_BOARD_INFO("psc5415a", (psc5415a_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver psc5415a_driver = {
	.driver = {
		.name = "psc5415a",
#ifdef CONFIG_OF
		.of_match_table = psc5415a_of_match,
#endif
		},
	.probe = psc5415a_driver_probe,
	.id_table = psc5415a_i2c_id,
};

static int __init psc5415a_init(void)
{

	if (i2c_add_driver(&psc5415a_driver) != 0)
		pr_info("Failed to register psc5415a i2c driver.\n");
	else
		pr_info("Success to register psc5415a i2c driver.\n");

	return 0;
}

static void __exit psc5415a_exit(void)
{
	i2c_del_driver(&psc5415a_driver);
}

module_init(psc5415a_init);
module_exit(psc5415a_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C psc5415a Driver");
MODULE_AUTHOR("Henry Chen<henryc.chen@mediatek.com>");
