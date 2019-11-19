/* drivers/power/mediatek/charger_class.c
 * Switching Charger Class Device Driver
 *
 * Copyright (C) 2013 Richtek Technology Corp.
 * Author: Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <mt-plat/charger_class.h>

#ifdef CONFIG_CHARGER_PMIC_VOTER
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>

#define MTK_SETTING_VOTER			"MTK_SETTING_VOTER"
#define CAS_SETTING_VOTER			"CAS_SETTING_VOTER"
#define ADB_SETTING_VOTER			"ADB_SETTING_VOTER"
#define PRIMARY_CHARGER_DEVICE_NMAE			"primary_chg"
#define SECONDARY_CHARGER_DEVICE_NMAE		"secondary_chg"


#define cas_debug(fmt, args...)	pr_info("VOTE: %s(): "fmt, __func__, ## args)

struct charge_votable_struct {
	/*struct charger_device *chg1_dev;*/
	struct power_supply	*interface_psy;
	struct votable		*fcc_votable;
	struct votable		*fcv_votable;
	struct votable		*topoff_votable;
	struct votable		*recharge_soc_votable;
	struct votable		*recharge_voltage_votable;
	struct votable		*usb_icl_votable;
	atomic_t			init_finished;
};

static struct charge_votable_struct charge_votable_data;
#endif

static struct class *charger_class;

static ssize_t charger_show_name(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct charger_device *charger_dev = to_charger_device(dev);

	return snprintf(buf, 20, "%s\n",
		       charger_dev->props.alias_name ?
		       charger_dev->props.alias_name : "anonymous");
}

static int charger_suspend(struct device *dev, pm_message_t state)
{
	struct charger_device *charger_dev = to_charger_device(dev);

	if (charger_dev->ops->suspend)
		return charger_dev->ops->suspend(charger_dev, state);

	return -ENOTSUPP;
}

static int charger_resume(struct device *dev)
{
	struct charger_device *charger_dev = to_charger_device(dev);

	if (charger_dev->ops->resume)
		return charger_dev->ops->resume(charger_dev);

	return -ENOTSUPP;
}

static void charger_device_release(struct device *dev)
{
	struct charger_device *charger_dev = to_charger_device(dev);

	kfree(charger_dev);
}

int charger_dev_enable(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable)
		return charger_dev->ops->enable(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable);

int charger_dev_is_enabled(struct charger_device *charger_dev, bool *en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->is_enabled)
		return charger_dev->ops->is_enabled(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_enabled);

int charger_dev_plug_in(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->plug_in)
		return charger_dev->ops->plug_in(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_plug_in);

int charger_dev_plug_out(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->plug_out)
		return charger_dev->ops->plug_out(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_plug_out);

int charger_dev_do_event(struct charger_device *charger_dev, u32 event, u32 args)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->event)
		return charger_dev->ops->event(charger_dev, event, args);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_do_event);

int charger_dev_set_charging_current(struct charger_device *charger_dev, u32 uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	/*charger_dev2 is persent, Not doing vote*/
	if (charger_dev2) {
		if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_charging_current)
			return charger_dev->ops->set_charging_current(charger_dev, uA);

		return -ENOTSUPP;
	}

	/*charger_dev2 is not persent, vote fcc*/
	if (get_client_vote(charge_votable_data.fcc_votable, MTK_SETTING_VOTER) != uA)
		vote(charge_votable_data.fcc_votable, MTK_SETTING_VOTER, true, uA);
	else
		rerun_election(charge_votable_data.fcc_votable);

	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_charging_current)
		return charger_dev->ops->set_charging_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_charging_current);

int charger_dev_get_charging_current(struct charger_device *charger_dev, u32 *uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	if (charger_dev2) {
		if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_charging_current)
			return charger_dev->ops->get_charging_current(charger_dev, uA);

		return -ENOTSUPP;
	}

	*uA = get_client_vote(charge_votable_data.fcc_votable, MTK_SETTING_VOTER);
	cas_debug("MTK get fcc %d\n", *uA);
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_charging_current)
		return charger_dev->ops->get_charging_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_charging_current);

int charger_dev_get_min_charging_current(struct charger_device *charger_dev, u32 *uA)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_min_charging_current)
		return charger_dev->ops->get_min_charging_current(charger_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_min_charging_current);

int charger_dev_enable_chip(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_chip)
		return charger_dev->ops->enable_chip(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_chip);

int charger_dev_is_chip_enabled(struct charger_device *charger_dev, bool *en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->is_chip_enabled)
		return charger_dev->ops->is_chip_enabled(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_chip_enabled);

int charger_dev_enable_direct_charging(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_direct_charging)
		return charger_dev->ops->enable_direct_charging(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_direct_charging);

int charger_dev_kick_direct_charging_wdt(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->kick_direct_charging_wdt)
		return charger_dev->ops->kick_direct_charging_wdt(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_kick_direct_charging_wdt);

int charger_dev_get_ibus(struct charger_device *charger_dev, u32 *ibus)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_ibus_adc)
		return charger_dev->ops->get_ibus_adc(charger_dev, ibus);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_ibus);

int charger_dev_get_temperature(struct charger_device *charger_dev, int *tchg_min,
		int *tchg_max)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_tchg_adc)
		return charger_dev->ops->get_tchg_adc(charger_dev, tchg_min, tchg_max);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_temperature);

int charger_dev_set_input_current(struct charger_device *charger_dev, u32 uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	if (!strcmp(dev_name(&charger_dev->dev), PRIMARY_CHARGER_DEVICE_NMAE)) {
		if (get_client_vote(charge_votable_data.usb_icl_votable, MTK_SETTING_VOTER) != uA)
			vote(charge_votable_data.usb_icl_votable, MTK_SETTING_VOTER, true, uA);
		else
			rerun_election(charge_votable_data.usb_icl_votable);
	}
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_input_current)
		return charger_dev->ops->set_input_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_input_current);

int charger_dev_get_input_current(struct charger_device *charger_dev, u32 *uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	*uA = get_client_vote(charge_votable_data.usb_icl_votable, MTK_SETTING_VOTER);
	cas_debug("MTK get icl %d\n", *uA);
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_input_current)
		return charger_dev->ops->get_input_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_input_current);

int charger_dev_set_ship_mode(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_ship_mode)
		return charger_dev->ops->set_ship_mode(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_ship_mode);

int charger_dev_get_ship_mode(struct charger_device *charger_dev, bool *en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_ship_mode)
		return charger_dev->ops->get_ship_mode(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_ship_mode);

int charger_dev_get_min_input_current(struct charger_device *charger_dev, u32 *uA)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_min_input_current)
		return charger_dev->ops->get_min_input_current(charger_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_min_input_current);

int charger_dev_set_eoc_current(struct charger_device *charger_dev, u32 uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	/*charger_dev2 is persent, Not doing vote*/
	if (charger_dev2) {
		if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_eoc_current)
			return charger_dev->ops->set_eoc_current(charger_dev, uA);

		return -ENOTSUPP;
	}

	/*charger_dev2 is not persent, vote topoff*/
	if (get_client_vote(charge_votable_data.topoff_votable, MTK_SETTING_VOTER) != uA)
		vote(charge_votable_data.topoff_votable, MTK_SETTING_VOTER, true, uA);
	else
		rerun_election(charge_votable_data.topoff_votable);

	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_eoc_current)
		charger_dev->ops->set_eoc_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_eoc_current);

int charger_dev_get_eoc_current(struct charger_device *charger_dev, u32 *uA)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	if (charger_dev2) {
		if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_eoc_current)
			return charger_dev->ops->get_eoc_current(charger_dev, uA);

		return -ENOTSUPP;
	}

	*uA = get_client_vote(charge_votable_data.topoff_votable, MTK_SETTING_VOTER);
	cas_debug("MTK get topoff %d\n", *uA);
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_eoc_current)
		return charger_dev->ops->get_eoc_current(charger_dev, uA);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_eoc_current);

int charger_dev_kick_wdt(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->kick_wdt)
		return charger_dev->ops->kick_wdt(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_kick_wdt);

int charger_dev_set_constant_voltage(struct charger_device *charger_dev, u32 uV)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	if (!strcmp(dev_name(&charger_dev->dev), PRIMARY_CHARGER_DEVICE_NMAE)) {
		if (get_client_vote(charge_votable_data.fcv_votable, MTK_SETTING_VOTER) != uV)
			vote(charge_votable_data.fcv_votable, MTK_SETTING_VOTER, true, uV);
		else
			rerun_election(charge_votable_data.fcv_votable);
	}
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_constant_voltage)
		return charger_dev->ops->set_constant_voltage(charger_dev, uV);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_constant_voltage);

int charger_dev_get_constant_voltage(struct charger_device *charger_dev, u32 *uV)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	*uV = get_effective_result(charge_votable_data.fcv_votable);
	cas_debug("get fcv %d\n", *uV);
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_constant_voltage)
		return charger_dev->ops->get_constant_voltage(charger_dev, uV);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_constant_voltage);

#ifdef CONFIG_CHARGER_PMIC_VOTER
int charger_dev_set_recharge_voltage(struct charger_device *charger_dev, u32 uV)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	if (!strcmp(dev_name(&charger_dev->dev), PRIMARY_CHARGER_DEVICE_NMAE)) {
		if (get_client_vote(charge_votable_data.recharge_voltage_votable, MTK_SETTING_VOTER) != uV)
			vote(charge_votable_data.recharge_voltage_votable, MTK_SETTING_VOTER, true, uV);
		else
			rerun_election(charge_votable_data.recharge_voltage_votable);
	}
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_recharge_voltage)
		return charger_dev->ops->set_recharge_voltage(charger_dev, uV);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_recharge_voltage);

int charger_dev_get_recharge_voltage(struct charger_device *charger_dev, u32 *uV)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	*uV = get_client_vote(charge_votable_data.recharge_voltage_votable, MTK_SETTING_VOTER);
	cas_debug("MTK get recharge voltage %d\n", *uV);
	return 0;
#else
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_recharge_voltage)
		return charger_dev->ops->get_recharge_voltage(charger_dev, uV);
#endif

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_recharge_voltage);
#endif

int charger_dev_dump_registers(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->dump_registers)
		return charger_dev->ops->dump_registers(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_dump_registers);

int charger_dev_is_charging_done(struct charger_device *charger_dev, bool *done)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->is_charging_done)
		return charger_dev->ops->is_charging_done(charger_dev, done);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_charging_done);

int charger_dev_enable_vbus_ovp(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_vbus_ovp)
		return charger_dev->ops->enable_vbus_ovp(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_vbus_ovp);

int charger_dev_set_mivr(struct charger_device *charger_dev, u32 uV)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_mivr)
		return charger_dev->ops->set_mivr(charger_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_mivr);

int charger_dev_enable_powerpath(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_powerpath)
		return charger_dev->ops->enable_powerpath(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_powerpath);

int charger_dev_is_powerpath_enabled(struct charger_device *charger_dev, bool *en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->is_powerpath_enabled)
		return charger_dev->ops->is_powerpath_enabled(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_powerpath_enabled);

int charger_dev_enable_safety_timer(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_safety_timer)
		return charger_dev->ops->enable_safety_timer(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_safety_timer);

int charger_dev_is_safety_timer_enabled(struct charger_device *charger_dev, bool *en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->is_safety_timer_enabled)
		return charger_dev->ops->is_safety_timer_enabled(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_safety_timer_enabled);

int charger_dev_enable_termination(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_termination)
		return charger_dev->ops->enable_termination(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_termination);

int charger_dev_get_mivr_state(struct charger_device *charger_dev, bool *in_loop)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_mivr_state)
		return charger_dev->ops->get_mivr_state(charger_dev, in_loop);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_mivr_state);

int charger_dev_send_ta_current_pattern(struct charger_device *charger_dev, bool is_increase)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->send_ta_current_pattern)
		return charger_dev->ops->send_ta_current_pattern(charger_dev, is_increase);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_send_ta_current_pattern);

#ifdef ZTE_PE_PLUS_INCREASE_TA_VCHR
int charger_dev_send_ta_current_pattern_zte(struct charger_device *charger_dev, bool is_increase)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->send_ta_current_pattern_zte)
		return charger_dev->ops->send_ta_current_pattern_zte(charger_dev, is_increase);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_send_ta_current_pattern_zte);
#endif

int charger_dev_send_ta20_current_pattern(struct charger_device *charger_dev, u32 uV)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->send_ta20_current_pattern)
		return charger_dev->ops->send_ta20_current_pattern(charger_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_send_ta20_current_pattern);

int charger_dev_reset_ta(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->reset_ta)
		return charger_dev->ops->reset_ta(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_reset_ta);

int charger_dev_set_pe20_efficiency_table(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_pe20_efficiency_table)
		return charger_dev->ops->set_pe20_efficiency_table(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_pe20_efficiency_table);

int charger_dev_enable_cable_drop_comp(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_cable_drop_comp)
		return charger_dev->ops->enable_cable_drop_comp(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_cable_drop_comp);

int charger_dev_set_direct_charging_ibusoc(struct charger_device *charger_dev, u32 uA)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_direct_charging_ibusoc)
		return charger_dev->ops->set_direct_charging_ibusoc(charger_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_direct_charging_ibusoc);

int charger_dev_set_direct_charging_vbusov(struct charger_device *charger_dev, u32 uV)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_direct_charging_vbusov)
		return charger_dev->ops->set_direct_charging_vbusov(charger_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_direct_charging_vbusov);

int charger_dev_enable_chg_type_det(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_chg_type_det)
		return charger_dev->ops->enable_chg_type_det(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_chg_type_det);

int charger_dev_enable_otg(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_otg)
		return charger_dev->ops->enable_otg(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_otg);

int charger_dev_enable_discharge(struct charger_device *charger_dev, bool en)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->enable_discharge)
		return charger_dev->ops->enable_discharge(charger_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_discharge);

int charger_dev_set_boost_current_limit(struct charger_device *charger_dev, u32 uA)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_boost_current_limit)
		return charger_dev->ops->set_boost_current_limit(charger_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_boost_current_limit);

int charger_dev_get_zcv(struct charger_device *charger_dev, u32 *uV)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->get_zcv)
		return charger_dev->ops->get_zcv(charger_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_zcv);

int charger_dev_run_aicl(struct charger_device *charger_dev, u32 *uA)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->run_aicl)
		return charger_dev->ops->run_aicl(charger_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_run_aicl);

int charger_dev_reset_eoc_state(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->reset_eoc_state)
		return charger_dev->ops->reset_eoc_state(charger_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_reset_eoc_state);

int charger_dev_safety_check(struct charger_device *charger_dev)
{
	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->safety_check)
		return charger_dev->ops->safety_check(charger_dev);

	return -ENOTSUPP;
}

int charger_dev_notify(struct charger_device *charger_dev, int event)
{
	return srcu_notifier_call_chain(
		&charger_dev->evt_nh, event, &charger_dev->noti);
}

static DEVICE_ATTR(name, S_IRUGO, charger_show_name, NULL);

static struct attribute *charger_class_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group charger_group = {
	.attrs = charger_class_attrs,
};

static const struct attribute_group *charger_groups[] = {
	&charger_group,
	NULL,
};

int register_charger_device_notifier(struct charger_device *charger_dev,
			      struct notifier_block *nb)
{
	int ret;

	ret = srcu_notifier_chain_register(&charger_dev->evt_nh, nb);
	return ret;
}
EXPORT_SYMBOL(register_charger_device_notifier);

int unregister_charger_device_notifier(struct charger_device *charger_dev,
				struct notifier_block *nb)
{
	return srcu_notifier_chain_unregister(&charger_dev->evt_nh, nb);
}
EXPORT_SYMBOL(unregister_charger_device_notifier);

/**
 * charger_device_register - create and register a new object of
 *   charger_device class.
 * @name: the name of the new object
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use.
 * The methods may retrieve it by using charger_get_data(charger_dev).
 * @ops: the charger operations structure.
 *
 * Creates and registers new charger device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct charger_device *charger_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct charger_ops *ops,
		const struct charger_properties *props)
{
	struct charger_device *charger_dev;
	int rc;

	pr_debug("charger_device_register: name=%s\n", name);
	charger_dev = kzalloc(sizeof(*charger_dev), GFP_KERNEL);
	if (!charger_dev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&charger_dev->ops_lock);
	charger_dev->dev.class = charger_class;
	charger_dev->dev.parent = parent;
	charger_dev->dev.release = charger_device_release;
	dev_set_name(&charger_dev->dev, name);
	dev_set_drvdata(&charger_dev->dev, devdata);
	srcu_init_notifier_head(&charger_dev->evt_nh);

	/* Copy properties */
	if (props) {
		memcpy(&charger_dev->props, props,
		       sizeof(struct charger_properties));
	}
	rc = device_register(&charger_dev->dev);
	if (rc) {
		kfree(charger_dev);
		return ERR_PTR(rc);
	}
	charger_dev->ops = ops;
	return charger_dev;
}
EXPORT_SYMBOL(charger_device_register);

/**
 * charger_device_unregister - unregisters a switching charger device
 * object.
 * @charger_dev: the switching charger device object to be unregistered
 * and freed.
 *
 * Unregisters a previously registered via charger_device_register object.
 */
void charger_device_unregister(struct charger_device *charger_dev)
{
	if (!charger_dev)
		return;

	mutex_lock(&charger_dev->ops_lock);
	charger_dev->ops = NULL;
	mutex_unlock(&charger_dev->ops_lock);
	device_unregister(&charger_dev->dev);
}
EXPORT_SYMBOL(charger_device_unregister);


static int charger_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct charger_device *get_charger_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct charger_device *)NULL;
	dev = class_find_device(charger_class, NULL, name,
				charger_match_device_by_name);

	return dev ? to_charger_device(dev) : NULL;

}
EXPORT_SYMBOL(get_charger_by_name);

static void __exit charger_class_exit(void)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	atomic_set(&charge_votable_data.init_finished, 0);
	destroy_votable(charge_votable_data.fcc_votable);
	destroy_votable(charge_votable_data.fcv_votable);
	destroy_votable(charge_votable_data.topoff_votable);
	destroy_votable(charge_votable_data.recharge_soc_votable);
	destroy_votable(charge_votable_data.recharge_voltage_votable);
	destroy_votable(charge_votable_data.usb_icl_votable);
#endif

	class_destroy(charger_class);
}

#ifdef CONFIG_CHARGER_PMIC_VOTER
static int charger_fcc_vote_callback(struct votable *votable,
			void *data, int max_fcc_ua, const char *client)
{
	/*struct charge_votable_struct *charge_votable_data = data;*/
	struct charger_device *charger_dev1 = get_charger_by_name(PRIMARY_CHARGER_DEVICE_NMAE);
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	if (charger_dev2) {
		cas_debug("charger_dev2 present, return\n");
		return 0;
	}

	cas_debug("client: %s max_fcc_ua: %d\n", client, max_fcc_ua);

	if (charger_dev1 != NULL && charger_dev1->ops != NULL && charger_dev1->ops->set_charging_current)
		charger_dev1->ops->set_charging_current(charger_dev1, max_fcc_ua);

	return 0;
}

static int charger_fcv_vote_callback(struct votable *votable,
			void *data, int max_fcv_uv, const char *client)
{
	struct charger_device *charger_dev1 = get_charger_by_name(PRIMARY_CHARGER_DEVICE_NMAE);
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	cas_debug("client: %s max_fcv_uv: %d\n", client, max_fcv_uv);

	if (charger_dev1 != NULL && charger_dev1->ops != NULL && charger_dev1->ops->set_constant_voltage)
		charger_dev1->ops->set_constant_voltage(charger_dev1, max_fcv_uv);

	if (charger_dev2 != NULL && charger_dev2->ops != NULL && charger_dev2->ops->set_constant_voltage)
		charger_dev2->ops->set_constant_voltage(charger_dev2, max_fcv_uv + 200000);

	return 0;
}

static int charger_topoff_vote_callback(struct votable *votable,
			void *data, int min_topoff_ua, const char *client)
{
	/*struct charge_votable_struct *charge_votable_data = data;*/
	struct charger_device *charger_dev1 = get_charger_by_name(PRIMARY_CHARGER_DEVICE_NMAE);
	struct charger_device *charger_dev2 = get_charger_by_name(SECONDARY_CHARGER_DEVICE_NMAE);

	if (charger_dev2) {
		cas_debug("charger_dev2 present, return\n");
		return 0;
	}

	cas_debug("client: %s min_topoff_ua: %d\n", client, min_topoff_ua);

	if (charger_dev1 != NULL && charger_dev1->ops != NULL && charger_dev1->ops->set_eoc_current)
		charger_dev1->ops->set_eoc_current(charger_dev1, min_topoff_ua);

	return 0;
}

static int charger_recharge_soc_vote_callback(struct votable *votable,
			void *data, int recharge_soc, const char *client)
{
	/*struct charger_device *charger_dev = charge_votable_data->chg1_dev;*/

	return 0;
}

static int charger_recharge_voltage_vote_callback(struct votable *votable,
			void *data, int recharge_voltage, const char *client)
{
	struct charger_device *charger_dev = get_charger_by_name(PRIMARY_CHARGER_DEVICE_NMAE);

	cas_debug("client: %s recharge_voltage: %d\n", client, recharge_voltage);

	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_recharge_voltage)
		charger_dev->ops->set_recharge_voltage(charger_dev, recharge_voltage);

	return 0;
}

static int charger_usb_icl_vote_callback(struct votable *votable,
			void *data, int max_icl_ua, const char *client)
{
	struct charger_device *charger_dev = get_charger_by_name(PRIMARY_CHARGER_DEVICE_NMAE);

	cas_debug("client: %s enable: %d\n", client, max_icl_ua);

	if (charger_dev != NULL && charger_dev->ops != NULL && charger_dev->ops->set_input_current)
		charger_dev->ops->set_input_current(charger_dev, max_icl_ua);

	return 0;
}

static int interface_psy_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *pval)
{
	struct charge_votable_struct *pdata = power_supply_get_drvdata(psy);
	int rc = 0;

	if (!pdata || !atomic_read(&pdata->init_finished)) {
		cas_debug("interface Uninitialized!!!\n");
		return -ENODATA;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		pval->intval = get_client_vote(pdata->fcc_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		pval->intval = get_client_vote(pdata->fcv_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		pval->intval = get_client_vote(pdata->usb_icl_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		pval->intval = get_client_vote(pdata->topoff_votable, CAS_SETTING_VOTER);
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		pval->intval = get_client_vote(pdata->recharge_voltage_votable, CAS_SETTING_VOTER);
		break;
	default:
		cas_debug("interface unsupported property %d\n", psp);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int interface_psy_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *pval)
{
	struct charge_votable_struct *pdata = power_supply_get_drvdata(psy);
	int rc = 0;

	if (!pdata || !atomic_read(&pdata->init_finished)) {
		cas_debug("interface Uninitialized!!!\n");
		return -ENODATA;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (pval->intval > 0)
			vote(pdata->fcc_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(pdata->fcc_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		if (pval->intval > 0)
			vote(pdata->fcv_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(pdata->fcv_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		vote(pdata->usb_icl_votable, CAS_SETTING_VOTER, pval->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		if (pval->intval > 0)
			vote(pdata->topoff_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(pdata->topoff_votable, CAS_SETTING_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		if (pval->intval > 0)
			vote(pdata->recharge_voltage_votable, CAS_SETTING_VOTER, true, pval->intval);
		else
			vote(pdata->recharge_voltage_votable, CAS_SETTING_VOTER, false, 0);
		break;
	default:
		cas_debug("interface unsupported property %d\n", psp);
		rc = -EINVAL;
		break;
	}


	return rc;
}

static int interface_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CALIBRATE:
		return 1;
	default:
		break;
	}

	return 0;
}

static void interface_external_power_changed(struct power_supply *psy)
{
	cas_debug("power supply changed\n");
}

static enum power_supply_property interface_psy_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static const struct power_supply_desc interface_psy_desc = {
	.name = "interface",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = interface_psy_props,
	.num_properties = ARRAY_SIZE(interface_psy_props),
	.get_property = interface_psy_get_property,
	.set_property = interface_psy_set_property,
	.external_power_changed = interface_external_power_changed,
	.property_is_writeable = interface_property_is_writeable,
};
#endif

static int __init charger_class_init(void)
{
#ifdef CONFIG_CHARGER_PMIC_VOTER
	struct power_supply_config interface_psy_cfg;
	int rc = 0;
#endif

	charger_class = class_create(THIS_MODULE, "switching_charger");
	if (IS_ERR(charger_class)) {
		pr_err("Unable to create charger class; errno = %ld\n",
			PTR_ERR(charger_class));
		return PTR_ERR(charger_class);
	}
	charger_class->dev_groups = charger_groups;
	charger_class->suspend = charger_suspend;
	charger_class->resume = charger_resume;

#ifdef CONFIG_CHARGER_PMIC_VOTER
	atomic_set(&charge_votable_data.init_finished, 0);

	/* Register the power supply */
	interface_psy_cfg.drv_data = &charge_votable_data;
	interface_psy_cfg.of_node = NULL;
	interface_psy_cfg.supplied_to = NULL;
	interface_psy_cfg.num_supplicants = 0;
	charge_votable_data.interface_psy = power_supply_register(NULL, &interface_psy_desc,
					&interface_psy_cfg);
	if (IS_ERR(charge_votable_data.interface_psy)) {
		rc = PTR_ERR(charge_votable_data.interface_psy);
		cas_debug("failed to register bcl_psy rc = %ld\n",
				PTR_ERR(charge_votable_data.interface_psy));
		goto register_power_supply_failed;
	}

	charge_votable_data.fcc_votable = create_votable("FCC", VOTE_MIN,
					charger_fcc_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.fcc_votable)) {
		rc = PTR_ERR(charge_votable_data.fcc_votable);
		cas_debug("create %s failed\n", "fcc_votable");
		goto destroy_votable;
	}

	charge_votable_data.fcv_votable = create_votable("FCV", VOTE_MIN,
					charger_fcv_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.fcv_votable)) {
		rc = PTR_ERR(charge_votable_data.fcv_votable);
		cas_debug("create %s failed\n", "fcv_votable");
		goto destroy_votable;
	}

	charge_votable_data.topoff_votable = create_votable("TOPOFF", VOTE_MAX,
					charger_topoff_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.topoff_votable)) {
		rc = PTR_ERR(charge_votable_data.topoff_votable);
		cas_debug("create %s failed\n", "topoff_votable");
		goto destroy_votable;
	}

	charge_votable_data.recharge_soc_votable = create_votable("RECH_SOC", VOTE_MIN,
					charger_recharge_soc_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.recharge_soc_votable)) {
		rc = PTR_ERR(charge_votable_data.recharge_soc_votable);
		cas_debug("create %s failed\n", "recharge_soc_votable");
		goto destroy_votable;
	}

	charge_votable_data.recharge_voltage_votable = create_votable("RECH_VOLTAGE", VOTE_MAX,
					charger_recharge_voltage_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.recharge_voltage_votable)) {
		rc = PTR_ERR(charge_votable_data.recharge_voltage_votable);
		cas_debug("create %s failed\n", "recharge_voltage_votable");
		goto destroy_votable;
	}

	charge_votable_data.usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					charger_usb_icl_vote_callback,
					&charge_votable_data);
	if (IS_ERR(charge_votable_data.usb_icl_votable)) {
		rc = PTR_ERR(charge_votable_data.usb_icl_votable);
		cas_debug("create %s failed\n", "usb_icl_votable");
		goto destroy_votable;
	}

	atomic_set(&charge_votable_data.init_finished, 1);
#endif

	return 0;

#ifdef CONFIG_CHARGER_PMIC_VOTER
destroy_votable:
	atomic_set(&charge_votable_data.init_finished, 0);
	destroy_votable(charge_votable_data.fcc_votable);
	destroy_votable(charge_votable_data.fcv_votable);
	destroy_votable(charge_votable_data.topoff_votable);
	destroy_votable(charge_votable_data.recharge_soc_votable);
	destroy_votable(charge_votable_data.recharge_voltage_votable);
	destroy_votable(charge_votable_data.usb_icl_votable);
register_power_supply_failed:

	return rc;
#endif
}

subsys_initcall(charger_class_init);
module_exit(charger_class_exit);

MODULE_DESCRIPTION("Switching Charger Class Device");
MODULE_AUTHOR("Patrick Chang <patrick_chang@richtek.com>");
MODULE_VERSION("1.0.0_G");
MODULE_LICENSE("GPL");
