/*
 * Copyright (C) 2015 MediaTek Inc.
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

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#endif

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define GPIO_LCD_ENP    432/*89+343*/
#define GPIO_LCD_ENN    433/*90+343*/

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_ (0xf5)

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;


#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT									(1560)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(69120)
#define LCM_PHYSICAL_HEIGHT									(149760)
#define LCM_DENSITY											(320)

#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 60, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 60, {} },
};

static struct LCM_setting_table init_setting[] = {
	{0xFF, 3, {0x98, 0x81, 0x01} },
	{0x00, 1, {0x46} },
	{0x01, 1, {0x16} },
	{0x02, 1, {0x00} },
	{0x03, 1, {0x00} },
	{0x08, 1, {0x80} },
	{0x09, 1, {0x12} },
	{0x0A, 1, {0x71} },
	{0x0B, 1, {0x00} },
	{0x0C, 1, {0x00} },
	{0x0D, 1, {0x00} },
	{0x0E, 1, {0x01} },
	{0x0F, 1, {0x01} },
	{0x10, 1, {0x01} },
	{0x11, 1, {0x01} },
	{0x12, 1, {0x01} },
	{0x24, 1, {0x01} },
	{0x25, 1, {0x0b} },
	{0x26, 1, {0x00} },
	{0x27, 1, {0x00} },
	{0x31, 1, {0x21} },
	{0x32, 1, {0x07} },
	{0x33, 1, {0x01} },
	{0x34, 1, {0x00} },
	{0x35, 1, {0x02} },
	{0x36, 1, {0x07} },
	{0x37, 1, {0x07} },
	{0x38, 1, {0x07} },
	{0x39, 1, {0x07} },
	{0x3A, 1, {0x17} },
	{0x3B, 1, {0x15} },
	{0x3C, 1, {0x07} },
	{0x3D, 1, {0x07} },
	{0x3E, 1, {0x13} },
	{0x3F, 1, {0x11} },
	{0x40, 1, {0x09} },
	{0x41, 1, {0x07} },
	{0x42, 1, {0x07} },
	{0x43, 1, {0x07} },
	{0x44, 1, {0x07} },
	{0x45, 1, {0x07} },
	{0x46, 1, {0x07} },
	{0x47, 1, {0x20} },
	{0x48, 1, {0x07} },
	{0x49, 1, {0x01} },
	{0x4A, 1, {0x00} },
	{0x4B, 1, {0x02} },
	{0x4C, 1, {0x07} },
	{0x4D, 1, {0x07} },
	{0x4E, 1, {0x07} },
	{0x4F, 1, {0x07} },
	{0x50, 1, {0x16} },
	{0x51, 1, {0x14} },
	{0x52, 1, {0x07} },
	{0x53, 1, {0x07} },
	{0x54, 1, {0x12} },
	{0x55, 1, {0x10} },
	{0x56, 1, {0x08} },
	{0x57, 1, {0x07} },
	{0x58, 1, {0x07} },
	{0x59, 1, {0x07} },
	{0x5A, 1, {0x07} },
	{0x5B, 1, {0x07} },
	{0x5C, 1, {0x07} },
	{0x61, 1, {0x08} },
	{0x62, 1, {0x07} },
	{0x63, 1, {0x01} },
	{0x64, 1, {0x00} },
	{0x65, 1, {0x02} },
	{0x66, 1, {0x07} },
	{0x67, 1, {0x07} },
	{0x68, 1, {0x07} },
	{0x69, 1, {0x07} },
	{0x6A, 1, {0x10} },
	{0x6B, 1, {0x12} },
	{0x6C, 1, {0x07} },
	{0x6D, 1, {0x07} },
	{0x6E, 1, {0x14} },
	{0x6F, 1, {0x16} },
	{0x70, 1, {0x20} },
	{0x71, 1, {0x07} },
	{0x72, 1, {0x07} },
	{0x73, 1, {0x07} },
	{0x74, 1, {0x07} },
	{0x75, 1, {0x07} },
	{0x76, 1, {0x07} },
	{0x77, 1, {0x09} },
	{0x78, 1, {0x07} },
	{0x79, 1, {0x01} },
	{0x7A, 1, {0x00} },
	{0x7B, 1, {0x02} },
	{0x7C, 1, {0x07} },
	{0x7D, 1, {0x07} },
	{0x7E, 1, {0x07} },
	{0x7F, 1, {0x07} },
	{0x80, 1, {0x11} },
	{0x81, 1, {0x13} },
	{0x82, 1, {0x07} },
	{0x83, 1, {0x07} },
	{0x84, 1, {0x15} },
	{0x85, 1, {0x17} },
	{0x86, 1, {0x21} },
	{0x87, 1, {0x07} },
	{0x88, 1, {0x07} },
	{0x89, 1, {0x07} },
	{0x8A, 1, {0x07} },
	{0x8B, 1, {0x07} },
	{0x8C, 1, {0x07} },
	{0xB9, 1, {0x40} },
	{0xCD, 1, {0x11} },
	{0xD0, 1, {0x01} },
	{0xD1, 1, {0x00} },
	{0xDC, 1, {0x35} },
	{0xDD, 1, {0x42} },
	{0xE2, 1, {0x00} },
	{0xE6, 1, {0x22} },
	{0xE7, 1, {0x54} },
	{0xFF, 3, {0x98, 0x81, 0x02} },
	{0x40, 1, {0x49} },
	{0x4B, 1, {0x5A} },
	{0x4D, 1, {0x4E} },
	{0x4E, 1, {0x00} },
	{0x1A, 1, {0x48} },
	{0xFF, 3, {0x98, 0x81, 0x05} },
	{0x03, 1, {0x00} },
	{0x04, 1, {0xCE} },
	{0x58, 1, {0x62} },
	{0x63, 1, {0x85} },
	{0x64, 1, {0x85} },
	{0x68, 1, {0xA1} },
	{0x69, 1, {0xA7} },
	{0x6A, 1, {0xA1} },
	{0x6B, 1, {0x93} },
	{0x19, 1, {0xB1} },
	{0xFF, 3, {0x98, 0x81, 0x06} },
	{0x2E, 1, {0x01} },
	{0xC0, 1, {0x0B} },
	{0xC1, 1, {0x03} },
	{0xFF, 3, {0x98, 0x81, 0x06} },
	{0x11, 1, {0x03} },
	{0x12, 1, {0x00} },
	{0x13, 1, {0x45} },
	{0x14, 1, {0x41} },
	{0x15, 1, {0xF1} },
	{0x16, 1, {0x40} },
	{0x17, 1, {0xFF} },
	{0x18, 1, {0x00} },
	{0xC2, 1, {0x04} },
	{0x27, 1, {0x20} },
	{0x28, 1, {0x20} },
	{0x48, 1, {0x0F} },
	{0x4D, 1, {0x80} },
	{0x4E, 1, {0x40} },
	{0x7F, 1, {0x78} },
	{0xD6, 1, {0x87} },
	{0xFF, 3, {0x98, 0x81, 0x08} },
	{0xE0, 27, {0x40, 0x24, 0x86, 0xBF, 0x01,
		0x55, 0x34, 0x5A, 0x86, 0xA9,
		0xA9, 0xDE, 0x07, 0x2C, 0x50,
		0xAA, 0x76, 0xA6, 0xC5, 0xED,
		0xFF, 0x10, 0x3C, 0x72, 0x9E,
		0x03, 0xEC} },
	{0xE1, 27, {0x40, 0x24, 0x86, 0xBF, 0x01,
		0x55, 0x34, 0x5A, 0x86, 0xA9,
		0xA9, 0xDE, 0x07, 0x2C, 0x50,
		0xAA, 0x76, 0xA6, 0xC5, 0xED,
		0xFF, 0x10, 0x3C, 0x72, 0x9E,
		0x03, 0xEC} },

	{0xFF, 3, {0x98, 0x81, 0x0E} },

	{0x00, 1, {0xA0} },
	{0x13, 1, {0x10} },

	{0xff, 3, {0x98, 0x81, 0x00} },
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
	{REGFLAG_DELAY, 40, {} },
	{0x35, 1, {0x00} },
	{0xff, 3, {0x98, 0x81, 0x0F} },
};

static struct LCM_setting_table bl_level[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density            = LCM_DENSITY;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	lcm_dsi_mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
	LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 40;
	params->dsi.vertical_frontporch = 255;
	params->dsi.vertical_frontporch_for_low_power = 540;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 35;
	params->dsi.horizontal_backporch = 65;
	params->dsi.horizontal_frontporch = 65;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 240;	/* this value must be in MTK suggested table */
#else
	params->dsi.PLL_CLOCK = 343;	/* this value must be in MTK suggested table */
#endif
	params->dsi.PLL_CK_CMD = 240;
	params->dsi.PLL_CK_VDO = 343;
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
	params->dsi.lcm_backlight_curve_mode = 350;
	params->dsi.lcm_vsp_vsn_voltage = 6000;
}

static void lcm_init_power(void)
{
	display_bias_enable();
}

extern unsigned int primary_display_get_shutdown_suspend_flag(void);
static void lcm_suspend_power(void)
{
	unsigned int is_shutdown_suspend = 0;

	is_shutdown_suspend = primary_display_get_shutdown_suspend_flag();

	if (is_shutdown_suspend == 1) {
		pr_info("%s: for shutdown\n", __func__);
		display_bias_disable();
		MDELAY(5);
		SET_RESET_PIN(0);

		return;
	}

	if (!suspend_tp_need_awake())
		display_bias_disable();
}

static void lcm_resume_power(void)
{
	SET_RESET_PIN(0);
	if (!suspend_tp_need_awake())
		display_bias_enable();
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(15);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	if (lcm_dsi_mode == CMD_MODE) {
		push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
		LCM_LOGI("lcm mode = cmd mode :%d----\n", lcm_dsi_mode);
	} else {
		push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
		LCM_LOGI("lcm mode = vdo mode :%d----\n", lcm_dsi_mode);
	}
}

static void lcm_suspend(void)
{
	LCM_LOGI("%s: kernel ----zte_durian_ili9881h_720p_6p49_skyworth_huajiacai_drv----\n", __func__);
	push_table(NULL, lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
}

static void lcm_resume(void)
{
	LCM_LOGI("%s: kernel----zte_durian_ili9881h_720p_6p49_skyworth_huajiacai_drv----\n", __func__);
	lcm_init();
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

static unsigned int lcm_compare_id(void)
{
	return 1;
}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x53, buffer, 1);

	if (buffer[0] != 0x24) {
		LCM_LOGI("[LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_LOGI("[LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif

}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00043700;	/* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x2A, read_buf, 4);

	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
	    && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	LCM_LOGI("%s,backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;

	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

static void *lcm_switch_mode(int mode)
{
#ifndef BUILD_LK
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
	if (mode == 0) {	/* V2C */
		lcm_switch_mode_cmd.mode = CMD_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;	/* mode control addr */
		lcm_switch_mode_cmd.val[0] = 0x13;	/* enabel GRAM firstly, ensure writing one frame to GRAM */
		lcm_switch_mode_cmd.val[1] = 0x10;	/* disable video mode secondly */
	} else {		/* C2V */
		lcm_switch_mode_cmd.mode = SYNC_PULSE_VDO_MODE;
		lcm_switch_mode_cmd.addr = 0xBB;
		lcm_switch_mode_cmd.val[0] = 0x03;	/* disable GRAM and enable video mode */
	}
	return (void *)(&lcm_switch_mode_cmd);
#else
	return NULL;
#endif
}


LCM_DRIVER zte_durian_ili9881h_720p_6p49_skyworth_huajiacai_lcm_drv = {
	.name = "zte_durian_ili9881h_720p_6p49_skyworth_huajiacai_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.esd_check = lcm_esd_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
	.switch_mode = lcm_switch_mode,
};
