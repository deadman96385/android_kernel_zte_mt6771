/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* The following software/firmware and/or related documentation ("MediaTek Software")
* have been modified by MediaTek Inc. All revisions are subject to any receiver\'s
* applicable license agreements with MediaTek Inc.
*/

#include "lcm_drv.h"
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
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1440)

#define GPIO_LCD_ENN 	464//121+343
#define GPIO_LCD_ENP 	465//122+343

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0



static LCM_setting_table_V3 lcm_initialization_setting[] = {
	


{0x39, 0xB9,  3 ,{0xFF, 0x83, 0x94}},

{0x39, 0xBA,  6 ,{0x63, 0x03, 0x68, 0x6B, 
				0xB2, 0xC0}},

{0x39, 0xB1,  10 ,{0x48, 0x07, 0x67, 0x09, 
				0x31, 0x54, 0x71, 0x31, 
				0x50, 0x34}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,10,{}},

{0x39, 0xB2,  6 ,{0x00,0x80, 0x78, 0x0E, 
				0x08, 0x22}},


{0x39, 0xB4,  20 ,{0x01, 0x72, 0x29, 0x72, 
				0x29, 0x72, 0x01, 0x0C, 
				0x7C, 0x33, 0x00, 0x3F, 
				0x01, 0x72, 0x29, 0x72, 
				0x29, 0x72, 0x01, 0x0C}},


{0x39, 0xE0,  58 ,{0x00, 0x03, 0x0B, 0x10, 
				0x13, 0x16, 0x1C, 0x18, 
				0x34, 0x46, 0x59, 0x5C, 
				0x6B, 0x82, 0x8A, 0x91, 
				0x9F, 0xA3, 0xA0, 0xAD, 
				0xBD, 0x5D, 0x5C, 0x60, 
				0x64, 0x67, 0x6F, 0x7E, 
				0x7F, 0x00, 0x03, 0x0B, 
				0x10, 0x13, 0x16, 0x1C,
				0x18, 0x34, 0x46, 0x59,
				0x5C, 0x6B, 0x82, 0x8A, 
				0x91,  0x9F, 0xA3, 0xA0, 
				0xAD, 0xBD, 0x5D, 0x5C, 
				0x60, 0x64, 0x67, 0x6F, 
				0x7E, 0x7F}},


{0x39, 0xD3,  33 ,{0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x17, 0x17, 
				0x32, 0x10, 0x04, 0x00, 
				0x04, 0x52, 0x15, 0xB6, 
				0x05, 0xB6, 0x32, 0x10, 
				0x00, 0x00, 0x00, 0x57, 
				0x45, 0x05, 0x05, 0x5B, 
				0x0E, 0x0E, 0x27, 0x0E, 
				0x40}},

{0x15, 0xBD,  1 ,{0x02}},

{0x39, 0xD3,  4 ,{0x01, 0x04,0x00, 0x01}},

{0x15, 0xBD,  1 ,{0x00}},

{0x39, 0xD5,  44 ,{0x24, 0x25, 0x18, 0x18, 
				0x19, 0x19, 0x00, 0x01, 
				0x06, 0x07, 0x02, 0x03, 
				0x08, 0x09, 0x04, 0x05, 
				0x0A, 0x0B, 0x18, 0x18, 
				0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x20, 0x21, 
				0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18}},


{0x39, 0xD6,  44 ,{0x21, 0x20, 0x19, 0x19, 
				0x18, 0x18, 0x0B, 0x0A, 
				0x05, 0x04, 0x09, 0x08, 
				0x03, 0x02, 0x07, 0x06, 
				0x01, 0x00, 0x18, 0x18, 
				0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x18, 0x18, 
				0x18, 0x18, 0x25, 0x24, 
				0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18}},

{0x39, 0xB6,  2 ,{0x4B, 0x4B}},


{0x15, 0xCC,  1 ,{0x0B}},//0x03

{0x39, 0xC0,  2 ,{0x1F, 0x31}},

{0x15, 0xD4,  1 ,{0x02}},

{0x15, 0xBD,  1 ,{0x01}},

{0x15, 0xB1,  1 ,{0x00}},

{0x15, 0xBD,  1 ,{0x00}},

{0x15, 0xC6,  1 ,{0xED}},

//{0x15, 0x36,  1 ,{0x02}},//revers

{0x05, 0x11,0,{}},//
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},

{0x05, 0x29,0,{}},//
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},	  



};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 0x04;// 3    2
		params->dsi.vertical_backporch					= 0x20;// 20   1
		params->dsi.vertical_frontporch					= 0x50; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 0x6;// 50  2
		params->dsi.horizontal_backporch				= 0x20;
		params->dsi.horizontal_frontporch				= 0x20;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		params->dsi.ssc_disable = 1;
		params->dsi.PLL_CLOCK = 240;

		params->physical_width = 62;
		params->physical_height = 124;

		params->dsi.HS_TRAIL = 0x5;

		params->dsi.lcm_backlight_curve_mode = 350;
		
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
		params->corner_pattern_width = 24;
		params->corner_pattern_height = 24;
#endif

}

static void lcm_init(void)
{

	gpio_request(GPIO_LCD_ENP ,"lcd-enp");
       gpio_set_value(GPIO_LCD_ENP, 1);

	MDELAY(5);
	gpio_request(GPIO_LCD_ENN ,"lcd-enn");
       gpio_set_value(GPIO_LCD_ENN, 1);
	MDELAY(8);
	
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
    	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(20);

	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);

}



static void lcm_suspend(void)
{

	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	gpio_request(GPIO_LCD_ENN ,"lcd-enn");
       gpio_set_value(GPIO_LCD_ENN, 0);
	MDELAY(5);
	gpio_request(GPIO_LCD_ENP ,"lcd-enp");
       gpio_set_value(GPIO_LCD_ENP, 0);
	MDELAY(5);

}


static void lcm_resume(void)
{

	lcm_init();

    #ifdef BUILD_LK
	  printf("[LK]-----hx8394f_hd720_lead_cpt----%s------\n",__func__);
    #else
	  printk("[KERNEL]-----hx8394f_hd720_lead_cpt----%s------\n",__func__);
    #endif	
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	return 1;

}



LCM_DRIVER hd720_hx8394f_lead_cpt_drv = 
{
    .name			= "hd720_hx8394f_lead_cpt",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
