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

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0XFD
#define REGFLAG_END_OF_TABLE      							0xFE   // END OF REGISTERS MARKER



// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

//<2015-12-10-Lever.ni, [A110][DRV]modify mipi timing
#define NS_TO_CYCLE(n, c)	((n) / (c))
//>2015-12-10-Lever.ni

       

//#define LCM_DSI_CMD_MODE

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#if 1 
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,3,{0x98,0x81,0x03}},
{0x01,1,{0x00}},
{0x02,1,{0x00}},
{0x03,1,{0x53}},
{0x04,1,{0x13}},
{0x05,1,{0x00}},
{0x06,1,{0x04}},
{0x07,1,{0x02}},
{0x08,1,{0x00}},
{0x09,1,{0x28}},
{0x0a,1,{0x28}},
{0x0b,1,{0x00}},
{0x0c,1,{0x01}},
{0x0d,1,{0x00}},
{0x0e,1,{0x00}},
{0x0f,1,{0x16}},
{0x10,1,{0x16}},
{0x11,1,{0x00}},
{0x12,1,{0x00}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0x00}},
{0x16,1,{0x00}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1a,1,{0x00}},
{0x1b,1,{0x00}},
{0x1c,1,{0x00}},
{0x1d,1,{0x00}},
{0x1e,1,{0x44}},
{0x1f,1,{0x80}},
{0x20,1,{0x02}},
{0x21,1,{0x03}},
{0x22,1,{0x00}},
{0x23,1,{0x00}},
{0x24,1,{0x00}},
{0x25,1,{0x00}},
{0x26,1,{0x00}},
{0x27,1,{0x00}},
{0x28,1,{0x33}},
{0x29,1,{0x03}},
{0x2a,1,{0x00}},
{0x2b,1,{0x00}},
{0x2c,1,{0x00}},
{0x2d,1,{0x00}},
{0x2e,1,{0x00}},
{0x2f,1,{0x00}},
{0x30,1,{0x00}},
{0x31,1,{0x00}},
{0x32,1,{0x00}},
{0x33,1,{0x00}},
{0x34,1,{0x04}},
{0x35,1,{0x00}},
{0x36,1,{0x00}},
{0x37,1,{0x00}},
{0x38,1,{0x3c}},
{0x39,1,{0x00}},
{0x3a,1,{0x40}},
{0x3b,1,{0x40}},
{0x3c,1,{0x00}},
{0x3d,1,{0x00}},
{0x3e,1,{0x00}},
{0x3f,1,{0x00}},
{0x40,1,{0x00}},
{0x41,1,{0x00}},
{0x42,1,{0x00}},
{0x43,1,{0x00}},
{0x44,1,{0x00}},
{0x50,1,{0x01}},
{0x51,1,{0x23}},
{0x52,1,{0x45}},
{0x53,1,{0x67}},
{0x54,1,{0x89}},
{0x55,1,{0xab}},
{0x56,1,{0x01}},
{0x57,1,{0x23}},
{0x58,1,{0x45}},
{0x59,1,{0x67}},
{0x5a,1,{0x89}},
{0x5b,1,{0xab}},
{0x5c,1,{0xcd}},
{0x5d,1,{0xef}},

{0x5e,1,{0x11}},
{0x5f,1,{0x01}},
{0x60,1,{0x00}},
{0x61,1,{0x15}}, 
{0x62,1,{0x14}},
{0x63,1,{0x0c}},
{0x64,1,{0x0d}},
{0x65,1,{0x0e}},
{0x66,1,{0x0f}},
{0x67,1,{0x06}},
{0x68,1,{0x02}},
{0x69,1,{0x02}},
{0x6a,1,{0x02}},
{0x6b,1,{0x02}},
{0x6c,1,{0x02}},
{0x6d,1,{0x02}},
{0x6e,1,{0x08}},
{0x6f,1,{0x02}},
{0x70,1,{0x02}},
{0x71,1,{0x02}},
{0x72,1,{0x02}},
{0x73,1,{0x02}},
{0x74,1,{0x02}},
{0x75,1,{0x01}},
{0x76,1,{0x00}},
{0x77,1,{0x15}},
{0x78,1,{0x14}},
{0x79,1,{0x0c}},
{0x7a,1,{0x0d}},
{0x7b,1,{0x0e}},
{0x7c,1,{0x0f}},
{0x7d,1,{0x08}},
{0x7e,1,{0x02}},
{0x7f,1,{0x02}},
{0x80,1,{0x02}},
{0x81,1,{0x02}},
{0x82,1,{0x02}},
{0x83,1,{0x02}},
{0x84,1,{0x06}},
{0x85,1,{0x02}},
{0x86,1,{0x02}},
{0x87,1,{0x02}},
{0x88,1,{0x02}},
{0x89,1,{0x02}},
{0x8a,1,{0x02}},


{0xff,3,{0x98,0x81,0x04}},
{0x6c,1,{0x15}},
{0x6e,1,{0x2b}},  
{0x6f,1,{0x35}},
{0x35,1,{0x1f}},
{0x38,1,{0x01}},
{0x39,1,{0x00}},
{0x3a,1,{0x24}},

{0x8d,1,{0x1a}},  
{0x87,1,{0xba}},
{0x26,1,{0x76}},
{0xb2,1,{0xd1}},
{0xb5,1,{0x06}},  
{0x33,1,{0x14}},

{0x4c,1,{0x02}},
{0x52,1,{0x37}},
{0x53,1,{0x37}},
{0x54,1,{0x46}},


{0xff,3,{0x98,0x81,0x01}},
{0x22,1,{0x0a}}, 
{0x31,1,{0x00}},
{0x53,1,{0x8f}},    
{0x55,1,{0x9c}},
{0x50,1,{0xc7}}, 
{0x51,1,{0xc4}},
{0x60,1,{0x1a}},
{0x62,1,{0x00}},
{0x63,1,{0x00}},
{0x2e,1,{0xf0}},
{0xA0,1,{0x00}}, 
{0xA1,1,{0x1f}}, 
{0xA2,1,{0x2d}}, 
{0xA3,1,{0x1f}}, 
{0xA4,1,{0x18}}, 
{0xA5,1,{0x2b}}, 
{0xA6,1,{0x1f}}, 
{0xA7,1,{0x20}}, 
{0xA8,1,{0x8b}}, 
{0xA9,1,{0x1c}}, 
{0xAA,1,{0x28}}, 
{0xAB,1,{0x76}}, 
{0xAC,1,{0x1b}}, 
{0xAD,1,{0x19}}, 
{0xAE,1,{0x4e}}, 
{0xAF,1,{0x22}}, 
{0xb0,1,{0x28}}, 
{0xb1,1,{0x50}}, 
{0xb2,1,{0x60}}, 
{0xb3,1,{0x2f}}, 
{0xC0,1,{0x00}}, 
{0xC1,1,{0x1f}}, 
{0xC2,1,{0x2d}}, 
{0xC3,1,{0x15}}, 
{0xC4,1,{0x18}}, 
{0xC5,1,{0x2b}}, 
{0xC6,1,{0x1f}}, 
{0xC7,1,{0x20}}, 
{0xC8,1,{0x8b}}, 
{0xC9,1,{0x1c}}, 
{0xCA,1,{0x28}}, 
{0xCB,1,{0x76}}, 
{0xCC,1,{0x1b}}, 
{0xCD,1,{0x19}}, 
{0xCE,1,{0x4e}}, 
{0xCF,1,{0x22}},
{0xd0,1,{0x28}}, 
{0xd1,1,{0x50}}, 
{0xd2,1,{0x60}}, 
{0xd3,1,{0x2f}},


{0xff,3,{0x98,0x81,0x00}},  
{0x35,1,{0x00}},
{0x11,1,{0x00 }}, 
{REGFLAG_DELAY, 60, {}}, 
{0x29,1,{0x00}}, 
{REGFLAG_DELAY, 20, {}},                                 



	// Setting ending by predefined flag
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif 
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

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

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if defined(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573

	//params->dsi.DSI_WMEM_CONTI=0x3C; 
	//params->dsi.DSI_RMEM_CONTI=0x3E; 

		
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

        params->dsi.vertical_sync_active                                = 8;   //3// 
        params->dsi.vertical_backporch                                        = 24; 
        params->dsi.vertical_frontporch                                        = 16;   //2// 
        params->dsi.vertical_active_line                                = FRAME_HEIGHT; 

        params->dsi.horizontal_sync_active                                = 8; 
        params->dsi.horizontal_backporch                                = 80;  //85
        params->dsi.horizontal_frontporch                                = 80; //65
        params->dsi.horizontal_active_pixel                                = FRAME_WIDTH; 
	params->dsi.PLL_CLOCK				= 248;
	params->dsi.ssc_disable = 1;

	params->physical_width = 62;
	params->physical_height = 124;

	params->dsi.HS_TRAIL = 0x5;


	params->dsi.lcm_backlight_curve_mode = 350;
	
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->corner_pattern_width = 24;
	params->corner_pattern_height = 24;
#endif

}


/*
#define GPIO_PCD_ID0 GPIO58
#define GPIO_PCD_ID1 GPIO77
#define GPIO_LCM_PWR_P_EN GPIO60
#define GPIO_LCM_PWR_N_EN GPIO123
*/

static unsigned int lcm_compare_id(void)
{

		return 1;

}


static void lcm_init(void)
{

	gpio_request(GPIO_LCD_ENP ,"lcd-enp");
       gpio_set_value(GPIO_LCD_ENP, 1);

	MDELAY(5);
	gpio_request(GPIO_LCD_ENN ,"lcd-enn");
       gpio_set_value(GPIO_LCD_ENN, 1);
	MDELAY(8);

	SET_RESET_PIN(1);
    	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//strcpy(lcm_vendor,"DJ-HSD");

}



static void lcm_suspend(void)
{

	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	
	MDELAY(120);
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
}



LCM_DRIVER hd720_ili9881c_skyworth_hsd_drv = 
{
    .name			= "hd720_ili9881c_skyworth_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.compare_id     = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if defined(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };


