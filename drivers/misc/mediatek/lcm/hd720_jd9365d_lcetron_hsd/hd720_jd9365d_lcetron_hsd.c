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

{0xE0,1,{0x00}},

{0xE1,1,{0x93}},
{0xE2,1,{0x65}},
{0xE3,1,{0xF8}},


{0xE0,1,{0x01}},

{0x00,1,{0x00}},
{0x01,1,{0xA0}},
{0x03,1,{0x00}},
{0x04,1,{0xAD}},

{0x17,1,{0x00}},
{0x18,1,{0xD7}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0xD7}},
{0x1C,1,{0x00}},

{0x1F,1,{0x6B}},
{0x20,1,{0x24}},
{0x21,1,{0x24}},
{0x22,1,{0x4E}},
{0x25,1,{0x20}},


{0x37,1,{0x09}},

{0x38,1,{0x04}},
{0x39,1,{0x08}},
{0x3A,1,{0x12}},
{0x3C,1,{0x6A}},
{0x3D,1,{0xFF}},
{0x3E,1,{0xFF}},
{0x3F,1,{0x6A}},

{0x40,1,{0x04}},
{0x41,1,{0xB4}},
{0x42,1,{0x70}},
{0x43,1,{0x12}},
{0x44,1,{0x0F}},
{0x45,1,{0x46}},


{0x55,1,{0x01}},
{0x56,1,{0x01}},
{0x57,1,{0x69}},
{0x58,1,{0x0A}},
{0x59,1,{0x0A}},
{0x5A,1,{0x29}},
{0x5B,1,{0x10}},

{0x5D,1,{0x6F}},
{0x5E,1,{0x58}},
{0x5F,1,{0x4A}},
{0x60,1,{0x3F}},
{0x61,1,{0x3D}},
{0x62,1,{0x30}},
{0x63,1,{0x37}},
{0x64,1,{0x23}},
{0x65,1,{0x3E}},
{0x66,1,{0x3E}},
{0x67,1,{0x3F}},
{0x68,1,{0x5D}},
{0x69,1,{0x4B}},
{0x6A,1,{0x52}},
{0x6B,1,{0x44}},
{0x6C,1,{0x3F}},
{0x6D,1,{0x32}},
{0x6E,1,{0x20}},
{0x6F,1,{0x10}},
{0x70,1,{0x6F}},
{0x71,1,{0x58}},
{0x72,1,{0x4A}},
{0x73,1,{0x3F}},
{0x74,1,{0x3D}},
{0x75,1,{0x30}},
{0x76,1,{0x37}},
{0x77,1,{0x23}},
{0x78,1,{0x3E}},
{0x79,1,{0x3E}},
{0x7A,1,{0x3F}},
{0x7B,1,{0x5D}},
{0x7C,1,{0x4B}},
{0x7D,1,{0x52}},
{0x7E,1,{0x44}},
{0x7F,1,{0x3F}},
{0x80,1,{0x32}},
{0x81,1,{0x20}},
{0x82,1,{0x10}},


{0xE0,1,{0x02}},

{0x00,1,{0x1E}},
{0x01,1,{0x1F}},
{0x02,1,{0x57}},
{0x03,1,{0x58}},
{0x04,1,{0x44}},
{0x05,1,{0x46}},
{0x06,1,{0x48}},
{0x07,1,{0x4A}},
{0x08,1,{0x40}},
{0x09,1,{0x1D}},
{0x0A,1,{0x1D}},
{0x0B,1,{0x1D}},
{0x0C,1,{0x1D}},
{0x0D,1,{0x1D}},
{0x0E,1,{0x1D}},
{0x0F,1,{0x50}},
{0x10,1,{0x1F}},
{0x11,1,{0x1F}},
{0x12,1,{0x1F}},
{0x13,1,{0x1F}},
{0x14,1,{0x1F}},
{0x15,1,{0x1F}},

{0x16,1,{0x1E}},
{0x17,1,{0x1F}},
{0x18,1,{0x57}},
{0x19,1,{0x58}},
{0x1A,1,{0x45}},
{0x1B,1,{0x47}},
{0x1C,1,{0x49}},
{0x1D,1,{0x4B}},
{0x1E,1,{0x41}},
{0x1F,1,{0x1D}},
{0x20,1,{0x1D}},
{0x21,1,{0x1D}},
{0x22,1,{0x1D}},
{0x23,1,{0x1D}},
{0x24,1,{0x1D}},
{0x25,1,{0x51}},
{0x26,1,{0x1F}},
{0x27,1,{0x1F}},
{0x28,1,{0x1F}},
{0x29,1,{0x1F}},
{0x2A,1,{0x1F}},
{0x2B,1,{0x1F}},

{0x2C,1,{0x1F}},
{0x2D,1,{0x1E}},
{0x2E,1,{0x17}},
{0x2F,1,{0x18}},
{0x30,1,{0x0B}},
{0x31,1,{0x09}},
{0x32,1,{0x07}},
{0x33,1,{0x05}},
{0x34,1,{0x11}},
{0x35,1,{0x1F}},
{0x36,1,{0x1F}},
{0x37,1,{0x1F}},
{0x38,1,{0x1F}},
{0x39,1,{0x1F}},
{0x3A,1,{0x1F}},
{0x3B,1,{0x01}},
{0x3C,1,{0x1F}},
{0x3D,1,{0x1F}},
{0x3E,1,{0x1F}},
{0x3F,1,{0x1F}},
{0x40,1,{0x1F}},
{0x41,1,{0x1F}},

{0x42,1,{0x1F}},
{0x43,1,{0x1E}},
{0x44,1,{0x17}},
{0x45,1,{0x18}},
{0x46,1,{0x0A}},
{0x47,1,{0x08}},
{0x48,1,{0x06}},
{0x49,1,{0x04}},
{0x4A,1,{0x10}},
{0x4B,1,{0x1F}},
{0x4C,1,{0x1F}},
{0x4D,1,{0x1F}},
{0x4E,1,{0x1F}},
{0x4F,1,{0x1F}},
{0x50,1,{0x1F}},
{0x51,1,{0x00}},
{0x52,1,{0x1F}},
{0x53,1,{0x1F}},
{0x54,1,{0x1F}},
{0x55,1,{0x1F}},
{0x56,1,{0x1F}},
{0x57,1,{0x1F}},

{0x58,1,{0x40}},
{0x59,1,{0x00}},
{0x5A,1,{0x00}},
{0x5B,1,{0x10}},
{0x5C,1,{0x0B}},
{0x5D,1,{0x30}},
{0x5E,1,{0x01}},
{0x5F,1,{0x02}},
{0x60,1,{0x30}},
{0x61,1,{0x03}},
{0x62,1,{0x04}},
{0x63,1,{0x1C}},
{0x64,1,{0x58}},
{0x65,1,{0x55}},
{0x66,1,{0xB0}},
{0x67,1,{0x73}},
{0x68,1,{0x0D}},
{0x69,1,{0x1C}},
{0x6A,1,{0x58}},
{0x6B,1,{0x00}},
{0x6C,1,{0x00}},
{0x6D,1,{0x00}},
{0x6E,1,{0x00}},
{0x6F,1,{0x08}},
{0x70,1,{0x00}},
{0x71,1,{0x00}},
{0x72,1,{0x06}},
{0x73,1,{0x7B}},
{0x74,1,{0x00}},
{0x75,1,{0xBC}},
{0x76,1,{0x00}},
{0x77,1,{0x0D}},
{0x78,1,{0xC1}},
{0x79,1,{0x00}},
{0x7A,1,{0x00}},
{0x7B,1,{0x00}},
{0x7C,1,{0x00}},
{0x7D,1,{0x03}},
{0x7E,1,{0x7B}},


{0xE0,1,{0x04}},
{0x09,1,{0x10}},
{0x0E,1,{0x4A}},



{0xE0,1,{0x00}},

{0x11,1,{0x00}},
{REGFLAG_DELAY,120,{}},

{0x29,1,{0x00}},
{REGFLAG_DELAY,5,{}},

{0x35,1,{0x00}},







{REGFLAG_END_OF_TABLE,0x00,{}}
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

        params->dsi.vertical_sync_active                                = 4;   //3// 
        params->dsi.vertical_backporch                                        = 12; 
        params->dsi.vertical_frontporch                                        = 18;   //2// 
        params->dsi.vertical_active_line                                = FRAME_HEIGHT; 

        params->dsi.horizontal_sync_active                                = 30; 
        params->dsi.horizontal_backporch                                = 30;  //85
        params->dsi.horizontal_frontporch                                = 40; //65
        params->dsi.horizontal_active_pixel                                = FRAME_WIDTH; 
	params->dsi.PLL_CLOCK				= 226;
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
	
	SET_RESET_PIN(0);
	MDELAY(10);
	
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



LCM_DRIVER hd720_jd9365d_lcetron_hsd_drv =
{
    .name			= "hd720_jd9365d_lcetron_hsd",
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


