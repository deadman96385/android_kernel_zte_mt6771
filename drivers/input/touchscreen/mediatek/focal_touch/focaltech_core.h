/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, Focaltech Ltd. All rights reserved.
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
/*******************************************************************************
*
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <asm/unistd.h>
//#include <mach/irqs.h>
#include <linux/jiffies.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/mount.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <../fs/proc/internal.h>
#include "focaltech_config.h"
#include "focaltech_i2c.h"
#include "tpd_sys.h"
#include "tpd.h"
#include "firmware_config.h"
#include <linux/wakelock.h>

#define FTS_DRIVER_VERSION                      "Focaltech V1.0 20160812"


#define FTS_MAX_ID                              0x0F
#define FTS_TOUCH_STEP                          6
#define FTS_FACE_DETECT_POS                     1
#define FTS_TOUCH_X_H_POS                       3
#define FTS_TOUCH_X_L_POS                       4
#define FTS_TOUCH_Y_H_POS                       5
#define FTS_TOUCH_Y_L_POS                       6
#define FTS_TOUCH_EVENT_POS                     3
#define FTS_TOUCH_ID_POS                        5
#define FT_TOUCH_POINT_NUM                      2
#define FTS_TOUCH_XY_POS                        7
#define FTS_TOUCH_MISC                          8
#define POINT_READ_BUF                          (3 + FTS_TOUCH_STEP * FTS_MAX_POINTS)
#define FT_FW_NAME_MAX_LEN                      50
#define TPD_DELAY                               (2*HZ/100)


/*register address*/
#define FTS_REG_CHIP_ID                         0xA3            // chip ID 
#define FTS_REG_FW_VER                          0xA6            // FW  version 
#define FTS_REG_VENDOR_ID                       0xA8            // TP vendor ID 
#define FTS_REG_POINT_RATE                      0x88            // report rate  
#define FT_GESTRUE_MODE_SWITCH_REG              0xD0
#define FT_GESTRUE_GETID_REG                    0xD3



/* pramboot */
#define FTS_PRAMBOOT_8716   "include/pramboot/FT8716_Pramboot_V0.5_20160723.h"
#define FTS_PRAMBOOT_8607   "include/pramboot/FT8607_Pramboot_V0.3_20160727.h"

/* ic types */
#if (FTS_CHIP_TYPE == _FT8716)
#define FTS_CHIP_ID             0x87
#define FTS_UPGRADE_PRAMBOOT    FTS_PRAMBOOT_8716
#elif (FTS_CHIP_TYPE == _FT8607)
#define FTS_CHIP_ID             0x86
#define FTS_UPGRADE_PRAMBOOT    FTS_PRAMBOOT_8607
#else
#error "unsupported chip type!"
#endif



/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
/* IC info */
struct fts_Upgrade_Info
{
    u8 CHIP_ID;
    u8 TPD_MAX_POINTS;
    u8 AUTO_CLB;
    u16 delay_aa;          /* delay of write FT_UPGRADE_AA */
    u16 delay_55;          /* delay of write FT_UPGRADE_55 */
    u8 upgrade_id_1;       /* upgrade id 1 */
    u8 upgrade_id_2;       /* upgrade id 2 */
    u16 delay_readid;      /* delay of read id */
    u16 delay_erase_flash;     /* delay of earse flash */
};

struct touch_info
{
    int y[FTS_MAX_POINTS];
    int x[FTS_MAX_POINTS];
    int p[FTS_MAX_POINTS];
    int id[FTS_MAX_POINTS];
    int count;
};

/*touch event info*/
struct ts_event
{
    u16 au16_x[FTS_MAX_POINTS];               /* x coordinate */
    u16 au16_y[FTS_MAX_POINTS];               /* y coordinate */
    u8 au8_touch_event[FTS_MAX_POINTS];       /* touch event: 0 -- down; 1-- up; 2 -- contact */
    u8 au8_finger_id[FTS_MAX_POINTS];         /* touch ID */
    u16 pressure[FTS_MAX_POINTS];
    u16 area[FTS_MAX_POINTS];
    u8 touch_point;
    int touchs;
    u8 touch_point_num;
};
struct fts_ts_data
{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct ts_event event;
    const struct ftxxxx_ts_platform_data *pdata;
    struct work_struct  touch_event_work;
    struct workqueue_struct *ts_workqueue;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    char fw_name[FT_FW_NAME_MAX_LEN];
    bool loading_fw;
    u8 family_id;
    struct dentry *dir;
    u16 addr;
    bool suspended;
    char *ts_info;
    u8 *tch_data;
    u32 tch_data_len;
    u8 fw_ver[3];
    u8 fw_vendor_id;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
};

struct focal_chip_data {
	struct i2c_client *i2c_client;
	bool enable_wakeup_gesture;
	bool tpd_suspend; 
	unsigned int tp_resx;
	unsigned int tp_resy;
	unsigned int lcd_resx;
	unsigned int lcd_resy;
	bool report_button;
	int tpd_fwversion;
	int tpd_vendorid;
	struct wake_lock tpd_bsg_wake_lock;
	bool need_stay_awake;
	
	struct regulator *avdd_regulator;
	struct regulator *iovdd_regulator;
	unsigned int touch_irq;
	unsigned int tpd_rst_gpio_number;
	unsigned int tpd_int_gpio_number;
};

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

extern struct input_dev *fts_input_dev;
extern struct tpd_device *tpd;
extern unsigned int tpd_rst_gpio_number;
extern struct fts_Upgrade_Info fts_updateinfo_curr;
extern unsigned int ft_touch_irq;
extern struct i2c_client *fts_i2c_client;

#if FTS_AUTO_UPGRADE_EN
extern struct workqueue_struct *touch_wq;
extern struct work_struct fw_update_work;
#endif


//gestrue
#if FTS_GESTURE_EN
extern int fts_gesture_init(struct input_dev *input_dev);
extern int fts_gesture_readdata(void);
int fts_gesture_suspend(struct i2c_client *i2c_client);
#endif

//IIC
extern int fts_rw_iic_drv_init(struct i2c_client *client);
extern void  fts_rw_iic_drv_exit(void);

//sys
extern int fts_remove_sysfs(struct i2c_client *client);
extern int fts_create_sysfs(struct i2c_client *client);

//upgrade
extern int tpd_auto_upgrade(struct i2c_client *client);
extern void fts_get_upgrade_array(void);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);

int fts_workqueue_init(void);
#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
extern int fts_ctpm_auto_upgrade_test(struct i2c_client *client);
#endif

//Apk debug
#if FTS_CTL_IIC_NODE_EN
extern int ft_rw_iic_drv_init(struct i2c_client *client);
extern void  ft_rw_iic_drv_exit(void);
#endif

extern int fts_create_apk_debug_channel(struct i2c_client * client);
extern void fts_release_apk_debug_channel(void);
extern char* mtkfb_find_lcm_driver(void);


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
void fts_sensor_suspend(struct i2c_client *i2c_client);
int fts_sensor_init(void);
void fts_sensor_enable(struct i2c_client *client);
ssize_t show_scp_ctrl(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t store_scp_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#endif

int fts_reset_proc(void);
#if FTS_ESDCHECK_EN
extern int fts_esdcheck_init(void);
extern int fts_esdcheck_exit(void);
extern int fts_esdcheck_switch(bool enable);
extern int fts_esdcheck_proc_busy(bool proc_debug);
extern int fts_esdcheck_set_intr(bool intr);
extern int fts_esdcheck_suspend(bool gesture_en);
extern int fts_esdcheck_resume(void);

#define FTS_REG_FLOW_WORK_CNT                   0x91
#define FTS_REG_WORKMODE                        0x00
#define FTS_FACTORYMODE_VALUE                   0x40
#define FTS_WORKMODE_VALUE                      0x00
#define ENABLE                                  1
#define DISABLE                                 0
#endif

#if FTS_TEST_EN
int fts_test_init(struct i2c_client *client);
int fts_test_exit(struct i2c_client *client);
#endif

//others
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down);
typedef void (*GES_CBFUNC)(u8);
/*****************************************************************************
 * ENUM
 ****************************************************************************/
enum GTP_WORK_STATE
{
    GTP_UNKNOWN = 0,
    GTP_NORMAL,
    GTP_DOZE,
    GTP_SLEEP,
};

enum TOUCH_DOZE_T1
{
    DOZE_INPOCKET = 0,
    DOZE_NOT_INPOCKET = 1,
};

enum TOUCH_DOZE_T2
{
    DOZE_DISABLE = 0,
    DOZE_ENABLE = 1,
};

enum TOUCH_WAKE_T
{
    TOUCH_WAKE_BY_NONE,
    TOUCH_WAKE_BY_INT,
    TOUCH_WAKE_BY_IPI,
    TOUCH_WAKE_BY_SWITCH
};

/*****************************************************************************
 * STRUCTURE
 ****************************************************************************/
struct Touch_SmartWake_ID
{
    u8 id;
    GES_CBFUNC cb;
};


#if FTS_DEBUG_EN
#define FTS_DEBUG(fmt, args...) printk(KERN_ERR "[FTS]"fmt"\n", ##args)
#define FTS_FUNC_ENTER() printk(KERN_ERR "[FTS]%s: Enter\n", __func__)
#define FTS_FUNC_EXIT()  printk(KERN_ERR "[FTS]%s: Exit(%d)\n", __func__, __LINE__)
#else
#define FTS_DEBUG(fmt, args...)
#define FTS_FUNC_ENTER() printk("[FTS]%s: Enter\n", __func__)
#define FTS_FUNC_EXIT()
#endif

#define FTS_INFO(fmt, args...) printk( "[FTS][Info]"fmt"\n", ##args)
#define FTS_ERROR(fmt, args...) printk(KERN_ERR "[FTS][Error]"fmt"\n", ##args)


#endif /* __LINUX_FOCALTECH_CORE_H__ */


