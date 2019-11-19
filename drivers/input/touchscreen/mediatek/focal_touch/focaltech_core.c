/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
/*******************************************************************************
*
* File Name: focaltech_core.c

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*******************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

#include "focaltech_core.h"
//#include "base.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/input/mt.h>

#include <linux/wakelock.h>



struct task_struct *thread_tpd;
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
struct i2c_client *fts_i2c_client;
struct input_dev *fts_input_dev;
struct wake_lock ps_lock;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static int tpd_flag;

#if FTS_CHARGE_DETECT_EN
u8 b_usb_plugin = 0;
int ctp_is_probe = 0;
#endif

unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int ft_touch_irq = 0;

static DEFINE_MUTEX(i2c_rw_access);

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local_normal[8]  = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

static const struct i2c_device_id focal_tpd_id[] = {{"focaltech", 0}, {} };
static const struct of_device_id focal_dt_match[] = {
    {.compatible = "mediatek,cap_touch"},
    {},
};
MODULE_DEVICE_TABLE(of, ft5x0x_dt_match);

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .of_match_table = of_match_ptr(focal_dt_match),
        .name = "focal_touch",
    },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = focal_tpd_id,
    .detect = tpd_i2c_detect,
};

#ifdef CONFIG_CREATE_SYS_INTERFACE
#ifdef CONFIG_PROC_FS
#define TOUCH_PROC_FILE "driver/tpd_touch"
static struct proc_dir_entry *tpd_debug_proc_entry;

static ssize_t proc_read_val(struct file *file,
    char __user *buffer, size_t count, loff_t *offset)
{
    ssize_t len = 0;
    char data[800];
    //unsigned char uc_reg_value = 0;
    struct focal_chip_data *ts_data = i2c_get_clientdata(fts_i2c_client);
    //int rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval, rst_gpio_mode;
    //int int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval, int_gpio_mode;

    len += sprintf(data + len, "Focal Touchscreen 20161018\n");
    len += sprintf(data + len, "tpd fw version:0x%x manufacturer:0x%x.\n", ts_data->tpd_fwversion, ts_data->tpd_vendorid);
    len += sprintf(data + len, "tpd is suspend:%d , need stay awake:%d , bsg enable? :%d .\n", ts_data->tpd_suspend, ts_data->need_stay_awake, 
        ts_data->enable_wakeup_gesture);
    len += sprintf(data + len, "tp (%u, %u) lcd(%u, %u) report button (%d).\n", ts_data->tp_resx,  ts_data->tp_resy, 
        ts_data->lcd_resx, ts_data->lcd_resy, ts_data->report_button);
    len += sprintf(data + len, "tp (rst pin:%u, int pin:%u) (int irq:%u)", ts_data->tpd_rst_gpio_number, ts_data->tpd_int_gpio_number, 
        ts_data->touch_irq);
    /*
    rst_gpio_dir = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
    rst_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_RST_PIN);
    rst_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_RST_PIN);
    rst_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_RST_PIN);
    rst_gpio_outval = mt_get_gpio_out(GPIO_CTP_RST_PIN);
    rst_gpio_inval = mt_get_gpio_in(GPIO_CTP_RST_PIN);
    rst_gpio_mode = mt_get_gpio_dir(GPIO_CTP_RST_PIN);
    
    int_gpio_dir = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
    int_gpio_pullen = mt_get_gpio_pull_enable(GPIO_CTP_EINT_PIN);
    int_gpio_pullsel = mt_get_gpio_pull_select(GPIO_CTP_EINT_PIN);
    int_gpio_inver = mt_get_gpio_inversion(GPIO_CTP_EINT_PIN);
    int_gpio_outval = mt_get_gpio_out(GPIO_CTP_EINT_PIN);
    int_gpio_inval = mt_get_gpio_in(GPIO_CTP_EINT_PIN);
    int_gpio_mode = mt_get_gpio_dir(GPIO_CTP_EINT_PIN);
    
    len += sprintf(data + len, "rst pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
        GPIO_CTP_RST_PIN, rst_gpio_mode, rst_gpio_dir, rst_gpio_pullen, rst_gpio_pullsel, rst_gpio_inver, rst_gpio_outval, rst_gpio_inval);
    len += sprintf(data + len, "int pin 0x%x, mode:%d, dir:%d, pullen:%d, sel:%d, inver:%d, outval:%d, in:%d.\n",
        GPIO_CTP_EINT_PIN, int_gpio_mode, int_gpio_dir, int_gpio_pullen, int_gpio_pullsel, int_gpio_inver, int_gpio_outval, int_gpio_inval);
    */
    //fts_i2c_read_reg(0xa5, &uc_reg_value);
    //len += sprintf(data + len, "Ft5x06 Touchscreen, workmode:%d.\n", uc_reg_value);
    
    return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t proc_write_val(struct file *filp,
                     const char *buff, size_t len,
                     loff_t * off)
{
    return len;
}

static struct file_operations tpd_touch_proc_ops = {
    .read = proc_read_val,
    .write = proc_write_val,
};

static void create_tpd_debug_proc_entry(void *data)
{
    tpd_debug_proc_entry = proc_create(TOUCH_PROC_FILE, 0644, NULL, &tpd_touch_proc_ops);
    if (tpd_debug_proc_entry) {
        //tpd_debug_proc_entry->data = (void*)data;
        //g_data = data;
        printk(KERN_INFO "create proc file sucess!\n");
    } else
        printk(KERN_INFO "create proc file failed!\n");
}
#endif
extern int tpd_fts_ctpm_firmware_upgrade(struct tpd_classdev_t *cdev, unsigned char * pbt_buf, unsigned int size, unsigned int force_upg);
extern int tpd_get_chipfw_version_etc(struct i2c_client *i2c_client, int * chip_num_in_chip, 
    int *fwver_in_chip, int *module_id_in_chip);

static int tpd_require_stay_awake(struct tpd_classdev_t *cdev)
{
    struct focal_chip_data *focal_data = (struct focal_chip_data*) cdev->private;

    focal_data->need_stay_awake = true;
    msleep(50);
    if(focal_data->tpd_suspend) {
        printk("tpd %s ts in suspend mode, wait 5 sec \n", __func__);
        msleep(3000);
        if(focal_data->tpd_suspend) {
            printk("tpd %s ts still in suspend mode, resume. \n", __func__);
            focal_data->need_stay_awake = false;
            tpd_resume(NULL);
        }
        focal_data->need_stay_awake = true;
    }
    return 0;
}

static int tpd_release_stay_awake(struct tpd_classdev_t *cdev)
{
    struct focal_chip_data *focal_data = (struct focal_chip_data*) cdev->private;

    focal_data->need_stay_awake = false;
    return 0;
}

static int tpd_init_tpinfo(struct tpd_classdev_t *cdev)
{
    int fwver_in_chip = 0, module_id_in_chip = 0, chip_num_in_chip = 0;
    struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;

    tpd_require_stay_awake(cdev);

    tpd_get_chipfw_version_etc(data->i2c_client, &chip_num_in_chip, &fwver_in_chip, &module_id_in_chip);
    
    data->tpd_fwversion = fwver_in_chip;
    data->tpd_vendorid = module_id_in_chip;

    TPD_DMESG("%s Type:0x%x, Partner:0x%x, FwVersion:0x%x.\n", __func__, 
        chip_num_in_chip, module_id_in_chip, fwver_in_chip);

    strcpy(cdev->ic_tpinfo.tp_name, "Focal");
    cdev->ic_tpinfo.chip_model_id = 4;
    
    cdev->ic_tpinfo.chip_part_id= chip_num_in_chip;
    cdev->ic_tpinfo.module_id= module_id_in_chip;
    cdev->ic_tpinfo.chip_ver = 0;
    cdev->ic_tpinfo.firmware_ver= fwver_in_chip;
    cdev->ic_tpinfo.i2c_type = 0;
    cdev->ic_tpinfo.i2c_addr = fts_i2c_client->addr;

#ifdef CONFIG_WIND_DEVICE_INFO
    {
        extern u16 g_ctp_fwvr; 
        extern u16 g_ctp_vendor;
        extern char g_ctp_id_str[21];

        unsigned char uc_reg_value;

        fts_i2c_read_reg(fts_i2c_client, FTS_REG_FW_VER,&uc_reg_value);
        g_ctp_fwvr = uc_reg_value;
        fts_i2c_read_reg(fts_i2c_client, FTS_REG_VENDOR_ID,&uc_reg_value);
        g_ctp_vendor = uc_reg_value;
        if(0xF0 == g_ctp_vendor) {
            snprintf(&g_ctp_id_str[0], 20, "FT3427");
        } else {
            snprintf(&g_ctp_id_str[0], 20, "FT54x6i");
        }
    }
#endif

    //tpd_get_tplcd_res(data);
    tpd_release_stay_awake(cdev);

    return 0;
}

static int tpd_compare_tp(struct tpd_classdev_t *cdev, unsigned char *buf)
{
    u8 uc_host_fm_ver;
    u8 uc_tp_fm_ver;
    int i_ret;
    int fwver_in_chip = 0, module_id_in_chip = 0, chip_num_in_chip = 0;
    u8 fwver_in_file = 0, module_id_in_file = 0;
    struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;

    tpd_require_stay_awake(cdev);

    tpd_get_chipfw_version_etc(data->i2c_client, &chip_num_in_chip, &fwver_in_chip, &module_id_in_chip);

    fwver_in_file = buf[cdev->tp_fw.data_len- 2];
    module_id_in_file = buf[cdev->tp_fw.data_len- 1];

    uc_tp_fm_ver = fwver_in_file;
    uc_host_fm_ver = fwver_in_chip;

    if((module_id_in_chip == module_id_in_file || module_id_in_chip == FTS_REG_VENDOR_ID) && 
    (fwver_in_chip == FTS_REG_FW_VER || fwver_in_chip < fwver_in_file) ) {
        TPD_DMESG("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
        uc_tp_fm_ver, uc_host_fm_ver);
        i_ret = 0;
    } else {
        TPD_DMESG("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
        uc_tp_fm_ver, uc_host_fm_ver);
        TPD_DMESG("[FTS] Not need upgrade firmware\n");
        i_ret = 0xff;
    }

    strcpy(tpd_fw_cdev.file_tpinfo.tp_name, tpd_fw_cdev.ic_tpinfo.tp_name);
    tpd_fw_cdev.file_tpinfo.module_id = module_id_in_file;
    tpd_fw_cdev.file_tpinfo.firmware_ver = fwver_in_file;

    tpd_release_stay_awake(cdev);

    return i_ret;
}

static int tpd_flash_firmware(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size, int force_upg)
{
    char ret = -1;
    
#ifdef CONFIG_SYS_FIRMWARE_UPGRADE
    struct focal_chip_data *focal_data = (struct focal_chip_data*) cdev->private;

    printk("tpd %s function in. \n", __func__);
#ifdef FTS_ESDCHECK_EN
    //add esd cancel code.
#endif
    tpd_require_stay_awake(cdev);
    
    disable_irq(focal_data->touch_irq);
    /*update firmware*/
    if(focal_data->tpd_fwversion == 0x00) {
        ret = tpd_fts_ctpm_firmware_upgrade(cdev, data, size, 1);
    } else {
        ret = tpd_fts_ctpm_firmware_upgrade(cdev, data, size, force_upg);
    }
    if( 0 != ret ) {
        printk("tpd: Update firmware failed! try again...\n");
        ret = tpd_fts_ctpm_firmware_upgrade(cdev, data, size, 1);
    } else {
        printk("tpd: Update firmware success!\n");
        ret = 0;
    }
    enable_irq(focal_data->touch_irq); 
    msleep(100);

    //tpd_init_tpinfo(cdev);
#ifdef FTS_ESDCHECK_EN
    //add esd start code.
#endif

//out:
    tpd_release_stay_awake(cdev);

    return ret;
#else
    ret = 0;

    return ret;
#endif
}

static int tpd_read_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
    int ret = 0;
    int i = 0;
    struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
    unsigned char regaddr = addr;

    ret = fts_i2c_read(data->i2c_client, &regaddr, 1, buf, len);
    TPD_DMESG("Read from addr:0x%x val=", addr);
    for(i = 0; i < (len < 8? len : 8); i++)
    {
        TPD_DMESG("0x%x ", buf[i]);
    }
    TPD_DMESG("\n");
    
    return ret;
}

static int tpd_write_block(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len)
{
    int ret = 0;
    struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
    unsigned char buffer[256];

    buffer[0] = (u8) addr;
    memcpy(&buffer[1], buf, len);
    ret = fts_i2c_write(data->i2c_client, buffer, len + 1);
    
    return ret;
}

static int tpd_enable_wakegesture(struct tpd_classdev_t *cdev, int enable)
{
    struct focal_chip_data *data = (struct focal_chip_data*) cdev->private;
    TPD_DMESG("%s previous val is:%d, current val is:%d.\n", __func__, data->enable_wakeup_gesture, enable);

    data->enable_wakeup_gesture = enable;
    
    return enable;
}

static int tpd_register_fw_class(struct focal_chip_data *data)
{
    tpd_fw_cdev.name = "touchscreen";
    tpd_fw_cdev.private = (void*)data;
    tpd_fw_cdev.flash_fw = tpd_flash_firmware;
    tpd_fw_cdev.read_block = tpd_read_block;
    tpd_fw_cdev.write_block = tpd_write_block;
    tpd_fw_cdev.compare_tp_version = tpd_compare_tp;
    tpd_fw_cdev.get_tpinfo = tpd_init_tpinfo;
    //for black wakeup gesture.
    tpd_fw_cdev.wake_gesture = tpd_enable_wakegesture;
    tpd_classdev_register(&(data->i2c_client->dev), &tpd_fw_cdev);
  //  tpd_init_tpinfo(&tpd_fw_cdev);

#ifdef CONFIG_PROC_FS
    create_tpd_debug_proc_entry(data);
#endif
    return 0;
}
#endif
#define DEBUG_POINT_NUM 5
struct debug_point_desc {
	int id;
	int x;
	int y;
	unsigned int  downnum;
	unsigned char finger_status;
};
static struct debug_point_desc g_point_array[DEBUG_POINT_NUM];
static unsigned long g_var_irq_num = 0;
static unsigned long g_var_point_num = 0;

static void debug_point_print(int id, int x, int y, int w, int down)
{
	int i = 0;
	int down_num = 0;
	int pre_finger_status= 0;

	if(id < DEBUG_POINT_NUM && 1 == down) {
		pre_finger_status = g_point_array[id].finger_status;
		g_var_point_num++;
		g_point_array[id].finger_status = down;
		g_point_array[id].downnum++;
		g_point_array[id].x = x;
		g_point_array[id].y = y;
		down_num = g_point_array[id].downnum;
		if((0 == pre_finger_status && 1 == down) || down_num % 90 == 0) {
			//printk("tpd down\n");
			printk("tpd d %d(%d,%d)%d D:%d Q:%ld N:%ld\n", id, x, y, w, 
				g_point_array[id].downnum, g_var_irq_num, g_var_point_num);
		}
	} else if(0 == down) {
		for(i = 0; i < DEBUG_POINT_NUM; i++) {
			if(1==g_point_array[i].finger_status) {
				//printk("tpd up\n");
				printk("tpd u %d(%d,%d) D:%d\n",i, g_point_array[i].x, g_point_array[i].y, g_point_array[i].downnum);
			}
			g_point_array[i].finger_status=0;
			g_point_array[i].downnum=0;
			g_point_array[i].x=0;
			g_point_array[i].y=0;
		}
	}
}
#if FTS_CHARGE_DETECT_EN
void tpd_usb_plugin(u8 plugin)
{
    int ret = -1;
    b_usb_plugin = plugin;

    if (!ctp_is_probe)
    {
        return;
    }
    FTS_DEBUG("Fts usb detect: %s %d.\n",__func__,b_usb_plugin);

    ret = i2c_smbus_write_i2c_block_data(fts_i2c_client, 0x8B, 1, &b_usb_plugin);
    if ( ret < 0 )
    {
        FTS_DEBUG("Fts usb detect write err: %s %d.\n",__func__,b_usb_plugin);
    }
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif



#if (!FTS_MT_PROTOCOL_B_EN)
static void tpd_down(int x, int y, int p, int id)
{
    FTS_DEBUG("Point%d(%d, %d) DOWN!", id, x, y );
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);

    debug_point_print(id, x, y, p, 1);
}

static void tpd_up(int x, int y)
{
    FTS_DEBUG("All Up!");
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);

    debug_point_print(0, x, y, 0, 0);
}


static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
    char data[80] = {0};
    u16 high_byte, low_byte;
    char writebuf[10]= {0};

    writebuf[0]=0x00;
    fts_i2c_read(fts_i2c_client, writebuf,  1, data, 32);
    if ((data[0] & 0x70) != 0)
        return false;

    memcpy(pinfo, cinfo, sizeof(struct touch_info));
    memset(cinfo, 0, sizeof(struct touch_info));
    for (i = 0; i < FTS_MAX_POINTS; i++)
        cinfo->p[i] = 1;    /* Put up */

    /*get the number of the touch points*/
    cinfo->count = data[2] & 0x0f;
    FTS_DEBUG("Number of touch points = %d", cinfo->count);

    for (i = 0; i < cinfo->count; i++)
    {
        cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003; /* event flag */
        cinfo->id[i] = data[3+6*i+2]>>4;                        // touch id

        /*get the X coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 1];
        low_byte &= 0x00FF;
        cinfo->x[i] = high_byte | low_byte;

        /*get the Y coordinate, 2 bytes*/
        high_byte = data[3 + 6 * i + 2];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 3];
        low_byte &= 0x00FF;
        cinfo->y[i] = high_byte | low_byte;

        FTS_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d\n", i,
                  cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
    }

    return true;
};
#else
/************************************************************************
* Name: fts_read_touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static int fts_read_touchdata(struct ts_event *data)
{
    u8 buf[POINT_READ_BUF] = { 0 };//0xFF
    int ret = -1;
    int i = 0;
    u8 pointid = FTS_MAX_ID;

    FTS_FUNC_ENTER();
    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
    if (ret < 0)
    {
        FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
        FTS_FUNC_EXIT();
        return ret;
    }

    memset(data, 0, sizeof(struct ts_event));
    data->touch_point = 0;
    data->touch_point_num=buf[FT_TOUCH_POINT_NUM] & 0x0F;

    for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
    {
        pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else
            data->touch_point++;
        data->au16_x[i] =
            (s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
            8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
        data->au16_y[i] =
            (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) <<
            8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
        data->au8_touch_event[i] =
            buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
        data->au8_finger_id[i] =
            (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

        data->pressure[i] =
            (buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);//cannot constant value
        data->area[i] =
            (buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
        if ((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&((data->touch_point_num==0)||(data->pressure[i]==0 && data->area[i]==0  )))
            return 1;

    }
    FTS_FUNC_EXIT();
    return 0;
}
/************************************************************************
* Name: fts_report_key
* Brief: report key event
* Input: event info
* Output: no
* Return: 0: is key event, -1: isn't key event
***********************************************************************/
static int fts_report_key(struct ts_event *data)
{
    int i = 0;

    if (1 != data->touch_point)
        return -1;

    for (i = 0; i < FTS_MAX_POINTS; i++)
    {
        if (data->au16_y[i] <= TPD_RES_Y)
        {
            return -1;
        }
    }
    if(i >= FTS_MAX_POINTS) {
        goto out;
    }
    if (data->au8_touch_event[i]== 0 ||
        data->au8_touch_event[i] == 2)
    {
        tpd_button(data->au16_x[i], data->au16_y[i], 1);
        printk("tpd : [B]Key(%d, %d) DOWN!", data->au16_x[i], data->au16_y[i]);
    }
    else
    {
        tpd_button(data->au16_x[i], data->au16_y[i], 0);
        printk("tpd : [B]Key(%d, %d) UP!", data->au16_x[i], data->au16_y[i]);
    }

    input_sync(tpd->dev);

out:
    return 0;
}

/************************************************************************
* Name: fts_report_value
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_value(struct ts_event *data)
{
    int i = 0,j=0;
    int up_point = 0;
    int touchs = 0;

    for (i = 0; i < data->touch_point; i++)
    {
        input_mt_slot(tpd->dev, data->au8_finger_id[i]);

        if (data->au8_touch_event[i]== 0 || data->au8_touch_event[i] == 2)
        {
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
            input_report_key(tpd->dev, BTN_TOUCH, 1);
            input_report_abs(tpd->dev, ABS_MT_PRESSURE,data->pressure[i]);//0x3f
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,data->area[i]);//0x05
            input_report_abs(tpd->dev, ABS_MT_POSITION_X,data->au16_x[i]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y,data->au16_y[i]);
            touchs |= BIT(data->au8_finger_id[i]);
            data->touchs |= BIT(data->au8_finger_id[i]);

            debug_point_print(data->au8_finger_id[i], data->au16_x[i], data->au16_y[i], data->area[i], 1);

            FTS_DEBUG("[B]Point%d(%d, %d) DOWN!", data->au8_finger_id[i], data->au16_x[i], data->au16_y[i]);

        }
        else
        {
            up_point++;
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
            data->touchs &= ~BIT(data->au8_finger_id[i]);
            FTS_DEBUG("[B]Point%d UP!", data->au8_finger_id[i]);
        }

    }
    for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
    {
        if (BIT(i) & (data->touchs ^ touchs))
        {
            FTS_DEBUG("[B]Point%d UP!", i);
            data->touchs &= ~BIT(i);
            input_mt_slot(tpd->dev, i);
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
        }
    }
    data->touchs = touchs;

    if ((data->touch_point_num==0))   // release all touches in final
    {
        for (j = 0; j <fts_updateinfo_curr.TPD_MAX_POINTS; j++)
        {
            input_mt_slot( tpd->dev, j);
            input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
        }
        data->touchs=0;
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_sync(tpd->dev);
        FTS_DEBUG("[B]All UP!");
        debug_point_print(0, 0, 0, 0, 0);
        return 0;
    }

    if (data->touch_point == up_point)
    {
        input_report_key(tpd->dev, BTN_TOUCH, 0);
    }
    else
        input_report_key(tpd->dev, BTN_TOUCH, 1);

    input_sync(tpd->dev);
    return 0;
}
#endif

static int touch_event_handler(void *unused)
{
    int i = 0;
    int ret;
#if FTS_GESTURE_EN
    u8 state = 0;
    struct focal_chip_data *focal_data = i2c_get_clientdata(fts_i2c_client);
#endif
#if FTS_MT_PROTOCOL_B_EN
    struct ts_event pevent;
#else
    struct touch_info  cinfo, pinfo;
#endif

    struct touch_info  finfo;
    struct sched_param param = { .sched_priority = 4 };// 4 == RTPM_PRIO_TPD

    if (tpd_dts_data.use_tpd_button)
    {
        memset(&finfo, 0, sizeof(struct touch_info));
        for (i = 0; i < FTS_MAX_POINTS; i++)
            finfo.p[i] = 1;
    }

    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);

#if FTS_GESTURE_EN
        if(focal_data->enable_wakeup_gesture && focal_data->tpd_suspend){

	     wake_lock_timeout(&focal_data->tpd_bsg_wake_lock, 2 * HZ);

            ret = fts_i2c_read_reg(fts_i2c_client, 0xd0,&state);
            if (ret<0)
            {
                printk("[Focal][Touch] read value fail");
            }
            if (state ==1)
            {
                fts_gesture_readdata();
                continue;
            }
        }
#endif

        FTS_DEBUG("touch_event_handler start");
#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(1);
#endif
#if FTS_MT_PROTOCOL_B_EN
        {
            ret = fts_read_touchdata(&pevent);
            if (ret == 0)
            {
                if (tpd_dts_data.use_tpd_button)
                {
                    ret = !fts_report_key(&pevent);
                }
                if (ret == 0)
                {
                    fts_report_value(&pevent);
                }
            }
        }
#else //FTS_MT_PROTOCOL_B_EN
        {
            if (tpd_touchinfo(&cinfo, &pinfo))
            {
                if (tpd_dts_data.use_tpd_button)
                {
                    if (cinfo.p[0] == 0)
                        memcpy(&finfo, &cinfo, sizeof(struct touch_info));
                }

                if ((cinfo.y[0] >= TPD_RES_Y) && (pinfo.y[0] < TPD_RES_Y)
                    && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                {
                    FTS_DEBUG("All up");
                    tpd_up(pinfo.x[0], pinfo.y[0]);
                    input_sync(tpd->dev);
                    continue;
                }

                if (tpd_dts_data.use_tpd_button)
                {
                    if ((cinfo.y[0] <= TPD_RES_Y && cinfo.y[0] != 0) && (pinfo.y[0] > TPD_RES_Y)
                        && ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                    {
                        printk("All key up");
                        tpd_button(pinfo.x[0], pinfo.y[0], 0);
                        input_sync(tpd->dev);
                        continue;
                    }

                    if ((cinfo.y[0] > TPD_RES_Y) || (pinfo.y[0] > TPD_RES_Y))
                    {
                        if (finfo.y[0] > TPD_RES_Y)
                        {
                            if ((cinfo.p[0] == 0) || (cinfo.p[0] == 2))
                            {
                                printk("Key(%d,%d) Down", pinfo.x[0], pinfo.y[0]);
                                tpd_button(pinfo.x[0], pinfo.y[0], 1);
                            }
                            else if ((cinfo.p[0] == 1) &&
                                     ((pinfo.p[0] == 0) || (pinfo.p[0] == 2)))
                            {
                                printk("Key(%d,%d) Up!", pinfo.x[0], pinfo.y[0]);
                                tpd_button(pinfo.x[0], pinfo.y[0], 0);
                            }
                            input_sync(tpd->dev);
                        }
                        continue;
                    }
                }
                if (cinfo.count > 0)
                {
                    for (i = 0; i < cinfo.count; i++)
                        tpd_down(cinfo.x[i], cinfo.y[i], i + 1, cinfo.id[i]);
                }
                else
                {
                    tpd_up(cinfo.x[0], cinfo.y[0]);
                }
                input_sync(tpd->dev);

            }
        }
#endif
#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(0);
#endif
    }
    while (!kthread_should_stop());

    return 0;
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);

    return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
    //FTS_DEBUG("TPD interrupt has been triggered");
    tpd_flag = 1;
    g_var_irq_num++;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
    struct focal_chip_data *focal_data = i2c_get_clientdata(fts_i2c_client);
    int ret = 0;

    FTS_FUNC_ENTER();

    if (focal_data->touch_irq)    //(node)
    {

        ret = request_irq(focal_data->touch_irq, tpd_eint_interrupt_handler,
                    IRQF_ONESHOT |IRQF_TRIGGER_FALLING, TPD_DEVICE, NULL);
        if (ret > 0)
            FTS_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
    }
    else
    {
        FTS_ERROR("Can not find touch eint device node!");
    }

		enable_irq_wake(focal_data->touch_irq);
    FTS_FUNC_EXIT();
    return 0;
}


static int of_get_focal_platform_data(struct device *dev)
{
    int retval = -1;
    struct focal_chip_data *focal_data = i2c_get_clientdata(fts_i2c_client);


	dev->of_node = of_find_matching_node(dev->of_node, touch_of_match);
    
    /*int ret, num;*/
    if (dev->of_node) {
    focal_data->avdd_regulator = regulator_get(dev, "vtouch");
    retval = regulator_set_voltage(focal_data->avdd_regulator, 2800000, 2800000);
    if (retval != 0) {
        TPD_DMESG("Failed to set reg-vgp1 voltage: %d\n", retval);
        return -1;
    	}
    focal_data->touch_irq = irq_of_parse_and_map(dev->of_node, 0);
//    focal_data->tpd_rst_gpio_number = of_get_named_gpio(dev->of_node, "rst-gpio", 0);
//    focal_data->tpd_int_gpio_number = of_get_named_gpio(dev->of_node, "int-gpio", 0);

//    TPD_DMESG("rst_gpio_number %d\n", focal_data->tpd_rst_gpio_number);
//    TPD_DMESG("int_gpio_number %d\n", focal_data->tpd_int_gpio_number);
    TPD_DMESG("touch_irq %d\n", focal_data->touch_irq);
    	}
    return 0;
	
}

int fts_reset_proc(void)
{

    tpd_gpio_output(tpd_rst_gpio_number, 0);
    msleep(20);
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    msleep(200);

    return 0;
}

/************************************************************************
* Name: fts_probe
* Brief: i2c read
* Input: i2c info, write buf, write len, read buf, read len
* Output: get data in the 3rd buf
* Return: fail < 0
***********************************************************************/
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = 0;
    unsigned char uc_reg_value;
    struct focal_chip_data *focal_data;

    FTS_FUNC_ENTER();

    fts_i2c_client = client;
    fts_input_dev = tpd->dev;
    
    focal_data = kzalloc(sizeof(*focal_data), GFP_KERNEL);
    if (!focal_data) {
        dev_err(&client->dev,
                "%s: Failed to alloc mem for focal_data\n",
                __func__);
        return -ENOMEM;
    }
    focal_data->i2c_client = client;
    i2c_set_clientdata(client, focal_data);
    
    of_get_focal_platform_data(&client->dev);

    wake_lock_init(&focal_data->tpd_bsg_wake_lock, WAKE_LOCK_SUSPEND, "tpd_bsg_wake");

    if (fts_i2c_client->addr != 0x38)
    {
        FTS_INFO("Change i2c addr 0x%02x to 0x38", fts_i2c_client->addr);
        fts_i2c_client->addr = 0x38;
        FTS_DEBUG("fts_i2c_client->addr=%x\n", fts_i2c_client->addr);
    }

    /* GITAR*/

			if(focal_data->avdd_regulator) {
			        retval = regulator_enable(focal_data->avdd_regulator);
			        if (retval != 0)
			            TPD_DMESG("Failed to enable reg-vgp6: %d\n", retval);
			    }


		tpd_gpio_output(tpd_rst_gpio_number, 0);
			 mdelay(5);
		tpd_gpio_output(tpd_rst_gpio_number, 1);

			    /* set INT mode */
		tpd_gpio_output(tpd_int_gpio_number, 1);
			  msleep(300);


    retval = fts_i2c_init();

    if(retval < 0) {
        printk("mtk_tpd[FTS] Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
        goto error_i2c_failed;
    }
	

    tpd_gpio_as_int(tpd_int_gpio_number);
    tpd_irq_registration();



#if FTS_GESTURE_EN
    fts_gesture_init(tpd->dev);
#endif

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(tpd->dev, FTS_MAX_POINTS,1);
#endif

    //fts_reset_proc();
    tpd_load_status = 1;



#if FTS_SYSFS_NODE_EN
    fts_create_sysfs(fts_i2c_client);
#endif

    fts_get_upgrade_array();
#if FTS_CTL_IIC_NODE_EN
    if (fts_rw_iic_drv_init(fts_i2c_client) < 0)
        dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
#endif

#if FTS_APK_NODE_EN
    fts_create_apk_debug_channel(fts_i2c_client);
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_init();
#endif
    thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread_tpd))
    {
        retval = PTR_ERR(thread_tpd);
        FTS_DEBUG(" failed to create kernel thread_tpd: %d\n", retval);
    }

    FTS_DEBUG("Touch Panel Device Probe %s\n", (retval < 0) ? "FAIL" : "PASS");

#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "tp_wakelock");
#endif

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
    fts_sensor_init();
#endif

#if FTS_CHARGE_DETECT_EN  //add by wangyang
    if (ctp_is_probe == 0)
    {
        i2c_smbus_write_i2c_block_data(fts_i2c_client, 0x8B, 1, &b_usb_plugin);
        ctp_is_probe =1;
    }
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(ENABLE);
#endif

#if FTS_AUTO_UPGRADE_EN
    {
        retval = fts_workqueue_init();
        if ( retval != 0 )
        {
            printk( "fts_workqueue_init failed\n" );
        }
        else
            queue_work ( touch_wq, &fw_update_work );
    }
#endif


#ifdef CONFIG_CREATE_SYS_INTERFACE
    tpd_register_fw_class(focal_data);
#endif

#if FTS_TEST_EN
    fts_test_init(fts_i2c_client);
#endif

    FTS_FUNC_EXIT();

    return 0;

error_i2c_failed:
    gpio_free(focal_data->tpd_int_gpio_number);
    gpio_free(focal_data->tpd_rst_gpio_number);
    fts_i2c_exit();
    return -1;
}

static int tpd_remove(struct i2c_client *client)
{
    FTS_FUNC_ENTER();

#if FTS_TEST_EN
    fts_test_exit(fts_i2c_client);
#endif

#if FTS_CTL_IIC_NODE_EN
    fts_rw_iic_drv_exit();
#endif

#if FTS_SYSFS_NODE_EN
    fts_remove_sysfs(client);
#endif

#if FTS_APK_NODE_EN
    fts_release_apk_debug_channel();
#endif

#if FTS_AUTO_UPGRADE_EN
    cancel_work_sync(&fw_update_work);
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_exit();
#endif
    fts_i2c_exit();

    FTS_FUNC_EXIT();

    return 0;
}


/*set TP volt*/
static int tpd_local_init(void)
{

    FTS_FUNC_ENTER();

	
#if FTS_POWER_SOURCE_CUST_EN
    {
        int retval;

        tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
        retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
        if (retval != 0)
        {
            FTS_DEBUG("%s: Failed to enable reg-vgp6,ret=%d\n", __func__, retval);
            FTS_FUNC_EXIT();
            return -1;
        }
    }
#endif
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        FTS_ERROR("%s: Unable to add fts i2c driver!\n", __func__);
        FTS_FUNC_EXIT();
        return -1;
    }
    /* tpd_load_status = 1; */
    if (tpd_dts_data.use_tpd_button)
    {
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
                           tpd_dts_data.tpd_key_dim_local);
    }

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

    memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
#endif

    tpd_type_cap = 1;

    FTS_FUNC_EXIT();
    return 0;
}

static void fts_release_all_finger(void)
{
#if FTS_MT_PROTOCOL_B_EN
    unsigned int finger_count=0;
#endif

    FTS_DEBUG("Release all finger");
#if (!FTS_MT_PROTOCOL_B_EN)
    input_mt_sync ( tpd->dev );
#else
    for (finger_count = 0; finger_count < FTS_MAX_POINTS; finger_count++)
    {
        input_mt_slot( tpd->dev, finger_count);
        input_mt_report_slot_state( tpd->dev, MT_TOOL_FINGER, false);
    }
    input_report_key(tpd->dev, BTN_TOUCH, 0);
#endif
    input_sync ( tpd->dev );

    g_var_irq_num = 0;
    g_var_point_num = 0;
}

static void tpd_resume(struct device *h)
{
    //int retval = 0;
    //char val;
    struct focal_chip_data *focal_data = i2c_get_clientdata(fts_i2c_client);

    printk("tpd: fts %s func in. \n", __func__);

    FTS_FUNC_ENTER();
    if(focal_data->need_stay_awake) {
        printk("%s need_stay_awake return.\n", __func__);
        focal_data->tpd_suspend = false;
        return;
    }
    focal_data->tpd_suspend = false;

    fts_release_all_finger();


#if FTS_POWER_SOURCE_CUST_EN
    {
        retval = regulator_enable(tpd->reg);
        if (retval != 0)
        {
            FTS_DEBUG("%s: Failed to enable reg-vgp6,ret=%d\n", __func__, retval);
        }
    }
#endif

    fts_reset_proc();

#if FTS_GESTURE_EN
    if(focal_data->enable_wakeup_gesture) {
        fts_i2c_write_reg(fts_i2c_client,0xD0,0x00);
    }
#endif


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
    fts_sensor_enable(fts_i2c_client);
#else
    enable_irq(ft_touch_irq);
#endif

    //fts_release_all_finger();

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

#if FTS_CHARGE_DETECT_EN
    msleep(120);
    tpd_usb_plugin(b_usb_plugin);
#endif



    printk("tpd: fts %s func out. \n", __func__);
}

static void tpd_suspend(struct device *h)
{
#if FTS_POWER_SOURCE_CUST_EN
    int retval = 0;
#endif
    static char data = 0x3;
    struct focal_chip_data *focal_data = i2c_get_clientdata(fts_i2c_client);

    printk("tpd: fts %s func in. \n", __func__);

    FTS_FUNC_ENTER();
    if(focal_data->need_stay_awake) {
        printk("%s need_stay_awake return.\n", __func__);
        return;
    }
    focal_data->tpd_suspend = true;

#if FTS_GESTURE_EN
    if(focal_data->enable_wakeup_gesture) {
#if FTS_ESDCHECK_EN
        fts_esdcheck_suspend(ENABLE);
#endif
        fts_gesture_suspend(fts_i2c_client);
        return;
    }
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend(DISABLE);
#endif


#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
    fts_sensor_suspend(fst_i2c_client);
#else
    disable_irq(ft_touch_irq);
    fts_i2c_write_reg(fts_i2c_client, 0xA5, data);  /* TP enter sleep mode */

#if FTS_POWER_SOURCE_CUST_EN
    retval = regulator_disable(tpd->reg);
    if (retval != 0)
        FTS_DEBUG("Failed to disable reg-vgp6: %d\n", retval);
#endif

#endif


    //fts_release_all_finger();
    FTS_FUNC_EXIT();
    printk("tpd: fts %s func out. \n", __func__);
}

#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
static DEVICE_ATTR(tpd_scp_ctrl, 0664, show_scp_ctrl, store_scp_ctrl);
#endif

struct device_attribute *fts_attrs[] =
{
#ifdef CONFIG_MTK_SENSOR_HUB_SUPPORT
    &dev_attr_tpd_scp_ctrl,
#endif
};


static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "focaltech",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
    .attrs = {
        .attr = fts_attrs,
        .num  = ARRAY_SIZE(fts_attrs),
    },
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    FTS_FUNC_ENTER();
    FTS_INFO("Driver version: %s", FTS_DRIVER_VERSION);
    tpd_get_dts_info();
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        FTS_ERROR("%s: Add FTS Touch driver failed!\n", __func__);
    }

    FTS_FUNC_EXIT();
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    FTS_FUNC_ENTER();
    tpd_driver_remove(&tpd_device_driver);
    FTS_FUNC_EXIT();
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver for Mediatek");
MODULE_LICENSE("GPL v2");
