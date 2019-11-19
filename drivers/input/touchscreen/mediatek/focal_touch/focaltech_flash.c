/*
 *
 * FocalTech fts TouchScreen driver.
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
* File Name: focaltech_flash.c
*
* Author:    fupeipei
*
* Created:    2016-08-08
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include "focaltech_flash.h"
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/kthread.h>


/*******************************************************************************
* Static variables
*******************************************************************************/
struct fts_Upgrade_Info fts_updateinfo[] =
{
    {0x54,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 20, 2000}, //,"FT5x46"
    {0x55,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, //,"FT5x06"
    {0x08,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000}, //,"FT5606"
    {0x0a,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500}, //,"FT5x16"
    {0x06,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000}, //,"FT6x06"
    {0x36,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000}, //,"FT6x36"
    {0x64,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 2000}, //,"FT6336GU"
    {0x55,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, //,"FT5x06i"
    {0x14,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, //,"FT5336"
    {0x13,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, //,"FT3316"
    {0x12,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, //,"FT5436i"
    {0x11,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, //,"FT5336i"
    {0x58,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x58, 0x2c, 20, 2000},//"FT5822"
    {0x59,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,30, 50, 0x79, 0x10, 1, 2000},//"FT5x26"
    {0x86,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x86, 0xA7, 20, 2000},//"FT8607"
    {0x87,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xA6, 20, 2000},//"FT8716"
    {0x0E,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000}, //,"FT3x07"
};

static unsigned  char *CTPM_FW;
/*{
	//#include FTS_UPGRADE_FW_APP
	//#include "include/firmware/SKI500_B18_FT34x7_V01_D0B_20160815_app.h"
	#include"include/firmware/ZTE_P637S15_K15_FT3327_LCE_0X87_BOE_V15_20170712_app.h"
};*/

static unsigned char CTPM_FW1[] =
{
         #include FW1_PATH
     
};
static unsigned char CTPM_FW2[] =
{
         #include FW2_PATH
     
};
static unsigned char CTPM_FW3[] =
{
         #include FW3_PATH
     
};
static unsigned char CTPM_FW4[] =
{
         #include FW4_PATH
     
};
static unsigned char CTPM_FW5[] =
{
         #include FW5_PATH
     
};

static unsigned char aucFW_PRAM_BOOT[] =
{
	#include FTS_UPGRADE_PRAMBOOT
};

int fw_size;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
int tpd_get_chipfw_version_etc(struct i2c_client *i2c_client, int * chip_num_in_chip, 
    int *fwver_in_chip, int *module_id_in_chip);

struct fts_Upgrade_Info fts_updateinfo_curr;

#if FTS_AUTO_UPGRADE_EN
static bool is_update = false;
struct workqueue_struct *touch_wq;
struct work_struct fw_update_work;
#endif

#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
int uc_UpgradeTimes = 0;
int uc_UpgradeReturnTime = 0;
u8 uc_AUOUpgradeTimes = 0;
#endif

static int g_tpd_forceUpgrade = 0;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/


/************************************************************************
* Name: fts_i2c_hid2std
* Brief:  HID to I2C
* Input: i2c info
* Output: no
* Return: fail =0
***********************************************************************/
int fts_i2c_hid2std(struct i2c_client * client)
{
    u8 auc_i2c_write_buf[5] = {0};
    int bRet = 0;
#if HIDTOI2C_DISABLE
    return 0;
#endif

    auc_i2c_write_buf[0] = 0xeb;
    auc_i2c_write_buf[1] = 0xaa;
    auc_i2c_write_buf[2] = 0x09;
    bRet =fts_i2c_write(client, auc_i2c_write_buf, 3);
    msleep(10);
    auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
    fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);

    if (0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
    {
        pr_info("fts_i2c_hid2std successful.\n");
        bRet = 1;
    }
    else
    {
        pr_err("fts_i2c_hid2std error.\n");
        bRet = 0;
    }

    return bRet;
}

/*******************************************************************************
* Name: fts_update_fw_vendor_id
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_vendor_id(struct fts_ts_data *data)
{
    struct i2c_client *client = data->client;
    u8 reg_addr;
    int err;

    reg_addr = FTS_REG_FW_VENDOR_ID;
    err = fts_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
    if (err < 0)
        dev_err(&client->dev, "fw vendor id read failed");
}

/*******************************************************************************
* Name: fts_update_fw_ver
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_ver(struct fts_ts_data *data)
{
    struct i2c_client *client = data->client;
    u8 reg_addr;
    int err;

    reg_addr = FTS_REG_FW_VER;
    err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
    if (err < 0)
        dev_err(&client->dev, "fw major version read failed");

    reg_addr = FTS_REG_FW_MIN_VER;
    err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
    if (err < 0)
        dev_err(&client->dev, "fw minor version read failed");

    reg_addr = FTS_REG_FW_SUB_MIN_VER;
    err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
    if (err < 0)
        dev_err(&client->dev, "fw sub minor version read failed");

    dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
             data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadVendorID
* Brief:  read vendor ID
* Input: i2c info, vendor ID
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadVendorID(struct i2c_client *client, u8 *ucPVendorID)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u8 auc_i2c_write_buf[10];
    int i_ret;

    *ucPVendorID = 0;
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        i_ret = fts_i2c_hid2std(client);
        if (i_ret == 0)
        {
            FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
        }
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("failed writing  0x55 and 0xaa ! \n");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(10);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            FTS_DEBUG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*********Step 4: read vendor id from app param area***********************/
    msleep(10);
    auc_i2c_write_buf[0] = 0x03;
    auc_i2c_write_buf[1] = 0x00;
    auc_i2c_write_buf[2] = 0xd7;
    auc_i2c_write_buf[3] = 0x84;
    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        fts_i2c_write(client, auc_i2c_write_buf, 4);
        msleep(5);
        reg_val[0] = reg_val[1] = 0x00;
        i_ret = fts_i2c_read(client, auc_i2c_write_buf, 0, reg_val, 2);
        if (0 != reg_val[0])
        {
            *ucPVendorID = 0;
            FTS_DEBUG("In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n", reg_val[0], reg_val[1], 0, i_ret);
        }
        else
        {
            *ucPVendorID = reg_val[0];
            FTS_DEBUG("In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n", reg_val[0], reg_val[1]);
            break;
        }
    }
    msleep(50);
    /*********Step 5: reset the new FW***********************/
    FTS_DEBUG("Step 5: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
    }
    msleep(10);
    return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadProjectCode
* Brief:  read project code
* Input: i2c info, project code
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadProjectCode(struct i2c_client *client, char *pProjectCode)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u8 j = 0;
    u8 auc_i2c_write_buf[10];
    int i_ret;
    u32 temp;
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
    }
    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        i_ret = fts_i2c_hid2std(client);
        if (i_ret == 0)
        {
            FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
        }
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("failed writing  0x55 and 0xaa ! \n");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(10);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            FTS_DEBUG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*********Step 4: read vendor id from app param area***********************/
    msleep(10);
    /*read project code*/
    auc_i2c_write_buf[0] = 0x03;
    auc_i2c_write_buf[1] = 0x00;
    for (j = 0; j < 33; j++)
    {
        temp = 0xD7A0 + j;
        auc_i2c_write_buf[2] = (u8)(temp>>8);
        auc_i2c_write_buf[3] = (u8)temp;
        fts_i2c_read(client, auc_i2c_write_buf, 4, pProjectCode+j, 1);
        if (*(pProjectCode+j) == '\0')
            break;
    }
    pr_info("project code = %s \n", pProjectCode);
    msleep(50);
    /*********Step 5: reset the new FW***********************/
    FTS_DEBUG("Step 5: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("hidi2c change to stdi2c fail ! \n");
    }
    msleep(10);
    return 0;
}

/************************************************************************
* Name: fts_get_upgrade_array
* Brief: decide which ic
* Input: no
* Output: get ic info in fts_updateinfo_curr
* Return: no
***********************************************************************/
void fts_get_upgrade_array(void)
{

    u8 chip_id;
    u32 i;
    int ret = 0;
    unsigned char auc_i2c_write_buf[10];
    unsigned char reg_val[4] = {0};

    fts_i2c_hid2std(fts_i2c_client);

    for (i=0; i<5; i++)
    {
        ret = fts_i2c_read_reg(fts_i2c_client, FTS_REG_ID,&chip_id);
        if (ret<0)
        {
            FTS_DEBUG("[Focal][Touch] read value fail");
        }
        else
        {
            break;
        }
    }
    FTS_DEBUG("%s chip_id = %x\n", __func__, chip_id);
    if (ret<0 || chip_id == 0xEF)
    {
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        fts_i2c_write(fts_i2c_client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_readid);

        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        fts_i2c_read(fts_i2c_client, auc_i2c_write_buf, 4, reg_val, 2);

        if (reg_val[0] == FTS_CHIP_ID)
        {
            chip_id = FTS_CHIP_ID;
            FTS_DEBUG("[UPGRADE]: << %s >> : read id for success , id is: ID1 = 0x%x,ID2 = 0x%x:\n", __func__, reg_val[0], reg_val[1]);
        }
        else
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : read id for test error: id is: ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
        }
    }
    for (i=0; i<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info); i++)
    {
        if (chip_id==fts_updateinfo[i].CHIP_ID || FTS_CHIP_ID==fts_updateinfo[i].CHIP_ID)
        {
            memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct fts_Upgrade_Info));
            break;
        }
    }

    if (i >= sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info))
    {
        memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct fts_Upgrade_Info));
    }
}

/************************************************************************
* Name: fts_8716_ctpm_fw_write_pram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
void fts_rom_or_pram_reset(struct i2c_client * client)
{
    u8 auc_i2c_write_buf[10];

    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
}

/************************************************************************
* Name: fts_ctpm_auto_clb
* Brief:  auto calibration
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_clb(struct i2c_client *client)
{
    unsigned char uc_temp = 0x00;
    unsigned char i = 0;

    /*start auto CLB */
    msleep(200);

    fts_i2c_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
    /*make sure already enter factory mode */
    msleep(100);
    /*write command to start calibration */
    fts_i2c_write_reg(client, 2, 0x4);
    msleep(300);
    if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)) //5x36,5x36i
    {
        for (i=0; i<100; i++)
        {
            fts_i2c_read_reg(client, 0x02, &uc_temp);
            if (0x02 == uc_temp ||
                0xFF == uc_temp)
            {
                break;
            }
            msleep(20);
        }
    }
    else
    {
        for (i=0; i<100; i++)
        {
            fts_i2c_read_reg(client, 0, &uc_temp);
            if (0x0 == ((uc_temp&0x70)>>4))
            {
                break;
            }
            msleep(20);
        }
    }
    fts_i2c_write_reg(client, 0, 0x40);
    msleep(200);
    fts_i2c_write_reg(client, 2, 0x5);
    msleep(300);
    fts_i2c_write_reg(client, 0, FTS_WORKMODE_VALUE);
    msleep(300);
    return 0;
}

/************************************************************************
* Name: fts_6x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u32 fw_length;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;

    if (pbt_buf[0] != 0x02)
    {
        FTS_DEBUG("[FTS] FW first byte is not 0x02. so it is invalid \n");
        return -1;
    }

    if (dw_lenth > 0x11f)
    {
        fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
        if (dw_lenth < fw_length)
        {
            FTS_DEBUG("[FTS] Fw length is invalid \n");
            return -1;
        }
    }
    else
    {
        FTS_DEBUG("[FTS] Fw length is invalid \n");
        return -1;
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_readid);
        /*********Step 3:check READ-ID***********************/
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
                      reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
                    reg_val[0], reg_val[1]);
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;

    auc_i2c_write_buf[0] = FTS_READ_ID_REG;
    auc_i2c_write_buf[1] = 0x00;
    auc_i2c_write_buf[2] = 0x00;
    auc_i2c_write_buf[3] = 0x00;
    auc_i2c_write_buf[4] = 0x00;
    fts_i2c_write(client, auc_i2c_write_buf, 5);

    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(fts_updateinfo_curr.delay_erase_flash);

    for (i = 0; i < 200; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        auc_i2c_write_buf[1] = 0x00;
        auc_i2c_write_buf[2] = 0x00;
        auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (0xb0 == reg_val[0] && 0x02 == reg_val[1])
        {
            FTS_DEBUG("[FTS] erase app finished \n");
            break;
        }
        msleep(50);
    }

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");

    dw_lenth = fw_length;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }


    /*********Step 6: read out checksum***********************/
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);
        return -EIO;
    }

    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    return 0;
}
/************************************************************************
* Name: fts_6336GU_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6336GU_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u32 fw_length;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;

    if (pbt_buf[0] != 0x02)
    {
        FTS_DEBUG("[FTS] FW first byte is not 0x02. so it is invalid \n");
        return -1;
    }

    if (dw_lenth > 0x11f)
    {
        fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
        if (dw_lenth < fw_length)
        {
            FTS_DEBUG("[FTS] Fw length is invalid \n");
            return -1;
        }
    }
    else
    {
        FTS_DEBUG("[FTS] Fw length is invalid \n");
        return -1;
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_readid);
        /*********Step 3:check READ-ID***********************/
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
                      reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
                    reg_val[0], reg_val[1]);
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;

    auc_i2c_write_buf[0] = FTS_READ_ID_REG;
    auc_i2c_write_buf[1] = 0x00;
    auc_i2c_write_buf[2] = 0x00;
    auc_i2c_write_buf[3] = 0x00;
    auc_i2c_write_buf[4] = 0x00;
    fts_i2c_write(client, auc_i2c_write_buf, 5);

    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(fts_updateinfo_curr.delay_erase_flash);

    for (i = 0; i < 200; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        auc_i2c_write_buf[1] = 0x00;
        auc_i2c_write_buf[2] = 0x00;
        auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (0xb0 == reg_val[0] && 0x02 == reg_val[1])
        {
            FTS_DEBUG("[FTS] erase app finished \n");
            break;
        }
        msleep(50);
    }

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");

    dw_lenth = fw_length;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }


    /*********Step 6: read out checksum***********************/
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);
        return -EIO;
    }

    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    return 0;
}
/************************************************************************
* Name: fts_6x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);

        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);

        msleep(fts_updateinfo_curr.delay_55);

        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        do
        {
            i++;
            i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
            msleep(5);
        }
        while (i_ret <= 0 && i < 5);


        /*********Step 3:check READ-ID***********************/
        msleep(fts_updateinfo_curr.delay_readid);
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: CTPM ID OK ,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
        }
    }
    if (i > FTS_UPGRADE_LOOP)
        return -EIO;
    auc_i2c_write_buf[0] = 0xcd;

    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);


    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(fts_updateinfo_curr.delay_erase_flash);
    /*erase panel parameter area */
    auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(100);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");

    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH / 6 + 1);
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, temp + 6);
        msleep(20);
    }


    /*send the last six byte */
    for (i = 0; i < 6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = 1;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        packet_buf[6] = pbt_buf[dw_lenth + i];
        bt_ecc ^= packet_buf[6];
        fts_i2c_write(client, packet_buf, 7);
        msleep(20);
    }


    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);
        return -EIO;
    }

    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    return 0;
}
/************************************************************************
* Name: fts_5x26_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret=0;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {

        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);

        /*********Step 2:Enter upgrade mode and switch protocol*****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("failed writing  0x55 and 0xaa ! \n");
            continue;
        }

        /*********Step 3:check READ-ID***********************/
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP) return -EIO;
    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = 0x61;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    /*erase app area*/
    auc_i2c_write_buf[0] = 0x63;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    /*erase panel paramenter area*/
    auc_i2c_write_buf[0] = 0x04;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    /*erase panel paramenter area*/
    msleep(fts_updateinfo_curr.delay_erase_flash);
    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH / 6 + 1);
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp+6);
        msleep(20);
    }
    /*********Step 6: read out checksum***********************/
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = 0xcc;
    reg_val[0] = reg_val[1] = 0x00;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    FTS_DEBUG(KERN_WARNING "Checksum FT5X26:%X %X \n", reg_val[0], bt_ecc);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);
        return -EIO;
    }

    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);

    /********Step 8 Disable Write Flash*****/
    FTS_DEBUG("Step 8: Disable Write Flash\n");
    auc_i2c_write_buf[0] = 0x04;
    fts_i2c_write(client, auc_i2c_write_buf, 1);

    msleep(300);
    auc_i2c_write_buf[0] =auc_i2c_write_buf[1]= 0x00;
    fts_i2c_write(client,auc_i2c_write_buf,2);

    return 0;
}

/************************************************************************
* Name: fts_5x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u8 is_5336_new_bootloader = 0;
    u8 is_5336_fwsize_30 = 0;
    u32  packet_number;
    u32  j;
    u32  temp;
    u32  lenght;
    u8  packet_buf[FTS_PACKET_LENGTH + 6];
    u8      auc_i2c_write_buf[10];
    u8      bt_ecc;
    int i_ret;
    int fw_filenth = sizeof(CTPM_FW);

    if (CTPM_FW[fw_filenth-12] == 30)
    {
        is_5336_fwsize_30 = 1;
    }
    else
    {
        is_5336_fwsize_30 = 0;
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);

        /*write 0x55 to register FTS_RST_CMD_REG1*/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);


        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;

        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);

        /*********Step 3:check READ-ID***********************/
        msleep(fts_updateinfo_curr.delay_readid);
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAILD,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
            continue;
        }

    }

    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;

    auc_i2c_write_buf[0] = 0xcd;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    /*********20130705 mshl ********************/
    if (reg_val[0] <= 4)
    {
        is_5336_new_bootloader = BL_VERSION_LZ4 ;
    }
    else if (reg_val[0] == 7)
    {
        is_5336_new_bootloader = BL_VERSION_Z7 ;
    }
    else if (reg_val[0] >= 0x0f)
    {
        is_5336_new_bootloader = BL_VERSION_GZF ;
    }

    /*********Step 4:erase app and panel paramenter area ********************/
    if (is_5336_fwsize_30)
    {
        auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_erase_flash);

        auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(50);
    }
    else
    {
        auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_erase_flash);
    }

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;

    if (is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
    {
        dw_lenth = dw_lenth - 8;
    }
    else if (is_5336_new_bootloader == BL_VERSION_GZF)
    {
        dw_lenth = dw_lenth - 14;
    }
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;
    for (j=0; j<packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(lenght>>8);
        packet_buf[5] = (u8)lenght;

        for (i=0; i<FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH+6);
        msleep(FTS_PACKET_LENGTH/6 + 1);
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8)(temp>>8);
        packet_buf[3] = (u8)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8)(temp>>8);
        packet_buf[5] = (u8)temp;

        for (i=0; i<temp; i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        fts_i2c_write(client, packet_buf, temp+6);
        msleep(20);
    }
    /*send the last six byte*/
    if (is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
    {
        for (i = 0; i<6; i++)
        {
            if (is_5336_new_bootloader  == BL_VERSION_Z7)
            {
                temp = 0x7bfa + i;
            }
            else if (is_5336_new_bootloader == BL_VERSION_LZ4)
            {
                temp = 0x6ffa + i;
            }
            packet_buf[2] = (u8)(temp>>8);
            packet_buf[3] = (u8)temp;
            temp =1;
            packet_buf[4] = (u8)(temp>>8);
            packet_buf[5] = (u8)temp;
            packet_buf[6] = pbt_buf[ dw_lenth + i];
            bt_ecc ^= packet_buf[6];
            fts_i2c_write(client, packet_buf, 7);
            msleep(10);
        }
    }
    else if (is_5336_new_bootloader == BL_VERSION_GZF)
    {
        for (i = 0; i<12; i++)
        {
            if (is_5336_fwsize_30)
            {
                temp = 0x7ff4 + i;
            }
            else
            {
                temp = 0x7bf4 + i;
            }
            packet_buf[2] = (u8)(temp>>8);
            packet_buf[3] = (u8)temp;
            temp =1;
            packet_buf[4] = (u8)(temp>>8);
            packet_buf[5] = (u8)temp;
            packet_buf[6] = pbt_buf[ dw_lenth + i];
            bt_ecc ^= packet_buf[6];
            fts_i2c_write(client, packet_buf, 7);
            msleep(10);
        }
    }

    /*********Step 6: read out checksum***********************/
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
        return -EIO;
    }
    /*********Step 7: reset the new FW***********************/
    auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    return 0;
}
/************************************************************************
* Name: fts_5822_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5822_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    u8 bt_ecc_check;
    int i_ret;

    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("HidI2c change to StdI2c fail ! \n");
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        i_ret = fts_i2c_hid2std(client);
        if (i_ret == 0)
        {
            FTS_DEBUG("HidI2c change to StdI2c fail ! \n");
        }
        msleep(5);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("failed writing  0x55 and 0xaa ! \n");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = 0x61;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(1350);
    for (i = 0; i < 15; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
        if (0xF0 == reg_val[0] && 0xAA == reg_val[1])
        {
            break;
        }
        msleep(50);
    }
    FTS_DEBUG("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
    auc_i2c_write_buf[0] = 0xB0;
    auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
    auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
    auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
    fts_i2c_write(client, auc_i2c_write_buf, 4);
    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    bt_ecc_check = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;
        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc_check ^= pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        FTS_DEBUG("[FTS][%s] bt_ecc = %x \n", __func__, bt_ecc);
        if (bt_ecc != bt_ecc_check)
            FTS_DEBUG("[FTS][%s] Host checksum error bt_ecc_check = %x \n", __func__, bt_ecc_check);
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            msleep(1);
        }
    }
    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc_check ^= pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);
        FTS_DEBUG("[FTS][%s] bt_ecc = %x \n", __func__, bt_ecc);
        if (bt_ecc != bt_ecc_check)
            FTS_DEBUG("[FTS][%s] Host checksum error bt_ecc_check = %x \n", __func__, bt_ecc_check);
        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            msleep(1);
        }
    }
    msleep(50);
    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = 0x64;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);
    temp = 0;
    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);
    temp = dw_lenth;
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);
    for (i = 0; i < 100; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
        dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
        if (0xF0 == reg_val[0] && 0x55 == reg_val[1])
        {
            dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
            break;
        }
        msleep(1);
    }
    auc_i2c_write_buf[0] = 0x66;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
        return -EIO;
    }
    FTS_DEBUG(KERN_WARNING "checksum %X %X \n", reg_val[0], bt_ecc);
    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("HidI2c change to StdI2c fail ! \n");
    }
    return 0;
}

/************************************************************************
* Name: fts_5x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register FTS_RST_CMD_REG1 */
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);

        /*write 0x55 to register FTS_RST_CMD_REG1 */
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        do
        {
            i++;
            i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
            msleep(5);
        }
        while (i_ret <= 0 && i < 5);


        /*********Step 3:check READ-ID***********************/
        msleep(fts_updateinfo_curr.delay_readid);
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;
    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);    /*erase app area */
    msleep(fts_updateinfo_curr.delay_erase_flash);
    /*erase panel parameter area */
    auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(100);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;
    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;
        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH / 6 + 1);
    }
    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, temp + 6);
        msleep(20);
    }
    /*send the last six byte */
    for (i = 0; i < 6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = 1;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        packet_buf[6] = pbt_buf[dw_lenth + i];
        bt_ecc ^= packet_buf[6];
        fts_i2c_write(client, packet_buf, 7);
        msleep(20);
    }
    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);
        return -EIO;
    }
    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);    /*make sure CTP startup normally */
    return 0;
}

/************************************************************************
* Name: fts_5x46_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("[FTS] hid change to i2c fail ! \n");
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        /*write 0xaa to register FTS_RST_CMD_REG1 */
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);

        //write 0x55 to register FTS_RST_CMD_REG1
        fts_i2c_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        i_ret = fts_i2c_hid2std(client);

        if (i_ret == 0)
        {
            FTS_DEBUG("[FTS] hid change to i2c fail ! \n");
        }
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("[FTS] failed writing  0x55 and 0xaa ! \n");
            continue;
        }
        /*********Step 3:check READ-ID***********************/
        msleep(1);
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if( (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)  || g_tpd_forceUpgrade)
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);

            continue;
        }
    }
    if (i >= FTS_UPGRADE_LOOP )
        return -EIO;
    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(1350);
    for (i = 0; i < 15; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
        if (0xF0==reg_val[0] && 0xAA==reg_val[1])
        {
            break;
        }
        msleep(50);
    }
    FTS_DEBUG("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
    auc_i2c_write_buf[0] = 0xB0;
    auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
    auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
    auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
    fts_i2c_write(client, auc_i2c_write_buf, 4);
    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;
        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
        msleep(20);
        /*
        for(i = 0;i < 30;i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            msleep(1);
        }*/
    }
    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;
        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);
        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            FTS_DEBUG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
            msleep(1);

        }
    }

    msleep(50);

    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = 0x64;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    temp = 0;
    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);
    temp = dw_lenth;
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
        dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
        if (0xF0==reg_val[0] && 0x55==reg_val[1])
        {
            dev_err(&client->dev, "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
            break;
        }
        msleep(1);

    }
    auc_i2c_write_buf[0] = 0x66;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);

        return -EIO;
    }
    FTS_DEBUG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);
    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);
    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("HidI2c change to StdI2c fail ! \n");
    }
    return 0;
}

/************************************************************************
*   Name: fts_8606_writepram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8606_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{

    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    FTS_DEBUG("8606 dw_lenth= %d",dw_lenth);
    if (dw_lenth > 0x10000 || dw_lenth ==0)
    {
        return -EIO;
    }

    for (i = 0; i < 20; i++)
    {
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
        if (i_ret < 0)
        {
            FTS_DEBUG("[FTS] failed writing  0x55 ! \n");
            continue;
        }

        /*********Step 3:check READ-ID***********************/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
                                   0x00;
        reg_val[0] = reg_val[1] = 0x00;

        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if ((reg_val[0] == 0x86
             && reg_val[1] == 0x06) || (reg_val[0] == 0x86
                                        && reg_val[1] == 0x07))
        {
            msleep(50);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
                    reg_val[0], reg_val[1]);

            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP )
        return -EIO;

    /*********Step 4:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");
    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xae;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);
    }

    /*********Step 5: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = 0xcc;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);
        return -EIO;
    }
    FTS_DEBUG("checksum %X %X \n",reg_val[0],bt_ecc);
    FTS_DEBUG("Read flash and compare\n");

    msleep(50);

    /*********Step 6: start app***********************/
    FTS_DEBUG("Step 6: start app\n");
    auc_i2c_write_buf[0] = 0x08;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(20);

    return 0;
}

/************************************************************************
*   Name: fts_8606_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8606_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u8 reg_val_id[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;
    unsigned char cmd[20];
    unsigned char Checksum = 0;

    auc_i2c_write_buf[0] = 0x05;
    reg_val_id[0] = 0x00;

    i_ret =fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);
    if (dw_lenth == 0)
    {


        return -EIO;
    }

    if (0x81 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*60)
        {
            return -EIO;
        }
    }
    else if (0x80 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*64)
        {
            return -EIO;
        }
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG("failed writing  0x55 and 0xaa ! \n");
            continue;
        }

        /*********Step 3:check READ-ID***********************/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

        reg_val[0] = reg_val[1] = 0x00;

        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1
             && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)|| (reg_val[0] == 0x86 && reg_val[1] == 0xA6))
        {
            FTS_DEBUG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
                      reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
                    reg_val[0], reg_val[1]);

            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP )
        return -EIO;

    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");

    {
        cmd[0] = 0x05;
        cmd[1] = reg_val_id[0];//0x80;
        cmd[2] = 0x00;//???
        fts_i2c_write(client, cmd, 3);
    }

    {
        cmd[0] = 0x09;
        cmd[1] = 0x0B;
        fts_i2c_write(client, cmd, 2);
    }

    for (i=0; i<dw_lenth ; i++)
    {
        Checksum ^= pbt_buf[i];
    }
    msleep(50);

    auc_i2c_write_buf[0] = 0x61;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(1350);

    for (i = 0; i < 15; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0xAA==reg_val[1])
        {
            break;
        }
        msleep(50);
    }

    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");

    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;

    for (j = 0; j < packet_number; j++)
    {
        temp = 0x1000+j * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            msleep(1);

        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = 0x1000+packet_number * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            msleep(1);

        }
    }

    msleep(50);

    /*********Step 6: read out checksum***********************/
    /*send the opration head */
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = 0x64;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);
    temp = 0x1000+0;

    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);

    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;
    }
    else
    {
        temp = dw_lenth;
        FTS_DEBUG("Step 6_1: read out checksum\n");
    }
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0x55==reg_val[1])
        {
            break;
        }
        msleep(1);

    }
    //----------------------------------------------------------------------
    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;//??? 0x1000+LEN_FLASH_ECC_MAX
        auc_i2c_write_buf[0] = 0x65;
        auc_i2c_write_buf[1] = (u8)(temp >> 16);
        auc_i2c_write_buf[2] = (u8)(temp >> 8);
        auc_i2c_write_buf[3] = (u8)(temp);
        temp = dw_lenth-LEN_FLASH_ECC_MAX;
        auc_i2c_write_buf[4] = (u8)(temp >> 8);
        auc_i2c_write_buf[5] = (u8)(temp);
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);

        msleep(dw_lenth/256);

        for (i = 0; i < 100; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if (0xF0==reg_val[0] && 0x55==reg_val[1])
            {
                break;
            }
            msleep(1);

        }
    }
    auc_i2c_write_buf[0] = 0x66;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);

        return -EIO;
    }
    FTS_DEBUG("checksum %X %X \n",reg_val[0],bt_ecc);
    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);   //make sure CTP startup normally
    return 0;
}

/************************************************************************
* Name: fts_8716_ctpm_fw_write_pram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8607_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{

    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    FTS_FUNC_ENTER();

    /*check pramboot length*/
    FTS_DEBUG("8716 dw_lenth= %d",dw_lenth);
    if (dw_lenth > 0x10000 || dw_lenth ==0)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : pramboot is too loog. dw_lenth = 0x%x\n", __func__, dw_lenth);
        return -EIO;
    }

    for (i = 0; i < 20; i++)
    {
        /*send 0xAA 0x55 to FW, and start upgrade*/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);

        /*Enter upgrade mode*/
        i_ret = fts_i2c_hid2std(client);
        if (i_ret == 0)
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : hid change to i2c fail ! \n", __func__);
        }

        msleep(10);

        /*read ROM ID, check TP run in ROM or NOT*/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
        if (i_ret < 0)
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : failed writing 0x55 ! \n", __func__);
            continue;
        }

        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if (((reg_val[0] == 0x86) && (reg_val[1] == 0x07))) /*read ROM ID success, run in ROM*/
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : read rom boot id ok,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[UPGRADE]: << %s >> : read rom boot id fail,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP )/*read ROM ID fail, can not upgrade*/
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : read rom boot id: %d times, but fail,ID1 = 0x%x,ID2 = 0x%x\n", __func__, FTS_UPGRADE_LOOP, reg_val[0], reg_val[1]);
        return -EIO;
    }

    /*write pramboot to pram*/
    bt_ecc = 0;
    FTS_DEBUG("[UPGRADE]: << %s >> : write pramboot!!\n", __func__);

    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xae;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

#if FTS_UPGRADE_ERROR_TEST
        if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < ((FTS_UPGRADE_TEST_NUMBER/3)+1)))
        {
            uc_UpgradeReturnTime++;
            FTS_DEBUG("[UPGRADE]: << %s >> : write pram return for ERROR test!!\n", __func__);
            return -EIO;
        }
#endif
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);
    }

    /*read out checksum*/
    /* send the opration head */
    FTS_DEBUG("[UPGRADE]: << %s >> : read out checksum\n", __func__);
    auc_i2c_write_buf[0] = 0xcc;
    msleep(2);
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : checksum fail : FW=%02x bt_ecc=%02x\n", __func__, reg_val[0], bt_ecc);
        dev_err(&client->dev, "[UPGRADE]: << %s >> : ecc error! FW=%02x bt_ecc=%02x\n", __func__, reg_val[0], bt_ecc);
        return -EIO;
    }
    printk(KERN_WARNING "[UPGRADE]: << %s >> : checksum %X %X \n", __func__, reg_val[0], bt_ecc);
    msleep(100);

    /*start app*/
    FTS_DEBUG("[UPGRADE]: << %s >> : start app\n", __func__);
    auc_i2c_write_buf[0] = 0x08;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(20);

    FTS_FUNC_EXIT();

    return 0;
}

/************************************************************************
* Name: fts_8716_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8607_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u8 reg_val_id[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;
    unsigned char cmd[20];
    unsigned char Checksum = 0;

    FTS_FUNC_ENTER();

    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : hid change to i2c fail ! \n", __func__);
    }

    /*read flash id*/
    auc_i2c_write_buf[0] = 0x05;
    reg_val_id[0] = 0x00;
    i_ret =fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);

    /*check APP length by falsh type*/
    if (dw_lenth == 0)
    {
        return -EIO;
    }

    if (0x81 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*60)
        {
            return -EIO;
        }
    }
    else if (0x80 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*64)
        {
            return -EIO;
        }
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*send 0x55 0xAA to init the pramboot*/
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : write 0x55, 0xaa fail !!\n", __func__);
            continue;
        }

        /*read PRAM id, check run in PRAM or not*/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

        reg_val[0] = reg_val[1] = 0x00;

        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if ((reg_val[0] == 0x86) && (reg_val[1] == 0xA7))
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : read pram boot id ok,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[UPGRADE]: << %s >> : read pram boot id fail,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP )/*read PRAM id fail, can not upgrade*/
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : read pram boot id fail,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
        return -EIO;
    }

    /*set falsh clk*/
    {
        cmd[0] = 0x05;
        cmd[1] = reg_val_id[0];//0x80;
        cmd[2] = 0x00;
        fts_i2c_write(client, cmd, 3);
    }

    /*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
    {
        cmd[0] = 0x09;
        cmd[1] = 0x0B;
        fts_i2c_write(client, cmd, 2);
    }

    /*cal the check sum of app*/
    for (i=0; i<dw_lenth ; i++)
    {

        Checksum ^= pbt_buf[i];
    }
    msleep(50);

    /*start to erase the flash*/
    FTS_DEBUG("[UPGRADE]: << %s >> : erase flash\n", __func__);
    auc_i2c_write_buf[0] = 0x61;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(800);

    /*read the erase status*/
    for (i = 0; i < 1000; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0xAA==reg_val[1])
        {
            break;
        }
        msleep(50);
    }

#if FTS_UPGRADE_ERROR_TEST
    if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < (((2*FTS_UPGRADE_TEST_NUMBER)/3)+1)))
    {
        uc_UpgradeReturnTime++;
        msleep(50);
        FTS_DEBUG("[UPGRADE]: << %s >> : erase app return for ERROR test !!\n", __func__);
        return -EIO;
    }
#endif

    /*write firmware(FW) to flash*/
    bt_ecc = 0;
    FTS_DEBUG("[UPGRADE]: << %s >> : write app to flash\n", __func__);

    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    for (j = 0; j < packet_number; j++)
    {
        temp = 0x1000+j * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x20+0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            msleep(1);
        }

#if FTS_UPGRADE_ERROR_TEST
        if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < (FTS_UPGRADE_TEST_NUMBER+1)))
        {
            uc_UpgradeReturnTime++;
            msleep(50);
            FTS_DEBUG("[UPGRADE]: << %s >> : write flash return for ERROR test!!\n", __func__);
            return -EIO;
        }
#endif
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        j++;
        temp = 0x1000+packet_number * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x20+ 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                FTS_DEBUG("[UPGRADE]: << %s >> : write app to flash ok!!\n", __func__);
                break;
            }
            msleep(1);
        }
    }

    msleep(50);

    /*read out checksum*/
    /*send the opration head */
    FTS_DEBUG("[UPGRADE]: << %s >> : read out checksum\n", __func__);
    auc_i2c_write_buf[0] = 0x64;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    temp = 0x1000+0;


    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);

    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;
    }
    else
    {
        temp = dw_lenth;
    }
    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0x55==reg_val[1])
        {
            break;
        }
        msleep(1);

    }
    //----------------------------------------------------------------------
    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;
        auc_i2c_write_buf[0] = 0x65;
        auc_i2c_write_buf[1] = (u8)(temp >> 16);
        auc_i2c_write_buf[2] = (u8)(temp >> 8);
        auc_i2c_write_buf[3] = (u8)(temp);
        temp = dw_lenth-LEN_FLASH_ECC_MAX;
        auc_i2c_write_buf[4] = (u8)(temp >> 8);
        auc_i2c_write_buf[5] = (u8)(temp);
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);

        msleep(dw_lenth/256);

        for (i = 0; i < 100; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if (0xF0==reg_val[0] && 0x55==reg_val[1])
            {
                break;
            }
            msleep(1);

        }
    }
    auc_i2c_write_buf[0] = 0x66;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)     /*check sum fail, upgrade fail*/
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : ecc error: FW=%02x bt_ecc=%02x\n", __func__, reg_val[0], bt_ecc);
        dev_err(&client->dev, "[UPGRADE]: << %s >> : ecc error: FW=%02x bt_ecc=%02x\n", __func__, reg_val[0], bt_ecc);
        return -EIO;
    }
    printk(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);

    /*reset the new FW*/
    FTS_DEBUG("[UPGRADE]: << %s >> : reset the new FW\n", __func__);
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);

    FTS_FUNC_EXIT();

    return 0;
}

/************************************************************************
* Name: fts_8716_ctpm_fw_write_pram
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8716_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{

    u8 reg_val[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;

    FTS_FUNC_ENTER();

    /*check the length of the pramboot*/
    FTS_DEBUG("[UPGRADE]: << %s >> : %d  8716 dw_lenth= %d\n", __func__,  __LINE__, dw_lenth);
    if (dw_lenth > 0x10000 || dw_lenth ==0)
    {
        return -EIO;
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*send the soft upgrade commond to FW, and start upgrade*/
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, 0xfc, FTS_UPGRADE_55);
        msleep(200);

        /*Enter upgrade mode*/
        i_ret = fts_i2c_hid2std(client);
        if (i_ret == 0)
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : hid change to i2c fail ! \n",  __func__);
        }

        msleep(10);

        /*send 0x55 in time windows*/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
        if (i_ret < 0)
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : failed writing  0x55 ! \n", __func__);
            continue;
        }
        msleep(1);
        auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
        if (i_ret < 0)
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : failed writing  0xAA ! \n", __func__);
            continue;
        }

        /*read ROM ID, check run in ROM or not*/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if ((reg_val[0] == 0x87 && reg_val[1] == 0x16)) /*run in ROM*/
        {
            FTS_DEBUG("[UPGRADE]: << %s >> : READ CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[UPGRADE]: << %s >> : READ CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            FTS_DEBUG("[UPGRADE]: << %s >> : READ CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
            continue;
        }
    }

    if (i >= FTS_UPGRADE_LOOP )         /*not run in ROM, can not upgrade*/
        return -EIO;

    /*write pramboot to pram*/
    bt_ecc = 0;
    FTS_DEBUG("[UPGRADE]: << %s >> : write pramboot to pram!!\n", __func__);

    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xae;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
#if FTS_UPGRADE_ERROR_TEST
        if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < ((FTS_UPGRADE_TEST_NUMBER/3)+1)))
        {
            uc_UpgradeReturnTime++;
            FTS_DEBUG("[UPGRADE]: << %s >> : write pram return for ERROR test!!\n", __func__);
            return -EIO;
        }
#endif
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);
    }

    /*read out checksum*/
    /* send the opration head */
    FTS_DEBUG("[UPGRADE]: << %s >> : read out checksum\n", __func__);
    auc_i2c_write_buf[0] = 0xcc;
    msleep(2);
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);
        return -EIO;
    }
    FTS_DEBUG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);
    msleep(100);

    /*start pram*/
    FTS_DEBUG("[UPGRADE]: << %s >> : start app\n", __func__);
    auc_i2c_write_buf[0] = 0x08;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(20);

    FTS_FUNC_EXIT();

    return 0;
}

/************************************************************************
* Name: fts_8716_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_8716_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
    u8 reg_val[4] = {0};
    u8 reg_val_id[4] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;
    int i_ret;
    unsigned char cmd[20];
    unsigned char Checksum = 0;

    FTS_FUNC_ENTER();

    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG( "[UPGRADE]: << %s >> : hid change to i2c fail !!\n", __func__);
    }

    /*read flash ID*/
    auc_i2c_write_buf[0] = 0x05;
    reg_val_id[0] = 0x00;
    i_ret =fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);

    if (dw_lenth == 0)
    {
        return -EIO;
    }

    /*check the fw length by falsh type*/
    if (0x81 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*60)
        {
            return -EIO;
        }
    }
    else if (0x80 == (int)reg_val_id[0])
    {
        if (dw_lenth > 1024*64)
        {
            return -EIO;
        }
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*send 0x55 0xAA to init the pramboot*/
        msleep(10);
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        if (i_ret < 0)
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : write 0x55, 0xaa fail !!\n", __func__);
            continue;
        }

        /*read PRAM id, check run in PRAM or not*/
        msleep(1);
        auc_i2c_write_buf[0] = 0x90;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

        reg_val[0] = reg_val[1] = 0x00;

        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

        if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)/*|| (reg_val[0] == 0x87 && reg_val[1] == 0xA6)*/)
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : read PRAM ID OK:  0x%x,ID2 = 0x%x !!\n", __func__, reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : read PRAM ID ERROR:  0x%x,ID2 = 0x%x !!\n", __func__, reg_val[0], reg_val[1]);
            dev_err(&client->dev, "[FTS] Step 3: READ PRAM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
            continue;
        }
    }

    /*not run in PRAMBOOT, can not upgrade. return. upgrade fail*/
    if (i >= FTS_UPGRADE_LOOP )
        return -EIO;

    /*set flash clk*/
    {
        cmd[0] = 0x05;
        cmd[1] = reg_val_id[0];//0x80;
        cmd[2] = 0x00;
        fts_i2c_write(client, cmd, 3);
    }

    /*send upgrade type to reg 0x09: 0x0B: upgrade; 0x0A: download*/
    {
        cmd[0] = 0x09;
        cmd[1] = 0x0B;
        fts_i2c_write(client, cmd, 2);
    }

    /*get the check num of pramboot*/
    for (i=0; i<dw_lenth ; i++)
    {
        Checksum ^= pbt_buf[i];
    }
    msleep(50);

    /*erase the app*/
    auc_i2c_write_buf[0] = 0x61;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(1350);

    for (i = 0; i < 15; i++)
    {
        /*get the erase app status, if get 0xF0AA£¬erase flash success*/
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0xAA==reg_val[1]) /*erase flash success*/
        {
            break;
        }
        msleep(50);
    }

    if ((0xF0!=reg_val[0] || 0xAA!=reg_val[1]) && (i >= 15)) /*erase flash fail*/
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : erase app ERROR.reset tp and reload FW !!\n", __func__);
        return -EIO;
    }

#if FTS_UPGRADE_ERROR_TEST
    if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < (((2*FTS_UPGRADE_TEST_NUMBER)/3)+1)))
    {
        uc_UpgradeReturnTime++;
        msleep(50);
        FTS_DEBUG("[UPGRADE]: << %s >> : erase app return for ERROR test !!\n", __func__);
        return -EIO;
    }
#endif

    /*start to write app to flash*/
    bt_ecc = 0;
    FTS_DEBUG( "[UPGRADE]: << %s >> : write FW to ctpm flash !!\n", __func__);

    temp = 0;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    for (j = 0; j < packet_number; j++)
    {
        temp = 0x1000+j * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x20+0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            msleep(1);
        }
#if FTS_UPGRADE_ERROR_TEST
        if ((uc_AUOUpgradeTimes == 1) && (uc_UpgradeTimes < (FTS_UPGRADE_TEST_NUMBER+1)))
        {
            uc_UpgradeReturnTime++;
            msleep(50);
            FTS_DEBUG("[UPGRADE]: << %s >> : write flash return for ERROR test!!\n", __func__);
            return -EIO;
        }
#endif
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        j += 1;
        temp = 0x1000+packet_number * FTS_PACKET_LENGTH;
        packet_buf[1] = (u8) (temp >> 16);
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }
        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if ((j + 0x20+ 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
            {
                break;
            }
            msleep(1);
        }
    }

    msleep(50);

    /*read check sum*/
    /*send the opration head */
    FTS_DEBUG( "[UPGRADE]: << %s >> : read out checksum !!\n", __func__);

    auc_i2c_write_buf[0] = 0x64;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    temp = 0x1000+0;


    auc_i2c_write_buf[0] = 0x65;
    auc_i2c_write_buf[1] = (u8)(temp >> 16);
    auc_i2c_write_buf[2] = (u8)(temp >> 8);
    auc_i2c_write_buf[3] = (u8)(temp);

    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;
    }
    else
    {
        temp = dw_lenth;
    }

    auc_i2c_write_buf[4] = (u8)(temp >> 8);
    auc_i2c_write_buf[5] = (u8)(temp);
    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
    msleep(dw_lenth/256);

    for (i = 0; i < 100; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        reg_val[0] = reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

        if (0xF0==reg_val[0] && 0x55==reg_val[1])
        {
            break;
        }
        msleep(1);

    }
    //----------------------------------------------------------------------
    if (dw_lenth > LEN_FLASH_ECC_MAX)
    {
        temp = LEN_FLASH_ECC_MAX;
        auc_i2c_write_buf[0] = 0x65;
        auc_i2c_write_buf[1] = (u8)(temp >> 16);
        auc_i2c_write_buf[2] = (u8)(temp >> 8);
        auc_i2c_write_buf[3] = (u8)(temp);
        temp = dw_lenth-LEN_FLASH_ECC_MAX;
        auc_i2c_write_buf[4] = (u8)(temp >> 8);
        auc_i2c_write_buf[5] = (u8)(temp);
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);

        msleep(dw_lenth/256);

        for (i = 0; i < 100; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            reg_val[0] = reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

            if (0xF0==reg_val[0] && 0x55==reg_val[1])
            {
                break;
            }
            msleep(1);

        }
    }
    auc_i2c_write_buf[0] = 0x66;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)     /*if check sum fail, upgrade fail*/
    {
        FTS_DEBUG( "[UPGRADE]: << %s >> : ecc error! FW=%02x bt_ecc=%02x !!\n", __func__, reg_val[0],bt_ecc);
        dev_err(&client->dev, "[UPGRADE]: << %s >> : ecc error! FW=%02x bt_ecc=%02x\n", __func__, reg_val[0],bt_ecc);
        return -EIO;
    }
    FTS_DEBUG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);

    /*upgrade success, reset the FW*/
    FTS_DEBUG( "[UPGRADE]: << %s >> : reset the new FW !!\n", __func__);

    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(200);

    FTS_FUNC_EXIT();

    return 0;
}

/************************************************************************
* Name: fts_3x07_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_3x07_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
    u8 reg_val[2] = {0};
    u32 i = 0;
    u32 packet_number;
    u32 j;
    u32 temp;
    u32 lenght;
    u32 fw_length;
    u8 packet_buf[FTS_PACKET_LENGTH + 6];
    u8 auc_i2c_write_buf[10];
    u8 bt_ecc;

    if (pbt_buf[0] != 0x02)
    {
        FTS_DEBUG("[FTS] FW first byte is not 0x02. so it is invalid \n");
        return -1;
    }

    if (dw_lenth > 0x11f)
    {
        fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
        if (dw_lenth < fw_length)
        {
            FTS_DEBUG("[FTS] Fw length is invalid \n");
            return -1;
        }
    }
    else
    {
        FTS_DEBUG("[FTS] Fw length is invalid \n");
        return -1;
    }

    for (i = 0; i < FTS_UPGRADE_LOOP; i++)
    {
        /*********Step 1:Reset  CTPM *****/
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
        msleep(fts_updateinfo_curr.delay_aa);
        fts_i2c_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);
        msleep(fts_updateinfo_curr.delay_55);
        /*********Step 2:Enter upgrade mode *****/
        auc_i2c_write_buf[0] = FTS_UPGRADE_55;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
        fts_i2c_write(client, auc_i2c_write_buf, 1);
        msleep(fts_updateinfo_curr.delay_readid);
        /*********Step 3:check READ-ID***********************/
        auc_i2c_write_buf[0] = FTS_READ_ID_REG;
        auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


        if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
            && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        {
            FTS_DEBUG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
                      reg_val[0], reg_val[1]);
            break;
        }
        else
        {
            dev_err(&client->dev, "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
                    reg_val[0], reg_val[1]);
        }
    }
    if (i >= FTS_UPGRADE_LOOP)
        return -EIO;

    auc_i2c_write_buf[0] = FTS_READ_ID_REG;
    auc_i2c_write_buf[1] = 0x00;
    auc_i2c_write_buf[2] = 0x00;
    auc_i2c_write_buf[3] = 0x00;
    auc_i2c_write_buf[4] = 0x00;
    fts_i2c_write(client, auc_i2c_write_buf, 5);

    /*Step 4:erase app and panel paramenter area*/
    FTS_DEBUG("Step 4:erase app and panel paramenter area\n");
    auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(fts_updateinfo_curr.delay_erase_flash);

    for (i = 0; i < 200; i++)
    {
        auc_i2c_write_buf[0] = 0x6a;
        auc_i2c_write_buf[1] = 0x00;
        auc_i2c_write_buf[2] = 0x00;
        auc_i2c_write_buf[3] = 0x00;
        reg_val[0] = 0x00;
        reg_val[1] = 0x00;
        fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
        if (0xb0 == reg_val[0] && 0x02 == reg_val[1])
        {
            FTS_DEBUG("[FTS] erase app finished \n");
            break;
        }
        msleep(50);
    }

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    FTS_DEBUG("Step 5:write firmware(FW) to ctpm flash\n");

    dw_lenth = fw_length;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = FTS_FW_WRITE_CMD;
    packet_buf[1] = 0x00;

    for (j = 0; j < packet_number; j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (lenght >> 8);
        packet_buf[5] = (u8) lenght;

        for (i = 0; i < FTS_PACKET_LENGTH; i++)
        {
            packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (u8) (temp >> 8);
        packet_buf[3] = (u8) temp;
        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (u8) (temp >> 8);
        packet_buf[5] = (u8) temp;

        for (i = 0; i < temp; i++)
        {
            packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6 + i];
        }

        fts_i2c_write(client, packet_buf, temp + 6);

        for (i = 0; i < 30; i++)
        {
            auc_i2c_write_buf[0] = 0x6a;
            auc_i2c_write_buf[1] = 0x00;
            auc_i2c_write_buf[2] = 0x00;
            auc_i2c_write_buf[3] = 0x00;
            reg_val[0] = 0x00;
            reg_val[1] = 0x00;
            fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
            if (0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
            {
                FTS_DEBUG("[FTS] write a block data finished \n");
                break;
            }
            msleep(1);
        }
    }


    /*********Step 6: read out checksum***********************/
    FTS_DEBUG("Step 6: read out checksum\n");
    auc_i2c_write_buf[0] = FTS_REG_ECC;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    if (reg_val[0] != bt_ecc)
    {
        dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
                reg_val[0],
                bt_ecc);
        return -EIO;
    }

    /*********Step 7: reset the new FW***********************/
    FTS_DEBUG("Step 7: reset the new FW\n");
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(300);

    return 0;
}
/*
*note:the firmware default path is sdcard.
    if you want to change the dir, please modify by yourself.
*/
/************************************************************************
* Name: fts_GetFirmwareSize
* Brief:  get file size
* Input: file name
* Output: no
* Return: file size
***********************************************************************/
static int fts_GetFirmwareSize(char *firmware_name)
{
    struct file *pfile = NULL;
    struct inode *inode;
    unsigned long magic;
    off_t fsize = 0;
    char filepath[128];

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
    if (NULL == pfile)
    {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile))
    {
        pr_err("error occured while opening file %s.\n", filepath);
        return -EIO;
    }
    //10190256
    inode = pfile->f_path.dentry->d_inode;
    magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    filp_close(pfile, NULL);
    return fsize;
}

/************************************************************************
* Name: fts_ReadFirmware
* Brief:  read firmware buf for .bin file.
* Input: file name, data buf
* Output: data buf
* Return: 0
***********************************************************************/
/*
note:the firmware default path is sdcard.
    if you want to change the dir, please modify by yourself.
*/
static int fts_ReadFirmware(char *firmware_name,unsigned char *firmware_buf)
{
    struct file *pfile = NULL;
    struct inode *inode;
    unsigned long magic;
    off_t fsize;
    char filepath[128];
    loff_t pos;
    mm_segment_t old_fs;

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
    if (NULL == pfile)
    {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile))
    {
        pr_err("error occured while opening file %s.\n", filepath);
        return -EIO;
    }
    inode = pfile->f_path.dentry->d_inode;
    magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(pfile, firmware_buf, fsize, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);
    return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_app_file
* Brief:  upgrade with *.bin file
* Input: i2c info, file name
* Output: no
* Return: success =0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name)
{
    u8 *pbt_buf = NULL;
    int i_ret=0;
    int fwsize = fts_GetFirmwareSize(firmware_name);
    FTS_DEBUG("**********fts_ctpm_fw_upgrade_with_app_file start****************\n");

    if (fwsize <= 0)
    {
        dev_err(&client->dev, "%s ERROR:Get firmware size failed\n",__func__);
        return -EIO;
    }
    if (fwsize < 8 || fwsize > 60 * 1024)
    {
        dev_err(&client->dev, "FW length error\n");
        return -EIO;
    }
    /*=========FW upgrade========================*/
    pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_ATOMIC);
    if (fts_ReadFirmware(firmware_name, pbt_buf))
    {
        dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n",__func__);
        kfree(pbt_buf);
        return -EIO;
    }
    if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a))
    {
        i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14))
    {
        i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x06))
    {
        i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x36))
    {
        i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x64))
    {
        i_ret = fts_6336GU_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x54))
    {
        i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x58))
    {
        i_ret =  fts_5822_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x59))
    {
        i_ret =  fts_5x26_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x86))
    {
        /*call the upgrade function*/
        i_ret = fts_8606_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));

        if (i_ret != 0)
        {
            dev_err(&client->dev, "%s:fts_8606_writepram failed. err.\n",__func__);
            return -EIO;
        }

        i_ret =  fts_8606_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x87))
    {
        /*call the upgrade function*/
        i_ret = fts_8716_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));

        if (i_ret != 0)
        {
            dev_err(&client->dev, "%s:fts_8716_writepram failed. err.\n",__func__);
            return -EIO;
        }
        i_ret =  fts_8716_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x0E))
    {
        i_ret = fts_3x07_ctpm_fw_upgrade(client, pbt_buf, fwsize);
    }
    if (i_ret != 0)
        dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
                __func__);
    else if (fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
    {
        fts_ctpm_auto_clb(client);
    }

    kfree(pbt_buf);

    return i_ret;
}
/************************************************************************
* Name: fts_ctpm_get_i_file_ver
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_ctpm_get_i_file_ver(void)
{
    u16 ui_sz;
    ui_sz = fw_size;
    if (ui_sz > 2)
    {
        if (fts_updateinfo_curr.CHIP_ID==0x36 || fts_updateinfo_curr.CHIP_ID==0x64)
            return CTPM_FW[0x10A];
        else if (fts_updateinfo_curr.CHIP_ID==0x87 || fts_updateinfo_curr.CHIP_ID==0x86)
            return CTPM_FW[0x10E];
        else if (fts_updateinfo_curr.CHIP_ID==0x58)
            return CTPM_FW[0x1D0A];
        else
            return CTPM_FW[ui_sz - 2];
    }

    return 0x00;
}
/************************************************************************
* Name: fts_ctpm_update_project_setting
* Brief:  update project setting, only update these settings for COB project, or for some special case
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
    u8 uc_i2c_addr;
    u8 uc_io_voltage;
    u8 uc_panel_factory_id;
    u8 buf[FTS_SETTING_BUF_LEN];
    u8 reg_val[2] = {0};
    u8 auc_i2c_write_buf[10] = {0};
    u8 packet_buf[FTS_SETTING_BUF_LEN + 6];
    u32 i = 0;
    int i_ret;

    uc_i2c_addr = client->addr;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;


    /*Step 1:Reset  CTPM*/
    if (fts_updateinfo_curr.CHIP_ID==0x06 || fts_updateinfo_curr.CHIP_ID==0x36)
    {
        fts_i2c_write_reg(client, 0xbc, 0xaa);
    }
    else
    {
        fts_i2c_write_reg(client, 0xfc, 0xaa);
    }
    msleep(50);

    /*write 0x55 to register 0xfc */
    if (fts_updateinfo_curr.CHIP_ID==0x06 || fts_updateinfo_curr.CHIP_ID==0x36)
    {
        fts_i2c_write_reg(client, 0xbc, 0x55);
    }
    else
    {
        fts_i2c_write_reg(client, 0xfc, 0x55);
    }
    msleep(30);

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i++;
        i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
        msleep(5);
    }
    while (i_ret <= 0 && i < 5);


    /*********Step 3:check READ-ID***********************/
    auc_i2c_write_buf[0] = 0x90;
    auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
                               0x00;

    fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

    if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
        dev_dbg(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
    else
        return -EIO;

    auc_i2c_write_buf[0] = 0xcd;
    fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
    dev_dbg(&client->dev, "bootloader version = 0x%x\n", reg_val[0]);

    /*--------- read current project setting  ---------- */
    /*set read start address */
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;

    fts_i2c_read(client, buf, 4, buf, FTS_SETTING_BUF_LEN);
    dev_dbg(&client->dev, "[FTS] old setting: uc_i2c_addr = 0x%x,\
            uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
            buf[0], buf[2], buf[4]);

    /*--------- Step 4:erase project setting --------------*/
    auc_i2c_write_buf[0] = 0x63;
    fts_i2c_write(client, auc_i2c_write_buf, 1);
    msleep(100);

    /*----------  Set new settings ---------------*/
    buf[0] = uc_i2c_addr;
    buf[1] = ~uc_i2c_addr;
    buf[2] = uc_io_voltage;
    buf[3] = ~uc_io_voltage;
    buf[4] = uc_panel_factory_id;
    buf[5] = ~uc_panel_factory_id;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    packet_buf[2] = 0x78;
    packet_buf[3] = 0x0;
    packet_buf[4] = 0;
    packet_buf[5] = FTS_SETTING_BUF_LEN;

    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
        packet_buf[6 + i] = buf[i];

    fts_i2c_write(client, packet_buf, FTS_SETTING_BUF_LEN + 6);
    msleep(100);

    /********* reset the new FW***********************/
    auc_i2c_write_buf[0] = 0x07;
    fts_i2c_write(client, auc_i2c_write_buf, 1);

    msleep(200);
    return 0;
}
/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_i_file
* Brief:  upgrade with *.i file
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
    u8 *pbt_buf = NULL;
    int i_ret=0;
    int fw_len = fw_size;

    FTS_FUNC_ENTER();

    /*judge the fw that will be upgraded
    * if illegal, then stop upgrade and return.
    */
    if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)
        ||(fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x06) ||(fts_updateinfo_curr.CHIP_ID==0x0a) ||(fts_updateinfo_curr.CHIP_ID==0x08))
    {
        if (fw_len < 8 || fw_len > 32 * 1024)
        {
            dev_err(&client->dev, "%s:FW length error\n", __func__);
            return -EIO;
        }

        if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
            && (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
            && (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF)
        {
            /*FW upgrade */
            pbt_buf = CTPM_FW;
            /*call the upgrade function */
            if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a))
            {
                i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
            }
            else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14))
            {
                i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
            }
            else if ((fts_updateinfo_curr.CHIP_ID==0x06))
            {
                i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
            }
            if (i_ret != 0)
                dev_err(&client->dev, "%s:upgrade failed. err.\n",__func__);
            else if (fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
            {
                fts_ctpm_auto_clb(client);
            }
        }
        else
        {
            dev_err(&client->dev, "%s:FW format error\n", __func__);
            return -EIO;
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x36))
    {
        if (fw_len < 8 || fw_len > 32 * 1024)
        {
            dev_err(&client->dev, "%s:FW length error\n", __func__);
            return -EIO;
        }
        pbt_buf = CTPM_FW;
        i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
            dev_err(&client->dev, "%s:upgrade failed. err.\n",__func__);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x64))
    {
        if (fw_len < 8 || fw_len > 48 * 1024)
        {
            dev_err(&client->dev, "%s:FW length error\n", __func__);
            return -EIO;
        }
        pbt_buf = CTPM_FW;
        i_ret = fts_6336GU_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
            dev_err(&client->dev, "%s:upgrade failed. err.\n",__func__);
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x54))
    {
        if (fw_len < 8 || fw_len > 54 * 1024)
        {
            pr_err("FW length error\n");
            return -EIO;
        }
        /*FW upgrade*/
        pbt_buf = CTPM_FW;
        /*call the upgrade function*/
        i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf,fw_size);  
        if (i_ret != 0)
        {
            dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
        }
        else
        {
#if FTS_AUTO_CLB_EN
            fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x58))
    {
        if (fw_len < 8 || fw_len > 54*1024)
        {
            pr_err("FW length error\n");
            return -EIO;
        }

        /*FW upgrade*/
        pbt_buf = CTPM_FW;
        /*call the upgrade function*/
        i_ret = fts_5822_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
        {
            dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
        }
        else
        {
#if FTS_AUTO_CLB_EN
            fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x59))
    {
        if (fw_len < 8 || fw_len > 54*1024)
        {
            pr_err("FW length error\n");
            return -EIO;
        }

        /*FW upgrade*/
        pbt_buf = CTPM_FW;
        /*call the upgrade function*/
        i_ret = fts_5x26_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
        {
            dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
        }
        else
        {
#if FTS_AUTO_CLB_EN
            fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x86))   /* ft8607 upgrade function */
    {
        /*FW upgrade*/
        pbt_buf = CTPM_FW;
        /*call the upgrade function*/
        i_ret = fts_8607_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));

        if (i_ret != 0)
        {
            dev_err(&client->dev, "%s:upgrade failed. err.\n",__func__);
            return -EIO;
        }

        i_ret =  fts_8607_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));

        if (i_ret != 0)
        {
            dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
        }
        else
        {
#if FTS_AUTO_CLB_EN
            fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x87))   /* ft8716 upgrade function */
    {
        /*app buf*/
        pbt_buf = CTPM_FW;

        FTS_DEBUG( "[UPGRADE]: << %s >> : write pram now!!\n", __func__);
        /*call the write pramboot function*/
        i_ret = fts_8716_writepram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
        if (i_ret != 0)
        {
            dev_err(&client->dev, "[UPGRADE]: << %s >> : write pram error!!\n",__func__);
            FTS_DEBUG( "[UPGRADE]: << %s >> : write pram error!!\n", __func__);
            return -EIO;
        }

        /*call the write fw upgrade function*/
        i_ret =  fts_8716_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
        {
            dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
        }
        else
        {
#if FTS_AUTO_CLB_EN
            fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
        }
    }
    else if ((fts_updateinfo_curr.CHIP_ID==0x0E))
    {
        if (fw_len < 8 || fw_len > 32 * 1024)
        {
            dev_err(&client->dev, "%s:FW length error\n", __func__);
            return -EIO;
        }
        pbt_buf = CTPM_FW;
        i_ret = fts_3x07_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
        if (i_ret != 0)
            dev_err(&client->dev, "%s:upgrade failed. err.\n",__func__);
    }
    FTS_FUNC_EXIT();

    return i_ret;
}


/************************************************************************
* Name: ft8716_ctpm_VidFWid_get_from_boot
* Brief:  read chip ID
* Input: i2c info
* Output: chip ID
* Return: fail <0
***********************************************************************/
static unsigned char fts_flash_get_pram_or_rom_id(struct i2c_client *client)
{
    unsigned char auc_i2c_write_buf[4];
    unsigned char reg_val[2] = {0};
    unsigned char inRomBoot = 0x00;  //0x01 : run in rom boot;  0x02 : run in pram boot
    int i_ret;

    i_ret = fts_i2c_hid2std(client);
    if (i_ret == 0)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : hidi2c change to stdi2c fail ! \n", __func__);
    }
    auc_i2c_write_buf[0] = FTS_UPGRADE_55;
    fts_i2c_write(fts_i2c_client, auc_i2c_write_buf, 1);
    msleep(fts_updateinfo_curr.delay_readid);

    auc_i2c_write_buf[0] = 0x90;
    auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
    fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

    if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
    {
        inRomBoot = FTS_RUN_IN_PRAM;
        FTS_DEBUG("[UPGRADE]: << %s >> : read pram boot id for success , pram boot id is: ID1 = 0x%x,ID2 = 0x%x:\n", __func__, reg_val[0], reg_val[1]);
    }
    else if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] != fts_updateinfo_curr.upgrade_id_2)
    {
        inRomBoot = FTS_RUN_IN_ROM;
        FTS_DEBUG("[UPGRADE]: << %s >> : read rom boot id for success , rom boot id is: ID1 = 0x%x,ID2 = 0x%x:\n", __func__, reg_val[0], reg_val[1]);
    }
    else
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : read pram boot id for test error: pram boot id is: ID1 = 0x%x,ID2 = 0x%x\n", __func__, reg_val[0], reg_val[1]);
    }

    return inRomBoot;
}

u8 fts_check_fw_status(struct i2c_client *client)
{
    u8 uc_chip_id = 0;
    u8 uc_tp_vendor_id;
    u8 fw_status = 0;
    u8 i = 0;
    int fwver_in_chip = 0, module_id_in_chip = 0, chip_num_in_chip = 0;
    //char * tp_name;
    unsigned char inRomBoot = 0x00;  //0x01 : run in rom boot;  0x02 : run in pram boot

    FTS_FUNC_ENTER();

    for (i=0; i<5; i++)
    {
        fts_i2c_read_reg(client, FTS_REG_VENDOR_ID, &uc_tp_vendor_id);
        fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &uc_chip_id);

        if ((uc_chip_id != fts_updateinfo_curr.CHIP_ID) ||
            ((uc_tp_vendor_id != FTS_VENDOR_1_ID)|| (uc_tp_vendor_id != FTS_VENDOR_2_ID)||(uc_tp_vendor_id != FTS_VENDOR_3_ID)||(uc_tp_vendor_id != FTS_VENDOR_4_ID)))
        {
            continue;
        }
        else
        {
            break;
        }
    }
    printk("[UPGRADE]: << %s >> : uc_tp_vendor_id = %d, uc_chip_id = %d \n", __func__, uc_tp_vendor_id, uc_chip_id);
   if (uc_tp_vendor_id == 0xEF||uc_tp_vendor_id == 0xFF||uc_tp_vendor_id == 0x00){
   	
   	  tpd_get_chipfw_version_etc(client, &chip_num_in_chip, &fwver_in_chip, &module_id_in_chip);
	  
         fts_updateinfo_curr.CHIP_ID = chip_num_in_chip; // modify by zhangqiyi      
         uc_chip_id = chip_num_in_chip;
         uc_tp_vendor_id = module_id_in_chip;	 
	  g_tpd_forceUpgrade = 1 ;
	
   }

	switch(uc_tp_vendor_id)
	{

		case FTS_VENDOR_1_ID:
			CTPM_FW = CTPM_FW1;
			fw_size = sizeof(CTPM_FW1);
			printk("[UPGRADE]tpd: << %s >> : skyworth\n", __func__);
			break;

		case FTS_VENDOR_2_ID:
			CTPM_FW = CTPM_FW2;
			fw_size = sizeof(CTPM_FW2);
			printk("[UPGRADE]tpd: << %s >> : lead\n", __func__);
			break;

		case FTS_VENDOR_3_ID:
			CTPM_FW = CTPM_FW3;
			fw_size = sizeof(CTPM_FW3);
			printk("[UPGRADE]tpd: << %s >> :lcetron\n", __func__);
			break;

		case FTS_VENDOR_4_ID:
			CTPM_FW = CTPM_FW4;
			fw_size = sizeof(CTPM_FW4);
			printk("[UPGRADE]tpd: << %s >> :dijing\n", __func__);
			break;

		case FTS_VENDOR_5_ID:
			CTPM_FW = CTPM_FW5;
			fw_size = sizeof(CTPM_FW5);
			printk("[UPGRADE]tpd: << %s >> : helitai\n", __func__);
			break;

		default:
			printk("[UPGRADE]tpd:<< %s >> Vendor id check failed", __func__);
			break;

	}
		


	

    if ((uc_chip_id == fts_updateinfo_curr.CHIP_ID) &&
        ((uc_tp_vendor_id == FTS_VENDOR_1_ID)|| (uc_tp_vendor_id == FTS_VENDOR_2_ID)||(uc_tp_vendor_id != FTS_VENDOR_3_ID)||(uc_tp_vendor_id != FTS_VENDOR_4_ID)))//call fts_flash_get_upgrade_info in probe function firstly.
    {
        fw_status = FTS_RUN_IN_APP;
        FTS_DEBUG("[UPGRADE]: << %s >> : APP OK!! \n", __func__);
    }
    else if ((uc_tp_vendor_id == FTS_VENDOR_1_ID)|| (uc_tp_vendor_id == FTS_VENDOR_2_ID)||(uc_tp_vendor_id != FTS_VENDOR_3_ID)||(uc_tp_vendor_id != FTS_VENDOR_4_ID))
    {
        inRomBoot = fts_flash_get_pram_or_rom_id(client);
        FTS_DEBUG( "[UPGRADE]: << %s >> : inRomBoot = %d\n", __func__, inRomBoot);
        if (inRomBoot == FTS_RUN_IN_ROM)
        {
            fw_status = FTS_RUN_IN_ROM;
            FTS_DEBUG( "[UPGRADE]: << %s >> : run in rom\n", __func__);
        }
        else
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : not run in rom\n", __func__);
        }

        FTS_DEBUG("[UPGRADE]: << %s >> : APP invalid!! \n", __func__);
    }
    else
    {
        inRomBoot = fts_flash_get_pram_or_rom_id(client);
        FTS_DEBUG( "[UPGRADE]: << %s >> : inRomBoot = %d\n", __func__, inRomBoot);
        if (inRomBoot == FTS_RUN_IN_PRAM)
        {
            fw_status = FTS_RUN_IN_PRAM;
            FTS_DEBUG( "[UPGRADE]: << %s >> : run in pram\n", __func__);
        }
        else
        {
            FTS_DEBUG( "[UPGRADE]: << %s >> : not run in pram\n", __func__);
        }

        FTS_DEBUG("[UPGRADE]: << %s >> : APP invalid!! \n", __func__);
    }

    FTS_FUNC_EXIT();

    return fw_status;
}

bool fts_check_need_upgrade(struct i2c_client *client)
{
    u8 i = 0;
    u8 fw_status = 0;
    bool bUpgradeFlag = false;
    u8 uc_tp_fm_ver;
    u8 uc_host_fm_ver = 0;

    FTS_FUNC_ENTER();

    for (i=0; i<5; i++)
    {
        fw_status = fts_check_fw_status(client);
        if (!fw_status)
        {
            msleep(5);
        }
        else
        {
            break;
        }
        FTS_DEBUG( "[UPGRADE]: << %s >> : fw_status = %d\n", __func__, fw_status);
    }

    if (fw_status == FTS_RUN_IN_APP) //call fts_flash_get_upgrade_info in probe function firstly.
    {
        fts_i2c_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);

	if(uc_tp_fm_ver == 0xEF||uc_tp_fm_ver == 0xFF)
           uc_tp_fm_ver = 0;
	
        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
        printk("[UPGRADE]tpd: << %s >> : uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", __func__, uc_tp_fm_ver, uc_host_fm_ver);
        if (uc_tp_fm_ver < uc_host_fm_ver )
        {
            bUpgradeFlag = true;
        }
    }
    else if (fw_status == FTS_RUN_IN_ROM)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : run in rom!!\n", __func__);
        bUpgradeFlag = true;
    }
    else if (fw_status == FTS_RUN_IN_PRAM)
    {
        FTS_DEBUG("[UPGRADE]: << %s >> : run in pram, reset and upgrade!!\n", __func__);
        fts_rom_or_pram_reset(client);
        bUpgradeFlag = true;
    }

    FTS_FUNC_EXIT();

    return bUpgradeFlag;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
    u8 uc_tp_fm_ver;
    int i_ret;
    bool bUpgradeFlag = false;
    u8 uc_upgrade_times = 0;

    FTS_FUNC_ENTER();

    /* check need upgrade or not */
    bUpgradeFlag = fts_check_need_upgrade(client);
    printk("[UPGRADE]tpd: << %s >> : bUpgradeFlag = 0x%x\n", __func__, bUpgradeFlag);

    /* pingpang test need upgrade */
#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
    bUpgradeFlag = true;
#endif

    /* will upgrade */
    if (bUpgradeFlag)
    {
        do
        {
            uc_upgrade_times++;

#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
            uc_AUOUpgradeTimes = uc_upgrade_times;
#endif

            if (bUpgradeFlag)
            {
                /* esd check */
#if FTS_ESDCHECK_EN
                fts_esdcheck_switch(DISABLE);
#endif

                /* upgrade with i file */
                i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
#if FTS_ESDCHECK_EN
                fts_esdcheck_switch(ENABLE);
#endif
                if (i_ret == 0)    /* upgrade success */
                {
                    msleep(300);
                    fts_i2c_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
                    FTS_DEBUG("[UPGRADE]: << %s >> : upgrade to new version 0x%x\n", __func__, uc_tp_fm_ver);
                }
                else /* upgrade fail */
                {
                    /* if upgrade fail, reset to run ROM. if app in flash is ok. TP will work success */
                    fts_rom_or_pram_reset(client);
                }
            }
        }
        while ((i_ret != 0) && (uc_upgrade_times < 2));  /* if upgrade fail, upgrade again. then return */
    }

    FTS_FUNC_EXIT();
    return 0;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade_test
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
int fts_ctpm_auto_upgrade_test(struct i2c_client *client)
{
    int i_ret;
    static int uc_ErrorTimes = 0;
#if FTS_GET_UPGRADE_TIME
    struct timeval tpstart,tpend;
    int timeuse;
#endif
    wake_lock(&ps_lock);

    do
    {
        uc_UpgradeTimes++;
        FTS_DEBUG( "[UPGRADE]:##############################################################################\n");
        FTS_DEBUG( "[UPGRADE]: << %s >> : start to upgrade %d times !!\n", __func__, uc_UpgradeTimes);
        FTS_DEBUG( "[UPGRADE]:##############################################################################\n");

#if FTS_GET_UPGRADE_TIME
        do_gettimeofday(&tpstart);
#endif
        i_ret = fts_ctpm_auto_upgrade(client);
        if (i_ret == 0)
        {
#if FTS_GET_UPGRADE_TIME
            do_gettimeofday(&tpend);
            timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+ tpend.tv_usec-tpstart.tv_usec;
            timeuse/=1000000;
            FTS_DEBUG( "[UPGRADE]: << %s >> : upgrade ### SUCCESS ###: Use time: %d Seconds!!\n", __func__, timeuse);
#endif
        }
        else
        {
            uc_ErrorTimes++;
        }

#if (FTS_UPGRADE_PINGPONG_TEST ||FTS_UPGRADE_ERROR_TEST)
        FTS_DEBUG( "[UPGRADE]:##############################################################################\n");
        FTS_DEBUG( "[UPGRADE]: << %s >> : upgrade %d times, return %d times, error %d times !!\n", __func__, uc_UpgradeTimes, uc_UpgradeReturnTime, uc_ErrorTimes);
        FTS_DEBUG( "[UPGRADE]:##############################################################################\n\n\n");
#endif
    }
#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
    while (uc_UpgradeTimes < (FTS_UPGRADE_TEST_NUMBER));
#endif
    wake_unlock(&ps_lock);

    return 0;
}
#endif

#if  FTS_AUTO_UPGRADE_EN
static void fts_fw_update_work_func(struct work_struct *work)
{
    printk("********************FTS Enter CTP Auto Upgrade ********************\n");
    disable_irq(ft_touch_irq);

    is_update = true ;
#if (FTS_UPGRADE_PINGPONG_TEST || FTS_UPGRADE_ERROR_TEST)
    fts_ctpm_auto_upgrade_test(fts_i2c_client);
#else
    fts_ctpm_auto_upgrade(fts_i2c_client);
#endif
    is_update = false;
    enable_irq(ft_touch_irq);
}

int fts_workqueue_init(void)
{

    touch_wq = create_singlethread_workqueue("touch_wq");
    if (touch_wq)
    {
        INIT_WORK(&fw_update_work, fts_fw_update_work_func);
    }
    else
    {
        goto err_workqueue_init;
    }
    return 0;

err_workqueue_init:
    FTS_ERROR("create_singlethread_workqueue failed\n");
    return -1;
}
#endif

#ifdef CONFIG_CREATE_SYS_INTERFACE
extern int fts_reset_proc(void);

//#define DEFINE_FT6446I
int fts_ctpm_fw_upgrade(struct i2c_client *client, unsigned char * pbt_buf, unsigned int fwsize)
{
	int i_ret = 0;

	if (fwsize < 8 || fwsize > 54 * 1024) {
		dev_err(&client->dev, "%s:FW length error\n", __func__);
		return -EIO;
	}
#ifdef DEFINE_FT6446I
	i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
#else
	if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a)) {
		i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)) {
		i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID==0x06)) {
		i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID==0x36)) {
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	} else if ((fts_updateinfo_curr.CHIP_ID==0x54)) {
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
#endif
	if (i_ret != 0) {
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	} else if (fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)	{
		fts_ctpm_auto_clb(client);
	}
	
	return i_ret;
}
#ifdef GET_VENDORID_FROM_BOOTLOADER
int tpd_get_chipvendor_from_bootloader(struct i2c_client *i2c_client, int *module_id_in_chip, int * chip_part_id)
{

	u8 auc_i2c_write_buf[FTS_SETTING_BUF_LEN] = {0};
	u8 reg_val[2] = {0};
	int i = 0, i_ret = -1;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		/*write 0xaa to register 0xfc */
		fts_i2c_write_reg(i2c_client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		//write 0x55 to register 0xfc 
		fts_i2c_write_reg(i2c_client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = fts_i2c_hid2std(i2c_client);
		if(i_ret == 0)	{
			FTS_DEBUG("[FTS] hid change to i2c fail ! \n");
			continue;
		}
		msleep(50);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(i2c_client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			FTS_DEBUG("[FTS] failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(50);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		
		reg_val[0] = reg_val[1] = 0x00;
		
		fts_i2c_read(i2c_client, auc_i2c_write_buf, 4, reg_val, 2);
		
		if(0x79 == reg_val[0] && 0x03 == reg_val[1]) {
			*chip_part_id = 0x55;
		} else if(0x79 == reg_val[0] && 0x06 == reg_val[1]) {
			*chip_part_id = 0x08;
		} else if(0x79 == reg_val[0] && 0x07 == reg_val[1]) {
			*chip_part_id = 0x0a;
		} else if(0x79 == reg_val[0] && 0x08 == reg_val[1]) {
			*chip_part_id = 0x06;
		} else if(0x79 == reg_val[0] && 0x18 == reg_val[1]) {
			*chip_part_id = 0x36;
		} else if(0x79 == reg_val[0] && 0x11 == reg_val[1]) {
			*chip_part_id = 0x11;
		} else if(0x54 == reg_val[0] && 0x2c == reg_val[1]) {
			*chip_part_id = 0x54;
		} 

		if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) || g_tpd_forceUpgrade) {
			FTS_DEBUG("tpd [FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} else {
			dev_err(&i2c_client->dev, "tpd [FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		}
	}
	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_read(i2c_client, auc_i2c_write_buf, 1, reg_val, 1);
	printk("tpd fts bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/*set read start address */
	auc_i2c_write_buf[0] = 0x3;
	auc_i2c_write_buf[1] = 0x0;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0x80;

	fts_i2c_read(i2c_client, auc_i2c_write_buf, 4, auc_i2c_write_buf, FTS_SETTING_BUF_LEN);
	printk("tpd [FTS] read from 0x80d70003 old setting: uc_i2c_addr = 0x%x,\
			uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			auc_i2c_write_buf[0], auc_i2c_write_buf[2], auc_i2c_write_buf[4]);
	*module_id_in_chip = auc_i2c_write_buf[4];
	
	fts_reset_proc();

	return 0;
}
#endif
int tpd_get_chipfw_version_etc(struct i2c_client *i2c_client, int * chipid_in_chip, 
	int *fwver_in_chip, int *module_id_in_chip)
{
	unsigned char uc_reg_value = 0;
#ifdef GET_VENDORID_FROM_BOOTLOADER
	int vendor_id = 0, chip_part_id = 0;
#endif
	int i = 0;

	while (i < 2) {
		i++;
		fts_i2c_read_reg(i2c_client, FTS_REG_CHIP_ID, &uc_reg_value);
		*chipid_in_chip = uc_reg_value;
		fts_i2c_read_reg(i2c_client, FTS_REG_FW_VER, &uc_reg_value);
		*fwver_in_chip = uc_reg_value;
		fts_i2c_read_reg(i2c_client, FTS_REG_VENDOR_ID, &uc_reg_value);
		*module_id_in_chip = uc_reg_value;
		
		if(0x00 == *module_id_in_chip || 0xff == *module_id_in_chip || FTS_REG_VENDOR_ID == *module_id_in_chip|| 0xef == *module_id_in_chip ) {
			printk("tpd vendor id read abnormal, reset and read again. \n");
#ifdef GET_VENDORID_FROM_BOOTLOADER
			tpd_get_chipvendor_from_bootloader(i2c_client, &vendor_id, &chip_part_id);
			*module_id_in_chip = vendor_id;
			*fwver_in_chip = 0x00;
			*chipid_in_chip = chip_part_id;
			break;
#else
			fts_reset_proc();
#endif
		} else {
			break;
		}
	}

	return 0;
}

int tpd_fts_ctpm_firmware_upgrade(struct tpd_classdev_t *cdev, unsigned char * pbt_buf, unsigned int size, unsigned int force_upg)
{
	int i_ret = 0;
	struct focal_chip_data *focal_data = (struct focal_chip_data*) cdev->private;
	struct i2c_client *client = focal_data->i2c_client;
	
	printk("%s curr chipid:0x%x, chip part id:0x%x.\n", __func__, fts_updateinfo_curr.CHIP_ID, cdev->ic_tpinfo.chip_part_id);
	
	if(cdev->ic_tpinfo.chip_part_id != fts_updateinfo_curr.CHIP_ID) {
		int i = 0;
		for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info);i++)
		{
			if(cdev->ic_tpinfo.chip_part_id == fts_updateinfo[i].CHIP_ID)
			{
				memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct fts_Upgrade_Info));
				break;
			}
		}
	}

	g_tpd_forceUpgrade = force_upg;

	i_ret = fts_ctpm_fw_upgrade(client, pbt_buf, size);
	if (i_ret != 0) {
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	}

	g_tpd_forceUpgrade = 0;
	return i_ret;
}
#endif


