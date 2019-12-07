/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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
* File Name: Focaltech_ex_fun.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
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

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE                            0
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_AUTOCLB                            4
#define PROC_UPGRADE_INFO                       5
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define FTS_DEBUG_DIR_NAME                      "fts_debug"
#define PROC_NAME                               "ftxxxx-debug"
#define WRITE_BUF_SIZE                          1016
#define READ_BUF_SIZE                           1016

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
#if FTS_ESDCHECK_EN
static void esd_process(u8 *writebuf, int buflen, bool flag)
{
    if (flag)
    {
        if ( (writebuf[1] == 0xFC) && (writebuf[2] == 0x55) && (buflen == 0x03) ) // Upgrade command
        {
            FTS_DEBUG("%s: Upgrade command(%x %x %x)", __func__, writebuf[0], writebuf[1], writebuf[2]);
            fts_esdcheck_switch(DISABLE);
        }
        else if ( (writebuf[1] == 0x00) && ((writebuf[2] & 0x70) == 0x40) && (buflen == 0x03) ) /* factory mode bit 4 5 6 */
        {
            FTS_DEBUG("%s: Entry factory mode(%x %x %x)", __func__, writebuf[0], writebuf[1], writebuf[2]);
            fts_esdcheck_switch(DISABLE);
        }
        else if ( (writebuf[1] == 0x00) && ((writebuf[2] & 0x70) == 0x00) && (buflen == 0x03) ) /* normal mode bit 4 5 6 */
        {
            FTS_DEBUG("%s: Exit factory mode(%x %x %x)", __func__, writebuf[0], writebuf[1], writebuf[2]);
            fts_esdcheck_switch(ENABLE);
        }
        else
        {
            fts_esdcheck_proc_busy(1);
        }
    }
    else
    {
        if ( (writebuf[1] == 0x07) && (buflen == 0x02) )
        {
            FTS_DEBUG("%s: Upgrade finish-trigger reset(07)(%x %x)", __func__, writebuf[0], writebuf[1]);
            fts_esdcheck_switch(ENABLE);
        }
        else
        {
            fts_esdcheck_proc_busy(0);
        }
    }
}
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    unsigned char writebuf[WRITE_BUF_SIZE];
    int buflen = count;
    int writelen = 0;
    int ret = 0;

    if (copy_from_user(&writebuf, buff, buflen))
    {
        dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
        return -EFAULT;
    }
#if FTS_ESDCHECK_EN
    esd_process(writebuf, buflen, 1);
#endif
    proc_operate_mode = writebuf[0];
    switch (proc_operate_mode)
    {
        case PROC_UPGRADE:
        {
            char upgrade_file_path[128];
            memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
            sprintf(upgrade_file_path, "%s", writebuf + 1);
            upgrade_file_path[buflen-1] = '\0';
            FTS_DEBUG("%s\n", upgrade_file_path);
            disable_irq(fts_i2c_client->irq);
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif
            ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
            enable_irq(fts_i2c_client->irq);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
//                return ret;
            }
        }
        break;

        case PROC_SET_TEST_FLAG:
            FTS_DEBUG("%s: PROC_SET_TEST_FLAG = %x\n", __func__, writebuf[1]);
#if FTS_ESDCHECK_EN
            if (writebuf[1] == 0)
            {
                fts_esdcheck_switch(DISABLE);
            }
            else
            {
                fts_esdcheck_switch(ENABLE);
            }
#endif
            break;
        case PROC_READ_REGISTER:
            writelen = 1;
            ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                return ret;
            }
            break;
        case PROC_WRITE_REGISTER:
            writelen = 2;
            ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                return ret;
            }
            break;
        case PROC_AUTOCLB:
            FTS_DEBUG("%s: autoclb\n", __func__);
            fts_ctpm_auto_clb(fts_i2c_client);
            break;
        case PROC_READ_DATA:
        case PROC_WRITE_DATA:
            writelen = count - 1;
            if (writelen>0)
            {
                ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
                if (ret < 0)
                {
                    dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                    return ret;
                }
            }
            break;
        default:
            break;
    }

#if FTS_ESDCHECK_EN
    esd_process(writebuf, buflen, 0);
#endif

    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return count;
    }
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    int num_read_chars = 0;
    int readlen = 0;
    u8 regvalue = 0x00, regaddr = 0x00;
    unsigned char buf[READ_BUF_SIZE];

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    switch (proc_operate_mode)
    {
        case PROC_UPGRADE:
            // after calling fts_debug_write to upgrade
            regaddr = 0xA6;
            ret = fts_i2c_read_reg(fts_i2c_client, regaddr, &regvalue);
            if (ret < 0)
                num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
            else
                num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
            break;
        case PROC_READ_REGISTER:
            readlen = 1;
            ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
            if (ret < 0)
            {
#if FTS_ESDCHECK_EN
                fts_esdcheck_proc_busy(0);
#endif
                dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
                return ret;
            }
            num_read_chars = 1;
            break;
        case PROC_READ_DATA:
            readlen = count;
            ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
            if (ret < 0)
            {
#if FTS_ESDCHECK_EN
                fts_esdcheck_proc_busy(0);
#endif
                dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
                return ret;
            }

            num_read_chars = readlen;
            break;
        case PROC_WRITE_DATA:
            break;
        default:
            break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    if (copy_to_user(buff, buf, num_read_chars))
    {
        dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
        return -EFAULT;
    }

    return num_read_chars;
}
static const struct file_operations fts_proc_fops =
{
    .owner  = THIS_MODULE,
    .read   = fts_debug_read,
    .write  = fts_debug_write,

};
#else
/* interface of write proc */
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp,
                           const char __user *buff, unsigned long len, void *data)
{
    //struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
    unsigned char writebuf[WRITE_BUF_SIZE];
    int buflen = len;
    int writelen = 0;
    int ret = 0;

    if (copy_from_user(&writebuf, buff, buflen))
    {
        dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
        return -EFAULT;
    }
#if FTS_ESDCHECK_EN
    esd_process(writebuf, buflen, 1);
#endif
    proc_operate_mode = writebuf[0];
    switch (proc_operate_mode)
    {

        case PROC_UPGRADE:
        {
            char upgrade_file_path[128];
            memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
            sprintf(upgrade_file_path, "%s", writebuf + 1);
            upgrade_file_path[buflen-1] = '\0';
            FTS_DEBUG("%s\n", upgrade_file_path);
            disable_irq(fts_i2c_client->irq);
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif
            ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
            enable_irq(fts_i2c_client->irq);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
//                return ret;
            }
        }
        break;
        case PROC_SET_TEST_FLAG:
            FTS_DEBUG("%s: PROC_SET_TEST_FLAG = %x\n", __func__, writebuf[1]);
#if FTS_ESDCHECK_EN
            if (writebuf[1] == 0)
            {
                fts_esdcheck_switch(DISABLE);
            }
            else
            {
                fts_esdcheck_switch(ENABLE);
            }
#endif
            break;
        case PROC_READ_REGISTER:
            writelen = 1;
            ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                return ret;
            }
            break;
        case PROC_WRITE_REGISTER:
            writelen = 2;
            ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
            if (ret < 0)
            {
                dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                return ret;
            }
            break;
        case PROC_AUTOCLB:
            FTS_DEBUG("%s: autoclb\n", __func__);
            fts_ctpm_auto_clb(fts_i2c_client);
            break;
        case PROC_READ_DATA:
        case PROC_WRITE_DATA:
            writelen = len - 1;
            if (writelen>0)
            {
                ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
                if (ret < 0)
                {
                    dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
//                    return ret;
                }
            }
            break;
        default:
            break;
    }

#if FTS_ESDCHECK_EN
    esd_process(writebuf, buflen, 0);
#endif

    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return len;
    }
}

/* interface of read proc */
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
                           off_t off, int count, int *eof, void *data )
{
    //struct i2c_client *client = (struct i2c_client *)fts_proc_entry->data;
    int ret = 0;
    unsigned char buf[READ_BUF_SIZE];
    int num_read_chars = 0;
    int readlen = 0;
    u8 regvalue = 0x00, regaddr = 0x00;

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    switch (proc_operate_mode)
    {
        case PROC_UPGRADE:
            // after calling fts_debug_write to upgrade
            regaddr = 0xA6;
            ret = fts_i2c_read_reg(fts_i2c_client, regaddr, &regvalue);
            if (ret < 0)
                num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
            else
                num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
            break;
        case PROC_READ_REGISTER:
            readlen = 1;
            ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
            if (ret < 0)
            {
#if FTS_ESDCHECK_EN
                fts_esdcheck_proc_busy(0);
#endif
                dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
                return ret;
            }
            num_read_chars = 1;
            break;
        case PROC_READ_DATA:
            readlen = count;
            ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
            if (ret < 0)
            {
#if FTS_ESDCHECK_EN
                fts_esdcheck_proc_busy(0);
#endif
                dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
                return ret;
            }

            num_read_chars = readlen;
            break;
        case PROC_WRITE_DATA:
            break;
        default:
            break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    memcpy(page, buf, num_read_chars);
    return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
    fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
#else
    fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
#endif
    if (NULL == fts_proc_entry)
    {
        dev_err(&client->dev, "Couldn't create proc entry!\n");

        return -ENOMEM;
    }
    else
    {
        dev_info(&client->dev, "Create proc entry success!\n");

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
        //fts_proc_entry->data = client;
        fts_proc_entry->write_proc = fts_debug_write;
        fts_proc_entry->read_proc = fts_debug_read;
#endif
    }
    return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{

    if (fts_proc_entry)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
        proc_remove(fts_proc_entry);
#else
        remove_proc_entry(PROC_NAME, NULL);
#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;
    u8 fwver = 0;
    //struct i2c_client *client = container_of(dev, struct i2c_client, dev);  jacob use globle fts_wq_data data
    mutex_lock(&fts_input_dev->mutex);

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    if (fts_i2c_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
    {
        num_read_chars = snprintf(buf, PAGE_SIZE,"I2c transfer error!\n");
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    if (fwver == 255)
        num_read_chars = snprintf(buf, PAGE_SIZE,"get tp fw version fail!\n");
    else
    {
        num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);
    }

    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

static int fts_isNumCh(const char ch)
{
    int result = 0;
    if (ch >= '0' && ch <= '9')
    {
        result = 1;//(int)(ch - '0');
    }
    else if (ch >= 'a' && ch <= 'f')
    {
        result = 1;//(int)(ch - 'a') + 10;
    }
    else if (ch >= 'A' && ch <= 'F')
    {
        result = 1;//(int)(ch - 'A') + 10;
    }
    else
    {
        result = 0;
    }

    return result;
}

static int fts_hexCharToValue(const char ch)
{
    int result = 0;
    if (ch >= '0' && ch <= '9')
    {
        result = (int)(ch - '0');
    }
    else if (ch >= 'a' && ch <= 'f')
    {
        result = (int)(ch - 'a') + 10;
    }
    else if (ch >= 'A' && ch <= 'F')
    {
        result = (int)(ch - 'A') + 10;
    }
    else
    {
        result = -1;
    }

    return result;
}

static int ftx_hexToStr(char *hex, int iHexLen, char *ch, int *iChLen)
{
    int high=0;
    int low=0;
    int tmp = 0;
    int i = 0;
    int iCharLen = 0;
    if (hex == NULL || ch == NULL)
    {
        return -1;
    }

    FTS_DEBUG("iHexLen: %d in function:%s\n\n", iHexLen, __func__);

    if (iHexLen %2 == 1)
    {
        return -2;
    }

    for (i=0; i<iHexLen; i+=2)
    {
        high = fts_hexCharToValue(hex[i]);
        if (high < 0)
        {
            ch[iCharLen] = '\0';
            return -3;
        }

        low = fts_hexCharToValue(hex[i+1]);
        if (low < 0)
        {
            ch[iCharLen] = '\0';
            return -3;
        }
        tmp = (high << 4) + low;
        ch[iCharLen++] = (char)tmp;
    }
    ch[iCharLen] = '\0';
    *iChLen = iCharLen;
    FTS_DEBUG("iCharLen: %d, iChLen: %d in function:%s\n\n", iCharLen, *iChLen, __func__);
    return 0;
}

static void fts_strToBytes(char * bufStr, int iLen, char* uBytes, int *iBytesLen)
{
    int i=0;
    int iNumChLen=0;

    *iBytesLen=0;

    for (i=0; i<iLen; i++)
    {
        if (fts_isNumCh(bufStr[i])) //filter illegal chars
        {
            bufStr[iNumChLen++] = bufStr[i];
        }
    }

    bufStr[iNumChLen] = '\0';

    ftx_hexToStr(bufStr, iNumChLen, uBytes, iBytesLen);
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* place holder for future use */
    return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    ssize_t num_read_chars = 0;
    int retval;
    /*u32 wmreg=0;*/
    long unsigned int wmreg=0;
    u8 regaddr=0xff,regvalue=0xff;
    u8 valbuf[5]= {0};

    memset(valbuf, 0, sizeof(valbuf));
    mutex_lock(&fts_input_dev->mutex);
    num_read_chars = count - 1;
    if (num_read_chars != 2)
    {
        if (num_read_chars != 4)
        {
            dev_err(dev, "please input 2 or 4 character\n");
            goto error_return;
        }
    }
    memcpy(valbuf, buf, num_read_chars);
    retval = kstrtoul(valbuf, 16, &wmreg);
    fts_strToBytes((char*)buf, num_read_chars, valbuf, &retval);

    if (1==retval)
    {
        regaddr = valbuf[0];
        retval = 0;
    }
    else if (2==retval)
    {
        regaddr = valbuf[0];
        regvalue = valbuf[1];
        retval = 0;
    }
    else retval =-1;
    if (0 != retval)
    {
        dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
        goto error_return;
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    if (2 == num_read_chars)
    {
        /*read register*/
        regaddr = wmreg;
        FTS_DEBUG("(0x%02x)\n", regaddr);
        if (fts_i2c_read_reg(client, regaddr, &regvalue) < 0)
            FTS_DEBUG("%s : Could not read the register(0x%02x)\n", __func__, regaddr);
        else
            FTS_DEBUG("%s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
    }
    else
    {
        regaddr = wmreg>>8;
        regvalue = wmreg;
        if (fts_i2c_write_reg(client, regaddr, regvalue)<0)
            dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
        else
            dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
error_return:
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* place holder for future use */
    return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    //struct fts_ts_data *data = NULL;
    u8 uc_host_fm_ver;
    int i_ret;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    //data = (struct fts_ts_data *) i2c_get_clientdata(client);

    mutex_lock(&fts_input_dev->mutex);
    disable_irq(client->irq);
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(DISABLE);
#endif
    i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(ENABLE);
#endif
    if (i_ret == 0)
    {
        msleep(300);
        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
        dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
    }
    else
    {
        dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
    }

    //fts_ctpm_auto_upgrade(client);
    enable_irq(client->irq);
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    /* place holder for future use */
    return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[128];
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count-1] = '\0';

    mutex_lock(&fts_input_dev->mutex);
    disable_irq(client->irq);
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(DISABLE);
#endif
    fts_ctpm_fw_upgrade_with_app_file(client, fwname);
#if FTS_ESDCHECK_EN
    fts_esdcheck_switch(ENABLE);
#endif
    enable_irq(client->irq);
    mutex_unlock(&fts_input_dev->mutex);

    return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_driverversion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;

    mutex_lock(&fts_input_dev->mutex);

    count = sprintf(buf, FTS_DRIVER_VERSION "\n");

    mutex_unlock(&fts_input_dev->mutex);

    return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_driverversion_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

/************************************************************************
* Name: fts_dumpreg_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_dumpreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    /* place holder for future use */
    return -EPERM;
}

/************************************************************************
* Name: fts_dumpreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_dumpreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char tmp[256];
    int count = 0;
    u8 regvalue = 0;
    struct i2c_client *client;

    mutex_lock(&fts_input_dev->mutex);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    client = container_of(dev, struct i2c_client, dev);
    fts_i2c_read_reg(client, 0xA5, &regvalue);       //power mode 0:active 1:monitor 3:sleep
    count += sprintf(tmp + count, "Power Mode:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0xA6, &regvalue);       //FWver
    count += sprintf(tmp + count, "FW Ver:0x%02x\n",regvalue);

    fts_i2c_read_reg(client, 0xA8, &regvalue);       //Vendor ID
    count += sprintf(tmp + count, "Vendor ID:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0xAB, &regvalue);       //LCD Busy number
    count += sprintf(tmp + count, "LCD Busy Number:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0xD0, &regvalue);       // 1 Gesture mode,0 Normal mode
    count += sprintf(tmp + count, "Gesture Mode:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0x8b, &regvalue);       // 3 charge in
    count += sprintf(tmp + count, "charge stat:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0x8f, &regvalue);       //Interrupt counter
    count += sprintf(tmp + count, "INT count:0x%02x\n", regvalue);

    fts_i2c_read_reg(client, 0x91, &regvalue);       //Flow work counter
    count += sprintf(tmp + count, "ESD count:0x%02x\n", regvalue);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    memcpy(buf, tmp, count);
    mutex_unlock(&fts_input_dev->mutex);
    return count;
}

static ssize_t fts_tpd_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    char tmp[64];
    int count = 0;
    count += sprintf(tmp + count, "TPD gpio dump:\n");
    count += sprintf(tmp + count, "TPD reset : %d\n", gpio_get_value(TPD_RESET_GPIO));
    count += sprintf(tmp + count, "TPD wakeup: %d\n", gpio_get_value(TPD_WAKEUP_GPIO));
    count += sprintf(tmp + count, "TPD Eint  : %d\n", gpio_get_value(TPD_EINT_GPIO));
    memcpy(buf, tmp, count);
    return count;
}

static ssize_t fts_tpd_gpio_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/****************************************/
/* sysfs */
/* get the fw version
*   example:cat fw_version
*/
static DEVICE_ATTR(fts_fw_version, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

/* upgrade from *.i
*   example: echo 1 > fw_update
*/
static DEVICE_ATTR(fts_fw_update, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/* read and write register
*   read example: echo 88 > rw_reg ---read register 0x88
*   write example:echo 8807 > rw_reg ---write 0x07 into register 0x88
*
*   note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(fts_rw_reg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*  upgrade from app.bin
*    example:echo "*_app.bin" > upgrade_app
*/
static DEVICE_ATTR(fts_upgrade_app, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(fts_driver_version, S_IRUGO|S_IWUSR, fts_driverversion_show, fts_driverversion_store);
static DEVICE_ATTR(fts_tpd_gpio, S_IRUGO|S_IWUSR, fts_tpd_gpio_show, fts_tpd_gpio_store);
static DEVICE_ATTR(fts_dump_reg, S_IRUGO|S_IWUSR, fts_dumpreg_show, fts_dumpreg_store);


/* add your attr in here*/
static struct attribute *fts_attributes[] =
{
    &dev_attr_fts_fw_version.attr,
    &dev_attr_fts_fw_update.attr,
    &dev_attr_fts_rw_reg.attr,
    &dev_attr_fts_dump_reg.attr,
    &dev_attr_fts_tpd_gpio.attr,
    &dev_attr_fts_upgrade_app.attr,
    &dev_attr_fts_driver_version.attr,
    NULL
};

static struct attribute_group fts_attribute_group =
{
    .attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
    int err;
    err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
    if (0 != err)
    {
        dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
        sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
        return -EIO;
    }
    else
    {
        pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
    }
    //HidI2c_To_StdI2c(client);
    return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
    sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
    return 0;
}
