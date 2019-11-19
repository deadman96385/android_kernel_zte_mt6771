/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
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

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author:     Software Department, FocalTech
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <asm/uaccess.h>

#include <linux/i2c.h>//iic
#include <linux/delay.h>//msleep
#include <linux/time.h>
#include <linux/rtc.h>

#include "../../focaltech_core.h"
#include "../include/focaltech_test_main.h"
#include "../include/focaltech_test_ini.h"
#include "../focaltech_test_global.h"

#include "tpd_sys.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_TEST_INFO  "File Version of  focaltech_test.c:  V1.1.0 2016-08-01"

// Define the configuration file storage directory
#define FTS_INI_FILE_PATH "/mnt/sdcard/"

#define FTS_TEST_BUFFER_SIZE        80*1024
#define FTS_TEST_PRINT_SIZE     128
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char g_str_save_file_path[256];
static unsigned char g_str_ini_file_path[256];
static unsigned char g_str_ini_filename[128];

static int g_getNodeDataType = -1;
static int g_getNodeDataCmd = -1;
static int g_tpdReturnDataType = -1;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
int g_int_tptest_result = 0;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_test_get_ini_size(char *config_name);
static int fts_test_read_ini_data(char *config_name, char *config_buf);
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen);
static int fts_test_get_testparam_from_ini(char *config_name);
static int fts_test_entry(char *ini_file_name);

static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
static int fts_test_i2c_write(unsigned char *writebuf, int writelen);


/*******************************************************************************
* functions body
*******************************************************************************/
#if 0
//  old fts_i2c_read/write function. need to set fts_i2c_client.
extern struct i2c_client* fts_i2c_client;
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
#endif
static int fts_test_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
    int iret = -1;
#if 1
    //  old fts_i2c_read function. need to set fts_i2c_client.
    //Modify the i2c_read function that is used in this project
    iret = fts_i2c_read(fts_i2c_client, writebuf, writelen, readbuf, readlen);
#else
    iret = fts_i2c_read(writebuf, writelen, readbuf, readlen);
#endif

    return iret;

}

static int fts_test_i2c_write(unsigned char *writebuf, int writelen)
{
    int iret = -1;
#if 1
    //  old fts_i2c_write function.  need to set fts_i2c_client.
    //Modify the i2c_read function that is used in this project
    iret = fts_i2c_write(fts_i2c_client, writebuf, writelen);
#else
    iret = fts_i2c_write(writebuf, writelen);
#endif

    return iret;
}
static int fts_test_get_save_filename(char *filename, int len)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	snprintf(filename, len, "test_data%04d%02d%02d_%02d%02d%02d.csv",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);
	return 0;
}

//Gets the configuration file size for allocating memory to read configuration
static int fts_test_get_ini_size(char *config_name)
{
    struct file *pfile = NULL;
    struct inode *inode = NULL;
    //unsigned long magic;
    off_t fsize = 0;
    char filepath[128];
    memset(filepath, 0, sizeof(filepath));

    sprintf(filepath, "%s%s", g_str_ini_file_path, config_name);
    //sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);

    if (NULL == pfile)
        pfile = filp_open(filepath, O_RDONLY, 0);

    if (IS_ERR(pfile))
    {
        FTS_TEST_DBG("error occured while opening file %s.",  filepath);
        return -EIO;
    }

   inode = pfile->f_path.dentry->d_inode;
    //magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    filp_close(pfile, NULL);

    return fsize;
}

//Read configuration to memory
static int fts_test_read_ini_data(char *config_name, char *config_buf)
{
    struct file *pfile = NULL;
    struct inode *inode = NULL;
    //unsigned long magic;
    off_t fsize = 0;
    char filepath[128];
    loff_t pos = 0;
    mm_segment_t old_fs;

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", g_str_ini_file_path, config_name);
    //sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, config_name);
    if (NULL == pfile)
    {
        pfile = filp_open(filepath, O_RDONLY, 0);
    }
    if (IS_ERR(pfile))
    {
        FTS_TEST_DBG("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    inode = pfile->f_path.dentry->d_inode;
    //magic = inode->i_sb->s_magic;
    fsize = inode->i_size;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(pfile, config_buf, fsize, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);

    return 0;
}

//Save test data to SD card etc.
static int fts_test_save_test_data(char *file_name, char *data_buf, int iLen)
{
    struct file *pfile = NULL;

    char filepath[128];
    loff_t pos;
    mm_segment_t old_fs;

    memset(filepath, 0, sizeof(filepath));
    sprintf(filepath, "%s%s", g_str_save_file_path, file_name);
    //sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, file_name);
    if (NULL == pfile)
    {
        pfile = filp_open(filepath, O_CREAT|O_RDWR, 0);
    }
    if (IS_ERR(pfile))
    {
        FTS_TEST_DBG("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(pfile, data_buf, iLen, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);

    return 0;
}

//Read, parse the configuration file, initialize the test variable
static int fts_test_get_testparam_from_ini(char *config_name)
{
    char *pcfiledata = NULL;
    int ret = 0;

    int inisize = fts_test_get_ini_size(config_name);

    FTS_TEST_DBG("ini_size = %d ", inisize);
    if (inisize <= 0)
    {
        FTS_TEST_DBG("%s ERROR:Get firmware size failed",  __func__);
        return -EIO;
    }

    pcfiledata = fts_malloc(inisize + 1);
    if (NULL == pcfiledata)
    {
        FTS_TEST_DBG("fts_malloc failed in function:%s",  __func__);
        return -1;
    }

    memset(pcfiledata, 0, inisize + 1);

    if (fts_test_read_ini_data(config_name, pcfiledata))
    {
        FTS_TEST_DBG(" - ERROR: fts_test_read_ini_data failed" );
        fts_free(pcfiledata);
        pcfiledata = NULL;

        return -EIO;
    }
    else
    {
        FTS_TEST_DBG("fts_test_read_ini_data successful");
    }

    ret = set_param_data(pcfiledata);

    fts_free(pcfiledata);   // lifengshi add. 20160608
    pcfiledata = NULL;

    if (ret < 0)
        return ret;

    return 0;
}

/////////////////////////////////
//Test library call entry
///////////////////////////////////
static int fts_test_entry(char *ini_file_name)
{
    /* place holder for future use */
    char cfgname[128];
    char *testdata = NULL;
    char *printdata = NULL;
    int iTestDataLen=0; //The actual length of the test data in the library is used to save the data to the file.
    int ret = 0;
    int icycle = 0, i =0;
    int print_index = 0;


    FTS_TEST_DBG("");
    FTS_TEST_DBG("ini_file_name:%s.", ini_file_name);
    /*Used to obtain the test data stored in the library, pay attention to the size of the distribution space.*/
    FTS_TEST_DBG("Allocate memory, size: %d", FTS_TEST_BUFFER_SIZE);
    testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
    if (NULL == testdata)
    {
        FTS_TEST_DBG("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
    if (NULL == printdata)
    {
        FTS_TEST_DBG("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    /*Initialize the platform related I2C read and write functions*/

#if 0
    init_i2c_write_func(fts_i2c_write);
    init_i2c_read_func(fts_i2c_read);
#else
    init_i2c_write_func(fts_test_i2c_write);
    init_i2c_read_func(fts_test_i2c_read);
#endif

    /*Initialize pointer memory*/
    ret = focaltech_test_main_init();
    if (ret < 0)
    {
        FTS_TEST_DBG("focaltech_test_main_init() error.");
        goto TEST_ERR;
    }

    /*Read parse configuration file*/
    memset(cfgname, 0, sizeof(cfgname));
    sprintf(cfgname, "%s", ini_file_name);
    FTS_TEST_DBG("ini_file_name = %s", cfgname);
    if (fts_test_get_testparam_from_ini(cfgname) <0)
    {
        FTS_TEST_DBG("get testparam from ini failure");
        goto TEST_ERR;
    }

    /*Start testing according to the test configuration*/
    if (true == start_test_tp())
        FTS_TEST_DBG("tp test pass");
    else
        FTS_TEST_DBG("tp test failure");

    /*Gets the number of tests in the test library and saves it*/
    iTestDataLen = get_test_data(testdata);
    //FTS_TEST_DBG("\n%s", testdata);

    icycle = 0;
    /*Print test data packets */
    FTS_TEST_DBG("print test data: \n");
    for (i = 0; i < iTestDataLen; i++)
    {
        if (('\0' == testdata[i]) //Meet the end
            ||(icycle == FTS_TEST_PRINT_SIZE -2)//Meet the print string length requirements
            ||(i == iTestDataLen-1)//The last character
           )
        {
            if (icycle == 0)
            {
                print_index++;
            }
            else
            {
                memcpy(printdata, testdata + print_index, icycle);
                printdata[FTS_TEST_PRINT_SIZE-1] = '\0';
                printk("%s", printdata);
                print_index += icycle;
                icycle = 0;
            }
        }
        else
        {
            icycle++;
        }
    }
    printk("\n");
    {
        char filename[64];
        fts_test_get_save_filename(filename, 64);	
        fts_test_save_test_data(filename, testdata, iTestDataLen);
    }
    //fts_test_save_test_data("testdata.csv", testdata, iTestDataLen);

    /*Release memory */
    focaltech_test_main_exit();


    //mutex_unlock(&g_device_mutex);
    if (NULL != testdata) fts_free(testdata);
    if (NULL != printdata) fts_free(printdata);
    return 0;

TEST_ERR:
    if (NULL != testdata) fts_free(testdata);
    if (NULL != printdata) fts_free(printdata);
    return -1;
}

/////////////////////////////////
//Test library call entry show
/************************************************************************
* Name: fts_test_entry_show
* Brief:  no
* Input:
* Output:
* Return:
***********************************************************************/
///////////////////////////////////
static int fts_test_entry_show(char *ini_file_name, char *bufdest, ssize_t* pinumread)
{
    /* place holder for future use */
    char cfgname[128] = {0};
    char *testdata = NULL;
    char *printdata = NULL;
    int iTestDataLen=0;//The actual length of the test data in the library is used to save the data to the file.
    int ret = 0;
    int icycle = 0, i =0;
    int print_index = 0;

    FTS_TEST_DBG("");
    FTS_TEST_DBG("ini_file_name:%s.", ini_file_name);

    /*Used to obtain the test data stored in the library, pay attention to the size of the distribution space.*/
    FTS_TEST_DBG("Allocate memory, size: %d", FTS_TEST_BUFFER_SIZE);
    testdata = fts_malloc(FTS_TEST_BUFFER_SIZE);
    if (NULL == testdata)
    {
        FTS_TEST_DBG("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    printdata = fts_malloc(FTS_TEST_PRINT_SIZE);
    if (NULL == printdata)
    {
        FTS_TEST_DBG("fts_malloc failed in function:%s",  __func__);
        return -1;
    }
    /*Initialize the platform related I2C read and write functions*/
    init_i2c_write_func(fts_test_i2c_write);
    init_i2c_read_func(fts_test_i2c_read);

    /*Initialize pointer memory*/
    ret = focaltech_test_main_init();
    if (ret < 0)
    {
        FTS_TEST_DBG("focaltech_test_main_init() error.");
        goto TEST_ERR;
    }

    /*Read parse configuration file*/
    memset(cfgname, 0, sizeof(cfgname));
    sprintf(cfgname, "%s", ini_file_name);
    FTS_TEST_DBG("ini_file_name = %s", cfgname);
    if (fts_test_get_testparam_from_ini(cfgname) <0)
    {
        FTS_TEST_DBG("get testparam from ini failure");
        goto TEST_ERR;
    }

    /*Start testing according to the test configuration*/
    if (true == start_test_tp())
        FTS_TEST_DBG("tp test pass");
    else
        FTS_TEST_DBG("tp test failure");

    /*Gets the number of tests in the test library and saves it*/
    iTestDataLen = get_test_data(testdata);
    //FTS_TEST_DBG("\n%s", testdata);
    FTS_TEST_DBG(" iTestDataLen:%d", iTestDataLen);

    if (iTestDataLen < PAGE_SIZE)
        *pinumread = (ssize_t)iTestDataLen;
    else
        *pinumread = PAGE_SIZE-1;   //  if = PAGE_SIZE. then  will report "ill_read_buffer: dev_attr_show+0x0/0x4c returned bad count"

    memcpy(bufdest, testdata, (int)(*pinumread));

    icycle = 0;
    /*Print test data packets  */
    FTS_TEST_DBG("print test data: \n");
    for (i = 0; i < iTestDataLen; i++)
    {
        if (('\0' == testdata[i]) //Meet the end
            ||(icycle == FTS_TEST_PRINT_SIZE -2)//Meet the print string length requirements
            ||(i == iTestDataLen-1)//The last character
           )
        {
            if (icycle == 0)
            {
                print_index++;
            }
            else
            {
                memcpy(printdata, testdata + print_index, icycle);
                printdata[FTS_TEST_PRINT_SIZE-1] = '\0';
                printk("%s", printdata);
                print_index += icycle;
                icycle = 0;
            }
        }
        else
        {
            icycle++;
        }
    }
    printk("\n");

    fts_test_save_test_data("testdata.csv", testdata, iTestDataLen);

    /*Release memory */
    focaltech_test_main_exit();

    //mutex_unlock(&g_device_mutex);
    if (NULL != testdata) fts_free(testdata);
    testdata = NULL;
    if (NULL != printdata) fts_free(printdata);
    return 0;

TEST_ERR:
    if (NULL != testdata) fts_free(testdata);
    if (NULL != printdata) fts_free(printdata);
    return -1;
}

/************************************************************************
* Name: fts_test_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;

    int nret = -1;
    struct i2c_client *client = fts_i2c_client;

    mutex_lock(&fts_input_dev->mutex);
    disable_irq(client->irq);

    nret = fts_test_entry_show( "test.ini", buf, &num_read_chars);
    if (0 != nret)
    {
        FTS_TEST_DBG("%s fts test fail!.", __func__);
        num_read_chars = snprintf(buf, PAGE_SIZE,"fts test fail!\n");
    } else {
        FTS_TEST_DBG("%s fts test pass!.", __func__);
    }

    enable_irq(client->irq);
    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

/************************************************************************
* Name: fts_test_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[128] = {0};
    struct i2c_client *client = fts_i2c_client;
    memset(fwname, 0, sizeof(fwname));
    sprintf(fwname, "%s", buf);
    fwname[count-1] = '\0';
    FTS_TEST_DBG("fwname:%s.", fwname);

    mutex_lock(&fts_input_dev->mutex);

    disable_irq(client->irq);
    fts_test_entry( fwname);
    enable_irq(client->irq);

    mutex_unlock(&fts_input_dev->mutex);

    return count;
}

static ssize_t tpd_test_save_file_path_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;

    mutex_lock(&fts_input_dev->mutex);

    num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_save_file_path);

    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

static ssize_t tpd_test_save_file_path_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    memset(g_str_save_file_path, 0, sizeof(g_str_save_file_path));
    snprintf(g_str_save_file_path, 256, "%s", buf);
    //g_str_save_file_path[count] = '\0';

    FTS_TEST_DBG("save file path:%s.", g_str_save_file_path);

    return count;
}


static ssize_t tpd_test_ini_file_path_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;

    mutex_lock(&fts_input_dev->mutex);

    num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_file_path);

    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

static ssize_t tpd_test_ini_file_path_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    memset(g_str_ini_file_path, 0, sizeof(g_str_ini_file_path));
    snprintf(g_str_ini_file_path, 256, "%s", buf);
    //g_str_ini_file_path[count] = '\0';

    FTS_TEST_DBG("ini file path:%s.", g_str_ini_file_path);

    return count;
}

static ssize_t tpd_test_filename_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;

    mutex_lock(&fts_input_dev->mutex);

    num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", g_str_ini_filename);

    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

static ssize_t tpd_test_filename_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    memset(g_str_ini_filename, 0, sizeof(g_str_ini_filename));
    snprintf(g_str_ini_filename, 128, "%s", buf);
    //g_str_ini_filename[count] = '\0';

    FTS_TEST_DBG("fwname:%s.", g_str_ini_filename);

    return count;
}

static ssize_t tpd_test_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;
    struct i2c_client *client = fts_i2c_client;

    mutex_lock(&fts_input_dev->mutex);
    disable_irq(client->irq);

    num_read_chars = snprintf(buf, PAGE_SIZE,"%d", g_int_tptest_result);
    FTS_TEST_DBG("%s result:%d.", __func__, g_int_tptest_result);

    enable_irq(client->irq);
    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

static ssize_t tpd_test_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

    struct i2c_client *client = fts_i2c_client;

    FTS_TEST_DBG("%s fwname:%s.", __func__, g_str_ini_filename);

    mutex_lock(&fts_input_dev->mutex);

    disable_irq(client->irq);
    g_int_tptest_result = 0;
    fts_test_entry( g_str_ini_filename);
    enable_irq(client->irq);

    mutex_unlock(&fts_input_dev->mutex);

    return count;
}

int node_data[512];
static ssize_t tpd_test_node_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;
    int iLen = 0;
    int iRow, iCol;
    int iTxNum = 12, iRxNum = 21;
    
    mutex_lock(&fts_input_dev->mutex);

    if(1 == g_getNodeDataCmd) {
        memset(node_data, 0, sizeof(node_data));
        iLen = get_tpd_node_data(node_data, 512, 3, g_getNodeDataType);
        if(1 == g_tpdReturnDataType) {
            memcpy(buf, (char*)node_data, iLen * sizeof(int));
            iLen = iLen * sizeof(int);
        } else {
            for (iRow = 0; iRow < iTxNum; iRow++)
            {
                for (iCol = 0; iCol < iRxNum; iCol++)
                {
                    if(iCol == (iRxNum -1))
                        iLen += sprintf(buf + iLen,"%d, \n",  node_data[iRow * iRxNum + iCol]); 
                    else
                        iLen += sprintf(buf + iLen,"%d, ", node_data[iRow * iRxNum + iCol]);    
                }
            }
        }
    }

    num_read_chars = iLen;
    mutex_unlock(&fts_input_dev->mutex);

    return num_read_chars;
}

static ssize_t tpd_test_node_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

    struct i2c_client *client = fts_i2c_client;
    int current_DataCmd = 0, current_DataType = 0, current_ReturnDataType = 0;

    sscanf(buf, "%d %d %d", &current_DataCmd, &current_DataType, &current_ReturnDataType);

    FTS_TEST_DBG("%s before cmd:%d, type:%d , current cmd:%d, type:%d .", __func__, 
		g_getNodeDataCmd, g_getNodeDataType , current_DataCmd, current_DataType);
	
    if((1 == g_getNodeDataCmd && 1 == current_DataCmd) || (2 == g_getNodeDataCmd && 2 == current_DataCmd)) {
        FTS_TEST_DBG("%s ERROR operation AS before .", __func__);
        return -1;
    }
    if(2 == current_DataCmd && current_DataType != g_getNodeDataType) {
        FTS_TEST_DBG("%s warning close type not same.", __func__);
        current_DataType = g_getNodeDataType;
    }
    g_getNodeDataType = current_DataType;
    g_getNodeDataCmd = current_DataCmd;

    FTS_TEST_DBG("%s get node data cmd:%d, type:%d .", __func__, g_getNodeDataCmd, g_getNodeDataType);  

    mutex_lock(&fts_input_dev->mutex);
    /*cmd 1:start 2:stop*/
    init_i2c_write_func(fts_test_i2c_write);
    init_i2c_read_func(fts_test_i2c_read);
    switch(g_getNodeDataCmd) {
        case 1:
            disable_irq(client->irq);
            g_tpdReturnDataType = current_ReturnDataType;
	     get_tpd_node_data(node_data, 512, 1, g_getNodeDataType);
            break;
        case 2:
            enable_irq(client->irq);
	     get_tpd_node_data(node_data, 512, 2, g_getNodeDataType);
            break;
        default:
            break;
    }  

    mutex_unlock(&fts_input_dev->mutex);

    return count;
}
static ssize_t tpd_test_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t num_read_chars = 0;

    num_read_chars = snprintf(buf, PAGE_SIZE, "21 12 20 12 21");

    return num_read_chars;
}

static DEVICE_ATTR(tpd_test_channel_setting, S_IRUGO|S_IRUSR, tpd_test_channel_show, NULL);
static DEVICE_ATTR(tpd_test_save_file_path, S_IRUGO|S_IWUSR, tpd_test_save_file_path_show, tpd_test_save_file_path_store);
static DEVICE_ATTR(tpd_test_ini_file_path, S_IRUGO|S_IWUSR, tpd_test_ini_file_path_show, tpd_test_ini_file_path_store);
static DEVICE_ATTR(tpd_test_filename, S_IRUGO|S_IWUSR, tpd_test_filename_show, tpd_test_filename_store);
static DEVICE_ATTR(tpd_test_cmd, S_IRUGO|S_IWUSR, tpd_test_cmd_show, tpd_test_cmd_store);
static DEVICE_ATTR(tpd_test_node_data, S_IRUGO|S_IWUSR, tpd_test_node_data_show, tpd_test_node_data_store);
/*  upgrade from app.bin
*    example:echo "***.ini" > fts_test
*/
static DEVICE_ATTR(fts_test, S_IRUGO|S_IWUSR, fts_test_show, fts_test_store);

/* add your attr in here*/
static struct attribute *fts_test_attributes[] =
{
    &dev_attr_fts_test.attr,
    &dev_attr_tpd_test_filename.attr,
    &dev_attr_tpd_test_node_data.attr,
    &dev_attr_tpd_test_cmd.attr,
    &dev_attr_tpd_test_ini_file_path.attr,
    &dev_attr_tpd_test_save_file_path.attr,
    &dev_attr_tpd_test_channel_setting.attr,
    NULL
};

static struct attribute_group fts_test_attribute_group =
{
    .attrs = fts_test_attributes
};


int fts_test_init(struct i2c_client *client)
{
    int err=0;

    FTS_TEST_DBG("[focal] %s ",  FOCALTECH_TEST_INFO);  //show version
    FTS_TEST_DBG("");//default print: current function name and line number

    strncpy(g_str_save_file_path, FTS_INI_FILE_PATH, 256);
    strncpy(g_str_ini_file_path, FTS_INI_FILE_PATH, 256);
    strncpy(g_str_ini_filename, "test.ini", 128);

    err = sysfs_create_group(&client->dev.kobj, &fts_test_attribute_group);
    if (0 != err)
    {
        FTS_TEST_DBG( "[focal] %s() - ERROR: sysfs_create_group() failed.",  __func__);
        //sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);
        return -EIO;
    }
    else
    {
        FTS_TEST_DBG("[focal] %s() - sysfs_create_group() succeeded.", __func__);
    }
    if (IS_ERR(tpd_fw_cdev.dev)) {
        FTS_TEST_DBG("[focal] %s() - tpd_fw_cdev dev null.", __func__);;
    } else {
        err = sysfs_create_group(&tpd_fw_cdev.dev->kobj, &fts_test_attribute_group);
        if (0 != err)
        {
            FTS_TEST_DBG( "[focal] %s() - ERROR: sysfs_create_group() failed.",  __func__);
        }
        else
        {
            FTS_TEST_DBG("[focal] %s() - sysfs_create_group() succeeded.", __func__);
        }
    }
    //fts_protocol_windows_to_android(client);
    return err;
}
int fts_test_exit(struct i2c_client *client)
{
    FTS_TEST_DBG("");//default print: current function name and line number
    sysfs_remove_group(&client->dev.kobj, &fts_test_attribute_group);

    return 0;
}

