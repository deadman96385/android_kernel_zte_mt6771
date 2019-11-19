/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_main.c
*
* Author: Software Development Team, AE
*
* Created: 2016-08-01
*
* Abstract: test entry for all IC
*
************************************************************************/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/slab.h>

#include "../../focaltech_core.h"

#include "../include/focaltech_test_main.h"
#include "../focaltech_test_global.h"

#include "../include/focaltech_test_supported_ic.h"
#include "../include/focaltech_test_ini.h"

#define FTS_DRIVER_LIB_INFO  "Test_Lib_Version   V1.6.1 2016-08-01"

#define FTS_TEST_STORE_DATA_SIZE        80*1024

FTS_I2C_READ_FUNCTION fts_i2c_read_test;
FTS_I2C_WRITE_FUNCTION fts_i2c_write_test;

char *g_testparamstring = NULL;

/////////////////////IIC communication
int init_i2c_read_func(FTS_I2C_READ_FUNCTION fpI2C_Read)
{
    unsigned char value = 0;
    unsigned char recode = 0;

    FTS_TEST_DBG("");

    fts_i2c_read_test = fpI2C_Read;
    if (NULL == fts_i2c_read_test)
    {
        FTS_TEST_DBG("[focal] %s fts_i2c_read_test == NULL ",  __func__);
    }

    //debug start
    recode = ReadReg(0xa6, &value);
    if (recode != ERROR_CODE_OK)
    {
        FTS_TEST_DBG("[focal] ReadReg Error, code: %d ",  recode);
    }
    else
    {
        FTS_TEST_DBG("[focal] ReadReg successed, Addr: 0xa6, value: 0x%02x ",  value);
    }
    //debug end

    return 0;
}

int init_i2c_write_func(FTS_I2C_WRITE_FUNCTION fpI2C_Write)
{
    FTS_TEST_DBG("");

    fts_i2c_write_test = fpI2C_Write;
    if (NULL == fts_i2c_write_test)
    {
        FTS_TEST_DBG("[focal] fts_i2c_read_test == NULL ");
    }
    return 0;
}



/************************************************************************
* Name: set_param_data
* Brief:  load Config. Set IC series, init test items, init basic threshold, int detailThreshold, and set order of test items
* Input: TestParamData, from ini file.
* Output: none
* Return: 0. No sense, just according to the old format.
***********************************************************************/
int set_param_data(char * TestParamData)
{
    int ret = 0;

    FTS_TEST_DBG("Enter  set_param_data.");
    g_testparamstring = TestParamData;//get param of ini file
    ret = ini_get_key_data(g_testparamstring);//get param to struct
    if (ret < 0)
    {
        FTS_TEST_DBG("ini_get_key_data error.");
        return ret;
    }

    //Read the selected chip from the configuration
    //Set g_ScreenSetParam.iSelectedIC
    OnInit_InterfaceCfg(g_testparamstring);

    /*Get IC Name*/
    fts_ic_table_get_ic_name_from_ic_code(g_ScreenSetParam.iSelectedIC, g_strIcName);


    // test configuration
    OnInit_TestItem(g_testparamstring);
    OnInit_BasicThreshold(g_testparamstring);
    if (IC_Capacitance_Type == Mutual_Capacitance || IC_Capacitance_Type == IDC_Capacitance)
    {
        OnInit_MCap_DetailThreshold(g_testparamstring);
    }
    else
    {
        OnInit_SCap_DetailThreshold(g_testparamstring);
    }
    SetTestItem();

    FTS_TEST_DBG("end of set_param_data.");
    return 0;
}

/************************************************************************
* Name: start_test_tp
* Brief:  Test entry. Select test items based on IC series
* Input: none
* Output: none
* Return: Test Result, PASS or FAIL
***********************************************************************/
#ifndef FTS_AUTO_RESET_EN
#define FTS_AUTO_RESET_EN 0
#endif

boolean start_test_tp(void)
{
    boolean bTestResult = false;

    FTS_TEST_DBG("[focal] %s start ",  __func__);
    FTS_TEST_DBG("IC_%s Test",  g_strIcName);

#if FTS_AUTO_RESET_EN
    fts_auto_reset_suspend();
    fts_auto_reset_record_time();
#endif

    bTestResult = Start_Test();

#if FTS_AUTO_RESET_EN
    fts_auto_reset_resume();
#endif

    EnterWork();

    return bTestResult;
}
/************************************************************************
* Name: get_test_data
* Brief:  Get test data based on IC series
* Input: none
* Output: pTestData, External application for memory, buff size >= 1024*8
* Return: the length of test data. if length > 0, got data;else ERR.
***********************************************************************/
int get_test_data(char *pTestData)
{
    int iLen = 0;
    FTS_TEST_DBG("[focal] %s start ",  __func__);

    iLen = Get_test_data(pTestData);

    return iLen;
}

int get_tpd_node_data(int *data, int length, int cmd, int type)
{
    int iLen = 0;

    if(Get_test_node_data != NULL)
	    iLen = Get_test_node_data(data, length, cmd, type);

    return iLen;
}

int focaltech_test_main_init(void)
{
    int ret = 0;

    FTS_TEST_DBG("[focal] %s ",  FTS_DRIVER_LIB_INFO);  //show lib version

    /*Allocate memory, storage test results*/
    g_pStoreAllData = NULL;
    if (NULL == g_pStoreAllData)
        g_pStoreAllData = fts_malloc(FTS_TEST_STORE_DATA_SIZE);
    if (NULL == g_pStoreAllData)
        return -1;

    /*  Allocate memory,  assigned to detail threshold structure*/
    ret = malloc_struct_DetailThreshold();
    if (ret < 0)
        return ret;

    return 0;
}
/************************************************************************
* Name: free_test_param_data
* Brief:  release printer memory
* Input: none
* Output: none
* Return: none.
***********************************************************************/
int focaltech_test_main_exit(void)
{

    FTS_TEST_DBG("[focal] release memory -start.");

    // it has been freed in fts_test_get_testparam_from_ini(). lifengshi.

    /* The release of the contents of the file configuration parameters */
    /*  if(NULL != g_testparamstring)
        {
            FTS_TEST_DBG("[FTS] release memory g_testparamstring.");
            fts_free(g_testparamstring);
            g_testparamstring = NULL;
        }
    */
    /* Release memory test results */
    if (NULL != g_pStoreAllData)
    {
        FTS_TEST_DBG("[FTS] release memory g_pStoreAllData.");
        fts_free(g_pStoreAllData);
        g_pStoreAllData = NULL;
    }

    /* Releasing the memory of the detailed threshold structure */
    FTS_TEST_DBG("[FTS] release memory  free_struct_DetailThreshold.");
    free_struct_DetailThreshold();

    /* release memory of key data for ini file */
    release_key_data();
    FTS_TEST_DBG("[focal] release memory -end.");
    return 0;
}

