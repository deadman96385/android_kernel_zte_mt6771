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
* File Name: focaltech_gestrue.c
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
#if FTS_GESTURE_EN
#include "focaltech_gesture.h"
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/
short pointnum                                  = 0;
unsigned short coordinate_x[150]                = {0};
unsigned short coordinate_y[150]                = {0};

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/

int tpd_set_bsg_input_capability(struct input_dev *bsg_dev){
    input_set_capability(bsg_dev, EV_KEY, KEY_POWER);    //FOR wakeup TEST
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_UP); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_DOWN);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_LEFT); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_RIGHT); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_C); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_E); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_M); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_L);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_U); 
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(bsg_dev, EV_KEY, KEY_GESTURE_Z);
        
    __set_bit(KEY_GESTURE_RIGHT, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_UP, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_DOWN, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_U, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_O, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_E, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_M, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_W, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_L, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_S, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_V, bsg_dev->keybit);
    __set_bit(KEY_GESTURE_Z, bsg_dev->keybit);

    return 0;
}

/*******************************************************************************
*   Name: fts_gesture_init
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
int fts_gesture_init(struct input_dev *input_dev)
{
    tpd_set_bsg_input_capability(input_dev);
#if 0
    //init_para(480,854,60,0,0);
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    //input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    //input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

    __set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
    //__set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_C, input_dev->keybit);
    //__set_bit(KEY_GESTURE_Z, input_dev->keybit);
#endif
    return 0;
}

static void tpd_handle_bsg_code(struct input_dev *input_dev, int gesture_id)
{
	unsigned int keycode = KEY_RESERVED;
	
	printk("tpd fts gesture_id==0x%x\n ", gesture_id);
	
	switch(gesture_id) {
	case GESTURE_LEFT:
		keycode = KEY_GESTURE_LEFT;
		printk("tpd fts gesture_id==0x%x, bsg slide left.\n ", gesture_id);
		break;
	case GESTURE_RIGHT:
		keycode = KEY_GESTURE_RIGHT;
		printk("tpd fts gesture_id==0x%x, bsg slide right.\n ", gesture_id);
		break;
	case GESTURE_UP:
		keycode = KEY_GESTURE_UP;
		printk("tpd fts gesture_id==0x%x, bsg slide up.\n ", gesture_id);
		break;
	case GESTURE_DOWN:
		keycode = KEY_GESTURE_DOWN;
		printk("tpd fts gesture_id==0x%x, bsg slide down.\n ", gesture_id);
		break;
	case GESTURE_DOUBLECLICK:
		keycode = KEY_GESTURE_DOUBLE_CLICK;
		printk("tpd fts gesture_id==0x%x, bsg double click.\n ", gesture_id);
		break;
	case GESTURE_O:
		keycode = KEY_GESTURE_O;
		printk("tpd fts gesture_id==0x%x, bsg 'o'.\n ", gesture_id);
		break;
	case GESTURE_W:
		keycode = KEY_GESTURE_W;
		printk("tpd fts gesture_id==0x%x, bsg 'w'.\n ", gesture_id);
		break;
	case GESTURE_M:
		keycode = KEY_GESTURE_M;
		printk("tpd fts gesture_id==0x%x, bsg 'm'.\n ", gesture_id);
		break;
	case GESTURE_C:
		keycode = KEY_GESTURE_C;
		printk("tpd fts gesture_id==0x%x, bsg 'c'.\n ", gesture_id);
		break;
	case GESTURE_E:
		keycode = KEY_GESTURE_E;
		printk("tpd fts gesture_id==0x%x, bsg 'e'.\n ", gesture_id);
		break;
	case GESTURE_L:
		keycode = KEY_GESTURE_L;
		printk("tpd fts gesture_id==0x%x, bsg 'l'.\n ", gesture_id);
		break;
	case GESTURE_S:
		keycode = KEY_GESTURE_S;
		printk("tpd fts gesture_id==0x%x, bsg 's'.\n ", gesture_id);
		break;
	case GESTURE_V:
		keycode = KEY_GESTURE_V;
		printk("tpd fts gesture_id==0x%x, bsg 'v'.\n ", gesture_id);
		break;
	case GESTURE_Z:
		keycode = KEY_GESTURE_Z;
		printk("tpd fts gesture_id==0x%x, bsg 'z'.\n ", gesture_id);
		break;
	default:
		keycode = KEY_RESERVED;
		printk("tpd fts gesture_id==0x%x, bsg is not defined.\n ", gesture_id);
		break;
	}
	input_report_key(input_dev, keycode, 1);
	input_sync(input_dev);
	input_report_key(input_dev, keycode, 0);
	input_sync(input_dev);
}


/*******************************************************************************
*   Name: fts_check_gesture
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
static void fts_check_gesture(struct input_dev *input_dev,int gesture_id)
{
#if 1
    tpd_handle_bsg_code(input_dev, gesture_id);
#else
    char *envp[2];
    printk("fts gesture_id==0x%x\n ",gesture_id);

    switch (gesture_id)
    {
        case GESTURE_LEFT:
            /*input_report_key(input_dev, KEY_GESTURE_LEFT, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_LEFT, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=LEFT";
            break;
        case GESTURE_RIGHT:
            /*input_report_key(input_dev, KEY_GESTURE_RIGHT, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_RIGHT, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=RIGHT";
            break;
        case GESTURE_UP:
            /*input_report_key(input_dev, KEY_GESTURE_UP, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_UP, 0);
            input_sync(input_dev);  */
            envp[0]="GESTURE=UP";
            break;
        case GESTURE_DOWN:
            /*input_report_key(input_dev, KEY_GESTURE_DOWN, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_DOWN, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=DOWN";
            break;
        case GESTURE_DOUBLECLICK:
            /*input_report_key(input_dev, KEY_POWER, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_POWER, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=DOUBLE_CLICK";
            break;
        case GESTURE_O:
            /*input_report_key(input_dev, KEY_GESTURE_O, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_O, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=O";
            break;
        case GESTURE_W:
            /*input_report_key(input_dev, KEY_GESTURE_W, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_W, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=W";
            break;
        case GESTURE_M:
            /*input_report_key(input_dev, KEY_GESTURE_M, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_M, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=M";
            break;
        case GESTURE_E:
            /*input_report_key(input_dev, KEY_GESTURE_E, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_E, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=E";
            break;
        case GESTURE_L:
            /*input_report_key(input_dev, KEY_GESTURE_L, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_L, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=L";
            break;
        case GESTURE_S:
            /*input_report_key(input_dev, KEY_GESTURE_S, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_S, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=S";
            break;
        case GESTURE_V:
            /*input_report_key(input_dev, KEY_GESTURE_V, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_V, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=V";
            break;
        /*case GESTURE_Z:
                input_report_key(input_dev, KEY_GESTURE_Z, 1);
                input_sync(input_dev);
                input_report_key(input_dev, KEY_GESTURE_Z, 0);
                input_sync(input_dev);
                break;*/
        case  GESTURE_C:
            /*input_report_key(input_dev, KEY_GESTURE_C, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_GESTURE_C, 0);
            input_sync(input_dev);*/
            envp[0]="GESTURE=C";
            break;
        default:
            break;

    }
    envp[1]=NULL;
    kobject_uevent_env(&tpd->tpd_dev->kobj, KOBJ_CHANGE, envp);
    sysfs_notify(&tpd->tpd_dev->kobj, NULL, "GESTURE_ID");
#endif
}

/************************************************************************
*   Name: fts_gesture_readdata
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int fts_gesture_readdata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
    int gestrue_id = 0;

    buf[0] = 0xd3;
    pointnum = 0;

    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    //printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");

    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

    /* FW */
    if (fts_updateinfo_curr.CHIP_ID==0x54 || fts_updateinfo_curr.CHIP_ID==0x58 || fts_updateinfo_curr.CHIP_ID==0x86 || fts_updateinfo_curr.CHIP_ID==0x87  || fts_updateinfo_curr.CHIP_ID == 0x64)
    {
        gestrue_id = buf[0];
        pointnum = (short)(buf[1]) & 0xff;
        buf[0] = 0xd3;

        if ((pointnum * 4 + 2)<255)
        {
            ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 2));
        }
        else
        {
            ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 255);
            ret = fts_i2c_read(fts_i2c_client, buf, 0, buf+255, (pointnum * 4 + 2) -255);
        }
        if (ret < 0)
        {
            printk( "%s read touchdata failed.\n", __func__);
            return ret;
        }

        fts_check_gesture(fts_input_dev,gestrue_id);
        for (i = 0; i < pointnum; i++)
        {
            coordinate_x[i] =  (((s16) buf[0 + (4 * i+2)]) & 0x0F) <<
                               8 | (((s16) buf[1 + (4 * i+2)])& 0xFF);
            coordinate_y[i] = (((s16) buf[2 + (4 * i+2)]) & 0x0F) <<
                              8 | (((s16) buf[3 + (4 * i+2)]) & 0xFF);
        }
        return -1;
    }
    // other IC's gestrue in driver
    if (0x24 == buf[0])
    {
        gestrue_id = 0x24;
        fts_check_gesture(fts_input_dev,gestrue_id);
        printk( "%d check_gesture gestrue_id.\n", gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;
    if ((pointnum * 4 + 8)<255)
    {
        ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, (pointnum * 4 + 8));
    }
    else
    {
        ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 255);
        ret = fts_i2c_read(fts_i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
    }
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

    //gestrue_id = fetch_object_sample(buf, pointnum);//need gesture lib.a
    gestrue_id = 0x24;
    fts_check_gesture(fts_input_dev,gestrue_id);
    printk( "%d read gestrue_id.\n", gestrue_id);

    for (i = 0; i < pointnum; i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i+8)]) & 0x0F) <<
                           8 | (((s16) buf[1 + (4 * i+8)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i+8)]) & 0x0F) <<
                          8 | (((s16) buf[3 + (4 * i+8)]) & 0xFF);
    }
    return -1;
}

int fts_gesture_suspend(struct i2c_client *i2c_client)
{
    int i;
    char state;

    printk("tpd: fts %s func in. \n", __func__);

    FTS_FUNC_ENTER();

    fts_i2c_write_reg(i2c_client, 0xd0, 0x01);
    fts_i2c_write_reg(i2c_client, 0xd1, 0xff);
    fts_i2c_write_reg(i2c_client, 0xd2, 0xff);
    fts_i2c_write_reg(i2c_client, 0xd5, 0xff);
    fts_i2c_write_reg(i2c_client, 0xd6, 0xff);
    fts_i2c_write_reg(i2c_client, 0xd7, 0xff);
    fts_i2c_write_reg(i2c_client, 0xd8, 0xff);

    msleep(10);

    for (i = 0; i < 10; i++)
    {
        fts_i2c_read_reg(i2c_client, 0xd0, &state);

        if (state == 1)
        {
            FTS_INFO("TPD gesture suspend\n");
            FTS_FUNC_EXIT();
            return 0;
        }
        else
        {
            fts_i2c_write_reg(i2c_client, 0xd0, 0x01);
            fts_i2c_write_reg(i2c_client, 0xd1, 0xff);
            fts_i2c_write_reg(i2c_client, 0xd2, 0xff);
            fts_i2c_write_reg(i2c_client, 0xd5, 0xff);
            fts_i2c_write_reg(i2c_client, 0xd6, 0xff);
            fts_i2c_write_reg(i2c_client, 0xd7, 0xff);
            fts_i2c_write_reg(i2c_client, 0xd8, 0xff);
            msleep(10);
        }
    }

    if (i >= 9)
    {
        FTS_ERROR("TPD gesture suspend failed!\n");
        FTS_FUNC_EXIT();
        return -1;
    }
    FTS_FUNC_EXIT();

    return 0;
}
#endif
