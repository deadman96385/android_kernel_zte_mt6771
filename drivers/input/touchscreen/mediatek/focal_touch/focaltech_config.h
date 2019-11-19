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
* File Name: focaltech_config.h
*
*    Author: Focaltech Driver Team
*
*   Created: 2016-08-08
*
*  Abstract: global configurations
*
*   Version: v1.0
*
************************************************************************/
#ifndef _LINUX_FOCLATECH_CONFIG_H_
#define _LINUX_FOCLATECH_CONFIG_H_

/**************************************************/
/****** chip type defines, do not modify *********/

#define _FT8716     0x8716
#define _FT8607     0x8607

/*************************************************/

/*
 * choose your ic chip type of focaltech
 */
#define FTS_CHIP_TYPE   _FT8716

/******************* Enables *********************/
/*********** 1 to enable, 0 to disable ***********/

/*
 * show debug log info
 * enable it for debug, disable it for release
 */
#define FTS_DEBUG_EN                            0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define FTS_MT_PROTOCOL_B_EN                    0

/*
 * Gesture function enable
 * default: disable
 */
#define FTS_GESTURE_EN                          1

/*
 * ESD check & protection
 * default: disable
 */
#define FTS_ESDCHECK_EN                         0

/*
 * usb charge plug in & out detect
 * default: disable
 */
#define FTS_CHARGE_DETECT_EN                    0

/*
 * Production test enable
 * 1: enable, 0:disable(default)
 */
#define FTS_TEST_EN                             1

/*
 * Nodes for tools, please keep enable
 */
#define FTS_SYSFS_NODE_EN                       1
#define FTS_APK_NODE_EN                         1
#define FTS_CTL_IIC_NODE_EN                     1

/*
 * Customer power enable
 * enable it when customer need control TP power
 * default: disable
 */
#define FTS_POWER_SOURCE_CUST_EN                0

/****************************************************/

/*
 * max touch points number
 * default: 10
 */
#define FTS_MAX_POINTS                          5


/********************** Upgrade ****************************/

/* auto cb check
 * default: disable
 */
#define FTS_AUTO_CLB_EN                         0

/* get vender id from bootloader when upgrade error
 * default: disable
 */
#define GET_VENDORID_FROM_BOOTLOADER

/*
 * FW_APP.i files for upgrade
 * define your own fw_app, the sample one is invalid
 */
#define FTS_UPGRADE_FW_APP        "include/firmware/FT8716_app_sample.h"

/*
 * vendor_id(s) for the ic
 * you need confirm vendor_id for upgrade
 * if only one vendor, ignore vendor_2_id, otherwise
 * you need define both of them
 */



/* show upgrade time in log*/
#define FTS_GET_UPGRADE_TIME                    0

/*
 * upgrade ping-pong test for debug
 * enable it for upgrade debug if needed
 * default: disable
 */
#define FTS_UPGRADE_PINGPONG_TEST               0
/* error test for debug */
#define FTS_UPGRADE_ERROR_TEST                  0
/* pingpong or error test times, default: 1000 */
#define FTS_UPGRADE_TEST_NUMBER                 1000

/*********************************************************/


//#define CONFIG_MTK_SENSOR_HUB_SUPPORT
#define TPD_I2C_NUMBER                          0

/* for sysfs debug info */
#define TPD_WAKEUP_GPIO                         62
#define TPD_RESET_GPIO                          62
#define TPD_EINT_GPIO                           10


#define CONFIG_CREATE_SYS_INTERFACE
#define CONFIG_SYS_FIRMWARE_UPGRADE


#endif /* _LINUX_FOCLATECH_CONFIG_H_ */