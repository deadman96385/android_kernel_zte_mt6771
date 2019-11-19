
/********************************* (C) COPYRIGHT 2017 STMicroelectronics ********************************
 *
 * File Name         : lsm6dse.h
 * Authors           : IAN YANG, William Zeng
 * Version           : V3.0.7
 * Date              : 01/22/2018
 * Description       : LSM6DSE driver source file
 *
 *********************************************************************************************************
 * Copyright (c) 2017, STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *********************************************************************************************************
 * REVISON HISTORY
 *
 * VERSION | DATE          | DESCRIPTION
 *
 * 3.0.0   | 05/15/2017    | MTK driver initial version
 * 3.0.1   | 06/15/2017    | fixed some logical bug
 * 3.0.2   | 07/03/2017	   | optimized gyro driver
 * 3.0.3   | 07/05/2017	   | fixed ATA gyro test bug
 * 3.0.4   | 08/25/2017    | add gyro suspend and resume operation
 * 3.0.5   | 09/25/2017	   | reserved for SPI driver version
 * 3.0.6   | 09/25/2017	   | fixed bugs in factory mode
 * 3.0.7   | 01/22/2018	   | modified driver to compatible with Android O
 *
 ****************************************************************************************************/


#ifndef __LSM6DSE_H__
#define __LSM6DSE_H__

#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <accel.h>
#include <gyroscope.h>
#include <step_counter.h>
#include <cust_acc.h>
#include <cust_gyro.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/proc_fs.h>


#define DEBUG 1

#define CONFIG_STEP_COUNTER              0
#define CONFIG_STEP_DETECT               0
#define CONFIG_SIGNIFICANT_MOTION        0
#define CONFIG_TILT                      0

#define CONFIG_PEDOMETER_ALWAYS_ON       0   /* this mean the peodometer still work even system in susupend . */

#if (CONFIG_STEP_DETECT || CONFIG_SIGNIFICANT_MOTION || CONFIG_TILT)
#define CONFIG_HARDWARE_INTERRUPT        1   /* if use above feature , suggset use hardware interrupt */
#else
#define CONFIG_HARDWARE_INTERRUPT        0
#endif

#define ST_SENSOR_STEP_COUNTER              0
#define ST_SENSOR_STEP_DETECT               1
#define ST_SENSOR_SIGNIFICANT_MOTION        2
#define ST_SENSOR_TILT                      3

/* lsm6dse Register Map  (Please refer to lsm6dse Specifications) */
#define LSM6DSE_REG_FUNC_CFG_ACCESS     0x01
#define LSM6DSE_REG_INT1_CTRL           0X0D
#define LSM6DSE_REG_INT2_CTRL           0X0E
#define LSM6DSE_REG_WHO_AM_I			0x0F
#define LSM6DSE_REG_CTRL1_XL			0x10
#define LSM6DSE_REG_CTRL2_G				0x11
#define LSM6DSE_REG_CTRL3_C				0x12
#define LSM6DSE_REG_CTRL4_C				0x13
#define LSM6DSE_REG_CTRL5_C				0x14
#define LSM6DSE_REG_CTRL6_C				0x15
#define LSM6DSE_REG_CTRL7_G				0x16
#define LSM6DSE_REG_CTRL8_XL			0x17
#define LSM6DSE_REG_CTRL9_XL			0x18
#define LSM6DSE_REG_CTRL10_C			0x19

#define LSM6DSE_REG_OUTX_L_G			0x22
#define LSM6DSE_REG_OUTX_H_G			0x23
#define LSM6DSE_REG_OUTY_L_G			0x24
#define LSM6DSE_REG_OUTY_H_G			0x25
#define LSM6DSE_REG_OUTZ_L_G			0x26
#define LSM6DSE_REG_OUTZ_H_G			0x27

#define LSM6DSE_REG_OUTX_L_XL			0x28
#define LSM6DSE_REG_OUTX_H_XL			0x29
#define LSM6DSE_REG_OUTY_L_XL			0x2A
#define LSM6DSE_REG_OUTY_H_XL			0x2B
#define LSM6DSE_REG_OUTZ_L_XL			0x2C
#define LSM6DSE_REG_OUTZ_H_XL			0x2D
#define LSM6DSE_REG_STEP_COUNTER_L      0x4B
#define LSM6DSE_REG_STEP_COUNTER_H      0x4C
#define LSM6DSE_REG_FUNC_SRC            0x53
#define LSM6DSE_REG_TAP_CFG             0x58
#define LSM6DSE_REG_MD1_CFG             0x5E

/* step ,tilt , significant , func */
#define LSM6DSE_FUNC_ENABLE_MASK        0X04
#define LSM6DSE_SIGN_MOTION_ENABLE_MASK 0X01
#define LSM6DSE_STEP_ENABLE_MASK        0X10
#define LSM6DSE_TILT_ENABLE_MASK        0X08
#define LSM6DSE_INT_ACTIVE_MASK       0x20

/*  interrupt det mask*/
#define LSM6DSE_STEP_INT_MASK           0X80
#define LSM6DSE_SIGN_MOTION_INT_MASK    0X40
#define LSM6DSE_TILT_INT_MASK           0X02

/*  interrupt det flag mask*/
#define LSM6DSE_FLAG_STEP_MASK           0X10
#define LSM6DSE_FLAG_TILT_MASK           0X20
#define LSM6DSE_FlAG_SIGN_MOTION_MASK    0X40

/*
*  embeded register
*/
#define LSM6DSE_REG_PEDO_THS            0x0F
#define LSM6DSE_REG_PEDO_DEB            0x14

#define LSM6DSE_EN			            0x01
#define LSM6DSE_DIS			            0x00

/*
*  register mask bit
*/
#define LSM6DSE_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN        0x80
#define LSM6DSE_REG_CTRL1_XL_MASK_FS_XL			            0x0C
#define LSM6DSE_REG_CTRL1_XL_MASK_ODR_XL		            0xF0
#define LSM6DSE_REG_CTRL2_G_MASK_FS_G			            0x0C
#define LSM6DSE_REG_CTRL2_G_MASK_ODR_G			            0xF0
#define LSM6DSE_REG_CTRL2_G_MASK_ODR_G			            0xF0
#define LSM6DSE_REG_CTRL10_C_MASK_FUNC_EN                   0x04
#define LSM6DSE_REG_CTRL10_C_MASK_PEDO_RST_STEP             0x02
#define LSM6DSE_REG_TAP_CFG_MASK_PEDO_EN                    0x40
/*
*  embeded register mask bit
*/
#define LSM6DSE_REG_PEDO_THS_MASK_PEDO_4G                   0x80
#define LSM6DSE_REG_PEDO_THS_MASK_PEDO_THS_MIN              0x1F
#define LSM6DSE_REG_PEDO_DEB_MASK_DEB_TIME                  0xF8
#define LSM6DSE_REG_PEDO_DEB_MASK_DEB_STEP                  0x07

/*
*  register value
*/
#define LSM6DSE_FIXED_DEVID									0x6A

#define LSM6DSE_REG_FUNC_CFG_ACCESS_ENABLE                  0x80
#define LSM6DSE_REG_FUNC_CFG_ACCESS_DISABLE                 0x00

#define LSM6DSE_REG_CTRL1_XL_ODR_208HZ			0x05
#define LSM6DSE_REG_CTRL1_XL_ODR_104HZ			0x04
#define LSM6DSE_REG_CTRL1_XL_ODR_52HZ			0x03
#define LSM6DSE_REG_CTRL1_XL_ODR_26HZ			0x02
#define LSM6DSE_REG_CTRL1_XL_ODR_13HZ			0x01
#define LSM6DSE_REG_CTRL1_XL_ODR_0HZ			0x00

#define LSM6DSE_REG_CTRL1_XL_FS_2G			                0x00
#define LSM6DSE_REG_CTRL1_XL_FS_4G			                0x02
#define LSM6DSE_REG_CTRL1_XL_FS_8G			                0x03
#define LSM6DSE_REG_CTRL1_XL_FS_16G			                0x01


#define LSM6DSE_REG_CTRL2_G_ODR_208HZ			            0x05
#define LSM6DSE_REG_CTRL2_G_ODR_104HZ			            0x04
#define LSM6DSE_REG_CTRL2_G_ODR_52HZ			            0x03
#define LSM6DSE_REG_CTRL2_G_ODR_26HZ			            0x03
#define LSM6DSE_REG_CTRL2_G_ODR_0HZ				            0x00

#define LSM6DSE_REG_CTRL2_G_FS_245DPS			            0x00
#define LSM6DSE_REG_CTRL2_G_FS_500DPS			            0x01
#define LSM6DSE_REG_CTRL2_G_FS_1000DPS			            0x02
#define LSM6DSE_REG_CTRL2_G_FS_2000DPS			            0x03

#define LSM6DSE_REG_CTRL10_C_FUNC_ENABLE                    0x04
#define LSM6DSE_REG_CTRL10_C_FUNC_DISABLE                   0x00
#define LSM6DSE_REG_CTRL10_C_FUNC_PEDO_RST_STEP             0x02

#define LSM6DSE_REG_TAP_CFG_PEDO_ENABLE                     0x40
#define LSM6DSE_REG_TAP_CFG_PEDO_DISABLE                    0x00

/*
*  embeded register value
*/
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_128MG            0x84
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_160MG            0x85
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_192MG            0x86
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_224MG            0x87
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_256MG            0x88
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_288MG            0x89
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_320MG            0x8A
#define LSM6DSE_REG_PEDO_THS_PEDO_4G_352MG            0x8B

#define LSM6DSE_REG_PEDO_DEB_TIME_800MS               0x50
#define LSM6DSE_REG_PEDO_DEB_TIME_880MS               0x58
#define LSM6DSE_REG_PEDO_DEB_TIME_960MS               0x60
#define LSM6DSE_REG_PEDO_DEB_TIME_1040MS              0x68
#define LSM6DSE_REG_PEDO_DEB_TIME_1120MS              0x70

#define LSM6DSE_REG_PEDO_DEB_STEP_5STEP               0x05
#define LSM6DSE_REG_PEDO_DEB_STEP_6STEP               0x06
#define LSM6DSE_REG_PEDO_DEB_STEP_7STEP               0x07

/*
*  return value
*/
#define LSM6DSE_SUCCESS					    0
#define LSM6DSE_ERR_I2C					    -1
#define LSM6DSE_ERR_STATUS					-3
#define LSM6DSE_ERR_SETUP_FAILURE			-4
#define LSM6DSE_ERR_GETGSENSORDATA			-5
#define LSM6DSE_ERR_IDENTIFICATION			-6

#define LSM6DSE_BUFSIZE			64

#define LSM6DSE_AXIS_X          0
#define LSM6DSE_AXIS_Y          1
#define LSM6DSE_AXIS_Z          2
#define LSM6DSE_AXES_NUM        3
#define LSM6DSE_DATA_LEN        6
#define LSM6DSE_DEV_NAME        "LSM6DSE"
#define LSM6DSE_CALI_LEN       (10)
#define LSM6DSE_CALI_TOLERANCE (1000) /* mg */
/* #define DEGREE_TO_RAD_1000		17 */
/* 1 rad = 180/PI degree, MAX_LSB = 131, */
/* 180*131/PI = 7506 */
#define DEGREE_TO_RAD    7506
/* #define CONFIG_LSM6DSE_LOWPASS   //apply low pass filter on output*/

/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL   = 0x04,
	ADX_TRC_CALI    = 0X08,
	ADX_TRC_INFO    = 0X10,
} ADX_TRC;

/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};

/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor	scalefactor;
	int					sensitivity;
};

/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][LSM6DSE_AXES_NUM];
	int sum[LSM6DSE_AXES_NUM];
	int num;
	int idx;
};

/*----------------------------------------------------------------------------*/
struct lsm6dse_acc {
	/* struct i2c_client *client; */
	u8				name[32];
	struct acc_hw	*lsm6dse_acc_hw;
	struct hwmsen_convert   cvt;

	/*misc*/
	struct data_resolution *reso;
	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t                filter;
	s32                     cali_sw[LSM6DSE_AXES_NUM];

	/*data*/
	s32                     offset[LSM6DSE_AXES_NUM];
	s16                     data[LSM6DSE_AXES_NUM];
	bool                    lsm6dse_acc_power;
	int                     odr;
	/* int                     acc_enabled; */
#if defined(CONFIG_LSM6DSE_LOWPASS)
	atomic_t                firlen;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif
};

struct lsm6dse_gyro {
	/*struct i2c_client *client;*/
	u8     name[32];
	struct gyro_hw *lsm6dse_gyro_hw;
	struct hwmsen_convert   cvt;

	/*misc*/
	struct data_resolution *reso;
	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t                filter;
	s32                     cali_sw[LSM6DSE_AXES_NUM];

	/*data*/
	s32                     offset[LSM6DSE_AXES_NUM];
	s16                     data[LSM6DSE_AXES_NUM];
	bool                    lsm6dse_gyro_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LSM6DSE_LOWPASS)
	atomic_t                firlen;
	atomic_t                fir_en;
	struct data_filter      fir;
#endif
};

struct lsm6dse_pedo {
	u8     name[32];

	atomic_t                trace;
	atomic_t                suspend;
	atomic_t                selftest;
	atomic_t                filter;

	u32                     data;
	int                     overflow;
	bool                    lsm6dse_pedo_power;
	int                     odr;
	/*int                     enabled;*/
};

struct lsm6dse_data {
	u8     name[32];
	u8     chip_id;
#if CONFIG_HARDWARE_INTERRUPT
	int    irq;
	atomic_t   irq_enabled;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
#endif
	struct i2c_client *client;
	struct lsm6dse_acc lsm6dse_acc_data;
	struct lsm6dse_gyro lsm6dse_gyro_data;
	struct lsm6dse_pedo lsm6dse_pedo_data;
	u8     reg_addr;
	int    acc_enabled;
	int    gyro_enabled;
	int    step_c_enabled;
	int    step_d_enabled;
	int    significant_enabled;
	int    tilt_enabled;
};

#define ST_TAG                  "STMEMS "
#define ST_ERR(fmt, args...)    pr_err(ST_TAG "%s %d : "fmt, __func__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    pr_err(ST_TAG fmt, ##args)

#if defined DEBUG
#define ST_FUN(f)               pr_info(ST_TAG "%s\n", __func__)
#define ST_DBG(fmt, args...)    pr_err(ST_TAG fmt, ##args)
#else
#define ST_FUN(f)
#define ST_DBG(fmt, args...)
#endif

extern struct lsm6dse_data *obj_i2c_data;

int lsm6dse_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6dse_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6dse_i2c_write_with_mask(struct i2c_client *client, u8 addr, u8 mask, u8 data);


void dumpReg(struct lsm6dse_data *obj);
int lsm6dse_set_interrupt(void);

int lsm6dse_acc_set_power_mode(struct lsm6dse_acc *acc_obj, bool enable);
int lsm6dse_gyro_set_power_mode(struct lsm6dse_gyro *gyro_obj, bool enable);

extern struct acc_init_info lsm6dse_acc_init_info;

extern struct gyro_init_info lsm6dse_gyro_init_info;
extern struct tilt_init_info  lsm6dse_tilt_init_info;
extern struct step_c_init_info lsm6dse_pedo_init_info;
extern int lsm6dse_module_init_flag;

#endif
