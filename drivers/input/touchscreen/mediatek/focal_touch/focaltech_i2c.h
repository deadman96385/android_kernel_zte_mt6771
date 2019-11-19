/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_i2c.h
*
*    Author: fupeipei
*
*   Created: 2016-08-04
*
*  Abstract:
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_I2C_H__
#define __LINUX_FOCALTECH_I2C_H__

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
extern int fts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int fts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int fts_i2c_hid2std(struct i2c_client * client);
extern int fts_i2c_init(void);
extern int fts_i2c_exit(void);

#endif
