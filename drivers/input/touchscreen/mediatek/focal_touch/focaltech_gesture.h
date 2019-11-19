/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_gesture.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: function for hand recognition
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_GESTURE_H__
#define __LINUX_FOCALTECH_GESTURE_H__

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
//#define  KEY_GESTURE_U                        KEY_U
//#define  KEY_GESTURE_UP                         KEY_UP
//#define  KEY_GESTURE_DOWN                       KEY_DOWN
//#define  KEY_GESTURE_LEFT                       KEY_LEFT
//#define  KEY_GESTURE_RIGHT                      KEY_RIGHT
//#define  KEY_GESTURE_O                          KEY_O
//#define  KEY_GESTURE_E                          KEY_E
//#define  KEY_GESTURE_M                          KEY_M
//#define  KEY_GESTURE_L                          KEY_L
//#define  KEY_GESTURE_W                          KEY_W
//#define  KEY_GESTURE_S                          KEY_S
//#define  KEY_GESTURE_V                          KEY_V
//#define  KEY_GESTURE_C                          KEY_C
//#define  KEY_GESTURE_Z                        KEY_Z

#define GESTURE_LEFT                            0x20
#define GESTURE_RIGHT                           0x21
#define GESTURE_UP                              0x22
#define GESTURE_DOWN                            0x23
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_O                               0x30
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_L                               0x44
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_Z                               0x41
#define GESTURE_C                               0x34
#define FTS_GESTRUE_POINTS                      255
#define FTS_GESTRUE_POINTS_ONETIME              62
#define FTS_GESTRUE_POINTS_HEADER               8
#define FTS_GESTURE_OUTPUT_ADRESS               0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH          4


//int fetch_object_sample(unsigned short *datax,unsigned short *datay,unsigned char *dataxy,short pointnum,unsigned long time_stamp);

int fetch_object_sample(unsigned char *buf,short pointnum);

void init_para(int x_pixel,int y_pixel,int time_slot,int cut_x_pixel,int cut_y_pixel);

//ft_gesture_lib_v1.0_20140820.a

#endif
