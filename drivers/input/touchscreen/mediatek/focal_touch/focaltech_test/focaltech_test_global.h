/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_global.h
*
* Author: Software Development Team, AE
*
* Created: 2016-08-01
*
* Abstract: global function for test
*
************************************************************************/
#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "../focaltech_global/focaltech_global.h"
#include "include/focaltech_test_main.h"


/*-----------------------------------------------------------
IC Type Test
-----------------------------------------------------------*/

#define FT3C47_TEST     0
#define FT5822_TEST     0
#define FT5X46_TEST     1
#define FT6X36_TEST     0
#define FT8606_TEST     0
#define FT8607_TEST     0
#define FT8716_TEST     0
#define FT8736_TEST     0
#define FTE716_TEST     0
#define FTE736_TEST     0

/*-----------------------------------------------------------
IC Capacitance Type
-----------------------------------------------------------*/
#define IC_Capacitance_Type     1//0:Self Capacitance, 1:Mutual Capacitance, 2:IDC
enum enum_Report_Protocol_Type
{
    Self_Capacitance = 0,
    Mutual_Capacitance= 1,
    IDC_Capacitance = 2,
};

#endif
