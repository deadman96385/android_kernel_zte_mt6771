/************************************************************************
* Copyright (C) 2012-2015, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_test_supported_ic.c
*
* Author: Software Development
*
* Created: 2016-08-01
*
* Abstract: test item for FT8716
*
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/


#include "../include/focaltech_test_supported_ic.h"

void (*OnInit_TestItem)(char *);
void (*OnInit_BasicThreshold)(char *);
void (*SetTestItem)(void);
boolean (*Start_Test)(void);
int (*Get_test_data)(char *);
int (*Get_test_node_data)(int *, int , int, int);

#if (FT8607_TEST)
void OnInit_FT8607_TestItem(char*  strIniFile);
void OnInit_FT8607_BasicThreshold(char* strIniFile);
void SetTestItem_FT8607(void);
boolean FT8607_StartTest(void);
int FT8607_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80


void (*OnInit_TestItem)(char *) = OnInit_FT8607_TestItem;
void (*OnInit_BasicThreshold)(char *) = OnInit_FT8607_BasicThreshold;
void (*SetTestItem)(void) = SetTestItem_FT8607;
boolean (*Start_Test)(void) = FT8607_StartTest;
int (*Get_test_data)(char *) = FT8607_get_test_data;
int (*Get_test_node_data)(int *, int , int , int ) = NULL;

#elif (FT8716_TEST)

boolean FT8716_StartTest(void);
int FT8716_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

void OnInit_FT8716_TestItem(char *strIniFile);
void OnInit_FT8716_BasicThreshold(char *strIniFile);
void SetTestItem_FT8716(void);

void (*OnInit_TestItem)(char *) = OnInit_FT8716_TestItem;
void (*OnInit_BasicThreshold)(char *) = OnInit_FT8716_BasicThreshold;
void (*SetTestItem)(void) = SetTestItem_FT8716;
boolean (*Start_Test)(void) = FT8716_StartTest;
int (*Get_test_data)(char *) = FT8716_get_test_data;
int (*Get_test_node_data)(int *, int , int , int ) = NULL;

#elif (FT5X46_TEST)

boolean FT5X46_StartTest(void);
int FT5X46_get_test_data(char *pTestData);//pTestData, External application for memory, buff size >= 1024*80

void OnInit_FT5X46_TestItem(char *strIniFile);
void OnInit_FT5X46_BasicThreshold(char *strIniFile);
void SetTestItem_FT5X46(void);
int FT5X46_get_node_data(int *data, int length, int cmd, int type);

void (*OnInit_TestItem)(char *) = OnInit_FT5X46_TestItem;
void (*OnInit_BasicThreshold)(char *) = OnInit_FT5X46_BasicThreshold;
void (*SetTestItem)(void) = SetTestItem_FT5X46;
boolean (*Start_Test)(void) = FT5X46_StartTest;
int (*Get_test_data)(char *) = FT5X46_get_test_data;
int (*Get_test_node_data)(int *, int , int, int ) = FT5X46_get_node_data;

#endif


