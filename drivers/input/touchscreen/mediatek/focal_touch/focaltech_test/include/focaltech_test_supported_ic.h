/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R) All Rights Reserved.
*
* File Name: focaltech_test_supported_ic.h
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

#include "../focaltech_test_global.h"

#if (FT8716_TEST)

struct stCfg_FT8716_TestItem
{
    bool FW_VERSION_TEST;
    bool FACTORY_ID_TEST;
    bool PROJECT_CODE_TEST;
    bool IC_VERSION_TEST;
    bool RAWDATA_TEST;
    bool CHANNEL_NUM_TEST;
    bool INT_PIN_TEST;
    bool RESET_PIN_TEST;
    bool NOISE_TEST;
    bool CB_TEST;
    bool SHORT_CIRCUIT_TEST;
    bool OPEN_TEST;
    bool CB_UNIFORMITY_TEST;
    bool DIFFER_UNIFORMITY_TEST;
    bool DIFFER2_UNIFORMITY_TEST;

};
struct stCfg_FT8716_BasicThreshold
{
    BYTE FW_VER_VALUE;
    BYTE Factory_ID_Number;
    char Project_Code[32];
    BYTE IC_Version;
    int RawDataTest_Min;
    int RawDataTest_Max;
    BYTE ChannelNumTest_ChannelXNum;
    BYTE ChannelNumTest_ChannelYNum;
    BYTE ChannelNumTest_KeyNum;
    BYTE ResetPinTest_RegAddr;
    BYTE IntPinTest_RegAddr;
    int NoiseTest_Coefficient;
    int NoiseTest_Frames;
    int NoiseTest_Time;
    BYTE NoiseTest_SampeMode;
    BYTE NoiseTest_NoiseMode;
    BYTE NoiseTest_ShowTip;
    bool bCBTest_VA_Check;
    int CbTest_Min;
    int CbTest_Max;
    bool bCBTest_VKey_Check;
    int CbTest_Min_Vkey;
    int CbTest_Max_Vkey;

    int ShortCircuit_ResMin;
    //int ShortTest_K2Value;
    int OpenTest_CBMin;

    bool CBUniformityTest_Check_CHX;
    bool CBUniformityTest_Check_CHY;
    bool CBUniformityTest_Check_MinMax;
    int CBUniformityTest_CHX_Hole;
    int CBUniformityTest_CHY_Hole;
    int CBUniformityTest_MinMax_Hole;

    bool DifferUniformityTest_Check_CHX;
    bool DifferUniformityTest_Check_CHY;
    bool DifferUniformityTest_Check_MinMax;
    int DifferUniformityTest_CHX_Hole;
    int DifferUniformityTest_CHY_Hole;
    int DifferUniformityTest_MinMax_Hole;
    int DeltaVol;

    bool Differ2UniformityTest_Check_CHX;
    bool Differ2UniformityTest_Check_CHY;
    int Differ2UniformityTest_CHX_Hole;
    int Differ2UniformityTest_CHY_Hole;
    int Differ2UniformityTest_Differ_Min;
    int Differ2UniformityTest_Differ_Max;

};
enum enumTestItem_FT8716
{
    Code_FT8716_ENTER_FACTORY_MODE,//All IC are required to test items
    Code_FT8716_DOWNLOAD,//All IC are required to test items
    Code_FT8716_UPGRADE,//All IC are required to test items
    Code_FT8716_FACTORY_ID_TEST,
    Code_FT8716_PROJECT_CODE_TEST,
    Code_FT8716_FW_VERSION_TEST,
    Code_FT8716_IC_VERSION_TEST,
    Code_FT8716_RAWDATA_TEST,
    Code_FT8716_CHANNEL_NUM_TEST,
    //Code_FT8716_CHANNEL_SHORT_TEST,
    Code_FT8716_INT_PIN_TEST,
    Code_FT8716_RESET_PIN_TEST,
    Code_FT8716_NOISE_TEST,
    Code_FT8716_CB_TEST,
    //Code_FT8716_DELTA_CB_TEST,
    //Code_FT8716_CHANNELS_DEVIATION_TEST,
    //Code_FT8716_TWO_SIDES_DEVIATION_TEST,
    //Code_FT8716_FPC_SHORT_TEST,
    //Code_FT8716_FPC_OPEN_TEST,
    //Code_FT8716_SREF_OPEN_TEST,
    //Code_FT8716_TE_TEST,
    //Code_FT8716_CB_DEVIATION_TEST,
    Code_FT8716_WRITE_CONFIG,//All IC are required to test items
    //Code_FT8716_DIFFER_TEST,
    Code_FT8716_SHORT_CIRCUIT_TEST,
    Code_FT8716_OPEN_TEST,
    Code_FT8716_CB_UNIFORMITY_TEST,
    Code_FT8716_DIFFER_UNIFORMITY_TEST,
    Code_FT8716_DIFFER2_UNIFORMITY_TEST,
};



#elif (FT8607_TEST)

struct stCfg_FT8607_TESTItem
{
    bool FW_VERSION_TEST;
    bool FACTORY_ID_TEST;
    bool PROJECT_CODE_TEST;
    bool IC_VERSION_TEST;
    bool RAWDATA_TEST;
    bool CHANNEL_NUM_TEST;
    bool INT_PIN_TEST;
    bool RESET_PIN_TEST;
    bool NOISE_TEST;
    bool CB_TEST;
    bool SHORT_CIRCUIT_TEST;
    bool LCD_NOISE_TEST;
    bool OSC60MHZ_TEST;
    bool OSCTRM_TEST;
    bool SNR_TEST;
    bool LPWG_RAWDATA_TEST;
    bool LPWG_CB_TEST;
    bool LPWG_NOISE_TEST;
    bool DIFFER_TEST;
    bool DIFFER_UNIFORMITY_TEST;
};

struct stCfg_FT8607_BasicThreshold
{
    BYTE FW_VER_VALUE;
    BYTE Factory_ID_Number;
    char Project_Code[32];
    BYTE IC_Version;
    int RawDataTest_Min;
    int RawDataTest_Max;
    BYTE ChannelNumTest_ChannelXNum;
    BYTE ChannelNumTest_ChannelYNum;
    BYTE ChannelNumTest_KeyNum;
    BYTE ResetPinTest_RegAddr;
    BYTE IntPinTest_RegAddr;
    int NoiseTest_Coefficient;
    int NoiseTest_Frames;
    int NoiseTest_Time;
    BYTE NoiseTest_SampeMode;
    BYTE NoiseTest_NoiseMode;
    BYTE NoiseTest_ShowTip;
    BYTE IsDifferMode;
    bool bCBTest_VA_Check;
    int CbTest_Min;
    int CbTest_Max;
    bool bCBTest_VKey_Check;
    int CbTest_Min_Vkey;
    int CbTest_Max_Vkey;
    int ShortCircuit_ResMin;
    /*int ShortTest_Max;
    int ShortTest_K2Value;
    bool ShortTest_Tip;*/
    int iLCDNoiseTestFrame;
    int iLCDNoiseTestMax;

    int iLCDNoiseCoefficient;
    int OSC60MHZTest_OSCMin;
    int OSC60MHZTest_OSCMax;

    int OSCTRMTest_OSCMin;
    int OSCTRMTest_OSCMax;
    int OSCTRMTest_OSCDetMin;
    int OSCTRMTest_OSCDetMax;
    int SNRTest_FrameNum;
    int SNRTest_Min;

    int DIFFERTest_FrameNum;
    int DIFFERTest_DifferMax;
    int DIFFERTest_DifferMin;

    bool DifferUniformityTest_Check_CHX;
    bool DifferUniformityTest_Check_CHY;
    bool DifferUniformityTest_Check_MinMax;
    int DifferUniformityTest_CHX_Hole;
    int DifferUniformityTest_CHY_Hole;
    int DifferUniformityTest_MinMax_Hole;

    int LPWG_RawDataTest_Min;
    int LPWG_RawDataTest_Max;

    bool bLPWG_CBTest_VA_Check;
    int LPWG_CbTest_Min;
    int LPWG_CbTest_Max;
    bool bLPWG_CBTest_VKey_Check;
    int LPWG_CbTest_Min_Vkey;
    int LPWG_CbTest_Max_Vkey;

    int LPWG_NoiseTest_Coefficient;
    int LPWG_NoiseTest_Frames;
    int LPWG_NoiseTest_Time;
    BYTE LPWG_NoiseTest_SampeMode;
    BYTE LPWG_NoiseTest_NoiseMode;
    BYTE LPWG_NoiseTest_ShowTip;
    BYTE LPWG_IsDifferMode;
};


enum enumTestItem_FT8607
{
    Code_FT8607_ENTER_FACTORY_MODE,//All IC are required to test items
    Code_FT8607_DOWNLOAD,//All IC are required to test items
    Code_FT8607_UPGRADE,//All IC are required to test items
    Code_FT8607_FACTORY_ID_TEST,
    Code_FT8607_PROJECT_CODE_TEST,
    Code_FT8607_FW_VERSION_TEST,
    Code_FT8607_IC_VERSION_TEST,
    Code_FT8607_RAWDATA_TEST,
    Code_FT8607_CHANNEL_NUM_TEST,
    //Code_FT8607_CHANNEL_SHORT_TEST,
    Code_FT8607_INT_PIN_TEST,
    Code_FT8607_RESET_PIN_TEST,
    Code_FT8607_NOISE_TEST,
    Code_FT8607_CB_TEST,
    //Code_FT8607_DELTA_CB_TEST,
    //Code_FT8607_CHANNELS_DEVIATION_TEST,
    //Code_FT8607_TWO_SIDES_DEVIATION_TEST,
    //Code_FT8607_FPC_SHORT_TEST,
    //Code_FT8607_FPC_OPEN_TEST,
    //Code_FT8607_SREF_OPEN_TEST,
    //Code_FT8607_TE_TEST,
    //Code_FT8607_CB_DEVIATION_TEST,
    Code_FT8607_WRITE_CONFIG,//All IC are required to test items
    //Code_FT8607_DIFFER_TEST,
    Code_FT8607_SHORT_CIRCUIT_TEST,
    Code_FT8607_LCD_NOISE_TEST,

    Code_FT8607_OSC60MHZ_TEST,
    Code_FT8607_OSCTRM_TEST,
    Code_FT8607_SNR_TEST,
    Code_FT8607_DIFFER_TEST,
    Code_FT8607_DIFFER_UNIFORMITY_TEST,

    Code_FT8607_LPWG_RAWDATA_TEST,
    Code_FT8607_LPWG_CB_TEST,
    Code_FT8607_LPWG_NOISE_TEST,
};


#elif (FT5X46_TEST)
struct stCfg_FT5X46_TestItem
{
	bool FW_VERSION_TEST;
	bool FACTORY_ID_TEST;
	bool PROJECT_CODE_TEST;
	bool IC_VERSION_TEST;
	bool RAWDATA_TEST;
	bool ADC_DETECT_TEST;
	bool SCAP_CB_TEST;
	bool SCAP_RAWDATA_TEST;
	bool CHANNEL_NUM_TEST;
	bool INT_PIN_TEST;
	bool RESET_PIN_TEST;
	bool NOISE_TEST;
	bool WEAK_SHORT_CIRCUIT_TEST;
	bool UNIFORMITY_TEST;
	bool CM_TEST;

	bool RAWDATA_MARGIN_TEST;
	bool PANEL_DIFFER_TEST;
	bool PANEL_DIFFER_UNIFORMITY_TEST;
	
	bool LCM_ID_TEST;

	bool TE_TEST;
	bool SITO_RAWDATA_UNIFORMITY_TEST;
	bool PATTERN_TEST;
};
struct stCfg_FT5X45_BasicThreshold
{
	BYTE FW_VER_VALUE;
	BYTE Factory_ID_Number;
	char Project_Code[32];
	BYTE IC_Version;
	BYTE LCM_ID;
	int RawDataTest_low_Min;
	int RawDataTest_Low_Max;
	int RawDataTest_high_Min;
	int RawDataTest_high_Max;
	BYTE RawDataTest_SetLowFreq;
	BYTE RawDataTest_SetHighFreq;
	int AdcDetect_Max;
	//int RxShortTest_Min;
	//int RxShortTest_Max;
	//int TxShortTest_Min;
	//int TxShortTest_Max;
	int SCapCbTest_OFF_Min;
	int SCapCbTest_OFF_Max;
	int SCapCbTest_ON_Min;
	int SCapCbTest_ON_Max;
	bool SCapCbTest_LetTx_Disable;
	BYTE SCapCbTest_SetWaterproof_OFF;
	BYTE SCapCbTest_SetWaterproof_ON;
	//int SCapDifferTest_Min;
	//int SCapDifferTest_Max;
	//BYTE SCapDifferTest_CbLevel;
	int SCapRawDataTest_OFF_Min;
	int SCapRawDataTest_OFF_Max;
	int SCapRawDataTest_ON_Min;
	int SCapRawDataTest_ON_Max;
	bool SCapRawDataTest_LetTx_Disable;
	BYTE SCapRawDataTest_SetWaterproof_OFF;
	BYTE SCapRawDataTest_SetWaterproof_ON;
	bool bChannelTestMapping;
	bool bChannelTestNoMapping;
	BYTE ChannelNumTest_TxNum;
	BYTE ChannelNumTest_RxNum;
	BYTE ChannelNumTest_TxNpNum;
	BYTE ChannelNumTest_RxNpNum;
	BYTE ResetPinTest_RegAddr;
	BYTE IntPinTest_RegAddr;
	BYTE IntPinTest_TestNum;
	int NoiseTest_Max;
	int GloveNoiseTest_Coefficient;
	int NoiseTest_Frames;
	int NoiseTest_Time;
	BYTE NoiseTest_SampeMode;
	BYTE NoiseTest_NoiseMode;
	BYTE NoiseTest_ShowTip;
	bool bNoiseTest_GloveMode;
	int NoiseTest_RawdataMin;
	unsigned char Set_Frequency;
	bool bNoiseThreshold_Choose;
	int NoiseTest_Threshold;
	int NoiseTest_MinNgFrame;
	//BYTE SCapClbTest_Frame;
	//int SCapClbTest_Max;
	//int RxCrosstalkTest_Min;
	//int RxCrosstalkTest_Max;
	//int RawDataRxDeviationTest_Max;
	//int RawDataUniformityTest_Percent;
	//int RxLinearityTest_Max;
	//int TxLinearityTest_Max;
	//int DifferDataUniformityTest_Percent;
	int WeakShortTest_CG;
	int WeakShortTest_CC;
	int WeakShortTest_CC_Rsen;
	bool WeakShortTest_CapShortTest;
	
	//int WeakShortTest_ChannelNum;
	bool Uniformity_CheckTx;
	bool Uniformity_CheckRx;
	bool Uniformity_CheckMinMax;
	int  Uniformity_Tx_Hole;
	int  Uniformity_Rx_Hole;
	int  Uniformity_MinMax_Hole;
    bool CMTest_CheckMin;
	bool CMTest_CheckMax;
	int  CMTest_MinHole;
	int  CMTest_MaxHole;

	int RawdataMarginTest_Min;
	int RawdataMarginTest_Max;

	int PanelDifferTest_Min;
	int PanelDifferTest_Max;

	bool PanelDiffer_UniformityTest_Check_Tx;
	bool PanelDiffer_UniformityTest_Check_Rx;
	bool PanelDiffer_UniformityTest_Check_MinMax;
	int  PanelDiffer_UniformityTest_Tx_Hole;
	int  PanelDiffer_UniformityTest_Rx_Hole;
	int  PanelDiffer_UniformityTest_MinMax_Hole;

	bool SITO_RawdtaUniformityTest_Check_Tx;
	bool SITO_RawdtaUniformityTest_Check_Rx;
	int  SITO_RawdtaUniformityTest_Tx_Hole;
	int  SITO_RawdtaUniformityTest_Rx_Hole;

	bool bPattern00;
	bool bPatternFF;
	bool bPattern55;
	bool bPatternAA;
	bool bPatternBin;
};
enum enumTestItem_FT5X46
{
	Code_FT5X46_ENTER_FACTORY_MODE,//All IC are required to test items
	Code_FT5X46_DOWNLOAD,//All IC are required to test items
	Code_FT5X46_UPGRADE,//All IC are required to test items
	Code_FT5X46_FACTORY_ID_TEST,
	Code_FT5X46_PROJECT_CODE_TEST,
	Code_FT5X46_FW_VERSION_TEST,
	Code_FT5X46_IC_VERSION_TEST,
	Code_FT5X46_RAWDATA_TEST,
	Code_FT5X46_ADCDETECT_TEST,
	Code_FT5X46_SCAP_CB_TEST,
	Code_FT5X46_SCAP_RAWDATA_TEST,
	Code_FT5X46_CHANNEL_NUM_TEST,
	Code_FT5X46_INT_PIN_TEST,
	Code_FT5X46_RESET_PIN_TEST,
	Code_FT5X46_NOISE_TEST,
	Code_FT5X46_WEAK_SHORT_CIRCUIT_TEST,
	Code_FT5X46_UNIFORMITY_TEST,
	Code_FT5X46_CM_TEST,
	Code_FT5X46_RAWDATA_MARGIN_TEST,
	Code_FT5X46_WRITE_CONFIG,//All IC are required to test items
	Code_FT5X46_PANELDIFFER_TEST,
	Code_FT5X46_PANELDIFFER_UNIFORMITY_TEST,
	Code_FT5X46_LCM_ID_TEST,
	Code_FT5X46_JUDEG_NORMALIZE_TYPE,
	Code_FT5X46_TE_TEST,
	Code_FT5X46_SITO_RAWDATA_UNIFORMITY_TEST,
    Code_FT5X46_PATTERN_TEST,
};



#endif


