/************************************************************************
* Copyright (C) 2012-2016, Focaltech Systems (R)£¬All Rights Reserved.
*
* File Name: focaltech_flash.h
*
*    Author: fupeipei
*
*   Created: 2016-08-07
*
*  Abstract:
*
************************************************************************/
#ifndef __LINUX_FOCALTECH_FLASH_H__
#define __LINUX_FOCALTECH_FLASH_H__

/*******************************************************************************
* 1.Included header files
*******************************************************************************/

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_REG_FW_MAJ_VER                           0xB1
#define FTS_REG_FW_MIN_VER                           0xB2
#define FTS_REG_FW_SUB_MIN_VER                       0xB3
#define FTS_FW_MIN_SIZE                              8
#define FTS_FW_MAX_SIZE                              (54 * 1024)
/* Firmware file is not supporting minor and sub minor so use 0 */
#define FTS_FW_FILE_MAJ_VER(x)                       ((x)->data[(x)->size - 2])
#define FTS_FW_FILE_MIN_VER(x)                       0
#define FTS_FW_FILE_SUB_MIN_VER(x)                   0
#define FTS_FW_FILE_VENDOR_ID(x)                     ((x)->data[(x)->size - 1])
#define FTS_FW_FILE_MAJ_VER_FT6X36(x)                ((x)->data[0x10a])
#define FTS_FW_FILE_VENDOR_ID_FT6X36(x)              ((x)->data[0x108])
#define FTS_MAX_TRIES                                5
#define FTS_RETRY_DLY                                20
#define FTS_MAX_WR_BUF                               10
#define FTS_MAX_RD_BUF                               2
#define FTS_FW_PKT_META_LEN                          6
#define FTS_FW_PKT_DLY_MS                            20
#define FTS_FW_LAST_PKT                              0x6ffa
#define FTS_EARSE_DLY_MS                             100
#define FTS_55_AA_DLY_NS                             5000
#define FTS_CAL_START                                0x04
#define FTS_CAL_FIN                                  0x00
#define FTS_CAL_STORE                                0x05
#define FTS_CAL_RETRY                                100
#define FTS_REG_CAL                                  0x00
#define FTS_CAL_MASK                                 0x70
#define FTS_BLOADER_SIZE_OFF                         12
#define FTS_BLOADER_NEW_SIZE                         30
#define FTS_DATA_LEN_OFF_OLD_FW                      8
#define FTS_DATA_LEN_OFF_NEW_FW                      14
#define FTS_FINISHING_PKT_LEN_OLD_FW                 6
#define FTS_FINISHING_PKT_LEN_NEW_FW                 12
#define FTS_MAGIC_BLOADER_Z7                         0x7bfa
#define FTS_MAGIC_BLOADER_LZ4                        0x6ffa
#define FTS_MAGIC_BLOADER_GZF_30                     0x7ff4
#define FTS_MAGIC_BLOADER_GZF                        0x7bf4
#define FTS_REG_ECC                                  0xCC
#define FTS_RST_CMD_REG2                             0xBC
#define FTS_READ_ID_REG                              0x90
#define FTS_ERASE_APP_REG                            0x61
#define FTS_ERASE_PARAMS_CMD                         0x63
#define FTS_FW_WRITE_CMD                             0xBF
#define FTS_REG_RESET_FW                             0x07
#define FTS_RST_CMD_REG1                             0xFC
#define FTS_FACTORYMODE_VALUE                        0x40
#define FTS_WORKMODE_VALUE                           0x00
#define FTS_APP_INFO_ADDR                            0xd7f8
#define LEN_FLASH_ECC_MAX                            0xFFFE

#define BL_VERSION_LZ4                               0
#define BL_VERSION_Z7                                1
#define BL_VERSION_GZF                               2
#define FTS_REG_ID                                   0xA3
#define FTS_REG_FW_VENDOR_ID                         0xA8

#define FTS_PACKET_LENGTH                            128
#define FTS_SETTING_BUF_LEN                          128

#define FTS_UPGRADE_LOOP                             30
#define FTS_MAX_POINTS_2                             2
#define FTS_MAX_POINTS_5                             5
#define FTS_MAX_POINTS_10                            10
#define AUTO_CLB_NEED                                1
#define AUTO_CLB_NONEED                              0
#define FTS_UPGRADE_AA                               0xAA
#define FTS_UPGRADE_55                               0x55
#define HIDTOI2C_DISABLE                             0
#define FTXXXX_INI_FILEPATH_CONFIG                   "/sdcard/"

#define FTS_RUN_IN_APP                               0x01
#define FTS_RUN_IN_ROM                               0x02
#define FTS_RUN_IN_PRAM                              0x03

#define FTS_INI_FILE_PATH                            "/sdcard/TPD_Error.txt"

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
extern  struct wake_lock ps_lock;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_6336GU_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_5822_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_8606_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8606_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8607_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8607_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8716_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8716_writepram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_3x07_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_i2c_hid2std(struct i2c_client * client);
void fts_rom_or_pram_reset(struct i2c_client * client);
static unsigned char fts_flash_get_pram_or_rom_id(struct i2c_client *client);

#endif
