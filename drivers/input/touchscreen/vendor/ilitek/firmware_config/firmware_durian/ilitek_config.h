/*
 * Ilitek TouchScreen oem config.
 */

 #ifndef _ILITEK_CONFIG_H_
 #define _ILITEK_CONFIG_H_

/* Options */
#define TDDI_INTERFACE			BUS_I2C /* BUS_I2C(0x18) or BUS_SPI(0x1C) */
#define VDD_VOLTAGE			1800000
#define VCC_VOLTAGE			1800000
#define SPI_CLK				(10*M)
#define SPI_RETRY			5
#define IRQ_GPIO_NUM			66
#define WQ_ESD_DELAY			4000
#define WQ_BAT_DELAY			2000
#define MT_B_TYPE			ENABLE
#define TDDI_RST_BIND			DISABLE
#define MT_PRESSURE			DISABLE
#define ENABLE_WQ_ESD			DISABLE
#define ENABLE_WQ_BAT			DISABLE
#define ENABLE_GESTURE			DISABLE
#define READ_GL_INFO			DISABLE
#define REGULATOR_POWER			DISABLE
#define TP_SUSPEND_PRIO			ENABLE
#define TP_X_SWAP

/* Plaform compatibility */
/* #define CONFIG_PLAT_SPRD */
#define SPI_DMA_TRANSFER_SPLIT

 /* define the width and heigth of a screen. */
#define TOUCH_SCREEN_X_MIN			0
#define TOUCH_SCREEN_Y_MIN			0
#define TOUCH_SCREEN_X_MAX			719
#define TOUCH_SCREEN_Y_MAX			1559
#define MAX_TOUCH_NUM				10

/* define the range on panel */
#define TPD_HEIGHT				2048
#define TPD_WIDTH				2048

#define ILI_UPGRADE_FW_FILE		"empty_fw.ili"
#endif

