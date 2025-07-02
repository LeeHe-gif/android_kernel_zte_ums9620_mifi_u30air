#ifndef CUSTOM_FUNC_H__
#define CUSTOM_FUNC_H__

/* #define SITRONIX_INTERFACE_I2C */
#define SITRONIX_INTERFACE_SPI
/* #define SITRONIX_TP_WITH_FLASH */
#define ST_UPGRADE_USE_REQUESTFW_BUF
#define SITRONIX_DEFAULT_FIRMWARE        "sitronix_6_517_default_firmware"
#define SITRONIX_MONITOR_THREAD
#define STP_X_CHS  14
#define STP_Y_CHS  24
#define STP_N_CHS   4
#define ST_DEFAULT_RES_X		480
#define ST_DEFAULT_RES_Y		854
#define ST_DEFAULT_MAX_TOUCH	2

#define STS_REPORT_BY_ZTE_ALGO

#define SKIP_TPD_SELFTEST

#define STP_VENDOR_ID_0 0x1b
#define STP_VENDOR_ID_1 0x6c
#define STP_VENDOR_ID_2 0x00
#define STP_VENDOR_ID_3 0x00
#define STP_VENDOR_0_NAME                         "easyquick"
#define STP_VENDOR_1_NAME                         "unknown"
#define STP_VENDOR_2_NAME                         "unknown"
#define STP_VENDOR_3_NAME                         "unknown"
#endif /* CUSTOM_FUNC_H__ */
