#pragma once

#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#ifndef ZEPHYR_INCLUDE_INPUT_TLV493D_H_
#define ZEPHYR_INCLUDE_INPUT_TLV493D_H_


// ====================================================================


/* Logging configuration */
#define TLV493D_LOG_INTERVAL_MS   3000   // ログ出力間隔(ms)

/* Sensor parameters */
#define TLV493D_THRESHOLD        5.0f    // 動作開始閾値
#define TLV493D_HYSTERESIS      2.0f    // ヒステリシス幅
#define TLV493D_SCALE           0.2f    // 磁気値からの変換係数

/* Configuration values */
#define TLV493D_POWER_UP_DELAY_MS  250   // パワーアップ待ち時間(ms)
#define TLV493D_RESET_DELAY_MS     100   // リセット後の待ち時間(ms)

/* TLX493D Register addresses and masks */
#define TLV493D_ADDR_DEFAULT    0x5E    /* Default I2C address */

/* Register map */
#define TLV493D_REG_B_X        0x00    /* X magnetic data */
#define TLV493D_REG_B_Y        0x01    /* Y magnetic data */
#define TLV493D_REG_B_Z        0x02    /* Z magnetic data */
#define TLV493D_REG_TEMP       0x03    /* Temperature data */
#define TLV493D_REG_BX2        0x04    /* X magnetic data LSBs */
#define TLV493D_REG_BZ2        0x05    /* Z magnetic data LSBs */
#define TLV493D_REG_CONFIG     0x10    /* Configuration register */
#define TLV493D_REG_B_MOD1     0x11    /* Mode register 1 */

/* Mode configurations */
#define TLV493D_MODE_DISABLE    0x00    /* Power down mode */
#define TLV493D_MODE_MCM        0x11    /* Master Controlled Mode */
#define TLV493D_MODE_MCM_FAST   0x13    /* Master Controlled Mode + Fast */
#define TLV493D_MODE_LOW_POWER  0x03    /* Low power mode */

/* Configuration bits */
#define TLV493D_CONFIG_T        BIT(7)  /* Temperature measurement enable */
#define TLV493D_CONFIG_LP       BIT(6)  /* Low power mode enable */
#define TLV493D_CONFIG_FAST     BIT(4)  /* Fast mode enable */
#define TLV493D_CONFIG_INT      BIT(3)  /* Interrupt enable */

/* Default configuration */
#define TLV493D_DEFAULT_CONFIG    (TLV493D_CONFIG_FAST)  /* Fast mode only */

/* Function Declarations */
int tlv493d_set_sleep(const struct device *dev, bool sleep);
int tlv493d_calibrate(const struct device *dev);

