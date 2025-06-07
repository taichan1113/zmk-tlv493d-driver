#include <stdint.h>
#include <stdlib.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "../include/tlv493d.h"

LOG_MODULE_REGISTER(tlv493d, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT infineon_tlv493d


// ====================================================================



#define HYSTERESIS_HIGH_THRESHOLD 15  // 動きを検出する上側しきい値
#define HYSTERESIS_LOW_THRESHOLD  5   // 動きを停止する下側しきい値

#define TLV493D_LOG_INTERVAL_MS 3000
#define TLV493D_SENSITIVITY_DIVISOR 10
#define TLV493D_CALIBRATION_DELAY_MS 50

#define GRAPH_WIDTH 40     // グラフの最大幅
#define GRAPH_SCALE 100    // グラフのスケール（値をこの値で割って正規化）

struct tlv493d_data {
    const struct device *dev;
    struct k_work_delayable work;
    int16_t last_x;
    int16_t last_y;
    int16_t last_z;
    bool sleep_mode;
    uint32_t log_timer;  // Add timer for logging
    bool active_x;        // X軸の動作状態
    bool active_y;        // Y軸の動作状態
    float scale;         // 変換係数
    bool calibrated;         // Add missing member
    bool movement_active;    // Add missing member
};

struct tlv493d_config {
    struct i2c_dt_spec i2c;
};

static int tlv493d_read_sensor_data(const struct device *dev, int16_t *x, int16_t *y, int16_t *z) {
    const struct tlv493d_config *config = dev->config;
    uint8_t data[6];
    int ret;
    
    // バースト読み出し（6バイト: BX, BY, BZ, TEMP, BX2, BZ2）
    ret = i2c_burst_read_dt(&config->i2c, TLV493D_REG_B_X, data, sizeof(data));
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d", ret);
        return ret;
    }

    // データシートに従って12ビット値に変換
    *x = ((data[0] << 4) | ((data[4] & 0xF0) >> 4));
    *y = ((data[1] << 4) | (data[4] & 0x0F));
    *z = ((data[2] << 4) | ((data[5] & 0xF0) >> 4));

    // 12ビット符号付き整数への変換
    if (*x & 0x800) *x |= 0xF000;
    if (*y & 0x800) *y |= 0xF000;
    if (*z & 0x800) *z |= 0xF000;

    return 0;
}

static int convert_to_mouse_movement(float value, bool *is_active) {
    if (!*is_active && (abs(value) > TLV493D_THRESHOLD)) {
        *is_active = true;
    } else if (*is_active && (abs(value) < (TLV493D_THRESHOLD - TLV493D_HYSTERESIS))) {
        *is_active = false;
    }

    if (*is_active) {
        float movement = value * TLV493D_SCALE;
        return (int)CLAMP(movement, -10, 10);
    }
    return 0;
}

static void log_bar_graph(const char *label, int16_t value) {
    char bar[GRAPH_WIDTH + 1] = {0};
    int bars = (abs(value) * GRAPH_WIDTH) / GRAPH_SCALE;
    if (bars > GRAPH_WIDTH) bars = GRAPH_WIDTH;

    // 棒グラフの作成
    if (value > 0) {
        memset(bar, '>', bars);
    } else {
        memset(bar, '<', bars);
    }
    
    LOG_INF("%s: %4d [%c%s%*s]", label, value, 
            value >= 0 ? '|' : ' ', bar, 
            GRAPH_WIDTH - bars, "");
}

static void tlv493d_work_cb(struct k_work *work) {
    struct tlv493d_data *data = CONTAINER_OF(work, struct tlv493d_data, work.work);
    int16_t x, y, z;
    
    if (data->sleep_mode) {
        // Skip reading when in sleep mode
        k_work_reschedule(&data->work, K_MSEC(CONFIG_INPUT_TLV493D_POLLING_INTERVAL_MS));
        return;
    }

    if (tlv493d_read_sensor_data(data->dev, &x, &y, &z) == 0) {
        float x_val = (float)x;
        float y_val = (float)y;
        
        int dx = convert_to_mouse_movement(x_val, &data->active_x);
        int dy = convert_to_mouse_movement(y_val, &data->active_y);

        // Log sensor values
        uint32_t now = k_uptime_get_32();
        if ((now - data->log_timer) >= TLV493D_LOG_INTERVAL_MS) {
            LOG_INF("Sensor: X=%d(%d) Y=%d(%d) Z=%d [%s]", 
                   x, dx, y, dy, z,
                   (data->active_x || data->active_y) ? "ACTIVE" : "IDLE");
            data->log_timer = now;
        }

        if (dx != 0 || dy != 0) {
            input_report_rel(data->dev, INPUT_REL_X, dx, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_FOREVER);
        }
    }

    k_work_reschedule(&data->work, K_MSEC(CONFIG_INPUT_TLV493D_POLLING_INTERVAL_MS));
}

int tlv493d_set_sleep(const struct device *dev, bool sleep) {
    struct tlv493d_data *data = dev->data;
    const struct tlv493d_config *config = dev->config;
    uint8_t mode;

    data->sleep_mode = sleep;
    
    // Set power mode according to sleep state
    mode = sleep ? TLV493D_MODE_LOW_POWER : TLV493D_MODE_MCM_FAST;
    
    return i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_B_MOD1, mode);
}

static int tlv493d_reset(const struct device *dev) {
    const struct tlv493d_config *config = dev->config;
    int ret;

    // Power down mode
    ret = i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_B_MOD1, TLV493D_MODE_DISABLE);
    if (ret < 0) return ret;
    k_sleep(K_MSEC(TLV493D_RESET_DELAY_MS));

    // Fast mode + MCM configuration
    uint8_t config_val = TLV493D_CONFIG_FAST;  // Fast mode, temp disabled
    ret = i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_CONFIG, config_val);
    if (ret < 0) return ret;

    // Enable MCM mode
    ret = i2c_reg_write_byte_dt(&config->i2c, TLV493D_REG_B_MOD1, TLV493D_MODE_MCM_FAST);
    if (ret < 0) return ret;

    k_sleep(K_MSEC(TLV493D_POWER_UP_DELAY_MS));
    return 0;
}

int tlv493d_calibrate(const struct device *dev) {
    struct tlv493d_data *data = dev->data;
    int16_t x, y, z;
    int ret;

    // Wait for sensor to stabilize after power up
    k_sleep(K_MSEC(TLV493D_CALIBRATION_DELAY_MS));

    // Read initial values
    ret = tlv493d_read_sensor_data(dev, &x, &y, &z);
    if (ret < 0) {
        LOG_ERR("Failed to read calibration data: %d", ret);
        return ret;
    }

    // Store initial values as reference
    data->last_x = x;
    data->last_y = y;
    data->last_z = z;

    LOG_INF("Calibration values - X: %d, Y: %d, Z: %d", x, y, z);
    data->calibrated = true;

    return 0;
}

static int tlv493d_init(const struct device *dev) {
    struct tlv493d_data *data = dev->data;
    const struct tlv493d_config *config = dev->config;
    int ret;

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // デバイスの初期化
    ret = tlv493d_reset(dev);
    if (ret < 0) {
        return ret;
    }

    data->dev = dev;
    data->sleep_mode = false;
    data->log_timer = k_uptime_get_32();
    data->calibrated = false;
    data->movement_active = false;  // 初期状態は非アクティブ

    // センサーのキャリブレーションを実行
    ret = tlv493d_calibrate(dev);
    if (ret < 0) {
        LOG_ERR("Failed to calibrate sensor");
        return ret;
    }

    // Initialize and start polling work
    k_work_init_delayable(&data->work, tlv493d_work_cb);
    k_work_schedule(&data->work, K_MSEC(CONFIG_INPUT_TLV493D_POLLING_INTERVAL_MS));

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int tlv493d_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        return tlv493d_set_sleep(dev, true);
    case PM_DEVICE_ACTION_RESUME:
        return tlv493d_set_sleep(dev, false);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define TLV493D_INIT(n) \
    static struct tlv493d_data tlv493d_data_##n; \
    static const struct tlv493d_config tlv493d_config_##n = { \
        .i2c = I2C_DT_SPEC_INST_GET(n), \
    }; \
    PM_DEVICE_DT_INST_DEFINE(n, tlv493d_pm_action); \
    DEVICE_DT_INST_DEFINE(n, tlv493d_init, \
                         PM_DEVICE_DT_INST_GET(n), \
                         &tlv493d_data_##n, \
                         &tlv493d_config_##n, \
                         POST_KERNEL, \
                         CONFIG_INPUT_TLV493D_INIT_PRIORITY, \
                         NULL);

DT_INST_FOREACH_STATUS_OKAY(TLV493D_INIT)

