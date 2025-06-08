#ifndef TLV493D_H
#define TLV493D_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>


#define TLV493D_I2C_ADDR 0x5E
#define TLV493D_BUSIF_READSIZE 6
// #define SCALE_FACTOR 0.098  // mT per LSB
// #define SENSITIVITY 5.0  // カーソル移動の感度

/* TLV493D構造体 */
struct tlv493d_config {
	struct i2c_dt_spec i2c;  /* I2C設定情報 */
};

struct tlv493d_data {
	int16_t x, y, z;  /* 3軸の磁気データ */
};

// struct tlv493d_data {
//     const struct device *i2c_dev;
//     int16_t rx;
//     int16_t ry;
//     int16_t rz;
//     int16_t x;
//     int16_t y;
//     int16_t z;
// };


int tlv493d_init(const struct device *dev);
// int tlv493d_read_data(const struct device *dev, struct tlv493d_data *data);
// void process_sensor_data(struct tlv493d_data *data);  // データ処理用関数
int tlv493d_update_data(struct tlv493d_data *data);
int tlv493d_set_power_state(const struct device *dev, bool active);  /* スリープ制御 */


// float tlv493d_convert_to_mT(int16_t raw_value);
// float tlv493d_convert_to_cursor_movement(float magnetic_field);

#endif /* TLV493D_H */
