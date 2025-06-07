#ifndef TLV493D_H
#define TLV493D_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define TLV493D_I2C_ADDR 0x5E
#define SCALE_FACTOR 0.098  // mT per LSB
#define SENSITIVITY 5.0  // カーソル移動の感度

struct tlv493d_data {
    const struct device *i2c_dev;
    int16_t rx;
    int16_t ry;
    int16_t rz;
    int16_t x;
    int16_t y;
    int16_t z;
};


int tlv493d_init(const struct device *dev);
int tlv493d_read_data(const struct device *dev, struct tlv493d_data *data);
void process_sensor_data(struct tlv493d_data *data);  // データ処理用関数

float tlv493d_convert_to_mT(int16_t raw_value);
float tlv493d_convert_to_cursor_movement(float magnetic_field);

#endif /* TLV493D_H */
