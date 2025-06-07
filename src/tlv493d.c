#include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "../include/tlv493d.h"

static struct tlv493d_data sensor_data;
static struct k_timer tlv493d_timer;

static void tlv493d_timer_handler(struct k_timer *timer) {
    tlv493d_read_data(device_get_binding("TLV493D"), &sensor_data);
    process_sensor_data(&sensor_data);  // 切り分けたデータ処理関数を呼び出す
    // printk("TLV493D Data: RX=%d, RY=%d, RZ=%d, X=%d, Y=%d, Z=%d\n",
        //    tlv493d_convert_to_mT(sensor_data.rx), tlv493d_convert_to_mT(sensor_data.ry), tlv493d_convert_to_mT(sensor_data.rz),
        //    tlv493d_convert_to_mT(sensor_data.x), tlv493d_convert_to_mT(sensor_data.y), tlv493d_convert_to_mT(sensor_data.z));

}

int tlv493d_init(const struct device *dev) {
    struct tlv493d_data *data = dev->data;
    data->i2c_dev = device_get_binding(DT_LABEL(DT_NODELABEL(i2c0)));

    if (!data->i2c_dev) {
        return -ENODEV;
    }

    k_timer_init(&tlv493d_timer, tlv493d_timer_handler, NULL);
    k_timer_start(&tlv493d_timer, K_MSEC(CONFIG_TLV493D_POLL_RATE_MS), K_MSEC(CONFIG_TLV493D_POLL_RATE_MS));

    return 0;
}

int tlv493d_read_data(const struct device *dev, struct tlv493d_data *data) {
    uint8_t buf[7];
    if (i2c_read(data->i2c_dev, buf, sizeof(buf), TLV493D_I2C_ADDR) < 0) {
        return -EIO;
    }

    // data->rx = (buf[0] << 4) | (buf[4] & 0x0F);  // 回転X
    // data->ry = (buf[1] << 4) | (buf[5] & 0x0F);  // 回転Y
    // data->rz = (buf[2] << 4) | (buf[6] & 0x0F);  // 回転Z
    // data->x  = (buf[0] << 4) | (buf[4] & 0x0F);  // 移動X
    // data->y  = (buf[1] << 4) | (buf[5] & 0x0F);  // 移動Y
    // data->z  = (buf[2] << 4) | (buf[6] & 0x0F);  // 移動Z

    // 直線移動用のデータ（X, Y, Z）
    data->x  = (buf[0] << 4) | (buf[4] & 0x0F);
    data->y  = (buf[1] << 4) | (buf[5] & 0x0F);
    data->z  = (buf[2] << 4) | (buf[6] & 0x0F);

    // 回転データ（RX, RY, RZ）は計算が必要（仮の例）
    data->rx = (data->y - data->z); // YとZの磁場差を回転Xとして使用
    data->ry = (data->z - data->x); // ZとXの磁場差を回転Yとして使用
    data->rz = (data->x - data->y); // XとYの磁場差を回転Zとして使用

    return 0;
}

void process_sensor_data(struct tlv493d_data *data) {
    // 生データをmTに変換
    data->rx = tlv493d_convert_to_mT(data->rx);
    data->ry = tlv493d_convert_to_mT(data->ry);
    data->rz = tlv493d_convert_to_mT(data->rz);
    data->x  = tlv493d_convert_to_mT(data->x);
    data->y  = tlv493d_convert_to_mT(data->y);
    data->z  = tlv493d_convert_to_mT(data->z);

    // 変換後のデータをカーソル移動量へ変換
    // 今後、データフィルタリング、平均化、カスタムマッピングを追加する
    data->rx = tlv493d_convert_to_cursor_movement(data->rx);
    data->ry = tlv493d_convert_to_cursor_movement(data->ry);
    data->rz = tlv493d_convert_to_cursor_movement(data->rz);
    data->x  = tlv493d_convert_to_cursor_movement(data->x);
    data->y  = tlv493d_convert_to_cursor_movement(data->y);
    data->z  = tlv493d_convert_to_cursor_movement(data->z);

}

float tlv493d_convert_to_mT(int16_t raw_value) {
    return raw_value * SCALE_FACTOR;
}

float tlv493d_convert_to_cursor_movement(float magnetic_field) {
    return magnetic_field * SENSITIVITY;
}


DEVICE_DEFINE(tlv493d, "TLV493D", tlv493d_init, NULL, NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);
