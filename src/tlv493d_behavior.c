#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/event_manager.h>
#include <zmk/input.h>
// #include <zmk/input_listener.h>

#include "../include/tlv493d_behavior.h"

static int tlv493d_process_event(const struct device *dev, struct tlv493d_data *data) {
    process_sensor_data(data);  // 事前にデータ処理を実施

    // float rel_rx = tlv493d_convert_to_cursor_movement(data->rx);
    // float rel_ry = tlv493d_convert_to_cursor_movement(data->ry);
    // float rel_rz = tlv493d_convert_to_cursor_movement(data->rz);
    // float rel_x = tlv493d_convert_to_cursor_movement(data->x);
    // float rel_y = tlv493d_convert_to_cursor_movement(data->y);
    // float rel_z = tlv493d_convert_to_cursor_movement(data->z);

    // 6自由度の入力イベントを発行
    // zmk_event_manager_emit(INPUT_REL_RX, data->rx);
    // zmk_event_manager_emit(INPUT_REL_RY, data->ry);
    // zmk_event_manager_emit(INPUT_REL_RZ, data->rz);
    // zmk_event_manager_emit(INPUT_REL_X, data->x);
    // zmk_event_manager_emit(INPUT_REL_Y, data->y);
    // zmk_event_manager_emit(INPUT_REL_Z, data->z);

    input_report_rel(dev, INPUT_REL_X, data->x, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, data->y, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Z, data->z, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_RX, data->rx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_RY, data->ry, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_RZ, data->rz, false, K_FOREVER);


    // マウス移動イベントの発行
    // zmk_event_manager_emit(ZMK_EVENT_MOUSE_MOVE, cursor_x, cursor_y);

    // キー入力イベントの発行（閾値を超えた場合）
    // if (cursor_x > THRESHOLD) {
    //     zmk_event_manager_emit(ZMK_EVENT_KEY_PRESS, ZMK_KEY_RIGHT);
    // }
    // if (cursor_x < -THRESHOLD) {
    //     zmk_event_manager_emit(ZMK_EVENT_KEY_PRESS, ZMK_KEY_LEFT);
    // }

    return 0;
}

// static int tlv493d_process_event(const struct device *dev, struct tlv493d_data *data) {
//     process_sensor_data(data);  // 事前にデータ処理を実施

//     struct zmk_input_event event = {
//         .type = ZMK_INPUT_EVENT_RELATIVE,
//         .code = INPUT_REL_X,
//         .value = data->x
//     };
//     zmk_input_listener_emit(&event);

//     event.code = INPUT_REL_Y;
//     event.value = data->y;
//     zmk_input_listener_emit(&event);

//     event.code = INPUT_REL_Z;
//     event.value = data->z;
//     zmk_input_listener_emit(&event);

//     event.code = INPUT_REL_RX;
//     event.value = data->rx;
//     zmk_input_listener_emit(&event);

//     event.code = INPUT_REL_RY;
//     event.value = data->ry;
//     zmk_input_listener_emit(&event);

//     event.code = INPUT_REL_RZ;
//     event.value = data->rz;
//     zmk_input_listener_emit(&event);

//     return 0;
// }

ZMK_BEHAVIOR_DEFINE(tlv493d_behavior, tlv493d_process_event);
