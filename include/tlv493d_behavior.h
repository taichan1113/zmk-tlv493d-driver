#ifndef TLV493D_BEHAVIOR_H
#define TLV493D_BEHAVIOR_H

// #include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zmk/event_manager.h>
#include <zmk/behavior.h>

struct tlv493d_behavior_config {
    float sensitivity;
};

int tlv493d_process_event(const struct device *dev, struct sensor_value *data);

#endif /* TLV493D_BEHAVIOR_H */