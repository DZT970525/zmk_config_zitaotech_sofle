/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* ========= Motion GPIO ========= */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
static const struct device *motion_gpio_dev;

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_I2C_ADDR 0x15
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

/* ========= 全局状态 ========= */
static const struct device *trackpoint_dev_ref = NULL;
static bool space_pressed = false;
uint32_t last_packet_time = 0;

/* ========= Space 按键监听 ========= */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) {
        return 0;
    }

    if (ev->position == 61) { // Space position code
        space_pressed = ev->state;
        LOG_INF("space position=61 %s", space_pressed ? "PRESSED" : "RELEASED");
    }
    return 0;
}
ZMK_LISTENER(trackpoint_space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_space_listener, zmk_position_state_changed);

/* ========= TrackPoint 配置结构 ========= */
struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
};

/* ========= 读取数据包 ========= */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};
    int ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);
    if (ret < 0) {
        LOG_ERR("I2C read failed: %d", ret);
        return ret;
    }
    if (buf[0] != TRACKPOINT_MAGIC_BYTE0) {
        LOG_WRN("Invalid packet header: 0x%02X", buf[0]);
        return -EIO;
    }
    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* ========= Polling 任务 ========= */
static void trackpoint_poll_work(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, poll_work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    if (pin_state == 0) {
        /* INTPIN 拉低，读取数据包 */
        int8_t dx = 0, dy = 0;
        if (trackpoint_read_packet(dev, &dx, &dy) == 0) {
            if (!space_pressed) {
                /* Space 按住时作为滚轮 */
                int16_t scroll_x = 0, scroll_y = 0;
                if (abs(dy) >= 128) {
                    scroll_x = -dx / 24;
                    scroll_y = -dy / 24;
                } else if (abs(dy) >= 64) {
                    scroll_x = -dx / 16;
                    scroll_y = -dy / 16;
                } else if (abs(dy) >= 32) {
                    scroll_x = -dx / 12;
                    scroll_y = -dy / 12;
                } else if (abs(dy) >= 21) {
                    scroll_x = -dx / 8;
                    scroll_y = -dy / 8;
                } else if (abs(dy) >= 3) {
                    scroll_x = (dx > 0) ? -1 : (dx < 0) ? 1 : 0;
                    scroll_y = (dy > 0) ? -1 : (dy < 0) ? 1 : 0;
                } else {
                    scroll_x = (dx > 0) ? -1 : (dx < 0) ? 1 : 0;
                    scroll_y = 0;
                }
                input_report_rel(dev, INPUT_REL_HWHEEL, scroll_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_WHEEL, -scroll_y, true, K_FOREVER);
                k_sleep(K_MSEC(40));
            } else {
                /* 正常鼠标移动 */
                uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
                float tp_factor = 0.4f + 0.01f * tp_led_brt;
                dx = dx * 3 / 2 * tp_factor;
                dy = dy * 3 / 2 * tp_factor;
                input_report_rel(dev, INPUT_REL_X, -dx, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, -dy, true, K_FOREVER);
            }
        }
        last_packet_time = now;
    }

    k_work_schedule(&data->poll_work, K_MSEC(5));
}

/* ========= 初始化函数 ========= */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;

    LOG_DBG("Initializing TrackPoint I2C @0x%02x", cfg->i2c.addr);
    k_sleep(K_MSEC(10));
    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    motion_gpio_dev = DEVICE_DT_GET(MOTION_GPIO_NODE);
    if (!device_is_ready(motion_gpio_dev)) {
        LOG_ERR("Motion GPIO device not ready");
        return -ENODEV;
    }

    gpio_pin_configure(motion_gpio_dev, MOTION_GPIO_PIN, GPIO_INPUT | GPIO_PULL_UP);

    data->dev = dev;
    trackpoint_dev_ref = dev;

    k_work_init_delayable(&data->poll_work, trackpoint_poll_work);
    k_work_schedule(&data->poll_work, K_MSEC(5));

    LOG_DBG("TrackPoint initialized successfully");
    return 0;
}

/* ========= 设备注册 ========= */
#define TRACKPOINT_INIT_PRIORITY CONFIG_INPUT_INIT_PRIORITY

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),                          \
        .x_input_code = DT_PROP_OR(DT_DRV_INST(inst), x_input_code, INPUT_REL_X),                  \
        .y_input_code = DT_PROP_OR(DT_DRV_INST(inst), y_input_code, INPUT_REL_Y),                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, TRACKPOINT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
