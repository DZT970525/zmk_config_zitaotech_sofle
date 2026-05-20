/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (minimal modification)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* ========= ⭐ TrackPoint 专用 Work Queue ========= */
#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 5

/* ========= ⭐ NEW: I2C Mutex ========= */
static struct k_mutex trackpoint_i2c_mutex;

K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

/* ========================================================================= */
/* 鼠标与滚轮可调参数 (已映射至 Kconfig，用户可在 .conf 中配置)                 */
/* ========================================================================= */

// --- 滚轮方向配置 ---
#define SCROLL_X_DIR (-CONFIG_TRACKPOINT_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_TRACKPOINT_SCROLL_Y_DIR

// --- 滚轮灵敏度与粒度配置 ---
#define SCROLL_DEADZONE CONFIG_TRACKPOINT_SCROLL_DEADZONE
#define SCROLL_INPUT_MAX CONFIG_TRACKPOINT_SCROLL_INPUT_MAX
#define SCROLL_DIVISOR_SLOW CONFIG_TRACKPOINT_SCROLL_DIVISOR_SLOW
#define SCROLL_DIVISOR_FAST CONFIG_TRACKPOINT_SCROLL_DIVISOR_FAST

// --- Arrow key threshold / divisor ---
#define ARROW_DEADZONE CONFIG_TRACKPOINT_SCROLL_DEADZONE
#define ARROW_INPUT_MAX 256
#define ARROW_DIVISOR_SLOW CONFIG_TRACKPOINT_SCROLL_DIVISOR_SLOW
#define ARROW_DIVISOR_FAST CONFIG_TRACKPOINT_SCROLL_DIVISOR_FAST

// --- 防误触锁定比例配置 ---
#define DOMINANT_NUMERATOR CONFIG_TRACKPOINT_DOMINANT_NUMERATOR
#define DOMINANT_DENOMINATOR CONFIG_TRACKPOINT_DOMINANT_DENOMINATOR

// --- 鼠标指针基础配置 (Kconfig 为整数百分比，这里除以 100 转为浮点数) ---
#define MOUSE_BASE_SPEED (CONFIG_TRACKPOINT_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_TRACKPOINT_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_TRACKPOINT_MOUSE_SENS_STEP_PERCENT / 100.0f)

/* ========= Motion GPIO ========= */

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_I2C_ADDR 0x15
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

#define SLOW_KEY_MULTIPLIER 0.5f

/* ========= Watch Dog ========= */
static uint32_t last_activity_time = 0;
#define TRACKPOINT_WDT_TIMEOUT 200
/* ========= 全局状态 ========= */
static bool scroll_key_pressed = false;
static bool arrow_key_pressed = false;
static bool slow_key_pressed = false;
static bool last_scroll_key_pressed = false; // ★ NEW
static bool last_arrow_key_pressed = false;
uint32_t last_packet_time = 0;

/* ==== HID indicators ==== */
static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1 << 1)
/* =========================
 *   HID indicator listener
 * ========================= */
static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev) {
        current_indicators = ev->indicators;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);

/* ========= Space + Slow 按键监听 ========= */
static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev)
        return 0;
    if (ev->position == 34) {
        arrow_key_pressed = ev->state;
        LOG_INF("space position=49 %s", arrow_key_pressed ? "PRESSED" : "RELEASED");
    }

    // Scroll key (Space)
    if (ev->position == 61) {
        scroll_key_pressed = ev->state;
        LOG_INF("space position=49 %s", scroll_key_pressed ? "PRESSED" : "RELEASED");
    }

    // ★ NEW: Slow key
    if (ev->position == 36) {
        slow_key_pressed = ev->state;
        LOG_INF("slow_key position=37 %s", slow_key_pressed ? "PRESSED" : "RELEASED");
    }

    return 0;
}
ZMK_LISTENER(trackpoint_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_special_key_listener, zmk_position_state_changed);

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work; // ⭐ 新增
    uint32_t last_packet_time;
    int16_t scroll_residue_x;
    int16_t scroll_residue_y;
    int16_t arrow_residue_x;
    int16_t arrow_residue_y;
};

/* ========= 指数加速计算 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_MAX_MULT 2.0f
static inline float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0)
        delta_ms = 1;

    int dist = abs(dx) + abs(dy);
    if (dist < 1)
        return 1.0f;

    float speed = (float)dist / (float)delta_ms;
    float mult = expf(speed * 1.307357f);

    return (mult > TP_MAX_MULT) ? TP_MAX_MULT : mult;
}
#endif

/* ========= 读取数据 ========= */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};

    int ret;

    k_mutex_lock(&trackpoint_i2c_mutex, K_NO_WAIT);

    ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);

    k_mutex_unlock(&trackpoint_i2c_mutex);

    if (ret < 0)
        return ret;

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0)
        return -EIO;

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* ========= ★ 抽象复用：滚轮单轴处理函数 ========= */
static inline void process_scroll_axis(const struct device *dev, int8_t delta, int16_t *residue,
                                       uint16_t input_code, int8_t dir_mult) {
    int abs_delta = abs(delta);

    // ★ 不清零，保持连续性
    if (abs_delta <= SCROLL_DEADZONE) {
        return;
    }

    if (abs_delta > SCROLL_INPUT_MAX) {
        abs_delta = SCROLL_INPUT_MAX;
    }

    // ★ 非线性 divisor（更丝滑）
    float t = (float)abs_delta / SCROLL_INPUT_MAX;
    t = t * t;

    float f_div = SCROLL_DIVISOR_SLOW - (SCROLL_DIVISOR_SLOW - SCROLL_DIVISOR_FAST) * t;

    int divisor = (int)f_div;
    if (divisor < 1)
        divisor = 1;

    *residue += (delta * dir_mult);

    int16_t scroll_ticks = *residue / divisor;
    if (scroll_ticks != 0) {
        input_report_rel(dev, input_code, scroll_ticks, true, K_NO_WAIT);
        *residue %= divisor;
    }

    // ★ 阻尼（关键）
    *residue = (*residue * 3) / 4;
}

static inline void process_arrow_axis(const struct device *dev, int8_t delta, int16_t *residue,
                                      uint16_t key_neg, uint16_t key_pos) {

    int abs_delta = abs(delta);

    if (abs_delta <= ARROW_DEADZONE) {
        return;
    }

    if (abs_delta > ARROW_INPUT_MAX) {
        abs_delta = ARROW_INPUT_MAX;
    }

    // ★ 非线性 divisor（更丝滑）
    float t = (float)abs_delta / SCROLL_INPUT_MAX;
    t = t * t;

    float f_div = SCROLL_DIVISOR_SLOW - (SCROLL_DIVISOR_SLOW - SCROLL_DIVISOR_FAST) * t;

    int divisor = (int)f_div;
    if (divisor < 1)
        divisor = 1;

    *residue += delta; // 替换掉 dir_mult
    int16_t arrow_ticks = *residue / divisor;
    if (arrow_ticks != 0) {
        uint16_t key = (arrow_ticks > 0) ? key_pos : key_neg;

        // 触发 key press + release（脉冲）
        input_report_key(dev, key, 1, true, K_NO_WAIT);
        input_report_key(dev, key, 0, true, K_NO_WAIT);

        *residue %= divisor;
    }

    // 阻尼（防止漂移）
    *residue = (*residue * 3) / 4;
}

static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;

    uint32_t now = k_uptime_get_32();

    /* ========= WATCHDOG ========= */
    if (now - last_activity_time > TRACKPOINT_WDT_TIMEOUT) {
        LOG_WRN("TrackPoint watchdog recovery");

        data->scroll_residue_x = 0;
        data->scroll_residue_y = 0;
        data->arrow_residue_x = 0;
        data->arrow_residue_y = 0;
        last_scroll_key_pressed = scroll_key_pressed;
        return;
    }

    int8_t dx = 0, dy = 0;

    /* ========= 单次读取（核心改动） ========= */
    int ret = trackpoint_read_packet(dev, &dx, &dy);

    if (ret != 0) {
        LOG_WRN("TrackPoint I2C read failed (soft recover)");

        /* ⚠️ 不 break，不 sleep，不卡住 */
        data->scroll_residue_x = 0;
        data->scroll_residue_y = 0;

        return;
    }

    last_activity_time = now;

    /* ========= scroll mode 切换检测 ========= */
    bool just_enter_scroll = scroll_key_pressed && !last_scroll_key_pressed;
    bool just_enter_arrow = arrow_key_pressed && !last_arrow_key_pressed;
    bool capslock = current_indicators & HID_INDICATORS_CAPS_LOCK;

    if (arrow_key_pressed) {

        if (just_enter_arrow) {
            data->arrow_residue_x = dx;
            data->arrow_residue_y = dy;
        }

        int abs_dx = abs(dx);
        int abs_dy = abs(dy);

        // ★ 同 scroll 的主方向锁定
        if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
            dx = 0;
        } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
            dy = 0;
        } else {
            dx = 0;
            dy = 0;
        }

        // ★ X → 左右
        process_arrow_axis(dev, dx, &data->arrow_residue_x,
                           INPUT_BTN_0,  // 左
                           INPUT_BTN_1); // 右

        // ★ Y → 上下
        process_arrow_axis(dev, dy, &data->arrow_residue_y,
                           INPUT_BTN_2,  // 上
                           INPUT_BTN_3); // 下
    } else if (scroll_key_pressed || capslock) {

        if (just_enter_scroll) {
            data->scroll_residue_x = dx * SCROLL_X_DIR;
            data->scroll_residue_y = dy * SCROLL_Y_DIR;
        }

        int abs_dx = abs(dx);
        int abs_dy = abs(dy);

        if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
            dx = 0;
        } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
            dy = 0;
        } else {
            dx = 0;
            dy = 0;
        }

        process_scroll_axis(dev, dx, &data->scroll_residue_x, INPUT_REL_HWHEEL, SCROLL_X_DIR);
        process_scroll_axis(dev, dy, &data->scroll_residue_y, INPUT_REL_WHEEL, SCROLL_Y_DIR);

    } else {

        uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
        float tp_factor = MOUSE_SENS_BASE + MOUSE_SENS_STEP * tp_led_brt;

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
        uint32_t delta = now - data->last_packet_time;
        float exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
        float exp_mult = 1.0f;
#endif

        float slow_mult = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;

        float fx = dx * MOUSE_BASE_SPEED * tp_factor * exp_mult * slow_mult;
        float fy = dy * MOUSE_BASE_SPEED * tp_factor * exp_mult * slow_mult;

        input_report_rel(dev, INPUT_REL_X, -(int)fx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, -(int)fy, true, K_NO_WAIT);
    }

    last_scroll_key_pressed = scroll_key_pressed;
    last_arrow_key_pressed = arrow_key_pressed;
    data->last_packet_time = now;
}

/* ========= ★ GPIO 中断 ========= */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);

    last_activity_time = k_uptime_get_32();

    // ⭐ 改这里：提交到专用 work queue
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static void trackpoint_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, enable_irq_work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    LOG_INF("TrackPoint IRQ enabled (delayed)");
}
/* ========= 初始化函数 ========= */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;
    if (!i2c_is_ready_dt(&cfg->i2c))
        return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio))
        return -ENODEV;

    k_mutex_init(&trackpoint_i2c_mutex);

    data->dev = dev;
    data->scroll_residue_x = 0;
    data->scroll_residue_y = 0;
    data->arrow_residue_x = 0;
    data->arrow_residue_y = 0;
    data->last_packet_time = k_uptime_get_32();

    k_work_init(&data->work, trackpoint_work_cb);

    /* ========= ⭐ 启动独立 Work Queue ========= */
    k_work_queue_start(&tp_workq, tp_workq_stack, K_THREAD_STACK_SIZEOF(tp_workq_stack),
                       TP_WORKQ_PRIORITY, NULL);

    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);

    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    k_work_init_delayable(&data->enable_irq_work, trackpoint_enable_irq_work_cb);
    k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("TrackPoint Driver Initialized (IRQ delayed)");
    return 0;
}

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                    \
                        .dt_flags = MOTION_GPIO_FLAGS},                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
