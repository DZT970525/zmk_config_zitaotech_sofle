/*
 * bbtrackball_input_handler.c - BB Trackball (GPIO interrupt + periodic report + arrow key mode)
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_bbtrackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <math.h>
#include <stdlib.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(bbtrackball_input_handler, LOG_LEVEL_INF);

/* ==== GPIO Pins ==== */
#define DOWN_GPIO_PIN 9
#define LEFT_GPIO_PIN 12
#define UP_GPIO_PIN 5
#define RIGHT_GPIO_PIN 27

#define GPIO0_DEV DT_NODELABEL(gpio0)
#define GPIO1_DEV DT_NODELABEL(gpio1)

/* ==== Config ==== */
#define BASE_MOVE_PIXELS 3
#define EXPONENTIAL_BASE 1.12f
#define SPEED_SCALE 60.0f
#define REPORT_INTERVAL_MS 10
#define SCROLL_DELAY_MS 40

/* ==== 状态 ==== */
static bool moved = false;
static bool space_pressed = false;
static const struct device *trackball_dev_ref = NULL;
static int dx_acc = 0;
static int dy_acc = 0;

/* ==== GPIO 回调相关 ==== */
typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint32_t last_time;
    int sign; /* -1 or +1 */
} DirInput;

static DirInput dir_inputs[] = {
    {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, -1},
    {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 0, +1},
    {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 0, -1},
    {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 0, +1},
};

static struct gpio_callback gpio_cbs[ARRAY_SIZE(dir_inputs)];

/* ==== Device Config/Data ==== */
struct bbtrackball_dev_config {
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct bbtrackball_data {
    const struct device *dev;
    struct k_work_delayable report_work;
    struct k_work_delayable arrow_repeat_work;
};

/* ==== 外部接口 ==== */
bool trackball_is_moving(void) { return moved; }

/* ==== Space Listener ==== */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev)
        return 0;

    if (ev->position == 60) {
        space_pressed = ev->state;
        LOG_INF("Space %s", space_pressed ? "HELD" : "RELEASED");
    }
    return 0;
}

ZMK_LISTENER(space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(space_listener, zmk_position_state_changed);

/* ==== GPIO 中断回调 ==== */
static void dir_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    for (size_t i = 0; i < ARRAY_SIZE(dir_inputs); i++) {
        DirInput *d = &dir_inputs[i];
        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {
            int val = gpio_pin_get(dev, d->pin);
            if (val != d->last_state) {
                uint32_t now = k_uptime_get_32();
                uint32_t delta = now - d->last_time;
                if (delta == 0)
                    delta = 1;

                float speed_factor = SPEED_SCALE / (float)delta;
                float mult = powf(EXPONENTIAL_BASE, speed_factor);
                int delta_px = (int)roundf(BASE_MOVE_PIXELS * mult);

                if (i < 2)
                    dx_acc += d->sign * delta_px;
                else
                    dy_acc += d->sign * delta_px;

                d->last_state = val;
                d->last_time = now;
            }
        }
    }
}

/* ==== Arrow key / Scroll repeat task ==== */
static void arrow_repeat_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct bbtrackball_data *data = CONTAINER_OF(dwork, struct bbtrackball_data, arrow_repeat_work);

    if (!dx_acc && !dy_acc) {
        k_work_schedule(&data->arrow_repeat_work, K_MSEC(SCROLL_DELAY_MS));
        return;
    }

    int dx = -dx_acc;
    int dy = -dy_acc;

    /* === Space held → Scroll mode === */
    if (!space_pressed) {
        int scroll_x = dx;
        int scroll_y = dy;

        input_report_rel(data->dev, INPUT_REL_HWHEEL, scroll_x, false, K_FOREVER);
        input_report_rel(data->dev, INPUT_REL_WHEEL, -scroll_y, true, K_FOREVER);

        dx_acc = 0;
        dy_acc = 0;

        k_work_schedule(&data->arrow_repeat_work, K_MSEC(SCROLL_DELAY_MS));
        return;
    }

    dx_acc = 0;
    dy_acc = 0;

    k_work_schedule(&data->arrow_repeat_work, K_MSEC(10));
}

/* ==== HID 报告定时任务（Regular mouse movement） ==== */
static void report_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct bbtrackball_data *data = CONTAINER_OF(dwork, struct bbtrackball_data, report_work);
    const struct device *dev = data->dev;
    trackball_dev_ref = dev;

    if (dx_acc || dy_acc) {
        moved = true;

        /* Space held → 禁止鼠标移动（只允许 scroll / arrow） */
        if (space_pressed) {
            int dx = -dx_acc;
            int dy = -dy_acc;
            input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
            dx_acc = 0;
            dy_acc = 0;
        }
    } else {
        moved = false;
    }

    k_work_schedule(&data->report_work, K_MSEC(REPORT_INTERVAL_MS));
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (interrupt + workqueue + scroll mode)...");
    for (size_t i = 0; i < ARRAY_SIZE(dir_inputs); i++) {
        DirInput *d = &dir_inputs[i];
        gpio_pin_configure(d->gpio_dev, d->pin, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_BOTH);
        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);
        d->last_time = k_uptime_get_32();

        gpio_init_callback(&gpio_cbs[i], dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &gpio_cbs[i]);
        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);
    }

    data->dev = dev;
    trackball_dev_ref = dev;

    k_work_init_delayable(&data->report_work, report_work_handler);
    k_work_schedule(&data->report_work, K_MSEC(REPORT_INTERVAL_MS));

    k_work_init_delayable(&data->arrow_repeat_work, arrow_repeat_work_handler);
    k_work_schedule(&data->arrow_repeat_work, K_MSEC(SCROLL_DELAY_MS));

    return 0;
}

/* ==== 驱动实例注册 ==== */
#define BBTRACKBALL_INIT_PRIORITY CONFIG_INPUT_INIT_PRIORITY
#define BBTRACKBALL_DEFINE(inst)                                                                   \
    static struct bbtrackball_data bbtrackball_data_##inst;                                        \
    static const struct bbtrackball_dev_config bbtrackball_config_##inst = {                       \
        .x_input_code = DT_PROP_OR(DT_DRV_INST(inst), x_input_code, INPUT_REL_X),                  \
        .y_input_code = DT_PROP_OR(DT_DRV_INST(inst), y_input_code, INPUT_REL_Y),                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, bbtrackball_init, NULL, &bbtrackball_data_##inst,                  \
                          &bbtrackball_config_##inst, POST_KERNEL, BBTRACKBALL_INIT_PRIORITY,      \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(BBTRACKBALL_DEFINE);
