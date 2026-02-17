/*
 * bbtrackball_input_handler.c - BB Trackball (动态窗口加速)
 *
 * 算法：检测最近3个脉冲的间隔密度
 * - 稀疏脉冲(慢速) → 1格
 * - 密集脉冲(快速) → 多格
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_bbtrackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
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

/* ==== 动态窗口参数 ==== */
#define WINDOW_SIZE 3              /* 看最近3个脉冲 */
#define BASE_INTERVAL_MS 40        /* 基础处理间隔 */

/* 速度档位 (根据平均脉冲间隔划分) */
#define SPEED_SLOW_MS 180          /* >180ms: 慢速,1格 */
#define SPEED_MED_MS 120           /* 120-180ms: 中速,2格 */
#define SPEED_FAST_MS 70           /* 70-120ms: 快速,3格 */
#define SPEED_VFAST_MS 40          /* 40-70ms: 很快,5格 */
                                   /* <40ms: 极速,8格 */

/* ==== 方向定义 ==== */
enum {
    DIR_LEFT = 0,
    DIR_RIGHT,
    DIR_UP,
    DIR_DOWN,
    DIR_COUNT
};

/* ==== 状态 ==== */
static bool space_pressed = false;
static const struct device *trackball_dev_ref = NULL;

/* ==== 每个方向的状态 ==== */
typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint32_t pulse_times[WINDOW_SIZE];  /* 最近脉冲时间窗口 */
    uint8_t pulse_idx;                  /* 当前写入位置 */
    uint8_t pulse_count;                /* 窗口中有效脉冲数 */
    int pending_steps;                  /* 待执行步数 */
} DirState;

/* 方向修正 */
static DirState dir_states[DIR_COUNT] = {
    [DIR_LEFT]  = {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, {0}, 0, 0, 0},
    [DIR_RIGHT] = {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, {0}, 0, 0, 0},
    [DIR_UP]    = {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, {0}, 0, 0, 0},
    [DIR_DOWN]  = {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, {0}, 0, 0, 0},
};

static struct gpio_callback gpio_cbs[DIR_COUNT];
static struct k_work_delayable process_work;

/* ==== Device Config/Data ==== */
struct bbtrackball_dev_config {
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct bbtrackball_data {
    const struct device *dev;
};

/* ==== 外部接口 (供trackball_led.c使用) ==== */
bool trackball_is_moving(void) {
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        /* 有pending步数或者在最近100ms内有脉冲 */
        if (d->pending_steps > 0) {
            return true;
        }
        if (d->pulse_count > 0) {
            int newest_idx = (d->pulse_idx - 1 + WINDOW_SIZE) % WINDOW_SIZE;
            if (now - d->pulse_times[newest_idx] < 100) {
                return true;
            }
        }
    }
    return false;
}

/* ==== 发送方向键 ==== */
static void send_arrow_key(uint8_t keycode, bool pressed) {
    if (pressed) {
        zmk_hid_keyboard_press(keycode);
    } else {
        zmk_hid_keyboard_release(keycode);
    }
    zmk_endpoints_send_report(0x07);
}

/* ==== 触发一次方向键 ==== */
static void trigger_key_press(uint8_t dir) {
    uint8_t keycode;
    switch (dir) {
        case DIR_LEFT:  keycode = 0x50; break;
        case DIR_RIGHT: keycode = 0x4F; break;
        case DIR_UP:    keycode = 0x52; break;
        case DIR_DOWN:  keycode = 0x51; break;
        default: return;
    }
    send_arrow_key(keycode, true);
    send_arrow_key(keycode, false);
}

/* ==== 计算速度档位 ==== */
static int calc_speed_steps(DirState *d, uint32_t now) {
    if (d->pulse_count < 2) {
        /* 脉冲太少，算1格 */
        return 1;
    }

    /* 计算窗口内平均间隔 */
    uint32_t total_interval = 0;
    uint8_t valid_count = 0;

    for (int i = 1; i < d->pulse_count; i++) {
        int idx_curr = (d->pulse_idx - i + WINDOW_SIZE) % WINDOW_SIZE;
        int idx_prev = (d->pulse_idx - i - 1 + WINDOW_SIZE) % WINDOW_SIZE;
        uint32_t interval = d->pulse_times[idx_curr] - d->pulse_times[idx_prev];
        if (interval > 0 && interval < 500) {  /* 过滤异常值 */
            total_interval += interval;
            valid_count++;
        }
    }

    if (valid_count == 0) return 1;

    uint32_t avg_interval = total_interval / valid_count;
    LOG_DBG("Avg interval: %d ms", avg_interval);

    /* 根据平均间隔返回跳跃数 */
    if (avg_interval > SPEED_SLOW_MS) {
        return 1;  /* 慢速: 1格 */
    } else if (avg_interval > SPEED_MED_MS) {
        return 2;  /* 中速: 2格 */
    } else if (avg_interval > SPEED_FAST_MS) {
        return 3;  /* 快速: 3格 */
    } else if (avg_interval > SPEED_VFAST_MS) {
        return 5;  /* 很快: 5格 */
    } else {
        return 8;  /* 极速: 8格封顶 */
    }
}

/* ==== Space Listener ==== */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) return 0;

    if (ev->position == 60) {
        space_pressed = ev->state;
        LOG_INF("Space %s", space_pressed ? "HELD (scroll mode)" : "RELEASED");
    }
    return 0;
}

ZMK_LISTENER(space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(space_listener, zmk_position_state_changed);

/* ==== GPIO 中断回调 ==== */
static void dir_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t now = k_uptime_get_32();

    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {
            int val = gpio_pin_get(dev, d->pin);
            if (val != d->last_state) {
                d->last_state = val;
                if (val == 0) {  /* 下降沿 */
                    /* 记录脉冲时间 */
                    d->pulse_times[d->pulse_idx] = now;
                    d->pulse_idx = (d->pulse_idx + 1) % WINDOW_SIZE;
                    if (d->pulse_count < WINDOW_SIZE) d->pulse_count++;

                    /* 计算步数并加入pending */
                    int steps = calc_speed_steps(d, now);
                    d->pending_steps += steps;

                    LOG_DBG("Dir %d: pulse detected, steps=%d", i, steps);
                }
            }
            break;
        }
    }
}

/* ==== 处理步进 ==== */
static void process_handler(struct k_work *work) {
    /* X轴 (左右) */
    DirState *d_left = &dir_states[DIR_LEFT];
    DirState *d_right = &dir_states[DIR_RIGHT];

    if (d_left->pending_steps > 0 && d_right->pending_steps > 0) {
        /* 两个方向都有，抵消 */
        int diff = d_left->pending_steps - d_right->pending_steps;
        if (diff > 0) {
            trigger_key_press(DIR_LEFT);
        } else if (diff < 0) {
            trigger_key_press(DIR_RIGHT);
        }
        d_left->pending_steps = 0;
        d_right->pending_steps = 0;
    } else if (d_left->pending_steps > 0) {
        trigger_key_press(DIR_LEFT);
        d_left->pending_steps--;
    } else if (d_right->pending_steps > 0) {
        trigger_key_press(DIR_RIGHT);
        d_right->pending_steps--;
    }

    /* Y轴 (上下) */
    DirState *d_up = &dir_states[DIR_UP];
    DirState *d_down = &dir_states[DIR_DOWN];

    if (d_up->pending_steps > 0 && d_down->pending_steps > 0) {
        int diff = d_up->pending_steps - d_down->pending_steps;
        if (diff > 0) {
            trigger_key_press(DIR_UP);
        } else if (diff < 0) {
            trigger_key_press(DIR_DOWN);
        }
        d_up->pending_steps = 0;
        d_down->pending_steps = 0;
    } else if (d_up->pending_steps > 0) {
        trigger_key_press(DIR_UP);
        d_up->pending_steps--;
    } else if (d_down->pending_steps > 0) {
        trigger_key_press(DIR_DOWN);
        d_down->pending_steps--;
    }

    /* 清理旧脉冲 (超过300ms的视为无效) */
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        if (d->pulse_count > 0) {
            int newest_idx = (d->pulse_idx - 1 + WINDOW_SIZE) % WINDOW_SIZE;
            if (now - d->pulse_times[newest_idx] > 300) {
                d->pulse_count = 0;  /* 清空窗口 */
            }
        }
    }

    k_work_schedule(&process_work, K_MSEC(BASE_INTERVAL_MS));
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (dynamic window acceleration)...");
    LOG_INF("  Window size: %d, Speed tiers: >%dms(1), >%dms(2), >%dms(3), >%dms(5), fast(8)",
            WINDOW_SIZE, SPEED_SLOW_MS, SPEED_MED_MS, SPEED_FAST_MS, SPEED_VFAST_MS);

    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        gpio_pin_configure(d->gpio_dev, d->pin,
                          GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_BOTH);
        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);

        gpio_init_callback(&gpio_cbs[i], dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &gpio_cbs[i]);
        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);
    }

    data->dev = dev;
    trackball_dev_ref = dev;

    k_work_init_delayable(&process_work, process_handler);
    k_work_schedule(&process_work, K_MSEC(BASE_INTERVAL_MS));

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
