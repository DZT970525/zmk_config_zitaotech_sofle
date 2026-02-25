/*
 * bbtrackball_input_handler.c - BB Trackball (节流状态机)
 *
 * 算法：节流状态机
 * - IDLE: 等待首次脉冲
 * - FIRST_OUTPUT: 输出1格，进入冷却
 * - COOLDOWN: 200ms冷却期，累计脉冲
 * - UNIFORM: 60ms固定间隔匀速输出
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


/* ==== 滚轮节流控制参数 ==== */
#define COOLDOWN_MS 200            /* 首次触发后的冷却期 */
#define UNIFORM_INTERVAL_MS 60     /* 匀速输出间隔 */
#define RESET_IDLE_MS 300          /* 停止移动后重置状态的时间（从100增加到300ms以过滤干扰）*/
#define DEBOUNCE_MS 5              /* 防抖时间：5ms内的重复脉冲忽略 */

/* 节流状态机 */
enum throttle_state {
    THROTTLE_IDLE = 0,             /* 空闲状态 */
    THROTTLE_FIRST_OUTPUT,         /* 首次输出 */
    THROTTLE_COOLDOWN,             /* 冷却期 */
    THROTTLE_UNIFORM               /* 匀速输出 */
};

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
    int pending_steps;                  /* 待执行步数 */
    int accumulated_steps;              /* 冷却期累计的步数 */
    enum throttle_state throttle;       /* 节流状态机 */
    uint32_t last_output_time;          /* 上次输出时间 */
    uint32_t last_pulse_time;           /* 上次脉冲时间 */
} DirState;

/* 方向修正 */
static DirState dir_states[DIR_COUNT] = {
    [DIR_LEFT]  = {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 0, 0, THROTTLE_IDLE, 0, 0},
    [DIR_RIGHT] = {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, 0, THROTTLE_IDLE, 0, 0},
    [DIR_UP]    = {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 0, 0, THROTTLE_IDLE, 0, 0},
    [DIR_DOWN]  = {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 0, 0, THROTTLE_IDLE, 0, 0},
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
        if (d->pending_steps > 0 || d->accumulated_steps > 0) {
            return true;
        }
        if (now - d->last_pulse_time < 100) {
            return true;
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
                    /* 防抖：5ms内的重复脉冲忽略（过滤电磁干扰）*/
                    if (now - d->last_pulse_time < DEBOUNCE_MS) {
                        LOG_DBG("Dir %d: debounced pulse (interval=%dms)", i, (int)(now - d->last_pulse_time));
                        break;
                    }
                    d->last_pulse_time = now;

                    switch (d->throttle) {
                        case THROTTLE_IDLE:
                            /* 首次脉冲：输出1格，进入FIRST_OUTPUT状态 */
                            d->pending_steps = 1;
                            d->accumulated_steps = 0;
                            d->throttle = THROTTLE_FIRST_OUTPUT;
                            LOG_INF("Dir %d: IDLE -> FIRST_OUTPUT", i);
                            break;

                        case THROTTLE_FIRST_OUTPUT:
                            /* 首次输出后立即进入冷却期，此状态不应再收到脉冲 */
                            d->accumulated_steps++;
                            LOG_DBG("Dir %d: FIRST_OUTPUT pulse accumulated", i);
                            break;

                        case THROTTLE_COOLDOWN:
                            /* 冷却期内：累计步数，不输出 */
                            d->accumulated_steps++;
                            LOG_DBG("Dir %d: COOLDOWN pulse accumulated, total=%d", i, d->accumulated_steps);
                            break;

                        case THROTTLE_UNIFORM:
                            /* 匀速期：累计步数 */
                            d->accumulated_steps++;
                            LOG_DBG("Dir %d: UNIFORM pulse accumulated, total=%d", i, d->accumulated_steps);
                            break;
                    }
                }
            }
            break;
        }
    }
}

/* ==== 处理单个方向的节流状态机 ==== */
static void process_dir_throttle(DirState *d, int dir_id, uint32_t now) {
    switch (d->throttle) {
        case THROTTLE_IDLE:
            /* 空闲状态，等待脉冲 */
            break;

        case THROTTLE_FIRST_OUTPUT:
            /* 输出首次按键，记录时间，进入冷却期 */
            if (d->pending_steps > 0) {
                trigger_key_press(dir_id);
                d->pending_steps = 0;
                d->last_output_time = now;
                d->throttle = THROTTLE_COOLDOWN;
                LOG_INF("Dir %d: FIRST_OUTPUT -> COOLDOWN", dir_id);
            }
            break;

        case THROTTLE_COOLDOWN:
            /* 检查冷却期是否结束 */
            if (now - d->last_output_time >= COOLDOWN_MS) {
                /* 冷却结束，将累计的步数移到pending */
                d->pending_steps = d->accumulated_steps;
                d->accumulated_steps = 0;
                d->throttle = THROTTLE_UNIFORM;
                d->last_output_time = now;
                LOG_INF("Dir %d: COOLDOWN -> UNIFORM (pending=%d)", dir_id, d->pending_steps);

                /* 如果有步数，立即输出一个 */
                if (d->pending_steps > 0) {
                    trigger_key_press(dir_id);
                    d->pending_steps--;
                    d->last_output_time = now;
                }
            }
            break;

        case THROTTLE_UNIFORM:
            /* 匀速输出：每60ms输出一个 */
            if (now - d->last_output_time >= UNIFORM_INTERVAL_MS) {
                if (d->pending_steps > 0) {
                    trigger_key_press(dir_id);
                    d->pending_steps--;
                    d->last_output_time = now;
                    LOG_DBG("Dir %d: UNIFORM output, remaining=%d", dir_id, d->pending_steps);
                } else if (d->accumulated_steps > 0) {
                    /* pending空了，从accumulated补充 */
                    d->pending_steps = d->accumulated_steps;
                    d->accumulated_steps = 0;
                    trigger_key_press(dir_id);
                    d->pending_steps--;
                    d->last_output_time = now;
                    LOG_DBG("Dir %d: UNIFORM output from accumulated, remaining=%d", dir_id, d->pending_steps);
                }
            }
            break;
    }

    /* 检查是否需要重置：100ms无脉冲 */
    if (d->throttle != THROTTLE_IDLE && (now - d->last_pulse_time > RESET_IDLE_MS)) {
        LOG_INF("Dir %d: timeout reset to IDLE", dir_id);
        d->throttle = THROTTLE_IDLE;
        d->pending_steps = 0;
        d->accumulated_steps = 0;
    }
}

/* ==== 处理步进 ==== */
static void process_handler(struct k_work *work) {
    uint32_t now = k_uptime_get_32();

    /* 处理X轴（左右）- 互斥 */
    DirState *d_left = &dir_states[DIR_LEFT];
    DirState *d_right = &dir_states[DIR_RIGHT];

    /* 如果两个方向都有输入，优先处理pending_steps多的方向 */
    if ((d_left->pending_steps > 0 || d_left->accumulated_steps > 0) &&
        (d_right->pending_steps > 0 || d_right->accumulated_steps > 0)) {
        int left_total = d_left->pending_steps + d_left->accumulated_steps;
        int right_total = d_right->pending_steps + d_right->accumulated_steps;
        if (left_total > right_total) {
            process_dir_throttle(d_left, DIR_LEFT, now);
        } else {
            process_dir_throttle(d_right, DIR_RIGHT, now);
        }
    } else {
        process_dir_throttle(d_left, DIR_LEFT, now);
        process_dir_throttle(d_right, DIR_RIGHT, now);
    }

    /* 处理Y轴（上下）- 互斥 */
    DirState *d_up = &dir_states[DIR_UP];
    DirState *d_down = &dir_states[DIR_DOWN];

    if ((d_up->pending_steps > 0 || d_up->accumulated_steps > 0) &&
        (d_down->pending_steps > 0 || d_down->accumulated_steps > 0)) {
        int up_total = d_up->pending_steps + d_up->accumulated_steps;
        int down_total = d_down->pending_steps + d_down->accumulated_steps;
        if (up_total > down_total) {
            process_dir_throttle(d_up, DIR_UP, now);
        } else {
            process_dir_throttle(d_down, DIR_DOWN, now);
        }
    } else {
        process_dir_throttle(d_up, DIR_UP, now);
        process_dir_throttle(d_down, DIR_DOWN, now);
    }

    k_work_schedule(&process_work, K_MSEC(10));  /* 10ms轮询间隔 */
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (throttle state machine)...");
    LOG_INF("  COOLDOWN: %dms, UNIFORM: %dms, RESET: %dms",
            COOLDOWN_MS, UNIFORM_INTERVAL_MS, RESET_IDLE_MS);

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
    k_work_schedule(&process_work, K_MSEC(10));

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
