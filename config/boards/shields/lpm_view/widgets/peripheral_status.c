/*
 * Copyright (c) 2023 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>    // 补充备注，用于获取系统运行时间
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/battery.h>
#include <zmk/display.h>    // 补充备注：用于获取最后活动时间
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/split/bluetooth/peripheral.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/usb.h>
#include <zmk/ble.h>

#include "peripheral_status.h"

// 定义一个全局变量，记录副手最后感知到活跃的时间
static uint32_t last_anim_activity = 0; // 用于动画自动停止。

// ==================== 动画帧声明 ====================
// 新加的：
/*
LV_IMG_DECLARE(figure0);
LV_IMG_DECLARE(figure1);
LV_IMG_DECLARE(figure2);
LV_IMG_DECLARE(jianlai);
LV_IMG_DECLARE(yushi);
LV_IMG_DECLARE(skeleton);
*/
// 太极图
LV_IMG_DECLARE(resized_frame_1);
LV_IMG_DECLARE(resized_frame_2);
LV_IMG_DECLARE(resized_frame_3);
LV_IMG_DECLARE(resized_frame_4);
LV_IMG_DECLARE(resized_frame_5);
LV_IMG_DECLARE(resized_frame_6);
LV_IMG_DECLARE(resized_frame_7);
LV_IMG_DECLARE(resized_frame_8);
LV_IMG_DECLARE(resized_frame_9);
LV_IMG_DECLARE(resized_frame_10);
LV_IMG_DECLARE(resized_frame_11);
LV_IMG_DECLARE(resized_frame_12);
LV_IMG_DECLARE(resized_frame_13);
LV_IMG_DECLARE(resized_frame_14);
LV_IMG_DECLARE(resized_frame_15);
LV_IMG_DECLARE(resized_frame_16);
LV_IMG_DECLARE(resized_frame_17);
LV_IMG_DECLARE(resized_frame_18);
LV_IMG_DECLARE(resized_frame_19);
LV_IMG_DECLARE(resized_frame_20);

// 我注释的
/*
LV_IMG_DECLARE(bunnygirl1);
LV_IMG_DECLARE(bunnygirl3);
LV_IMG_DECLARE(bunnygirl6);
LV_IMG_DECLARE(bunnygirl9);
LV_IMG_DECLARE(bunnygirl12);
LV_IMG_DECLARE(bunnygirl15);
LV_IMG_DECLARE(bunnygirl18);
LV_IMG_DECLARE(bunnygirl21);
*/
LV_IMG_DECLARE(landspace1);

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

// ==================== 状态结构体 ====================
struct peripheral_status_state {
    bool connected;
};

// 动画状态
struct art_state {
    lv_obj_t *art;
    lv_timer_t *timer;
    uint8_t frame_index;
};

// 所有帧的引用数组
/*static const lv_img_dsc_t *bunny_frames[] = {
    &landspace1,
};
*/

// 我添加的
// ==================== 动画数组 ====================
static const lv_img_dsc_t *bunny_frames[] = {
    &resized_frame_1,
    &resized_frame_2,
    &resized_frame_3,
    &resized_frame_4,
    &resized_frame_5,
    &resized_frame_6,
    &resized_frame_7,
    &resized_frame_8,
    &resized_frame_9,
    &resized_frame_10,
    &resized_frame_11,
    &resized_frame_12,
    &resized_frame_13,
    &resized_frame_14,
    &resized_frame_15,
    &resized_frame_16,
    &resized_frame_17,
    &resized_frame_18,
    &resized_frame_19,
    &resized_frame_20,
};

// 我注释的
/*
static const lv_img_dsc_t *bunny_frames[] = {
    &bunnygirl1,  &bunnygirl3,  &bunnygirl6,  &bunnygirl9,
    &bunnygirl12, &bunnygirl15, &bunnygirl18, &bunnygirl21,
};
*/

#define BUNNY_FRAME_COUNT (sizeof(bunny_frames) / sizeof(bunny_frames[0]))

// ================= 顶部绘制 =================
static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 0);

    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);

    // Fill background
    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    // Draw battery
    draw_battery(canvas, state);

    // Draw connection icon
    lv_canvas_draw_text(canvas, 0, 0, CANVAS_SIZE, &label_dsc,
                        state->connected ? LV_SYMBOL_WIFI : LV_SYMBOL_CLOSE);

    // Rotate canvas
    rotate_canvas(canvas, cbuf);
}

// ================= 电池状态 =================
static void set_battery_status(struct zmk_widget_status *widget,
                               struct battery_status_state state) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    widget->state.charging = state.usb_present;
#endif
    widget->state.battery = state.level;
    draw_top(widget->obj, widget->cbuf, &widget->state);
}

static void battery_status_update_cb(struct battery_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_battery_status(widget, state); }
}

static struct battery_status_state battery_status_get_state(const zmk_event_t *eh) {
    return (struct battery_status_state){
        .level = zmk_battery_state_of_charge(),
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
        .usb_present = zmk_usb_is_powered(),
#endif
    };
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_battery_status, struct battery_status_state,
                            battery_status_update_cb, battery_status_get_state)
ZMK_SUBSCRIPTION(widget_battery_status, zmk_battery_state_changed);
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(widget_battery_status, zmk_usb_conn_state_changed);
#endif

// ================= 连接状态 =================
static struct peripheral_status_state get_state(const zmk_event_t *eh) {
    return (struct peripheral_status_state){.connected = zmk_split_bt_peripheral_is_connected()};
}

static void set_connection_status(struct zmk_widget_status *widget,
                                  struct peripheral_status_state state) {
    widget->state.connected = state.connected;
    draw_top(widget->obj, widget->cbuf, &widget->state);
}

static void output_status_update_cb(struct peripheral_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_connection_status(widget, state); }
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_peripheral_status, struct peripheral_status_state,
                            output_status_update_cb, get_state)
ZMK_SUBSCRIPTION(widget_peripheral_status, zmk_split_peripheral_status_changed);

// ================= 22帧动画回调 =================
static void art_anim_timer_cb(lv_timer_t *timer) {
    // 【核心逻辑】：
    // k_uptime_get() 获取系统当前运行了多少毫秒
    // zmk_display_get_last_activity() 获取上一次按键操作的时间戳
    // 30000 毫秒 = 30 秒 (你可以根据需求改为 10000 即 10 秒)
    /*
    if (k_uptime_get() - zmk_display_get_last_activity() > 3000) {
        // 如果超过 30 秒没动键盘，直接返回，不执行后面的切图和刷新逻辑
        // 这就是“动画自动睡眠”，此时屏幕保持静止，Flash 停止读取，CPU 降频
        return; 
    }
    */
    // 上面这个有问题，使用自己定义的额时间变量。
    if (k_uptime_get() - last_anim_activity > 3000) {
        return; 
    }
    
    struct art_state *state = timer->user_data;
    state->frame_index = (state->frame_index + 1) % BUNNY_FRAME_COUNT;
    lv_img_set_src(state->art, bunny_frames[state->frame_index]);
}

// ================= 初始化 =================
int zmk_widget_status_init(struct zmk_widget_status *widget, lv_obj_t *parent) {
    widget->obj = lv_obj_create(parent);
    lv_obj_set_size(widget->obj, 144, 72);

    lv_obj_t *top = lv_canvas_create(widget->obj);
    lv_obj_align(top, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_canvas_set_buffer(top, widget->cbuf, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);

    // --- 动画状态 ---
    static struct art_state astate;
    astate.art = lv_img_create(widget->obj);
    astate.frame_index = 0;
    lv_img_set_src(astate.art, bunny_frames[0]);
    lv_obj_align(astate.art, LV_ALIGN_TOP_LEFT, 20, 0);
    
    // 每秒切换一帧
    astate.timer = lv_timer_create(art_anim_timer_cb, 300, &astate); // 这个单位应该是ms

    lv_obj_set_user_data(widget->obj, &astate);

    sys_slist_append(&widgets, &widget->node);
    widget_battery_status_init();
    widget_peripheral_status_init();

    return 0;
}

lv_obj_t *zmk_widget_status_obj(struct zmk_widget_status *widget) { return widget->obj; }

// 只要副手检测到任何按键动作（或者是小红点的移动），就更新 last_anim_activity。
#include <zmk/events/keycode_state_changed.h>

// 之前我们尝试过的按键监听逻辑，现在正好用来更新时间
ZMK_EVENT_IMPL(zmk_keycode_state_changed);

int activity_monitor_listener(const zmk_event_t *eh) {
    // 只要有按键按下或松开的消息经过副手，就刷新时间戳
    last_anim_activity = k_uptime_get();
    return 0;
}

ZMK_LISTENER(activity_monitor_listener, activity_monitor_listener);
ZMK_SUBSCRIPTION(activity_monitor_listener, zmk_keycode_state_changed);
