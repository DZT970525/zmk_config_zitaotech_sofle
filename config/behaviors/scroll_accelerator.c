/*
 * Copyright (c) 2025 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 *
 * 自定义滚轮加速器 Behavior
 * 功能：
 * 1. 首次检测到滚轮转动时立即响应一次
 * 2. 如果持续转动，按平均速度连续响应（避免过快但保持流畅）
 * 3. 停止转动后，累积值清零
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/keymap.h>
#include <zmk/sensors.h>

#define DT_DRV_COMPAT zitao_behavior_scroll_accelerator

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// 状态定义
struct scroll_accelerator_state {
    int64_t last_event_time;      // 上次事件时间戳
    int32_t accumulated_ticks;    // 累积的脉冲数
    uint32_t last_output_time;    // 上次输出时间
    uint8_t is_active : 1;        // 是否处于活跃状态
    uint8_t direction : 1;        // 当前方向 (0=CW, 1=CCW)
};

// 配置结构
struct scroll_accelerator_config {
    struct zmk_behavior_binding cw_binding;    // 顺时针绑定
    struct zmk_behavior_binding ccw_binding;   // 逆时针绑定
    uint32_t tap_ms;                           // 点击持续时间
    uint32_t idle_reset_ms;                    // 空闲重置时间 (默认100ms)
    uint32_t min_interval_ms;                  // 最小输出间隔 (默认20ms)
    uint32_t max_interval_ms;                  // 最大输出间隔 (默认80ms)
};

// 计算动态输出间隔 - 基于滚动速度
static uint32_t calculate_output_interval(const struct scroll_accelerator_config *config,
                                          int32_t ticks_per_second)
{
    // 速度越快，间隔越短 (但不超过最大/最小限制)
    // 目标：低速时约80ms，高速时约20ms
    if (ticks_per_second <= 5) {
        return config->max_interval_ms;
    }
    if (ticks_per_second >= 50) {
        return config->min_interval_ms;
    }

    // 线性插值
    uint32_t range = config->max_interval_ms - config->min_interval_ms;
    uint32_t interval = config->max_interval_ms - (ticks_per_second * range / 50);

    return CLAMP(interval, config->min_interval_ms, config->max_interval_ms);
}

// 执行滚动输出
static void execute_scroll(const struct device *dev, int8_t direction)
{
    const struct scroll_accelerator_config *config = dev->config;
    struct scroll_accelerator_state *state = dev->data;

    struct zmk_behavior_binding binding =
        (direction > 0) ? config->cw_binding : config->ccw_binding;

    // 提交行为到队列
    zmk_behavior_queue_add(0, binding, true, config->tap_ms);

    state->last_output_time = k_uptime_get_32();
}

// 传感器数据处理回调
static int scroll_accelerator_sensor_binding_accept_data(struct zmk_behavior_binding *binding,
                                                         struct zmk_behavior_binding_event *event,
                                                         const struct zmk_sensor_config *sensor_config,
                                                         struct sensor_value value)
{
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct scroll_accelerator_state *state = dev->data;
    const struct scroll_accelerator_config *config = dev->config;

    int64_t now = k_uptime_get();
    int32_t ticks = value.val1;  // 编码器脉冲数

    if (ticks == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    int8_t current_direction = (ticks > 0) ? 1 : -1;
    ticks = (ticks > 0) ? ticks : -ticks;  // 取绝对值

    // 检查是否需要重置（方向改变或空闲超时）
    if (state->is_active) {
        bool direction_changed = (current_direction > 0) != (state->direction > 0);
        bool idle_timeout = (now - state->last_event_time) > config->idle_reset_ms;

        if (direction_changed || idle_timeout) {
            // 重置状态
            state->accumulated_ticks = 0;
            state->is_active = false;
        }
    }

    // 更新状态
    state->last_event_time = now;
    state->direction = (current_direction > 0) ? 1 : 0;
    state->is_active = true;

    // 累积脉冲
    state->accumulated_ticks += ticks;

    // 检查是否需要立即输出
    // 首次脉冲或达到输出间隔时输出
    uint32_t time_since_last = (uint32_t)(now - (int64_t)state->last_output_time);

    if (state->accumulated_ticks == ticks) {
        // 这是第一个脉冲，立即输出
        execute_scroll(dev, current_direction);
        state->accumulated_ticks = 0;
    } else {
        // 计算基于速度的间隔
        int32_t time_delta = (int32_t)(now - state->last_event_time);
        int32_t ticks_per_second = (time_delta > 0) ?
            (ticks * 1000 / time_delta) : 10;

        uint32_t output_interval = calculate_output_interval(config, ticks_per_second);

        if (time_since_last >= output_interval) {
            // 输出一次滚动
            execute_scroll(dev, current_direction);
            state->accumulated_ticks = 0;
        }
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

// 处理累积的滚动（由定时器调用）
static int scroll_accelerator_sensor_binding_process(struct zmk_behavior_binding *binding,
                                                     struct zmk_behavior_binding_event *event,
                                                     const struct zmk_sensor_config *sensor_config)
{
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct scroll_accelerator_state *state = dev->data;
    const struct scroll_accelerator_config *config = dev->config;

    if (!state->is_active || state->accumulated_ticks == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    int64_t now = k_uptime_get();

    // 检查空闲超时
    if ((now - state->last_event_time) > config->idle_reset_ms) {
        state->is_active = false;
        state->accumulated_ticks = 0;
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // 检查是否需要输出
    uint32_t time_since_last = (uint32_t)(now - (int64_t)state->last_output_time);
    int32_t time_delta = (int32_t)(now - state->last_event_time);
    int32_t ticks_per_second = (time_delta > 0) ?
        (state->accumulated_ticks * 1000 / time_delta) : 10;

    uint32_t output_interval = calculate_output_interval(config, ticks_per_second);

    if (time_since_last >= output_interval) {
        int8_t dir = state->direction ? 1 : -1;
        execute_scroll(dev, dir);
        state->accumulated_ticks = 0;
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

// 设备初始化
static int scroll_accelerator_init(const struct device *dev)
{
    struct scroll_accelerator_state *state = dev->data;

    state->last_event_time = 0;
    state->accumulated_ticks = 0;
    state->last_output_time = 0;
    state->is_active = false;
    state->direction = 0;

    return 0;
}

// 驱动API
static const struct behavior_driver_api scroll_accelerator_driver_api = {
    .sensor_binding_accept_data = scroll_accelerator_sensor_binding_accept_data,
    .sensor_binding_process = scroll_accelerator_sensor_binding_process,
};

// 设备树解析宏
#define SCROLL_ACCELERATOR_INST(n)                                           \
    static struct scroll_accelerator_state                                   \
        scroll_accelerator_state_##n = {0};                                  \
                                                                             \
    static const struct scroll_accelerator_config                            \
        scroll_accelerator_config_##n = {                                    \
            .cw_binding = ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),     \
            .ccw_binding = ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n)),    \
            .tap_ms = DT_INST_PROP_OR(n, tap_ms, 30),                        \
            .idle_reset_ms = DT_INST_PROP_OR(n, idle_reset_ms, 100),         \
            .min_interval_ms = DT_INST_PROP_OR(n, min_interval_ms, 20),      \
            .max_interval_ms = DT_INST_PROP_OR(n, max_interval_ms, 80),      \
    };                                                                       \
                                                                             \
    DEVICE_DT_INST_DEFINE(n, scroll_accelerator_init, NULL,                  \
                          &scroll_accelerator_state_##n,                     \
                          &scroll_accelerator_config_##n,                    \
                          APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,  \
                          &scroll_accelerator_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_ACCELERATOR_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
