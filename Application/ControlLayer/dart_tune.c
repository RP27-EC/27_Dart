/**
 ******************************************************************************
 * @file    dart_tune.c
 * @author  RobotPilots 镖架电控
 * @brief   PID 调参接口实现
 * @version V0.1
 * @date    2026-11-03
 ******************************************************************************
 * @attention
 *  由 dart_fsm.c 在 MANUAL 分支调用；非手动模式时状态机负责调用 Dart_Tune_Stop。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_tune.h"
#include "loader.h"
#include "tension.h"
#include "yaw.h"

/* Exported variables --------------------------------------------------------*/
/* 调试器写入的调参控制块；固件只写 active/deny */
volatile dart_tune_t g_dart_tune = {
    .enable = 0U,
    .axis = TUNE_AXIS_YAW,
    .mode = TUNE_MODE_OFF,
    .speed_rpm = 0.0f,
    .pos_target = 0,
    .pos_speed_limit = 0.0f,
    .seq = 0U,
    .active = 0U,
    .deny = 0U,
};

/* Private variables ---------------------------------------------------------*/
static mech_axis_t *s_active_axis;   /* 当前接管的轴 */
static uint32_t s_last_seq;          /* 已执行的位置请求序号 */
static uint8_t  s_last_mode;         /* 上周期模式 */

/* Private functions ---------------------------------------------------------*/
static mech_axis_t *Tune_Axis(uint8_t axis)
{
    switch (axis) {
        case TUNE_AXIS_LOADER:
            return Loader_Axis();
        case TUNE_AXIS_TENSION:
            return Tension_Axis();
        case TUNE_AXIS_YAW:
            return Yaw_Axis();
        default:
            return NULL;
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  退出调参：被接管的轴原地保持
 */
void Dart_Tune_Stop(void)
{
    if (s_active_axis != NULL) {
        Mech_Axis_Hold(s_active_axis);
        s_active_axis = NULL;
    }
    g_dart_tune.active = 0U;
}

/**
 * @brief  调参周期处理
 * @param  manual_mode 当前为手动模式
 * @param  loader_safe 储能为 RELEASED 且不在卸力中(调上膛的前提)
 * @param  now         当前时刻 ms
 * @retval true 本周期调参接管中(状态机应跳过遥控手动操作)
 */
bool Dart_Tune_Run(bool manual_mode, bool loader_safe, uint32_t now)
{
#if DART_TUNE_ENABLE
    mech_axis_t *axis;
    uint32_t deny = 0U;
    bool ok = true;

    if ((g_dart_tune.enable == 0U) || (g_dart_tune.mode == TUNE_MODE_OFF)) {
        Dart_Tune_Stop();
        return false;
    }
    axis = Tune_Axis(g_dart_tune.axis);
    if (!manual_mode) {
        deny |= TUNE_DENY_NOT_MANUAL;
    }
    if (axis == NULL) {
        deny |= TUNE_DENY_AXIS;
    } else if (!axis->zeroed) {
        deny |= TUNE_DENY_NOT_ZEROED;
    }
    if ((g_dart_tune.axis == TUNE_AXIS_LOADER) && !loader_safe) {
        deny |= TUNE_DENY_ENERGY;
    }
    if (deny != 0U) {
        g_dart_tune.deny = deny;
        g_dart_tune.enable = 0U;
        Dart_Tune_Stop();
        return false;
    }

    /* 换轴：旧轴先保持 */
    if ((s_active_axis != NULL) && (s_active_axis != axis)) {
        Mech_Axis_Hold(s_active_axis);
    }
    if (s_active_axis != axis) {
        s_last_seq = g_dart_tune.seq;
        s_active_axis = axis;
        s_last_mode = g_dart_tune.mode;
    }
    /* 切换速度/位置模式：先原地保持，避免沿用上一模式的目标 */
    if (g_dart_tune.mode != s_last_mode) {
        Mech_Axis_Hold(axis);
        s_last_seq = g_dart_tune.seq;
        s_last_mode = g_dart_tune.mode;
    }

    if (g_dart_tune.mode == TUNE_MODE_SPEED) {
        ok = Mech_Axis_Tune_Speed(axis, g_dart_tune.speed_rpm);
    } else if (g_dart_tune.seq != s_last_seq) {
        s_last_seq = g_dart_tune.seq;
        ok = Mech_Axis_Request_Move(axis, g_dart_tune.pos_target, g_dart_tune.pos_speed_limit, now);
    } else {
        /* 位置模式等待新的 seq */
    }
    if (!ok) {
        g_dart_tune.deny = TUNE_DENY_REJECT;
        g_dart_tune.enable = 0U;
        Dart_Tune_Stop();
        return false;
    }
    g_dart_tune.deny = 0U;
    g_dart_tune.active = 1U;
    return true;
#else
    (void)manual_mode;
    (void)loader_safe;
    (void)now;
    return false;
#endif
}
