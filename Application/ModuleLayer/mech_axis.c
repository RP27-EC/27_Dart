/**
 ******************************************************************************
 * @file    mech_axis.c
 * @author  RobotPilots 镖架电控
 * @brief   单电机位置轴实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  串级：位置误差 -> 位置环 -> 目标转速(限 speed_limit)
 *        -> 速度环 -> 电流指令(限 out_max)。
 *  PID 误差由本文件计算后写入 err，再调用 single_pid_ctrl。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "mech_axis.h"
#include <string.h>

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  清空两个环的积分与历史误差，切换控制方式时调用
 */
static void Mech_Axis_Reset_Pid(mech_axis_t *axis)
{
    axis->pos_pid.integral = 0.0f;
    axis->pos_pid.last_err = 0.0f;
    axis->pos_pid.dout     = 0.0f;
    axis->spd_pid.integral = 0.0f;
    axis->spd_pid.last_err = 0.0f;
    axis->spd_pid.dout     = 0.0f;
    axis->zero_pid.integral = 0.0f;
    axis->zero_pid.last_err = 0.0f;
    axis->zero_pid.dout     = 0.0f;
}

/**
 * @brief  转速闭环
 * @param  pid          所用速度环(位置内环或找零环)
 * @param  target_speed 目标转速 rpm
 * @param  out_limit    输出限幅 raw
 * @retval 输出 raw
 */
static float Mech_Axis_Speed_Loop(mech_axis_t *axis, pid_ctrl_t *pid,
                                  float target_speed, float out_limit)
{
    pid->target  = target_speed;
    pid->measure = axis->speed;
    pid->err     = target_speed - axis->speed;
    pid->out_max = out_limit;
    single_pid_ctrl(pid);
    return pid->out;
}

/**
 * @brief  位置串级闭环
 * @retval 输出 raw
 */
static float Mech_Axis_Position_Loop(mech_axis_t *axis)
{
    axis->pos_pid.target  = (float)axis->target;
    axis->pos_pid.measure = (float)axis->pos;
    axis->pos_pid.err     = (float)(axis->target - axis->pos);
    axis->pos_pid.out_max = axis->speed_limit;
    single_pid_ctrl(&axis->pos_pid);
    return Mech_Axis_Speed_Loop(axis, &axis->spd_pid, axis->pos_pid.out, axis->cfg->out_max);
}

/**
 * @brief  堵转条件判定：|输出|大、|转速|小并持续 stall_hold_ms
 * @retval true 判定为堵转
 * @note   判据与 26 一致(指令电流+转速+持续 100ms)；26 转速未取绝对值、
 *         且多电机共用计数，此处改为取绝对值并每轴独立计时
 */
static bool Mech_Axis_Stall_Update(mech_axis_t *axis, uint32_t now)
{
    const mech_axis_cfg_t *cfg = axis->cfg;
    bool cond = (Dart_Abs_F(axis->output) >= cfg->stall_out_min) &&
                (Dart_Abs_F(axis->speed) <= cfg->stall_speed_max);

    if (!cond) {
        axis->stall_active = false;
        return false;
    }
    if (!axis->stall_active) {
        axis->stall_active = true;
        axis->stall_tick = now;
    }
    return Dart_Time_Elapsed(now, axis->stall_tick, cfg->stall_hold_ms);
}

/**
 * @brief  到位条件判定：误差与转速同时满足并持续 arrive_hold_ms
 * @retval true 到位稳定
 */
static bool Mech_Axis_Settle_Update(mech_axis_t *axis, uint32_t now)
{
    if (!Mech_Axis_Is_At(axis, axis->target)) {
        axis->settle_active = false;
        return false;
    }
    if (!axis->settle_active) {
        axis->settle_active = true;
        axis->settle_tick = now;
    }
    return Dart_Time_Elapsed(now, axis->settle_tick, axis->cfg->arrive_hold_ms);
}

/**
 * @brief  进入故障；hold_on_fault 时在当前位置保持，否则无输出
 */
static void Mech_Axis_Enter_Fault(mech_axis_t *axis, mech_axis_err_e err)
{
    axis->status = MECH_AXIS_FAULT;
    axis->err    = err;
    axis->target = axis->pos;
    axis->speed_limit = axis->cfg->speed_max;
    Mech_Axis_Reset_Pid(axis);
    if (err == MECH_AXIS_ERR_OFFLINE) {
        axis->zeroed = false;
    }
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化轴对象，上电为 DISABLED 且无零点
 * @param  axis 轴对象
 * @param  cfg  配置表(须为静态常量)
 */
void Mech_Axis_Init(mech_axis_t *axis, const mech_axis_cfg_t *cfg)
{
    memset(axis, 0, sizeof(*axis));
    axis->cfg = cfg;
    axis->pos_pid.kp = cfg->pos_kp;
    axis->pos_pid.out_max = cfg->speed_max;
    axis->spd_pid.kp = cfg->spd_kp;
    axis->spd_pid.ki = cfg->spd_ki;
    axis->spd_pid.integral_max = cfg->spd_imax;
    axis->spd_pid.out_max = cfg->out_max;
    axis->zero_pid.kp = cfg->zero_spd_kp;
    axis->zero_pid.ki = cfg->zero_spd_ki;
    axis->zero_pid.integral_max = cfg->zero_spd_imax;
    axis->zero_pid.out_max = cfg->zero_out_max;
    axis->speed_limit = cfg->speed_max;
    axis->home_pos = cfg->home_pos;
    axis->pos_min = cfg->pos_min;
    axis->pos_max = cfg->pos_max;
    axis->move_timeout = cfg->move_timeout_ms;
    axis->status = MECH_AXIS_DISABLED;
    axis->err = MECH_AXIS_ERR_NONE;
}

/**
 * @brief  周期计算，ControlTask 每 1ms 调用一次
 * @param  axis 轴对象
 * @param  now  当前时刻 ms
 * @note   计算结果直接写入对应电机发送缓存
 */
void Mech_Axis_Step(mech_axis_t *axis, uint32_t now)
{
    const mech_axis_cfg_t *cfg = axis->cfg;
    int32_t raw_sum = Dart_Motor_Get_Encoder_Sum(cfg->motor_id);

    axis->pos   = (int32_t)cfg->dir_sign * (raw_sum - axis->zero_ref);
    axis->speed = (float)cfg->dir_sign * Dart_Motor_Get_Speed_Rpm(cfg->motor_id);

    if (!Dart_Motor_Is_Online(cfg->motor_id)) {
        /* 离线期间累计计数不可信，零点作废 */
        axis->zeroed = false;
        if ((axis->status != MECH_AXIS_DISABLED) &&
            !((axis->status == MECH_AXIS_FAULT) && (axis->err == MECH_AXIS_ERR_OFFLINE))) {
            Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_OFFLINE);
        }
    }

    switch (axis->status) {
        case MECH_AXIS_ZEROING:
            axis->output = Mech_Axis_Speed_Loop(axis, &axis->zero_pid, cfg->zero_speed, cfg->zero_out_max);
            if (Mech_Axis_Stall_Update(axis, now)) {
                /* 堵转点即零点 */
                axis->zero_ref = raw_sum;
                axis->zeroed   = true;
                axis->pos      = 0;
                axis->target   = 0;
                axis->stall_active = false;
                Mech_Axis_Reset_Pid(axis);
                axis->status = MECH_AXIS_HOLD;
                if (cfg->home_after_zero) {
                    (void)Mech_Axis_Request_Move(axis, axis->home_pos, cfg->speed_max, now);
                }
            } else if (Dart_Time_Elapsed(now, axis->phase_tick, cfg->zero_timeout_ms)) {
                Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_ZERO_TIMEOUT);
            }
            break;

        case MECH_AXIS_MOVING:
            axis->output = Mech_Axis_Position_Loop(axis);
            if (Mech_Axis_Settle_Update(axis, now)) {
                axis->status = MECH_AXIS_ARRIVED;
            } else if (cfg->stall_check_in_move && Mech_Axis_Stall_Update(axis, now)) {
                Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_STALL);
            } else if (Dart_Time_Elapsed(now, axis->phase_tick, axis->move_timeout)) {
                Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_MOVE_TIMEOUT);
            }
            break;

        case MECH_AXIS_HOLD:
            axis->output = Mech_Axis_Position_Loop(axis);
            /* 目标追不上(如点动顶死)才判卡死；正常承载保持误差小，不判 */
            if (cfg->stall_check_in_move &&
                (Dart_Abs_F((float)(axis->target - axis->pos)) > (float)cfg->hold_stall_err)) {
                if (Mech_Axis_Stall_Update(axis, now)) {
                    Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_STALL);
                }
            } else {
                axis->stall_active = false;
            }
            break;

        case MECH_AXIS_ARRIVED:
            axis->output = Mech_Axis_Position_Loop(axis);
            break;

        case MECH_AXIS_TUNE_SPEED:
            if (((axis->pos >= axis->pos_max) && (axis->tune_speed > 0.0f)) ||
                ((axis->pos <= axis->pos_min) && (axis->tune_speed < 0.0f))) {
                /* 到软限位：转为原地保持 */
                Mech_Axis_Hold(axis);
                axis->output = Mech_Axis_Position_Loop(axis);
            } else {
                axis->output = Mech_Axis_Speed_Loop(axis, &axis->spd_pid, axis->tune_speed, cfg->out_max);
                if (cfg->stall_check_in_move && Mech_Axis_Stall_Update(axis, now)) {
                    Mech_Axis_Enter_Fault(axis, MECH_AXIS_ERR_STALL);
                }
            }
            break;

        case MECH_AXIS_FAULT:
            if (cfg->hold_on_fault && (axis->err != MECH_AXIS_ERR_OFFLINE)) {
                /* NOTE: 上膛中途故障时保持，避免滑台回弹 */
                axis->output = Mech_Axis_Position_Loop(axis);
            } else {
                axis->output = 0.0f;
            }
            break;

        case MECH_AXIS_DISABLED:
        default:
            axis->target = axis->pos;
            axis->output = 0.0f;
            break;
    }

    Dart_Motor_Set_Output(cfg->motor_id, (float)cfg->dir_sign * axis->output);
}

/**
 * @brief  DISABLED -> HOLD，在当前位置保持
 */
void Mech_Axis_Enable(mech_axis_t *axis)
{
    if (axis->status != MECH_AXIS_DISABLED) {
        return;
    }
    axis->target = axis->pos;
    axis->speed_limit = axis->cfg->speed_max;
    Mech_Axis_Reset_Pid(axis);
    axis->status = MECH_AXIS_HOLD;
}

/**
 * @brief  任意状态 -> DISABLED(无输出)，保留零点与故障记录之外的状态
 * @note   FAULT 不会被 Disable 覆盖，需 Clear_Fault 显式确认
 */
void Mech_Axis_Disable(mech_axis_t *axis)
{
    if (axis->status == MECH_AXIS_FAULT) {
        return;
    }
    axis->status = MECH_AXIS_DISABLED;
    axis->output = 0.0f;
    Mech_Axis_Reset_Pid(axis);
}

/**
 * @brief  请求堵转找零
 * @retval false 配置为人工零点，或当前为 DISABLED/FAULT/已在找零
 */
bool Mech_Axis_Request_Zero(mech_axis_t *axis, uint32_t now)
{
    if ((axis->cfg->zero_mode != MECH_AXIS_ZERO_STALL) ||
        (axis->status == MECH_AXIS_DISABLED) ||
        (axis->status == MECH_AXIS_FAULT) ||
        (axis->status == MECH_AXIS_ZEROING)) {
        return false;
    }
    axis->zeroed = false;
    axis->stall_active = false;
    axis->phase_tick = now;
    Mech_Axis_Reset_Pid(axis);
    axis->status = MECH_AXIS_ZEROING;
    return true;
}

/**
 * @brief  请求运动到逻辑目标
 * @param  target      目标 count
 * @param  speed_limit 转速上限 rpm，≤0 时用默认值
 * @retval false 未找零、目标越限、找零中或处于 DISABLED/FAULT
 * @note   目标与当前已到位目标相同时保持 ARRIVED，不重新计时
 */
bool Mech_Axis_Request_Move(mech_axis_t *axis, int32_t target, float speed_limit, uint32_t now)
{
    const mech_axis_cfg_t *cfg = axis->cfg;

    if (!axis->zeroed ||
        (axis->status == MECH_AXIS_DISABLED) ||
        (axis->status == MECH_AXIS_FAULT) ||
        (axis->status == MECH_AXIS_ZEROING)) {
        return false;
    }
    if ((target < axis->pos_min) || (target > axis->pos_max)) {
        return false;
    }
    if ((axis->status == MECH_AXIS_ARRIVED) && (axis->target == target)) {
        return true;
    }
    axis->target = target;
    axis->speed_limit = ((speed_limit > 0.0f) && (speed_limit < cfg->speed_max)) ?
                        speed_limit : cfg->speed_max;
    axis->move_timeout = Mech_Axis_Move_Timeout(axis, target - axis->pos, axis->speed_limit);
    axis->phase_tick = now;
    axis->settle_active = false;
    axis->stall_active = false;
    axis->status = MECH_AXIS_MOVING;
    return true;
}

/**
 * @brief  在当前位置保持(中止运动)
 */
void Mech_Axis_Hold(mech_axis_t *axis)
{
    if ((axis->status == MECH_AXIS_DISABLED) || (axis->status == MECH_AXIS_FAULT)) {
        return;
    }
    axis->target = axis->pos;
    axis->speed_limit = axis->cfg->speed_max;
    axis->status = MECH_AXIS_HOLD;
}

/**
 * @brief  回到 home_pos(找零后回转位)
 * @retval false 未找零或轴不接受
 */
bool Mech_Axis_Request_Home(mech_axis_t *axis, uint32_t now)
{
    return Mech_Axis_Request_Move(axis, axis->home_pos, 0.0f, now);
}

/**
 * @brief  调试速度模式：内环单独闭环到 speed_rpm
 * @param  speed_rpm 逻辑目标转速 rpm，可每周期更新
 * @retval false 未找零或处于 DISABLED/FAULT/ZEROING
 * @note   仅供 PID 调参；退出时调用 Mech_Axis_Hold
 */
bool Mech_Axis_Tune_Speed(mech_axis_t *axis, float speed_rpm)
{
    if (!axis->zeroed ||
        (axis->status == MECH_AXIS_DISABLED) ||
        (axis->status == MECH_AXIS_FAULT) ||
        (axis->status == MECH_AXIS_ZEROING)) {
        return false;
    }
    if (axis->status != MECH_AXIS_TUNE_SPEED) {
        axis->spd_pid.integral = 0.0f;
        axis->stall_active = false;
    }
    axis->tune_speed = speed_rpm;
    axis->status = MECH_AXIS_TUNE_SPEED;
    return true;
}

/**
 * @brief  点动：目标增加 delta
 * @param  delta 增量 count
 * @note   已找零时受软限位约束；未找零时不限位，仅供人工对零前使用
 */
void Mech_Axis_Jog(mech_axis_t *axis, int32_t delta)
{
    const mech_axis_cfg_t *cfg = axis->cfg;
    int32_t target;

    if ((axis->status == MECH_AXIS_DISABLED) ||
        (axis->status == MECH_AXIS_FAULT) ||
        (axis->status == MECH_AXIS_ZEROING) ||
        (delta == 0)) {
        return;
    }
    target = axis->target + delta;
    /* 目标最多超前实际位置 jog_lead_max，顶住时不会越积越远 */
    if (target > axis->pos + cfg->jog_lead_max) {
        target = axis->pos + cfg->jog_lead_max;
    }
    if (target < axis->pos - cfg->jog_lead_max) {
        target = axis->pos - cfg->jog_lead_max;
    }
    if (axis->zeroed) {
        if (target < axis->pos_min) {
            target = axis->pos_min;
        }
        if (target > axis->pos_max) {
            target = axis->pos_max;
        }
    }
    axis->target = target;
    axis->speed_limit = cfg->speed_max;
    axis->status = MECH_AXIS_HOLD;
}

/**
 * @brief  人工零点：把当前位置定义为 0
 * @note   用于不允许撞限位的机构，须人工对准基准后调用
 */
void Mech_Axis_Set_Zero_Here(mech_axis_t *axis)
{
    if ((axis->status == MECH_AXIS_DISABLED) || (axis->status == MECH_AXIS_FAULT)) {
        return;
    }
    axis->zero_ref = Dart_Motor_Get_Encoder_Sum(axis->cfg->motor_id);
    axis->zeroed = true;
    axis->pos = 0;
    axis->target = 0;
    Mech_Axis_Reset_Pid(axis);
    axis->status = MECH_AXIS_HOLD;
}

/**
 * @brief  外部强制故障(如上膛 R 电机离线)
 */
void Mech_Axis_Force_Fault(mech_axis_t *axis, mech_axis_err_e err)
{
    /* 离线优先级最高：可覆盖已有的其它故障，确保停止输出 */
    if ((axis->status == MECH_AXIS_FAULT) &&
        ((err != MECH_AXIS_ERR_OFFLINE) || (axis->err == MECH_AXIS_ERR_OFFLINE))) {
        return;
    }
    Mech_Axis_Enter_Fault(axis, err);
}

/**
 * @brief  确认故障：FAULT -> DISABLED，需再次 Enable
 */
void Mech_Axis_Clear_Fault(mech_axis_t *axis)
{
    if (axis->status != MECH_AXIS_FAULT) {
        return;
    }
    axis->err = MECH_AXIS_ERR_NONE;
    axis->status = MECH_AXIS_DISABLED;
    axis->output = 0.0f;
    Mech_Axis_Reset_Pid(axis);
}

/**
 * @brief  当前是否在指定位置附近且基本静止(瞬时判断)
 * @param  pos 逻辑位置 count
 */
bool Mech_Axis_Is_At(const mech_axis_t *axis, int32_t pos)
{
    return (Dart_Abs_F((float)(pos - axis->pos)) <= axis->cfg->arrive_err) &&
           (Dart_Abs_F(axis->speed) <= axis->cfg->arrive_speed);
}

/**
 * @brief  已找零、到位稳定且当前仍在目标附近
 */
bool Mech_Axis_Is_Ready(const mech_axis_t *axis)
{
    return axis->zeroed &&
           (axis->status == MECH_AXIS_ARRIVED) &&
           Mech_Axis_Is_At(axis, axis->target);
}

/**
 * @brief  发射用就绪判断(保持带)：曾到位稳定，且当前仍在较宽的带内
 * @param  err_band   允许位置偏离 count
 * @param  speed_band 允许转速 rpm
 * @note   任何新请求、点动或保持都会使状态离开 ARRIVED，旧就绪随之失效
 */
bool Mech_Axis_Is_Ready_Band(const mech_axis_t *axis, float err_band, float speed_band)
{
    return axis->zeroed &&
           (axis->status == MECH_AXIS_ARRIVED) &&
           (Dart_Abs_F((float)(axis->target - axis->pos)) <= err_band) &&
           (Dart_Abs_F(axis->speed) <= speed_band);
}

/**
 * @brief  按距离与限速计算运动超时
 * @param  dist      运动距离 count(可为负)
 * @param  speed_rpm 限速 rpm(转子)
 * @retval 超时 ms = max(配置下限, 理论时间 x 1.5 + 1000)
 */
uint32_t Mech_Axis_Move_Timeout(const mech_axis_t *axis, int32_t dist, float speed_rpm)
{
    float count_per_ms = Dart_Abs_F(speed_rpm) * 8192.0f / 60000.0f;
    float need_ms;
    uint32_t timeout;

    if (count_per_ms < 1.0f) {
        count_per_ms = 1.0f;
    }
    need_ms = Dart_Abs_F((float)dist) / count_per_ms;
    timeout = (uint32_t)(need_ms * 1.5f) + 1000U;
    return (timeout > axis->cfg->move_timeout_ms) ? timeout : axis->cfg->move_timeout_ms;
}
