/**
 ******************************************************************************
 * @file    tension.c
 * @author  RobotPilots 镖架电控
 * @brief   变力机构实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  找零仿照 26 国赛：独立速度环(kp1 ki0.1)以固定转速顶向限位，
 *  指令电流与转速满足阈值持续 100ms 判为堵转，零点=堵转点，再移动到 home。
 *  NOTE: 机械曾建议非拨弹板传动尽量不撞限位，已按用户决定保留堵转找零；
 *  如需改人工零点，将 zero_mode 改为 MECH_AXIS_ZERO_MANUAL。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tension.h"
#include "loadcell.h"

/* Private variables ---------------------------------------------------------*/
static const mech_axis_cfg_t s_tension_cfg = {
    .motor_id            = DART_MOTOR_TENSION,
    .dir_sign            = TENSION_DIR_SIGN,
    .out_max             = DART_3508_OUT_MAX,
    .speed_max           = TENSION_SPEED_MAX,
    .pos_min             = TENSION_POS_MIN,
    .pos_max             = TENSION_POS_MAX,
    .zero_mode           = MECH_AXIS_ZERO_STALL,
    .zero_speed          = TENSION_ZERO_SPEED,
    .zero_out_max        = TENSION_ZERO_OUT_MAX,
    .zero_spd_kp         = TENSION_ZERO_SPD_KP,
    .zero_spd_ki         = TENSION_ZERO_SPD_KI,
    .zero_spd_imax       = TENSION_ZERO_SPD_IMAX,
    .zero_timeout_ms     = TENSION_ZERO_TIMEOUT_MS,
    .home_after_zero     = true,
    .home_pos            = TENSION_ZERO_BACKOFF,
    .arrive_err          = TENSION_ARRIVE_ERR,
    .arrive_speed        = TENSION_ARRIVE_SPEED,
    .arrive_hold_ms      = TENSION_ARRIVE_HOLD_MS,
    .move_timeout_ms     = TENSION_MOVE_TIMEOUT_MS,
    .hold_stall_err      = TENSION_HOLD_STALL_ERR,
    .jog_lead_max        = TENSION_JOG_LEAD_MAX,
    .stall_check_in_move = true,
    .stall_out_min       = TENSION_STALL_OUT_MIN,
    .stall_speed_max     = TENSION_STALL_SPEED_MAX,
    .stall_hold_ms       = STALL_HOLD_MS,
    .hold_on_fault       = false,
    .pos_kp              = TENSION_POS_KP,
    .spd_kp              = TENSION_SPD_KP,
    .spd_ki              = TENSION_SPD_KI,
    .spd_imax            = TENSION_SPD_IMAX,
};

/* 每发目标位置 count，下标 0~3 对应第 1~4 发 */
static const int32_t s_tension_shot_pos[DART_SHOT_MAX] = TENSION_SHOT_POS_TABLE;

static mech_axis_t s_tension_axis;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  变力机构初始化
 */
void Tension_Init(void)
{
    Mech_Axis_Init(&s_tension_axis, &s_tension_cfg);
}

/**
 * @brief  周期计算
 * @param  now 当前时刻 ms
 */
void Tension_Step(uint32_t now)
{
    Mech_Axis_Step(&s_tension_axis, now);
}

/**
 * @brief  取得变力轴对象
 */
mech_axis_t *Tension_Axis(void)
{
    return &s_tension_axis;
}

/**
 * @brief  请求运动到第 shot_no 发的预紧位置
 * @param  shot_no 发序 1~4
 * @retval false 发序非法或轴不接受
 */
bool Tension_Request_Shot(uint8_t shot_no, uint32_t now)
{
    int32_t target;

    if ((shot_no == 0U) || (shot_no > DART_SHOT_MAX)) {
        return false;
    }
    target = s_tension_shot_pos[shot_no - 1U] + Tension_Force_Trim();
    return Mech_Axis_Request_Move(&s_tension_axis, target, 0.0f, now);
}

/**
 * @brief  变力已到位稳定且力就绪
 */
bool Tension_Is_Ready(void)
{
    return Mech_Axis_Is_Ready(&s_tension_axis) && Loadcell_Force_Ready();
}

/**
 * @brief  发射时变力就绪(保持带)且力就绪
 */
bool Tension_Is_Fire_Ready(void)
{
    return Mech_Axis_Is_Ready_Band(&s_tension_axis, TENSION_FIRE_HOLD_ERR, TENSION_FIRE_HOLD_SPEED) &&
           Loadcell_Force_Ready();
}

/**
 * @brief  力闭环修正量(钩子)
 * @retval 位置修正 count；测力未接入时为 0
 * @note   TODO: 锁止后以新样本做慢速有界修正，单次/总修正量受限
 */
int32_t Tension_Force_Trim(void)
{
    return 0;
}
