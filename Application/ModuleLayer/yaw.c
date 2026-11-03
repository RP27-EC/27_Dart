/**
 ******************************************************************************
 * @file    yaw.c
 * @author  RobotPilots 镖架电控
 * @brief   平 Yaw 瞄准轴实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  找零仿照 26 国赛：独立速度环(kp1 ki0.1)以固定转速顶向限位，
 *  指令电流与转速满足阈值持续 100ms 判为堵转，零点=堵转点，再移动到 home。
 *  NOTE: 机械曾建议非拨弹板传动尽量不撞限位，已按用户决定保留堵转找零；
 *  如需改人工零点，将 zero_mode 改为 MECH_AXIS_ZERO_MANUAL。
 *  点动累计小数部分，避免 1ms 周期下小增量被截断为 0。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "yaw.h"
#include "vision_protocol.h"

/* Private macro -------------------------------------------------------------*/
#define YAW_DEG_TO_COUNT(deg)    ((int32_t)((deg) * YAW_COUNT_PER_DEG))

/* Private variables ---------------------------------------------------------*/
static const mech_axis_cfg_t s_yaw_cfg = {
    .motor_id            = DART_MOTOR_YAW,
    .dir_sign            = YAW_DIR_SIGN,
    .out_max             = DART_2006_OUT_MAX,
    .speed_max           = YAW_SPEED_MAX,
    .pos_min             = YAW_DEG_TO_COUNT(YAW_DEG_MIN),
    .pos_max             = YAW_DEG_TO_COUNT(YAW_DEG_MAX),
    .zero_mode           = MECH_AXIS_ZERO_STALL,
    .zero_speed          = YAW_ZERO_SPEED,
    .zero_out_max        = YAW_ZERO_OUT_MAX,
    .zero_spd_kp         = YAW_ZERO_SPD_KP,
    .zero_spd_ki         = YAW_ZERO_SPD_KI,
    .zero_spd_imax       = YAW_ZERO_SPD_IMAX,
    .zero_timeout_ms     = YAW_ZERO_TIMEOUT_MS,
    .home_after_zero     = true,
    .home_pos            = YAW_DEG_TO_COUNT(YAW_DEG_CENTER),
    .arrive_err          = YAW_ARRIVE_ERR,
    .arrive_speed        = YAW_ARRIVE_SPEED,
    .arrive_hold_ms      = YAW_ARRIVE_HOLD_MS,
    .move_timeout_ms     = YAW_MOVE_TIMEOUT_MS,
    .hold_stall_err      = YAW_HOLD_STALL_ERR,
    .jog_lead_max        = YAW_JOG_LEAD_MAX,
    .stall_check_in_move = true,
    .stall_out_min       = YAW_STALL_OUT_MIN,
    .stall_speed_max     = YAW_STALL_SPEED_MAX,
    .stall_hold_ms       = STALL_HOLD_MS,
    .hold_on_fault       = false,
    .pos_kp              = YAW_POS_KP,
    .spd_kp              = YAW_SPD_KP,
    .spd_ki              = YAW_SPD_KI,
    .spd_imax            = YAW_SPD_IMAX,
};

/* 每发目标角 deg，下标 0~3 对应第 1~4 发 */
static const float s_yaw_shot_deg[DART_SHOT_MAX] = YAW_SHOT_DEG_TABLE;

static mech_axis_t s_yaw_axis;
static float s_jog_remainder;   /* 点动未满 1count 的余量 */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Yaw 初始化
 */
void Yaw_Init(void)
{
    Mech_Axis_Init(&s_yaw_axis, &s_yaw_cfg);
    s_jog_remainder = 0.0f;
}

/**
 * @brief  周期计算
 * @param  now 当前时刻 ms
 */
void Yaw_Step(uint32_t now)
{
    Mech_Axis_Step(&s_yaw_axis, now);
}

/**
 * @brief  取得 Yaw 轴对象
 */
mech_axis_t *Yaw_Axis(void)
{
    return &s_yaw_axis;
}

/**
 * @brief  请求转到指定角度
 * @param  deg 目标 deg(相对找零基准)
 * @retval false 越限或轴不接受
 */
bool Yaw_Request_Deg(float deg, uint32_t now)
{
    /* 越限由轴对象按 RAM 软限位判断，便于测量时在线放宽 */
    return Mech_Axis_Request_Move(&s_yaw_axis, YAW_DEG_TO_COUNT(deg), 0.0f, now);
}

/**
 * @brief  请求转到第 shot_no 发预设角(含视觉修正)
 * @param  shot_no 发序 1~4
 */
bool Yaw_Request_Shot(uint8_t shot_no, uint32_t now)
{
    if ((shot_no == 0U) || (shot_no > DART_SHOT_MAX)) {
        return false;
    }
    return Yaw_Request_Deg(s_yaw_shot_deg[shot_no - 1U] + Yaw_Vision_Trim_Deg(), now);
}

/**
 * @brief  点动
 * @param  delta_deg 本周期增量 deg
 */
void Yaw_Jog_Deg(float delta_deg)
{
    float count_f = delta_deg * YAW_COUNT_PER_DEG + s_jog_remainder;
    int32_t count_i = (int32_t)count_f;

    s_jog_remainder = count_f - (float)count_i;
    Mech_Axis_Jog(&s_yaw_axis, count_i);
}

/**
 * @brief  当前角度 deg
 */
float Yaw_Get_Deg(void)
{
    return (float)s_yaw_axis.pos / YAW_COUNT_PER_DEG;
}

/**
 * @brief  Yaw 已到位稳定
 */
bool Yaw_Is_Ready(void)
{
    return Mech_Axis_Is_Ready(&s_yaw_axis);
}

/**
 * @brief  发射时 Yaw 就绪(保持带，比到位判定宽)
 */
bool Yaw_Is_Fire_Ready(void)
{
    return Mech_Axis_Is_Ready_Band(&s_yaw_axis, YAW_FIRE_HOLD_ERR, YAW_FIRE_HOLD_SPEED);
}

/**
 * @brief  视觉修正量(钩子)
 * @retval 修正角 deg；视觉未接入时为 0
 * @note   TODO: 仅使用新时间戳帧，误差稳定判定与限幅
 */
float Yaw_Vision_Trim_Deg(void)
{
#if DART_VISION_ENABLE
    float err_deg;
    uint32_t stamp_ms;

    if (Vision_Get_Yaw_Error(&err_deg, &stamp_ms)) {
        return 0.0f;
    }
#endif
    return 0.0f;
}
