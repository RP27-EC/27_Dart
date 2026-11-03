/**
 ******************************************************************************
 * @file    loader.c
 * @author  RobotPilots 镖架电控
 * @brief   上膛机构实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  预设位置与速度见 dart_config.h 上膛段，全部待新机构标定。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "loader.h"

/* Private variables ---------------------------------------------------------*/
static const mech_axis_cfg_t s_loader_cfg = {
    .motor_id            = DART_MOTOR_LOADER_L,
    .dir_sign            = LOADER_DIR_SIGN,
    .out_max             = DART_3508_OUT_MAX,
    .speed_max           = LOADER_SPEED_FAST,
    .pos_min             = LOADER_POS_MIN,
    .pos_max             = LOADER_POS_MAX,
    .zero_mode           = MECH_AXIS_ZERO_STALL,
    .zero_speed          = LOADER_ZERO_SPEED,
    .zero_out_max        = LOADER_ZERO_OUT_MAX,
    .zero_spd_kp         = LOADER_ZERO_SPD_KP,
    .zero_spd_ki         = LOADER_ZERO_SPD_KI,
    .zero_spd_imax       = LOADER_ZERO_SPD_IMAX,
    .zero_timeout_ms     = LOADER_ZERO_TIMEOUT_MS,
    .home_after_zero     = true,
    .home_pos            = LOADER_POS_UP,
    .arrive_err          = LOADER_ARRIVE_ERR,
    .arrive_speed        = LOADER_ARRIVE_SPEED,
    .arrive_hold_ms      = LOADER_ARRIVE_HOLD_MS,
    .move_timeout_ms     = LOADER_MOVE_TIMEOUT_MS,
    .hold_stall_err      = LOADER_HOLD_STALL_ERR,
    .jog_lead_max        = LOADER_JOG_LEAD_MAX,
    .stall_check_in_move = true,
    .stall_out_min       = LOADER_STALL_OUT_MIN,
    .stall_speed_max     = LOADER_STALL_SPEED_MAX,
    .stall_hold_ms       = STALL_HOLD_MS,
    .hold_on_fault       = true,
    .pos_kp              = LOADER_POS_KP,
    .spd_kp              = LOADER_SPD_KP,
    .spd_ki              = LOADER_SPD_KI,
    .spd_imax            = LOADER_SPD_IMAX,
};

/* 预设位置 count 与转速上限 rpm，下标为 loader_pos_e */
static const int32_t s_loader_pos[] = { LOADER_POS_UP, LOADER_POS_MID, LOADER_POS_DOWN, LOADER_POS_CATCH };
static const float   s_loader_speed[] = { LOADER_SPEED_FAST, LOADER_SPEED_FAST, LOADER_SPEED_SLOW, LOADER_SPEED_SLOW };

static mech_axis_t s_loader_axis;
static bool     s_single_side;        /* 仅 R 离线，L 单独工作 */
static uint32_t s_single_side_tick;   /* 进入单侧状态时刻 ms */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  上膛机构初始化
 */
void Loader_Init(void)
{
    Mech_Axis_Init(&s_loader_axis, &s_loader_cfg);
}

/**
 * @brief  周期计算：L 闭环，R 镜像输出
 * @param  now 当前时刻 ms
 */
void Loader_Step(uint32_t now)
{
    float out_l_raw;
    bool r_online = Dart_Motor_Is_Online(DART_MOTOR_LOADER_R);

    /* 仅 R 离线：L 原地保持(下降沿一次)，零点仍以 L 编码器为准 */
    if (!r_online && Dart_Motor_Is_Online(DART_MOTOR_LOADER_L)) {
        if (!s_single_side) {
            s_single_side = true;
            s_single_side_tick = now;
            Mech_Axis_Hold(&s_loader_axis);
        }
    } else {
        s_single_side = false;
    }
    Mech_Axis_Step(&s_loader_axis, now);

    out_l_raw = (float)s_loader_cfg.dir_sign * s_loader_axis.output;
    Dart_Motor_Set_Output(DART_MOTOR_LOADER_R, r_online ? (LOADER_R_MIRROR_SIGN * out_l_raw) : 0.0f);
}

/**
 * @brief  取得上膛轴对象(供控制层发通用请求)
 */
mech_axis_t *Loader_Axis(void)
{
    return &s_loader_axis;
}

/**
 * @brief  请求运动到预设位置
 * @retval false 越界或轴不接受
 */
bool Loader_Request_Pos(loader_pos_e pos, uint32_t now)
{
    if ((uint32_t)pos >= (sizeof(s_loader_pos) / sizeof(s_loader_pos[0]))) {
        return false;
    }
    return Mech_Axis_Request_Move(&s_loader_axis, s_loader_pos[pos], s_loader_speed[pos], now);
}

/**
 * @brief  已找零且当前处于预设位置附近(瞬时)
 */
bool Loader_Is_At(loader_pos_e pos)
{
    if ((uint32_t)pos >= (sizeof(s_loader_pos) / sizeof(s_loader_pos[0]))) {
        return false;
    }
    return s_loader_axis.zeroed && Mech_Axis_Is_At(&s_loader_axis, s_loader_pos[pos]);
}

/**
 * @brief  目标为该预设位置且已到位稳定
 */
bool Loader_Is_Ready_At(loader_pos_e pos)
{
    if ((uint32_t)pos >= (sizeof(s_loader_pos) / sizeof(s_loader_pos[0]))) {
        return false;
    }
    return Mech_Axis_Is_Ready(&s_loader_axis) && (s_loader_axis.target == s_loader_pos[pos]);
}

/**
 * @brief  发射时上膛已回到 UP 附近(保持带)，拨弹板与滑台分离
 */
bool Loader_Is_Fire_Clear(void)
{
    return s_loader_axis.zeroed &&
           (Dart_Abs_F((float)(LOADER_POS_UP - s_loader_axis.pos)) <= LOADER_FIRE_HOLD_ERR) &&
           (Dart_Abs_F(s_loader_axis.speed) <= LOADER_FIRE_HOLD_SPEED);
}

/**
 * @brief  是否处于单侧工作(仅 R 离线)
 */
bool Loader_Is_Single_Side(void)
{
    return s_single_side;
}

/**
 * @brief  进入单侧工作的时刻 ms
 */
uint32_t Loader_Single_Side_Tick(void)
{
    return s_single_side_tick;
}
