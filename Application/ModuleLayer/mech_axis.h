/**
 ******************************************************************************
 * @file    mech_axis.h
 * @author  RobotPilots 镖架电控
 * @brief   单电机位置轴：堵转找零、串级位置控制、到位/超时/堵转判定
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  上膛、变力、Yaw 三个机构共用本对象，差异只在配置表。
 *  1. 逻辑位置 pos = dir_sign * (encoder_sum - zero_ref)，单位 count；
 *     逻辑转速与输出同样乘 dir_sign，安装方向只改配置。
 *  2. 只在 ControlTask 中调用 Step 与各请求函数(单一写者)。
 *  3. 到位判定要求误差与转速同时满足并持续 arrive_hold_ms，
 *     不以单次误差为零判定到位。
 *  4. 电机离线时轴进入 FAULT/OFFLINE 并失去零点，必须重新找零。
 *  5. home_pos 在初始化时从配置表复制到 RAM，调试器改写后下次找零或
 *     Mech_Axis_Request_Home 即按新值回转；掉电恢复为配置值。
 *  6. TUNE_SPEED 只供调参：纯速度环，到达软限位方向即转为保持。
 *  7. 软限位 pos_min/pos_max 同样复制到 RAM，测量新行程时可用调试器临时放宽。
 *  8. 运动超时 = max(配置下限, 距离/限速 x 1.5 + 1s)，长行程低速不会误报超时。
 *  9. HOLD 时若目标与位置相差超过 hold_stall_err 且满足堵转条件，判为卡死；
 *     正常顶着负载保持(误差小)不误报。点动目标最多超前实际位置 jog_lead_max。
 * 10. 离线故障优先级最高，可覆盖已有的其它故障。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __MECH_AXIS_H
#define __MECH_AXIS_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "dart_motor.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  轴运行状态
 */
typedef enum {
    MECH_AXIS_DISABLED = 0,  /* 无输出，跟踪当前位置 */
    MECH_AXIS_ZEROING,       /* 转速环堵转找零 */
    MECH_AXIS_HOLD,          /* 位置保持(含点动) */
    MECH_AXIS_MOVING,        /* 运动到目标中 */
    MECH_AXIS_ARRIVED,       /* 已到位稳定 */
    MECH_AXIS_TUNE_SPEED,    /* 调试：纯速度环 */
    MECH_AXIS_FAULT,         /* 故障，见 err */
} mech_axis_status_e;

/**
 * @brief  轴故障原因
 */
typedef enum {
    MECH_AXIS_ERR_NONE = 0,      /* 无 */
    MECH_AXIS_ERR_ZERO_TIMEOUT,  /* 找零超时 */
    MECH_AXIS_ERR_MOVE_TIMEOUT,  /* 运动超时 */
    MECH_AXIS_ERR_STALL,         /* 运动中堵转 */
    MECH_AXIS_ERR_OFFLINE,       /* 电机离线 */
} mech_axis_err_e;

/**
 * @brief  找零方式
 */
typedef enum {
    MECH_AXIS_ZERO_STALL = 0,    /* 转速环堵转找零(26 做法) */
    MECH_AXIS_ZERO_MANUAL,       /* 仅人工零点 */
} mech_axis_zero_mode_e;

/**
 * @brief  轴配置(常量表)
 */
typedef struct {
    dart_motor_id_e motor_id;     /* 电机编号 */
    int8_t   dir_sign;            /* 逻辑方向 +1/-1 */
    float    out_max;             /* 输出限幅 raw */
    float    speed_max;           /* 默认转速上限 rpm */
    int32_t  pos_min;             /* 软限位下限 count */
    int32_t  pos_max;             /* 软限位上限 count */
    mech_axis_zero_mode_e zero_mode; /* 找零方式 */
    float    zero_speed;          /* 找零转速 rpm，含方向 */
    float    zero_out_max;        /* 找零输出限幅 raw */
    float    zero_spd_kp;         /* 找零速度环 Kp */
    float    zero_spd_ki;         /* 找零速度环 Ki */
    float    zero_spd_imax;       /* 找零速度环积分限幅 */
    uint32_t zero_timeout_ms;     /* 找零超时 ms */
    bool     home_after_zero;     /* 找零后自动去 home */
    int32_t  home_pos;            /* 找零后回转位初值 count */
    float    arrive_err;          /* 到位误差 count */
    float    arrive_speed;        /* 到位转速 rpm */
    uint32_t arrive_hold_ms;      /* 到位持续 ms */
    uint32_t move_timeout_ms;     /* 运动超时下限 ms */
    int32_t  hold_stall_err;      /* 保持卡死判定误差 count */
    int32_t  jog_lead_max;        /* 点动目标最大超前 count */
    bool     stall_check_in_move; /* 运动中检测堵转 */
    float    stall_out_min;       /* 堵转 |输出|下限 raw */
    float    stall_speed_max;     /* 堵转 |转速|上限 rpm */
    uint32_t stall_hold_ms;       /* 堵转持续 ms */
    bool     hold_on_fault;       /* 故障时保持位置 */
    float    pos_kp;              /* 位置环 Kp */
    float    spd_kp;              /* 速度环 Kp */
    float    spd_ki;              /* 速度环 Ki */
    float    spd_imax;            /* 速度环积分限幅 */
} mech_axis_cfg_t;

/**
 * @brief  轴运行对象
 */
typedef struct {
    const mech_axis_cfg_t *cfg;   /* 配置表 */
    pid_ctrl_t pos_pid;           /* 位置环 */
    pid_ctrl_t spd_pid;           /* 速度环 */
    pid_ctrl_t zero_pid;          /* 找零专用速度环 */
    mech_axis_status_e status;    /* 当前状态 */
    mech_axis_err_e err;          /* 故障原因 */
    bool     zeroed;              /* 零点有效 */
    int32_t  zero_ref;            /* 零点处原始累计 count */
    int32_t  pos;                 /* 逻辑位置 count */
    float    speed;               /* 逻辑转速 rpm */
    int32_t  target;              /* 逻辑目标 count */
    int32_t  home_pos;            /* 找零后回转位 count，可在线改 */
    int32_t  pos_min;             /* 软限位下限 count，可在线改 */
    int32_t  pos_max;             /* 软限位上限 count，可在线改 */
    uint32_t move_timeout;        /* 本次运动超时 ms(按距离算) */
    float    tune_speed;          /* 调试速度模式目标 rpm */
    float    speed_limit;         /* 本次转速上限 rpm */
    float    output;              /* 逻辑输出 raw */
    uint32_t phase_tick;          /* 找零/运动起始 ms */
    uint32_t settle_tick;         /* 到位条件起始 ms */
    bool     settle_active;       /* 到位条件成立中 */
    uint32_t stall_tick;          /* 堵转条件起始 ms */
    bool     stall_active;        /* 堵转条件成立中 */
} mech_axis_t;

/* Exported functions --------------------------------------------------------*/
void Mech_Axis_Init(mech_axis_t *axis, const mech_axis_cfg_t *cfg);
void Mech_Axis_Step(mech_axis_t *axis, uint32_t now);
void Mech_Axis_Enable(mech_axis_t *axis);
void Mech_Axis_Disable(mech_axis_t *axis);
bool Mech_Axis_Request_Zero(mech_axis_t *axis, uint32_t now);
bool Mech_Axis_Request_Move(mech_axis_t *axis, int32_t target, float speed_limit, uint32_t now);
void Mech_Axis_Hold(mech_axis_t *axis);
bool Mech_Axis_Request_Home(mech_axis_t *axis, uint32_t now);
bool Mech_Axis_Tune_Speed(mech_axis_t *axis, float speed_rpm);
void Mech_Axis_Jog(mech_axis_t *axis, int32_t delta);
void Mech_Axis_Set_Zero_Here(mech_axis_t *axis);
void Mech_Axis_Force_Fault(mech_axis_t *axis, mech_axis_err_e err);
void Mech_Axis_Clear_Fault(mech_axis_t *axis);
bool Mech_Axis_Is_At(const mech_axis_t *axis, int32_t pos);
bool Mech_Axis_Is_Ready(const mech_axis_t *axis);
bool Mech_Axis_Is_Ready_Band(const mech_axis_t *axis, float err_band, float speed_band);
uint32_t Mech_Axis_Move_Timeout(const mech_axis_t *axis, int32_t dist, float speed_rpm);

#endif /* __MECH_AXIS_H */
