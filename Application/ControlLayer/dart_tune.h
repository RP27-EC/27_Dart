/**
 ******************************************************************************
 * @file    dart_tune.h
 * @author  RobotPilots 镖架电控
 * @brief   PID 调参接口：调试器写 g_dart_tune 给单轴速度/位置阶跃
 * @version V0.1
 * @date    2026-11-03
 ******************************************************************************
 * @attention
 *  1. 仅 DART_TUNE_ENABLE=1(随 DART_TEST_MODE)时编译生效，比赛固件必须关闭。
 *  2. 只在手动模式(S1 上)且 enable=1 时接管所选轴；其间遥控摇杆/拨轮操作
 *     被忽略，拨轮下拨立即退出。离开手动模式或拨回安全位立即退出并原地保持。
 *  3. 所选轴必须已找零；调上膛时储能状态必须为 RELEASED 且不在卸力中。
 *  4. 速度模式：每周期把 speed_rpm 作为内环目标，外环不参与；到软限位即保持。
 *     位置模式：每次 seq 变化发起一次 pos_target 运动，外环+内环串级。
 *  5. PID 参数直接改各轴对象中的 pos_pid / spd_pid(RAM)，掉电恢复配置值。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_TUNE_H
#define __DART_TUNE_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported macro ------------------------------------------------------------*/
#define DART_TUNE_ENABLE        (DART_TEST_MODE)   /* 调参接口随测试模式 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  调参对象轴
 */
typedef enum {
    TUNE_AXIS_LOADER = 0,   /* 上膛(L 闭环，R 镜像) */
    TUNE_AXIS_TENSION,      /* 变力 */
    TUNE_AXIS_YAW,          /* Yaw */
    TUNE_AXIS_NUM,          /* 轴数 */
} tune_axis_e;

/**
 * @brief  调参模式
 */
typedef enum {
    TUNE_MODE_OFF = 0,      /* 不接管 */
    TUNE_MODE_SPEED,        /* 纯速度环(调内环) */
    TUNE_MODE_POSITION,     /* 位置阶跃(调外环) */
} tune_mode_e;

/**
 * @brief  调参拒绝原因位
 */
typedef enum {
    TUNE_DENY_NOT_MANUAL = (1UL << 0),  /* 不在手动模式 */
    TUNE_DENY_AXIS       = (1UL << 1),  /* 轴编号非法 */
    TUNE_DENY_NOT_ZEROED = (1UL << 2),  /* 轴未找零 */
    TUNE_DENY_ENERGY     = (1UL << 3),  /* 调上膛但可能储能/卸力中 */
    TUNE_DENY_REJECT     = (1UL << 4),  /* 轴拒绝请求(故障/越限) */
} tune_deny_e;

/**
 * @brief  调参控制块(调试器读写)
 */
typedef struct {
    uint8_t  enable;           /* 写 1 生效，写 0 退出 */
    uint8_t  axis;             /* tune_axis_e */
    uint8_t  mode;             /* tune_mode_e */
    float    speed_rpm;        /* 速度模式目标 rpm(逻辑方向) */
    int32_t  pos_target;       /* 位置模式目标 count(相对零点) */
    float    pos_speed_limit;  /* 位置模式限速 rpm，≤0 用默认 */
    uint32_t seq;              /* 位置模式：改变一次发起一次运动 */
    uint8_t  active;           /* 只读：当前已接管 */
    uint32_t deny;             /* 只读：最近拒绝原因 tune_deny_e */
} dart_tune_t;

/* Exported variables --------------------------------------------------------*/
extern volatile dart_tune_t g_dart_tune;

/* Exported functions --------------------------------------------------------*/
bool Dart_Tune_Run(bool manual_mode, bool loader_safe, uint32_t now);
void Dart_Tune_Stop(void);

#endif /* __DART_TUNE_H */
