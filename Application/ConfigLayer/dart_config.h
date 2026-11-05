/**
 ******************************************************************************
 * @file    dart_config.h
 * @author  RobotPilots 镖架电控
 * @brief   27 镖架编译开关、硬件映射与全部待标定参数
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  1. 本文件只放宏和参数，不放可执行逻辑；被 Module/Control 层引用。
 *  2. 标注 "待标定" 的数值均沿用 26 源码或为占位值，未经新机构实测，
 *     上机前必须由人工逐项确认方向、行程与脉宽。
 *  3. 位置单位统一为电机转子编码器累计计数(count，8192/转)，
 *     速度单位为电调反馈转子转速(rpm)，输出单位为电调原始电流指令(raw)。
 *
 * Copyright 2026 RobotPilots
 *
 * 修改日志：
 *  V0.1 2026-11-02 首版，从 26 工程迁移参数初值
 ******************************************************************************
 */
#ifndef __DART_CONFIG_H
#define __DART_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* 版本 ----------------------------------------------------------------------*/
#define DART_FW_VERSION              (0x00000100U)  /* 固件版本 0.1.0 */
#define DART_PARAM_VERSION           (1U)           /* 参数表版本 */

/* 编译开关 ------------------------------------------------------------------*/
#define DART_FOOLPROOF_TWO_STEP      (0U)   /* 两步：储能/释放分开 */
#define DART_FOOLPROOF_DUAL_INPUT    (1U)   /* 双控件：拨轮+右摇杆 */
#define DART_FOOLPROOF_MODE          DART_FOOLPROOF_TWO_STEP

#define DART_FEED_BEFORE_LOAD        (0)    /* 1:下拉前落镖 0:锁止后落镖 */

#define DART_FEEDER_CALIBRATED       (0)    /* 1:供弹步骤表已人工标定 */
#define DART_TEST_MODE               (1)    /* 1:测试，裁判许可旁路 */
#define DART_IWDG_ENABLE             (0)    /* 看门狗，人工验证后再开 */
#define DART_AUTO_ZERO_ENABLE        (1)    /* 1:首次离开安全位自动找零 */
#define DART_BUZZER_ENABLE           (1)    /* 1:板载蜂鸣器提示音 */

/* 暂不接入模块：置 1 前必须先实现对应驱动 */
#define DART_JUDGE_ENABLE            (0)    /* 裁判系统 USART6 */
#define DART_VISION_ENABLE           (0)    /* 小电脑 USB CDC */
#define DART_SCREEN_ENABLE           (0)    /* 串口屏 USART1 */
#define DART_LINK24G_ENABLE          (1)    /* 2.4G 基地同步 USART1，仅测试 */
#define DART_LOADCELL_ENABLE         (0)    /* 双路测力 ADS1232 */
#define DART_TF_LOG_ENABLE           (0)    /* TF 卡日志 SPI2 */

/* 2.4G 测试基地(DL20，USART1 115200) ------------------------------------*/
#define LINK24G_RESEND_MS            (60U)  /* 未回执重发间隔 ms */
#define LINK24G_MAX_TRIES            (15U)  /* 单条命令最多发送次数 */
#define LINK24G_ONLINE_TIMEOUT_MS    (500U) /* 状态超时判离线 ms */
#define LINK24G_TX_TIMEOUT_MS        (5U)   /* 单帧阻塞发送超时 ms */
#define LINK24G_PING_PERIOD_MS       (200U) /* 空闲心跳周期 ms */

/* 通用 ----------------------------------------------------------------------*/
#define DART_SHOT_MAX                (4U)   /* 每局最多装 4 镖 */
#define DART_CTRL_PERIOD_MS          (1U)   /* 控制任务周期 */
#define DART_MONITOR_PERIOD_MS       (1U)   /* 监控任务周期 */
#define DART_COMMUNITY_PERIOD_MS     (10U)  /* 外设轮询周期 */
#define DART_LED_PERIOD_MS           (50U)  /* 指示灯周期 */

/* CAN 映射 ------------------------------------------------------------------*/
/* CAN1：上膛 L/R 与变力，控制帧 0x200 的 0~5 字节 */
#define DART_CAN_ID_LOADER_L         (0x201U)
#define DART_CAN_ID_LOADER_R         (0x202U)
#define DART_CAN_ID_TENSION          (0x203U)
/* CAN2：Yaw 2006，电调 ID 保持 4，控制帧 0x200 的 6~7 字节 */
#define DART_CAN_ID_YAW              (0x204U)
#define DART_CAN_TX_ID_GROUP_LOW     (0x200U)  /* ID1~4 控制帧 */

#define DART_MOTOR_OFFLINE_CNT       (100U) /* 心跳计数上限，1ms/次 */

/* 输出限幅(raw) -------------------------------------------------------------*/
#define DART_3508_OUT_MAX            (16000.0f)  /* 电调上限 16384 */
#define DART_2006_OUT_MAX            (9000.0f)   /* 电调上限 10000 */

/* 舵机 PWM(µs)：TIM1 1MHz 计数，周期 20ms ---------------------------------*/
#define DART_SERVO_PULSE_MIN         (500U)   /* 舵机绝对下限 */
#define DART_SERVO_PULSE_MAX         (2500U)  /* 舵机绝对上限 */

/* 撒放器 TIM1_CH1 PE9，待标定 */
#define TRIGGER_PULSE_LOCK           (500U)   /* 26 值：锁止 */
#define TRIGGER_PULSE_RELEASE        (880U)   /* 26 值：释放 */
#define TRIGGER_SETTLE_MS            (300U)   /* 舵机动作等待 */

/* 供弹舵机，待选型与标定(DS3225/DG995 候选)，尚未采购。
 * NOTE: 动作顺序为限位放行、镖靠重力落到剪叉、限位挡住、剪叉放下、剪叉回接镖位。 */
#define FEEDER_LIFT_A_PULSE_HOME     (1500U)  /* 剪叉A 接镖位(初始) */
#define FEEDER_LIFT_A_PULSE_DOWN     (1500U)  /* 剪叉A 放镖位 */
#define FEEDER_LIFT_B_PULSE_HOME     (1500U)  /* 剪叉B 接镖位(初始) */
#define FEEDER_LIFT_B_PULSE_DOWN     (1500U)  /* 剪叉B 放镖位 */
#define FEEDER_STOP_PULSE_LOCK       (1500U)  /* 限位 挡住 */
#define FEEDER_STOP_PULSE_OPEN       (1500U)  /* 限位 放行 */
#define FEEDER_STEP_HOLD_MS          (600U)   /* 每步保持时间 */

/* 上膛双 3508(CAN1 0x201/0x202) -------------------------------------------*/
#define LOADER_DIR_SIGN              (1)       /* 逻辑正向 ±1，待标定 */
#define LOADER_R_MIRROR_SIGN         (-1.0f)   /* R 输出=-L，沿用26 */
#define LOADER_ZERO_SPEED            (-1000.0f)/* 堵转找零转速 rpm */
#define LOADER_ZERO_OUT_MAX          (4000.0f) /* 找零电流限幅 raw */
#define LOADER_ZERO_SPD_KP           (1.0f)    /* 找零速度环，26 值 */
#define LOADER_ZERO_SPD_KI           (0.1f)
#define LOADER_ZERO_SPD_IMAX         (10000.0f)
#define LOADER_STALL_OUT_MIN         (1000.0f) /* 26：|指令|≥1000 */
#define LOADER_STALL_SPEED_MAX       (10.0f)   /* 26：转速≤10 rpm */
#define LOADER_POS_UP                (20000)   /* 回升位=找零后回转量 count，待确认 */
#define LOADER_POS_MID               (1300000) /* 快速段终点(接触点前) count，待实测 */
#define LOADER_POS_DOWN              (1340000) /* 下拉锁止位 count，待实测 */
#define LOADER_POS_CATCH             (LOADER_POS_DOWN) /* 卸力接住位 count，待实测 */
#define LOADER_POS_MIN               (0)       /* 软限位下限 */
#define LOADER_POS_MAX               (1360000) /* 软限位上限，待标定 */
#define LOADER_SPEED_FAST            (7500.0f) /* 空程转速 rpm */
#define LOADER_SPEED_SLOW            (1200.0f) /* 下拉末段 rpm */
#define LOADER_ARRIVE_ERR            (400.0f)  /* 到位误差 count */
#define LOADER_ARRIVE_SPEED          (20.0f)   /* 到位转速 rpm */
#define LOADER_ARRIVE_HOLD_MS        (20U)     /* 到位持续 */
#define LOADER_ZERO_TIMEOUT_MS       (8000U)   /* 找零超时 */
#define LOADER_MOVE_TIMEOUT_MS       (6000U)   /* 运动超时下限，实际按距离放大 */
#define LOADER_HOLD_STALL_ERR        (20000)   /* 保持卡死判定误差 count，待确认 */
#define LOADER_JOG_LEAD_MAX          (50000)   /* 点动最大超前 count，待确认 */
#define LOADER_JOG_GAIN              (0.5f)    /* 点动增益，满杆≈2400rpm */

/* 变力 3508 丝杆(CAN1 0x203) ----------------------------------------------*/
#define TENSION_DIR_SIGN             (1)       /* 逻辑正向 ±1，待标定 */
#define TENSION_ZERO_SPEED           (-1500.0f)/* 26 实际下发 -1500 rpm */
#define TENSION_ZERO_OUT_MAX         (16000.0f)/* 26 值，实际受 PID 限制 */
#define TENSION_ZERO_SPD_KP          (1.0f)    /* 找零速度环，26 值 */
#define TENSION_ZERO_SPD_KI          (0.1f)
#define TENSION_ZERO_SPD_IMAX        (10000.0f)
#define TENSION_STALL_OUT_MIN        (2000.0f) /* 26：|指令|≥2000 */
#define TENSION_STALL_SPEED_MAX      (10.0f)   /* 26：转速≤10 rpm */
#define TENSION_POS_HOME             (4786247) /* 26 找零后偏移 count */
#define TENSION_ZERO_BACKOFF         (TENSION_POS_HOME) /* 找零后回转量 count，待确认 */
#define TENSION_POS_MIN              (0)       /* 软限位下限 */
#define TENSION_POS_MAX              (9000000) /* 软限位上限，待标定 */
#define TENSION_SPEED_MAX            (6000.0f) /* 最大转速 rpm */
#define TENSION_ARRIVE_ERR           (3000.0f) /* 到位误差 count */
#define TENSION_ARRIVE_SPEED         (100.0f)  /* 到位转速 rpm */
#define TENSION_ARRIVE_HOLD_MS       (50U)     /* 到位持续 */
#define TENSION_ZERO_TIMEOUT_MS      (15000U)  /* 找零超时 */
#define TENSION_MOVE_TIMEOUT_MS      (8000U)   /* 运动超时下限，实际按距离放大 */
#define TENSION_HOLD_STALL_ERR       (20000)   /* 保持卡死判定误差 count，待确认 */
#define TENSION_JOG_LEAD_MAX         (50000)   /* 点动最大超前 count，待确认 */
#define TENSION_JOG_GAIN             (0.5f)    /* 点动 count/ms 每摇杆原始量 */

/* 平 Yaw 2006(CAN2 0x204) -------------------------------------------------*/
#define YAW_DIR_SIGN                 (-1)      /* 使找零物理方向同 26，待标定 */
#define YAW_COUNT_PER_DEG            (819.2f)  /* 8192*36/360，未含外传动 */
#define YAW_ZERO_SPEED               (-2500.0f)/* 逻辑值；物理下发 +2500 同 26 */
#define YAW_ZERO_OUT_MAX             (9000.0f) /* 2006 上限内，实际受 PID 限制 */
#define YAW_ZERO_SPD_KP              (1.0f)    /* 找零速度环，26 值 */
#define YAW_ZERO_SPD_KI              (0.1f)
#define YAW_ZERO_SPD_IMAX            (10000.0f)
#define YAW_STALL_OUT_MIN            (1000.0f) /* 26：|指令|≥1000 */
#define YAW_STALL_SPEED_MAX          (50.0f)   /* 26：转速≤50 rpm */
#define YAW_DEG_MIN                  (0.5f)    /* 软限位 deg，设计 0~18 */
#define YAW_DEG_MAX                  (17.5f)   /* 软限位 deg */
#define YAW_DEG_CENTER               (9.0f)    /* 找零后中位 deg，待确认 */
#define YAW_SPEED_MAX                (8000.0f) /* 最大转速 rpm */
#define YAW_ARRIVE_ERR               (41.0f)   /* ≈0.05deg count */
#define YAW_ARRIVE_SPEED             (50.0f)   /* 到位转速 rpm */
#define YAW_ARRIVE_HOLD_MS           (100U)    /* 到位持续 */
#define YAW_ZERO_TIMEOUT_MS          (10000U)  /* 找零超时 */
#define YAW_MOVE_TIMEOUT_MS          (5000U)   /* 运动超时下限，实际按距离放大 */
#define YAW_HOLD_STALL_ERR           (300)     /* 保持卡死判定误差 count，待确认 */
#define YAW_JOG_LEAD_MAX             (800)     /* 点动超前≈1deg，待确认 */
#define YAW_JOG_GAIN                 (0.00001f)/* 点动增益，满杆≈6.6deg/s */

/* PID(26 源码初值：外环 位置->转速，内环 转速->电流) ---------------------*/
#define LOADER_POS_KP                (0.4f)    /* 位置环 rpm/count */
#define LOADER_SPD_KP                (18.0f)   /* 速度环 raw/rpm */
#define LOADER_SPD_KI                (0.45f)   /* 速度环积分 */
#define LOADER_SPD_IMAX              (20000.0f)/* 积分限幅 */
#define TENSION_POS_KP               (0.3f)
#define TENSION_SPD_KP               (10.0f)
#define TENSION_SPD_KI               (0.1f)
#define TENSION_SPD_IMAX             (10000.0f)
#define YAW_POS_KP                   (0.4f)
#define YAW_SPD_KP                   (13.0f)
#define YAW_SPD_KI                   (0.2f)
#define YAW_SPD_IMAX                 (10000.0f)

/* 堵转持续时间(26：连续 100 次，1ms 调用) -------------------------------*/
#define STALL_HOLD_MS                (100U)    /* 持续时间 ms */

/* 释放许可保持带：曾到位后，发射时允许的偏离(待标定) --------------------*/
#define YAW_FIRE_HOLD_ERR            (123.0f)  /* ≈0.15deg，到位带 3 倍 */
#define YAW_FIRE_HOLD_SPEED          (100.0f)  /* rpm */
#define TENSION_FIRE_HOLD_ERR        (9000.0f) /* count，到位带 3 倍 */
#define TENSION_FIRE_HOLD_SPEED      (200.0f)  /* rpm */
#define LOADER_FIRE_HOLD_ERR         (2000.0f) /* count，离滑台仍远 */
#define LOADER_FIRE_HOLD_SPEED       (100.0f)  /* rpm */

/* 蜂鸣器：TIM4 84MHz/(PSC+1)/(ARR+1=500) --------------------------------*/
#define BUZZER_PSC                   (83U)     /* 2kHz */
#define BUZZER_PWM                   (250U)    /* 占空比 50% */

/* 单发时序 ------------------------------------------------------------------*/
#define SHOT_FIRE_SETTLE_MS          (800U)    /* 释放后等待出膛 */
#define SHOT_STEP_TIMEOUT_MS         (10000U)  /* 非运动步骤超时 */
#define MANUAL_RELEASE_CONFIRM_MS    (3000U)   /* 卸力二次确认窗口 */

/* 受控回退与人工卸力(待标定) -----------------------------------------------*/
#define LOADER_SPEED_RETREAT         (800.0f)  /* 回退/卸力送回限速 rpm */
#define LOADER_SPEED_RETREAT_SINGLE  (400.0f)  /* 单侧电机回退限速 rpm，待确认 */
#define LOADER_SINGLE_HOLD_MAX_MS    (5000U)   /* 单侧电机保持上限，超时自动回退 */
#define UNLOAD_REDUCE_TENSION        (0)       /* 1:卸力前先降预紧 */
#define TENSION_POS_UNLOAD           (TENSION_POS_HOME) /* 卸力预紧位 count，占位 */
#define UNLOAD_OPEN_HOLD_MS          (300U)    /* 开锁后电机接住等待 */

/* 遥控 ----------------------------------------------------------------------*/
#define RC_WHEEL_TRIG_THRESH         (500)     /* 拨轮触发阈值 */
#define RC_WHEEL_RELEASE_THRESH      (100)     /* 拨轮回中阈值 */
#define RC_STICK_DEADBAND            (30)      /* 摇杆死区 */
#define RC_STICK_CONFIRM_THRESH      (600)     /* 双控件确认阈值 */
#define RC_STICK_FULL                (660.0f)  /* 摇杆满量程 */
#define RC_WHEEL_UP_SIGN             (-1)      /* 上拨为负，待实物确认 */

/* 每发预设(待人工标定) ----------------------------------------------------*/
/* Yaw 目标角 deg，下标 0~3 对应第 1~4 发 */
#define YAW_SHOT_DEG_TABLE           { 15.8f, 15.8f, 15.8f, 15.8f }
/* 变力目标位置 count，下标 0~3 对应第 1~4 发 */
#define TENSION_SHOT_POS_TABLE       { TENSION_POS_HOME, TENSION_POS_HOME, \
                                       TENSION_POS_HOME, TENSION_POS_HOME }

/* 配置保护 ----------------------------------------------------------------*/
#if (DART_TEST_MODE == 0) && (DART_JUDGE_ENABLE == 0)
#error "正式配置(DART_TEST_MODE=0)必须先实现并启用裁判系统"
#endif
#if (DART_LINK24G_ENABLE != 0) && (DART_SCREEN_ENABLE != 0)
#error "2.4G 与串口屏共用 USART1，不能同时启用"
#endif
#if (DART_LINK24G_ENABLE != 0) && (DART_TEST_MODE == 0)
#error "2.4G 基地同步只用于测试，正式配置必须关闭"
#endif

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  浮点绝对值，替代 rp_math.h 中会与 stdlib 冲突的 abs 宏
 */
static inline float Dart_Abs_F(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief  浮点限幅
 */
static inline float Dart_Clamp_F(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * @brief  判断 now 是否已超过 start 后 span 毫秒(兼容计数回绕)
 */
static inline bool Dart_Time_Elapsed(uint32_t now, uint32_t start, uint32_t span)
{
    return (uint32_t)(now - start) >= span;
}

#endif /* __DART_CONFIG_H */
