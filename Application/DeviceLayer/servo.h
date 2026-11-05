/**
 ******************************************************************************
 * @file    servo.h
 * @author  RobotPilots 镖架电控
 * @brief   位置型舵机设备对象(无反馈，按时间推定到位)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  舵机没有位置反馈，"到位"只是发出目标后等待 settle_ms 的时间推定，
 *  不代表机构真实到位。待机/中止时保持最后一次脉宽，不断开 PWM。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __SERVO_H
#define __SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"
#include "drv_pwm.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  镖架舵机编号
 */
typedef enum {
    SERVO_TRIGGER = 0,   /* 撒放器 CH1 */
    SERVO_LIFT_A,        /* 剪叉升降 A CH2 */
    SERVO_LIFT_B,        /* 剪叉升降 B CH3 */
    SERVO_STOPPER,       /* 储弹限位 CH4 */
    SERVO_NUM,           /* 舵机总数 */
} servo_id_e;

/**
 * @brief  舵机对象
 */
typedef struct {
    pwm_servo_ch_e ch;          /* PWM 通道 */
    uint16_t pulse_min;         /* 允许最小脉宽 µs */
    uint16_t pulse_max;         /* 允许最大脉宽 µs */
    uint16_t pulse_init;        /* 上电初始脉宽 µs */
    uint16_t pulse_now;         /* 当前输出脉宽 µs */
    uint32_t settle_ms;         /* 本次动作等待 ms */
    uint32_t set_tick;          /* 本次下发时刻 ms */
} servo_t;

/* Exported macro ------------------------------------------------------------*/
#define SERVO_DEBUG_ENABLE      (DART_TEST_MODE)   /* 调试接管随测试模式 */
#define SERVO_DEBUG_RANGE_DEG   (270.0f)           /* 270° 舵机满行程 */

/**
 * @brief  调试器舵机接管块(Watch 窗口读写)
 * @note   enable=1 时按 angle_deg 输出，覆盖状态机脉宽；写 0 恢复原脉宽。
 *         换算：pulse = 500 + angle / 270 × 2000 µs，按 270° 舵机线性推定。
 */
typedef struct {
    uint8_t  enable;                /* 写 1 接管，写 0 退出 */
    float    angle_deg[SERVO_NUM];  /* 目标角 0~270° */
    uint16_t pulse_us[SERVO_NUM];   /* 只读：实际输出 µs */
} servo_debug_t;

/* Exported variables --------------------------------------------------------*/
#if SERVO_DEBUG_ENABLE
extern volatile servo_debug_t g_servo_debug;
#endif

/* Exported functions --------------------------------------------------------*/
void Servo_Init(void);
void Servo_Set_Pulse(servo_id_e id, uint16_t pulse_us, uint32_t settle_ms, uint32_t now);
bool Servo_Is_Settled(servo_id_e id, uint32_t now);
uint16_t Servo_Get_Pulse(servo_id_e id);
void Servo_Output_All(void);

#endif /* __SERVO_H */
