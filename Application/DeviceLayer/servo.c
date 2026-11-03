/**
 ******************************************************************************
 * @file    servo.c
 * @author  RobotPilots 镖架电控
 * @brief   位置型舵机设备对象
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  脉宽写入集中在 Servo_Output_All，由 ControlTask 每周期调用一次。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"

/* Private variables ---------------------------------------------------------*/
/* NOTE: 上电初始脉宽：撒放器锁止，供弹舵机在初始位 */
static servo_t s_servo[SERVO_NUM] = {
    [SERVO_TRIGGER] = {
        .ch = PWM_SERVO_CH1,
        .pulse_min = DART_SERVO_PULSE_MIN,
        .pulse_max = DART_SERVO_PULSE_MAX,
        .pulse_init = TRIGGER_PULSE_LOCK,
    },
    [SERVO_LIFT_A] = {
        .ch = PWM_SERVO_CH2,
        .pulse_min = DART_SERVO_PULSE_MIN,
        .pulse_max = DART_SERVO_PULSE_MAX,
        .pulse_init = FEEDER_LIFT_A_PULSE_HOME,
    },
    [SERVO_LIFT_B] = {
        .ch = PWM_SERVO_CH3,
        .pulse_min = DART_SERVO_PULSE_MIN,
        .pulse_max = DART_SERVO_PULSE_MAX,
        .pulse_init = FEEDER_LIFT_B_PULSE_HOME,
    },
    [SERVO_STOPPER] = {
        .ch = PWM_SERVO_CH4,
        .pulse_min = DART_SERVO_PULSE_MIN,
        .pulse_max = DART_SERVO_PULSE_MAX,
        .pulse_init = FEEDER_STOP_PULSE_LOCK,
    },
};

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  舵机对象初始化并立即输出初始脉宽
 * @param  无
 * @retval 无
 * @note   须在 PWM_Servo_Init 之后调用
 */
void Servo_Init(void)
{
    uint8_t servo_index;

    for (servo_index = 0U; servo_index < (uint8_t)SERVO_NUM; servo_index++) {
        s_servo[servo_index].pulse_now = s_servo[servo_index].pulse_init;
        s_servo[servo_index].settle_ms = 0U;
        s_servo[servo_index].set_tick  = 0U;
    }
    Servo_Output_All();
}

/**
 * @brief  下发舵机目标脉宽
 * @param  id        舵机编号
 * @param  pulse_us  目标脉宽 µs，超出范围被限幅
 * @param  settle_ms 推定到位所需时间 ms
 * @param  now       当前时刻 ms
 * @retval 无
 * @note   脉宽不变时不刷新计时，避免重复调用使到位时间被无限推迟
 */
void Servo_Set_Pulse(servo_id_e id, uint16_t pulse_us, uint32_t settle_ms, uint32_t now)
{
    servo_t *servo;

    if (id >= SERVO_NUM) {
        return;
    }
    servo = &s_servo[id];
    if (pulse_us < servo->pulse_min) {
        pulse_us = servo->pulse_min;
    }
    if (pulse_us > servo->pulse_max) {
        pulse_us = servo->pulse_max;
    }
    if (pulse_us == servo->pulse_now) {
        return;
    }
    servo->pulse_now = pulse_us;
    servo->settle_ms = settle_ms;
    servo->set_tick  = now;
}

/**
 * @brief  按时间推定舵机是否到位
 * @param  id  舵机编号
 * @param  now 当前时刻 ms
 * @retval true 已超过等待时间
 */
bool Servo_Is_Settled(servo_id_e id, uint32_t now)
{
    if (id >= SERVO_NUM) {
        return false;
    }
    return Dart_Time_Elapsed(now, s_servo[id].set_tick, s_servo[id].settle_ms);
}

/**
 * @brief  读取舵机当前输出脉宽
 * @param  id 舵机编号
 * @retval 脉宽 µs
 */
uint16_t Servo_Get_Pulse(servo_id_e id)
{
    if (id >= SERVO_NUM) {
        return 0U;
    }
    return s_servo[id].pulse_now;
}

/**
 * @brief  把全部舵机当前脉宽写入 PWM
 * @param  无
 * @retval 无
 */
void Servo_Output_All(void)
{
    uint8_t servo_index;

    for (servo_index = 0U; servo_index < (uint8_t)SERVO_NUM; servo_index++) {
        PWM_Servo_Set_Pulse(s_servo[servo_index].ch, s_servo[servo_index].pulse_now);
    }
}
