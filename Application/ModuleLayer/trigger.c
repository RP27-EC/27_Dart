/**
 ******************************************************************************
 * @file    trigger.c
 * @author  RobotPilots 镖架电控
 * @brief   撒放器实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  上电输出锁止脉宽(Servo_Init)，状态从 LOCKING 开始计时。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "trigger.h"
#include "servo.h"

/* Private variables ---------------------------------------------------------*/
static trigger_state_e s_trigger_state;
static uint32_t s_trigger_tick;   /* 最近一次下发时刻 ms */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  撒放器初始化，默认锁止
 * @param  now 当前时刻 ms
 */
void Trigger_Init(uint32_t now)
{
    Servo_Set_Pulse(SERVO_TRIGGER, TRIGGER_PULSE_LOCK, TRIGGER_SETTLE_MS, now);
    s_trigger_state = TRIGGER_LOCKING;
    s_trigger_tick = now;
}

/**
 * @brief  周期更新推定状态
 * @param  now 当前时刻 ms
 */
void Trigger_Step(uint32_t now)
{
    if (!Dart_Time_Elapsed(now, s_trigger_tick, TRIGGER_SETTLE_MS)) {
        return;
    }
    if (s_trigger_state == TRIGGER_LOCKING) {
        s_trigger_state = TRIGGER_LOCKED;
    } else if (s_trigger_state == TRIGGER_RELEASING) {
        s_trigger_state = TRIGGER_RELEASED;
    } else {
        /* 已稳定，无动作 */
    }
}

/**
 * @brief  请求锁止；已锁止/锁止中时不重新计时
 */
void Trigger_Request_Lock(uint32_t now)
{
    if ((s_trigger_state == TRIGGER_LOCKED) || (s_trigger_state == TRIGGER_LOCKING)) {
        return;
    }
    Servo_Set_Pulse(SERVO_TRIGGER, TRIGGER_PULSE_LOCK, TRIGGER_SETTLE_MS, now);
    s_trigger_state = TRIGGER_LOCKING;
    s_trigger_tick = now;
}

/**
 * @brief  请求释放；已释放/释放中时不重新计时
 * @note   见头文件 WARNING
 */
void Trigger_Request_Release(uint32_t now)
{
    if ((s_trigger_state == TRIGGER_RELEASED) || (s_trigger_state == TRIGGER_RELEASING)) {
        return;
    }
    Servo_Set_Pulse(SERVO_TRIGGER, TRIGGER_PULSE_RELEASE, TRIGGER_SETTLE_MS, now);
    s_trigger_state = TRIGGER_RELEASING;
    s_trigger_tick = now;
}

/**
 * @brief  读取推定状态
 */
trigger_state_e Trigger_Get_State(void)
{
    return s_trigger_state;
}
