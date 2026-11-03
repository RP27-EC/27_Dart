/**
 ******************************************************************************
 * @file    dart_app.c
 * @author  RobotPilots 镖架电控
 * @brief   镖架应用入口实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  控制周期内顺序固定：遥控快照 -> 状态机 -> 各机构计算 -> 输出。
 *  output_enabled=false 时在发送前统一清零电机输出，作为最后一道保护。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_app.h"
#include "dart_fsm.h"
#include "rc_cmd.h"
#include "fault.h"
#include "rc_sensor.h"
#include "dart_motor.h"
#include "servo.h"
#include "loadcell.h"
#include "tf_log.h"
#include "loader.h"
#include "tension.h"
#include "yaw.h"
#include "trigger.h"
#include "feeder.h"
#include "judge_protocol.h"
#include "vision_protocol.h"
#include "screen_protocol.h"
#include "link24g_protocol.h"
#include "led.h"
#include "buzzer.h"

#if DART_IWDG_ENABLE
#include "iwdg.h"
#endif

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  模块初始化(调度器启动前)
 */
void Dart_App_Init(void)
{
    uint32_t now = HAL_GetTick();

    Fault_Init();
    Rc_Cmd_Init();
    Loader_Init();
    Tension_Init();
    Yaw_Init();
    Trigger_Init(now);
    Feeder_Init();
    Judge_Init();
    Vision_Init();
    Screen_Init();
    Link24g_Init();
    Dart_Fsm_Init();
    /* NOTE: 上电事件在控制任务首周期记录；调度器启动前进入临界区会屏蔽中断至启动 */
}

/**
 * @brief  控制周期，ControlTask 每 1ms 调用
 */
void Dart_App_Control_Step(uint32_t now)
{
    static bool s_boot_logged = false;   /* 上电事件已记录 */

    if (!s_boot_logged) {
        s_boot_logged = true;
        (void)Log_Event(LOG_EVT_BOOT, (int32_t)DART_FW_VERSION);
    }
    Rc_Cmd_Update(now);
    Dart_Fsm_Step(now);

    Loader_Step(now);
    Tension_Step(now);
    Yaw_Step(now);
    Trigger_Step(now);
    Feeder_Step(now);

    if (!Dart_Fsm_Get()->output_enabled) {
        Dart_Motor_Zero_Output();
    }
    Dart_Motor_Send();
    Servo_Output_All();
}

/**
 * @brief  监控周期，MonitorTask 每 1ms 调用：心跳与在线类故障
 */
void Dart_App_Monitor_Step(uint32_t now)
{
    (void)now;
    rc_sensor.heart_beat(&rc_sensor);
    Dart_Motor_Heart_Beat();

    Fault_Update(FAULT_RC_OFFLINE,
                 (rc_sensor.work_state != DEV_ONLINE) || (rc_sensor.errno != NONE_ERR));
    Fault_Update(FAULT_LOADER_L_OFFLINE, !Dart_Motor_Is_Online(DART_MOTOR_LOADER_L));
    Fault_Update(FAULT_LOADER_R_OFFLINE, !Dart_Motor_Is_Online(DART_MOTOR_LOADER_R));
    Fault_Update(FAULT_TENSION_OFFLINE, !Dart_Motor_Is_Online(DART_MOTOR_TENSION));
    Fault_Update(FAULT_YAW_OFFLINE, !Dart_Motor_Is_Online(DART_MOTOR_YAW));

#if DART_IWDG_ENABLE
    (void)HAL_IWDG_Refresh(&hiwdg);
#endif
}

/**
 * @brief  外设轮询周期，CommunityTask 每 10ms 调用(当前均为桩)
 */
void Dart_App_Community_Step(uint32_t now)
{
    Judge_Poll(now);
    Vision_Poll(now);
    Screen_Poll(now);
    Link24g_Poll(now);
    Loadcell_Poll(now);
    Log_Poll(now);
}

/**
 * @brief  指示灯，LedTask 每 50ms 调用
 *         蓝：遥控在线；绿：电机全在线(找零中闪)；
 *         红：常亮=有故障，快闪=ARMED/已储能，慢闪=ABORTED
 */
void Dart_App_Led_Step(uint32_t now)
{
    static bool s_was_zeroing = false;   /* 上一周期是否有轴在找零 */
    static uint32_t s_last_fault = 0U;   /* 上一周期故障位图 */
    const dart_fsm_t *fsm = Dart_Fsm_Get();
    uint32_t fault_mask = Fault_Get_Mask();
    bool blink_fast = ((now / 125U) % 2U) == 0U;
    bool blink_slow = ((now / 500U) % 2U) == 0U;
    bool zeroing = (Loader_Axis()->status == MECH_AXIS_ZEROING) ||
                   (Tension_Axis()->status == MECH_AXIS_ZEROING) ||
                   (Yaw_Axis()->status == MECH_AXIS_ZEROING);
    bool red_on;

    /* 提示音：找零全部完成、出现新故障 */
    if (s_was_zeroing && !zeroing &&
        Loader_Axis()->zeroed && Tension_Axis()->zeroed && Yaw_Axis()->zeroed) {
        Buzzer_Play(BUZZER_TONE_ZERO_DONE);
    }
    if ((fault_mask & ~s_last_fault) != 0U) {
        Buzzer_Play(BUZZER_TONE_FAULT);
    }
    s_was_zeroing = zeroing;
    s_last_fault = fault_mask;
    Buzzer_Step(now);

    if (Rc_Cmd_Get()->online) {
        LED_BLUE_ON;
    } else {
        LED_BLUE_OFF;
    }

    if (Dart_Motor_All_Online() && (!zeroing || blink_fast)) {
        LED_GREEN_ON;
    } else {
        LED_GREEN_OFF;
    }

    if (fault_mask != 0U) {
        red_on = true;
    } else if ((fsm->step == SHOT_ARMED) || (fsm->energy == ENERGY_STORED)) {
        red_on = blink_fast;
    } else if (fsm->step == SHOT_ABORTED) {
        red_on = blink_slow;
    } else {
        red_on = false;
    }
    if (red_on) {
        LED_RED_ON;
    } else {
        LED_RED_OFF;
    }
}
