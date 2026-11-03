/**
 ******************************************************************************
 * @file    fire_gate.c
 * @author  RobotPilots 镖架电控
 * @brief   统一释放许可实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  逐项检查全部条件并累计原因，便于一次性定位"为何不能发"。
 *  Yaw/变力/上膛位置用保持带(比到位判定宽)，避免轻微抖动误拒；
 *  模式、遥控、电机、锁止、裁判、确认为硬条件，不放宽。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fire_gate.h"
#include "rc_cmd.h"
#include "fault.h"
#include "dart_motor.h"
#include "loader.h"
#include "tension.h"
#include "yaw.h"
#include "trigger.h"
#include "feeder.h"
#include "judge_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  复核全部释放条件
 * @param  mode_ok    调用方确认当前模式允许发射
 * @param  confirm_ok 双控件确认(两步模式下由调用方传 true)
 * @retval 许可结果与拒绝原因
 */
fire_gate_result_t Fire_Gate_Check(bool mode_ok, bool confirm_ok)
{
    fire_gate_result_t result;
    uint32_t deny = 0U;

    if (!mode_ok) {
        deny |= FIRE_DENY_MODE;
    }
    if (!Rc_Cmd_Get()->online) {
        deny |= FIRE_DENY_RC;
    }
    if (Fault_Get_Mask() != 0U) {
        deny |= FIRE_DENY_FAULT;
    }
    if (!Dart_Motor_All_Online()) {
        deny |= FIRE_DENY_MOTOR;
    }
    if (!Loader_Axis()->zeroed || !Tension_Axis()->zeroed || !Yaw_Axis()->zeroed) {
        deny |= FIRE_DENY_NOT_ZEROED;
    }
    if (Trigger_Get_State() != TRIGGER_LOCKED) {
        deny |= FIRE_DENY_NOT_LOCKED;
    }
    if (!Loader_Is_Fire_Clear()) {
        deny |= FIRE_DENY_LOADER_POS;
    }
    if (!Yaw_Is_Fire_Ready()) {
        deny |= FIRE_DENY_YAW;
    }
    if (!Tension_Is_Fire_Ready()) {
        deny |= FIRE_DENY_TENSION;
    }
    if (Feeder_Get_Status() == FEEDER_RUNNING) {
        deny |= FIRE_DENY_FEEDER;
    }
    if (!Judge_Fire_Permit()) {
        deny |= FIRE_DENY_JUDGE;
    }
    if (!confirm_ok) {
        deny |= FIRE_DENY_CONFIRM;
    }

    result.deny_mask = deny;
    result.allow = (deny == 0U);
    return result;
}
