/**
 ******************************************************************************
 * @file    control_task.c
 * @author  RobotPilots 镖架电控
 * @brief   控制任务：遥控快照、状态机、机构闭环、CAN/PWM 输出
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  任务由 CubeMX freertos.c 创建(弱定义被本文件覆盖)，常驻不删除。
 *  Realtime，周期 DART_CTRL_PERIOD_MS(1ms)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "control_task.h"
#include "dart_app.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  任务入口
 * @param  argument 未使用
 */
void StartControlTask(void const *argument)
{
    (void)argument;
    for (;;) {
        Dart_App_Control_Step(HAL_GetTick());
        osDelay(DART_CTRL_PERIOD_MS);
    }
}
