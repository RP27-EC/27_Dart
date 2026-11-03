/**
 ******************************************************************************
 * @file    led_task.c
 * @author  RobotPilots 镖架电控
 * @brief   指示灯任务
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  任务由 CubeMX freertos.c 创建(弱定义被本文件覆盖)，常驻不删除。
 *  AboveNormal(CubeMX 配置)，周期 DART_LED_PERIOD_MS(50ms)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led_task.h"
#include "dart_app.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  任务入口
 * @param  argument 未使用
 */
void StartLedTask(void const *argument)
{
    (void)argument;
    for (;;) {
        Dart_App_Led_Step(HAL_GetTick());
        osDelay(DART_LED_PERIOD_MS);
    }
}
