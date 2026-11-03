/**
 ******************************************************************************
 * @file    led_task.h
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
#ifndef __LED_TASK
#define __LED_TASK

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
void StartLedTask(void const *argument);

#endif /* __LED_TASK */
