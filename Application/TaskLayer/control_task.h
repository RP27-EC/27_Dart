/**
 ******************************************************************************
 * @file    control_task.h
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
#ifndef __CONTROL_TASK
#define __CONTROL_TASK

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
void StartControlTask(void const *argument);

#endif /* __CONTROL_TASK */
