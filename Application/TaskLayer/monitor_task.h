/**
 ******************************************************************************
 * @file    monitor_task.h
 * @author  RobotPilots 镖架电控
 * @brief   监控任务：心跳离线检测、在线类故障、看门狗
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  任务由 CubeMX freertos.c 创建(弱定义被本文件覆盖)，常驻不删除。
 *  Realtime(CubeMX 配置)，周期 DART_MONITOR_PERIOD_MS(1ms)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __MONITOR_TASK
#define __MONITOR_TASK

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
void StartMonitorTask(void const *argument);

#endif /* __MONITOR_TASK */
