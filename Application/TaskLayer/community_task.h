/**
 ******************************************************************************
 * @file    community_task.h
 * @author  RobotPilots 镖架电控
 * @brief   通信任务：裁判/视觉/串口屏/2.4G/测力/日志轮询(当前为桩)
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  任务由 CubeMX freertos.c 创建(弱定义被本文件覆盖)，常驻不删除。
 *  High(CubeMX 配置)，周期 DART_COMMUNITY_PERIOD_MS(10ms)，不得阻塞。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __COMMUNITY_TASK
#define __COMMUNITY_TASK

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"

/* Exported functions --------------------------------------------------------*/
void StartCommunityTask(void const *argument);

#endif /* __COMMUNITY_TASK */
