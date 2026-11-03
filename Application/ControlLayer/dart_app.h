/**
 ******************************************************************************
 * @file    dart_app.h
 * @author  RobotPilots 镖架电控
 * @brief   镖架应用入口：初始化顺序与各任务的周期函数
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  任务层只调用本文件接口，保持任务函数为薄壳。
 *  Dart_App_Init 必须在调度器启动前、DRIVER_Init/DEVICE_Init 之后调用。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_APP_H
#define __DART_APP_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported functions --------------------------------------------------------*/
void Dart_App_Init(void);
void Dart_App_Control_Step(uint32_t now);
void Dart_App_Monitor_Step(uint32_t now);
void Dart_App_Community_Step(uint32_t now);
void Dart_App_Led_Step(uint32_t now);

#endif /* __DART_APP_H */
