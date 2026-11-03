/**
 ******************************************************************************
 * @file    device.h
 * @author  RobotPilots 镖架电控
 * @brief   设备层统一初始化入口
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *
 * Copyright 2026 RobotPilots
 *
 * 修改日志：
 *  V0.2 2026-11-02 改为镖架设备表
 ******************************************************************************
 */
#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "rc_sensor.h"
#include "dart_motor.h"
#include "servo.h"
#include "loadcell.h"
#include "tf_log.h"
#include "buzzer.h"

/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void);

#endif /* __DEVICE_H */
