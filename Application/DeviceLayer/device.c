/**
 ******************************************************************************
 * @file    device.c
 * @author  RobotPilots 镖架电控
 * @brief   设备层统一初始化入口
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  调用顺序：DRIVER_Init(启动 PWM/串口/CAN) 之后调用本函数。
 *  电机对象在 CAN 过滤器启动前已由 main 单独初始化，见 main.c。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "device.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化遥控、舵机与预留设备
 * @param  无
 * @retval 无
 */
void DEVICE_Init(void)
{
    rc_sensor.init(&rc_sensor);
    Servo_Init();
    Loadcell_Init();
    Log_Init();
    Buzzer_Init();
}
