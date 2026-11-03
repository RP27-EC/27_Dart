/**
 ******************************************************************************
 * @file    dart_motor.h
 * @author  RobotPilots 镖架电控
 * @brief   镖架 RM 电机实例与 CAN 帧发送
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  电机表：
 *    CAN1 0x201 上膛 L 3508 / 0x202 上膛 R 3508 / 0x203 变力 3508
 *    CAN2 0x204 Yaw 2006
 *  类型统一用 _3508_Single / _2006_Single：tx_info->torque 直接等于
 *  电调原始电流指令(raw)，不依赖减速比假设。
 *  反馈在 CAN 接收中断中由 RM_motor.c 更新(encoder_sum/encoder_speed)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_MOTOR_H
#define __DART_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "RM_motor.h"
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  镖架电机编号
 */
typedef enum {
    DART_MOTOR_LOADER_L = 0,  /* 上膛左 3508 */
    DART_MOTOR_LOADER_R,      /* 上膛右 3508 */
    DART_MOTOR_TENSION,       /* 变力丝杆 3508 */
    DART_MOTOR_YAW,           /* 平 Yaw 2006 */
    DART_MOTOR_NUM,           /* 电机总数 */
} dart_motor_id_e;

/* Exported variables --------------------------------------------------------*/
extern Motor_RM_t g_dart_motor[DART_MOTOR_NUM];

/* Exported functions --------------------------------------------------------*/
void Dart_Motor_Init(void);
void Dart_Motor_Heart_Beat(void);
bool Dart_Motor_Is_Online(dart_motor_id_e id);
bool Dart_Motor_All_Online(void);
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id);
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id);
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw);
void Dart_Motor_Zero_Output(void);
void Dart_Motor_Send(void);

#endif /* __DART_MOTOR_H */
