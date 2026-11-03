/**
 ******************************************************************************
 * @file    yaw.h
 * @author  RobotPilots 镖架电控
 * @brief   平 Yaw 瞄准轴：2006(CAN2 0x204)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  对外角度单位 deg，0deg 为找零基准；设计行程 0~18deg，基地约 15.8deg，
 *  均为机械设计值，未标定。deg->count 用 YAW_COUNT_PER_DEG。
 *  视觉修正由 Yaw_Vision_Trim_Deg 提供，未接入时为 0。
 *  目标变化后旧的就绪状态立即失效(由 Request_Move 重置到位计时)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __YAW_H
#define __YAW_H

/* Includes ------------------------------------------------------------------*/
#include "mech_axis.h"

/* Exported functions --------------------------------------------------------*/
void Yaw_Init(void);
void Yaw_Step(uint32_t now);
mech_axis_t *Yaw_Axis(void);
bool Yaw_Request_Deg(float deg, uint32_t now);
bool Yaw_Request_Shot(uint8_t shot_no, uint32_t now);
void Yaw_Jog_Deg(float delta_deg);
float Yaw_Get_Deg(void);
bool Yaw_Is_Ready(void);
bool Yaw_Is_Fire_Ready(void);
float Yaw_Vision_Trim_Deg(void);

#endif /* __YAW_H */
