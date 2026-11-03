/**
 ******************************************************************************
 * @file    tension.h
 * @author  RobotPilots 镖架电控
 * @brief   变力机构：3508 丝杆滑台调节拉簧预紧
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  当前为位置控制：每发目标 = TENSION_SHOT_POS_TABLE[发序] + 力修正量。
 *  力修正量由 Tension_Force_Trim 给出，测力未接入时恒为 0。
 *  位置相同不代表拉力/储能/初速相同；力闭环只在锁止后微调预紧。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __TENSION_H
#define __TENSION_H

/* Includes ------------------------------------------------------------------*/
#include "mech_axis.h"

/* Exported functions --------------------------------------------------------*/
void Tension_Init(void);
void Tension_Step(uint32_t now);
mech_axis_t *Tension_Axis(void);
bool Tension_Request_Shot(uint8_t shot_no, uint32_t now);
bool Tension_Is_Ready(void);
bool Tension_Is_Fire_Ready(void);
int32_t Tension_Force_Trim(void);

#endif /* __TENSION_H */
