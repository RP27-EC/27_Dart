/**
 ******************************************************************************
 * @file    fault.h
 * @author  RobotPilots 镖架电控
 * @brief   故障码登记：出现/恢复/计数/锁存
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  1. 在线类故障(离线)随状态自动置位/清除；
 *     锁存类故障(超时、堵转)只能在操作手拨回 SAFE 时统一确认清除。
 *  2. 首次出现与恢复各记一条日志，持续存在不重复刷日志，次数单独计数。
 *  3. MonitorTask 与 ControlTask 都会写入，内部用临界区保护。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __FAULT_H
#define __FAULT_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  故障码(位号)
 */
typedef enum {
    FAULT_RC_OFFLINE = 0,        /* 遥控离线/数据异常 */
    FAULT_LOADER_L_OFFLINE,      /* 上膛L 离线 */
    FAULT_LOADER_R_OFFLINE,      /* 上膛R 离线 */
    FAULT_TENSION_OFFLINE,       /* 变力电机离线 */
    FAULT_YAW_OFFLINE,           /* Yaw 电机离线 */
    FAULT_LOADER_AXIS,           /* 上膛轴超时/堵转，锁存 */
    FAULT_TENSION_AXIS,          /* 变力轴超时/堵转，锁存 */
    FAULT_YAW_AXIS,              /* Yaw 轴超时/堵转，锁存 */
    FAULT_SHOT_TIMEOUT,          /* 单发非运动步骤超时，锁存 */
    FAULT_SHOT_REQUEST_REJECT,   /* 单发内部请求被拒，锁存 */
    FAULT_NUM,                   /* 故障总数(≤32) */
} fault_code_e;

/* 锁存类故障掩码：只能经 SAFE 确认清除 */
#define FAULT_LATCHED_MASK  ((1UL << FAULT_LOADER_AXIS) | (1UL << FAULT_TENSION_AXIS) | \
                             (1UL << FAULT_YAW_AXIS) | (1UL << FAULT_SHOT_TIMEOUT) |    \
                             (1UL << FAULT_SHOT_REQUEST_REJECT))

/* Exported functions --------------------------------------------------------*/
void Fault_Init(void);
void Fault_Set(fault_code_e code);
void Fault_Clear(fault_code_e code);
void Fault_Update(fault_code_e code, bool active);
void Fault_Clear_Latched(void);
bool Fault_Is_Active(fault_code_e code);
uint32_t Fault_Get_Mask(void);
uint32_t Fault_Get_Count(fault_code_e code);

#endif /* __FAULT_H */
