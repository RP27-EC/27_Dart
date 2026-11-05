/**
 ******************************************************************************
 * @file    trigger.h
 * @author  RobotPilots 镖架电控
 * @brief   撒放器(扳机舵机，TIM1_CH1 PE9)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  WARNING: 滑台已锁止储能时调用 Trigger_Request_Release 即为发射。
 *  调用者只有三处：单发 FIRE 步骤(已过释放许可)、LOAD_PREP(已确认未储能)、
 *  MANUAL 模式人工切换。新增调用点必须同步更新 docs/状态机说明.md。
 *  无位置反馈，LOCKED/RELEASED 为按时间推定。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __TRIGGER_H
#define __TRIGGER_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  撒放器状态
 */
typedef enum {
    TRIGGER_LOCKING = 0,   /* 已发锁止，等待到位 */
    TRIGGER_LOCKED,        /* 推定已锁止 */
    TRIGGER_RELEASING,     /* 已发释放，等待到位 */
    TRIGGER_RELEASED,      /* 推定已释放 */
} trigger_state_e;

/* Exported functions --------------------------------------------------------*/
void Trigger_Init(uint32_t now);
void Trigger_Step(uint32_t now);
void Trigger_Request_Lock(uint32_t now);
void Trigger_Request_Release(uint32_t now);
trigger_state_e Trigger_Get_State(void);

#endif /* __TRIGGER_H */
