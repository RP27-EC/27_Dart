/**
 ******************************************************************************
 * @file    fire_gate.h
 * @author  RobotPilots 镖架电控
 * @brief   统一释放许可：每次释放前集中复核全部互锁并给出拒绝原因
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  只在单发 FIRE 步骤入口调用；不缓存结果，每次释放都重新检查。
 *  "允许释放" 不等于 "有效命中" 或 "战术上应当发射"。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __FIRE_GATE_H
#define __FIRE_GATE_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  拒绝原因位
 */
typedef enum {
    FIRE_DENY_MODE        = (1UL << 0),   /* 非半自动/自动模式 */
    FIRE_DENY_RC          = (1UL << 1),   /* 遥控离线 */
    FIRE_DENY_FAULT       = (1UL << 2),   /* 存在故障 */
    FIRE_DENY_MOTOR       = (1UL << 3),   /* 电机离线 */
    FIRE_DENY_NOT_ZEROED  = (1UL << 4),   /* 机构未找零 */
    FIRE_DENY_NOT_LOCKED  = (1UL << 5),   /* 撒放器未锁止 */
    FIRE_DENY_LOADER_POS  = (1UL << 6),   /* 上膛未回到 UP */
    FIRE_DENY_YAW         = (1UL << 7),   /* Yaw 未就绪 */
    FIRE_DENY_TENSION     = (1UL << 8),   /* 变力/力未就绪 */
    FIRE_DENY_FEEDER      = (1UL << 9),   /* 供弹未结束 */
    FIRE_DENY_JUDGE       = (1UL << 10),  /* 裁判侧不允许 */
    FIRE_DENY_CONFIRM     = (1UL << 11),  /* 双控件确认缺失 */
} fire_deny_e;

/**
 * @brief  许可结果
 */
typedef struct {
    bool     allow;       /* true 允许释放 */
    uint32_t deny_mask;   /* fire_deny_e 位或 */
} fire_gate_result_t;

/* Exported functions --------------------------------------------------------*/
fire_gate_result_t Fire_Gate_Check(bool mode_ok, bool confirm_ok);

#endif /* __FIRE_GATE_H */
