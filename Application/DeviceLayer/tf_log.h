/**
 ******************************************************************************
 * @file    tf_log.h
 * @author  RobotPilots 镖架电控
 * @brief   事件日志与累计打击次数接口(TF 后端为桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  拟定硬件：SPI2 PB13 SCK / PB14 MISO / PB15 MOSI，PB12 CS(未配置)。
 *  业务层只调用 Log_Event 快速提交，不得在控制路径等待写卡。
 *  累计打击次数当前只在 RAM 计数，掉电丢失。
 *  TODO: 有界环形缓冲 + 低优先级写卡任务 + FatFs；次数持久化。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __TF_LOG_H
#define __TF_LOG_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  日志事件码
 */
typedef enum {
    LOG_EVT_BOOT = 0,         /* 上电，arg=固件版本 */
    LOG_EVT_MODE_CHANGE,      /* 模式切换，arg=新模式 */
    LOG_EVT_SHOT_STEP,        /* 单发步骤，arg=步骤 */
    LOG_EVT_SHOT_RELEASED,    /* 已发出释放命令，arg=发序 */
    LOG_EVT_MANUAL_RELEASE,   /* 人工释放，arg=当前发序 */
    LOG_EVT_UNLOAD_DONE,      /* 人工卸力完成，arg=0 */
    LOG_EVT_SAFE_RETREAT,     /* SAFE 受控回退，arg=1开始/0结束 */
    LOG_EVT_FIRE_DENIED,      /* 释放被拒，arg=原因掩码 */
    LOG_EVT_SHOT_ABORTED,     /* 流程中止，arg=步骤 */
    LOG_EVT_FAULT_SET,        /* 故障出现，arg=故障码 */
    LOG_EVT_FAULT_CLEAR,      /* 故障恢复，arg=故障码 */
    LOG_EVT_NUM,              /* 事件总数 */
} log_event_e;

/* Exported functions --------------------------------------------------------*/
void Log_Init(void);
bool Log_Event(log_event_e code, int32_t arg);
void Log_Poll(uint32_t now);
uint32_t Log_Get_Total_Shot_Count(void);
uint32_t Log_Get_Drop_Count(void);

#endif /* __TF_LOG_H */
