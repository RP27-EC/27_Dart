/**
 ******************************************************************************
 * @file    feeder.h
 * @author  RobotPilots 镖架电控
 * @brief   供弹机构：三舵机(剪叉升降 A/B + 储弹限位)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  每发供弹为一张步骤表，每步同时给出三只舵机脉宽并保持 hold_ms。
 *  第 1 发默认已放在导轨上，步骤数为 0。
 *  无有镖/卡镖传感器：DONE 只表示步骤表执行完毕，不代表镖已到位。
 *  DART_FEEDER_CALIBRATED=0 时步骤表为占位值，状态机禁止自动供弹。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __FEEDER_H
#define __FEEDER_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported macro ------------------------------------------------------------*/
#define FEEDER_STEP_MAX     (6U)   /* 单发最多步骤数 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  供弹单步
 */
typedef struct {
    uint16_t lift_a_pulse;    /* 剪叉A 脉宽 µs */
    uint16_t lift_b_pulse;    /* 剪叉B 脉宽 µs */
    uint16_t stopper_pulse;   /* 限位 脉宽 µs */
    uint32_t hold_ms;         /* 本步保持 ms */
} feeder_step_t;

/**
 * @brief  单发供弹序列
 */
typedef struct {
    uint8_t step_num;                      /* 步骤数 0~FEEDER_STEP_MAX */
    feeder_step_t step[FEEDER_STEP_MAX];   /* 步骤表 */
} feeder_seq_t;

/**
 * @brief  供弹状态
 */
typedef enum {
    FEEDER_IDLE = 0,    /* 空闲 */
    FEEDER_RUNNING,     /* 执行中 */
    FEEDER_DONE,        /* 序列完成，等待确认 */
    FEEDER_ABORTED,     /* 被中止，舵机保持当前位 */
} feeder_status_e;

/* Exported functions --------------------------------------------------------*/
void Feeder_Init(void);
void Feeder_Step(uint32_t now);
bool Feeder_Request_Shot(uint8_t shot_no, uint32_t now);
bool Feeder_Shot_Needs_Feed(uint8_t shot_no);
bool Feeder_Request_Home(uint32_t now);
void Feeder_Abort(void);
void Feeder_Ack(void);
feeder_status_e Feeder_Get_Status(void);

#endif /* __FEEDER_H */
