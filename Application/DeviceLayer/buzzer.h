/**
 ******************************************************************************
 * @file    buzzer.h
 * @author  RobotPilots 镖架电控
 * @brief   板载蜂鸣器提示音(TIM4_CH3 / PD14，非阻塞)
 * @version V0.1
 * @date    2026-11-03
 ******************************************************************************
 * @attention
 *  Buzzer_Play 只登记请求，可在任意任务中调用；高优先级提示音可打断低优先级。
 *  Buzzer_Step 由 LedTask 周期调用，时序分辨率等于其周期(50ms)。
 *  PD14 按 C 板手册为板载蜂鸣器，上机前需对照原理图确认。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __BUZZER_H
#define __BUZZER_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  提示音(数值越大优先级越高)
 */
typedef enum {
    BUZZER_TONE_NONE = 0,    /* 无 */
    BUZZER_TONE_ZERO_DONE,   /* 找零完成：短响 1 声 */
    BUZZER_TONE_ARMED,       /* 进入待发：短响 2 声 */
    BUZZER_TONE_DENY,        /* 操作被拒：一长一短 */
    BUZZER_TONE_FAULT,       /* 新故障：短响 3 声 */
    BUZZER_TONE_NUM,         /* 提示音总数 */
} buzzer_tone_e;

/* Exported functions --------------------------------------------------------*/
void Buzzer_Init(void);
void Buzzer_Play(buzzer_tone_e tone);
void Buzzer_Step(uint32_t now);

#endif /* __BUZZER_H */
