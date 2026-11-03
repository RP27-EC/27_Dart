/**
 ******************************************************************************
 * @file    buzzer.c
 * @author  RobotPilots 镖架电控
 * @brief   板载蜂鸣器提示音实现
 * @version V0.1
 * @date    2026-11-03
 ******************************************************************************
 * @attention
 *  每个提示音是一串"响/停"时长(ms)，以 0 结束，第 1 段为响。
 *  请求与播放分离：生产者只写 s_request，播放状态只在 Buzzer_Step 中修改。
 *  请求的"比较后写入"与"读取后清空"都在临界区内完成，避免跨任务抢占丢失请求。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"
#include "drv_tim.h"
#include "cmsis_os.h"

/* Private macro -------------------------------------------------------------*/
#define BUZZER_SEG_MAX      (6U)   /* 单个提示音最多段数 */

/* Private variables ---------------------------------------------------------*/
/* 各提示音时序 ms：响,停,响,停...，0 结束 */
static const uint16_t s_tone_seq[BUZZER_TONE_NUM][BUZZER_SEG_MAX] = {
    [BUZZER_TONE_NONE]      = { 0U },
    [BUZZER_TONE_ZERO_DONE] = { 100U, 0U },
    [BUZZER_TONE_ARMED]     = { 100U, 100U, 100U, 0U },
    [BUZZER_TONE_DENY]      = { 400U, 100U, 100U, 0U },
    [BUZZER_TONE_FAULT]     = { 100U, 100U, 100U, 100U, 100U, 0U },
};

static volatile buzzer_tone_e s_request;   /* 待播放请求 */
static buzzer_tone_e s_playing;            /* 正在播放 */
static uint8_t  s_seg;                     /* 当前段号 */
static uint32_t s_seg_tick;                /* 当前段开始 ms */

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  按段号奇偶输出：偶数段响，奇数段停
 */
static void Buzzer_Output(bool on)
{
#if DART_BUZZER_ENABLE
    if (on) {
        buzzer_on(BUZZER_PSC, BUZZER_PWM);
    } else {
        buzzer_off();
    }
#else
    (void)on;
    buzzer_off();
#endif
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  蜂鸣器初始化：静音
 */
void Buzzer_Init(void)
{
    s_request = BUZZER_TONE_NONE;
    s_playing = BUZZER_TONE_NONE;
    s_seg = 0U;
    s_seg_tick = 0U;
    buzzer_off();
}

/**
 * @brief  请求播放提示音；低于当前请求优先级的被忽略
 * @param  tone 提示音
 */
void Buzzer_Play(buzzer_tone_e tone)
{
    if ((tone <= BUZZER_TONE_NONE) || (tone >= BUZZER_TONE_NUM)) {
        return;
    }
    taskENTER_CRITICAL();
    if (tone >= s_request) {
        s_request = tone;
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief  周期推进播放，LedTask 调用
 * @param  now 当前时刻 ms
 */
void Buzzer_Step(uint32_t now)
{
    buzzer_tone_e req;

    /* 领取请求：读取与清空原子完成；低于当前播放的请求保留到播放结束 */
    taskENTER_CRITICAL();
    req = s_request;
    if ((req != BUZZER_TONE_NONE) && (req >= s_playing)) {
        s_request = BUZZER_TONE_NONE;
    } else {
        req = BUZZER_TONE_NONE;
    }
    taskEXIT_CRITICAL();

    /* 新请求优先级不低于当前播放时打断重播 */
    if (req != BUZZER_TONE_NONE) {
        s_playing = req;
        s_seg = 0U;
        s_seg_tick = now;
        Buzzer_Output(true);
        return;
    }
    if (s_playing == BUZZER_TONE_NONE) {
        return;
    }
    if (!Dart_Time_Elapsed(now, s_seg_tick, s_tone_seq[s_playing][s_seg])) {
        return;
    }
    s_seg++;
    if ((s_seg >= BUZZER_SEG_MAX) || (s_tone_seq[s_playing][s_seg] == 0U)) {
        s_playing = BUZZER_TONE_NONE;
        Buzzer_Output(false);
        return;
    }
    s_seg_tick = now;
    Buzzer_Output((s_seg % 2U) == 0U);
}
