/**
 ******************************************************************************
 * @file    feeder.c
 * @author  RobotPilots 镖架电控
 * @brief   供弹机构实现(三舵机)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  步骤表全部为占位值(脉宽相同 = 舵机不动)，机构方案与舵机选型确定后
 *  由人工标定填写。默认序列(按用户 2026-11-04 描述)：限位放行 -> 镖靠重力
 *  落到剪叉 -> 限位挡住 -> 两剪叉把镖放下 -> 剪叉回接镖位。
 *  NOTE: 每次只放行一枚取决于放行保持时间和机构，舵机到货后需实测确认。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "feeder.h"
#include "servo.h"

/* Private macro -------------------------------------------------------------*/
/* 占位序列：所有发序共用，待按发序分别标定 */
#define FEEDER_SEQ_PLACEHOLDER                                                         \
    {                                                                                  \
        .step_num = 4U,                                                                \
        .step = {                                                                      \
            { FEEDER_LIFT_A_PULSE_HOME, FEEDER_LIFT_B_PULSE_HOME,                      \
              FEEDER_STOP_PULSE_OPEN, FEEDER_STEP_HOLD_MS },                           \
            { FEEDER_LIFT_A_PULSE_HOME, FEEDER_LIFT_B_PULSE_HOME,                      \
              FEEDER_STOP_PULSE_LOCK, FEEDER_STEP_HOLD_MS },                           \
            { FEEDER_LIFT_A_PULSE_DOWN, FEEDER_LIFT_B_PULSE_DOWN,                   \
              FEEDER_STOP_PULSE_LOCK, FEEDER_STEP_HOLD_MS },                           \
            { FEEDER_LIFT_A_PULSE_HOME, FEEDER_LIFT_B_PULSE_HOME,                      \
              FEEDER_STOP_PULSE_LOCK, FEEDER_STEP_HOLD_MS },                           \
        },                                                                             \
    }

/* Private variables ---------------------------------------------------------*/
/* 下标 0~3 对应第 1~4 发；第 1 发预置在导轨上 */
static const feeder_seq_t s_feed_seq[DART_SHOT_MAX] = {
    { .step_num = 0U },
    FEEDER_SEQ_PLACEHOLDER,
    FEEDER_SEQ_PLACEHOLDER,
    FEEDER_SEQ_PLACEHOLDER,
};

static feeder_status_e s_feeder_status;
static const feeder_seq_t *s_active_seq;   /* 当前序列 */
static uint8_t  s_step_index;              /* 当前步骤号 */
static uint32_t s_step_tick;               /* 本步开始 ms */

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  下发一步的三只舵机目标
 */
static void Feeder_Apply_Step(const feeder_step_t *step, uint32_t now)
{
    Servo_Set_Pulse(SERVO_LIFT_A, step->lift_a_pulse, step->hold_ms, now);
    Servo_Set_Pulse(SERVO_LIFT_B, step->lift_b_pulse, step->hold_ms, now);
    Servo_Set_Pulse(SERVO_STOPPER, step->stopper_pulse, step->hold_ms, now);
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  供弹初始化
 */
void Feeder_Init(void)
{
    s_feeder_status = FEEDER_IDLE;
    s_active_seq = NULL;
    s_step_index = 0U;
    s_step_tick = 0U;
}

/**
 * @brief  周期推进步骤
 * @param  now 当前时刻 ms
 */
void Feeder_Step(uint32_t now)
{
    if ((s_feeder_status != FEEDER_RUNNING) || (s_active_seq == NULL)) {
        return;
    }
    if (!Dart_Time_Elapsed(now, s_step_tick, s_active_seq->step[s_step_index].hold_ms)) {
        return;
    }
    s_step_index++;
    if (s_step_index >= s_active_seq->step_num) {
        s_feeder_status = FEEDER_DONE;
        return;
    }
    s_step_tick = now;
    Feeder_Apply_Step(&s_active_seq->step[s_step_index], now);
}

/**
 * @brief  开始第 shot_no 发的供弹序列
 * @param  shot_no 发序 1~4
 * @retval false 发序非法或正在执行
 */
bool Feeder_Request_Shot(uint8_t shot_no, uint32_t now)
{
    if ((shot_no == 0U) || (shot_no > DART_SHOT_MAX) || (s_feeder_status == FEEDER_RUNNING)) {
        return false;
    }
    s_active_seq = &s_feed_seq[shot_no - 1U];
    s_step_index = 0U;
    s_step_tick = now;
    if (s_active_seq->step_num == 0U) {
        s_feeder_status = FEEDER_DONE;
        return true;
    }
    s_feeder_status = FEEDER_RUNNING;
    Feeder_Apply_Step(&s_active_seq->step[0], now);
    return true;
}

/**
 * @brief  第 shot_no 发是否需要执行供弹动作
 * @param  shot_no 发序 1~4
 * @retval true 步骤数非 0
 */
bool Feeder_Shot_Needs_Feed(uint8_t shot_no)
{
    if ((shot_no == 0U) || (shot_no > DART_SHOT_MAX)) {
        return false;
    }
    return s_feed_seq[shot_no - 1U].step_num != 0U;
}

/**
 * @brief  三只供弹舵机回初始位(剪叉 HOME、限位挡住)
 * @param  now 当前时刻 ms
 * @retval false 序列执行中，拒绝
 */
bool Feeder_Request_Home(uint32_t now)
{
    if (s_feeder_status == FEEDER_RUNNING) {
        return false;
    }
    Servo_Set_Pulse(SERVO_LIFT_A, FEEDER_LIFT_A_PULSE_HOME, FEEDER_STEP_HOLD_MS, now);
    Servo_Set_Pulse(SERVO_LIFT_B, FEEDER_LIFT_B_PULSE_HOME, FEEDER_STEP_HOLD_MS, now);
    Servo_Set_Pulse(SERVO_STOPPER, FEEDER_STOP_PULSE_LOCK, FEEDER_STEP_HOLD_MS, now);
    s_feeder_status = FEEDER_IDLE;
    return true;
}

/**
 * @brief  中止：舵机保持当前脉宽，不回位
 */
void Feeder_Abort(void)
{
    if (s_feeder_status == FEEDER_RUNNING) {
        s_feeder_status = FEEDER_ABORTED;
    }
}

/**
 * @brief  确认 DONE/ABORTED，回到 IDLE
 */
void Feeder_Ack(void)
{
    if ((s_feeder_status == FEEDER_DONE) || (s_feeder_status == FEEDER_ABORTED)) {
        s_feeder_status = FEEDER_IDLE;
    }
}

/**
 * @brief  读取供弹状态
 */
feeder_status_e Feeder_Get_Status(void)
{
    return s_feeder_status;
}
