/**
 ******************************************************************************
 * @file    tf_log.c
 * @author  RobotPilots 镖架电控
 * @brief   事件日志接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  未接 TF 前：事件只计入丢弃计数，释放事件累加 RAM 打击次数，掉电丢失。
 *  ControlTask 与 MonitorTask(经 Fault)都会调用，计数用临界区保护；不得在中断中调用。
 *  "释放次数"只表示发出释放命令，不等于实际出膛、裁判出镖或命中。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tf_log.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_total_shot_count;  /* 累计释放命令次数(RAM) */
static volatile uint32_t s_drop_count;        /* 未写出事件数 */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  日志模块初始化(桩)
 */
void Log_Init(void)
{
    s_total_shot_count = 0U;
    s_drop_count = 0U;
    /* TODO: 从 TF/Flash 读取历史累计打击次数 */
}

/**
 * @brief  提交一条事件，非阻塞
 * @param  code 事件码
 * @param  arg  事件参数
 * @retval true 已入队；当前恒为 false(无后端)
 */
bool Log_Event(log_event_e code, int32_t arg)
{
    (void)arg;
    taskENTER_CRITICAL();
    if (code == LOG_EVT_SHOT_RELEASED) {
        s_total_shot_count++;
    }
    s_drop_count++;
    taskEXIT_CRITICAL();
    return false;
}

/**
 * @brief  后台写出入口，CommunityTask 调用(桩)
 * @param  now 当前时刻 ms
 */
void Log_Poll(uint32_t now)
{
    (void)now;
}

/**
 * @brief  读取累计打击次数
 * @retval 次数(当前仅本次上电)
 */
uint32_t Log_Get_Total_Shot_Count(void)
{
    return s_total_shot_count;
}

/**
 * @brief  读取丢弃事件数
 * @retval 丢弃数
 */
uint32_t Log_Get_Drop_Count(void)
{
    return s_drop_count;
}
