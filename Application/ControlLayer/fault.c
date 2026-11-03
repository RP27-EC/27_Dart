/**
 ******************************************************************************
 * @file    fault.c
 * @author  RobotPilots 镖架电控
 * @brief   故障码登记实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  不在中断中调用。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "fault.h"
#include "tf_log.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_fault_mask;        /* 当前故障位图 */
static uint32_t s_fault_count[FAULT_NUM];     /* 各故障出现次数 */

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  故障模块初始化
 */
void Fault_Init(void)
{
    uint8_t code_index;

    s_fault_mask = 0U;
    for (code_index = 0U; code_index < (uint8_t)FAULT_NUM; code_index++) {
        s_fault_count[code_index] = 0U;
    }
}

/**
 * @brief  置位故障；从无到有时计数并记日志
 */
void Fault_Set(fault_code_e code)
{
    uint32_t bit;
    bool rising = false;

    if (code >= FAULT_NUM) {
        return;
    }
    bit = 1UL << (uint32_t)code;
    taskENTER_CRITICAL();
    if ((s_fault_mask & bit) == 0U) {
        s_fault_mask |= bit;
        s_fault_count[code]++;
        rising = true;
    }
    taskEXIT_CRITICAL();
    if (rising) {
        (void)Log_Event(LOG_EVT_FAULT_SET, (int32_t)code);
    }
}

/**
 * @brief  清除故障；从有到无时记日志
 */
void Fault_Clear(fault_code_e code)
{
    uint32_t bit;
    bool falling = false;

    if (code >= FAULT_NUM) {
        return;
    }
    bit = 1UL << (uint32_t)code;
    taskENTER_CRITICAL();
    if ((s_fault_mask & bit) != 0U) {
        s_fault_mask &= ~bit;
        falling = true;
    }
    taskEXIT_CRITICAL();
    if (falling) {
        (void)Log_Event(LOG_EVT_FAULT_CLEAR, (int32_t)code);
    }
}

/**
 * @brief  按条件置位或清除(用于在线类故障)
 */
void Fault_Update(fault_code_e code, bool active)
{
    if (active) {
        Fault_Set(code);
    } else {
        Fault_Clear(code);
    }
}

/**
 * @brief  清除全部锁存类故障，操作手拨回 SAFE 时调用
 */
void Fault_Clear_Latched(void)
{
    uint8_t code_index;

    for (code_index = 0U; code_index < (uint8_t)FAULT_NUM; code_index++) {
        if ((FAULT_LATCHED_MASK & (1UL << code_index)) != 0U) {
            Fault_Clear((fault_code_e)code_index);
        }
    }
}

/**
 * @brief  查询单个故障
 */
bool Fault_Is_Active(fault_code_e code)
{
    if (code >= FAULT_NUM) {
        return false;
    }
    return (s_fault_mask & (1UL << (uint32_t)code)) != 0U;
}

/**
 * @brief  读取故障位图
 */
uint32_t Fault_Get_Mask(void)
{
    return s_fault_mask;
}

/**
 * @brief  读取故障出现次数
 */
uint32_t Fault_Get_Count(fault_code_e code)
{
    if (code >= FAULT_NUM) {
        return 0U;
    }
    return s_fault_count[code];
}
