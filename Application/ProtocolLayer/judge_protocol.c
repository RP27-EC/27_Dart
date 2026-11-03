/**
 ******************************************************************************
 * @file    judge_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   裁判系统接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  Judge_Rx 在 USART6 空闲中断中被调用，实现时只做拷贝/置标志。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "judge_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  裁判模块初始化(桩)
 */
void Judge_Init(void)
{
}

/**
 * @brief  串口接收入口(中断上下文，桩)
 * @param  buf 接收缓冲
 * @param  len 本次实际收到字节数
 */
void Judge_Rx(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
}

/**
 * @brief  周期处理：解析、离线计时(桩)
 * @param  now 当前时刻 ms
 */
void Judge_Poll(uint32_t now)
{
    (void)now;
}

/**
 * @brief  裁判数据是否在线且新鲜
 * @retval 当前恒为 false
 */
bool Judge_Is_Online(void)
{
    return false;
}

/**
 * @brief  裁判侧是否允许本次释放
 * @retval 测试模式 true；正式模式在协议实现前恒 false
 */
bool Judge_Fire_Permit(void)
{
#if DART_TEST_MODE
    /* WARNING: 测试旁路，比赛固件必须关闭 */
    return true;
#else
    /* TODO: 比赛阶段、闸门全开、窗口剩余、数据新鲜度 */
    return false;
#endif
}

/**
 * @brief  读取闸门状态
 * @retval 当前恒为 UNKNOWN
 */
judge_gate_e Judge_Get_Gate_State(void)
{
    return JUDGE_GATE_UNKNOWN;
}

/**
 * @brief  读取目标
 * @retval 当前恒为 UNKNOWN
 */
judge_target_e Judge_Get_Target(void)
{
    return JUDGE_TARGET_UNKNOWN;
}
