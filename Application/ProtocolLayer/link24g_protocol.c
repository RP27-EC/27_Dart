/**
 ******************************************************************************
 * @file    link24g_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   2.4G 队内测试同步接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "link24g_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  2.4G 模块初始化(桩)
 */
void Link24g_Init(void)
{
}

/**
 * @brief  发送一条基地/靶台命令(桩)
 * @param  cmd 命令
 * @param  arg 参数
 * @retval true 已提交；当前恒为 false
 */
bool Link24g_Send_Base_Cmd(link24g_cmd_e cmd, int32_t arg)
{
    (void)cmd;
    (void)arg;
    return false;
}

/**
 * @brief  周期处理：重发、心跳、回执(桩)
 * @param  now 当前时刻 ms
 */
void Link24g_Poll(uint32_t now)
{
    (void)now;
}

/**
 * @brief  对端是否在线
 * @retval 当前恒为 false
 */
bool Link24g_Is_Online(void)
{
    return false;
}
