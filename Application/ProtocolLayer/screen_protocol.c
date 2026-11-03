/**
 ******************************************************************************
 * @file    screen_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   串口屏接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  Screen_Rx 在 USART1 空闲中断中被调用，实现时只做拷贝/置标志。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "screen_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  串口屏初始化(桩)
 */
void Screen_Init(void)
{
}

/**
 * @brief  串口接收入口(中断上下文，桩)
 * @param  buf 接收缓冲
 * @param  len 本次实际收到字节数
 */
void Screen_Rx(const uint8_t *buf, uint16_t len)
{
    (void)buf;
    (void)len;
}

/**
 * @brief  周期处理：解析请求、刷新显示(桩)
 * @param  now 当前时刻 ms
 */
void Screen_Poll(uint32_t now)
{
    (void)now;
}
