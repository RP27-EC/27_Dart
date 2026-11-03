/**
 ******************************************************************************
 * @file    config_uart.c
 * @author  RobotPilots 镖架电控
 * @brief   串口接收回调路由(覆盖 drv_uart.c 中的弱定义)
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  USART1 -> 串口屏桩；USART3 -> DBUS 遥控；USART6 -> 裁判系统桩。
 *  全部运行在串口空闲中断上下文，只做解析/拷贝，不做控制。
 *
 * Copyright 2026 RobotPilots
 *
 * 修改日志：
 *  V0.2 2026-11-02 接入镖架各协议桩
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "config_uart.h"
#include "rc_sensor.h"
#include "drv_uart.h"
#include "judge_protocol.h"
#include "screen_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  USART1 接收：串口屏
 * @param  rxBuf DMA 缓冲
 * @param  len   本次实际收到字节数
 */
void USART1_rxDataHandler(uint8_t *rxBuf, uint16_t len)
{
    Screen_Rx(rxBuf, len);
}

/**
 * @brief  USART3 接收：遥控器 DBUS
 * @param  rxBuf 18 字节帧
 */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
    rc_sensor.update(&rc_sensor, rxBuf);
    rc_sensor.check(&rc_sensor);
}

/**
 * @brief  USART6 接收：裁判系统
 * @param  rxBuf DMA 缓冲
 * @param  len   本次实际收到字节数
 */
void USART6_rxDataHandler(uint8_t *rxBuf, uint16_t len)
{
    Judge_Rx(rxBuf, len);
}
