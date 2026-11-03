/**
 ******************************************************************************
 * @file    screen_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   串口屏接口(桩，TJC4827T143_011C_I_P20)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  链路：USART1 PA9 TX / PB7 RX，115200 8N1。
 *  屏幕只提交参数修改请求，镖架在静止安全状态下校验范围后才生效；
 *  参数存储需带版本与 CRC，写失败有界重试。
 *  TODO: 帧格式、参数表映射、状态/故障显示刷新；
 *        同时修复 drv_uart.c 中 USART1 空闲中断缓冲问题。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __SCREEN_PROTOCOL_H
#define __SCREEN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported functions --------------------------------------------------------*/
void Screen_Init(void);
void Screen_Rx(const uint8_t *buf, uint16_t len);
void Screen_Poll(uint32_t now);

#endif /* __SCREEN_PROTOCOL_H */
