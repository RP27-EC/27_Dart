/**
 ******************************************************************************
 * @file    can_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   CAN 接收分发(按反馈 ID 路由到电机对象)
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  由 drv_can.c 的接收中断回调调用；只解析反馈，不做控制计算。
 *
 * Copyright 2026 RobotPilots
 *
 * 修改日志：
 *  V0.2 2026-11-02 改为镖架电机表，删除模板遗留 HT/DM/KT 分发
 ******************************************************************************
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions --------------------------------------------------------*/
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf);
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

#endif /* __CAN_PROTOCOL_H */
