/**
 ******************************************************************************
 * @file    vision_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   小电脑/视觉接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  链路：USB OTG FS CDC(PA11/PA12)。
 *  Yaw 误差单位统一为 deg，正方向与 Yaw 逻辑正向一致，需与视觉同版约定；
 *  只有带新时间戳的帧才能用于瞄准稳定判定。
 *  TODO: 帧格式、CRC、长度检查与离线检测；状态回传。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __VISION_PROTOCOL_H
#define __VISION_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported functions --------------------------------------------------------*/
void Vision_Init(void);
void Vision_Rx(const uint8_t *buf, uint32_t len);
void Vision_Poll(uint32_t now);
bool Vision_Is_Online(void);
bool Vision_Get_Yaw_Error(float *err_deg, uint32_t *stamp_ms);
void Vision_Tx_Status(void);

#endif /* __VISION_PROTOCOL_H */
