/**
 ******************************************************************************
 * @file    link24g_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   2.4G 队内测试同步接口(桩，硬件与链路未定)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  用途：测试时同步控制自制基地/测试靶台。
 *  C 板 USART1/USART6 已被串口屏/裁判占用，接入方式(串口分时、CAN2、
 *  SPI 等)待硬件确定，此处不预设引脚与帧格式。
 *  约定：命令带序号；同序号重发不重复执行；停止优先于移动；
 *        正式比赛模式禁用发送；发送成功不等于对端到位。
 *  TODO: 选定模块与链路后实现。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __LINK24G_PROTOCOL_H
#define __LINK24G_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  基地/靶台同步命令
 */
typedef enum {
    LINK24G_CMD_STOP = 0,      /* 停止，最高优先级 */
    LINK24G_CMD_GOTO_PRESET,   /* 移动到预设点，arg=点号 */
    LINK24G_CMD_SYNC_SHOT,     /* 镖架已释放，arg=发序 */
} link24g_cmd_e;

/* Exported functions --------------------------------------------------------*/
void Link24g_Init(void);
bool Link24g_Send_Base_Cmd(link24g_cmd_e cmd, int32_t arg);
void Link24g_Poll(uint32_t now);
bool Link24g_Is_Online(void);

#endif /* __LINK24G_PROTOCOL_H */
