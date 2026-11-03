/**
 ******************************************************************************
 * @file    judge_protocol.h
 * @author  RobotPilots 镖架电控
 * @brief   裁判系统接口(桩，2027 正式协议未取得)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  链路：USART6 PG14 TX / PG9 RX，115200 8N1，DMA+空闲中断。
 *  1. 本文件只定义镖架业务需要的抽象量，不预设 2027 命令字与字段布局。
 *  2. 舱门"全开"、发射窗口、比赛阶段等必须来自有效且新鲜的裁判帧；
 *     离线或过期时一律视为不允许。
 *  3. 测试模式(DART_TEST_MODE=1)下 Judge_Fire_Permit 旁路为允许，
 *     正式比赛前必须置 0 并实现解析。
 *  TODO: 取得 2027 协议后实现帧头/CRC8/CRC16/长度边界检查与字段解析；
 *        同时修复 drv_uart.c 中 USART6 空闲中断未重置 DMA 计数的问题。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __JUDGE_PROTOCOL_H
#define __JUDGE_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported macro ------------------------------------------------------------*/
#define JUDGE_RX_BUF_LEN        (100U)   /* 与 drv_uart.c USART6 缓冲一致 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  镖架闸门状态(业务抽象，来源字段待定)
 */
typedef enum {
    JUDGE_GATE_UNKNOWN = 0,  /* 未知/数据无效 */
    JUDGE_GATE_CLOSED,       /* 关闭 */
    JUDGE_GATE_MOVING,       /* 开启或关闭中，禁止发射 */
    JUDGE_GATE_OPEN,         /* 全开 */
} judge_gate_e;

/**
 * @brief  打击目标(业务抽象)
 */
typedef enum {
    JUDGE_TARGET_UNKNOWN = 0,      /* 未知 */
    JUDGE_TARGET_OUTPOST,          /* 前哨站 */
    JUDGE_TARGET_BASE_FIXED,       /* 基地固定靶 */
    JUDGE_TARGET_BASE_RAND_FIXED,  /* 基地随机固定靶 */
    JUDGE_TARGET_BASE_RAND_MOVE,   /* 基地随机移动靶 */
    JUDGE_TARGET_BASE_END_MOVE,    /* 基地末端移动靶 */
} judge_target_e;

/* Exported functions --------------------------------------------------------*/
void Judge_Init(void);
void Judge_Rx(const uint8_t *buf, uint16_t len);
void Judge_Poll(uint32_t now);
bool Judge_Is_Online(void);
bool Judge_Fire_Permit(void);
judge_gate_e Judge_Get_Gate_State(void);
judge_target_e Judge_Get_Target(void);

#endif /* __JUDGE_PROTOCOL_H */
