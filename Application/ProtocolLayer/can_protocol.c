/**
 ******************************************************************************
 * @file    can_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   CAN 接收分发
 * @version V0.2
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  CAN1：0x201 上膛L / 0x202 上膛R / 0x203 变力
 *  CAN2：0x204 Yaw
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "can_protocol.h"
#include "dart_motor.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  CAN1 反馈分发(中断上下文)
 * @param  canId 标准帧 ID
 * @param  rxBuf 8 字节数据
 */
void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    switch (canId) {
        case DART_CAN_ID_LOADER_L:
            g_dart_motor[DART_MOTOR_LOADER_L].rx(&g_dart_motor[DART_MOTOR_LOADER_L], rxBuf);
            break;
        case DART_CAN_ID_LOADER_R:
            g_dart_motor[DART_MOTOR_LOADER_R].rx(&g_dart_motor[DART_MOTOR_LOADER_R], rxBuf);
            break;
        case DART_CAN_ID_TENSION:
            g_dart_motor[DART_MOTOR_TENSION].rx(&g_dart_motor[DART_MOTOR_TENSION], rxBuf);
            break;
        default:
            /* 未登记 ID 忽略 */
            break;
    }
}

/**
 * @brief  CAN2 反馈分发(中断上下文)
 * @param  canId 标准帧 ID
 * @param  rxBuf 8 字节数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
    switch (canId) {
        case DART_CAN_ID_YAW:
            g_dart_motor[DART_MOTOR_YAW].rx(&g_dart_motor[DART_MOTOR_YAW], rxBuf);
            break;
        default:
            /* 未登记 ID 忽略 */
            break;
    }
}
