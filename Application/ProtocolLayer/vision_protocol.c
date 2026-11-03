/**
 ******************************************************************************
 * @file    vision_protocol.c
 * @author  RobotPilots 镖架电控
 * @brief   小电脑/视觉接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  Vision_Rx 在 USB 中断上下文中被调用，实现时只做拷贝/置标志。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "vision_protocol.h"

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  视觉模块初始化(桩)
 */
void Vision_Init(void)
{
}

/**
 * @brief  USB CDC 接收入口(中断上下文，桩)
 * @param  buf 数据
 * @param  len 长度 byte
 */
void Vision_Rx(const uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
}

/**
 * @brief  周期处理(桩)
 * @param  now 当前时刻 ms
 */
void Vision_Poll(uint32_t now)
{
    (void)now;
}

/**
 * @brief  视觉是否在线
 * @retval 当前恒为 false
 */
bool Vision_Is_Online(void)
{
    return false;
}

/**
 * @brief  读取最近一帧 Yaw 误差
 * @param  err_deg  输出误差 deg
 * @param  stamp_ms 输出帧时刻 ms
 * @retval true 数据有效；当前恒为 false
 */
bool Vision_Get_Yaw_Error(float *err_deg, uint32_t *stamp_ms)
{
    if (err_deg != NULL) {
        *err_deg = 0.0f;
    }
    if (stamp_ms != NULL) {
        *stamp_ms = 0U;
    }
    return false;
}

/**
 * @brief  回传镖架状态(桩)
 */
void Vision_Tx_Status(void)
{
}
