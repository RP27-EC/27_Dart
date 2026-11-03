/**
 ******************************************************************************
 * @file    loadcell.h
 * @author  RobotPilots 镖架电控
 * @brief   双路测力接口(桩，未接入硬件)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  拟定硬件：S 型 50kg 传感器 x2 + ADS1232 x2，测同一根拉绳两端。
 *  拟定引脚：#1 PF1 SCLK / PF0 DOUT，#2 PI6 SCLK / PI7 DOUT(未配置 GPIO)。
 *  两路分别输出，不默认相加或乘滑轮倍率；无新样本时不得更新力环。
 *  TODO: 硬件确认后实现 ADS1232 位读、零点、标定与断线判定。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __LOADCELL_H
#define __LOADCELL_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported macro ------------------------------------------------------------*/
#define LOADCELL_CH_NUM     (2U)    /* 测力通道数 */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  单路测力样本
 */
typedef struct {
    float    force_n;       /* 标定后拉力 N */
    int32_t  raw;           /* ADC 原始码值 */
    uint32_t stamp_ms;      /* 采样时刻 ms */
    uint32_t seq;           /* 样本序号，递增 */
    bool     valid;         /* 样本有效(未断线/饱和) */
} loadcell_sample_t;

/* Exported functions --------------------------------------------------------*/
void Loadcell_Init(void);
void Loadcell_Poll(uint32_t now);
bool Loadcell_Get(uint8_t ch, loadcell_sample_t *sample);
bool Loadcell_Force_Ready(void);

#endif /* __LOADCELL_H */
