/**
 ******************************************************************************
 * @file    loader.h
 * @author  RobotPilots 镖架电控
 * @brief   上膛机构：双 3508 同步带拨弹板驱动滑台
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  L(0x201) 走位置串级，R(0x202) 输出 = LOADER_R_MIRROR_SIGN * L 输出。
 *  找零：向 UP 侧堵转(机械确认拨弹板允许堵转找零)，零点后自动回 UP。
 *  单侧离线策略(用户选定 A)：
 *   - L 离线：L 是位置来源，无法闭环 -> 轴离线故障、零点作废、两侧无输出；
 *   - 仅 R 离线：L 单独原地保持(零点仍有效，L 编码器可信)，由状态机中止流程，
 *     拨回 SAFE 或保持超过 LOADER_SINGLE_HOLD_MAX_MS 后，以低速单侧回退到 UP。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __LOADER_H
#define __LOADER_H

/* Includes ------------------------------------------------------------------*/
#include "mech_axis.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  上膛预设位置
 */
typedef enum {
    LOADER_AT_UP = 0,   /* 回升位，滑台已脱开 */
    LOADER_AT_MID,      /* 中间位，快速段终点 */
    LOADER_AT_DOWN,     /* 下拉锁止位 */
    LOADER_AT_CATCH,    /* 卸力接住位 */
} loader_pos_e;

/* Exported functions --------------------------------------------------------*/
void Loader_Init(void);
void Loader_Step(uint32_t now);
mech_axis_t *Loader_Axis(void);
bool Loader_Request_Pos(loader_pos_e pos, uint32_t now);
bool Loader_Is_At(loader_pos_e pos);
bool Loader_Is_Ready_At(loader_pos_e pos);
bool Loader_Is_Fire_Clear(void);
bool Loader_Is_Single_Side(void);
uint32_t Loader_Single_Side_Tick(void);

#endif /* __LOADER_H */
