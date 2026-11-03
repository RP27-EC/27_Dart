/**
 ******************************************************************************
 * @file    rc_cmd.h
 * @author  RobotPilots 镖架电控
 * @brief   遥控原始数据 -> 操作命令(拨杆电平、拨轮边沿、摇杆归一化)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  1. 每个控制周期在 ControlTask 开头调用 Rc_Cmd_Update 生成一次快照，
 *     其余模块只读快照，避免与 DBUS 中断交错读取。
 *  2. 拨轮事件只在越过阈值的那一个周期有效，必须回中后才能再次触发；
 *     不缓存事件，长按不会重复触发。
 *  3. 遥控离线/数据异常 -> online=false；重新上线后必须先把 S1 拨到下位
 *     (safe_seen=true)才允许离开安全模式。
 *  4. s1/s2 沿用模板命名(s1 为 26 工程的模式拨杆)，物理左右以实物为准。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __RC_CMD_H
#define __RC_CMD_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  拨杆位置
 */
typedef enum {
    RC_LEVER_INVALID = 0,   /* 无效/离线 */
    RC_LEVER_UP,            /* 上 */
    RC_LEVER_MID,           /* 中 */
    RC_LEVER_DOWN,          /* 下 */
} rc_lever_e;

/**
 * @brief  拨轮边沿事件
 */
typedef enum {
    RC_WHEEL_EVT_NONE = 0,  /* 无事件 */
    RC_WHEEL_EVT_UP,        /* 本周期上拨 */
    RC_WHEEL_EVT_DOWN,      /* 本周期下拨 */
} rc_wheel_evt_e;

/**
 * @brief  遥控命令快照
 */
typedef struct {
    bool online;               /* 在线且数据合法 */
    bool safe_seen;            /* 上线后已见 S1 下位 */
    rc_lever_e s1;             /* 模式拨杆 */
    rc_lever_e s2;             /* 功能拨杆 */
    float ch0;                 /* 右横 -1~1，含死区 */
    float ch1;                 /* 右竖 -1~1 */
    float ch2;                 /* 左横 -1~1 */
    float ch3;                 /* 左竖 -1~1 */
    rc_wheel_evt_e wheel_evt;  /* 本周期拨轮事件 */
    bool fire_confirm;         /* 双控件确认(右竖推满) */
} rc_cmd_t;

/* Exported functions --------------------------------------------------------*/
void Rc_Cmd_Init(void);
void Rc_Cmd_Update(uint32_t now);
const rc_cmd_t *Rc_Cmd_Get(void);

#endif /* __RC_CMD_H */
