/**
 ******************************************************************************
 * @file    dart_fsm.h
 * @author  RobotPilots 镖架电控
 * @brief   整机模式仲裁 + 单发状态机(唯一的机构请求发起者)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  模式(S1，向 2026 看齐)：上 MANUAL / 中 SEMI_AUTO / 下 SAFE；AUTO(裁判驱动)仅预留。
 *  单发：IDLE -> LOAD_PREP -> [FEED_EARLY] -> LOAD_MID -> LOAD_DOWN -> LOCK
 *        -> LOAD_RETURN -> [FEED] -> AIM_TENSION -> ARMED -> FIRING -> RESET -> IDLE
 *  1. 防呆(两步)：拨轮第一次上拨启动储能流程，到 ARMED 后再次上拨才释放；
 *     流程中的上拨被忽略，不缓存。双控件模式下释放还需右竖摇杆推满。
 *  2. 拨轮下拨 / 离开 SEMI_AUTO / 关控：流程中止为 ABORTED，机构保持，
 *     撒放器不动作；恢复须重新上拨，不会自动重放击发。
 *  3. 储能状态 energy：上电 UNKNOWN，须在 MANUAL 中走一次人工卸力流程确认。
 *     无法证明已卸能的操作(人工锁止、锁止后点动上膛、锁止步骤中止)一律置 UNKNOWN。
 *     可能储能时不允许直接打开撒放器，只能走卸力流程(接住 -> 开锁 -> 限速送回)。
 *  4. 供弹步骤中止后不允许原样重做；S2 下位上拨表示人工已确认储弹，跳过供弹继续。
 *  5. 上电后第一次离开 SAFE 时，对未找零的轴自动堵转找零(每次上电仅一次)；
 *     拨轮下拨(手动 S2 上)或拨回 SAFE 可取消。
 *  6. 进入 SAFE 时若滑台仅靠电机拉着(撒放器未锁止且上膛不在 UP)，
 *     先限速把上膛送回 UP 再清零输出；上膛故障/L 离线/未找零时无法回退。
 *     回退时长由轴按距离计算超时，不再使用固定时限。
 *  7. 上膛仅 R 离线(单侧)：中止流程，L 原地保持；拨回 SAFE 立即单侧低速回退，
 *     其它模式下保持超过 LOADER_SINGLE_HOLD_MAX_MS 自动回退。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DART_FSM_H
#define __DART_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "dart_config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  整机模式
 */
typedef enum {
    DART_MODE_SAFE = 0,    /* 电机无输出，舵机保持 */
    DART_MODE_MANUAL,      /* 单机构调试 */
    DART_MODE_SEMI_AUTO,   /* 遥控触发单发流程 */
    DART_MODE_AUTO,        /* 预留：裁判驱动 */
} dart_mode_e;

/**
 * @brief  单发步骤
 */
typedef enum {
    SHOT_IDLE = 0,       /* 空闲 */
    SHOT_LOAD_PREP,      /* 撒放器释放、上膛回 UP */
    SHOT_FEED_EARLY,     /* 下拉前供弹(可选) */
    SHOT_LOAD_MID,       /* 上膛快速到 MID */
    SHOT_LOAD_DOWN,      /* 上膛慢速到 DOWN */
    SHOT_LOCK,           /* 撒放器锁止 */
    SHOT_LOAD_RETURN,    /* 上膛回 UP */
    SHOT_FEED,           /* 锁止后供弹(默认) */
    SHOT_AIM_TENSION,    /* Yaw 与变力到位 */
    SHOT_ARMED,          /* 待发，等待第二步 */
    SHOT_FIRING,         /* 已释放，等待出膛 */
    SHOT_RESET,          /* 撒放器回锁止 */
    SHOT_ABORTED,        /* 已中止，等待恢复 */
    SHOT_STEP_NUM,       /* 步骤总数 */
} shot_step_e;

/**
 * @brief  储能状态(软件推定)
 */
typedef enum {
    ENERGY_UNKNOWN = 0,  /* 上电未确认 */
    ENERGY_RELEASED,     /* 撒放器已释放过 */
    ENERGY_STORED,       /* 已锁止储能 */
} energy_state_e;

/**
 * @brief  启动单发被拒原因位
 */
typedef enum {
    START_DENY_NOT_ZEROED   = (1UL << 0),  /* 未找零 */
    START_DENY_ENERGY_UNKNOWN = (1UL << 1),/* 储能状态未确认 */
    START_DENY_ENERGY_STORED = (1UL << 2), /* 已储能，只能恢复或人工释放 */
    START_DENY_MAGAZINE     = (1UL << 3),  /* 4 发已打完 */
    START_DENY_FAULT        = (1UL << 4),  /* 存在故障 */
    START_DENY_MOTOR        = (1UL << 5),  /* 电机离线 */
    START_DENY_BUSY         = (1UL << 6),  /* 供弹执行中 */
    START_DENY_FEED_BROKEN  = (1UL << 7),  /* 供弹中途中止，需人工处理 */
    START_DENY_FEED_UNCAL   = (1UL << 8),  /* 本发需供弹但步骤表未标定 */
} start_deny_e;

/**
 * @brief  人工卸力步骤
 */
typedef enum {
    UNLOAD_IDLE = 0,     /* 未进行 */
    UNLOAD_TENSION,      /* 变力降到卸力预紧位(可选) */
    UNLOAD_CATCH,        /* 上膛快速到 MID(接触点前) */
    UNLOAD_CATCH_SLOW,   /* 上膛慢速到接住位，顶住滑台 */
    UNLOAD_OPEN,         /* 撒放器释放，电机承力 */
    UNLOAD_RETURN,       /* 上膛限速送回 UP */
} unload_step_e;

/**
 * @brief  人工卸力被拒原因位
 */
typedef enum {
    UNLOAD_DENY_NOT_ZEROED = (1UL << 0),  /* 上膛未找零 */
    UNLOAD_DENY_MOTOR      = (1UL << 1),  /* 上膛电机离线 */
    UNLOAD_DENY_FAULT      = (1UL << 2),  /* 上膛/变力故障 */
    UNLOAD_DENY_TENSION    = (1UL << 3),  /* 需降预紧但变力未找零 */
} unload_deny_e;

/**
 * @brief  状态机对外可读信息
 */
typedef struct {
    dart_mode_e    mode;             /* 当前模式 */
    shot_step_e    step;             /* 当前步骤 */
    shot_step_e    abort_step;       /* 中止后恢复入口 */
    uint32_t       step_tick;        /* 进入当前步骤 ms */
    uint8_t        shot_no;          /* 下一发序号 1~4，5=空 */
    uint8_t        shots_released;   /* 本次上电释放次数 */
    energy_state_e energy;           /* 储能推定 */
    bool           batch_active;     /* 连续上膛(四发骨架) */
    bool           output_enabled;   /* 允许电机输出 */
    uint32_t       last_fire_deny;   /* 最近释放拒绝原因 */
    uint32_t       last_start_deny;  /* 最近启动拒绝原因 */
    uint32_t       manual_release_tick; /* 人工释放首拨 ms */
    bool           manual_release_pending; /* 卸力首拨待确认 */
    bool           retreat_active;   /* SAFE 受控回退中 */
    uint32_t       retreat_tick;     /* 回退开始 ms */
    unload_step_e  unload_step;      /* 人工卸力步骤 */
    uint32_t       unload_tick;      /* 进入当前卸力步骤 ms */
    uint32_t       last_unload_deny; /* 最近卸力拒绝原因 */
    bool           auto_zero_done;   /* 本次上电已自动找零 */
} dart_fsm_t;

/* Exported functions --------------------------------------------------------*/
void Dart_Fsm_Init(void);
void Dart_Fsm_Step(uint32_t now);
const dart_fsm_t *Dart_Fsm_Get(void);

#endif /* __DART_FSM_H */
