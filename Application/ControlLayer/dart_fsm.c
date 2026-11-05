/**
 ******************************************************************************
 * @file    dart_fsm.c
 * @author  RobotPilots 镖架电控
 * @brief   整机模式仲裁 + 单发状态机实现
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  遥控映射见 docs/遥控器手册.md，状态转移见 docs/状态机说明.md，修改时必须同步。
 *  本文件是三根轴、撒放器、供弹的唯一请求发起者(单一写者)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_fsm.h"
#include "rc_cmd.h"
#include "fault.h"
#include "fire_gate.h"
#include "dart_motor.h"
#include "loader.h"
#include "tension.h"
#include "yaw.h"
#include "trigger.h"
#include "feeder.h"
#include "tf_log.h"
#include "link24g_protocol.h"
#include "buzzer.h"
#include "dart_tune.h"

/* Private variables ---------------------------------------------------------*/
static dart_fsm_t s_fsm;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  三根轴批量操作
 */
static void Fsm_Axes_Enable(void)
{
    Mech_Axis_Enable(Loader_Axis());
    Mech_Axis_Enable(Tension_Axis());
    Mech_Axis_Enable(Yaw_Axis());
}

static void Fsm_Axes_Disable_And_Ack(void)
{
    /* 先确认故障(FAULT->DISABLED)，再统一失能 */
    Mech_Axis_Clear_Fault(Loader_Axis());
    Mech_Axis_Clear_Fault(Tension_Axis());
    Mech_Axis_Clear_Fault(Yaw_Axis());
    Mech_Axis_Disable(Loader_Axis());
    Mech_Axis_Disable(Tension_Axis());
    Mech_Axis_Disable(Yaw_Axis());
}

static void Fsm_Axes_Hold(void)
{
    Mech_Axis_Hold(Loader_Axis());
    Mech_Axis_Hold(Tension_Axis());
    Mech_Axis_Hold(Yaw_Axis());
}

/**
 * @brief  步骤是否属于"流程进行中"
 */
static bool Fsm_Is_Running(void)
{
    return (s_fsm.step != SHOT_IDLE) && (s_fsm.step != SHOT_ABORTED);
}

/**
 * @brief  中止当前流程：机构原地保持，撒放器不动作
 */
static void Fsm_Abort(void)
{
    shot_step_e old_step = s_fsm.step;

    if (!Fsm_Is_Running()) {
        return;
    }
    switch (old_step) {
        case SHOT_ARMED:
            /* 恢复时重新确认瞄准与变力 */
            s_fsm.abort_step = SHOT_AIM_TENSION;
            break;
        case SHOT_FIRING:
        case SHOT_RESET:
            /* 撒放器已释放，只剩复位 */
            s_fsm.abort_step = SHOT_RESET;
            break;
        default:
            s_fsm.abort_step = old_step;
            break;
    }
    /* NOTE: 锁止命令已发出但未确认，按可能储能处理 */
    if ((old_step == SHOT_LOCK) && (s_fsm.energy != ENERGY_STORED)) {
        s_fsm.energy = ENERGY_UNKNOWN;
    }
    s_fsm.step = SHOT_ABORTED;
    s_fsm.batch_active = false;
    Fsm_Axes_Hold();
    Feeder_Abort();
    (void)Log_Event(LOG_EVT_SHOT_ABORTED, (int32_t)old_step);
}

/**
 * @brief  执行进入步骤时的一次性请求
 * @retval false 请求被模块拒绝
 */
static bool Fsm_Step_Enter(shot_step_e step, uint32_t now)
{
    bool ok = true;

    switch (step) {
        case SHOT_LOAD_PREP:
            /* 仅在已确认未储能时到达这里 */
            Trigger_Request_Release(now);
            ok = Loader_Request_Pos(LOADER_AT_UP, now);
            break;
        case SHOT_FEED_EARLY:
        case SHOT_FEED:
            Feeder_Ack();
            ok = Feeder_Request_Shot(s_fsm.shot_no, now);
            break;
        case SHOT_LOAD_MID:
            ok = Loader_Request_Pos(LOADER_AT_MID, now);
            break;
        case SHOT_LOAD_DOWN:
            ok = Loader_Request_Pos(LOADER_AT_DOWN, now);
            break;
        case SHOT_LOCK:
        case SHOT_RESET:
            Trigger_Request_Lock(now);
            break;
        case SHOT_LOAD_RETURN:
            ok = Loader_Request_Pos(LOADER_AT_UP, now);
            break;
        case SHOT_AIM_TENSION:
            ok = Yaw_Request_Shot(s_fsm.shot_no, now) &&
                 Tension_Request_Shot(s_fsm.shot_no, now);
            break;
        default:
            break;
    }
    return ok;
}

/**
 * @brief  切换到下一步骤；请求被拒则记故障并中止
 */
static void Fsm_Goto(shot_step_e next, uint32_t now)
{
    s_fsm.step = next;
    s_fsm.step_tick = now;
    (void)Log_Event(LOG_EVT_SHOT_STEP, (int32_t)next);
    if (next == SHOT_ARMED) {
        Buzzer_Play(BUZZER_TONE_ARMED);
    }
    if (!Fsm_Step_Enter(next, now)) {
        Fault_Set(FAULT_SHOT_REQUEST_REJECT);
        Fsm_Abort();
    }
}

/**
 * @brief  启动新一发的前置检查
 * @retval 拒绝原因位，0 为允许
 */
static uint32_t Fsm_Start_Check(void)
{
    uint32_t deny = 0U;

    if (!Loader_Axis()->zeroed || !Tension_Axis()->zeroed || !Yaw_Axis()->zeroed) {
        deny |= START_DENY_NOT_ZEROED;
    }
    if (s_fsm.energy == ENERGY_UNKNOWN) {
        deny |= START_DENY_ENERGY_UNKNOWN;
    }
    if (s_fsm.energy == ENERGY_STORED) {
        deny |= START_DENY_ENERGY_STORED;
    }
    if (s_fsm.shot_no > DART_SHOT_MAX) {
        deny |= START_DENY_MAGAZINE;
    }
    if (Fault_Get_Mask() != 0U) {
        deny |= START_DENY_FAULT;
    }
    if (!Dart_Motor_All_Online()) {
        deny |= START_DENY_MOTOR;
    }
    if (Feeder_Get_Status() == FEEDER_RUNNING) {
        deny |= START_DENY_BUSY;
    }
#if (DART_FEEDER_CALIBRATED == 0)
    /* 占位步骤表不会真实供弹，禁止进入自动流程；MANUAL 单测不受限 */
    if ((s_fsm.shot_no <= DART_SHOT_MAX) && Feeder_Shot_Needs_Feed(s_fsm.shot_no)) {
        deny |= START_DENY_FEED_UNCAL;
    }
#endif
    return deny;
}

/**
 * @brief  尝试启动新一发
 */
static void Fsm_Try_Start(uint32_t now)
{
    s_fsm.last_start_deny = Fsm_Start_Check();
    if (s_fsm.last_start_deny != 0U) {
        s_fsm.batch_active = false;
        Buzzer_Play(BUZZER_TONE_DENY);
        return;
    }
    Fsm_Goto(SHOT_LOAD_PREP, now);
}

/**
 * @brief  供弹步骤之后的下一步
 */
static shot_step_e Fsm_Step_After_Feed(shot_step_e feed_step)
{
    return (feed_step == SHOT_FEED_EARLY) ? SHOT_LOAD_MID : SHOT_AIM_TENSION;
}

/**
 * @brief  从 ABORTED 恢复到记录的步骤
 * @param  feed_confirmed 操作手确认供弹已人工处理(S2 下位上拨)
 * @note   供弹步骤中止后不重做序列：未确认时拒绝，确认后跳过供弹
 */
static void Fsm_Try_Resume(bool feed_confirmed, uint32_t now)
{
    uint32_t deny = Fsm_Start_Check();
    shot_step_e target = s_fsm.abort_step;
    bool feed_broken = (target == SHOT_FEED) || (target == SHOT_FEED_EARLY);

    /* 恢复允许已储能，其余条件与启动相同 */
    deny &= ~((uint32_t)START_DENY_ENERGY_STORED | (uint32_t)START_DENY_MAGAZINE);
    if ((s_fsm.abort_step >= SHOT_LOCK) && (s_fsm.energy == ENERGY_UNKNOWN)) {
        deny &= ~(uint32_t)START_DENY_ENERGY_UNKNOWN;
    }
    if (feed_broken) {
        if (feed_confirmed) {
            target = Fsm_Step_After_Feed(target);
        } else {
            deny |= START_DENY_FEED_BROKEN;
        }
    }
    s_fsm.last_start_deny = deny;
    if (deny != 0U) {
        Buzzer_Play(BUZZER_TONE_DENY);
        return;
    }
    if (feed_broken) {
        Feeder_Ack();
    }
    Fsm_Goto(target, now);
}

/**
 * @brief  ARMED 下的释放请求：集中复核许可后释放
 * @param  confirm_ok 双控件确认
 */
static void Fsm_Try_Fire(bool confirm_ok, uint32_t now)
{
    fire_gate_result_t gate = Fire_Gate_Check(s_fsm.mode == DART_MODE_SEMI_AUTO, confirm_ok);

    s_fsm.last_fire_deny = gate.deny_mask;
    if (!gate.allow) {
        (void)Log_Event(LOG_EVT_FIRE_DENIED, (int32_t)gate.deny_mask);
        Buzzer_Play(BUZZER_TONE_DENY);
        return;
    }
    Trigger_Request_Release(now);
    s_fsm.energy = ENERGY_RELEASED;
    s_fsm.shots_released++;
    (void)Log_Event(LOG_EVT_SHOT_RELEASED, (int32_t)s_fsm.shot_no);
#if DART_LINK24G_ENABLE
    /* NOTE: 只入队，不阻塞控制周期；以释放命令时刻作为出镖计时起点 */
    Link24g_Notify_Shot(s_fsm.shot_no);
#endif
    s_fsm.step = SHOT_FIRING;
    s_fsm.step_tick = now;
    (void)Log_Event(LOG_EVT_SHOT_STEP, (int32_t)SHOT_FIRING);
}

/**
 * @brief  轴故障登记；流程中出现故障则中止
 */
static void Fsm_Check_Axis_Fault(uint32_t now)
{
    bool any_fault = false;

    (void)now;
    if (Loader_Axis()->status == MECH_AXIS_FAULT) {
        any_fault = true;
        if (Loader_Axis()->err != MECH_AXIS_ERR_OFFLINE) {
            Fault_Set(FAULT_LOADER_AXIS);
        }
    }
    if (Tension_Axis()->status == MECH_AXIS_FAULT) {
        any_fault = true;
        if (Tension_Axis()->err != MECH_AXIS_ERR_OFFLINE) {
            Fault_Set(FAULT_TENSION_AXIS);
        }
    }
    if (Yaw_Axis()->status == MECH_AXIS_FAULT) {
        any_fault = true;
        if (Yaw_Axis()->err != MECH_AXIS_ERR_OFFLINE) {
            Fault_Set(FAULT_YAW_AXIS);
        }
    }
    if (any_fault) {
        Fsm_Abort();
    }
}

/**
 * @brief  该步骤是否使用状态机兜底超时(轴运动类步骤除外)
 */
static bool Fsm_Step_Has_Timeout(shot_step_e step)
{
    switch (step) {
        case SHOT_FEED_EARLY:
        case SHOT_LOCK:
        case SHOT_FEED:
        case SHOT_FIRING:
        case SHOT_RESET:
            return true;
        default:
            return false;
    }
}

/**
 * @brief  单发步骤推进(仅 SEMI_AUTO 下调用)
 */
static void Fsm_Shot_Run(uint32_t now)
{
    switch (s_fsm.step) {
        case SHOT_LOAD_PREP:
            if ((Trigger_Get_State() == TRIGGER_RELEASED) && Loader_Is_Ready_At(LOADER_AT_UP)) {
                s_fsm.energy = ENERGY_RELEASED;
                Fsm_Goto(DART_FEED_BEFORE_LOAD ? SHOT_FEED_EARLY : SHOT_LOAD_MID, now);
            }
            break;
        case SHOT_FEED_EARLY:
            if (Feeder_Get_Status() == FEEDER_DONE) {
                Feeder_Ack();
                Fsm_Goto(SHOT_LOAD_MID, now);
            }
            break;
        case SHOT_LOAD_MID:
            if (Loader_Is_Ready_At(LOADER_AT_MID)) {
                Fsm_Goto(SHOT_LOAD_DOWN, now);
            }
            break;
        case SHOT_LOAD_DOWN:
            if (Loader_Is_Ready_At(LOADER_AT_DOWN)) {
                Fsm_Goto(SHOT_LOCK, now);
            }
            break;
        case SHOT_LOCK:
            if (Trigger_Get_State() == TRIGGER_LOCKED) {
                s_fsm.energy = ENERGY_STORED;
                Fsm_Goto(SHOT_LOAD_RETURN, now);
            }
            break;
        case SHOT_LOAD_RETURN:
            if (Loader_Is_Ready_At(LOADER_AT_UP)) {
                Fsm_Goto(DART_FEED_BEFORE_LOAD ? SHOT_AIM_TENSION : SHOT_FEED, now);
            }
            break;
        case SHOT_FEED:
            if (Feeder_Get_Status() == FEEDER_DONE) {
                Feeder_Ack();
                Fsm_Goto(SHOT_AIM_TENSION, now);
            }
            break;
        case SHOT_AIM_TENSION:
            if (Yaw_Is_Ready() && Tension_Is_Ready()) {
                Fsm_Goto(SHOT_ARMED, now);
            }
            break;
        case SHOT_FIRING:
            if ((Trigger_Get_State() == TRIGGER_RELEASED) &&
                Dart_Time_Elapsed(now, s_fsm.step_tick, SHOT_FIRE_SETTLE_MS)) {
                Fsm_Goto(SHOT_RESET, now);
            }
            break;
        case SHOT_RESET:
            if (Trigger_Get_State() == TRIGGER_LOCKED) {
                s_fsm.shot_no++;
                Fsm_Goto(SHOT_IDLE, now);
                /* 连续上膛：自动准备下一发，释放仍需操作手 */
                if (s_fsm.batch_active && (s_fsm.shot_no <= DART_SHOT_MAX)) {
                    Fsm_Try_Start(now);
                } else {
                    s_fsm.batch_active = false;
                }
            }
            break;
        case SHOT_IDLE:
        case SHOT_ARMED:
        case SHOT_ABORTED:
        default:
            break;
    }

    /* 兜底超时：只用于舵机/计时类步骤；轴运动步骤由轴按距离计算超时，ARMED 无限等待 */
    if (Fsm_Is_Running() && Fsm_Step_Has_Timeout(s_fsm.step) &&
        Dart_Time_Elapsed(now, s_fsm.step_tick, SHOT_STEP_TIMEOUT_MS)) {
        Fault_Set(FAULT_SHOT_TIMEOUT);
        Fsm_Abort();
    }
}

/**
 * @brief  SEMI_AUTO 下的遥控事件
 *         S2 上：拨轮上=启动/释放/恢复；S2 中：拨轮上=连续上膛；
 *         S2 下：拨轮上=确认供弹已人工处理后恢复；任意 S2 拨轮下=中止
 */
static void Fsm_Semi_Auto_Input(const rc_cmd_t *rc, uint32_t now)
{
    bool confirm_ok;

    if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
        Fsm_Abort();
        return;
    }
    if (rc->wheel_evt != RC_WHEEL_EVT_UP) {
        return;
    }
    switch (s_fsm.step) {
        case SHOT_IDLE:
            if (rc->s2 == RC_LEVER_UP) {
                Fsm_Try_Start(now);
            } else if (rc->s2 == RC_LEVER_MID) {
                s_fsm.batch_active = true;
                Fsm_Try_Start(now);
            } else {
                /* S2 下：预留 */
            }
            break;
        case SHOT_ARMED:
#if (DART_FOOLPROOF_MODE == DART_FOOLPROOF_DUAL_INPUT)
            confirm_ok = rc->fire_confirm;
#else
            confirm_ok = true;
#endif
            Fsm_Try_Fire(confirm_ok, now);
            break;
        case SHOT_ABORTED:
            Fsm_Try_Resume(rc->s2 == RC_LEVER_DOWN, now);
            break;
        default:
            /* 流程中的上拨忽略，不缓存 */
            break;
    }
}

/**
 * @brief  MANUAL 下任何人工操作都会丢弃中止现场
 */
static void Fsm_Manual_Discard_Abort(void)
{
    if (s_fsm.step == SHOT_ABORTED) {
        /* 丢弃现场不等于已卸能：锁止后中止的保持 STORED/UNKNOWN */
        s_fsm.step = SHOT_IDLE;
        Feeder_Ack();
    }
}

/**
 * @brief  人工卸力前置条件检查
 * @retval 拒绝原因位(unload_deny_e)，0 为允许
 */
static uint32_t Fsm_Unload_Check(void)
{
    uint32_t deny = 0U;

    if (!Loader_Axis()->zeroed) {
        deny |= UNLOAD_DENY_NOT_ZEROED;
    }
    if (!Dart_Motor_Is_Online(DART_MOTOR_LOADER_L) || !Dart_Motor_Is_Online(DART_MOTOR_LOADER_R)) {
        deny |= UNLOAD_DENY_MOTOR;
    }
    if (Loader_Axis()->status == MECH_AXIS_FAULT) {
        deny |= UNLOAD_DENY_FAULT;
    }
#if UNLOAD_REDUCE_TENSION
    if (Tension_Axis()->status == MECH_AXIS_FAULT) {
        deny |= UNLOAD_DENY_FAULT;
    }
    if (!Tension_Axis()->zeroed) {
        deny |= UNLOAD_DENY_TENSION;
    }
#endif
    return deny;
}

/**
 * @brief  中止人工卸力：上膛与变力原地保持，撒放器不动作
 * @note   若已开锁，滑台由电机拉着；之后进入 SAFE 会触发受控回退
 */
static void Fsm_Unload_Abort(void)
{
    if (s_fsm.unload_step == UNLOAD_IDLE) {
        return;
    }
    s_fsm.unload_step = UNLOAD_IDLE;
    Mech_Axis_Hold(Loader_Axis());
    Mech_Axis_Hold(Tension_Axis());
}

/**
 * @brief  切换卸力步骤并执行进入动作；请求被拒则记故障并中止
 */
static void Fsm_Unload_Goto(unload_step_e step, uint32_t now)
{
    bool ok = true;

    s_fsm.unload_step = step;
    s_fsm.unload_tick = now;
    switch (step) {
        case UNLOAD_TENSION:
            ok = Mech_Axis_Request_Move(Tension_Axis(), TENSION_POS_UNLOAD, 0.0f, now);
            break;
        case UNLOAD_CATCH:
            /* 快速段只走到接触点之前(MID)，避免高速撞上滑台 */
            ok = Loader_Request_Pos(LOADER_AT_MID, now);
            break;
        case UNLOAD_CATCH_SLOW:
            /* 慢速到接住位，重新顶住已锁止的滑台 */
            ok = Loader_Request_Pos(LOADER_AT_CATCH, now);
            break;
        case UNLOAD_OPEN:
            /* 此时拉簧力由上膛电机承担，开锁不会弹出 */
            Trigger_Request_Release(now);
            break;
        case UNLOAD_RETURN:
            ok = Mech_Axis_Request_Move(Loader_Axis(), LOADER_POS_UP, LOADER_SPEED_RETREAT, now);
            break;
        case UNLOAD_IDLE:
        default:
            break;
    }
    if (!ok) {
        Fault_Set(FAULT_SHOT_REQUEST_REJECT);
        Fsm_Unload_Abort();
    }
}

/**
 * @brief  启动人工卸力(2a 反向接力)：[降预紧] -> 接住 -> 开锁 -> 限速送回
 */
static void Fsm_Unload_Start(uint32_t now)
{
    s_fsm.last_unload_deny = Fsm_Unload_Check();
    if (s_fsm.last_unload_deny != 0U) {
        Buzzer_Play(BUZZER_TONE_DENY);
        return;
    }
    Fsm_Manual_Discard_Abort();
    Fsm_Unload_Goto(UNLOAD_REDUCE_TENSION ? UNLOAD_TENSION : UNLOAD_CATCH, now);
}

/**
 * @brief  人工卸力步骤推进(仅 MANUAL 下调用)
 */
static void Fsm_Unload_Run(uint32_t now)
{
    if (s_fsm.unload_step == UNLOAD_IDLE) {
        return;
    }
    if ((Loader_Axis()->status == MECH_AXIS_FAULT) ||
        (UNLOAD_REDUCE_TENSION && (Tension_Axis()->status == MECH_AXIS_FAULT))) {
        Fsm_Unload_Abort();
        return;
    }
    switch (s_fsm.unload_step) {
        case UNLOAD_TENSION:
            if (Mech_Axis_Is_Ready(Tension_Axis())) {
                Fsm_Unload_Goto(UNLOAD_CATCH, now);
            }
            break;
        case UNLOAD_CATCH:
            if (Loader_Is_Ready_At(LOADER_AT_MID)) {
                Fsm_Unload_Goto(UNLOAD_CATCH_SLOW, now);
            }
            break;
        case UNLOAD_CATCH_SLOW:
            if (Loader_Is_Ready_At(LOADER_AT_CATCH)) {
                Fsm_Unload_Goto(UNLOAD_OPEN, now);
            }
            break;
        case UNLOAD_OPEN:
            if ((Trigger_Get_State() == TRIGGER_RELEASED) &&
                Dart_Time_Elapsed(now, s_fsm.unload_tick, TRIGGER_SETTLE_MS + UNLOAD_OPEN_HOLD_MS)) {
                Fsm_Unload_Goto(UNLOAD_RETURN, now);
            }
            break;
        case UNLOAD_RETURN:
            if (Loader_Is_Ready_At(LOADER_AT_UP)) {
                s_fsm.unload_step = UNLOAD_IDLE;
                s_fsm.energy = ENERGY_RELEASED;
                (void)Log_Event(LOG_EVT_UNLOAD_DONE, 0);
            }
            break;
        case UNLOAD_IDLE:
        default:
            break;
    }
    /* 轴运动步骤由轴按距离计算超时；此处只兜底开锁等待 */
    if ((s_fsm.unload_step == UNLOAD_OPEN) &&
        Dart_Time_Elapsed(now, s_fsm.unload_tick, SHOT_STEP_TIMEOUT_MS)) {
        Fault_Set(FAULT_SHOT_TIMEOUT);
        Fsm_Unload_Abort();
    }
}

/**
 * @brief  人工切换撒放器
 *         撒放器已打开 -> 锁止；确认无储能 -> 直接打开(调试)；
 *         可能储能 -> 窗口内二次上拨后走人工卸力流程，不直接打开
 */
static void Fsm_Manual_Trigger_Toggle(uint32_t now)
{
    trigger_state_e state = Trigger_Get_State();

    if ((state == TRIGGER_RELEASED) || (state == TRIGGER_RELEASING)) {
        /* 人工锁止可能锁住已下拉的滑台，无法证明未储能 */
        Trigger_Request_Lock(now);
        if (s_fsm.energy != ENERGY_STORED) {
            s_fsm.energy = ENERGY_UNKNOWN;
        }
        return;
    }
    if (s_fsm.energy == ENERGY_RELEASED) {
        s_fsm.manual_release_pending = false;
        Trigger_Request_Release(now);
        (void)Log_Event(LOG_EVT_MANUAL_RELEASE, (int32_t)s_fsm.shot_no);
        return;
    }
    if (!s_fsm.manual_release_pending ||
        Dart_Time_Elapsed(now, s_fsm.manual_release_tick, MANUAL_RELEASE_CONFIRM_MS)) {
        s_fsm.manual_release_pending = true;
        s_fsm.manual_release_tick = now;
        return;
    }
    s_fsm.manual_release_pending = false;
    Fsm_Unload_Start(now);
}

/**
 * @brief  找零开关(同 26 MT 模式拨轮下拨)：
 *         任一轴找零中 -> 全部取消并原地保持；否则三轴同时开始堵转找零
 */
static void Fsm_Manual_Zero_Toggle(uint32_t now)
{
    if ((Loader_Axis()->status == MECH_AXIS_ZEROING) ||
        (Tension_Axis()->status == MECH_AXIS_ZEROING) ||
        (Yaw_Axis()->status == MECH_AXIS_ZEROING)) {
        Fsm_Axes_Hold();
        return;
    }
    (void)Mech_Axis_Request_Zero(Loader_Axis(), now);
    (void)Mech_Axis_Request_Zero(Tension_Axis(), now);
    (void)Mech_Axis_Request_Zero(Yaw_Axis(), now);
}

/**
 * @brief  MANUAL(S1 上)下的遥控操作，映射向 2026 看齐
 *         S2 上：右竖上膛点动 / 拨轮上 撒放器切换或卸力 / 拨轮下 找零开关
 *         S2 中：右横 Yaw、左竖变力点动 / 拨轮上 供弹单测 / 拨轮下 发序+1
 *         S2 下：拨轮上 供弹舵机回初始位 / 拨轮下 Yaw 与变力到当前发序预设
 *         卸力进行中只响应拨轮下拨(中止)
 */
static void Fsm_Manual_Input(const rc_cmd_t *rc, uint32_t now)
{
    bool jog = false;

    if (s_fsm.unload_step != UNLOAD_IDLE) {
        if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
            Fsm_Unload_Abort();
        }
        Fsm_Unload_Run(now);
        return;
    }

    switch (rc->s2) {
        case RC_LEVER_UP:
            if (rc->ch1 != 0.0f) {
                Mech_Axis_Jog(Loader_Axis(), (int32_t)(rc->ch1 * RC_STICK_FULL * LOADER_JOG_GAIN));
                jog = true;
                /* 撒放器未释放时点动上膛，滑台可能被锁住形成储能 */
                if ((Trigger_Get_State() != TRIGGER_RELEASED) && (s_fsm.energy != ENERGY_STORED)) {
                    s_fsm.energy = ENERGY_UNKNOWN;
                }
            }
            if (rc->wheel_evt == RC_WHEEL_EVT_UP) {
                Fsm_Manual_Trigger_Toggle(now);
            } else if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
                Fsm_Manual_Zero_Toggle(now);
            } else {
                /* 无事件 */
            }
            break;

        case RC_LEVER_MID:
            if (rc->ch0 != 0.0f) {
                Yaw_Jog_Deg(rc->ch0 * RC_STICK_FULL * YAW_JOG_GAIN);
                jog = true;
            }
            if (rc->ch3 != 0.0f) {
                Mech_Axis_Jog(Tension_Axis(), (int32_t)(rc->ch3 * RC_STICK_FULL * TENSION_JOG_GAIN));
                jog = true;
            }
            if (rc->wheel_evt == RC_WHEEL_EVT_UP) {
                Feeder_Ack();
                (void)Feeder_Request_Shot(s_fsm.shot_no, now);
            } else if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
                s_fsm.shot_no = (s_fsm.shot_no >= DART_SHOT_MAX) ? 1U : (uint8_t)(s_fsm.shot_no + 1U);
            } else {
                /* 无事件 */
            }
            break;

        case RC_LEVER_DOWN:
            if (rc->wheel_evt == RC_WHEEL_EVT_UP) {
                (void)Feeder_Request_Home(now);
            } else if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
                (void)Yaw_Request_Shot(s_fsm.shot_no, now);
                (void)Tension_Request_Shot(s_fsm.shot_no, now);
            } else {
                /* 无事件 */
            }
            break;

        default:
            break;
    }

    if ((jog || (rc->wheel_evt != RC_WHEEL_EVT_NONE)) && (s_fsm.unload_step == UNLOAD_IDLE)) {
        Fsm_Manual_Discard_Abort();
    }
}

/**
 * @brief  进入 SAFE 时是否需要受控回退
 * @retval true 滑台仅靠电机拉着(撒放器未锁止且上膛不在 UP)，且上膛可控
 */
static bool Fsm_Retreat_Needed(void)
{
    const mech_axis_t *loader = Loader_Axis();

    return loader->zeroed &&
           (loader->status != MECH_AXIS_FAULT) &&
           (loader->status != MECH_AXIS_DISABLED) &&
           Dart_Motor_Is_Online(DART_MOTOR_LOADER_L) &&
           !Loader_Is_At(LOADER_AT_UP) &&
           (Trigger_Get_State() != TRIGGER_LOCKED);
}

/**
 * @brief  发起受控回退：上膛限速回 UP(单侧时用更低限速)
 * @retval false 上膛不接受请求
 * @note   超时由轴按距离计算(Mech_Axis_Move_Timeout)，超时即轴故障并结束回退
 */
static bool Fsm_Retreat_Start(uint32_t now)
{
    float speed = Loader_Is_Single_Side() ? LOADER_SPEED_RETREAT_SINGLE : LOADER_SPEED_RETREAT;

    if (!Mech_Axis_Request_Move(Loader_Axis(), LOADER_POS_UP, speed, now)) {
        return false;
    }
    s_fsm.retreat_active = true;
    s_fsm.retreat_tick = now;
    /* 回退后拨弹板已离开滑台，恢复须从 MID 重新下拉；
       LOCK 中止时储能为 UNKNOWN，恢复会被拒绝，须先卸力 */
    if ((s_fsm.abort_step == SHOT_LOAD_MID) || (s_fsm.abort_step == SHOT_LOAD_DOWN) ||
        (s_fsm.abort_step == SHOT_LOCK)) {
        s_fsm.abort_step = SHOT_LOAD_MID;
    }
    (void)Log_Event(LOG_EVT_SAFE_RETREAT, 1);
    return true;
}

/**
 * @brief  受控回退推进：到 UP、轴故障(含按距离计算的超时)或失能即结束
 *         SAFE 下结束后清零输出；其它模式(单侧自动回退)下结束后原地保持
 */
static void Fsm_Safe_Retreat_Run(uint32_t now)
{
    (void)now;
    if (!s_fsm.retreat_active) {
        return;
    }
    if (Loader_Is_Ready_At(LOADER_AT_UP) ||
        (Loader_Axis()->status == MECH_AXIS_FAULT) ||
        (Loader_Axis()->status == MECH_AXIS_DISABLED)) {
        s_fsm.retreat_active = false;
        if (s_fsm.mode == DART_MODE_SAFE) {
            Mech_Axis_Clear_Fault(Loader_Axis());
            Mech_Axis_Disable(Loader_Axis());
            s_fsm.output_enabled = false;
        } else {
            Mech_Axis_Hold(Loader_Axis());
        }
        (void)Log_Event(LOG_EVT_SAFE_RETREAT, 0);
    }
}

/**
 * @brief  上膛单侧(仅 R 离线)处理：中止流程，超时自动回退
 */
static void Fsm_Single_Side_Check(uint32_t now)
{
    if (!Loader_Is_Single_Side()) {
        return;
    }
    Fsm_Abort();
    Fsm_Unload_Abort();
    if (!s_fsm.retreat_active && (s_fsm.mode != DART_MODE_SAFE) && Fsm_Retreat_Needed() &&
        Dart_Time_Elapsed(now, Loader_Single_Side_Tick(), LOADER_SINGLE_HOLD_MAX_MS)) {
        (void)Fsm_Retreat_Start(now);
    }
}

/**
 * @brief  撒放器已打开且上膛在 UP 时，滑台不可能储能，推定为已释放
 */
static void Fsm_Energy_Infer(void)
{
    if ((s_fsm.energy != ENERGY_RELEASED) &&
        (s_fsm.unload_step == UNLOAD_IDLE) &&
        !Fsm_Is_Running() &&
        (Trigger_Get_State() == TRIGGER_RELEASED) &&
        Loader_Is_At(LOADER_AT_UP)) {
        s_fsm.energy = ENERGY_RELEASED;
    }
}

/**
 * @brief  由遥控快照决定模式(向 2026 看齐：上 手动 / 中 半自动 / 下 安全)
 */
static dart_mode_e Fsm_Resolve_Mode(const rc_cmd_t *rc)
{
    if (!rc->online || !rc->safe_seen) {
        return DART_MODE_SAFE;
    }
    switch (rc->s1) {
        case RC_LEVER_UP:
            return DART_MODE_MANUAL;
        case RC_LEVER_MID:
            return DART_MODE_SEMI_AUTO;
        case RC_LEVER_DOWN:
        default:
            return DART_MODE_SAFE;
    }
}

/**
 * @brief  上电后第一次离开 SAFE：对未找零的轴发起堵转找零(每次上电仅一次)
 * @note   以操作手拨离安全位作为意图确认，避免上电瞬间机构自行动作；
 *         电机离线导致零点丢失后不自动重找，须人工找零
 */
static void Fsm_Auto_Zero(uint32_t now)
{
#if DART_AUTO_ZERO_ENABLE
    if (s_fsm.auto_zero_done) {
        return;
    }
    s_fsm.auto_zero_done = true;
    if (!Loader_Axis()->zeroed) {
        (void)Mech_Axis_Request_Zero(Loader_Axis(), now);
    }
    if (!Tension_Axis()->zeroed) {
        (void)Mech_Axis_Request_Zero(Tension_Axis(), now);
    }
    if (!Yaw_Axis()->zeroed) {
        (void)Mech_Axis_Request_Zero(Yaw_Axis(), now);
    }
#else
    (void)now;
#endif
}

/**
 * @brief  模式切换的进入/退出动作
 * @note   进入 SAFE 时若滑台仅靠电机拉着，先受控回退再清零(停止类别 1)
 */
static void Fsm_Mode_Change(dart_mode_e new_mode, uint32_t now)
{
    dart_mode_e old_mode = s_fsm.mode;
    bool retreat;

    if (new_mode == old_mode) {
        return;
    }
    if (new_mode == DART_MODE_SAFE) {
        retreat = Fsm_Retreat_Needed();
        /* 先退出调参，避免其"原地保持"覆盖随后发起的回退运动 */
        g_dart_tune.enable = 0U;
        Dart_Tune_Stop();
        Fsm_Abort();
        Fsm_Unload_Abort();
        Feeder_Abort();
        if (retreat) {
            retreat = Fsm_Retreat_Start(now);
        }
        if (retreat) {
            /* 只保留上膛输出，其余轴立即失能 */
            Mech_Axis_Clear_Fault(Tension_Axis());
            Mech_Axis_Clear_Fault(Yaw_Axis());
            Mech_Axis_Disable(Tension_Axis());
            Mech_Axis_Disable(Yaw_Axis());
        } else {
            Fsm_Axes_Disable_And_Ack();
            s_fsm.output_enabled = false;
        }
        Fault_Clear_Latched();
        s_fsm.manual_release_pending = false;
    } else {
        if (old_mode == DART_MODE_SAFE) {
            /* 回退未完成即离开 SAFE：取消回退，上膛继续完成回 UP */
            s_fsm.retreat_active = false;
            Fsm_Axes_Enable();
            s_fsm.output_enabled = true;
            Fsm_Auto_Zero(now);
        }
        if (old_mode == DART_MODE_SEMI_AUTO) {
            Fsm_Abort();
        }
        if (old_mode == DART_MODE_MANUAL) {
            Fsm_Unload_Abort();
            s_fsm.manual_release_pending = false;
        }
    }
    s_fsm.mode = new_mode;
    (void)Log_Event(LOG_EVT_MODE_CHANGE, (int32_t)new_mode);
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  状态机初始化：SAFE、储能未知、下一发为第 1 发
 */
void Dart_Fsm_Init(void)
{
    s_fsm.mode = DART_MODE_SAFE;
    s_fsm.step = SHOT_IDLE;
    s_fsm.abort_step = SHOT_IDLE;
    s_fsm.step_tick = 0U;
    s_fsm.shot_no = 1U;
    s_fsm.shots_released = 0U;
    s_fsm.energy = ENERGY_UNKNOWN;
    s_fsm.batch_active = false;
    s_fsm.output_enabled = false;
    s_fsm.last_fire_deny = 0U;
    s_fsm.last_start_deny = 0U;
    s_fsm.manual_release_tick = 0U;
    s_fsm.manual_release_pending = false;
    s_fsm.retreat_active = false;
    s_fsm.retreat_tick = 0U;
    s_fsm.unload_step = UNLOAD_IDLE;
    s_fsm.unload_tick = 0U;
    s_fsm.last_unload_deny = 0U;
    s_fsm.auto_zero_done = false;
}

/**
 * @brief  每控制周期调用一次(须在 Rc_Cmd_Update 之后、各模块 Step 之前)
 * @param  now 当前时刻 ms
 */
void Dart_Fsm_Step(uint32_t now)
{
    const rc_cmd_t *rc = Rc_Cmd_Get();

    Fsm_Mode_Change(Fsm_Resolve_Mode(rc), now);
    Fsm_Check_Axis_Fault(now);
    Fsm_Single_Side_Check(now);
    Fsm_Energy_Infer();

    /* 受控回退期间(SAFE 或单侧自动回退)忽略一切遥控与调参操作 */
    if (s_fsm.retreat_active) {
        g_dart_tune.enable = 0U;
        Dart_Tune_Stop();
        Fsm_Safe_Retreat_Run(now);
        return;
    }

    switch (s_fsm.mode) {
        case DART_MODE_MANUAL:
            /* 调参接管时忽略遥控手动操作，拨轮下拨退出调参 */
            if ((s_fsm.unload_step == UNLOAD_IDLE) &&
                Dart_Tune_Run(true, (s_fsm.energy == ENERGY_RELEASED) && !Loader_Is_Single_Side(), now)) {
                if (rc->wheel_evt == RC_WHEEL_EVT_DOWN) {
                    g_dart_tune.enable = 0U;
                    Dart_Tune_Stop();
                }
                break;
            }
            Fsm_Manual_Input(rc, now);
            break;
        case DART_MODE_SEMI_AUTO:
            (void)Dart_Tune_Run(false, false, now);
            Fsm_Semi_Auto_Input(rc, now);
            Fsm_Shot_Run(now);
            break;
        case DART_MODE_AUTO:
            (void)Dart_Tune_Run(false, false, now);
            /* TODO: 裁判协议到手后实现比赛自动流程 */
            break;
        case DART_MODE_SAFE:
        default:
            (void)Dart_Tune_Run(false, false, now);
            break;
    }
}

/**
 * @brief  读取状态机只读信息
 */
const dart_fsm_t *Dart_Fsm_Get(void)
{
    return &s_fsm;
}
