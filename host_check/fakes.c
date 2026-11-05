/**
 * @file    fakes.c
 * @brief   host_check 假模块：替代 dart_fsm.c 依赖的机构/设备/协议接口
 * @note    只模拟状态机判断所需的最小行为，不模拟物理过程与时序。
 *          位置用 loader_pos_e 粗粒度表示；g_auto_arrive=true 时请求的目标立即到位。
 */
#include "fakes.h"

/* ---------- 可控假状态 ---------- */
rc_cmd_t       g_rc;
mech_axis_t    g_loader_axis;
mech_axis_t    g_tension_axis;
mech_axis_t    g_yaw_axis;
int            g_loader_pos = LOADER_AT_UP;      /* 当前位置 */
int            g_loader_target = LOADER_AT_UP;   /* 请求目标 */
float          g_loader_last_speed = 0.0f;       /* 最近请求限速 */
bool           g_auto_arrive = true;             /* 目标立即到位 */
trigger_state_e g_trigger = TRIGGER_LOCKED;
bool           g_trigger_lock_instant = true;    /* 锁止立即完成 */
bool           g_motor_online = true;
bool           g_single_side = false;            /* 上膛仅 R 离线 */
uint32_t       g_single_side_tick = 0U;
uint32_t       g_fault_mask = 0U;
feeder_status_e g_feeder = FEEDER_IDLE;
int            g_trigger_release_cnt = 0;        /* 撒放器释放请求次数 */
int            g_zero_req_cnt[3];
int            g_tone_cnt[BUZZER_TONE_NUM];
int            g_link_sync_cnt = 0;
uint8_t        g_link_sync_shot = 0U;

void Fakes_Reset(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    memset(&g_loader_axis, 0, sizeof(g_loader_axis));
    memset(&g_tension_axis, 0, sizeof(g_tension_axis));
    memset(&g_yaw_axis, 0, sizeof(g_yaw_axis));
    g_loader_axis.zeroed = true;
    g_tension_axis.zeroed = true;
    g_yaw_axis.zeroed = true;
    g_loader_axis.status = MECH_AXIS_DISABLED;
    g_tension_axis.status = MECH_AXIS_DISABLED;
    g_yaw_axis.status = MECH_AXIS_DISABLED;
    g_loader_pos = LOADER_AT_UP;
    g_loader_target = LOADER_AT_UP;
    g_loader_last_speed = 0.0f;
    g_auto_arrive = true;
    g_trigger = TRIGGER_LOCKED;
    g_trigger_lock_instant = true;
    g_motor_online = true;
    g_single_side = false;
    g_single_side_tick = 0U;
    g_fault_mask = 0U;
    g_feeder = FEEDER_IDLE;
    g_trigger_release_cnt = 0;
    memset(g_zero_req_cnt, 0, sizeof(g_zero_req_cnt));
    memset(g_tone_cnt, 0, sizeof(g_tone_cnt));
    g_link_sync_cnt = 0;
    g_link_sync_shot = 0U;
}

/* ---------- 遥控 ---------- */
const rc_cmd_t *Rc_Cmd_Get(void) { return &g_rc; }

/* ---------- 电机 ---------- */
bool Dart_Motor_Is_Online(dart_motor_id_e id) { (void)id; return g_motor_online; }
bool Dart_Motor_All_Online(void) { return g_motor_online; }

/* ---------- 通用轴 ---------- */
static bool Axis_Can_Move(const mech_axis_t *axis)
{
    return axis->zeroed && (axis->status != MECH_AXIS_DISABLED) &&
           (axis->status != MECH_AXIS_FAULT) && (axis->status != MECH_AXIS_ZEROING);
}
void Mech_Axis_Enable(mech_axis_t *axis) { if (axis->status == MECH_AXIS_DISABLED) axis->status = MECH_AXIS_HOLD; }
void Mech_Axis_Disable(mech_axis_t *axis) { if (axis->status != MECH_AXIS_FAULT) axis->status = MECH_AXIS_DISABLED; }
void Mech_Axis_Clear_Fault(mech_axis_t *axis) { if (axis->status == MECH_AXIS_FAULT) axis->status = MECH_AXIS_DISABLED; }
void Mech_Axis_Hold(mech_axis_t *axis)
{
    if ((axis->status == MECH_AXIS_DISABLED) || (axis->status == MECH_AXIS_FAULT)) return;
    axis->status = MECH_AXIS_HOLD;
    if (axis == &g_loader_axis) g_loader_target = g_loader_pos;
}
bool Mech_Axis_Request_Zero(mech_axis_t *axis, uint32_t now)
{
    (void)now;
    if ((axis->status == MECH_AXIS_DISABLED) || (axis->status == MECH_AXIS_FAULT)) return false;
    g_zero_req_cnt[(axis == &g_loader_axis) ? 0 : ((axis == &g_tension_axis) ? 1 : 2)]++;
    axis->status = MECH_AXIS_ZEROING;
    return true;
}
void Mech_Axis_Jog(mech_axis_t *axis, int32_t delta) { (void)axis; (void)delta; }
bool Mech_Axis_Tune_Speed(mech_axis_t *axis, float speed_rpm)
{
    if (!Axis_Can_Move(axis)) return false;
    axis->tune_speed = speed_rpm;
    axis->status = MECH_AXIS_TUNE_SPEED;
    return true;
}
bool Mech_Axis_Is_Ready(const mech_axis_t *axis) { return axis->zeroed && g_auto_arrive; }
bool Mech_Axis_Request_Move(mech_axis_t *axis, int32_t target, float speed_limit, uint32_t now)
{
    (void)now;
    if (!Axis_Can_Move(axis)) return false;
    axis->status = MECH_AXIS_MOVING;
    axis->target = target;
    if (axis == &g_loader_axis) {
        g_loader_last_speed = speed_limit;
        g_loader_target = (target == LOADER_POS_UP) ? LOADER_AT_UP :
                          (target == LOADER_POS_MID) ? LOADER_AT_MID : LOADER_AT_DOWN;
    }
    return true;
}

/* ---------- 上膛 ---------- */
mech_axis_t *Loader_Axis(void) { return &g_loader_axis; }
bool Loader_Request_Pos(loader_pos_e pos, uint32_t now)
{
    static const int32_t pos_tab[] = { LOADER_POS_UP, LOADER_POS_MID, LOADER_POS_DOWN, LOADER_POS_CATCH };
    static const float spd_tab[] = { LOADER_SPEED_FAST, LOADER_SPEED_FAST, LOADER_SPEED_SLOW, LOADER_SPEED_SLOW };
    if (!Mech_Axis_Request_Move(&g_loader_axis, pos_tab[pos], spd_tab[pos], now)) return false;
    g_loader_target = (int)pos;   /* 按预设编号记录，CATCH 与 DOWN 数值相同也能区分 */
    return true;
}
bool Loader_Is_At(loader_pos_e pos) { return g_loader_axis.zeroed && (g_loader_pos == (int)pos); }
bool Loader_Is_Single_Side(void) { return g_single_side; }
uint32_t Loader_Single_Side_Tick(void) { return g_single_side_tick; }
bool Loader_Is_Ready_At(loader_pos_e pos)
{
    if (g_auto_arrive && (g_loader_target == (int)pos) &&
        ((g_loader_axis.status == MECH_AXIS_MOVING) || (g_loader_axis.status == MECH_AXIS_ARRIVED))) {
        g_loader_pos = (int)pos;
        g_loader_axis.status = MECH_AXIS_ARRIVED;
    }
    return (g_loader_axis.status == MECH_AXIS_ARRIVED) && (g_loader_pos == (int)pos) &&
           (g_loader_target == (int)pos);
}

/* ---------- 变力 / Yaw ---------- */
mech_axis_t *Tension_Axis(void) { return &g_tension_axis; }
bool Tension_Request_Shot(uint8_t shot_no, uint32_t now) { (void)shot_no; (void)now; return g_tension_axis.zeroed; }
bool Tension_Is_Ready(void) { return g_auto_arrive; }
mech_axis_t *Yaw_Axis(void) { return &g_yaw_axis; }
bool Yaw_Request_Shot(uint8_t shot_no, uint32_t now) { (void)shot_no; (void)now; return g_yaw_axis.zeroed; }
bool Yaw_Is_Ready(void) { return g_auto_arrive; }
void Yaw_Jog_Deg(float delta_deg) { (void)delta_deg; }

/* ---------- 撒放器 ---------- */
trigger_state_e Trigger_Get_State(void) { return g_trigger; }
void Trigger_Request_Lock(uint32_t now)
{
    (void)now;
    if ((g_trigger == TRIGGER_LOCKED) || (g_trigger == TRIGGER_LOCKING)) return;
    g_trigger = g_trigger_lock_instant ? TRIGGER_LOCKED : TRIGGER_LOCKING;
}
void Trigger_Request_Release(uint32_t now)
{
    (void)now;
    if ((g_trigger == TRIGGER_RELEASED) || (g_trigger == TRIGGER_RELEASING)) return;
    g_trigger = TRIGGER_RELEASED;
    g_trigger_release_cnt++;
}

/* ---------- 供弹 ---------- */
void Feeder_Ack(void) { if ((g_feeder == FEEDER_DONE) || (g_feeder == FEEDER_ABORTED)) g_feeder = FEEDER_IDLE; }
bool Feeder_Request_Shot(uint8_t shot_no, uint32_t now) { (void)shot_no; (void)now; g_feeder = FEEDER_DONE; return true; }
bool Feeder_Shot_Needs_Feed(uint8_t shot_no) { return shot_no > 1U; }
bool Feeder_Request_Home(uint32_t now) { (void)now; return true; }
void Feeder_Abort(void) { if (g_feeder == FEEDER_RUNNING) g_feeder = FEEDER_ABORTED; }
feeder_status_e Feeder_Get_Status(void) { return g_feeder; }

/* ---------- 故障 / 许可 / 日志 ---------- */
void Fault_Set(fault_code_e code) { g_fault_mask |= (1UL << code); }
void Fault_Clear_Latched(void) { g_fault_mask &= ~FAULT_LATCHED_MASK; }
uint32_t Fault_Get_Mask(void) { return g_fault_mask; }
fire_gate_result_t Fire_Gate_Check(bool mode_ok, bool confirm_ok)
{
    fire_gate_result_t r;
    r.allow = mode_ok && confirm_ok;
    r.deny_mask = r.allow ? 0U : 1U;
    return r;
}
bool Log_Event(log_event_e code, int32_t arg) { (void)code; (void)arg; return false; }

/* ---------- 2.4G ---------- */
void Link24g_Notify_Shot(uint8_t shot_no) { g_link_sync_cnt++; g_link_sync_shot = shot_no; }
void Buzzer_Play(buzzer_tone_e tone) { if (tone < BUZZER_TONE_NUM) g_tone_cnt[tone]++; }
