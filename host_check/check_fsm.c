/**
 * @file    check_fsm.c
 * @brief   上位机纯逻辑检查：dart_fsm.c 的模式映射、SAFE 受控回退、人工卸力判定
 * @note    依赖 fakes.c 替代机构模块；结论只表示"逻辑符合预期"，不代表功能或实机验证。
 *          构建运行见同目录 README.md。
 */
#include <stdio.h>
#include "fakes.h"

static int s_fail = 0;
static int s_pass = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (cond) { s_pass++; }                                       \
        else { s_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
    } while (0)

static uint32_t s_now = 1000U;

/* 一个控制周期：设置遥控快照后调用状态机，拨轮事件只保持一个周期 */
static void Tick(rc_lever_e s1, rc_lever_e s2, rc_wheel_evt_e wheel, uint32_t dt)
{
    s_now += dt;
    g_rc.online = true;
    g_rc.safe_seen = true;
    g_rc.s1 = s1;
    g_rc.s2 = s2;
    g_rc.wheel_evt = wheel;
    Dart_Fsm_Step(s_now);
    g_rc.wheel_evt = RC_WHEEL_EVT_NONE;
}

static void Boot(void)
{
    Fakes_Reset();
    Dart_Fsm_Init();
    s_now = 1000U;
    Tick(RC_LEVER_DOWN, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 1U);
}

/* 手动模式 S2 上：拨轮 3 秒内上拨两次，走卸力流程直到完成 */
static void Do_Unload(void)
{
    int guard;

    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 500U);
    for (guard = 0; (guard < 50) && (Dart_Fsm_Get()->unload_step != UNLOAD_IDLE); guard++) {
        Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 100U);
    }
}

/* 半自动 S2 上开始一发，推进到指定步骤后停住(关闭自动到位) */
static void Run_Shot_Until(shot_step_e stop_at)
{
    int guard;

    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    for (guard = 0; (guard < 50) && (Dart_Fsm_Get()->step != stop_at); guard++) {
        Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    }
    g_auto_arrive = false;
}

static void Check_Mode_Mapping(void)
{
    printf("1. 模式映射(上 手动 / 中 半自动 / 下 安全)\n");
    Boot();
    Tick(RC_LEVER_UP, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 1U);
    CHECK(Dart_Fsm_Get()->mode == DART_MODE_MANUAL, "S1 上 应为手动");
    Tick(RC_LEVER_MID, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 1U);
    CHECK(Dart_Fsm_Get()->mode == DART_MODE_SEMI_AUTO, "S1 中 应为半自动");
    Tick(RC_LEVER_DOWN, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 1U);
    CHECK(Dart_Fsm_Get()->mode == DART_MODE_SAFE, "S1 下 应为安全");
}

static void Check_Unload(void)
{
    printf("2. 人工卸力\n");
    Boot();
    CHECK(Dart_Fsm_Get()->energy == ENERGY_UNKNOWN, "上电储能应为 UNKNOWN");

    g_loader_axis.zeroed = false;
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 500U);
    CHECK((Dart_Fsm_Get()->last_unload_deny & UNLOAD_DENY_NOT_ZEROED) != 0U, "未找零应拒绝卸力");
    CHECK(g_trigger_release_cnt == 0, "被拒时不得打开撒放器");

    Boot();
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_IDLE, "单次上拨不应启动卸力");
    CHECK(g_trigger_release_cnt == 0, "单次上拨不得打开撒放器");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 500U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_CATCH, "二次上拨应进入接住第一段(快速到 MID)");
    CHECK(g_loader_last_speed == LOADER_SPEED_FAST, "接住第一段应快速");
    CHECK(g_trigger_release_cnt == 0, "接住前不得打开撒放器");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((Dart_Fsm_Get()->unload_step == UNLOAD_CATCH_SLOW) && (g_loader_last_speed == LOADER_SPEED_SLOW),
          "到 MID 后应慢速到接住位");
    CHECK(g_trigger_release_cnt == 0, "慢速接住完成前不得开锁");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_OPEN, "到接住位后应开锁");
    CHECK((g_trigger_release_cnt == 1) && (g_loader_pos == LOADER_AT_CATCH), "开锁时上膛应在接住位");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 100U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_OPEN, "开锁等待时间未到不应送回");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 600U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_RETURN, "等待后应限速送回");
    CHECK(g_loader_last_speed == LOADER_SPEED_RETREAT, "送回应使用回退限速");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((Dart_Fsm_Get()->unload_step == UNLOAD_IDLE) &&
          (Dart_Fsm_Get()->energy == ENERGY_RELEASED), "送回 UP 后储能应为 RELEASED");

    Boot();
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 500U);
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_DOWN, 10U);
    CHECK(Dart_Fsm_Get()->unload_step == UNLOAD_IDLE, "卸力中拨轮下拨应中止");
}

static void Check_Retreat(void)
{
    printf("3. SAFE 受控回退\n");

    /* 3a 下拉途中拨到 SAFE：回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_DOWN);
    CHECK(Dart_Fsm_Get()->step == SHOT_LOAD_DOWN, "应停在 LOAD_DOWN");
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(Dart_Fsm_Get()->retreat_active, "下拉途中进入 SAFE 应回退");
    CHECK(Dart_Fsm_Get()->output_enabled, "回退期间应保持输出");
    CHECK(g_loader_last_speed == LOADER_SPEED_RETREAT, "回退应使用回退限速");
    CHECK(Dart_Fsm_Get()->abort_step == SHOT_LOAD_MID, "恢复入口应退回 LOAD_MID");
    g_auto_arrive = true;
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(!Dart_Fsm_Get()->retreat_active && !Dart_Fsm_Get()->output_enabled, "到 UP 后应结束并清零");

    /* 3b 遥控离线同样回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_DOWN);
    g_rc.online = false;
    s_now += 10U;
    Dart_Fsm_Step(s_now);
    CHECK(Dart_Fsm_Get()->retreat_active, "下拉途中遥控离线应回退");

    /* 3c 已锁止(LOAD_RETURN 途中)：不回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_RETURN);
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(!Dart_Fsm_Get()->retreat_active && !Dart_Fsm_Get()->output_enabled, "已锁止时不应回退");

    /* 3d 上膛故障：不回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_DOWN);
    g_loader_axis.status = MECH_AXIS_FAULT;
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(!Dart_Fsm_Get()->retreat_active && !Dart_Fsm_Get()->output_enabled, "上膛故障时不应回退");

    /* 3e LOCK 中止 -> 回退 -> 恢复被拒 */
    Boot();
    Do_Unload();
    g_trigger_lock_instant = false;
    Run_Shot_Until(SHOT_LOCK);
    g_auto_arrive = true;
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_DOWN, 10U);
    CHECK((Dart_Fsm_Get()->step == SHOT_ABORTED) && (Dart_Fsm_Get()->energy == ENERGY_UNKNOWN),
          "LOCK 中止后储能应为 UNKNOWN");
    g_auto_arrive = false;   /* 避免回退在同一周期内完成 */
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(Dart_Fsm_Get()->retreat_active, "LOCK 中止进入 SAFE 应回退");
    CHECK(Dart_Fsm_Get()->abort_step == SHOT_LOAD_MID, "LOCK 回退后恢复入口应为 LOAD_MID");
    g_auto_arrive = true;
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    CHECK((Dart_Fsm_Get()->step == SHOT_ABORTED) &&
          ((Dart_Fsm_Get()->last_start_deny & START_DENY_ENERGY_UNKNOWN) != 0U),
          "LOCK 中止回退后恢复应被拒");
}

static void Check_Direct_Release(void)
{
    int guard;

    printf("4. 确认无储能时直接切换撒放器\n");
    Boot();
    Do_Unload();
    /* 打完一整发：结束后撒放器锁止、储能 RELEASED */
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    for (guard = 0; (guard < 50) && (Dart_Fsm_Get()->step != SHOT_ARMED); guard++) {
        Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    }
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    for (guard = 0; (guard < 50) && (Dart_Fsm_Get()->step != SHOT_IDLE); guard++) {
        Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 1000U);
    }
    CHECK((Dart_Fsm_Get()->step == SHOT_IDLE) && (g_trigger == TRIGGER_LOCKED) &&
          (Dart_Fsm_Get()->energy == ENERGY_RELEASED), "一发结束应锁止且 RELEASED");
    CHECK((g_link_sync_cnt == 1) && (g_link_sync_shot == 1U), "释放时应发出 1 次出镖同步，发序为 1");
    g_trigger_release_cnt = 0;
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    CHECK((g_trigger_release_cnt == 1) && (Dart_Fsm_Get()->unload_step == UNLOAD_IDLE),
          "RELEASED 时单次上拨应直接打开，不走卸力");
}

static void Check_Auto_Zero_And_Tone(void)
{
    printf("5. 首次离开安全位自动找零、提示音\n");
    Boot();
    g_loader_axis.zeroed = false;
    Yaw_Axis()->zeroed = false;
    Tick(RC_LEVER_UP, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_zero_req_cnt[0] == 1) && (g_zero_req_cnt[2] == 1), "首次离开安全位应对未找零轴找零");
    CHECK(g_zero_req_cnt[1] == 0, "已找零的轴不应重复找零");
    Tick(RC_LEVER_DOWN, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 10U);
    Tick(RC_LEVER_UP, RC_LEVER_MID, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_zero_req_cnt[0] == 1) && (g_zero_req_cnt[2] == 1), "第二次离开安全位不应再找零");

    /* 上电储能 UNKNOWN 时启动单发 -> 被拒提示 */
    Boot();
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, 10U);
    CHECK((Dart_Fsm_Get()->last_start_deny != 0U) && (g_tone_cnt[BUZZER_TONE_DENY] == 1),
          "启动被拒应有 DENY 提示音");

    /* 正常推进到 ARMED -> ARMED 提示音(释放被拒由 fire_gate 决定，此处假实现不检查) */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_ARMED);
    CHECK((Dart_Fsm_Get()->step == SHOT_ARMED) && (g_tone_cnt[BUZZER_TONE_ARMED] == 1),
          "进入待发应有 ARMED 提示音");
}

static void Check_Tune(void)
{
    printf("6. PID 调参接口\n");
    Boot();
    g_dart_tune.enable = 1U;
    g_dart_tune.axis = TUNE_AXIS_YAW;
    g_dart_tune.mode = TUNE_MODE_SPEED;
    g_dart_tune.speed_rpm = 300.0f;
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_dart_tune.enable == 0U) && ((g_dart_tune.deny & TUNE_DENY_NOT_MANUAL) != 0U),
          "非手动模式写入 enable 应被拒并清除");

    g_dart_tune.enable = 1U;
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_dart_tune.active == 1U) && (Yaw_Axis()->status == MECH_AXIS_TUNE_SPEED) &&
          (Yaw_Axis()->tune_speed == 300.0f), "手动模式下应接管 Yaw 速度环");
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_DOWN, 10U);
    CHECK((g_dart_tune.active == 0U) && (Yaw_Axis()->status == MECH_AXIS_HOLD),
          "拨轮下拨应退出调参并原地保持");

    g_dart_tune.enable = 1U;
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_dart_tune.active == 0U) && (g_dart_tune.enable == 0U), "拨回安全位应退出调参");

    /* 上膛：储能未确认时拒绝 */
    Boot();
    g_dart_tune.enable = 1U;
    g_dart_tune.axis = TUNE_AXIS_LOADER;
    g_dart_tune.mode = TUNE_MODE_SPEED;
    Tick(RC_LEVER_UP, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK((g_dart_tune.deny & TUNE_DENY_ENERGY) != 0U, "储能未确认时不允许调上膛");
    g_dart_tune.axis = TUNE_AXIS_YAW;
}

static void Check_Single_Side(void)
{
    printf("7. 上膛单侧离线(策略 A)\n");

    /* 7a 下拉途中 R 离线：中止流程，原地保持，超时后自动单侧回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_DOWN);
    g_single_side = true;
    g_single_side_tick = s_now;
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(Dart_Fsm_Get()->step == SHOT_ABORTED, "单侧离线应中止流程");
    CHECK(!Dart_Fsm_Get()->retreat_active, "保持时限内不应回退");
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_UP, LOADER_SINGLE_HOLD_MAX_MS);
    CHECK(Dart_Fsm_Get()->retreat_active, "超过保持时限应自动回退");
    CHECK(g_loader_last_speed == LOADER_SPEED_RETREAT_SINGLE, "单侧回退应使用单侧限速");
    CHECK(Dart_Fsm_Get()->step == SHOT_ABORTED, "回退期间拨轮上拨不应恢复流程");
    g_auto_arrive = true;
    Tick(RC_LEVER_MID, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(!Dart_Fsm_Get()->retreat_active && Dart_Fsm_Get()->output_enabled,
          "非安全位回退完成后应原地保持，不清零输出");

    /* 7b 单侧离线时拨到 SAFE：立即单侧回退 */
    Boot();
    Do_Unload();
    Run_Shot_Until(SHOT_LOAD_DOWN);
    g_single_side = true;
    g_single_side_tick = s_now;
    Tick(RC_LEVER_DOWN, RC_LEVER_UP, RC_WHEEL_EVT_NONE, 10U);
    CHECK(Dart_Fsm_Get()->retreat_active && (g_loader_last_speed == LOADER_SPEED_RETREAT_SINGLE),
          "单侧离线进入 SAFE 应立即单侧回退");
}

int main(void)
{
    Check_Mode_Mapping();
    Check_Unload();
    Check_Retreat();
    Check_Direct_Release();
    Check_Auto_Zero_And_Tone();
    Check_Tune();
    Check_Single_Side();
    printf("\n结果：通过 %d，失败 %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
