/**
 * @file    check_axis.c
 * @brief   上位机纯逻辑检查：mech_axis.c 的超时计算、软限位、保持卡死、点动超前、离线覆盖
 * @note    电机反馈与 PID 用本文件内的假实现；结论只表示"逻辑符合预期"。
 */
#include <stdio.h>
#include "mech_axis.h"

static int s_fail = 0;
static int s_pass = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (cond) { s_pass++; }                                       \
        else { s_fail++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
    } while (0)

/* ---------- 假电机：位置/转速由测试直接设定 ---------- */
static int32_t s_enc = 0;
static float   s_rpm = 0.0f;
static bool    s_online = true;
static float   s_out = 0.0f;

int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id) { (void)id; return s_enc; }
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id) { (void)id; return s_rpm; }
bool Dart_Motor_Is_Online(dart_motor_id_e id) { (void)id; return s_online; }
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw) { (void)id; s_out = out_raw; }

/* 与固件 PID.c 相同的计算 */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    pid->integral += pid->err;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
    pid->last_dout = pid->dout;
    pid->dout = pid->kd * (pid->err - pid->last_err);
    pid->out = pid->pout + pid->iout + pid->dout;
    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    pid->last_err = pid->err;
}

/* 仿上膛参数的测试配置 */
static const mech_axis_cfg_t s_cfg = {
    .motor_id = DART_MOTOR_LOADER_L, .dir_sign = 1, .out_max = 16000.0f, .speed_max = 7500.0f,
    .pos_min = 0, .pos_max = 1360000,
    .zero_mode = MECH_AXIS_ZERO_STALL, .zero_speed = -1000.0f, .zero_out_max = 4000.0f,
    .zero_spd_kp = 1.0f, .zero_spd_ki = 0.1f, .zero_spd_imax = 10000.0f, .zero_timeout_ms = 8000U,
    .home_after_zero = true, .home_pos = 20000,
    .arrive_err = 400.0f, .arrive_speed = 20.0f, .arrive_hold_ms = 20U,
    .move_timeout_ms = 6000U, .hold_stall_err = 20000, .jog_lead_max = 50000,
    .stall_check_in_move = true, .stall_out_min = 1000.0f, .stall_speed_max = 10.0f, .stall_hold_ms = 100U,
    .hold_on_fault = true, .pos_kp = 0.4f, .spd_kp = 18.0f, .spd_ki = 0.45f, .spd_imax = 20000.0f,
};

static mech_axis_t s_axis;
static uint32_t s_now = 1000U;

static void Step_N(uint32_t n)
{
    uint32_t i;
    for (i = 0U; i < n; i++) {
        s_now++;
        Mech_Axis_Step(&s_axis, s_now);
    }
}

/* 模拟按恒定速度运动：每 ms 位置变化 delta，反馈转速 rpm */
static void Step_Moving(uint32_t n, int32_t delta, float rpm)
{
    uint32_t i;
    s_rpm = rpm;
    for (i = 0U; i < n; i++) {
        s_enc += delta;
        s_now++;
        Mech_Axis_Step(&s_axis, s_now);
    }
}

/* 初始化并直接置为已找零、保持在 pos */
static void Setup(int32_t pos)
{
    s_enc = pos;
    s_rpm = 0.0f;
    s_online = true;
    Mech_Axis_Init(&s_axis, &s_cfg);
    Step_N(1U);
    Mech_Axis_Enable(&s_axis);
    s_axis.zeroed = true;
    s_axis.zero_ref = 0;
    Step_N(1U);
}

int main(void)
{
    uint32_t t;

    printf("1. 运动超时按距离计算\n");
    Setup(1340000);
    t = Mech_Axis_Move_Timeout(&s_axis, 20000 - 1340000, 800.0f);
    CHECK((t > 19000U) && (t < 19300U), "DOWN->UP 800rpm 超时应约 19.1s(理论 12.1s x1.5 + 1s)");
    t = Mech_Axis_Move_Timeout(&s_axis, 1000, 7500.0f);
    CHECK(t == 6000U, "短距离应取配置下限 6s");
    CHECK(Mech_Axis_Request_Move(&s_axis, 20000, 800.0f, s_now), "请求运动应被接受");
    CHECK(s_axis.move_timeout == Mech_Axis_Move_Timeout(&s_axis, 20000 - 1340000, 800.0f),
          "请求运动时应写入按距离计算的超时");
    Step_Moving(7000U, -50, -400.0f);       /* 正常运动但较慢 */
    CHECK(s_axis.status == MECH_AXIS_MOVING, "7s 时不应报运动超时(旧逻辑 6s 即超时)");
    Step_Moving(13000U, 0, -400.0f);        /* 原地"转动"不前进，直到超过 19.1s */
    CHECK((s_axis.status == MECH_AXIS_FAULT) && (s_axis.err == MECH_AXIS_ERR_MOVE_TIMEOUT),
          "超过按距离计算的超时后应报运动超时");

    printf("2. RAM 软限位可在线放宽\n");
    Setup(0);
    CHECK(!Mech_Axis_Request_Move(&s_axis, 1500000, 0.0f, s_now), "超出配置上限应被拒");
    s_axis.pos_max = 2000000;
    CHECK(Mech_Axis_Request_Move(&s_axis, 1500000, 0.0f, s_now), "放宽 RAM 上限后应接受");

    printf("3. 点动超前限制\n");
    Setup(100000);
    Mech_Axis_Jog(&s_axis, 1000000);
    CHECK(s_axis.target == 100000 + 50000, "点动目标最多超前 jog_lead_max");

    printf("4. 保持状态卡死判定\n");
    Setup(100000);
    Mech_Axis_Jog(&s_axis, 40000);          /* 目标超前 40000 > hold_stall_err */
    Step_N(300U);                            /* 编码器与转速不变 = 顶死 */
    CHECK((s_axis.status == MECH_AXIS_FAULT) && (s_axis.err == MECH_AXIS_ERR_STALL),
          "点动顶死应判为堵转故障");
    Setup(100000);
    Mech_Axis_Jog(&s_axis, 5000);           /* 误差 5000 < hold_stall_err：正常承载保持 */
    Step_N(300U);
    CHECK(s_axis.status == MECH_AXIS_HOLD, "小误差保持不应误判卡死");

    printf("5. 离线覆盖已有故障\n");
    Setup(100000);
    Mech_Axis_Force_Fault(&s_axis, MECH_AXIS_ERR_MOVE_TIMEOUT);
    Step_N(1U);
    CHECK(s_out != 0.0f || s_axis.err == MECH_AXIS_ERR_MOVE_TIMEOUT, "超时故障时上膛保持(hold_on_fault)");
    Mech_Axis_Force_Fault(&s_axis, MECH_AXIS_ERR_OFFLINE);
    Step_N(1U);
    CHECK((s_axis.err == MECH_AXIS_ERR_OFFLINE) && (s_out == 0.0f), "离线应覆盖超时故障并停止输出");

    printf("\n结果：通过 %d，失败 %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
