/**
 ******************************************************************************
 * @file    rc_cmd.c
 * @author  RobotPilots 镖架电控
 * @brief   遥控命令快照生成
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  模板 rc_sensor_check 在数据异常时会把 s1/s2 改成中位，
 *  本模块以 errno==DEV_DATA_ERR 判为离线，避免因此误入 MANUAL。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rc_cmd.h"
#include "rc_sensor.h"
#include "cmsis_os.h"

/* Private types -------------------------------------------------------------*/
/**
 * @brief  拨轮内部状态
 */
typedef enum {
    WHEEL_BLOCKED = 0,   /* 未回中，禁止触发 */
    WHEEL_CENTER,        /* 已回中，可触发 */
    WHEEL_HELD,          /* 已触发，等待回中 */
} wheel_state_e;

/* Private variables ---------------------------------------------------------*/
static rc_cmd_t s_rc_cmd;
static wheel_state_e s_wheel_state;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  DBUS 拨杆值 1/3/2 -> 枚举
 */
static rc_lever_e Rc_Cmd_Lever(uint8_t raw)
{
    switch (raw) {
        case RC_SW_UP:
            return RC_LEVER_UP;
        case RC_SW_MID:
            return RC_LEVER_MID;
        case RC_SW_DOWN:
            return RC_LEVER_DOWN;
        default:
            return RC_LEVER_INVALID;
    }
}

/**
 * @brief  摇杆原始值 -> -1~1，死区内为 0
 */
static float Rc_Cmd_Stick(int16_t raw)
{
    if ((raw < RC_STICK_DEADBAND) && (raw > -RC_STICK_DEADBAND)) {
        return 0.0f;
    }
    return Dart_Clamp_F((float)raw / RC_STICK_FULL, -1.0f, 1.0f);
}

/**
 * @brief  拨轮边沿检测：CENTER 越过阈值触发一次，回中后才能再次触发
 * @param  value 拨轮原始值(已减 1024)
 */
static rc_wheel_evt_e Rc_Cmd_Wheel(int16_t value)
{
    int16_t up_value = (int16_t)(value * RC_WHEEL_UP_SIGN);   /* 上拨为正 */
    bool centered = (value < RC_WHEEL_RELEASE_THRESH) && (value > -RC_WHEEL_RELEASE_THRESH);

    switch (s_wheel_state) {
        case WHEEL_CENTER:
            if (up_value >= RC_WHEEL_TRIG_THRESH) {
                s_wheel_state = WHEEL_HELD;
                return RC_WHEEL_EVT_UP;
            }
            if (up_value <= -RC_WHEEL_TRIG_THRESH) {
                s_wheel_state = WHEEL_HELD;
                return RC_WHEEL_EVT_DOWN;
            }
            break;
        case WHEEL_BLOCKED:
        case WHEEL_HELD:
        default:
            if (centered) {
                s_wheel_state = WHEEL_CENTER;
            }
            break;
    }
    return RC_WHEEL_EVT_NONE;
}

/**
 * @brief  离线时清空快照
 */
static void Rc_Cmd_Reset(void)
{
    s_rc_cmd.online = false;
    s_rc_cmd.safe_seen = false;
    s_rc_cmd.s1 = RC_LEVER_INVALID;
    s_rc_cmd.s2 = RC_LEVER_INVALID;
    s_rc_cmd.ch0 = 0.0f;
    s_rc_cmd.ch1 = 0.0f;
    s_rc_cmd.ch2 = 0.0f;
    s_rc_cmd.ch3 = 0.0f;
    s_rc_cmd.wheel_evt = RC_WHEEL_EVT_NONE;
    s_rc_cmd.fire_confirm = false;
    s_wheel_state = WHEEL_BLOCKED;
}

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化为离线
 */
void Rc_Cmd_Init(void)
{
    Rc_Cmd_Reset();
}

/**
 * @brief  生成本周期命令快照
 * @param  now 当前时刻 ms(预留)
 */
void Rc_Cmd_Update(uint32_t now)
{
    bool online;
    uint8_t s1_raw;
    uint8_t s2_raw;
    int16_t ch_raw[4];
    int16_t wheel_raw;

    (void)now;
    taskENTER_CRITICAL();
    online = (rc_sensor.work_state == DEV_ONLINE) && (rc_sensor.errno == NONE_ERR);
    s1_raw = rc_sensor_info.s1.value;
    s2_raw = rc_sensor_info.s2.value;
    ch_raw[0] = rc_sensor_info.ch0;
    ch_raw[1] = rc_sensor_info.ch1;
    ch_raw[2] = rc_sensor_info.ch2;
    ch_raw[3] = rc_sensor_info.ch3;
    wheel_raw = rc_sensor_info.thumbwheel.value;
    taskEXIT_CRITICAL();

    if (!online) {
        Rc_Cmd_Reset();
        return;
    }

    s_rc_cmd.online = true;
    s_rc_cmd.s1 = Rc_Cmd_Lever(s1_raw);
    s_rc_cmd.s2 = Rc_Cmd_Lever(s2_raw);
    s_rc_cmd.ch0 = Rc_Cmd_Stick(ch_raw[0]);
    s_rc_cmd.ch1 = Rc_Cmd_Stick(ch_raw[1]);
    s_rc_cmd.ch2 = Rc_Cmd_Stick(ch_raw[2]);
    s_rc_cmd.ch3 = Rc_Cmd_Stick(ch_raw[3]);
    s_rc_cmd.wheel_evt = Rc_Cmd_Wheel(wheel_raw);
    s_rc_cmd.fire_confirm = (ch_raw[1] >= RC_STICK_CONFIRM_THRESH);

    if (s_rc_cmd.s1 == RC_LEVER_DOWN) {
        s_rc_cmd.safe_seen = true;
    }
    if ((s_rc_cmd.s1 == RC_LEVER_INVALID) || (s_rc_cmd.s2 == RC_LEVER_INVALID)) {
        s_rc_cmd.safe_seen = false;
    }
}

/**
 * @brief  读取本周期快照
 */
const rc_cmd_t *Rc_Cmd_Get(void)
{
    return &s_rc_cmd;
}
