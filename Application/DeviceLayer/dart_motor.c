/**
 ******************************************************************************
 * @file    dart_motor.c
 * @author  RobotPilots 镖架电控
 * @brief   镖架 RM 电机实例与 CAN 帧发送
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  1. 只有 ControlTask 调用 Set_Output/Send，保证输出单一写者。
 *  2. 读取 encoder_sum 时关中断，避免与 CAN 中断撕裂读取。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "dart_motor.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static Motor_RM_Born_Info_t s_born[DART_MOTOR_NUM] = {
    [DART_MOTOR_LOADER_L] = {
        .order_correction = 1,
        .rxId  = (uint8_t)(DART_CAN_ID_LOADER_L - 0x201U),
        .stdId = DART_CAN_TX_ID_GROUP_LOW,
        .type  = _3508_Single,
        .hcan  = &hcan1,
    },
    [DART_MOTOR_LOADER_R] = {
        .order_correction = 1,
        .rxId  = (uint8_t)(DART_CAN_ID_LOADER_R - 0x201U),
        .stdId = DART_CAN_TX_ID_GROUP_LOW,
        .type  = _3508_Single,
        .hcan  = &hcan1,
    },
    [DART_MOTOR_TENSION] = {
        .order_correction = 1,
        .rxId  = (uint8_t)(DART_CAN_ID_TENSION - 0x201U),
        .stdId = DART_CAN_TX_ID_GROUP_LOW,
        .type  = _3508_Single,
        .hcan  = &hcan1,
    },
    [DART_MOTOR_YAW] = {
        .order_correction = 1,
        .rxId  = (uint8_t)(DART_CAN_ID_YAW - 0x201U),
        .stdId = DART_CAN_TX_ID_GROUP_LOW,
        .type  = _2006_Single,
        .hcan  = &hcan2,
    },
};

static Motor_RM_Rx_Info_t   s_rx[DART_MOTOR_NUM];
static Motor_RM_Tx_Info_t   s_tx[DART_MOTOR_NUM];
static Motor_RM_State_t     s_state[DART_MOTOR_NUM];
static Motor_RM_Ctrl_Info_t s_ctrl[DART_MOTOR_NUM];  /* 未使用模板内置闭环 */

/* CAN1 0x200 组：L/R/变力；CAN2 0x200 组：Yaw */
static Motor_RM_Group_t s_group_can1;
static Motor_RM_Group_t s_group_can2;

/* 各电机输出限幅 raw */
static const float s_out_max[DART_MOTOR_NUM] = {
    DART_3508_OUT_MAX,
    DART_3508_OUT_MAX,
    DART_3508_OUT_MAX,
    DART_2006_OUT_MAX,
};

/* Exported variables --------------------------------------------------------*/
/* 全局电机对象：CAN 中断写反馈，ControlTask 写输出，其余只读 */
Motor_RM_t g_dart_motor[DART_MOTOR_NUM];

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  初始化 4 台电机对象与两个 CAN 发送组
 * @param  无
 * @retval 无
 * @note   在 CAN 过滤器启动前调用，避免中断访问未初始化对象
 */
void Dart_Motor_Init(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < (uint8_t)DART_MOTOR_NUM; motor_index++) {
        Motor_RM_t *motor = &g_dart_motor[motor_index];

        motor->born_info = &s_born[motor_index];
        motor->rx_info   = &s_rx[motor_index];
        motor->tx_info   = &s_tx[motor_index];
        motor->state     = &s_state[motor_index];
        motor->ctrl      = &s_ctrl[motor_index];
    }

    s_group_can1.motor[0] = &g_dart_motor[DART_MOTOR_LOADER_L];
    s_group_can1.motor[1] = &g_dart_motor[DART_MOTOR_LOADER_R];
    s_group_can1.motor[2] = &g_dart_motor[DART_MOTOR_TENSION];
    s_group_can1.motor[3] = NULL;
    s_group_can1.stdId    = DART_CAN_TX_ID_GROUP_LOW;
    s_group_can1.hcan     = &hcan1;

    s_group_can2.motor[0] = NULL;
    s_group_can2.motor[1] = NULL;
    s_group_can2.motor[2] = NULL;
    s_group_can2.motor[3] = &g_dart_motor[DART_MOTOR_YAW];
    s_group_can2.stdId    = DART_CAN_TX_ID_GROUP_LOW;
    s_group_can2.hcan     = &hcan2;

    /* 内部会调用 RM_Motor_Init 并把离线阈值写死为 100 */
    RM_Group_Motor_Init(&s_group_can1);
    RM_Group_Motor_Init(&s_group_can2);

    /* NOTE: 初始化后覆盖阈值，并置为离线，收到首帧才转在线 */
    for (motor_index = 0U; motor_index < (uint8_t)DART_MOTOR_NUM; motor_index++) {
        s_state[motor_index].offline_cnt_max = DART_MOTOR_OFFLINE_CNT;
        s_state[motor_index].offline_cnt     = DART_MOTOR_OFFLINE_CNT + 1U;
        s_state[motor_index].status          = DEV_OFFLINE;
        s_tx[motor_index].torque             = 0.0f;
    }
}

/**
 * @brief  电机心跳，MonitorTask 每 1ms 调用
 * @param  无
 * @retval 无
 */
void Dart_Motor_Heart_Beat(void)
{
    s_group_can1.group_heartbeat(&s_group_can1);
    s_group_can2.group_heartbeat(&s_group_can2);
}

/**
 * @brief  查询单台电机是否在线
 * @param  id 电机编号
 * @retval true 在线
 */
bool Dart_Motor_Is_Online(dart_motor_id_e id)
{
    if (id >= DART_MOTOR_NUM) {
        return false;
    }
    /* 心跳计数未超限也可能尚未收到首帧，故同时检查计数 */
    return (s_state[id].status == DEV_ONLINE) &&
           (s_state[id].offline_cnt <= s_state[id].offline_cnt_max);
}

/**
 * @brief  查询全部电机是否在线
 * @param  无
 * @retval true 全部在线
 */
bool Dart_Motor_All_Online(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < (uint8_t)DART_MOTOR_NUM; motor_index++) {
        if (!Dart_Motor_Is_Online((dart_motor_id_e)motor_index)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  读取转子累计编码器计数
 * @param  id 电机编号
 * @retval 累计计数 count(8192/转)
 */
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id)
{
    int32_t encoder_sum;

    if (id >= DART_MOTOR_NUM) {
        return 0;
    }
    taskENTER_CRITICAL();
    encoder_sum = s_rx[id].encoder_sum;
    taskEXIT_CRITICAL();
    return encoder_sum;
}

/**
 * @brief  读取转子转速
 * @param  id 电机编号
 * @retval 转速 rpm(电调原始反馈)
 */
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id)
{
    if (id >= DART_MOTOR_NUM) {
        return 0.0f;
    }
    return (float)s_rx[id].encoder_speed;
}

/**
 * @brief  设置单台电机输出(限幅后写入发送缓存)
 * @param  id      电机编号
 * @param  out_raw 电调原始电流指令
 * @retval 无
 */
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw)
{
    if (id >= DART_MOTOR_NUM) {
        return;
    }
    s_tx[id].torque = Dart_Clamp_F(out_raw, -s_out_max[id], s_out_max[id]);
}

/**
 * @brief  全部电机输出清零(只改缓存，仍需 Send 发出)
 * @param  无
 * @retval 无
 */
void Dart_Motor_Zero_Output(void)
{
    uint8_t motor_index;

    for (motor_index = 0U; motor_index < (uint8_t)DART_MOTOR_NUM; motor_index++) {
        s_tx[motor_index].torque = 0.0f;
    }
}

/**
 * @brief  发送 CAN1/CAN2 的 0x200 控制帧
 * @param  无
 * @retval 无
 * @note   发送成功只表示进入邮箱，不代表电调已执行
 */
void Dart_Motor_Send(void)
{
    s_group_can1.group_set_torque(&s_group_can1);
    s_group_can2.group_set_torque(&s_group_can2);
}
