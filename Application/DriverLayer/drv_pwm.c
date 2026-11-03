/**
 ******************************************************************************
 * @file    drv_pwm.c
 * @author  RobotPilots 镖架电控
 * @brief   TIM1 四路舵机 PWM 驱动
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  启动时比较值为 0(无脉冲)，由 servo 层在初始化时写入初始脉宽。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_pwm.h"

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* 通道编号到 HAL 通道的映射 */
static const uint32_t s_pwm_hal_channel[PWM_SERVO_CH_NUM] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
};

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  启动 TIM1 四路 PWM 输出
 * @param  无
 * @retval 无
 * @note   在 MX_TIM1_Init 之后、调度器启动之前调用一次
 */
void PWM_Servo_Init(void)
{
    uint8_t ch_index;

    for (ch_index = 0U; ch_index < (uint8_t)PWM_SERVO_CH_NUM; ch_index++) {
        __HAL_TIM_SET_COMPARE(&htim1, s_pwm_hal_channel[ch_index], 0U);
        (void)HAL_TIM_PWM_Start(&htim1, s_pwm_hal_channel[ch_index]);
    }
}

/**
 * @brief  写入一路舵机脉宽
 * @param  ch       通道编号
 * @param  pulse_us 高电平宽度，单位 µs(1 计数 = 1µs)
 * @retval 无
 */
void PWM_Servo_Set_Pulse(pwm_servo_ch_e ch, uint16_t pulse_us)
{
    if (ch >= PWM_SERVO_CH_NUM) {
        return;
    }
    __HAL_TIM_SET_COMPARE(&htim1, s_pwm_hal_channel[ch], pulse_us);
}
