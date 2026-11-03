/**
 ******************************************************************************
 * @file    drv_pwm.h
 * @author  RobotPilots 镖架电控
 * @brief   TIM1 四路舵机 PWM 驱动接口
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  TIM1：168MHz，PSC=167，ARR=19999 -> 1µs/计数，周期 20ms(50Hz)。
 *  通道与引脚：CH1 PE9 / CH2 PE11 / CH3 PE13 / CH4 PE14。
 *  只负责写比较值，脉宽合法范围由 servo 设备层限制。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */
#ifndef __DRV_PWM_H
#define __DRV_PWM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  舵机 PWM 通道编号
 */
typedef enum {
    PWM_SERVO_CH1 = 0,   /* TIM1_CH1 PE9  撒放器 */
    PWM_SERVO_CH2,       /* TIM1_CH2 PE11 剪叉A */
    PWM_SERVO_CH3,       /* TIM1_CH3 PE13 剪叉B */
    PWM_SERVO_CH4,       /* TIM1_CH4 PE14 储弹限位 */
    PWM_SERVO_CH_NUM,    /* 通道总数 */
} pwm_servo_ch_e;

/* Exported functions --------------------------------------------------------*/
void PWM_Servo_Init(void);
void PWM_Servo_Set_Pulse(pwm_servo_ch_e ch, uint16_t pulse_us);

#endif /* __DRV_PWM_H */
