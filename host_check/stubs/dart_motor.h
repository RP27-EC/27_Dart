/* host_check 桩：只保留 dart_fsm.c / mech_axis.h 用到的电机接口，不含 HAL/CAN */
#ifndef __DART_MOTOR_STUB_H
#define __DART_MOTOR_STUB_H
#include "dart_config.h"

typedef enum {
    DART_MOTOR_LOADER_L = 0,
    DART_MOTOR_LOADER_R,
    DART_MOTOR_TENSION,
    DART_MOTOR_YAW,
    DART_MOTOR_NUM,
} dart_motor_id_e;

bool Dart_Motor_Is_Online(dart_motor_id_e id);
bool Dart_Motor_All_Online(void);
int32_t Dart_Motor_Get_Encoder_Sum(dart_motor_id_e id);
float Dart_Motor_Get_Speed_Rpm(dart_motor_id_e id);
void Dart_Motor_Set_Output(dart_motor_id_e id, float out_raw);

#endif
