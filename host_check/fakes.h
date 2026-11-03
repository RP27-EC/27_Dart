/**
 * @file    fakes.h
 * @brief   host_check 假模块的可控状态，供 check_fsm.c 设置场景
 */
#ifndef __HOST_FAKES_H
#define __HOST_FAKES_H

#include <string.h>
#include "dart_fsm.h"
#include "rc_cmd.h"
#include "fault.h"
#include "fire_gate.h"
#include "loader.h"
#include "tension.h"
#include "yaw.h"
#include "trigger.h"
#include "feeder.h"
#include "tf_log.h"
#include "buzzer.h"
#include "dart_tune.h"

extern rc_cmd_t        g_rc;
extern mech_axis_t     g_loader_axis;
extern int             g_loader_pos;
extern int             g_loader_target;
extern float           g_loader_last_speed;
extern bool            g_auto_arrive;
extern trigger_state_e g_trigger;
extern bool            g_trigger_lock_instant;
extern bool            g_motor_online;
extern bool            g_single_side;
extern uint32_t        g_single_side_tick;
extern uint32_t        g_fault_mask;
extern int             g_trigger_release_cnt;
extern int             g_zero_req_cnt[3];      /* 上膛/变力/Yaw 找零请求次数 */
extern int             g_tone_cnt[BUZZER_TONE_NUM]; /* 各提示音请求次数 */

void Fakes_Reset(void);

#endif
