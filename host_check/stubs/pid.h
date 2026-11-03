/* host_check 桩：与固件 PID.h 中 pid_ctrl_t 字段一致，供 mech_axis.h 编译 */
#ifndef __PID_STUB_H
#define __PID_STUB_H
typedef struct pid_ctrl {
    float target, measure, err, last_err, kp, ki, kd;
    float pout, iout, dout, out, last_dout, integral, integral_max, out_max;
} pid_ctrl_t;
void single_pid_ctrl(pid_ctrl_t *pid);
#endif
