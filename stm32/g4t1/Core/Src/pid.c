/**
 * Martin Egli
 * 2025-07-13
 * pid module
 */

#include "pid.h"
#include <stddef.h>
#include <stdbool.h>

uint16_t pid_set(pid_t *p, float kp, float ki, float kd) {
    if(p == NULL) {
        // error, invalid pid
        return false;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->error_old = 0.0f;
    p->error_sum = 0.0f;
    return pid_reset(p);
}

uint16_t pid_reset(pid_t *p) {
    if(p == NULL) {
        // error, invalid pid
        return false;
    }
    p->error_old = 0.0f;
    p->error_sum = 0.0f;
    return true;
}   

float pid_process(pid_t *p, float error, float dt) {
    if(p == NULL) {
        // error, invalid pid
        return 0.0f;
    }
    float y = 0.0f;
    if(p->kp != 0.0f) {
        y += p->kp * error;
    }
    if(p->ki != 0.0f) {
        p->error_sum += error;
        y += p->ki * p->error_sum * dt;
    }
    if(p->kd != 0.0f) {
        y += p->kd * (error - p->error_old) / dt;
        p->error_old = error;
    }
    return y;
}
