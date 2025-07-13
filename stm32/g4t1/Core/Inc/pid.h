/**
 * Martin Egli
 * 2025-07-13
 * pid module
 */

#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float error_sum, error_old;
} pid_t;

uint16_t pid_set(pid_t *p, float kp, float ki, float kd);
uint16_t pid_reset(pid_t *p);

/**
 * process pid
 * @param   p       pid context, see pid_t
 * @param   error   input error to process
 * @param   dt      delta t
 * @return  result y
 */
float pid_process(pid_t *p, float error, float dt);

#endif // _PID_H_
