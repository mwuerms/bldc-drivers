/**
 * Martin Egli
 * 2025-06-29
 * bldc motor module
 */
#ifndef _BLDC_MOTOR_H_ 
#define _BLDC_MOTOR_H_ 

#include <stdint.h>
#include <stdbool.h>
#include "project.h"
#include "bldc_driver.h"
#include "angle_sensor.h"
#include "pid.h"

//#ifndef STM32F103xB
#ifndef STM32G431xx
#error wrong mcu
#else
#warning info mcu defined STM32G431xx
#endif

typedef struct {
    bldc_driver_t *d;
    angle_sens_t *as;
    struct {
        float angle_rad; /// target angle in rad
        //float speed_rot_s; /// target speed in rotation / s
        float speed_rad_s; /// target speed in rad / s
    } target;
    struct {
        float angle_rad; /// current angle in rad
        float angle_rad_old;
        float delta_angle_rad;
		float speed_rad_s; /// current speed, angle velocity in rad / s
        float u_in, v_in, w_in; /// input to clarke transformation
        float el_angle_rad; ///
    } current;
    struct {
        float shaft_angle_rad; /// 
        float shaft_angle_rad_old; /// 
        float el_angle_rad; /// 
        float el_angle_rad_old; ///
        float alpha_in, beta_in; /// clarke transformation: calc alpha + beta from u, v, w
        float alpha_out, beta_out; /// inverse clarke transformation: calc u, v, w from alpha + beta
        float u_out, v_out, w_out; /// result from inverse clarke transformation
        float q_in, d_in; /// park transformation, calc d, q from alpha + beta
        float q_out, d_out; /// inverse park transformation, calc alpha + beta from d, q
        float q_velocity_out; // result of velocity pid
        
    } calc; /// for calculating purposes
    struct {
        float voltage;
        float speed_rad_s; /// limit speed in rotation / s
    } limit;
    struct {
        uint16_t nb_pole_pairs;
        float kv;
        float coil_resistance;
    } motor;
    struct {
        pid_t angle;
        pid_t speed;
        pid_t iq, id;
    } pid;
    struct {
        float iq, id;
    	float vq, vd;
    } set;
    struct {
        uint16_t status;
        uint16_t type;
    } ctrl;
} bldc_motor_t;

#define BLDC_MOTOR_STATUS_ENABLED (0x0001)

#define BLDC_MOTOR_CTRL_TYPE_VELOCITY_OPENLOOP (0)
#define BLDC_MOTOR_CTRL_TYPE_ANGLE_OPENLOOP (1)
#define BLDC_MOTOR_CTRL_TYPE_VELOCITY (2)
#define BLDC_MOTOR_CTRL_TYPE_ANGLE (3)

extern float dbg_el;
extern float dbg_va;
extern float dbg_vb;

extern float dbg_uu;
extern float dbg_uv;
extern float dbg_uw;

uint16_t bldc_motor_init(bldc_motor_t *m, bldc_driver_t *d, angle_sens_t *as);
uint16_t bldc_motor_set_motor_parameters(bldc_motor_t *m, uint16_t pp, float kv, float coil_res);
uint16_t bldc_motor_set_ctrl_type(bldc_motor_t *m, uint16_t ctype);
uint16_t bldc_motor_set_angle_pid(bldc_motor_t *m, float kp, float ki, float kd);
uint16_t bldc_motor_set_speed_pid(bldc_motor_t *m, float kp, float ki, float kd);
uint16_t bldc_motor_set_iq_pid(bldc_motor_t *m, float kp, float ki, float kd);
uint16_t bldc_motor_set_id_pid(bldc_motor_t *m, float kp, float ki, float kd);
uint16_t bldc_motor_set_voltage_limit(bldc_motor_t *m, float voltage_limit);
uint16_t bldc_motor_set_speed_limit(bldc_motor_t *m, float speed);
uint16_t bldc_motor_set_target_speed(bldc_motor_t *m, float speed);
uint16_t bldc_motor_set_target_angle_deg(bldc_motor_t *m, float angle_deg);

uint16_t bldc_motor_enable(bldc_motor_t *m);
uint16_t bldc_motor_disable(bldc_motor_t *m);
uint16_t bldc_motor_move(bldc_motor_t *m, float dt);

#endif // _BLDC_MOTOR_H_
