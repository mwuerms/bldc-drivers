/**
 * Martin Egli
 * 2025-06-28
 * bldc motor module
 */

#include "bldc_motor.h"
#include "calcs.h"
#include <math.h>

uint16_t bldc_motor_init(bldc_motor_t *m, bldc_driver_t *d, angle_sens_t *as) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    if(d == NULL) {
        // error, invalid bldc driver, this is mandatory
        return false;
    }
    m->d = d;
    if(as == NULL) {
        // error, invalid angle sensor, this is mandatory
        return false;
    }
	m->as = as;

    m->ctrl.status &= ~BLDC_MOTOR_STATUS_ENABLED;
    m->ctrl.type = BLDC_MOTOR_CTRL_TYPE_VELOCITY_OPENLOOP;
    m->limit.voltage = 0.0f;
    m->pid.kp = 0.0f;
    m->pid.ki = 0.0f;
    m->pid.kd = 0.0f;
    return true;
}

uint16_t bldc_motor_set_motor_parameters(bldc_motor_t *m, uint16_t pp, float kv, float coil_res) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->motor.nb_pole_pairs = pp;
    m->motor.kv = kv;
    m->motor.coil_resistance = coil_res;
    return true;
}

uint16_t bldc_motor_set_ctrl_type(bldc_motor_t *m, uint16_t ctype) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->ctrl.type = ctype;
    return true;
}

uint16_t bldc_motor_set_pid(bldc_motor_t *m, float kp, float ki, float kd) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->pid.kp = kp;
    m->pid.ki = ki;
    m->pid.kd = kd;
    return true;
}

uint16_t bldc_motor_set_voltage_limit(bldc_motor_t *m, float voltage_limit) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->limit.voltage = voltage_limit;
    return true;
}

uint16_t bldc_motor_set_speed_limit(bldc_motor_t *m, float speed) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->limit.speed_rot_s = speed;
    return true;
}

uint16_t bldc_motor_set_target_speed(bldc_motor_t *m, float speed) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->target.speed_rot_s = speed;
    return true;
}

uint16_t bldc_motor_set_target_angle_deg(bldc_motor_t *m, float angle_deg) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->target.angle_deg = angle_deg;
    return true;
}


/* set template
uint16_t bldc_motor_set_target_x(bldc_motor_t *m, float x) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m-> = x;
    return true;
}
*/

uint16_t bldc_motor_enable(bldc_motor_t *m) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    if(bldc_driver_enable(m->d) == false) {
        // error, could not enable bldc driver
        return false;
    }
    // OK, motor + driver enabled
    m->ctrl.status |= BLDC_MOTOR_STATUS_ENABLED;
    return true;
    
}
uint16_t bldc_motor_disable(bldc_motor_t *m) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->ctrl.status &= ~BLDC_MOTOR_STATUS_ENABLED;
    return bldc_driver_disable(m->d);
}

float dbg_el = 0.0f;
float dbg_va = 0.0f;
float dbg_vb = 0.0f;
float dbg_uu = 0.0f;
float dbg_uv = 0.0f;
float dbg_uw = 0.0f;

uint16_t statangle_u = 0;
uint16_t statangle_v = 0;
uint16_t statangle_w = 0;
uint16_t statangle_inc = 1;


#define SQRT3   (1.7320508075688772f)
#define SQRT3_2 (0.86602540378f)

// also SVPWM, space vector modulation
#define DEG_TO_RAD(a) (a*M_PI/180.0f)
#define RAD_TO_DEG(a) (a*180.0f/M_PI)
static uint16_t bldc_motor_drive_phase_voltage(bldc_motor_t *m, float vq, float vd, float el_angle_deg) {
	// inverse park transformations
	float theta_e = DEG_TO_RAD(el_angle_deg);
	float cos_theta = cosf(theta_e);
	float sin_theta = sinf(theta_e);

	float va = cos_theta * vd - sin_theta * vq;
	float vb = sin_theta * vd + cos_theta * vq;

	// Clarke transform
	float uu, uv, uw;
	float vcenter = 0.5f;
	uu = va + vcenter;
	uv = -0.5f * va + SQRT3_2 * vb + vcenter;
	uw = -0.5f * va - SQRT3_2 * vb + vcenter;

	dbg_el = el_angle_deg;
	dbg_va = va;
	dbg_vb = vb;

	dbg_uu = uu;
	dbg_uv = uv;
	dbg_uw = uw;

	return bldc_driver_set_phase_voltages(m->d, uu, uv, uw);
}

static uint16_t bldc_motor_velocity_openloop(bldc_motor_t *m, float dt) {

    m->calc.shaft_angle_deg = calc_angle_velocity_from_rotation_speed(m->target.speed_rot_s) * dt + m->calc.shaft_angle_deg_old; // next shaft angle
    if(m->calc.shaft_angle_deg > 360.0f) m->calc.shaft_angle_deg -= 360.0f;
    if(m->calc.shaft_angle_deg <   0.0f) m->calc.shaft_angle_deg += 360.0f;
    m->calc.el_angle_deg = m->calc.shaft_angle_deg * m->motor.nb_pole_pairs;// next electrical angle

    m->calc.shaft_angle_deg_old = m->calc.shaft_angle_deg;
    m->calc.el_angle_deg_old = m->calc.el_angle_deg;
    
	return bldc_motor_drive_phase_voltage(m, m->set.vq, 0, m->calc.el_angle_deg);
}

static uint16_t bldc_motor_angle_openloop(bldc_motor_t *m, float dt) {
	float error = (m->target.angle_deg) - m->calc.shaft_angle_deg;
	if(calc_absf(error) < 0.2f) {
		// error is small enough, do not continue
		bldc_motor_disable(m);
		return true;
	}
	float angle_deg_s = error / dt; // angle velocity needed
	if(calc_absf(angle_deg_s) > calc_angle_velocity_from_rotation_speed(m->limit.speed_rot_s)) {
		m->target.speed_rot_s = m->limit.speed_rot_s;
	}
	else {
		m->target.speed_rot_s = calc_rotation_speed_from_angle_velocity(angle_deg_s);
	}
    return bldc_motor_velocity_openloop(m, dt);
}

static uint16_t bldc_motor_velocity_pi(bldc_motor_t *m, float dt) {
	float error = calc_angle_velocity_from_rotation_speed(m->target.speed_rot_s) - m->current.angle_deg_s;

	// out = kp * err + ki * err * dt + old_i + kd * err / dt;
	m->pid.integrator += m->pid.ki * error * dt;
	// limit
	if(m->pid.integrator > +m->limit.voltage) m->pid.integrator = +m->limit.voltage;
	if(m->pid.integrator < -m->limit.voltage) m->pid.integrator = -m->limit.voltage;

	m->set.iq = m->pid.kp * error + m->pid.integrator;
    m->set.id = 0.0f; // shall be 0
    m->set.vq = m->set.iq; // TODO is a phase resistor set? if so, use it
	if(m->set.vq > +m->limit.voltage) m->set.vq = +m->limit.voltage;
	if(m->set.vq < -m->limit.voltage) m->set.vq = -m->limit.voltage;
    m->set.vd = m->set.id; // restrict no limits, shall be 0

    return bldc_motor_drive_phase_voltage(m, m->set.vq, 0, m->current.el_angle_deg);
}

#define A1_SIZE (8)
static float a1[A1_SIZE] = {0.0f};
static float a1_sum;
static uint16_t a1i = 0;
static uint16_t a1n = 0;
static uint16_t a1st = 2;
uint16_t bldc_motor_move(bldc_motor_t *m, float dt) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    if((m->ctrl.status & BLDC_MOTOR_STATUS_ENABLED) == 0) {
        // is disabled, stop here
        return false;
    }
    // get angle
    /*if(angle_sensor_get(m->as) == false) {
        // error, no valid angle value
    	return false;
    }*/
    m->current.angle_deg = m->as->angle_deg;
    m->current.angle_deg = 10.0f;

    a1[a1n] = m->current.angle_deg;
	a1n++;
	if(a1n >= A1_SIZE) {
		a1n = 0;
	}
	a1_sum = 0.0f;
	for(a1i = 0; a1i < A1_SIZE; a1i++) {
		a1_sum += a1[a1i];
	}
	m->current.angle_deg = a1_sum/A1_SIZE;

    // calc angle and angle velocity
    m->current.delta_angle_deg = (m->current.angle_deg - m->current.angle_deg_old);
    // limit
    if(m->current.delta_angle_deg >  180.0f) m->current.delta_angle_deg -= 360.0f;
	if(m->current.delta_angle_deg < -180.0f) m->current.delta_angle_deg += 360.0f;
	if(a1st) {
		a1st--;
		m->current.angle_deg_s = 0.0f;
	}
	else {
		m->current.angle_deg_s = m->current.delta_angle_deg / dt;
	}


    m->current.el_angle_deg = m->current.angle_deg * m->motor.nb_pole_pairs;
    m->current.angle_deg_old = m->current.angle_deg;

    switch(m->ctrl.type) {
        case BLDC_MOTOR_CTRL_TYPE_VELOCITY_OPENLOOP: 
            return bldc_motor_velocity_openloop(m, dt);
        case BLDC_MOTOR_CTRL_TYPE_ANGLE_OPENLOOP: 
            return bldc_motor_angle_openloop(m, dt);
        case BLDC_MOTOR_CTRL_TYPE_VELOCITY:
        	return bldc_motor_velocity_pi(m, dt);
    }
    // error, shall not reach here, this means no ctrl.type was set
    return false;
}
