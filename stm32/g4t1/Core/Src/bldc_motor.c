/**
 * Martin Egli
 * 2025-06-28
 * bldc motor module
 */

#include "bldc_motor.h"
#include "bldc_math.h"
#include "calcs.h"
#include <math.h>
#include "adc.h"

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
    pid_set(&(m->pid.angle), 0.0f, 0.0f, 0.0f);
    pid_set(&(m->pid.speed), 0.0f, 0.0f, 0.0f);
    pid_set(&(m->pid.iq), 0.0f, 0.0f, 0.0f);
    pid_set(&(m->pid.id), 0.0f, 0.0f, 0.0f);
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

uint16_t bldc_motor_set_angle_pid(bldc_motor_t *m, float kp, float ki, float kd) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    return pid_set(&(m->pid.angle), kp, ki, kd);
}

uint16_t bldc_motor_set_speed_pid(bldc_motor_t *m, float kp, float ki, float kd) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    return pid_set(&(m->pid.speed), kp, ki, kd);
}

uint16_t bldc_motor_set_iq_pid(bldc_motor_t *m, float kp, float ki, float kd) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    return pid_set(&(m->pid.iq), kp, ki, kd);
}

uint16_t bldc_motor_set_id_pid(bldc_motor_t *m, float kp, float ki, float kd) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
    return pid_set(&(m->pid.id), kp, ki, kd);
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
	m->limit.speed_rad_s = speed * 2* M_PI; //calc_angle_rad_s_from_rotation_speed_rot_s(speed);
    return true;
}

uint16_t bldc_motor_set_target_speed(bldc_motor_t *m, float speed) {
	if(m == NULL) {
        // error, invalid motor
        return false;
    }
	m->target.speed_rad_s = speed * 2* M_PI;//calc_angle_rad_s_from_rotation_speed_rot_s(speed);
    return true;
}

uint16_t bldc_motor_set_target_angle_deg(bldc_motor_t *m, float angle_deg) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    m->target.angle_rad = DEG_TO_RAD(angle_deg);
    return true;
}

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
    return bldc_driver_enable(m->d);
    
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
static uint16_t bldc_motor_drive_phase_voltage(bldc_motor_t *m, float vq, float vd, float el_angle_rad) {
	// inverse park transformations
	float cos_theta = cosf(el_angle_rad);
	float sin_theta = sinf(el_angle_rad);

	float va = cos_theta * vd - sin_theta * vq;
	float vb = sin_theta * vd + cos_theta * vq;

	// Clarke transform
	float uu, uv, uw;
	float vcenter = 0.5f;
	uu = va + vcenter;
	uv = -0.5f * va + SQRT3_2 * vb + vcenter;
	uw = -0.5f * va - SQRT3_2 * vb + vcenter;

    m->calc.u_out = uu;
    m->calc.v_out = uv;
    m->calc.w_out = uw;

	return bldc_driver_set_phase_voltages(m->d, uu, uv, uw);
}

uint16_t bldc_motor_svpwm(bldc_motor_t *m, float alpha, float beta) {
    /*if(m == NULL) {
        // error, invalid motor
        return false;
    }*/
    // alpha, beta (kartesian) to polar
    float mag = sqrt(alpha*alpha + beta*beta);
    float angle_rad = atan2(beta, alpha);

    // get sector of angle_rad
    uint16_t sector = 4;
    if(     (angle_rad >=      0.0f) && (angle_rad < +(1.0f/3.0f)*M_PI)) sector = 1;
    else if((angle_rad >= +(1.0f/3.0f)*M_PI) && (angle_rad < +(2.0f/3.0f)*M_PI)) sector = 2;
    else if((angle_rad >= +(2.0f/3.0f)*M_PI) && (angle_rad < +M_PI)) sector = 3;
    else if((angle_rad >= -(1.0f/3.0f)*M_PI) && (angle_rad <      0.0f)) sector = 6;
    else if((angle_rad >= -(2.0f/3.0f)*M_PI) && (angle_rad < -(1.0f/3.0f)*M_PI)) sector = 5;
    else sector = 4;

    // calc basic timings
    float t0, t1, t2;
    float tu, tv, tw;
    float angle_sector = angle_rad -(sector - 1) * M_PI/3;
    t1 = mag * sin(M_PI/3 - angle_sector) / SQRT3;
    t2 = mag * sin(angle_sector) / SQRT3;
    t0 = 1.0f - t1 - t2;

    // calc timings according to sector
    switch(sector) {
        case 1:
            tu = (t1 + t2 + t0/2);
            tv = (t2 + t0/2);
            tw = (t0/2);
            break;
        case 2:
            tu = (t1 + t0/2);
            tv = (t1 + t2 + t0/2);
            tw = (t0/2);
            break;
        case 3:
            tu = (t0/2);
            tv = (t1 + t2 + t0/2);
            tw = (t2 + t0/2);
            break;
        case 4:
            tu = (t0/2);
            tv = (t1 + t0/2);
            tw = (t1 + t2 + t0/2);
            break;
        case 5:
            tu = (t2 + t0/2);
            tv = (t0/2);
            tw = (t1 + t2 + t0/2);
            break;
        case 6:
            tu = (t1 + t2 + t0/2);
            tv = (t0/2);
            tw = (t1 + t0/2);
            break;
    }

    m->calc.u_out = tu;
    m->calc.v_out = tv;
    m->calc.w_out = tw;
    return bldc_driver_set_phase_voltages(m->d, tu, tv, tw);
}

static uint16_t bldc_motor_velocity_openloop(bldc_motor_t *m, float dt) {
    m->calc.shaft_angle_rad = m->target.speed_rad_s * dt + m->calc.shaft_angle_rad_old;
    m->calc.shaft_angle_rad = calc_prevent_float_to_overflow(m->calc.shaft_angle_rad, -M_PI, M_PI, 2*M_PI);
    m->calc.el_angle_rad = m->calc.shaft_angle_rad;// * m->motor.nb_pole_pairs;// next electrical angle

    m->calc.shaft_angle_rad_old = m->calc.shaft_angle_rad;
    m->calc.el_angle_rad_old = m->calc.el_angle_rad;

    //return bldc_motor_drive_phase_voltage(m, m->set.vq, 0, m->calc.el_angle_rad);

    m->calc.alpha_out = m->set.vq * sin(m->calc.el_angle_rad);
    m->calc.beta_out  = m->set.vq * cos(m->calc.el_angle_rad);
    return bldc_motor_svpwm(m, m->calc.alpha_out, m->calc.beta_out);
}

static uint16_t bldc_motor_angle_openloop(bldc_motor_t *m, float dt) {
	float error = (m->target.angle_rad) - m->calc.shaft_angle_rad;
	if(fabsf(error) < (0.02f * M_PI)) {
		// error is small enough, do not continue
		//bldc_motor_disable(m);
		//return true;
		m->target.speed_rad_s = 0;
		return true;
	}
	float angle_rad_s = error / dt; // angle velocity needed
	m->target.speed_rad_s = calc_constrain_float_to_min_max_values(angle_rad_s, -m->limit.speed_rad_s, +m->limit.speed_rad_s);
    return true;
}

static uint16_t bldc_motor_torque_pid(bldc_motor_t *m, float dt) {
    // pid(0 - d_in)-> (d_out)
    m->calc.d_out = pid_process(&(m->pid.id), (0.0f - m->calc.d_in), dt);
    // pid(q_velocity_out - q_in) -> q_out
    m->calc.q_out = pid_process(&(m->pid.iq), (m->calc.q_velocity_out - m->calc.q_in), dt);
    return true;
}

static uint16_t bldc_motor_velocity_pid(bldc_motor_t *m, float dt) {
    // pid(target.speed - current.angle_rot_s)-> (q_velocity_out)
    m->calc.q_velocity_out = pid_process(&(m->pid.speed), (m->target.speed_rad_s - m->current.speed_rad_s), dt);
    return true;
}
static uint16_t bldc_motor_angle_pid(bldc_motor_t *m, float dt) {
    // pid(target.angle_rad - current.angle_rad)-> (q_velocity_out)
    m->target.speed_rad_s = pid_process(&(m->pid.angle), (m->target.angle_rad - m->current.angle_rad), dt);
    return false;
}

#define A1_SIZE (8)
static float a1[A1_SIZE] = {0.0f};
static float a1_sum;
static uint16_t a1i = 0;
static uint16_t a1n = 0;
static uint16_t a1st = 2;

static uint16_t bldc_motor_get_sensor_values(bldc_motor_t *m, float dt) {
    // get angle
    if(angle_sensor_get(m->as) == false) {
        // error, no valid angle value
    	return false;
    }
	//m->as->angle_rad = 0.5f;
    m->current.angle_rad = m->as->angle_rad;

    a1[a1n] = m->current.angle_rad;
	a1n++;
	if(a1n >= A1_SIZE) {
		a1n = 0;
	}
	a1_sum = 0.0f;
	for(a1i = 0; a1i < A1_SIZE; a1i++) {
		a1_sum += a1[a1i];
	}
	m->current.angle_rad = a1_sum/A1_SIZE;

    // calc angle and angle velocity
    m->current.delta_angle_rad = (m->current.angle_rad - m->current.angle_rad_old);
    // limit
    m->current.delta_angle_rad = calc_prevent_float_to_overflow(m->current.delta_angle_rad, 0.0f, 2*M_PI, 2*M_PI);
	if(a1st) {
		a1st--;
		m->current.speed_rad_s = 0.0f;
	}
	else {
		m->current.speed_rad_s = m->current.delta_angle_rad / dt;
	}


    m->current.el_angle_rad = m->current.angle_rad * m->motor.nb_pole_pairs;
    m->current.angle_rad_old = m->current.angle_rad;
    // get u, v, w coil currents
    m->current.adc_u = ADC_GET_CURRENT_U();
    m->current.adc_v = ADC_GET_CURRENT_V();
    m->current.adc_w = ADC_GET_CURRENT_W();

    m->current.adc_poti = ADC_GET_POTI();
    m->current.adc_ntc = ADC_GET_NTC();
    return true;
}

uint16_t bldc_motor_move(bldc_motor_t *m, float dt) {
    if(m == NULL) {
        // error, invalid motor
        return false;
    }
    if((m->ctrl.status & BLDC_MOTOR_STATUS_ENABLED) == 0) {
        // is disabled, stop here
        return false;
    }
    bldc_motor_get_sensor_values(m, dt);

    switch(m->ctrl.type) {
        case BLDC_MOTOR_CTRL_TYPE_ANGLE_OPENLOOP: 
            // in case of openloop angle control, adjust target.speed_rad_s
            // and proceed as openloop speed control
            bldc_motor_angle_openloop(m, dt);
        case BLDC_MOTOR_CTRL_TYPE_VELOCITY_OPENLOOP: 
            return bldc_motor_velocity_openloop(m, dt);
        
        case BLDC_MOTOR_CTRL_TYPE_ANGLE:
            // in case of angle control, adjust target.speed_rot_s
            // and proceed as speed control
            bldc_motor_angle_pid(m, dt);
        case BLDC_MOTOR_CTRL_TYPE_VELOCITY:
            bldc_motor_velocity_pid(m, dt);
            // (u_in, v_in, w_in) -> (alpha_in, beta_in)
            bldc_calc_clarke_transform(m);
            // (alpha_in, beta_in) -> (d_in, q_in)
            bldc_calc_park_transform(m);
        	bldc_motor_torque_pid(m, dt);
            // (d_out, q_out) -> (alpha_out, beta_out)
            bldc_calc_inv_park_transform(m);
            // (alpha_out, beta_out) -> (u_out, v_out, w_out)
            //bldc_utils_calc_inv_clarke_transform(m);
            bldc_motor_svpwm(m, m->calc.alpha_out, m->calc.beta_out);

            dbg_uu = m->calc.u_out;
            dbg_uv = m->calc.v_out;
            dbg_uw = m->calc.w_out;

            return true;//bldc_driver_set_phase_voltages(m->d, m->calc.u_out, m->calc.v_out, m->calc.w_out);
    }
    // error, shall not reach here, this means no ctrl.type was set
    return false;
}
