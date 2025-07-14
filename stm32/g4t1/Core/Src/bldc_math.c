 /**
 * Martin Egli
 * 2025-07-13
 * bldc math module
 */

#include "bldc_math.h"
#include <stddef.h>
#include <math.h>

// using math.h
#define SQRT3_2 (M_SQRT3)/2

// calculate clarke + park transformations, see https://ww1.microchip.com/downloads/aemDocuments/documents/FPGA/ProductDocuments/UserGuides/ip_cores/directcores/foc_transformations_ug.pdf
// implement without shortcuts

uint16_t bldc_calc_clarke_transform(bldc_motor_t *m) {
    m->calc.alpha_in = m->current.u_in -(1/2 * m->current.v_in) - (1/2 * m->current.w_in);
    m->calc.beta_in  = (SQRT3_2 * m->current.v_in) - (SQRT3_2 * m->current.w_in);
    return true;
}

uint16_t bldc_calc_inv_clarke_transform(bldc_motor_t *m) {
    m->calc.u_out = m->calc.alpha_out;
    m->calc.v_out = (-1/2 * m->calc.alpha_out) + (SQRT3_2 * m->calc.beta_out);
    m->calc.w_out = (-1/2 * m->calc.alpha_out) - (SQRT3_2 * m->calc.beta_out);
    return true;
}

uint16_t bldc_calc_park_transform(bldc_motor_t *m) {
    float sin_theta = sin(m->current.angle_rad);
    float cos_theta = cos(m->current.angle_rad);
    m->calc.d_in = +(m->calc.alpha_in * cos_theta) + (m->calc.beta_in * sin_theta);
    m->calc.q_in = -(m->calc.alpha_in * sin_theta) + (m->calc.beta_in * cos_theta);
    return true;
}

uint16_t bldc_calc_inv_park_transform(bldc_motor_t *m) {
    float sin_theta = sin(m->current.angle_rad);
    float cos_theta = cos(m->current.angle_rad);
    m->calc.alpha_out = +(m->calc.d_out * cos_theta) - (m->calc.q_out * sin_theta);
    m->calc.beta_out  = -(m->calc.d_out * sin_theta) + (m->calc.q_out * cos_theta);
    return true;
}
