/**
 * Martin Egli
 * 2025-07-13
 * bldc math module
 */

#ifndef _BLDC_UTILS_H_
#define _BLDC_UTILS_H_

#include "bldc_motor.h"

/**
 * clarke transformation
 * (u_in, v_in, w_in) -> (alpha_in, beta_in)
 */
uint16_t bldc_calc_clarke_transform(bldc_motor_t *m);

/**
 * inverse clarke transformation
 * (alpha_out, beta_out) -> (u_out, v_out, w_out)
 */
uint16_t bldc_calc_inv_clarke_transform(bldc_motor_t *m);

/**
 * park transformation
 * (alpha_in, beta_in) -> (d_in, q_in)
 */
uint16_t bldc_calc_park_transform(bldc_motor_t *m);

/**
 * inverse park transformation
 * (d_out, q_out) -> (alpha_out, beta_out)
 */
uint16_t bldc_calc_inv_park_transform(bldc_motor_t *m);

#endif  // _BLDC_UTILS_H_
