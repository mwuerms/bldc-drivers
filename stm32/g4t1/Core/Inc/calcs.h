/**
 * Martin Egli
 * 2025-07-05
 * calculations module
 */

#ifndef _CALCS__H_
#define _CALCS__H_

#include "project.h"
#include "math.h"

#define CALC_MATH_SINF(a) sinf(a)

/**
 * get sin (float) from angle in deg using look up table, might be faster
 * @param angle_deg 0.0f ... 359.9999f, if angle_deg is outside +/- 360.0f to get into boundaries
 * @return sinus
 */
float calc_lut_sinf(float angle_deg);

/**
 * get sin values directly via index n from look up table
 * sin_lut[] 0 ... 0.5f
 * @param n index 0 ... 359
 * @return sin from lut
 */
float calc_sinf_from_lut(uint16_t n);

/**
 * calc absolute of float value
 */
float calc_absf(float value);
float calc_angle_velocity_from_rotation_speed(float speed_rot_s);
float calc_rotation_speed_from_angle_velocity(float angle_deg_s);

#endif // _CALCS__H_
