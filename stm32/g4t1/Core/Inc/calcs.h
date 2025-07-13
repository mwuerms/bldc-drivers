/**
 * Martin Egli
 * 2025-07-05
 * calculations module
 */

#ifndef _CALCS__H_
#define _CALCS__H_

#include "project.h"

#define DEG_TO_RAD(a) (a*M_PI/180.0f)
#define RAD_TO_DEG(a) (a*180.0f/M_PI)

/**
 * calc angle velocity in rad/s from rotation in speed 1/s
 */
float calc_angle_rad_s_from_rotation_speed_rot_s(float speed_rot_s);

/**
 * calc rotation speed in 1/s from angle velocity in rad/s
 */
float calc_rotation_speed_rot_s_from_angle_rad_s(float angle_rad_s);

/**
 * constrain a float value in between min and max values
 */
float calc_constrain_float_to_min_max_values(float value, float min_value, float max_value);

/**
 * prevent a float value from overvlow, + or - full if outside boundaries
 */
float calc_prevent_float_to_overflow(float value, float min_value, float max_value, float full);

#endif // _CALCS__H_
