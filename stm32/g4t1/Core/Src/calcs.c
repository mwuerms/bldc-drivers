/**
 * Martin Egli
 * 2025-07-05
 * calculations module
 */

#include "calcs.h"
#include "math.h"

float calc_angle_velocity_from_rotation_speed(float speed_rot_s) {
	return speed_rot_s * (2 * M_PI);
}

float calc_rotation_speed_from_angle_velocity(float angle_rad_s) {
	return angle_rad_s / (2 * M_PI);
}

float calc_constrain_float_to_min_max_values(float value, float min_value, float max_value) {
	if(value > max_value) return max_value;
	if(value < min_value) return min_value;
	return value;
}

float calc_prevent_float_to_overflow(float value, float min_value, float max_value, float full) {
	if(value > max_value) return (value - full);
	if(value < min_value) return (value + full);
	return value;
}
