/**
 * Martin Egli
 * 2025-06-30
 * angle sensor module
 */
#ifndef _ANGLE_SENSOR_H_ 
#define _ANGLE_SENSOR_H_ 

#include <stdint.h>
#include <stdbool.h>
#include "project.h"
#include "filter.h"

#define ANGLE_SENS_I2C_BUF_SIZE (8)
typedef struct {
    struct {
    	I2C_HandleTypeDef *hi2c;
        uint8_t addr;
        uint8_t buf[ANGLE_SENS_I2C_BUF_SIZE];
        uint16_t buf_len;
    } i2c;
    struct {
		GPIO_TypeDef *gpio_port;
		uint32_t pin_mask;
	} as_en;
    float angle_rad;
    uint16_t raw_angle;
    float angle_rad_filtered;
    ffir_t frad; // filter angle_rad
    uint16_t type;
} angle_sens_t;

// - angle sensor: AS5600 ------------------------------------------------------
#define ANGLE_SENSOR_TYPE_AS5600 (5600)
#define AS5600_I2C_ADDR (0x36)

uint16_t angle_sensor_init(angle_sens_t *as, I2C_HandleTypeDef *hi2c);
uint16_t angle_sensor_set_enable_pin(angle_sens_t *as, GPIO_TypeDef *port, uint32_t pm);
uint16_t angle_sensor_enable(angle_sens_t *as);
uint16_t angle_sensor_disable(angle_sens_t *as);
uint16_t angle_sensor_get(angle_sens_t *as);

#endif // _ANGLE_SENSOR_H_ 
