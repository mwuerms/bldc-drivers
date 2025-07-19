/**
 * Martin Egli
 * 2025-06-30
 * angle sensor module
 */

#include "angle_sensor.h"
#include "i2c.h"
#include <math.h>
#include "filter.h"

// - public functions ----------------------------------------------------------
uint16_t angle_sensor_init(angle_sens_t *as, I2C_HandleTypeDef *hi2c) {
	if(as == NULL) {
		// error, invalid sensor
		return false;
	}
	if(hi2c == NULL) {
		// error, invalid i2c device
		return false;
	}
	as->i2c.hi2c = hi2c;
	as->type = ANGLE_SENSOR_TYPE_AS5600;
	as->i2c.addr = AS5600_I2C_ADDR;

	// filter, simple mean value
	//float rfad_coeffs[] = {0.125f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f, 0.125f};
	//ffir_init(&(as->frad), rfad_coeffs, 8);
	//float rfad_coeffs[] = {0.111792111249065504f, 0.141863737748378932f, 0.161888230432384622f, 0.168911841140341912f, 0.161888230432384622f, 0.141863737748378932f, 0.111792111249065504f};
	//ffir_init(&(as->frad), rfad_coeffs, 7);
	//float rfad_coeffs[] = {-0.028266671092545680f, -0.015434099282916889f, 0.006130551418540303f, 0.034501122581089810f, 0.066388083339242496f, 0.097634984779482167f, 0.123898401426238480f, 0.141387032927829254f, 0.147521187806080117f, 0.141387032927829254f, 0.123898401426238480f, 0.097634984779482167f, 0.066388083339242496f, 0.034501122581089810f, 0.006130551418540303f, -0.015434099282916889f, -0.028266671092545680f};
	//ffir_init(&(as->frad), rfad_coeffs, 17); // !!!set FFIR_SIZE (20)
	// filter, use https://fiiir.com for coefficients
	// sampling rate: 125 Hz (~8 ms), curoff freq: 20 Hz, transition bandwidth: 20 Hz, window: rectangular
	float rfad_coeffs[] = {0.011345408975133899f, 0.122860066760245426f, 0.229290636286789890f, 0.273007775955661613f, 0.229290636286789890f, 0.122860066760245426f, 0.011345408975133899f};
	ffir_init(&(as->frad), rfad_coeffs, 7);

	return true;
}

uint16_t angle_sensor_set_enable_pin(angle_sens_t *as, GPIO_TypeDef *port, uint32_t pm) {
	if(as == NULL) {
		// error, invalid sensor
		return false;
	}
	as->as_en.gpio_port = port;
	as->as_en.pin_mask = pm;
	LL_GPIO_SetOutputPin(as->as_en.gpio_port, as->as_en.pin_mask);
	return true;
}
uint16_t angle_sensor_enable(angle_sens_t *as) {
	if(as == NULL) {
		// error, invalid sensor
		return false;
	}
	LL_GPIO_SetOutputPin(as->as_en.gpio_port, as->as_en.pin_mask);
	return true;
}

uint16_t angle_sensor_disable(angle_sens_t *as) {
	if(as == NULL) {
		// error, invalid sensor
		return false;
	}
	LL_GPIO_ResetOutputPin(as->as_en.gpio_port, as->as_en.pin_mask);
	return true;
}

static inline uint16_t _u16hl(uint8_t h, uint8_t l) {
	return (h * 256 + l);
}

uint16_t angle_sensor_get(angle_sens_t *as) {
    if(as == NULL) {
        // error, invalid sensor
        return false;
    }
    as->i2c.buf[0] = 0x0C;
    as->i2c.buf_len = 1;
    HAL_I2C_Master_Transmit(as->i2c.hi2c, (as->i2c.addr<<1|0), as->i2c.buf, 1, 10000);
    as->i2c.buf[0] = 0xFF;
	as->i2c.buf[1] = 0xFF;
	HAL_I2C_Master_Receive(as->i2c.hi2c, (as->i2c.addr<<1|1), as->i2c.buf, 4, 10000);
    //as->i2c.buf_len = (as->i2c.hi2c, as->i2c.addr, as->i2c.buf, 2);
    //as->raw_angle = ((uint16_t*)as->i2c.buf)[0];// AS5600 ist big endeian, STM32 is little endian
	as->raw_angle = _u16hl(as->i2c.buf[0], as->i2c.buf[1]);
    as->angle_rad = (float)(2*M_PI * as->raw_angle)/4096;
    //as->raw_angle = _u16hl(as->i2c.buf[2], as->i2c.buf[3]);

    ffir_filter(&(as->frad), as->angle_rad);
    as->angle_rad_filtered = as->frad.filtered_value;
    return true;
}
