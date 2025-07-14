/**
 * Martin Egli
 * 2025-06-30
 * angle sensor module
 */

#include "angle_sensor.h"
//#include "i2c.h"

// - public functions ----------------------------------------------------------
#if 0
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

	return true;
}

static inline _u16hl(uint8_t h, uint8_t l) {
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
    as->angle_deg = (float)(360.0f * as->raw_angle)/4096;
    as->angle_rad = (float)(2*M_PI * as->raw_angle)/4096;
    //as->raw_angle = _u16hl(as->i2c.buf[2], as->i2c.buf[3]);

    return true;
}
#endif

/*
uint16_t angle_sensor_init(angle_sens_t *as, I2C_TypeDef *hi2c) {
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
    return true;
}

uint16_t angle_sensor_get(angle_sens_t *as) {
    uint8_t buf[8];
    if(as == NULL) {
        // error, invalid sensor
        return false;
    }
    as->i2c.buf[0] = 0x0C;
    as->i2c.buf_len = 1;
    i2c_wr_blocking(as->i2c.hi2c, as->i2c.addr, as->i2c.buf, as->i2c.buf_len);
    as->i2c.buf_len = i2c_rd_blocking(as->i2c.hi2c, as->i2c.addr, as->i2c.buf, 2);
    as->raw_angle = ((uint16_t*)&buf[0])[0];
    as->angle_deg = (360 * as->raw_angle)/4096;
    return true;
}
*/
