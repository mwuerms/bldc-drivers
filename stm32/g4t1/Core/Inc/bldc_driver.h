/**
 * Martin Egli
 * 2025-06-29
 * bldc driver module
 */

#ifndef _BLDC_DRIVER_H_ 
#define _BLDC_DRIVER_H_ 

#include <stdint.h>
#include <stdbool.h>
#include "project.h"
#include "tim.h"

typedef struct {
	TIM_TypeDef *tim;
	struct {
		GPIO_TypeDef *gpio_port;
		uint32_t pin_mask;
	} mot_en;
    uint16_t channels;
    uint16_t state;
} bldc_driver_t;

uint16_t bldc_driver_init(bldc_driver_t *d, TIM_TypeDef *tim);
uint16_t bldc_driver_set_enable_pin(bldc_driver_t *d, GPIO_TypeDef *port, uint32_t pm);
uint16_t bldc_driver_enable(bldc_driver_t *d);
uint16_t bldc_driver_disable(bldc_driver_t *d);
uint16_t bldc_driver_set_phase_voltages(bldc_driver_t *d, float uu, float uv, float uw);
uint16_t bldc_driver_set_pwm(bldc_driver_t *d, uint16_t pwm_u, uint16_t pwm_v, uint16_t pwm_w);

#endif // _BLDC_DRIVER_H_
