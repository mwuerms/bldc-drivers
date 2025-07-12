/**
 * Martin Egli
 * 2025-06-29
 * bldc driver module
 * use STM32 LL functions
 */

#include "bldc_driver.h"

uint16_t bldc_driver_init(bldc_driver_t *d, TIM_TypeDef *tim) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    d->tim = tim;
    return true;
}

uint16_t bldc_driver_set_enable_pin(bldc_driver_t *d, GPIO_TypeDef *port, uint32_t pm) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    d->mot_en.gpio_port = port;
    d->mot_en.pin_mask = pm;
    LL_GPIO_SetOutputPin(d->mot_en.gpio_port, d->mot_en.pin_mask);
    return true;
}

uint16_t bldc_driver_enable(bldc_driver_t *d) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    uint32_t pwm_max = LL_TIM_GetAutoReload(d->tim);
    LL_TIM_OC_SetMode(d->tim, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetCompareCH1(d->tim, pwm_max/2); // 50 % -> stay still
	LL_TIM_OC_SetCompareCH2(d->tim, pwm_max/2);
	LL_TIM_OC_SetCompareCH3(d->tim, pwm_max/2);
	LL_TIM_EnableAllOutputs(d->tim);
    LL_TIM_CC_EnableChannel(d->tim, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N);
	LL_TIM_EnableCounter(d->tim);
	//LL_GPIO_SetOutputPin(d->mot_en.gpio_port, d->mot_en.pin_mask);
    return true;
}

uint16_t bldc_driver_disable(bldc_driver_t *d) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    //LL_GPIO_ResetOutputPin(d->mot_en.gpio_port, d->mot_en.pin_mask);
    LL_TIM_DisableCounter(d->tim);
    LL_TIM_OC_SetMode(d->tim, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N, LL_TIM_OCMODE_FORCED_INACTIVE);
    LL_TIM_OC_SetCompareCH1(d->tim, 0);
	LL_TIM_OC_SetCompareCH2(d->tim, 0);
	LL_TIM_OC_SetCompareCH3(d->tim, 0);
    // keep channels + outputs enabled, so they stay low
	LL_TIM_EnableAllOutputs(d->tim);
    LL_TIM_CC_EnableChannel(d->tim, LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N);
    return true;
}

uint16_t bldc_driver_set_phase_voltages(bldc_driver_t *d, float uu, float uv, float uw) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    uint32_t pwm_max = LL_TIM_GetAutoReload(d->tim);
    // phase voltages: 0.0 ... 1.0f
    uint16_t pwm_u = uu * pwm_max;
    uint16_t pwm_v = uv * pwm_max;
    uint16_t pwm_w = uw * pwm_max;

    LL_TIM_OC_SetCompareCH1(d->tim, pwm_u);
	LL_TIM_OC_SetCompareCH2(d->tim, pwm_v);
	LL_TIM_OC_SetCompareCH3(d->tim, pwm_w);
    return true;
}

uint16_t bldc_driver_set_pwm(bldc_driver_t *d, uint16_t pwm_u, uint16_t pwm_v, uint16_t pwm_w) {
    if(d == NULL) {
        // error, invalid driver
        return false;
    }
    LL_TIM_OC_SetCompareCH1(d->tim, pwm_u);
	LL_TIM_OC_SetCompareCH2(d->tim, pwm_v);
	LL_TIM_OC_SetCompareCH3(d->tim, pwm_w);
    return true;
}
