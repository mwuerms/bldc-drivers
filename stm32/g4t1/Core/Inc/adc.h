/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
uint16_t adc_read_channel(ADC_TypeDef *adc, uint32_t ch);

#define ADC_GET_CURRENT_U() adc_read_channel(ADC1, LL_ADC_CHANNEL_3)
#define ADC_GET_CURRENT_V() adc_read_channel(ADC2, LL_ADC_CHANNEL_3)
#define ADC_GET_CURRENT_W() adc_read_channel(ADC1, LL_ADC_CHANNEL_12)

#define ADC_GET_POTI() adc_read_channel(ADC1, LL_ADC_CHANNEL_11)
#define ADC_GET_NTC() adc_read_channel(ADC1, LL_ADC_CHANNEL_5)

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

