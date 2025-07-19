/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "project.h"
#include "uart.h"
#include "str_buf.h"
#include "bldc_motor.h"
#include "bldc_driver.h"
#include "angle_sensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bldc_motor_t m1;
bldc_driver_t md1;
angle_sens_t as1;
uart_t u1;

#define MAIN_STR_BUF_SIZE (256)
char main_str_buf[MAIN_STR_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void waitabit(uint32_t x) {
	while(x) x--;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t now, last_tick, waitcnt;
	float dt, sum_dt, tim_dt;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  uart_init(&u1, USART2);

  angle_sensor_init(&as1, &hi2c1);
  angle_sensor_set_enable_pin(&as1, AS_EN_GPIO_Port, AS_EN_Pin);
  angle_sensor_disable(&as1);

  bldc_driver_init(&md1, TIM1);
  //bldc_driver_set_enable_pin(&md1, MOT0_EN_GPIO_Port, MOT0_EN_Pin);

  bldc_motor_init(&m1, &md1, &as1);
  bldc_motor_set_ctrl_type(&m1, BLDC_MOTOR_CTRL_TYPE_VELOCITY_OPENLOOP);
  //bldc_motor_set_ctrl_type(&m1, BLDC_MOTOR_CTRL_TYPE_ANGLE_OPENLOOP);
  //bldc_motor_set_ctrl_type(&m1, BLDC_MOTOR_CTRL_TYPE_VELOCITY);
  bldc_motor_set_motor_parameters(&m1, 7, 360, 0.4f);

  bldc_motor_set_id_pid(&m1, 1.0f, 1.0f, 0.0f);
  bldc_motor_set_iq_pid(&m1, 1.0f, 1.0f, 0.0f);
  bldc_motor_set_angle_pid(&m1, 1.0f, 1.0f, 0.0f);
  bldc_motor_set_speed_pid(&m1, 1.0f, 1.0f, 0.0f);

  bldc_motor_set_voltage_limit(&m1, 0.25f);
  bldc_motor_set_speed_limit(&m1, 1.0f);
  bldc_motor_set_target_speed(&m1, 10.0f);
  bldc_motor_set_target_angle_deg(&m1, 90.0f);

  angle_sensor_enable(&as1);
  bldc_driver_enable(&md1);
  //bldc_driver_set_pwm(&md1, 100, 200, 300);
  bldc_motor_enable(&m1);
  m1.set.vq = 0.45f;

  //while(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  now = HAL_GetTick();
  last_tick = now;
  uart_send_string_blocking(&u1, "hello from the other side :-D\n");
  angle_sensor_get(&as1);
  str_buf_clear(main_str_buf, MAIN_STR_BUF_SIZE);
  str_buf_append_uint16(main_str_buf, MAIN_STR_BUF_SIZE, as1.raw_angle);
  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, "\n");
  uart_send_string_blocking(&u1, main_str_buf);
  waitcnt = 1;//0000;
  sum_dt = 0.0f;
  tim_dt = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  waitabit(waitcnt);

	  now = HAL_GetTick();
	  dt = (float)(now - last_tick)/1000.0f; // in ms
	  last_tick = now;

	  bldc_motor_move(&m1, dt);

	  str_buf_clear(main_str_buf, MAIN_STR_BUF_SIZE);
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, tim_dt, 6);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.el_angle_rad, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_uint16(main_str_buf, MAIN_STR_BUF_SIZE, as1.raw_angle);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, as1.angle_rad, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, as1.angle_rad_filtered, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.d_out, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.q_out, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.u_out, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.v_out, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");
	  str_buf_append_float(main_str_buf, MAIN_STR_BUF_SIZE, m1.calc.w_out, 5);
	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, ",");

	  str_buf_append_string(main_str_buf, MAIN_STR_BUF_SIZE, "\n");
	  uart_send_string_blocking(&u1, main_str_buf);


	  tim_dt += dt;
	  sum_dt += dt;
	  	  if(sum_dt > 2.0f) {
	  		  sum_dt = 0.0f;
	  		  /*
	  		  if(m1.target.speed_rot_s > 1.1f) {
	  			  bldc_motor_set_target_speed(&m1, 1.0f);
	  		  }
	  		  else {
	  			  bldc_motor_set_target_speed(&m1, 2.0f);
	  		  }
	  		  */
	  		  /*m1.calc.shaft_angle_rad = 0;
	  		  m1.calc.shaft_angle_rad_old = 0;
	  		  bldc_motor_set_target_angle_deg(&m1, 90.0f);
	  		  bldc_motor_enable(&m1);
	  		  */
	  	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
