/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_SCK___BMP_Pin LL_GPIO_PIN_2
#define SPI4_SCK___BMP_GPIO_Port GPIOE
#define GPIO_AUX___IN1_Pin LL_GPIO_PIN_3
#define GPIO_AUX___IN1_GPIO_Port GPIOE
#define SPI4_CS___BMP_Pin LL_GPIO_PIN_4
#define SPI4_CS___BMP_GPIO_Port GPIOE
#define SPI4_MISO___BMP_Pin LL_GPIO_PIN_5
#define SPI4_MISO___BMP_GPIO_Port GPIOE
#define SPI4_MOSI___BMP_Pin LL_GPIO_PIN_6
#define SPI4_MOSI___BMP_GPIO_Port GPIOE
#define GPIO_AUX___IN2_Pin LL_GPIO_PIN_13
#define GPIO_AUX___IN2_GPIO_Port GPIOC
#define GPIO_AUX___OUT1_Pin LL_GPIO_PIN_14
#define GPIO_AUX___OUT1_GPIO_Port GPIOC
#define GPIO_AUX___OUT2_Pin LL_GPIO_PIN_15
#define GPIO_AUX___OUT2_GPIO_Port GPIOC
#define ADC3_IN10___AUX_3_Pin LL_GPIO_PIN_0
#define ADC3_IN10___AUX_3_GPIO_Port GPIOC
#define ADC2_IN11___6V_Pin LL_GPIO_PIN_1
#define ADC2_IN11___6V_GPIO_Port GPIOC
#define ADC3_IN0___AUX_1_Pin LL_GPIO_PIN_2
#define ADC3_IN0___AUX_1_GPIO_Port GPIOC
#define ADC3_IN1___AUX_2_Pin LL_GPIO_PIN_3
#define ADC3_IN1___AUX_2_GPIO_Port GPIOC
#define TIM2_CH1_PWM___SERVO1_Pin LL_GPIO_PIN_0
#define TIM2_CH1_PWM___SERVO1_GPIO_Port GPIOA
#define TIM5_CH2_PWM___SERVO2_Pin LL_GPIO_PIN_1
#define TIM5_CH2_PWM___SERVO2_GPIO_Port GPIOA
#define ADC2_IN14___5V_Pin LL_GPIO_PIN_2
#define ADC2_IN14___5V_GPIO_Port GPIOA
#define ADC2_IN15___12V_Pin LL_GPIO_PIN_3
#define ADC2_IN15___12V_GPIO_Port GPIOA
#define SPI1_NSS___SDCARD_Pin LL_GPIO_PIN_4
#define SPI1_NSS___SDCARD_GPIO_Port GPIOA
#define SPI1_SCK___SDCARD_Pin LL_GPIO_PIN_5
#define SPI1_SCK___SDCARD_GPIO_Port GPIOA
#define ADC1_IN3___ANG_RATE_Pin LL_GPIO_PIN_6
#define ADC1_IN3___ANG_RATE_GPIO_Port GPIOA
#define SPI1_MOSI___SDCARD_Pin LL_GPIO_PIN_7
#define SPI1_MOSI___SDCARD_GPIO_Port GPIOA
#define ADC2_IN4___BAT_Pin LL_GPIO_PIN_4
#define ADC2_IN4___BAT_GPIO_Port GPIOC
#define ADC2_IN8___GAUG1_Pin LL_GPIO_PIN_5
#define ADC2_IN8___GAUG1_GPIO_Port GPIOC
#define ADC2_IN9___GAUG2_Pin LL_GPIO_PIN_0
#define ADC2_IN9___GAUG2_GPIO_Port GPIOB
#define ADC2_IN5___GAUG3_Pin LL_GPIO_PIN_1
#define ADC2_IN5___GAUG3_GPIO_Port GPIOB
#define SPI3_MOSI___AUX_Pin LL_GPIO_PIN_2
#define SPI3_MOSI___AUX_GPIO_Port GPIOB
#define GPIO_IN___SDCARD_DETECT_Pin LL_GPIO_PIN_7
#define GPIO_IN___SDCARD_DETECT_GPIO_Port GPIOE
#define GPIO_IN___PIANO3_Pin LL_GPIO_PIN_8
#define GPIO_IN___PIANO3_GPIO_Port GPIOE
#define GPIO_IN___PIANO4_Pin LL_GPIO_PIN_9
#define GPIO_IN___PIANO4_GPIO_Port GPIOE
#define GPIO_IN___BTN1_Pin LL_GPIO_PIN_10
#define GPIO_IN___BTN1_GPIO_Port GPIOE
#define GPIO_OUT___LED_1_Pin LL_GPIO_PIN_11
#define GPIO_OUT___LED_1_GPIO_Port GPIOE
#define GPIO_OUT___LED_2_Pin LL_GPIO_PIN_12
#define GPIO_OUT___LED_2_GPIO_Port GPIOE
#define GPIO_OUT___LED_3_Pin LL_GPIO_PIN_13
#define GPIO_OUT___LED_3_GPIO_Port GPIOE
#define GPIO_OUT___LED_4_Pin LL_GPIO_PIN_14
#define GPIO_OUT___LED_4_GPIO_Port GPIOE
#define GPIO_OUT___MAGNET_Pin LL_GPIO_PIN_15
#define GPIO_OUT___MAGNET_GPIO_Port GPIOE
#define I2C2_SCL___AUX_Pin LL_GPIO_PIN_10
#define I2C2_SCL___AUX_GPIO_Port GPIOB
#define I2C2_SDA___AUX_Pin LL_GPIO_PIN_11
#define I2C2_SDA___AUX_GPIO_Port GPIOB
#define SPI2_NSS___IMU_Pin LL_GPIO_PIN_12
#define SPI2_NSS___IMU_GPIO_Port GPIOB
#define SPI2_SCK___IMU_Pin LL_GPIO_PIN_13
#define SPI2_SCK___IMU_GPIO_Port GPIOB
#define SPI2_MISO___IMU_Pin LL_GPIO_PIN_14
#define SPI2_MISO___IMU_GPIO_Port GPIOB
#define SPI2_MOSI___IMU_Pin LL_GPIO_PIN_15
#define SPI2_MOSI___IMU_GPIO_Port GPIOB
#define USART3_TX___AUX_Pin LL_GPIO_PIN_8
#define USART3_TX___AUX_GPIO_Port GPIOD
#define USART3_RX___AUX_Pin LL_GPIO_PIN_9
#define USART3_RX___AUX_GPIO_Port GPIOD
#define TIM4_CH3_PWM___SERVO3_Pin LL_GPIO_PIN_14
#define TIM4_CH3_PWM___SERVO3_GPIO_Port GPIOD
#define TIM3_CH1_PWM___BUZZER_Pin LL_GPIO_PIN_6
#define TIM3_CH1_PWM___BUZZER_GPIO_Port GPIOC
#define USART1_TX___GPS_Pin LL_GPIO_PIN_9
#define USART1_TX___GPS_GPIO_Port GPIOA
#define USART1_RX___GPS_Pin LL_GPIO_PIN_10
#define USART1_RX___GPS_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI3_SCK___AUX_Pin LL_GPIO_PIN_10
#define SPI3_SCK___AUX_GPIO_Port GPIOC
#define SPI3_MISO___AUX_Pin LL_GPIO_PIN_11
#define SPI3_MISO___AUX_GPIO_Port GPIOC
#define USART2_TX___WIFI_Pin LL_GPIO_PIN_5
#define USART2_TX___WIFI_GPIO_Port GPIOD
#define USART2_RX___WIFI_Pin LL_GPIO_PIN_6
#define USART2_RX___WIFI_GPIO_Port GPIOD
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SPI1_MISO___SDCARD_Pin LL_GPIO_PIN_4
#define SPI1_MISO___SDCARD_GPIO_Port GPIOB
#define I2C1_SCL___INTERN_Pin LL_GPIO_PIN_6
#define I2C1_SCL___INTERN_GPIO_Port GPIOB
#define I2C1_SDA___INTERN_Pin LL_GPIO_PIN_7
#define I2C1_SDA___INTERN_GPIO_Port GPIOB
#define GPIO_OUT___CTRL_12V_Pin LL_GPIO_PIN_1
#define GPIO_OUT___CTRL_12V_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
