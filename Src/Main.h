/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  GPS_DISCONNECTED = 0,   /* no valid NMEA data at all        -> LED OFF   */
  GPS_NO_FIX,             /* NMEA ok, RMC status 'V' (void)   -> LED ON    */
  GPS_FIX_OK              /* NMEA ok, RMC status 'A' (valid)  -> LED BLINK */
} gps_state_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* ----- LED (on-board LED at PA6) ----- */
#define GPS_LED_PORT          GPIOA
#define GPS_LED_PIN           GPIO_PIN_6
#define GPS_LED_ACTIVE_LOW    1u    /* 1 = LED lights when PA6 is LOW (normal
                                       on the STM32F407VET6 black board).
                                       If LED works inverted, set this to 0.  */

/* ----- GPS behaviour ----- */
#define GPS_TIMEOUT_MS        2000u /* no valid NMEA for this long => GPS
                                       treated as DISCONNECTED                */
#define GPS_BLINK_MS          500u  /* LED toggle period when fix is valid    */
#define NMEA_MAX_LEN          100u  /* NMEA sentence is max 82 chars          */
#define GPS_RING_SIZE         256u  /* UART receive buffer, power of 2 only   */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#if GPS_LED_ACTIVE_LOW
  #define GPS_LED_ON()   HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_RESET)
  #define GPS_LED_OFF()  HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_SET)
#else
  #define GPS_LED_ON()   HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_SET)
  #define GPS_LED_OFF()  HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_RESET)
#endif
#define GPS_LED_TOGGLE() HAL_GPIO_TogglePin(GPS_LED_PORT, GPS_LED_PIN)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
