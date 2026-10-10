/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static char        nmea_line[NMEA_MAX_LEN];   /* sentence being assembled   */
static uint16_t    nmea_len;                  /* 0 = not inside a sentence  */

static bool        gps_seen;                  /* got >= 1 valid sentence    */
static uint32_t    last_ok_ms;                /* time of last valid one     */
static bool        fix_valid;                 /* last RMC status was 'A'    */
static gps_state_t gps_state = GPS_DISCONNECTED;

/* UART receive:  interrupt -> ring buffer -> parsed in main loop */
static uint8_t           rx_byte;                 /* 1-byte IT receive buffer  */
static volatile uint8_t  ring[GPS_RING_SIZE];     /* filled by the ISR         */
static volatile uint16_t ring_head;               /* written by ISR only       */
static volatile uint16_t ring_tail;               /* written by main loop only */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void gps_init(void);
static void gps_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------------------------------------------------------------------------- *
 *  NMEA helpers
 * ------------------------------------------------------------------------- */
static int hex_val(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* Sentence looks like "$GPRMC,....*6A".
 * XOR of all characters between '$' and '*' must equal the hex value after '*'. */
static bool nmea_checksum_ok(const char *s)
{
  uint8_t calc = 0;

  if (*s != '$') return false;
  s++;
  while (*s != '\0' && *s != '*') calc ^= (uint8_t)(*s++);
  if (*s != '*') return false;

  int hi = hex_val(s[1]);
  if (hi < 0) return false;
  int lo = hex_val(s[2]);
  if (lo < 0) return false;

  return calc == (uint8_t)((hi << 4) | lo);
}

/* $GPRMC,time,STATUS,lat,N,lon,E,...  ->  returns the STATUS character */
static char rmc_status(const char *s)
{
  const char *p = strchr(s, ',');          /* after "$GPRMC" */
  if (p == NULL) return 0;
  p = strchr(p + 1, ',');                  /* after time     */
  if (p == NULL) return 0;
  return p[1];                             /* 'A', 'V' or ',' (empty field) */
}

/* Called once for every complete line received from the GPS */
static void gps_handle_sentence(const char *s)
{
  if (!nmea_checksum_ok(s)) return;        /* garbage / wrong baud -> ignore */

  gps_seen   = true;                       /* GPS is alive and talking       */
  last_ok_ms = HAL_GetTick();

  if (strncmp(s + 3, "RMC", 3) == 0)       /* matches $GPRMC and $GNRMC      */
  {
    fix_valid = (rmc_status(s) == 'A');
  }
}

/* Builds a sentence character by character */
static void gps_feed_char(char c)
{
  if (c == '$')                            /* start of sentence: (re)sync    */
  {
    nmea_line[0] = '$';
    nmea_len     = 1;
    return;
  }

  if (nmea_len == 0) return;               /* not inside a sentence          */

  if (c == '\r' || c == '\n')              /* end of sentence                */
  {
    nmea_line[nmea_len] = '\0';
    nmea_len = 0;
    gps_handle_sentence(nmea_line);
    return;
  }

  if (nmea_len < (NMEA_MAX_LEN - 1u)) nmea_line[nmea_len++] = c;
  else                                nmea_len = 0;     /* too long -> drop  */
}

/* ------------------------------------------------------------------------- *
 *  USART2 receive by INTERRUPT
 *  ISR (HAL callback) -> ring buffer -> parsed in the main loop.
 *  The ISR only stores one byte, so it is very short and never blocks.
 * ------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint16_t next = (uint16_t)((ring_head + 1u) & (GPS_RING_SIZE - 1u));

    if (next != ring_tail)                 /* buffer full -> drop the byte   */
    {
      ring[ring_head] = rx_byte;
      ring_head = next;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* wait for the next byte   */
  }
}

/* Overrun / framing / noise error stops reception -> start it again.
 * (A corrupted byte is rejected later by the NMEA checksum.) */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

/* Main-loop side: take bytes out of the ring buffer and parse them */
static void gps_ring_process(void)
{
  while (ring_tail != ring_head)
  {
    char c = (char)ring[ring_tail];
    ring_tail = (uint16_t)((ring_tail + 1u) & (GPS_RING_SIZE - 1u));
    gps_feed_char(c);
  }
}

/* ------------------------------------------------------------------------- *
 *  Public functions used by main()
 * ------------------------------------------------------------------------- */
static void gps_init(void)
{
  /* Pull-up on PA3 (USART2_RX): idle line stays HIGH when GPS wire is
   * unplugged, so no false start bits / noise. */
  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_3;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);

  (void)huart2.Instance->SR;               /* clear any stale flags */
  (void)huart2.Instance->DR;

  GPS_LED_OFF();                           /* start as "disconnected" */

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* start interrupt reception */
}

static void gps_task(void)
{
  static uint32_t t_blink;

  /* 1) safety net: if reception ever stopped, start it again */
  if (huart2.RxState == HAL_UART_STATE_READY)
  {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }

  /* 2) take received bytes from the ring buffer and parse them */
  gps_ring_process();

  /* 3) decide the state */
  uint32_t now = HAL_GetTick();

  if (!gps_seen || (now - last_ok_ms) > GPS_TIMEOUT_MS)
  {
    gps_state = GPS_DISCONNECTED;
    fix_valid = false;                     /* forget old fix */
  }
  else
  {
    gps_state = fix_valid ? GPS_FIX_OK : GPS_NO_FIX;
  }

  /* 4) drive the LED (non-blocking) */
  switch (gps_state)
  {
    case GPS_DISCONNECTED:
      GPS_LED_OFF();                       /* no GPS data      -> OFF    */
      break;

    case GPS_NO_FIX:
      GPS_LED_ON();                        /* data but invalid -> ON     */
      break;

    case GPS_FIX_OK:
      if ((now - t_blink) >= GPS_BLINK_MS) /* valid data       -> TOGGLE */
      {
        t_blink = now;
        GPS_LED_TOGGLE();
      }
      break;

    default:
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  gps_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    gps_task();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
