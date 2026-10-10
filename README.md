# INTERRUPT_DATA_ACQUISITION_GPS_STM32
A bare-metal (HAL) project that reads NMEA data from a u-blox NEO-6M GPS module over USART2 (interrupt driven).

| GPS situation | What the MCU sees | LED (PA6) |
|---|---|---|
| GPS not connected / no power / wrong baud rate | No valid NMEA sentence for 2 s | **OFF** |
| GPS connected, still searching satellites or data invalid | Valid NMEA, `$GPRMC` status = `V` | **ON (solid)** |
| GPS has a valid position fix | Valid NMEA, `$GPRMC` status = `A` | **TOGGLES** (every 500 ms) |

No serial monitor or extra hardware is needed. The LED is the only output.

---

## 1. What you need

**Hardware**
- STM32F407VET6 development board (LED on **PA6**)
- NEO-6M GPS module (GY-GPS6MV2) with antenna
- SWD programmer, e.g. ST-Link V2, connected to the board's SWD header (SWDIO, SWCLK, GND, 3V3)
- 4 jumper wires, USB cable for board power

**Software**
- STM32CubeIDE (includes CubeMX and the STM32F4 HAL package)

---

## 2. Wiring

| NEO-6M pin | STM32F407 pin | Note |
|---|---|---|
| VCC | 3V3 | Module has its own regulator (3.3 V to 5 V accepted) |
| GND | GND | **Common ground is mandatory** |
| TX | **PA3** (USART2_RX) | GPS to MCU data line |
| RX | PA2 (USART2_TX) | Optional, the firmware never transmits |

> The GPS TX line is 3.3 V logic, which is safe for the STM32 pin.

---

## 3. CubeMX / .ioc configuration

Create a new project for **STM32F407VETx** and set:

| Item | Setting |
|---|---|
| **SYS → Debug** | **Serial Wire** (otherwise you may lock yourself out of re-flashing) |
| **PA6** | `GPIO_Output` (push-pull, no pull, low speed) |
| **USART2 → Mode** | Asynchronous |
| **USART2 → Parameters** | Baud **9600**, Word length 8 bits, Parity None, Stop bits 1, Mode RX/TX, Oversampling 16 |
| **NVIC Settings** | ☑ **USART2 global interrupt** enabled |

Then click **Generate Code**.

---

## 4. Add the firmware

Do it in **one of two ways**:

- **Option A, quickest:** replace `Core/Inc/main.h` and `Core/Src/main.c` with the two finished files from this repo. Nothing else to do.
- **Option B, paste into the auto-generated code:** keep the file CubeMX generated and paste only the snippets below (4.1 and 4.2).


### 4.1 `Core/Inc/main.h`

#### 4.1.1 `USER CODE BEGIN Includes`

Paste between `/* USER CODE BEGIN Includes */` and `/* USER CODE END Includes */`:

```c
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
```

#### 4.1.2 `USER CODE BEGIN ET`

Paste between `/* USER CODE BEGIN ET */` and `/* USER CODE END ET */`:

```c
typedef enum
{
  GPS_DISCONNECTED = 0,   /* no valid NMEA data at all        -> LED OFF   */
  GPS_NO_FIX,             /* NMEA ok, RMC status 'V' (void)   -> LED ON    */
  GPS_FIX_OK              /* NMEA ok, RMC status 'A' (valid)  -> LED BLINK */
} gps_state_t;
```

#### 4.1.3 `USER CODE BEGIN EC`

Paste between `/* USER CODE BEGIN EC */` and `/* USER CODE END EC */`:

```c
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
```

#### 4.1.4 `USER CODE BEGIN EM`

Paste between `/* USER CODE BEGIN EM */` and `/* USER CODE END EM */`:

```c
#if GPS_LED_ACTIVE_LOW
  #define GPS_LED_ON()   HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_RESET)
  #define GPS_LED_OFF()  HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_SET)
#else
  #define GPS_LED_ON()   HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_SET)
  #define GPS_LED_OFF()  HAL_GPIO_WritePin(GPS_LED_PORT, GPS_LED_PIN, GPIO_PIN_RESET)
#endif
#define GPS_LED_TOGGLE() HAL_GPIO_TogglePin(GPS_LED_PORT, GPS_LED_PIN)
```

### 4.2 `Core/Src/main.c`

#### 4.2.1 `USER CODE BEGIN PV`

Paste between `/* USER CODE BEGIN PV */` and `/* USER CODE END PV */`:

```c
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
```

#### 4.2.2 `USER CODE BEGIN PFP`

Paste between `/* USER CODE BEGIN PFP */` and `/* USER CODE END PFP */`:

```c
static void gps_init(void);
static void gps_task(void);
```

#### 4.2.3 `USER CODE BEGIN 0`

Paste between `/* USER CODE BEGIN 0 */` and `/* USER CODE END 0 */`:

```c
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
```

#### 4.2.4 `USER CODE BEGIN 2`

Paste between `/* USER CODE BEGIN 2 */` and `/* USER CODE END 2 */`:

```c
  gps_init();
  
```

#### 4.2.5 `USER CODE BEGIN 3`

Paste between `/* USER CODE BEGIN 3 */` and `/* USER CODE END 3 */`:

```c
    gps_task();
  }
  
```

**Where blocks 2 and 3 sit inside `main()`**

```c
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  gps_init();                 // <- block 2
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    gps_task();               // <- block 3
  }
```

**Check once.** `Core/Src/stm32f4xx_it.c` must contain the following. CubeMX creates it automatically when the USART2 NVIC box is ticked.

```c
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
```


---

## 5. Build and flash

1. **Project → Build Project** (Ctrl+B). Expect 0 errors, 0 warnings from `main.c`.
2. Connect the ST-Link to the SWD header and power the board.
3. **Run → Run** (or the green Run button) to flash.
4. Connect the GPS module as per section 2.

---

---

## 7. How it works

```
NEO-6M TX → PA3 (USART2 RX) → RX interrupt → ring buffer → main-loop parser → state → LED PA6
```

1. The GPS sends NMEA text at 9600 baud, once per second, even without satellites.
2. Each received byte triggers `HAL_UART_RxCpltCallback()`, which only stores it in a 256-byte ring buffer and re-arms reception. The interrupt stays very short.
3. `gps_task()` runs in the main loop. It empties the ring buffer and builds sentences from `$` to end-of-line.
4. A sentence is accepted only if its **XOR checksum** is correct, so noise or a wrong baud rate can never fake a "connected" GPS.
5. Any valid sentence refreshes a 2 s watchdog (connected or not).
6. For `$GPRMC` / `$GNRMC`, the status field decides the fix:

   ```
   $GPRMC,,V,,,,,,,,,,N*53                                                       ← no fix  (V)
   $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A          ← fix OK  (A)
   ```
## I uploaded the zip file of this project in case there is any issue. just extract the zip file and import in the stmcube IDE for the reference.
