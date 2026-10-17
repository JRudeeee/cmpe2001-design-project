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
#include "LCD1602_I2C.h"
#include "buttons.h"
#include "oximeter.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "sss_detect.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POLL_MS       250u /* sensor/display update period (ms) */
#define HIST_N        5    /* readings that must agree before HR is shown */
#define HR_SPREAD_MAX 12   /* max spread (bpm) across those readings */
#define LOST_GRACE    12   /* bad readings before finger counts as gone (3 s) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Pad to 16 chars so old text is overwritten */
static void pad16(char *s) {
  size_t n = strlen(s);
  while (n < 16u) {
    s[n++] = ' ';
  }
  s[16] = '\0';
}

/* HR display filter: show the median once the last HIST_N readings agree */
typedef struct {
  int32_t buf[HIST_N];
  uint8_t cnt;
  uint8_t idx;
} Hist;

static Hist s_hr_hist;
static int32_t s_hr_shown = -1;
static int32_t s_spo2_shown = -1;
static uint16_t s_lost = 0;

static void hist_reset(Hist *h) {
  h->cnt = 0;
  h->idx = 0;
}

static void hist_push(Hist *h, int32_t v) {
  h->buf[h->idx] = v;
  h->idx = (uint8_t)((h->idx + 1u) % HIST_N);
  if (h->cnt < HIST_N) {
    h->cnt++;
  }
}

/* Median if history is full and settled, else -1 */
static int32_t hist_stable(const Hist *h, int32_t spread_max) {
  int32_t t[HIST_N];
  uint8_t i, j;

  if (h->cnt < HIST_N) {
    return -1;
  }
  for (i = 0; i < HIST_N; i++) {
    t[i] = h->buf[i];
  }
  for (i = 1; i < HIST_N; i++) { /* insertion sort */
    int32_t k = t[i];
    j = i;
    while (j > 0u && t[j - 1] > k) {
      t[j] = t[j - 1];
      j--;
    }
    t[j] = k;
  }
  if ((t[HIST_N - 1] - t[0]) > spread_max) {
    return -1;
  }
  return t[HIST_N / 2];
}

typedef enum { ST_NO_FINGER, ST_MEASURING, ST_SHOW } DispState;

static DispState hr_step(int32_t raw_hr, int32_t raw_spo2) {
  if (raw_hr <= 0) {
    s_lost++;
    if (s_lost > LOST_GRACE) { /* finger really gone */
      hist_reset(&s_hr_hist);
      s_hr_shown = -1;
      s_spo2_shown = -1;
      return ST_NO_FINGER;
    }
    /* within grace period: hold what's on screen */
    if (s_hr_shown > 0) {
      return ST_SHOW;
    }
    if (s_hr_hist.cnt > 0) {
      return ST_MEASURING;
    }
    return ST_NO_FINGER;
  }

  s_lost = 0;
  hist_push(&s_hr_hist, raw_hr);
  {
    int32_t v = hist_stable(&s_hr_hist, HR_SPREAD_MAX);
    if (v > 0) {
      s_hr_shown = v;
      if (raw_spo2 > 0) {
        s_spo2_shown = raw_spo2;
      }
      return ST_SHOW;
    }
  }
  if (s_hr_shown > 0) {
    return ST_SHOW;
  } /* settled already, one bad tick */
  return ST_MEASURING;
}

static void build_lines(DispState st, uint8_t comm_ok, uint8_t spin, char *l1,
                        char *l2) {
  static const char *dots[4] = {"   ", ".  ", ".. ", "..."};

  if (!comm_ok) {
    strcpy(l1, "Sensor offline");
    strcpy(l2, "Check wiring");
  } else if (st == ST_NO_FINGER) {
    snprintf(l1, 17, "Place finger%s", dots[spin & 3u]);
    l2[0] = '\0';
  } else if (st == ST_MEASURING) {
    snprintf(l1, 17, "Measuring%s", dots[spin & 3u]);
    strcpy(l2, "Hold still");
  } else {
    int sp = (s_spo2_shown > 0 && s_spo2_shown <= 100) ? (int)s_spo2_shown : 0;
    int hr = (s_hr_shown > 0 && s_hr_shown <= 999) ? (int)s_hr_shown : 0;
    snprintf(l1, 17, "HR   %3d bpm", hr);
    snprintf(l2, 17, "SpO2 %3d %%", sp);
  }
  pad16(l1);
  pad16(l2);
}

/* Screens, menu items and settings */
typedef enum { UI_HOME, UI_MENU, UI_ALARM } UiState;
typedef enum { MI_LED, MI_SOUND, MI_SIM, MI_COUNT } MenuItem;

static UiState s_ui = UI_HOME;
static uint8_t s_cursor = 0;
static uint8_t s_led_alert = 1;
static uint8_t s_sound_alert = 1;
static uint8_t s_sim_mode = 0;

/* Button actions for the current screen */
static void ui_handle_buttons(void) {
    uint8_t menu = Buttons_Pressed(BTN_MENU);
    uint8_t back = Buttons_Pressed(BTN_BACK);
    uint8_t up = Buttons_Pressed(BTN_UP);
    uint8_t down = Buttons_Pressed(BTN_DOWN);

    switch (s_ui) {
        case UI_HOME:
            if (menu) {
                s_ui = UI_MENU;
                s_cursor = 0;
            }
            break;
        
        case UI_MENU:
            if (down) {
                s_cursor = (s_cursor + 1) % MI_COUNT;
            }
            if (up) {
                s_cursor = (s_cursor + MI_COUNT - 1) % MI_COUNT;
            }
            if (back) {
                s_ui = UI_HOME;
                s_cursor = 0;
            }
            

            else if (menu) {
                switch (s_cursor) {
                    case MI_LED:
                        s_led_alert = !s_led_alert;
                        break;
                    case MI_SOUND:
                        s_sound_alert = !s_sound_alert;                    
                        break;
                    case MI_SIM:
                        s_sim_mode = !s_sim_mode;
                        SSS_Init();
                        break;
                }
            }
            break;
        
        case UI_ALARM:
            if (menu) {
                SSS_Acknowledge(HAL_GetTick());
                s_ui = UI_HOME;
                s_cursor = 0;
            }
            break;
    }
}

/* Text for one menu item */
static void menu_label(uint8_t item, char *out) {
    switch (item) {
        case MI_LED:
            snprintf(out, 15, "LED alert %s", s_led_alert ? " ON" : "OFF");
            break;
        case MI_SOUND:
            snprintf(out, 15, "Sound     %s", s_sound_alert ? " ON" : "OFF");
            break;
        case MI_SIM:
            snprintf(out, 15, "Simulate  %s", s_sim_mode ? " ON" : "OFF");
            break;
    }
}

/* Set piezo frequency (timer ticks at 1 MHz) */
static void tone_set(uint32_t freq){
    uint32_t arr = 1000000u / freq -1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (arr + 1) / 2); 
}

/* LED triple flash and two-tone siren, gated by the menu settings */
static void alarm_output(uint8_t led_on, uint8_t sound_on){
    uint32_t now = HAL_GetTick();
    uint32_t led_phase = now % 1000;
    uint32_t tone_phase = now % 500;
    static uint32_t current_freq = 0;
    static uint8_t pwm_status = 0;
    
    if(led_on && s_led_alert){
        if (led_phase < 500){
            if (led_phase % 200 < 100){
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            }
        } else {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    if (sound_on && s_sound_alert) {
        if (pwm_status == 0) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
            pwm_status = 1;
        }
        if(tone_phase < 250) {
            if (current_freq != 3000){
                current_freq = 3000;
                tone_set(current_freq);
            }
        } else if (current_freq != 4000){
            current_freq = 4000;
            tone_set(4000);
        }
    } else if (pwm_status == 1) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        pwm_status = 0;
        current_freq = 0;
    }
}

/* Fake erratic BPM for the demo, new value every 4 s */
static float sim_bpm(void){
    static const float seq[] = { 60, 95, 58, 100, 65, 92, 55, 95 };

    uint8_t index = (HAL_GetTick() / 4000) % 8;

    return seq[index];
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  SSS_Init();
  lcd_init(&hi2c1);
  lcd_put_cur(0, 0);
  lcd_send_string(Oxi_IsPresent(&hi2c3) ? "HR OK" : "HR missing");
  Oxi_Start(&hi2c3);
  hist_reset(&s_hr_hist);
  HAL_Delay(1000);

  uint32_t last = 0;
  uint8_t spin = 0;
  uint32_t t_btn = 0;
  char line1[17], line2[17];
  char prev1[17] = "", prev2[17] = "";
  char a[15], b[15];

  /* Start-up beeps */
  tone_set(1000);
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_Delay(150);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_Delay(150);
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time
   * it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Buttons every 10 ms */
    if (HAL_GetTick() - t_btn >= 10) {
      t_btn = HAL_GetTick();
      Buttons_Task();
      ui_handle_buttons();
    }
    /* Sensor, detector and display every POLL_MS */
    if (HAL_GetTick() - last >= POLL_MS) {
      last = HAL_GetTick();

      OxiReading rd;
      uint8_t ok = (Oxi_Read(&rd) == HAL_OK) ? 1u : 0u;
      DispState st = ok ? hr_step(rd.heartbeat, rd.spo2) : ST_NO_FINGER;

      /* Real or simulated input to the detector */
      float bpm = s_sim_mode ? sim_bpm() : (float)rd.heartbeat;
      bool finger = s_sim_mode ? true : (st != ST_NO_FINGER);

      SSS_OnReading(HAL_GetTick(), bpm);
      SSS_Update(HAL_GetTick(), finger);

      const SSS_Status *s = SSS_GetStatus();

      /* Show the alarm screen when the piezo should sound */
      if (s->sound && s_ui == UI_HOME) {
        s_ui = UI_ALARM;
      }

      /* Build the two LCD lines for the current screen */
      if (s_ui == UI_MENU) {
        menu_label(s_cursor, a);
        menu_label((s_cursor + 1) % MI_COUNT, b);
        
        snprintf(line1, 17, "> %s", a);
        snprintf(line2, 17, "  %s", b);

        pad16(line1);
        pad16(line2);
      } else if (s_ui == UI_ALARM) {
        SSS_GetText(line1);
        strcpy(line2, "Menu = silence");
        pad16(line1);
        pad16(line2);
      } else {
        if (s_sim_mode){
            SSS_GetText(line1);
            strcpy(line2, "** SIMULATION **");
            pad16(line1);
            pad16(line2);
        } else {
            build_lines(st, ok, spin, line1, line2);
        }
      }
      

      /* Only redraw lines that changed */
      if (strncmp(prev1, line1, 16) != 0) {
        lcd_put_cur(0, 0);
        lcd_send_string(line1);
        strncpy(prev1, line1, sizeof prev1);
      }
      if (strncmp(prev2, line2, 16) != 0) {
        lcd_put_cur(1, 0);
        lcd_send_string(line2);
        strncpy(prev2, line2, sizeof prev2);
      }
      spin++;
    }
    /* Alarm outputs run every pass so the patterns stay smooth */
    const SSS_Status *s = SSS_GetStatus();
    alarm_output(s->alarm, s->sound);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_1_Pin Button_2_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin | Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_3_Pin Button_4_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin | Button_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
