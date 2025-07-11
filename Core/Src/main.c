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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "events.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_F1,
    STATE_F2,
    STATE_F3,
} AppState;

typedef enum { // A bit atypical, but I want to be able to read data from PC connected through UART for better UX
  KEY_0 = '0',
  KEY_1 = '1',
  KEY_2 = '2',
  KEY_3 = '3',
  KEY_4 = '4',
  KEY_5 = '5',
  KEY_6 = '6',
  KEY_7 = '7',
  KEY_8 = '8',
  KEY_9 = '9',
  KEY_Enter = 'e',
  KEY_Clear = 'c',
  KEY_BkSp = 'b',
  KEY_Start = 'S',
  KEY_Stop = 's',
  KEY_ESC = '`',
  KEY_F1 = '!',
  KEY_F2 = '@',
  KEY_F3 = '#',
  KEY_F4 = '$',
  KEY_F5 = '%',
  KEY_Dot = '.',
  KEY_Lock = 'l',
  KEY_OFF = 'f',
  KEY_ON = 'n'
} KeyboardButton;

typedef struct {
	AppState currentState;

	// F1 - screen where user enters voltage and can start/stop PWM
  uint16_t voltage; // register holding voltage that has been validated and is ready to be sent to PWM
  uint16_t inputValue; // register holding current value of "input field"
  bool isVoltageEntered; // flag if we are ready to start PWM. Maybe redundant, we can check against voltage register. But its more robust this way
  bool isPwmRunning;
  char message[64]; // ad hoc message to display

    // F2 - screen where user sets three calibration points - TODO later
    uint16_t calibration_points[3]; // For mapping
} AppContext;

const char keymap[5][5] = {
    {KEY_ESC,  KEY_3,   KEY_2,     KEY_1,     KEY_BkSp},
    {KEY_6,    KEY_5,   KEY_4,     KEY_Clear, KEY_9},
    {KEY_8,    KEY_7,   KEY_Enter, KEY_Dot,   KEY_0},
    {KEY_Lock, KEY_F5,  KEY_F4,    KEY_F3,    KEY_F2},
    {KEY_F1,   KEY_OFF, KEY_ON,    KEY_Stop,  KEY_Start}
};
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
volatile int currentRow = -1; // -1 means no row is active
volatile int lastRow = -1;
volatile int lastCol = -1;
volatile uint32_t lastTriggerTime = 0;
uint8_t receivedChar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void handle_event(AppContext *ctx, AppEvent *evt);
void renderState(AppContext *ctx);
void ClearScreen();
void setRowActive(int row);
void setAllRowsInactive();
void readFlexiKeyboard();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  setAllRowsInactive();
  HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
  setbuf(stdout, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  AppEvent evt;
  AppContext ctx;
  ctx.currentState = STATE_F1;
  ctx.isVoltageEntered = false;
  ctx.isPwmRunning = false;
  ctx.voltage = 0;
  ctx.inputValue = 0;

  renderState(&ctx);

  while (1)
  {
    if (event_queue_pop(&evt)) {
      handle_event(&ctx, &evt);
      renderState(&ctx);
    }

    readFlexiKeyboard(); // approx 25ms blocking code to scan the keyboard

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void readFlexiKeyboard()
{
  for (int row = 0; row < 5; row++)
  {
      setRowActive(row);       // Pull only one row LOW
      currentRow = row;        // Track active row
      HAL_Delay(5);            // Wait to detect keypress

      setAllRowsInactive();    // Set all rows HIGH again
      currentRow = -1;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (currentRow == -1) return; // No row active, ignore

    int col = -1;

    switch (GPIO_Pin)
    {
        case GPIO_PIN_3:  col = 0; break; // PB3
        case GPIO_PIN_5:  col = 1; break; // PB5
        case GPIO_PIN_4:  col = 2; break; // PB4
        case GPIO_PIN_10: col = 3; break; // PB10
        case GPIO_PIN_8:  col = 4; break; // PA8
    }

    if (col != -1)
    {
      uint32_t now = HAL_GetTick();
      if (lastRow == currentRow && lastCol == col && (now - lastTriggerTime < 300)) {
          return; // HACKY debounce/repeat suppression
      }
      lastRow = currentRow;
      lastCol = col;
      lastTriggerTime = now;
      // Key at (currentRow, col) was pressed!
      // printf("Pressed row %d and col %d\r\n", currentRow, col);

      receivedChar = keymap[currentRow][col];
      // printf("Which is hopefully %c\r\n", receivedChar);
      AppEvent evt = { .type = EVENT_KEY_PRESSED, .key = receivedChar };
      event_queue_push(evt);
    }
}

void setAllRowsInactive()
{
    // Set all rows HIGH (not active)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

void setRowActive(int row)
{
    setAllRowsInactive();

    switch (row)
    {
        case 0: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); break;
        case 4: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); break;
    }
}

void SetCursorPosition(int row, int col) {
  printf("\033[%d;%dH", row, col);
}

void ClearScreen() {
  printf("\033[2J");
  printf("\033[H"); // Move cursor to top-left corner
}

void clearInput(AppContext *ctx) {
  ctx->inputValue = 0;
}

void clearVoltage(AppContext *ctx) {
  ctx->voltage = 0;
  ctx->isVoltageEntered = false;
}

void backspace(AppContext *ctx) {
  ctx->inputValue = ctx->inputValue / 10;
}

void stopPWM(AppContext *ctx) {
  ctx->isPwmRunning = false;
  // TODO - stop PWM :)
}

void startPWM(AppContext *ctx) {
  ctx->isPwmRunning = true;
  // TODO - start PWM :)
}

void setSTATE_F2(AppContext *ctx) {
  ctx->currentState = STATE_F2;
}

void setSTATE_F1(AppContext *ctx) {
  ctx->currentState = STATE_F1;
}

void validateAndSetVoltage(AppContext *ctx) {
  if (ctx->inputValue < 80 || ctx->inputValue > 400)
  {
    strcpy(ctx->message, "Input voltage has to be in range 80 - 400. Resetting, try again!\r\n");
    clearInput(ctx);
    return;
  }
  ctx->isVoltageEntered = true;
  ctx->voltage = ctx->inputValue;
  ctx->inputValue = 0;
  sprintf(ctx->message, "Voltage %d has been successfully entered\r\n", ctx->voltage);

}

void updateInput(AppContext *ctx, KeyboardButton key) {
  uint8_t digit = key - '0';
  ctx->inputValue = ctx->inputValue * 10 + digit;
  if (ctx->inputValue > 400) {
    strcpy(ctx->message, "Input set too high, resetting. Try again\r\n");
    clearInput(ctx);
  }
}

void handle_event(AppContext *ctx, AppEvent *evt) {
  ctx->message[0] = '\0';
  switch (evt->type) {
    case EVENT_KEY_PRESSED:
      if (ctx->currentState == STATE_F1) {
        if (ctx->isPwmRunning == true)
        {
          if (evt->key == KEY_Stop) stopPWM(ctx);
          return; // when PWM is running, we can only press the "STOP" button
        }

        if (ctx->isVoltageEntered == true) // valid voltage has been entered
        {
          if (evt->key == KEY_Start) startPWM(ctx);
        }

        if (evt->key >= KEY_0 && evt->key <= KEY_9) updateInput(ctx, evt->key);
        if (evt->key == KEY_Clear) clearVoltage(ctx);
        if (evt->key == KEY_Enter) validateAndSetVoltage(ctx);
        if (evt->key == KEY_BkSp) backspace(ctx);
        if (evt->key == KEY_F2) setSTATE_F2(ctx);
        if (evt->key == KEY_ESC) clearInput(ctx);
      }

      if (ctx->currentState == STATE_F2) {
        if (evt->key == KEY_F1) setSTATE_F1(ctx);
        return;
      }
      break;
    case EVENT_TIMER_TICK:
      // Maybe update screen or timeout handler
      break;
    default:
      break;
  }
}

void renderState(AppContext *ctx) {
  ClearScreen();
  SetCursorPosition(1, 1);
  if (ctx->currentState == STATE_F1) {
    printf("Voltage control");
    SetCursorPosition(2, 1);
    printf("Current input: %d", ctx->inputValue);
    SetCursorPosition(3, 1);
    if (ctx->voltage > 0) {
      printf("Voltage: %dV", ctx->voltage);
    } else {
      printf("Voltage: N/A");
    }
    SetCursorPosition(4, 1);
    if (ctx->isPwmRunning == true) {
      printf("PWM is running at %dV", ctx->voltage);
    } else {
      printf("PWM is OFF");
    }
    SetCursorPosition(5, 1);
    printf(ctx->message);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    AppEvent evt = { .type = EVENT_KEY_PRESSED, .key = receivedChar };
    event_queue_push(evt);

    // Start receiving the next character again
    HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
  }
}

int __io_putchar(int ch) {
  if (HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY) != HAL_OK) {
    return -1;
  }
  return ch;
}
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
