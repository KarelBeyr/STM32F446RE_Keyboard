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

typedef struct {
	AppState currentState;

	// F1 - screen where user enters voltage and can start/stop PWM
    uint16_t voltage;
    uint16_t inputValue;
    bool isVoltageEntered;
    bool isPwmRunning;

    // F2 - screen where user sets three calibration points
    uint16_t calibration_points[3]; // For mapping
} AppContext;
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
uint8_t receivedChar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void handle_event(AppContext *ctx, AppEvent *evt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
  if (HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY) != HAL_OK) {
    return -1;
  }
  return ch;
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
  HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
  setbuf(stdout, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting the main loop\r\n");
  AppEvent evt;
  AppContext ctx;
  ctx.currentState = STATE_F1;
  ctx.isVoltageEntered = false;
  ctx.isPwmRunning = false;
  ctx.voltage = 0;
  ctx.inputValue = 0;

  while (1)
  {
    if (event_queue_pop(&evt)) {
      handle_event(&ctx, &evt);
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void clearInput(AppContext *ctx) {
  printf("Clearing input value\r\n");
  ctx->inputValue = 0;
}

void stopPWM(AppContext *ctx) {
  printf("Stopping PWM\r\n");
  ctx->isPwmRunning = false;
  // TODO - stop PWM :)
}

void startPWM(AppContext *ctx) {
  printf("Starting PWM for %dV\r\n", ctx->voltage);
  ctx->isPwmRunning = true;
  // TODO - start PWM :)
}

void setSTATE_F2(AppContext *ctx) {
  ctx->currentState = STATE_F2;
  printf("Settings state to STATE_F2\r\n");
}

void setSTATE_F1(AppContext *ctx) {
  ctx->currentState = STATE_F1;
  printf("Settings state to STATE_F1\r\n");
}

void validateAndSetVoltage(AppContext *ctx) {
  if (ctx->inputValue < 80 || ctx->inputValue > 400)
  {
    printf("Input voltage has to be in range 80 - 400. Resetting, try again!\r\n");
    clearInput(ctx);
    return;
  }
  ctx->isVoltageEntered = true;
  ctx->voltage = ctx->inputValue;
  ctx->inputValue = 0;
  printf("Voltage %d has been successfully entered\r\n", ctx->voltage);
}

void updateInput(AppContext *ctx, uint8_t key) {
  uint8_t digit = key - '0';
  ctx->inputValue = ctx->inputValue * 10 + digit;
  if (ctx->inputValue > 400) {
    printf("Input set too high, resetting. Try again\r\n");
    clearInput(ctx);
  }

  printf("Temporary input is %d. Press ENTER to set it.\r\n", ctx->inputValue);
}

void handle_event(AppContext *ctx, AppEvent *evt) {
  switch (evt->type) {
    case EVENT_KEY_PRESSED:
      printf("GOT %c from event. Current state is %i\r\n", evt->key, ctx->currentState);

      if (ctx->currentState == STATE_F1) {
        if (ctx->isPwmRunning == true)
        {
          if (evt->key == 's') stopPWM(ctx);
          return; // when PWM is running, we can only press the "STOP" button
        }

        if (ctx->isVoltageEntered == true) // valid voltage has been entered
        {
          if (evt->key == 'S') startPWM(ctx);
        }

        if (evt->key >= '0' && evt->key <= '9') updateInput(ctx, evt->key);
        if (evt->key == 'c') clearInput(ctx);
        if (evt->key == 'e') validateAndSetVoltage(ctx);
        if (evt->key == 'x') setSTATE_F2(ctx);
      }

      if (ctx->currentState == STATE_F2) {
        if (evt->key == 'z') setSTATE_F1(ctx);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    //printf("GOT %c\r\n", receivedChar);
    AppEvent evt = { .type = EVENT_KEY_PRESSED, .key = receivedChar };
    event_queue_push(evt);

    // Start receiving the next character again
    HAL_UART_Receive_IT(&huart2, &receivedChar, 1);
  }
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
