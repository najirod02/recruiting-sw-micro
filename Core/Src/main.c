/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "scheme.h"
#include <string.h>
#include <stdio.h>
#include <time.h> //for random values
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    RAW,
    MOVING_AVERAGE,
    RANDOM_NOISE
} FilterMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE 300
#define MOVING_AVG_SIZE 150
#define BUFFER_SIZE 50
#define RANDOM_RANGE 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef halStatus;

state_t current_state = STATE_INIT;
state_t next_state = STATE_WAIT_REQUEST;

/* USER CODE BEGIN PV */

//digital and analog variables
uint16_t lastAnalogValue = 0; 
uint16_t lastDigitalValue = 0;

uint16_t last_index = 0;//position of last written element on buffer
uint32_t adcBuffer[ADC_BUFFER_SIZE];//dma data structure

//moving average variables
uint16_t adc_moving_average[MOVING_AVG_SIZE];
uint16_t buffer_index = 0;
uint32_t sum = 0;

//millis for warning state
uint32_t lastTimer = 0;
uint8_t isWarningState = 0;

//filter mode for data variables
FilterMode currentFilterMode = RAW;
char cli_command[BUFFER_SIZE];
uint8_t sendRequest = 0;

//debug/info variables
char msg_buffer[BUFFER_SIZE];//buffer for serial msg

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Callback function for gpio digital input
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_2){
    lastDigitalValue = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
  }
  else if(GPIO_Pin == GPIO_PIN_13){
    //FIXME: probably, need to debounce button to avoid multilpe firing
    if(current_state == STATE_WAIT_REQUEST || current_state == STATE_PAUSE){
      //change state to listening and restore dma
      current_state = next_state = STATE_LISTENING;
      HAL_ADC_Start_DMA(&hadc1, adcBuffer, ADC_BUFFER_SIZE);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_UART_AbortReceive_IT(&huart2);
      lastTimer = HAL_GetTick();
    }else if(current_state == STATE_LISTENING){
      //change state to pause
      current_state = next_state = STATE_PAUSE;
      //wait for next input
      HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)cli_command, BUFFER_SIZE);
    }else if(current_state == STATE_WARNING){
      //change state to wait request
      current_state = next_state = STATE_WAIT_REQUEST;
      //wait for next input
      HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)cli_command, BUFFER_SIZE);
    }else if(current_state == STATE_ERROR){
      //reset the board
      NVIC_SystemReset();
    }

    sendRequest = 0;
    lastTimer = HAL_GetTick();//reset timer
  }
}

/*
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
}
*/

/**
Based on the user input, change the filter mode.
If the comand is unknown, the current filter is not changed but
a falsse value is returned
*/
u_int8_t handle_cli_command() {
    if (strcmp(cli_command, "raw") == 0) {
        currentFilterMode = RAW;
    } else if (strcmp(cli_command, "moving average") == 0) {
        currentFilterMode = MOVING_AVERAGE;
    } else if (strcmp(cli_command, "random noise") == 0) {
        currentFilterMode = RANDOM_NOISE;
    } else {
        //comand not recognized
        memset(cli_command, '\0', sizeof(cli_command)); 
        return 1;
    }
    //clear buffer
    memset(cli_command, '\0', sizeof(cli_command)); 
    return 0;
}

/**
 * Callback function to handle the receiving of a comand from the user
 * if the command is unknown, restart sending data anyway
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  //if we are not in pause or wait request state, ignore the comand received
  //wait for next input
  if(current_state!= STATE_PAUSE && current_state != STATE_WAIT_REQUEST){
    memset(cli_command, '\0', sizeof(cli_command)); 
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)cli_command, BUFFER_SIZE);
    return;
  }

  uint8_t unknownComand = handle_cli_command();

  if(unknownComand){
    sprintf(msg_buffer, "Command not valid. Try again.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
    sprintf(msg_buffer, "C:\r\n");//send comand request to user
    HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);  
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)cli_command, BUFFER_SIZE);
  }
  else{
    sprintf(msg_buffer, "New filter mode: %d\r\n", currentFilterMode);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
    sendRequest = 1;
    HAL_ADC_Start_DMA(&hadc1, adcBuffer, ADC_BUFFER_SIZE);
    current_state = next_state = STATE_LISTENING;
    lastTimer = HAL_GetTick();
  }

  //wait for next input
  //HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)cli_command, BUFFER_SIZE);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current_state = run_state(NULL);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//debug/info variables
char msg_buffer[BUFFER_SIZE];//buffer for serial msg

// GLOBALS
// State human-readable names
const char *state_names[] = {"init", "wait_request", "error", "listening", "warning", "pause"};

// List of state functions
state_func_t *const state_table[NUM_STATES] = {
  do_init,         // in state init
  do_wait_request, // in state wait_request
  do_error,        // in state error
  do_listening,    // in state listening
  do_warning,      // in state warning
  do_pause,        // in state pause
};

// No transition functions

/*  ____  _        _       
 * / ___|| |_ __ _| |_ ___ 
 * \___ \| __/ _` | __/ _ \
 *  ___) | || (_| | ||  __/
 * |____/ \__\__,_|\__\___|
 *                         
 *   __                  _   _                 
 *  / _|_   _ _ __   ___| |_(_) ___  _ __  ___ 
 * | |_| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 * |  _| |_| | | | | (__| |_| | (_) | | | \__ \
 * |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
 */                                             

// Function to be executed in state init
// valid return states: STATE_WAIT_REQUEST, STATE_ERROR
state_t do_init(state_data_t *data) {
  next_state = STATE_WAIT_REQUEST;
  
  //syslog(LOG_INFO, "[FSM] In state init");
  /* Your Code Here */
  srand(time(NULL)); //set seed for random number generator
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  halStatus = HAL_Init();

  if(halStatus != HAL_OK){
    next_state  = STATE_ERROR;
    return next_state;
  }

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();

  // Initialize the DMA conversion
  halStatus = HAL_ADC_Start_DMA(&hadc1, adcBuffer, ADC_BUFFER_SIZE);
  if(halStatus != HAL_OK){
    next_state  = STATE_ERROR;
    return next_state;
  }

  halStatus = HAL_UARTEx_ReceiveToIdle_IT(&huart2, (u_int8_t *)cli_command, BUFFER_SIZE);
  if(halStatus != HAL_OK){
    next_state  = STATE_ERROR;
    return next_state;
  }

  lastTimer = HAL_GetTick();//start time of MCU
  next_state = STATE_WAIT_REQUEST;

  return next_state;
}


// Function to be executed in state wait_request
// valid return states: STATE_ERROR, STATE_LISTENING
state_t do_wait_request(state_data_t *data) {
  next_state = NO_CHANGE;
  
  //syslog(LOG_INFO, "[FSM] In state wait_request");
  /* Your Code Here */
  if(!sendRequest){
    halStatus = HAL_ADC_Stop_DMA(&hadc1);
    if(halStatus != HAL_OK){
      next_state  = STATE_ERROR;
      return next_state;
    }

    sprintf(msg_buffer, "C:\r\n");//send comand request to user
    halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
    if(halStatus != HAL_OK){
      next_state  = STATE_ERROR;
      return next_state;
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    sendRequest = 1;
  }
  
  return next_state;
}


// Function to be executed in state error
// valid return states: NO_CHANGE
state_t do_error(state_data_t *data) {
  next_state = NO_CHANGE;
  
  //syslog(LOG_INFO, "[FSM] In state error");
  /* Your Code Here */
  sprintf(msg_buffer, "ERROR\r\n");
  halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY); 
  //TODO: use a TIME peripheral to blink led
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);

  return next_state;
}


// Function to be executed in state listening
// valid return states: STATE_ERROR, STATE_WARNING, STATE_PAUSE
state_t do_listening(state_data_t *data) {
  next_state = NO_CHANGE;
  
  //syslog(LOG_INFO, "[FSM] In state listening");
  /* Your Code Here */

  last_index = (BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_adc1)) % BUFFER_SIZE;
  lastAnalogValue = adcBuffer[last_index];
  //update moving average if in MOVING_AVERAGE mode
  if (currentFilterMode == MOVING_AVERAGE) {
    sum -= adc_moving_average[buffer_index];
    adc_moving_average[buffer_index] = lastAnalogValue;
    sum += lastAnalogValue;
    buffer_index = (buffer_index + 1) % MOVING_AVG_SIZE;
  }

  //print analog data
  switch (currentFilterMode)
  {
  case RAW:
    sprintf(msg_buffer, "A:%hu\r\n", (u_int16_t)lastAnalogValue);
    break;
  
  case MOVING_AVERAGE:
    float moving_avg = (float)sum / MOVING_AVG_SIZE;
    sprintf(msg_buffer, "A: %.3f\r\n", moving_avg);
    break;

  case RANDOM_NOISE:
    sprintf(msg_buffer, "A: %hu\r\n", (u_int16_t)(adcBuffer[0] + rand() % RANDOM_RANGE));
    break;

  default:
    break;
  }
  halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
  if(halStatus != HAL_OK){
    next_state  = STATE_ERROR;
    return next_state;
  }

  //print digital data
  sprintf(msg_buffer, "D: %hu\r\n", lastDigitalValue);
  halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
  if(halStatus != HAL_OK){
    next_state  = STATE_ERROR;
    return next_state;
  }

  if(lastDigitalValue){
    //check if is high for 5 seconds
    if(HAL_GetTick() - lastTimer >= 5000){
      //warning state
      next_state = STATE_WARNING;
      HAL_UART_AbortReceive_IT(&huart2);
    }
  }else{
    //reset timer
    lastTimer = HAL_GetTick();
  }

  return next_state;
}


// Function to be executed in state warning
// valid return states: STATE_WAIT_REQUEST, STATE_ERROR
state_t do_warning(state_data_t *data) {
  next_state = NO_CHANGE;
  
  //syslog(LOG_INFO, "[FSM] In state warning");
  /* Your Code Here */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  sprintf(msg_buffer, "WARNING\r\n");
  halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY); 
  if(halStatus != HAL_OK){
    next_state = STATE_ERROR;
    return next_state;
  }

  return next_state;
}


// Function to be executed in state pause
// valid return states: STATE_ERROR, STATE_LISTENING
state_t do_pause(state_data_t *data) {
  next_state = NO_CHANGE;
  
  //syslog(LOG_INFO, "[FSM] In state pause");
  /* Your Code Here */
  if(!sendRequest){
    halStatus = HAL_ADC_Stop_DMA(&hadc1);
    if(halStatus != HAL_OK){
      next_state  = STATE_ERROR;
      return next_state;
    }

    sprintf(msg_buffer, "C:\r\n");//send comand request to user
    halStatus = HAL_UART_Transmit(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer), HAL_MAX_DELAY);
    if(halStatus != HAL_OK){
      next_state  = STATE_ERROR;
      return next_state;
    }

    //TODO: use a TIME peripheral to blink led
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
    sendRequest = 1;
  }

  return next_state;
}



/*  ____  _        _        
 * / ___|| |_ __ _| |_ ___  
 * \___ \| __/ _` | __/ _ \
 *  ___) | || (_| | ||  __/ 
 * |____/ \__\__,_|\__\___| 
 *                          
 *                                              
 *  _ __ ___   __ _ _ __   __ _  __ _  ___ _ __ 
 * | '_ ` _ \ / _` | '_ \ / _` |/ _` |/ _ \ '__|
 * | | | | | | (_| | | | | (_| | (_| |  __/ |   
 * |_| |_| |_|\__,_|_| |_|\__,_|\__, |\___|_|   
 *                              |___/           
 */

state_t run_state(state_data_t *data) {
  state_t new_state = state_table[current_state](data);
  if (new_state == NO_CHANGE) new_state = current_state;
  return new_state;
};

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  next_state = STATE_ERROR;
  return
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
