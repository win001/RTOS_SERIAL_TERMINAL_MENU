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
// #include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "stdio.h"
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

// osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef struct APP_CMD
{
  uint8_t COMMAND_NUM;
  uint8_t COMMAND_ARGS[10];
} APP_CMD_t;

TaskHandle_t MenueTask_Handler;
TaskHandle_t AnimationTask_Handler;
TaskHandle_t ComdHandleTask_Handler;
TaskHandle_t CmdProcessTask_Handler;

QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

TimerHandle_t led_timer_handle = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
// void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void msDelay(uint32_t msTime);
uint8_t getCommandCode(uint8_t *buffer);
void getArguments(uint8_t *buffer);
void led_toggle(TimerHandle_t xTimer);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void deleteAllTasks(void);
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Menue_Task(void *argument);
void Animation_Task(void *argument);
void ComdHandle_Task(void *argument);
void CmdProcess_Task(void *argument);
// This is the menu
char menu[] = {"\
\r\nStart Animation in Terminal     ----> 1 \
\r\nBlink LEDs ID space Delay       ----> 2 \
\r\nStop all LEDs blinking          ----> 3 \
\r\nClose the terminal              ----> 4 \
\r\nWrite \"I am amazing\" :          ----> 0\r\n"};

char animation[] = {"\
\r\n*     ---     ---     *\n"};

uint8_t rx_buffer[50];
uint8_t rx_index = 0;
uint8_t rx_data = 'l';
uint8_t LED_ID = 1;
uint16_t LED_ID_ARRAY[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
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

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  command_queue = xQueueCreate(10, sizeof(APP_CMD_t *));
  uart_write_queue = xQueueCreate(10, sizeof(char *));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if ((command_queue != NULL) && (uart_write_queue != NULL))
  {
    xTaskCreate(Menue_Task, "MENUE", 256, NULL, 1, &MenueTask_Handler);
    xTaskCreate(Animation_Task, "ANIMATION", 512, NULL, 2, &AnimationTask_Handler);
    xTaskCreate(ComdHandle_Task, "CMDHNDL", 512, NULL, 2, &ComdHandleTask_Handler);
    xTaskCreate(CmdProcess_Task, "ANIMATION", 512, NULL, 2, &CmdProcessTask_Handler);
    vTaskStartScheduler();
  }
  else
  {
    HAL_UART_Transmit(&huart2, "Queue creation failed\r\n", strlen("Queue creation failed\r\n"), HAL_MAX_DELAY);
  }
  //  xTaskCreate(MPT_Task, "MPT", 128, NULL, 2, &MPT_Handler);
  //  xTaskCreate(LPT_Task, "LPT", 128, NULL, 1, &MPT_Handler);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    HAL_Delay(200);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void msDelay(uint32_t msTime)
{
  for (uint32_t i = 0; i < 168000; i++)
    ;
}

uint8_t getCommandCode(uint8_t *buffer)
{

  return buffer[0] - 48;
}

void getArguments(uint8_t *buffer)
{
  uint8_t cnt = 1;
  while(rx_buffer[cnt] != '\r'){
    cnt++;
    buffer[cnt-1] = rx_buffer[cnt] - 48;
  }
}

void Menue_Task(void *argument)
{
  //	uint8_t *strtosend = "IN HPT===========================\n";
  char *str = menu; // Rx_data;
  while (1)
  {
    //		char *str = "Entered HPT and Time delay 750\n";
    HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    //		vTaskDelay(1500);
  }
}

void Animation_Task(void *argument)
{
  while (1)
  {
    HAL_UART_Transmit(&huart2, animation, strlen(animation), HAL_MAX_DELAY);
    int l = 0, r = 22, m = 11;
    while (l <= r)
    {
      for (int i = 0; i <= 22; i++)
      {
        if (i == l || i == r)
        {
          HAL_UART_Transmit(&huart2, "*", 1, HAL_MAX_DELAY);
        }
        else if (i == m)
        {
          HAL_UART_Transmit(&huart2, "|", 1, HAL_MAX_DELAY);
        }
        else
        {
          HAL_UART_Transmit(&huart2, " ", 1, HAL_MAX_DELAY);
        }
      }
      ++l;
      --r;
      HAL_UART_Transmit(&huart2, "\n", 1, HAL_MAX_DELAY);
    }
    /*
        for (uint8_t j = 0; j < 12; j++)
        {
          uint8_t buff[23];
          uint8_t k = 100;
          uint8_t p = 100;
          for (uint8_t i = 0; i < 23; i++)
          {
            if(i == 11) {
              buff[i] = '|';
            } else if(i == 0){
              buff[0] = ' ';
              buff[i+j] = '*';
              k = i+j;
            } else if(i == 22){
              buff[22] = ' ';
              buff[i-j] = '*';
              p = i-j;
            } else {
              buff[i] = ' ';
            }
          }
          buff[k] = '*';
          buff[p] = '*';
          HAL_UART_Transmit(&huart2, buff, strlen(buff), HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart2, "\n", 1, HAL_MAX_DELAY);
        }
    */
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
  }
}

void ComdHandle_Task(void *argument)
{
  uint8_t command_code = 0;

  APP_CMD_t *new_cmd;

  while (1)
  {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    // 1. send command to queue
    new_cmd = (APP_CMD_t *)pvPortMalloc(sizeof(APP_CMD_t));

    taskENTER_CRITICAL();
    command_code = getCommandCode(rx_buffer);
    new_cmd->COMMAND_NUM = command_code;
    getArguments(new_cmd->COMMAND_ARGS);
    taskEXIT_CRITICAL();

    // send the command to the command queue
    xQueueSend(command_queue, &new_cmd, portMAX_DELAY);
  }
}

void CmdProcess_Task(void *argument)
{
  while (1)
  {
    APP_CMD_t *cmd;
    xQueueReceive(command_queue, &cmd, portMAX_DELAY);
    if (cmd->COMMAND_NUM == 1)
    {
      char *str = "\nStraing the Animation           ----> 1 \r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);      
      xTaskNotify(AnimationTask_Handler, 0, eNoAction);
    }
    else if (cmd->COMMAND_NUM == 2)
    {
      char *str = "\nBlink LEDs ID space Delay       ----> 2 \r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      led_toggle_start(cmd->COMMAND_ARGS[3]);
      LED_ID = cmd->COMMAND_ARGS[1];
    }
    else if (cmd->COMMAND_NUM == 3)
    {
      char *str = "\nStop all LEDs blinking          ----> 3 \r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      led_toggle_stop();
    }
    else if (cmd->COMMAND_NUM == 4)
    {
      char *str = "\nClosing the terminal, Need to restart the MCU ----> 4 \r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      deleteAllTasks();
    }
    else if (cmd->COMMAND_NUM == 0)
    {
      char *str = "\nWrite \"I am amazing\" :          ----> 0\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (huart->Instance == USART2)
  {

    if (rx_index == 0)
    {
      memset(&rx_buffer, 0, sizeof(rx_buffer));
    }

    rx_buffer[rx_index++] = rx_data;

    if (rx_data == '\r')
    {
      rx_index = 0;
      // xTaskNotifyFromISR(AnimationTask_Handler, 0, eNoAction, &xHigherPriorityTaskWoken);
      xTaskNotifyFromISR(MenueTask_Handler, 0, eNoAction, &xHigherPriorityTaskWoken);
      xTaskNotifyFromISR(ComdHandleTask_Handler, 0, eNoAction, &xHigherPriorityTaskWoken);
    }

    // Receive next character (1 byte)
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  }
  if (xHigherPriorityTaskWoken)
  {
    taskYIELD();
  }
}

void led_toggle_stop(void)
{
	 xTimerStop(led_timer_handle,portMAX_DELAY);
   HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[LED_ID - 1],GPIO_PIN_RESET);
}

void led_toggle(TimerHandle_t xTimer)
{
  HAL_GPIO_TogglePin(GPIOD,LED_ID_ARRAY[LED_ID - 1]);
}

void led_toggle_start(uint32_t duration)
{
  uint32_t toggle_duration = pdMS_TO_TICKS(duration*1000);

	if(led_timer_handle == NULL)
	{
		//1. lets create the software timer
		led_timer_handle = xTimerCreate("LED-TIMER",toggle_duration,pdTRUE,NULL,led_toggle);

		//2. start the software timer
		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
	else
	{
		//start the software timer
    HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[LED_ID - 1],GPIO_PIN_RESET);
    xTimerChangePeriod(led_timer_handle,toggle_duration,portMAX_DELAY);
		xTimerStart(led_timer_handle,portMAX_DELAY);
	}
}

void deleteAllTasks(void)
{
  vTaskDelete(MenueTask_Handler);
  vTaskDelete(AnimationTask_Handler);
  vTaskDelete(ComdHandleTask_Handler);
  vTaskDelete(CmdProcessTask_Handler);
}

/*-----------------------------------------------------------*/

/**
 * @brief This is to provide the memory that is used by the RTOS daemon/time task.
 *
 * If configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetTimerTaskMemory() to provide the memory that is
 * used by the RTOS daemon/time task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    // osDelay(500);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
