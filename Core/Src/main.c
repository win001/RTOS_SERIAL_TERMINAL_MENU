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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#include "DHT11.h"
#include "File_Handling_RTOS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define INVALID_COMMAND 100
#define COMMAND 0
#define ARGUMENT 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

// osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef struct APP_CMD
{
  uint8_t COMMAND_NUM;
  uint8_t COMMAND_ARGS[10];
} APP_CMD_t;
// typedef struct APP_ARG
// {
//   uint8_t COMMAND_ARGS[10];
// } APP_ARG_t;

#define LED_COUNT 4

TaskHandle_t MenueTask_Handler;
TaskHandle_t printTask_Handler;
TaskHandle_t AnimationTask_Handler;
TaskHandle_t ComdHandleTask_Handler;
TaskHandle_t CmdProcessTask_Handler;
TaskHandle_t ArgProcessTask_Handler;

QueueHandle_t command_queue = NULL;
QueueHandle_t argument_queue = NULL;
QueueHandle_t uart_print_queue = NULL;

void led1_toggle(TimerHandle_t xTimer);
void led2_toggle(TimerHandle_t xTimer);
void led3_toggle(TimerHandle_t xTimer);
void led4_toggle(TimerHandle_t xTimer);
void (*led_toggle_array[4])(TimerHandle_t xTimer) = { &led1_toggle, &led2_toggle, &led3_toggle, &led4_toggle };
TimerHandle_t led1_timer_handle = NULL;
TimerHandle_t led2_timer_handle = NULL;
TimerHandle_t led3_timer_handle = NULL;
TimerHandle_t led4_timer_handle = NULL;
TimerHandle_t led_timer_handle_array[LED_COUNT]; // = {led1_timer_handle, led2_timer_handle, led3_timer_handle, led4_timer_handle};
TimerHandle_t data_log_timer_handle = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
// void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void msDelay(uint32_t msTime);
uint8_t getCommandCode(uint8_t *buffer);
bool getArguments(uint8_t *buffer);
bool isNumber(uint8_t p);
int parseString(uint8_t *str, int strLength, char splitStrings[5][12]);
bool streq(char *str1, const char *str2);
void setCurrentTime(uint8_t *buffer);
void setCurrentDate(uint8_t *buffer);
// void led_toggle(TimerHandle_t xTimer);
void led_toggle_start(uint32_t duration);
void led_toggle_stop(void);
void deleteAllTasks(void);
void getDateTime(void);
void startSensorDataLogging(uint32_t duration);
void stopSensorDataLogging(void);
void log_sensor_data(TimerHandle_t xTimer);
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Menue_Task(void *argument);
void PrintTask_Task(void *argument);
void Animation_Task(void *argument);
void ComdHandle_Task(void *argument);
void CmdProcess_Task(void *argument);
void ArgProcess_Task(void *argument);
// This is the menu
char menu[] = {"\
\r\nStart Animation in Terminal     ----> 1 \
\r\nBlink LEDs ID space Delay       ----> 2 \
\r\nStop all LEDs blinking          ----> 3 \
\r\nGet current date and time       ----> 4 \
\r\nSet the current date            ----> 5 \
\r\nSet the current time            ----> 6 \
\r\nStart Data Logging              ----> 7 \
\r\nStop data logging               ----> 8 \
\r\nGet the data logged             ----> 9 \
\r\nSet Alarm date and time         ----> 10 \
\r\nClose the terminal              ----> 0 \
\r\n"};

char genPrintArr[] = {"\r\n"};

char animation[] = {"\
\r\n*     ---     ---     *\n"};

uint8_t rx_buffer[50];
uint8_t rx_index = 0;
uint8_t rx_data = '\r';
uint8_t LED_ID = 1;
uint16_t LED_ID_ARRAY[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
volatile bool isCmdOrArg = COMMAND;
uint8_t current_cmd_code = 0;

uint16_t ADC_VAL;
int indx=1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  led_timer_handle_array[0] = led1_timer_handle;
  led_timer_handle_array[1] = led2_timer_handle;
  led_timer_handle_array[2] = led3_timer_handle;
  led_timer_handle_array[3] = led4_timer_handle;
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
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  Mount_SD("/");
  Format_SD();
  Create_File("ADC_DATA.TXT");
  // Create_File("TEMP.TXT");
  Unmount_SD("/");  

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
  argument_queue = xQueueCreate(10, sizeof(APP_CMD_t *));
  command_queue = xQueueCreate(10, sizeof(uint8_t));
  uart_print_queue = xQueueCreate(10, sizeof(char[10]));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  // osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  if ((command_queue != NULL) && (argument_queue != NULL) && (uart_print_queue != NULL))
  {
    xTaskCreate(Menue_Task, "MENUE", 512, NULL, 1, &MenueTask_Handler);
    xTaskCreate(PrintTask_Task, "PRINTMENUE", 512, NULL, 2, &printTask_Handler);
    xTaskCreate(Animation_Task, "ANIMATION", 512, NULL, 2, &AnimationTask_Handler);
    xTaskCreate(ComdHandle_Task, "CMDHNDL", 512, NULL, 2, &ComdHandleTask_Handler);
    xTaskCreate(CmdProcess_Task, "CMDPROCESS", 512, NULL, 2, &CmdProcessTask_Handler);
    xTaskCreate(ArgProcess_Task, "ARGROCESS", 512, NULL, 2, &ArgProcessTask_Handler);
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
  // osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    msDelay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    msDelay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    msDelay(200);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    msDelay(200);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
  {
    sTime.Hours = 0x01;
    sTime.Minutes = 0x43;
    sTime.Seconds = 0x00;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_MARCH;
    sDate.Date = 0x12;
    sDate.Year = 0x24;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
      Error_Handler();
    }    

     HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);  // backup register
  }
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  // sTime.Hours = 0x23;
  // sTime.Minutes = 0x55;
  // sTime.Seconds = 0x55;
  // sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  // sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  // if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  // sDate.Month = RTC_MONTH_MARCH;
  // sDate.Date = 0x11;
  // sDate.Year = 0x24;

  // if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x01;
  sAlarm.AlarmTime.Minutes = 0x58;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x12;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
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
  // maximum number of command supported 99
  uint8_t cnt = 0;
  while(buffer[cnt] != '\r'){
    cnt++;
    if(cnt > 2) return INVALID_COMMAND;
  }
  if(cnt == 1) return (buffer[0] - 48);
  else {
    return (buffer[0] - 48) * 10 + (buffer[1] - 48);
  }
}

bool getArguments(uint8_t *buffer)
{
  uint8_t cnt = 0;
  while(rx_buffer[cnt] != '\r'){
    cnt++;
  }
  cnt = cnt + 1;
  char splitStrings[5][12] = {{0}};
  int cntArg = parseString(rx_buffer, cnt, splitStrings);

  if(cntArg < 1) return false;

  for (int i = 0; i < cntArg; i++)
  {
    buffer[i] = atoi(splitStrings[i]);
  }
  return true;
}

bool isNumber(uint8_t p)
{
  if(p >= 48 && p <= 57) return true;
  return false;
}

int parseString(uint8_t *str, int strLength, char splitStrings[5][12])
{
    int i = 0, j = 0, cnt = 0;
    int rows = 5;
    int colums = 12;

    for (i = 0; i < strLength && cnt < rows && j < colums; i++)
    {
        // if space or NULL found, assign NULL into splitStrings[cnt]
        if (str[i] == ' ' || str[i] == '\0' || str[i] == 13) // 13 is for checking carriage return
        {
            splitStrings[cnt][j] = '\0';
            cnt++; // for next word
            j = 0; // for next word, init index to 0
        }
        else
        {
            splitStrings[cnt][j] = str[i];
            j++;
        }
    }
    return cnt;
}

bool streq(char *str1, const char *str2)
{
    return (strcmp(str1, str2) == 0);
}

void setCurrentTime(uint8_t *buffer)
{
  RTC_TimeTypeDef sTime = {0};
  sTime.Hours = buffer[0];
  sTime.Minutes = buffer[1];
  sTime.Seconds = buffer[2];
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

void setCurrentDate(uint8_t *buffer)
{
  RTC_DateTypeDef sDate = {0};
  sDate.WeekDay = buffer[3];//RTC_WEEKDAY_MONDAY;
  sDate.Month = buffer[1];
  sDate.Date = buffer[0];
  sDate.Year = buffer[2];
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
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

void PrintTask_Task(void *argument)
{
  while (1)
  {
    char *str;
    xQueueReceive(uart_print_queue, &str, portMAX_DELAY);
    HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    vPortFree(str);
    // xTaskNotify(MenueTask_Handler, 0, eNoAction);
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
  uint8_t command_code = INVALID_COMMAND;

  APP_CMD_t *new_cmd;

  while (1)
  {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    // 1. send command to queue
    new_cmd = (APP_CMD_t *)pvPortMalloc(sizeof(APP_CMD_t));
    // &command_code = pvPortMalloc(sizeof(uint8_t));

    if(isCmdOrArg == COMMAND){
      taskENTER_CRITICAL();
      command_code = getCommandCode(rx_buffer);
      if(command_code == INVALID_COMMAND){
        isCmdOrArg = COMMAND;
        char *str = "\nPlease enter the correct command...\r\n";
        HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);        
        taskEXIT_CRITICAL();
        continue;
      }
      current_cmd_code = command_code;
      taskEXIT_CRITICAL();

      // send the command to the command queue
      xQueueSend(command_queue, &command_code, portMAX_DELAY);
    } else if(isCmdOrArg == ARGUMENT){
      taskENTER_CRITICAL();
      if(getArguments(new_cmd->COMMAND_ARGS) == false){
        isCmdOrArg = ARGUMENT;
        char *str = "\nPlease enter the correct argumant/s...\r\n";
        HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);        
        taskEXIT_CRITICAL();
        continue;
      }
      isCmdOrArg = COMMAND;
      taskEXIT_CRITICAL();
      xQueueSend(argument_queue, &new_cmd, portMAX_DELAY);
    } else {
      // do nothing
    }
  }
}

void CmdProcess_Task(void *argument)
{
  while (1)
  {
    APP_CMD_t *cmd;
    uint8_t cmd_code;
    xQueueReceive(command_queue, &cmd_code, portMAX_DELAY);
    if (cmd_code == 1)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nStraing the Animation...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);      
      xTaskNotify(AnimationTask_Handler, 0, eNoAction);
    }
    else if (cmd_code == 2)
    {
      isCmdOrArg = ARGUMENT;
      char *str = "\nPlease enter LED Id {space} blink rate in ms...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    else if (cmd_code == 3)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nStoping all the blinking LEDs...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      led_toggle_stop();
    }
    else if (cmd_code == 4)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nCurrent date and time are following...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      getDateTime();
    }
    else if (cmd_code == 5)
    {
      isCmdOrArg = ARGUMENT;
      char *str = "\nPlease enter the date in dd/mm/yy formate seperated by space...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    else if (cmd_code == 6)
    {
      isCmdOrArg = ARGUMENT;
      char *str = "\nPlease enter the current time in hh/mm/ss formate (24 hr) seperated by space...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    else if (cmd_code == 7)
    {
      isCmdOrArg = ARGUMENT;
      char *str = "\nPlease enter the data logging interval in ms...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      // startSensorDataLogging(cmd->COMMAND_ARGS[1]);
    }
    else if (cmd_code == 8)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nStopping sensor data logging...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      stopSensorDataLogging();
    }
    else if (cmd_code == 9)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nGetting the logged senspr data...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    else if (cmd_code == 10)
    {
      isCmdOrArg = ARGUMENT;
      char *strp;
      strp = pvPortMalloc(sizeof (char[500]));
      char str[] = {"\
      \r\nChoose the envent to trigger on alarm...\
      \r\nSet to close app                ----> 1 \
      \r\nSet to stop data logging        ----> 2 \
      \r\nSet to start data logging       ----> 3 \
      \r\nset to stop all blinking LEDs   ----> 4 \
      \r\n"};
      strcpy(strp, str);
      xQueueSend(uart_print_queue, &strp, portMAX_DELAY);
    }
    else if (cmd_code == 0)
    {
      isCmdOrArg = COMMAND;
      char *str = "\nClosing the terminal... Need to restart the MCU\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      deleteAllTasks();
    }    
    else
    {
      isCmdOrArg = COMMAND;
      char *str = "\nWrong command entered, Please enter valid command\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    if(isCmdOrArg == COMMAND){
      xTaskNotify(MenueTask_Handler, 0, eNoAction);
    }
  }
}

void ArgProcess_Task(void *argument)
{
  while (1)
  {
    APP_CMD_t *cmd;
    bool checkCmdMatched = true;
    xQueueReceive(argument_queue, &cmd, portMAX_DELAY);
    if (current_cmd_code == 1)
    {
      char *str = "\nStraing the Animation...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);      
      xTaskNotify(AnimationTask_Handler, 0, eNoAction);
    }
    else if (current_cmd_code == 2)
    {
      char *str = "\nBlinking the LED...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      LED_ID = cmd->COMMAND_ARGS[0];
      led_toggle_start(cmd->COMMAND_ARGS[1]);
    }
    else if (current_cmd_code == 3)
    {
      char *str = "\nStoping the LEDs...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      led_toggle_stop();
    }
    else if (current_cmd_code == 4)
    {
      char *str = "\nCurrent date and time...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      getDateTime();
    }
    else if (current_cmd_code == 5)
    {
      char *str = "\nSetting the current date...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      setCurrentDate(cmd->COMMAND_ARGS);
    }
    else if (current_cmd_code == 6)
    {
      char *str = "\nSetting the current time...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      setCurrentTime(cmd->COMMAND_ARGS);
    }
    else if (current_cmd_code == 7)
    {
      char *str = "\nStarting sensor data logging...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      startSensorDataLogging(cmd->COMMAND_ARGS[0]);
    }
    else if (current_cmd_code == 8)
    {
      char *str = "\nStopping sensor data logging...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      stopSensorDataLogging();
    }
    else if (current_cmd_code == 9)
    {
      char *str = "\nGetting the data logged...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      // cmd->COMMAND_ARGS[0];
    }
    else if (current_cmd_code == 10)
    {
      char *str = "\nSetting Alarm timing...\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      // cmd->COMMAND_ARGS[0];
    }
    else if (current_cmd_code == 0)
    {
      char *str = "\nClosing the terminal... Need to restart the MCU\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
      deleteAllTasks();
    }    
    else
    {
      checkCmdMatched = false;
      char *str = "\nWrong command entered, Please enter valid command\r\n";
      HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
    }
    if(checkCmdMatched == true){
      xTaskNotify(MenueTask_Handler, 0, eNoAction);
    }
    vPortFree(cmd);
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[0],GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[1],GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[2],GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[3],GPIO_PIN_SET);
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
      // xTaskNotifyFromISR(MenueTask_Handler, 0, eNoAction, &xHigherPriorityTaskWoken);
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
  for (uint8_t p = 0; p < LED_COUNT; p++)
  {
    xTimerStop(led_timer_handle_array[p], portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[p],GPIO_PIN_RESET);
  }
}

void led1_toggle(TimerHandle_t xTimer)
{
  HAL_GPIO_TogglePin(GPIOD,LED_ID_ARRAY[0]);
}

void led2_toggle(TimerHandle_t xTimer)
{
  HAL_GPIO_TogglePin(GPIOD,LED_ID_ARRAY[1]);
}

void led3_toggle(TimerHandle_t xTimer)
{
  HAL_GPIO_TogglePin(GPIOD,LED_ID_ARRAY[2]);
}

void led4_toggle(TimerHandle_t xTimer)
{
  HAL_GPIO_TogglePin(GPIOD,LED_ID_ARRAY[3]);
}

void led_toggle_start(uint32_t duration)
{
  uint32_t toggle_duration = pdMS_TO_TICKS(duration*1000);

	if(led_timer_handle_array[LED_ID - 1] == NULL)
	{
		//1. lets create the software timer
		led_timer_handle_array[LED_ID - 1] = xTimerCreate("LED-TIMER", toggle_duration, pdTRUE, NULL, led_toggle_array[LED_ID - 1]);

		//2. start the software timer
		xTimerStart(led_timer_handle_array[LED_ID - 1], portMAX_DELAY);
	}
	else
	{
		//start the software timer
    HAL_GPIO_WritePin(GPIOD,LED_ID_ARRAY[LED_ID - 1],GPIO_PIN_RESET);
    xTimerChangePeriod(led_timer_handle_array[LED_ID - 1],toggle_duration,portMAX_DELAY);
		xTimerStart(led_timer_handle_array[LED_ID - 1],portMAX_DELAY);
	}
}

void startSensorDataLogging(uint32_t duration)
{
  uint32_t logging_duration = pdMS_TO_TICKS(duration*1000);
  if(data_log_timer_handle == NULL)
  {
    //1. lets create the software timer
    data_log_timer_handle = xTimerCreate("SENSOR-TIMER",logging_duration,pdTRUE,NULL,log_sensor_data);

    //2. start the software timer
    xTimerStart(data_log_timer_handle,portMAX_DELAY);
  }
  else
  {
    //start the software timer
    // xTimerChangePeriod(data_log_timer_handle,logging_duration,portMAX_DELAY);
    int indx=1;
    xTimerStart(data_log_timer_handle,portMAX_DELAY);
  }
}

void log_sensor_data(TimerHandle_t xTimer)
{
  // int indx=1;

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  ADC_VAL = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  char *buffer = pvPortMalloc(50*sizeof(char));
  sprintf (buffer, "%d. %u\n", indx,ADC_VAL);
  Mount_SD("/");
  // Check_SD_Space();
  Update_File("ADC_DATA.TXT", buffer);

  // char *str = "\nStarting sensor data logging...\r\n";
  HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);  
  // sprintf (buffer, "%d. Temp = %d C\t RH = %d \n",indx, Temperature, Humidity);
  // Update_File("TEMP.TXT", buffer);
  vPortFree(buffer);
  Unmount_SD("/");

  indx++;
}

void stopSensorDataLogging(void)
{
  int indx=1;
  xTimerStop(data_log_timer_handle,portMAX_DELAY);
}

void deleteAllTasks(void)
{
  vTaskDelete(MenueTask_Handler);
  vTaskDelete(AnimationTask_Handler);
  vTaskDelete(ComdHandleTask_Handler);
  vTaskDelete(CmdProcessTask_Handler);
}

void getDateTime(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  char date[20];
  char time[20];
  char weekday[20];
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  sprintf(date, "Date: %02d-%02d-%02d\r\n", sdatestructureget.Date, sdatestructureget.Month, sdatestructureget.Year);
  sprintf(weekday, "Weekday: %02d\r\n", sdatestructureget.WeekDay);
  sprintf(time, "Time: %02d:%02d:%02d\r\n", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
  HAL_UART_Transmit(&huart2, date, strlen(date), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, weekday, strlen(weekday), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, time, strlen(time), HAL_MAX_DELAY);
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
void StartDefaultTask(void const * argument)
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
  if (htim->Instance == TIM1) {
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
