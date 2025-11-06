# FreeRTOS Serial Terminal -

<p align="center">
<img src="b0yzrits.bmp" width="150">
</p>

## Table of Contents
- [Project Overview](#project-overview)
- [Hardware Requirements](#hardware-requirements)
- [FreeRTOS Features Demonstrated](#freertos-features-demonstrated)
- [Project Architecture](#project-architecture)
- [Serial Terminal Commands](#serial-terminal-commands)
- [Task Details](#task-details)
- [FreeRTOS Synchronization Mechanisms](#freertos-synchronization-mechanisms)
- [Peripheral Integration](#peripheral-integration)
- [File Structure](#file-structure)
- [How It Works](#how-it-works)
- [Code Examples](#code-examples)

---

## Project Overview

This project is a comprehensive demonstration of **FreeRTOS** (Real-Time Operating System) capabilities on an **STM32F4** microcontroller. It implements a UART-based serial terminal menu system that showcases various FreeRTOS features including:
- Multi-tasking with multiple concurrent tasks
- Inter-task communication using queues
- Software timers for periodic operations
- Dynamic memory management
- Task notifications
- Interrupt handling integration with RTOS

The application provides an interactive command-line interface where users can control LEDs, manage real-time clock settings, log sensor data to an SD card, and more.

---

## Hardware Requirements

### Microcontroller
- **STM32F407** Discovery Board (or similar STM32F4xx series)
- **Clock**: 168 MHz (HSE with PLL)

### Peripherals Used
| Peripheral | Usage | Pins |
|------------|-------|------|
| USART2 | Serial communication (115200 baud) | TX, RX |
| RTC | Real-time clock with alarm functionality | Internal LSI |
| ADC1 | Analog sensor data acquisition | PA1 (Channel 1) |
| SPI1 | SD Card communication | MOSI, MISO, SCK |
| GPIO | 4x LEDs for visual feedback | PD12-PD15 |
| GPIO | DHT11 sensor interface | PA0 |
| GPIO | SD Card CS pin | PA4 |
| TIM1 | System timebase for HAL | Internal |

### External Components
- **SD Card Module** (SPI interface) - for data logging
- **DHT11 Sensor** (optional) - for temperature/humidity sensing
- **UART-to-USB Converter** - for serial terminal connection

---

## FreeRTOS Features Demonstrated

### 1. Multi-tasking
The project creates **6 concurrent tasks** with different priorities:

```c
xTaskCreate(Menue_Task, "MENUE", 512, NULL, 1, &MenueTask_Handler);
xTaskCreate(PrintTask_Task, "PRINTMENUE", 512, NULL, 2, &printTask_Handler);
xTaskCreate(Animation_Task, "ANIMATION", 512, NULL, 2, &AnimationTask_Handler);
xTaskCreate(ComdHandle_Task, "CMDHNDL", 512, NULL, 2, &ComdHandleTask_Handler);
xTaskCreate(CmdProcess_Task, "CMDPROCESS", 512, NULL, 2, &CmdProcessTask_Handler);
xTaskCreate(ArgProcess_Task, "ARGROCESS", 512, NULL, 2, &ArgProcessTask_Handler);
```

### 2. Queue Management
Three queues are used for inter-task communication:
- **command_queue**: Transmits command codes between tasks (10 elements)
- **argument_queue**: Transmits command arguments with pointers (10 elements)
- **uart_print_queue**: Manages UART printing from multiple tasks (10 elements)

### 3. Software Timers
Implements both auto-reload and one-shot timers:
- **LED Toggle Timers** (4 timers): Periodic auto-reload timers for LED blinking at user-defined rates
- **Data Logging Timer**: Periodic timer for sensor data acquisition and SD card logging

### 4. Memory Management
Demonstrates FreeRTOS dynamic memory allocation:
- Uses `pvPortMalloc()` and `vPortFree()` for heap management
- Implements proper memory cleanup to prevent leaks
- Configured with 15KB heap size (`configTOTAL_HEAP_SIZE = 15360`)

### 5. Task Notifications
Uses task notifications as a lightweight synchronization mechanism:
```c
xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);  // Wait for notification
xTaskNotify(MenueTask_Handler, 0, eNoAction); // Send notification
```

### 6. Critical Sections
Protects shared resources using critical sections:
```c
taskENTER_CRITICAL();
// Protected code
taskEXIT_CRITICAL();
```

### 7. Static Allocation
Implements static memory allocation for Idle and Timer tasks to meet specific memory requirements.

---

## Project Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     UART Interrupt (ISR)                     │
│         Receives user input character-by-character           │
└────────────────────────┬────────────────────────────────────┘
                         │ Notifies on '\r'
                         ▼
              ┌──────────────────────┐
              │  ComdHandle_Task     │
              │  Priority: 2         │
              │  - Parses commands   │
              │  - Validates input   │
              └──┬────────────────┬──┘
                 │                │
     ┌───────────▼─────┐    ┌─────▼──────────┐
     │ command_queue   │    │ argument_queue │
     └───────┬─────────┘    └─────┬──────────┘
             │                    │
    ┌────────▼──────────┐  ┌──────▼────────────┐
    │ CmdProcess_Task   │  │ ArgProcess_Task   │
    │ Priority: 2       │  │ Priority: 2       │
    │ - Processes cmds  │  │ - Processes args  │
    └────────┬──────────┘  └──────┬────────────┘
             │                    │
             └────────┬───────────┘
                      │ Notifies
              ┌───────▼──────────┐
              │  Menue_Task      │◄──────┐
              │  Priority: 1     │       │
              │  - Displays menu │       │
              └──────────────────┘       │
                                         │
    ┌──────────────────┐      ┌─────────┴─────────┐
    │ Animation_Task   │      │  PrintTask_Task   │
    │ Priority: 2      │      │  Priority: 2      │
    │ - Runs animation │      │  - UART printing  │
    └──────────────────┘      └───────────────────┘
             │                         │
             │                ┌────────▼────────┐
             │                │ uart_print_queue│
             │                └─────────────────┘
             │
    ┌────────▼──────────────────────────────────┐
    │          Software Timers                  │
    │  - LED Toggle Timers (x4)                 │
    │  - Data Logging Timer                     │
    └─────┬──────────────────┬──────────────────┘
          │                  │
    ┌─────▼────┐      ┌──────▼──────┐
    │   LEDs   │      │  SD Card +  │
    │  Toggle  │      │  ADC/DHT11  │
    └──────────┘      └─────────────┘
```

---

## Serial Terminal Commands

| Cmd | Description | Arguments | Status |
|-----|-------------|-----------|--------|
| 1 | Start animation in terminal | None | ✓ |
| 2 | Blink LED by ID | LED_ID (1-4), period (seconds) | ✓ |
| 3 | Stop all LEDs blinking | None | ✓ |
| 4 | Get current date and time | None | ✓ |
| 5 | Set the current date | dd mm yy (weekday) | ✓ |
| 6 | Set the current time | hh mm ss | ✓ |
| 7 | Start data logging | period (seconds) | ✓ |
| 8 | Stop data logging | None | ✓ |
| 9 | Get the data logged | None | ✗ |
| 10 | Set alarm date and time | hh mm ss dd | ✗ |
| 0 | Close the terminal | None | ✓ |

### Command Usage Examples
```
> 1          // Start animation
> 2 1 5      // Blink LED 1 with 5 second period
> 4          // Display current date/time
> 5 15 03 24 // Set date: 15-March-2024
> 6 14 30 00 // Set time: 14:30:00
> 7 10       // Start logging every 10 seconds
> 8          // Stop logging
> 0          // Close terminal
```

---

## Task Details

### 1. **Menue_Task** (Priority: 1)
- **Purpose**: Displays the main menu on the serial terminal
- **Stack Size**: 512 words
- **Behavior**: Blocks on task notification, displays menu when notified
- **Location**: Core/Src/main.c:734

```c
void Menue_Task(void *argument) {
    char *str = menu;
    while (1) {
        HAL_UART_Transmit(&huart2, str, strlen(str), HAL_MAX_DELAY);
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    }
}
```

### 2. **PrintTask_Task** (Priority: 2)
- **Purpose**: Centralized UART printing task
- **Stack Size**: 512 words
- **Behavior**: Waits on `uart_print_queue`, prints messages, frees memory
- **Benefits**: Prevents UART conflicts between tasks
- **Location**: Core/Src/main.c:747

### 3. **Animation_Task** (Priority: 2)
- **Purpose**: Displays ASCII art animation
- **Stack Size**: 512 words
- **Behavior**: Shows converging star pattern when notified
- **Location**: Core/Src/main.c:759

### 4. **ComdHandle_Task** (Priority: 2)
- **Purpose**: Command and argument handler
- **Stack Size**: 512 words
- **Behavior**:
  - Waits for UART interrupt notification
  - Parses received buffer
  - Validates input format
  - Routes to command or argument queue
- **Location**: Core/Src/main.c:818

### 5. **CmdProcess_Task** (Priority: 2)
- **Purpose**: Processes commands without arguments
- **Stack Size**: 512 words
- **Behavior**:
  - Receives command code from queue
  - Prompts for arguments if needed
  - Executes commands directly if no arguments required
- **Location**: Core/Src/main.c:865

### 6. **ArgProcess_Task** (Priority: 2)
- **Purpose**: Processes commands with arguments
- **Stack Size**: 512 words
- **Behavior**:
  - Receives argument data from queue
  - Executes command with provided parameters
  - Frees allocated memory after processing
- **Location**: Core/Src/main.c:965

---

## FreeRTOS Synchronization Mechanisms

### Queues
```c
// Queue creation (main.c:238-240)
argument_queue = xQueueCreate(10, sizeof(APP_CMD_t *));
command_queue = xQueueCreate(10, sizeof(uint8_t));
uart_print_queue = xQueueCreate(10, sizeof(char[10]));

// Sending to queue
xQueueSend(command_queue, &command_code, portMAX_DELAY);

// Receiving from queue
xQueueReceive(command_queue, &cmd_code, portMAX_DELAY);
```

### Task Notifications
```c
// From ISR (faster than queues for simple notifications)
xTaskNotifyFromISR(ComdHandleTask_Handler, 0, eNoAction, &xHigherPriorityTaskWoken);

// From normal task context
xTaskNotify(MenueTask_Handler, 0, eNoAction);

// Wait for notification
xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
```

### Software Timers
```c
// Create timer (main.c:1126)
led_timer_handle_array[LED_ID - 1] = xTimerCreate(
    "LED-TIMER",           // Timer name
    toggle_duration,       // Period in ticks
    pdTRUE,               // Auto-reload
    NULL,                 // Timer ID
    led_toggle_array[LED_ID - 1]  // Callback function
);

// Start/Stop timer
xTimerStart(led_timer_handle_array[LED_ID - 1], portMAX_DELAY);
xTimerStop(led_timer_handle_array[LED_ID - 1], portMAX_DELAY);
```

---

## Peripheral Integration

### 1. UART (USART2)
- **Baud Rate**: 115200
- **Mode**: Interrupt-driven reception, blocking transmission
- **Usage**: Serial terminal I/O
- **Implementation**:
  - RX uses `HAL_UART_Receive_IT()` for interrupt-based character reception
  - TX uses `HAL_UART_Transmit()` for blocking transmission
  - ISR callback notifies tasks when '\r' received (main.c:1060)

### 2. RTC (Real-Time Clock)
- **Clock Source**: LSI (Low-Speed Internal oscillator)
- **Features**:
  - 24-hour format
  - Date tracking with weekday
  - Alarm functionality with interrupt
  - Backup register for persistence
- **Functions**:
  - `getDateTime()`: Reads and displays current date/time (main.c:1211)
  - `setCurrentTime()`: Sets time in BCD format (main.c:687)
  - `setCurrentDate()`: Sets date in BCD format (main.c:701)
  - `setCurrentAlarm()`: Configures RTC alarm (main.c:714)

### 3. ADC (Analog-to-Digital Converter)
- **Resolution**: 12-bit
- **Channel**: ADC1 Channel 1 (PA1)
- **Usage**: Sensor data acquisition for logging
- **Mode**: Polling mode (main.c:1176)

### 4. SPI1 (SD Card Interface)
- **Mode**: Master
- **Speed**: Prescaler 32
- **Data Size**: 8-bit
- **Usage**: SD card communication via FatFS
- **Files**: File_Handling_RTOS.c provides abstraction layer

### 5. GPIO
- **LEDs**: PD12, PD13, PD14, PD15 (4 user LEDs)
- **DHT11**: PA0 (bidirectional for DHT11 protocol)
- **SD CS**: PA4 (SPI chip select)

---

## File Structure

```
Core/
├── Src/
│   ├── main.c                    # Main application, tasks, and command handlers
│   ├── freertos.c                # FreeRTOS initialization (minimal)
│   ├── stm32f4xx_it.c           # Interrupt handlers
│   ├── stm32f4xx_hal_msp.c      # HAL MSP (peripheral initialization)
│   ├── DHT11.c                   # DHT11 temperature/humidity sensor driver
│   ├── File_Handling_RTOS.c     # SD card file operations (FatFS wrapper)
│   ├── fatfs_sd.c                # FatFS SD card low-level driver
│   └── system_stm32f4xx.c       # System initialization
│
├── Inc/
│   ├── main.h                    # Main header file
│   ├── FreeRTOSConfig.h         # FreeRTOS configuration
│   ├── stm32f4xx_it.h           # Interrupt handler declarations
│   ├── stm32f4xx_hal_conf.h     # HAL configuration
│   ├── DHT11.h                   # DHT11 driver header
│   ├── File_Handling_RTOS.h     # File handling header
│   └── fatfs_sd.h                # FatFS SD driver header

Middlewares/
└── Third_Party/
    └── FreeRTOS/                 # FreeRTOS kernel source code

```

---

## How It Works

### System Initialization Flow
```
1. HAL_Init() - Initialize HAL library
2. SystemClock_Config() - Configure 168 MHz system clock
3. Peripheral Initialization:
   - GPIO (LEDs, SD CS, DHT11)
   - USART2 (115200 baud)
   - RTC (with backup register check)
   - ADC1 (12-bit resolution)
   - SPI1 (SD card interface)
   - FatFS
4. SD Card Setup:
   - Mount SD card
   - Format (delete existing files)
   - Create ADC_DATA.TXT
   - Unmount
5. FreeRTOS Setup:
   - Create 3 queues
   - Create 6 tasks
   - Start scheduler (vTaskStartScheduler)
6. Tasks run in parallel under RTOS control
```

### User Interaction Flow
```
1. User connects to serial terminal (115200 baud)
2. Menu is displayed by Menue_Task
3. User enters command (e.g., "2")
4. UART ISR:
   - Receives characters one by one
   - Stores in rx_buffer
   - On '\r', notifies ComdHandle_Task
5. ComdHandle_Task:
   - Parses command from buffer
   - Validates format
   - Sends to command_queue
6. CmdProcess_Task:
   - Receives command code
   - Determines if arguments needed
   - If no args: executes immediately
   - If args needed: prompts user, waits for input
7. If arguments required:
   - User enters arguments (e.g., "1 5")
   - UART ISR notifies ComdHandle_Task again
   - ComdHandle_Task sends to argument_queue
   - ArgProcess_Task executes command with arguments
8. Task notifies Menue_Task to redisplay menu
9. Cycle repeats
```

### LED Blinking Mechanism
```c
// When user enters: "2 3 2" (Blink LED 3 with 2-second period)
1. ArgProcess_Task receives command
2. Calls led_toggle_start(2000) with LED_ID=3
3. Creates/configures software timer:
   - Period: 2000 ticks (2 seconds)
   - Callback: led3_toggle()
   - Auto-reload: pdTRUE
4. Timer starts, callback executes every 2 seconds
5. Callback toggles GPIO PD14 (LED 3)
6. Timer runs until stopped by command "3"
```

### Data Logging Flow
```
1. User enters: "7 5" (Log every 5 seconds)
2. Creates data_log_timer with 5-second period
3. Timer callback log_sensor_data() executes periodically:
   a. Get RTC date/time
   b. Read ADC value (sensor data)
   c. Mount SD card
   d. Format: "[index] (Date Time) --> ADC_VALUE"
   e. Append to ADC_DATA.TXT using Update_File()
   f. Transmit data to UART
   g. Unmount SD card
   h. Increment index
4. Continues until user enters "8" (Stop logging)
```

---

## Code Examples

### FreeRTOS Configuration (FreeRTOSConfig.h)
```c
#define configUSE_PREEMPTION                     1    // Preemptive scheduling
#define configSUPPORT_STATIC_ALLOCATION          1    // Static memory allocation
#define configSUPPORT_DYNAMIC_ALLOCATION         1    // Dynamic allocation (heap)
#define configCPU_CLOCK_HZ                       (SystemCoreClock)  // 168 MHz
#define configTICK_RATE_HZ                       1000  // 1ms tick
#define configMAX_PRIORITIES                     7     // 0-6 priority levels
#define configMINIMAL_STACK_SIZE                 128   // Minimum stack (words)
#define configTOTAL_HEAP_SIZE                    15360 // 15KB heap
#define configUSE_TIMERS                         1     // Enable software timers
#define configTIMER_TASK_PRIORITY                6     // Highest priority for timers
#define configUSE_MUTEXES                        1     // Enable mutex support
```

### UART Interrupt Handler (main.c:1060)
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART2)
    {
        if (rx_index == 0) {
            memset(&rx_buffer, 0, sizeof(rx_buffer));
        }

        rx_buffer[rx_index++] = rx_data;

        if (rx_data == '\r')  // Command complete
        {
            rx_index = 0;
            // Notify command handler from ISR
            xTaskNotifyFromISR(ComdHandleTask_Handler, 0, eNoAction,
                              &xHigherPriorityTaskWoken);
        }

        HAL_UART_Receive_IT(&huart2, &rx_data, 1);  // Prepare for next char
    }

    if (xHigherPriorityTaskWoken) {
        taskYIELD();  // Context switch if needed
    }
}
```

### Memory Management Example (File_Handling_RTOS.c:60)
```c
// Allocate memory using FreeRTOS heap
char *path = pvPortMalloc(20 * sizeof(char));
sprintf(path, "%s", pat);

// Use the memory
fresult = f_opendir(&dir, path);

// Free when done to prevent memory leaks
vPortFree(path);
```

### RTC Alarm Callback (main.c:1052)
```c
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    // Turn on all LEDs when alarm triggers
    HAL_GPIO_WritePin(GPIOD, LED_ID_ARRAY[0], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, LED_ID_ARRAY[1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, LED_ID_ARRAY[2], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, LED_ID_ARRAY[3], GPIO_PIN_SET);
}
```

### DHT11 Sensor Reading (DHT11.c:94)
```c
uint8_t DHT11_Get_Data(int *Temperature, int *Humidity)
{
    DHT11_Start();  // Send start signal

    if (DHT11_Check_Response())
    {
        Rh_byte1 = DHT11_Read();
        Rh_byte2 = DHT11_Read();
        Temp_byte1 = DHT11_Read();
        Temp_byte2 = DHT11_Read();
        SUM = DHT11_Read();

        // Verify checksum
        if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2))
        {
            *Temperature = Temp_byte1;
            *Humidity = Rh_byte1;
            return 1;  // Success
        }
    }
    return -1;  // Error
}
```

---

## Key Learning Points

### 1. **Task Priority and Scheduling**
- Menu task has lower priority (1) than others (2)
- Timer task has highest priority (6) as configured
- Tasks with same priority share CPU time through round-robin scheduling

### 2. **Queue Usage Patterns**
- Command queue: Small data (uint8_t command codes)
- Argument queue: Pointers to dynamically allocated structures
- Print queue: Fixed-size character arrays
- All queues use `portMAX_DELAY` for blocking behavior

### 3. **Memory Management Best Practices**
- Always pair `pvPortMalloc()` with `vPortFree()`
- Memory allocated in one task, freed in another (via queues)
- Critical for long-running embedded systems to prevent heap exhaustion

### 4. **Interrupt Integration**
- UART ISR uses `xTaskNotifyFromISR()` instead of direct function calls
- `taskYIELD()` ensures immediate context switch if needed
- Keeps ISR short and fast, defers processing to tasks

### 5. **Software Timer Benefits**
- Runs in timer daemon task (no additional task needed)
- Automatic period management
- Callback executed in timer task context
- Multiple independent timers with different periods

### 6. **Critical Section Usage**
- Protects `rx_buffer` parsing when accessed from multiple contexts
- Ensures `isCmdOrArg` flag consistency
- Should be kept as short as possible to maintain real-time performance

---

## Future Enhancements

- [ ] Implement command 9 (retrieve logged data from SD card)
- [ ] Complete command 10 (alarm with configurable actions)
- [ ] Add DHT11 temperature/humidity logging
- [ ] Implement mutex for UART access instead of dedicated print task
- [ ] Add command history/recall functionality
- [ ] Implement low-power modes with tickless idle
- [ ] Add CRC checking for SD card data integrity
- [ ] Create binary semaphore for SD card access synchronization

---

## References

- **FreeRTOS Documentation**: https://www.freertos.org/
- **STM32F4 HAL Documentation**: https://www.st.com/
- **FatFS Documentation**: http://elm-chan.org/fsw/ff/
- **Project Repository**: Current directory

---

## License

This project uses:
- FreeRTOS Kernel V10.3.1 (MIT License)
- STM32 HAL Library (BSD-3-Clause)
- FatFS by ChaN (FatFs License)

---

**Created**: 2024
**Last Updated**: 2025
**Microcontroller**: STM32F407
**RTOS**: FreeRTOS V10.3.1
**Development Environment**: STM32CubeIDE
