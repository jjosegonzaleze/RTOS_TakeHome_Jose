/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief : FreeRTOS sensor sampling, UART telemetry, and host commands
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
/* Application sampling, framing, and command handling implemented by Jose. */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint32_t timestamp;
    uint16_t value;
    uint32_t sequence;
} SensorSample;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FRAME_SYNC1          0xAA
#define FRAME_SYNC2          0x55
#define FRAME_VERSION        0x01

#define FRAME_TYPE_SENSOR    0x01
#define FRAME_TYPE_STATUS    0x02

#define CMD_START            0x10
#define CMD_STOP             0x11
#define CMD_SET_RATE         0x12
#define CMD_STATUS           0x13

#define MAX_PAYLOAD_LENGTH   32
#define MAX_FRAME_LENGTH     (7 + MAX_PAYLOAD_LENGTH)

#define MIN_SAMPLE_PERIOD_MS 10U
#define MAX_SAMPLE_PERIOD_MS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
osThreadId_t secondTaskHandle;
osThreadId_t telemetryTaskHandle;
osThreadId_t commandTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t secondTask_attributes = {
    .name = "secondTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t commandTask_attributes = {
    .name = "commandTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

volatile uint8_t sampling_enabled = 1;
volatile uint32_t sample_period_ms = 100;
osMessageQueueId_t sensorQueueHandle;
osMutexId_t uartTxMutexHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartSecondTask(void *argument);
/* USER CODE BEGIN PFP */

void StartTelemetryTask(void *argument);
void StartCommandTask(void *argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/* Diagnostics: inspect RAM variables; do not watch USART2->DR. */
volatile uint32_t uart_rx_bytes = 0;
volatile uint32_t uart_rx_timeouts = 0;
volatile uint32_t uart_rx_errors = 0;
volatile uint32_t uart_rx_busy = 0;
volatile uint32_t uart_rx_ore_observed = 0;
volatile uint32_t uart_rx_last_error = 0;
volatile uint32_t command_frames_valid = 0;
volatile uint32_t command_crc_failures = 0;
volatile uint32_t command_bad_headers = 0;
volatile uint8_t uart_rx_trace[64];

static uint8_t commandRxBuffer[MAX_FRAME_LENGTH * 2];
static uint16_t commandRxLength = 0;

static void ProcessCommand(uint8_t type,
                           const uint8_t *payload,
                           uint8_t length);

static void CommandParserFeed(uint8_t byte)
{
    /* Avoid overflowing the receive buffer. */
    if (commandRxLength >= sizeof(commandRxBuffer))
    {
        commandRxLength = 0;
    }

    commandRxBuffer[commandRxLength++] = byte;

    for (;;)
    {
        /* Find the AA 55 synchronization sequence. */
        uint16_t syncIndex = 0;

        while (syncIndex + 1 < commandRxLength)
        {
            if (commandRxBuffer[syncIndex] == FRAME_SYNC1 &&
                commandRxBuffer[syncIndex + 1] == FRAME_SYNC2)
            {
                break;
            }

            syncIndex++;
        }

        if (syncIndex + 1 >= commandRxLength)
        {
            /* Preserve a trailing AA: it might start the next sync. */
            uint8_t keepSyncByte =
                (commandRxLength > 0 &&
                 commandRxBuffer[commandRxLength - 1] == FRAME_SYNC1);

            commandRxLength = 0;

            if (keepSyncByte)
            {
                commandRxBuffer[commandRxLength++] = FRAME_SYNC1;
            }

            return;
        }

        /* Discard noise before the sync sequence. */
        if (syncIndex > 0)
        {
            memmove(commandRxBuffer,
                    commandRxBuffer + syncIndex,
                    commandRxLength - syncIndex);

            commandRxLength -= syncIndex;
        }

        /* Need sync + version + type + length. */
        if (commandRxLength < 5)
        {
            return;
        }

        uint8_t version = commandRxBuffer[2];
        uint8_t type = commandRxBuffer[3];
        uint8_t payloadLength = commandRxBuffer[4];

        /* Reject invalid headers and search again. */
        if (version != FRAME_VERSION ||
            payloadLength > MAX_PAYLOAD_LENGTH)
        {
            command_bad_headers++;
            memmove(commandRxBuffer,
                    commandRxBuffer + 1,
                    --commandRxLength);

            continue;
        }

        uint16_t totalLength = 7U + payloadLength;

        if (commandRxLength < totalLength)
        {
            return;
        }

        uint16_t receivedCrc =
            (uint16_t)commandRxBuffer[totalLength - 2] |
            ((uint16_t)commandRxBuffer[totalLength - 1] << 8);

        uint16_t calculatedCrc =
            CRC16_CCITT(&commandRxBuffer[2],
                        3U + payloadLength);

        if (receivedCrc != calculatedCrc)
        {
            command_crc_failures++;
            /* Drop one byte, then search for the next sync. */
            memmove(commandRxBuffer,
                    commandRxBuffer + 1,
                    --commandRxLength);

            continue;
        }

        /* Valid frame: process its command. */
        command_frames_valid++;
        ProcessCommand(type,
                       &commandRxBuffer[5],
                       payloadLength);

        /* Consume this complete frame. */
        memmove(commandRxBuffer,
                commandRxBuffer + totalLength,
                commandRxLength - totalLength);

        commandRxLength -= totalLength;
    }
}


static void SendStatusResponse(void)
{
    uint8_t frame[12];

    frame[0] = FRAME_SYNC1;
    frame[1] = FRAME_SYNC2;
    frame[2] = FRAME_VERSION;
    frame[3] = FRAME_TYPE_STATUS;
    frame[4] = 5;

    frame[5] = sampling_enabled ? 1 : 0;

    uint32_t period = sample_period_ms;

    frame[6] = (uint8_t)(period);
    frame[7] = (uint8_t)(period >> 8);
    frame[8] = (uint8_t)(period >> 16);
    frame[9] = (uint8_t)(period >> 24);

    uint16_t crc = CRC16_CCITT(&frame[2], 8);

    frame[10] = (uint8_t)crc;
    frame[11] = (uint8_t)(crc >> 8);

    if (osMutexAcquire(uartTxMutexHandle, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(&huart2, frame, sizeof(frame), 100);
        osMutexRelease(uartTxMutexHandle);
    }
}

static void ProcessCommand(uint8_t type,
                           const uint8_t *payload,
                           uint8_t length)
{
    switch (type)
    {
        case CMD_START:
            if (length == 0)
            {
                sampling_enabled = 1;
            }
            break;

        case CMD_STOP:
            if (length == 0)
            {
                sampling_enabled = 0;
            }
            break;

        case CMD_SET_RATE:
            if (length == 4)
            {
                uint32_t period =
                    (uint32_t)payload[0] |
                    ((uint32_t)payload[1] << 8) |
                    ((uint32_t)payload[2] << 16) |
                    ((uint32_t)payload[3] << 24);

                if (period >= MIN_SAMPLE_PERIOD_MS &&
                    period <= MAX_SAMPLE_PERIOD_MS)
                {
                    sample_period_ms = period;
                }
            }
            break;

        case CMD_STATUS:
            if (length == 0)
            {
                SendStatusResponse();
            }
            break;

        default:
            /* Ignore unsupported commands. */
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uartTxMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  sensorQueueHandle = osMessageQueueNew(16, sizeof(SensorSample), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  secondTaskHandle = osThreadNew(StartSecondTask, NULL, &secondTask_attributes);
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  commandTaskHandle = osThreadNew(StartCommandTask, NULL, &commandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	for(;;)
	{
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  osDelay(500);
	}
  /* USER CODE END 5 */
}

void StartSecondTask(void *argument)
{
	SensorSample sample;
	uint32_t sequence = 0;

    for(;;)
    {
    	if (sampling_enabled)
    	        {
    	            HAL_ADC_Start(&hadc1);

    	            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    	            {
    	                sample.value = HAL_ADC_GetValue(&hadc1);
    	                sample.timestamp = HAL_GetTick();
    	                sample.sequence = sequence++;

    	                osMessageQueuePut(
    	                    sensorQueueHandle,
    	                    &sample,
    	                    0,
    	                    0
    	                );
    	            }

			HAL_ADC_Stop(&hadc1);
			}
    		osDelay(sample_period_ms);
}
}

void StartTelemetryTask(void *argument)
{
    SensorSample receivedSample;

    for(;;)
        {
            osStatus_t status = osMessageQueueGet(
                sensorQueueHandle,
                &receivedSample,
                NULL,
                osWaitForever
            );

            if (status == osOK)
            {
            	uint8_t frame[17];

            	frame[0] = FRAME_SYNC1;
            	frame[1] = FRAME_SYNC2;
            	frame[2] = FRAME_VERSION;
            	frame[3] = FRAME_TYPE_SENSOR;
            	frame[4] = 10;  // payload length

            	frame[5] = (uint8_t)(receivedSample.sequence >> 0);
            	frame[6] = (uint8_t)(receivedSample.sequence >> 8);
            	frame[7] = (uint8_t)(receivedSample.sequence >> 16);
            	frame[8] = (uint8_t)(receivedSample.sequence >> 24);

            	frame[9]  = (uint8_t)(receivedSample.timestamp >> 0);
            	frame[10] = (uint8_t)(receivedSample.timestamp >> 8);
            	frame[11] = (uint8_t)(receivedSample.timestamp >> 16);
            	frame[12] = (uint8_t)(receivedSample.timestamp >> 24);

            	frame[13] = (uint8_t)(receivedSample.value >> 0);
            	frame[14] = (uint8_t)(receivedSample.value >> 8);

            	uint16_t crc = CRC16_CCITT(&frame[2], 13);

            	frame[15] = (uint8_t)(crc >> 0);
            	frame[16] = (uint8_t)(crc >> 8);

                if (osMutexAcquire(uartTxMutexHandle, osWaitForever) == osOK)
                {
                    HAL_UART_Transmit(
                        &huart2,
                        frame,
                        sizeof(frame),
                        100
                    );

                    osMutexRelease(uartTxMutexHandle);
                }
            }
        }
}

//void StartCommandTask(void *argument)
//{
//    uint8_t byte;
//
//    for(;;)
//    {
//        if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK)
//        {
//            CommandParserFeed(byte);
//        }
//
//        osDelay(1);
//    }
//}

/* Diagnostic polling patch. Use interrupt/DMA RX for the final design. */
void StartCommandTask(void *argument)
{
    uint8_t byte;

    for (;;)
    {
        /* Capture ORE before HAL may clear it. This is an observation count,
           not a count of lost bytes; errors occurring inside HAL may be missed. */
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET)
        {
            uart_rx_ore_observed++;
        }

        HAL_StatusTypeDef rxStatus = HAL_UART_Receive(&huart2, &byte, 1, 10);
        uint32_t errorCode = HAL_UART_GetError(&huart2);
        if (errorCode != HAL_UART_ERROR_NONE)
        {
            uart_rx_last_error = errorCode;
        }

        if (rxStatus == HAL_OK)
        {
            uart_rx_trace[uart_rx_bytes % sizeof(uart_rx_trace)] = byte;
            uart_rx_bytes++;
            CommandParserFeed(byte);
            /* Immediately receive the next byte; do not sleep here. */
        }
        else
        {
            if (rxStatus == HAL_TIMEOUT) uart_rx_timeouts++;
            else if (rxStatus == HAL_ERROR) uart_rx_errors++;
            else if (rxStatus == HAL_BUSY) uart_rx_busy++;
            osDelay(1);
        }
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
