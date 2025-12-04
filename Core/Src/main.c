/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AHT10.h"
#include "SX1278.h"
uint8_t sensor_buffer[25];
uint8_t receive_buffer[50];
uint8_t payload[11] = {0};
uint8_t connected_node[5] = {0};
/*
 0 -> cmd
 1 -> dest
 2 -> dev_id
 3,4 -> temp
 5,6 -> humid
 7,8 -> moisture
 9,10 -> bat

 0 = 0x01 -> node, gateway connection
 0 = 0x10 -> node, node connection
 0 = 0x02 -> sending data
 */


#define dev_id 0x01
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

osThreadId ReceiveTaskHandle;
osThreadId TransmitTaskHandle;
osThreadId SensorTaskHandle;
osSemaphoreId dataReadyHandle;
osSemaphoreId transmitSuccessHandle;
osSemaphoreId retryHandle;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void ReceiveTaskInit(void const * argument);
void TransmitTaskInit(void const * argument);
void SensorTaskInit(void const * argument);

/* USER CODE BEGIN PFP */
void uart_printf(const char *format,...)
{
	char buff[128];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);

	if (len > 0) {
	    if (len > sizeof(buff)) len = sizeof(buff);
	    HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 500);
	}
}


int transmit_mode(uint8_t* buffer,uint32_t size)
{
    HAL_Delay(100);
    uart_printf("Sending package...\n");

    int ret = SX1278_LoRaEntryTx(&SX1278, size, 2000);
    uart_printf("Entry: %d\n", ret);
    if (!ret) return 0;

    uart_printf("TX HEX: ");
    for(int i=0;i<size;i++)
        uart_printf("%02X ", buffer[i]);
    uart_printf("\n");

    ret = SX1278_LoRaTxPacket(&SX1278, buffer, size, 2000);

    uart_printf("Transmission: %d\n", ret);
    uart_printf("Package sent...\n");

    return 1;
}

int receive_mode(uint8_t* buffer,uint32_t size)
{
    int ret;
    ret = SX1278_LoRaEntryRx(&SX1278, size, 2000);
    uart_printf("enter receive mode: %d\n", ret);
    if (!ret) return 0;

    HAL_Delay(800);

    uart_printf("Receiving package...\n");

    ret = SX1278_LoRaRxPacket(&SX1278);
    uart_printf("Received: %d bytes\n", ret);

    if (ret > 0)
    {
        SX1278_read(&SX1278, buffer, ret);

        uart_printf("Content HEX: ");
        for(int i=0;i<ret;i++)
        {
            uart_printf("%02X ", buffer[i]);
            //get dest = ack
            if (i == 0) payload[i] = buffer[i];
        }
        uart_printf("\n");
    }

    uart_printf("Package received...\n");
    return ret;   // <-- IMPORTANT: return number of bytes!
}

void packet_process(uint8_t *receive_buffer)
{
	switch (receive_buffer[0])
	        {
				case 0x01:
				{
					if (receive_buffer[1] != 0xFF)
						break;   // not gateway broadcast

					uart_printf("connecting...\n");

					uint8_t connected = 0;
					uint8_t empty_slot = 0;

					// check only once
					for (int i = 1; i <= 5; i++)
					{
						if (receive_buffer[i] == dev_id)
						{
							connected = 1;
							break;               // already in list → stop
						}

						if (receive_buffer[i] == 0)
						{
							empty_slot = 1;      // remember empty slot
						}
					}

					if (connected)
					{
						uart_printf("already in connection\n");
					}
					else if (empty_slot)
					{
						uart_printf("remain slot, add gateway to dest\n");
						payload[1] = 0xFF;
						payload[2] = dev_id;
					}
					else
					{
						uart_printf("no empty slot, cannot join\n");
					}
				}
				break;
	        	case 0x02:
	        		 if (receive_buffer[1] != dev_id) break;
	        		 else
	        		 {
	        			 uart_printf("receive data from %02X\n",receive_buffer[2]);
	        			 break;
	        		 }
	        	break;
	        	case 0x10:
	        		if (receive_buffer[1] != dev_id) break;
	        		else if (connected_node[4] != 0) break;
	        		else
	        		{
	        			for (int i = 0;i<5;i++)
	        			{
	        				if (receive_buffer[2] == connected_node[i])
	        				{
	        					uart_printf("node already connected");
	        				}
	        				else if (connected_node[i] == 0)
	        				{
	        					connected_node[i] = receive_buffer[2];
	        					break;
	        				}
	        			}
	        		}
	        	break;
	        	default:
	        	break;
	        }
}

sensor_typedef m_sensor;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SX1278_hw.dio0.port =GPIOA;
    	SX1278_hw.dio0.pin = GPIO_PIN_2;
    	SX1278_hw.nss.port = GPIOA;
    	SX1278_hw.nss.pin = GPIO_PIN_4;
    	SX1278_hw.reset.port = GPIOA;
    	SX1278_hw.reset.pin = GPIO_PIN_3;
    	SX1278_hw.spi = &hspi1;

    	SX1278.hw = &SX1278_hw;

    	uart_printf("Configuring LoRa module\r\n");
    	SX1278_init(&SX1278, 434000000, SX1278_POWER_20DBM, SX1278_LORA_SF_7,
    	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
    	uart_printf("Done configuring LoRaModule\r\n");
    	SX1278_LoRaEntryRx(&SX1278, 16, 2000);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of dataReady */
  osSemaphoreDef(dataReady);
  dataReadyHandle = osSemaphoreCreate(osSemaphore(dataReady), 1);

  /* definition and creation of transmitSuccess */
  osSemaphoreDef(transmitSuccess);
  transmitSuccessHandle = osSemaphoreCreate(osSemaphore(transmitSuccess), 1);

  /* definition and creation of retry */
  osSemaphoreDef(retry);
  retryHandle = osSemaphoreCreate(osSemaphore(retry), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ReceiveTask */
  osThreadDef(ReceiveTask, ReceiveTaskInit, osPriorityNormal, 0, 256);
  ReceiveTaskHandle = osThreadCreate(osThread(ReceiveTask), NULL);



  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, SensorTaskInit, osPriorityBelowNormal, 0, 256);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_PIN_Pin|NSS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PIN_Pin */
  GPIO_InitStruct.Pin = LED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO_PIN_Pin */
  GPIO_InitStruct.Pin = DIO_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_PIN_Pin NSS_PIN_Pin */
  GPIO_InitStruct.Pin = RESET_PIN_Pin|NSS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void ReceiveTaskInit(void const * argument)
{
    while(1)
    {
        int len = receive_mode(receive_buffer,sizeof(receive_buffer));

        if (len > 0)
        {
            uart_printf("receive success\n");
        }
        else
        {
            uart_printf("receive fail, retry...\n");

        }

        packet_process(receive_buffer);
        osSemaphoreRelease(dataReadyHandle);
        osDelay(100);
    }
}



void SensorTaskInit(void const * argument)
{
    while(1)
    {
        if(osSemaphoreWait(dataReadyHandle, osWaitForever) == osOK)
        {
            if(aht10_get_data(&hi2c1, AHT10_ADDRESS, &m_sensor))
            {
            	payload[2] = dev_id;
            	payload[3] = (uint8_t)(m_sensor.temp >> 8); // high byte
            	payload[4] = (uint8_t)(m_sensor.temp & 0xFF); // low byte

            	payload[5] = (uint8_t)(m_sensor.humidity >> 8); // high byte
            	payload[6] = (uint8_t)(m_sensor.humidity & 0xFF); // low byte
                uart_printf("sensor receive success\n");
                uart_printf("temp: %d, humid: %d\n",m_sensor.temp,m_sensor.humidity);
                uart_printf("passing semaphore to transmit task\n");

                uart_printf("transmit sensor value\n");
                		  if(transmit_mode(payload,sizeof(payload)))
                		  {
                			  uart_printf("transmit data success to:%02X, from:%02X\n",payload[1],payload[2]);
                		  }
                		  else
                		  {
                			  uart_printf("transmit data fail\n");
                		  }



            }
            else
            {
                uart_printf("sensor data not ready, going back to receive task\n");

        }

    }
}}






void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

