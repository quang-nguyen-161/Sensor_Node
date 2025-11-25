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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "AHT10.h"
#include "SX1278.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NODE_TO_GATEWAY_MODE 1
#define NODE_TO_NODE_MODE	 0

#define CONTINUOUS_MODE 	 1
#define TIMER_MODE			 0

#define dev_id 2

#define NODE
//#define GATEWAY

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;
sensor_typedef sensor;

int sensor_mode = CONTINUOUS_MODE;
int rf_mode;
int payload[6] = {0};
int ret;
char *ack = "255";
int node_to_node_check = 0;
char buffer[512] = {0};
uint32_t data[5] = {0};
int message;
int message_length;
int is_src = 0;
int connect_to_dest = 0;
int connected_node[5] = {0};
int dest;
void uart_printf(const char *fmt, ...)
{
    char buffer[64]={0};   // change size if needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, 64, 100);
}
uint8_t I2C_CheckDevice(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    // addr must be 7-bit, HAL will shift it automatically
    if (HAL_I2C_IsDeviceReady(hi2c, addr, 3, 100) == HAL_OK)
       {
    	uart_printf("device ok\n");
    	return 1;
       }
    else
    {
    	uart_printf("device not ok\n");
    	return 0;
    }
}
void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    uart_printf("\r\nStarting I2C Scan...\r\n");

    for(uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 2, 10) == HAL_OK)
        {
            uart_printf(" - Device found at 0x%02X\r\n", addr);
        }
    }

    uart_printf("I2C Scan Completed.\r\n\r\n");
}
// --- Return true on success, false on timeout/failure ---
// node_to_gateway_routing: listen for nodes, add to connected_node[], broadcast gateway ID when room
bool node_to_gateway_routing(int *connected_node, uint32_t timeout_ms)
{
	uart_printf("rssi: %d\n",SX1278_RSSI_LoRa(&SX1278));
    uart_printf("Waiting for nodes...\r\n");
    SX1278_LoRaEntryRx(&SX1278, 16, 2000);

    uint32_t start_time = HAL_GetTick();
    bool any_added = false;

    while (HAL_GetTick() - start_time < timeout_ms)
    {
        // Enter RX mode before checking packet
        ret = SX1278_LoRaRxPacket(&SX1278);
        if (ret > 0)
        {
            // reset timeout when receiving something
            start_time = HAL_GetTick();

            memset(buffer, 0, sizeof(buffer));
            SX1278_read(&SX1278, (uint8_t*)buffer, ret);

            int new_id = atoi((char*)buffer);
            // skip invalid id
            if (new_id <= 0) {
                HAL_Delay(10);
                continue;
            }

            uart_printf("Received node ID: %d\r\n", new_id);

            // Check for duplicate
            bool exists = false;
            for (int i = 0; i < 5; i++)
            {
                if (connected_node[i] == new_id)
                {
                    exists = true;
                    break;
                }
            }

            // Insert new node if there is an empty slot
            if (!exists)
            {
                for (int i = 0; i < 5; i++)
                {
                    if (connected_node[i] == 0)
                    {
                        connected_node[i] = new_id;
                        uart_printf("Node %d added at index %d\r\n", new_id, i);
                        any_added = true;
                        break;
                    }
                }
            }

            // Broadcast node ID if still room (last slot empty)
            if (connected_node[4] == 0)
            {
                HAL_Delay(20); // small delay to reduce collision risk

                message_length = sprintf(buffer, "%d", dev_id);
                uart_printf("Broadcast gateway ID: %s\r\n", buffer);

                SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
                HAL_Delay(5);
                SX1278_LoRaTxPacket(&SX1278, (uint8_t*)buffer, message_length, 2000);

                HAL_Delay(80); // give radio time to finish and go back to RX
            }
        }
        else
        {
            // no packet this iteration, small sleep to yield CPU
            HAL_Delay(30);
        }
    }

    if (any_added) {
        uart_printf("Finished listening: nodes discovered.\r\n");
        return true;
    } else {
        uart_printf("Timeout reached without discovering nodes.\r\n");
        return false;
    }
}


// node_to_node_routing: send own ID to find relay (up to 5 attempts),
// then if connected wait and discover neighbour nodes (with timeout)
bool node_to_node_routing(int id, int *dest, int *connected_node, uint32_t timeout_ms)
{
    // 1) If not connected, try to find relay by broadcasting up to 5 times
    if (!connect_to_dest)
    {
        uart_printf("Trying to find relay...\r\n");
        uint32_t start = HAL_GetTick();

        while (HAL_GetTick() - start < timeout_ms)
        {

            // Send our ID
            message_length = sprintf(buffer, "%d", id);
            SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
            SX1278_LoRaTxPacket(&SX1278, (uint8_t*)buffer, message_length, 2000);

            HAL_Delay(120); // small guard delay
        }

            // Enter RX mode and wait for reply with timeout_ms per attempt
            SX1278_LoRaEntryRx(&SX1278, 16, 2000);
           start = HAL_GetTick();

            while (HAL_GetTick() - start < timeout_ms)
            {
                ret = SX1278_LoRaRxPacket(&SX1278);
                if (ret > 0)
                {
                    memset(buffer, 0, sizeof(buffer));
                    SX1278_read(&SX1278, (uint8_t*)buffer, ret);

                    int reply_id = atoi((char*)buffer);
                    if (reply_id > 0)
                    {
                        *dest = reply_id;
                        uart_printf("Connected to relay ID: %d\r\n", *dest);

                        connect_to_dest = 1;
                        return true;    // success
                    }
                }
                HAL_Delay(10); // small pause inside wait loop



        }

        uart_printf("Failed to connect to relay after 5 attempts!\r\n");
        return false;
    }

    // 2) Already connected: wait for neighbor nodes with a discovery timeout
    uart_printf("Connected. Waiting for neighbor nodes...\r\n");

    uint32_t start_time = HAL_GetTick();
    bool any_added = false;

    while (HAL_GetTick() - start_time < timeout_ms)
    {
        // Enter RX mode
        SX1278_LoRaEntryRx(&SX1278, 16, 2000);

        ret = SX1278_LoRaRxPacket(&SX1278);
        if (ret > 0)
        {
            // reset timeout when receiving something
            start_time = HAL_GetTick();

            memset(buffer, 0, sizeof(buffer));
            SX1278_read(&SX1278, (uint8_t*)buffer, ret);

            int new_id = atoi((char*)buffer);
            if (new_id <= 0) {
                HAL_Delay(10);
                continue;
            }

            uart_printf("Received node ID: %d\r\n", new_id);

            // Check duplicate
            bool exists = false;
            for (int i = 0; i < 5; i++)
            {
                if (connected_node[i] == new_id)
                {
                    exists = true;
                    break;
                }
            }

            // Insert if new
            if (!exists)
            {
                for (int i = 0; i < 5; i++)
                {
                    if (connected_node[i] == 0)
                    {
                        connected_node[i] = new_id;
                        uart_printf("Node %d added at index %d\r\n", new_id, i);
                        any_added = true;
                        break;
                    }
                }
            }

            // Broadcast own ID if relay still needs nodes
            if (connected_node[4] == 0)
            {
                HAL_Delay(20);

                message_length = sprintf(buffer, "%d", id);
                uart_printf("Broadcast node ID to help discovery: %s\r\n", buffer);

                SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
                HAL_Delay(5);
                SX1278_LoRaTxPacket(&SX1278, (uint8_t*)buffer, message_length, 2000);

                HAL_Delay(80);
            }
        }
        else
        {
            // no packet
            HAL_Delay(20);
        }
    }

    if (any_added) {
        uart_printf("Node discovery finished (nodes added).\r\n");
    } else {
        uart_printf("Node discovery timed out (no nodes added).\r\n");
    }
    return true; // connected (even if no new nodes added)
}


// check_mode: decide whether node->gateway or node->node mode by reading a single packet
int check_mode(int *dest, int *connected_node_array, uint32_t listen_ms)
{
    memset(buffer, 0, sizeof(buffer));

    // ensure we are in RX before waiting
    SX1278_LoRaEntryRx(&SX1278, 16, 2000);

    uint32_t start = HAL_GetTick();
    // wait up to listen_ms for a packet
    while (HAL_GetTick() - start < listen_ms)
    {
        ret = SX1278_LoRaRxPacket(&SX1278);
        if (ret > 0)
        {
            SX1278_read(&SX1278, (uint8_t*)buffer, ret);
            uart_printf("check_mode rx: %s\n", buffer);
            break;
        }
        HAL_Delay(10);
    }

    int temp = atoi((char*)buffer);
    if (temp == 255)
    {
        uart_printf("node to gateway mode\r\n");

        // LED on to indicate gateway mode
        HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_SET);

        // Listen and populate connected_node_array for up to 10s
        node_to_gateway_routing(connected_node_array, 10000);
        *dest = 0;
        return NODE_TO_GATEWAY_MODE;
    }
    else
    {
        uart_printf("node to node mode\r\n");

        HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

        // Now attempt node-to-node routing (needs own id and dest variable)
        // Ensure you have 'dev_id' or pass node id here. Example using dev_id:
        node_to_node_routing(dev_id, dest, connected_node_array, 2000);
        return NODE_TO_NODE_MODE;
    }
}



void get_sensor()
{
	HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
}
/* USER CODE END 0 */
void set_timer(uint32_t time_ms)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 8000-1;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = time_ms-1;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
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
  //MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim2);
  	I2C_Scan(&hi2c1);
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
  	ret = SX1278_LoRaEntryRx(&SX1278, 16, 2000);
  	//enable invert IQ to communicate with gateway


  	//rf_mode = check_mode(&dest, connected_node, 10000);


  //	payload[0] = dest;
  //	payload[1] = dev_id;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef NODE


	     node_to_node_routing(1, dest, connected_node, 2000);
#endif
#ifdef GATEWAY

	  			uart_printf("gateway ...\r\n");
	  						HAL_Delay(1000);
	  						uart_printf("Sending package...\r\n");

	  						message_length = sprintf(buffer,"%d",0xFF);
	  						ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
	  						uart_printf("Entry: %d\r\n", ret);

	  						uart_printf("Sending %s\r\n", buffer);
	  						ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer,
	  								message_length, 2000);


	  						uart_printf("Transmission: %d\r\n", ret);
	  						uart_printf("Package sent...\r\n");
#endif
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // check correct timer
    {
       get_sensor();
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

