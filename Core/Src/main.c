
#include "main.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "AHT10.h"
#include "SX1278.h"

#define MAX_TX_PACKET_SIZE   60  // adjust as needed
#define MAX_RX_PACKET_SIZE   60
#define SENSOR_PACKET_LEN   8    // sensor fields you copy into TX (3..10 -> 8 bytes)
#define CONNECTED_NODE_COUNT 5

#define dev_id 0x36

#define GATEWAY_ADV_CMD 0X22
#define GATEWAY_BEACON_CMD 0X33
#define CONNECT_GATEWAY_CMD 0X01
#define CONNECT_NODE_CMD 0X10
#define NODE_ADV_CMD 0X11
#define NODE_BEACON_CMD (0x99)
#define SENSOR_PACKET_CMD 0X02
#define FORWARD_PACKET_CMD 0X20
#define CONFIG_PACKET_CMD 0xCF
#define TDMA_SLOT_TIME 2000
#define TDMA_GUARD_TIME 400
#define TDMA_TX_TIME 1200

#define TDMA_NUM_SLOTS 5

#define TDMA_CHILD_NUM_SLOTS 5

#define TDMA_CHILD_SLOT_TIME 800
#define TDMA_CHILD_GUARD_TIME 100
#define TDMA_CHILD_TX_TIME 600

uint8_t transmit_packet[70] = {0};   // same size you used previously
uint8_t sensor_packet[SENSOR_PACKET_LEN] = {0};
uint8_t forward_sensor_packet[65] = {0};
uint8_t con_dev = 0;                 // number of connected devices (maintain consistently)
uint8_t connected_node[CONNECTED_NODE_COUNT] = {0};
uint8_t dest = 0xFF;                 // default destination (0xFF = broadcast)
uint8_t receive_packet[65];
uint8_t connected_to_gateway = 0;
uint32_t gateway_timeout_check;
uint32_t node_timeout_check;
uint8_t connected_to_node = 0;
uint32_t node_dev_timeout_check[5] = {0};
uint32_t transmit_packet_size;
uint8_t forward_packet;
uint8_t gateway_available = 0;
uint8_t node_available = 0;
uint8_t slot = 0;
uint8_t frame  = 0;
uint8_t beacon_start = 0;
uint32_t beacon_tick = 0;
uint8_t beacon_child_start = 0;
uint32_t beacon_child_tick = 0;
uint8_t slot_child = 0;
uint8_t slot_sent = 0;
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

void uart_printf(const char *format,...)
{
	char packet[128];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(packet, sizeof(packet), format, args);
	va_end(args);

	if (len > 0) {
	    if (len > sizeof(packet)) len = sizeof(packet);
	    HAL_UART_Transmit(&huart1, (uint8_t*)packet, len, 500);
	}
}

uint8_t tx_mode_start(uint32_t max_size, uint32_t timeout)
{
    uint8_t ret = SX1278_LoRaEntryTx(&SX1278, max_size, timeout);
    uart_printf("[TX] Entry TX: %d\n", ret);
    return ret;
}

uint8_t tx_mode_send(uint8_t *packet, uint32_t size, uint32_t timeout)
{
    uint8_t ret;

    uart_printf("[TX] TX HEX: ");
    for (uint32_t i = 0; i < size; i++)
        uart_printf("%02X ", packet[i]);
    uart_printf("\n");

    ret = SX1278_LoRaTxPacket(&SX1278, packet, size, timeout);
    uart_printf("[TX] Transmit: %d\n", ret);

    return ret;
}


uint8_t rx_mode_start(uint32_t max_size, uint32_t timeout)
{
    uint8_t ret = SX1278_LoRaEntryRx(&SX1278, max_size, timeout);
    uart_printf("[RX] Entry RX: %d\n", ret);
    return ret;
}

uint8_t rx_mode_standby(uint8_t *packet)
{
    uint8_t ret;
    HAL_Delay(200);
    ret = SX1278_LoRaRxPacket(&SX1278);
    if (ret > 0)
    {
        SX1278_read(&SX1278, packet, ret);

        uart_printf("[RX] Content: ");
        for (uint8_t i = 0; i < ret; i++)
            uart_printf("%02X ", packet[i]);
        uart_printf("\n");
    if (ret > 100)
    {
    	rx_mode_start(60, 100);
    }
        return ret;
    }

    return 0;
}






uint8_t packet_build(uint8_t cmd, uint8_t *transmit_packet)
{
	switch (cmd)
	{
	case CONNECT_GATEWAY_CMD:
		transmit_packet[0] = CONNECT_GATEWAY_CMD;
		transmit_packet[1] = dev_id;
		return 2;
		break;
	case CONNECT_NODE_CMD:
		transmit_packet[0] = CONNECT_NODE_CMD;
		transmit_packet[1] = dev_id;
		return 2;
		break;
	case SENSOR_PACKET_CMD:
		transmit_packet[0] = SENSOR_PACKET_CMD;
		transmit_packet[1] = 0xFF;
		transmit_packet[2] = dev_id;
		transmit_packet[3] = slot;
		transmit_packet[4] = frame;
		for (uint8_t i = 5; i < 13; i++)
					{
						transmit_packet[i] = sensor_packet[i-5];
					}
		return 13;
		break;
	case FORWARD_PACKET_CMD:
		// if connected to gateway then forward the sending packet
		if (connected_to_gateway)
		{
			uint8_t node_count = 0;
			for (uint8_t i =0 ;i < 5; i++)
			{
				if (connected_node[i] != 0) node_count++;
			}
			transmit_packet[0] = FORWARD_PACKET_CMD;
			transmit_packet[1] = 0xFF;
			transmit_packet[2] = dev_id;
			uint8_t tx_size = 13 * node_count + 3;
			for (uint8_t i = 3; i < tx_size ; i++)
			{
				transmit_packet[i] = forward_sensor_packet[i-3];
			}
			return tx_size;
		}
		else if (connected_to_node)
		{
			transmit_packet[0] = FORWARD_PACKET_CMD;
			transmit_packet[1] = dest;
			transmit_packet[2] = dev_id;
			transmit_packet[3] = slot;
			transmit_packet[4] = frame;
			for (uint8_t i = 5; i < 13; i++)
					{
						transmit_packet[i] = sensor_packet[i-5];
					}
			return 13;
		}
		return 0;
		break;
	case NODE_ADV_CMD:
		transmit_packet[0] = NODE_ADV_CMD;
		transmit_packet[1] = dev_id;
		for (uint8_t i = 2; i < 7; i++)
			transmit_packet[i] = connected_node[i-2];
		return 7;
		break;
	case NODE_BEACON_CMD:
		transmit_packet[0] = NODE_BEACON_CMD;
		transmit_packet[1] = dev_id;
		transmit_packet[2] = frame;
		for (uint8_t i = 3; i < 8; i++)
			transmit_packet[i] = connected_node[i-3];
		return 8;
		break;
	}
return 0;
}

void packet_process(uint8_t *receive_packet, uint8_t size)
{
    if (receive_packet == NULL || size == 0) return;

    uint8_t cmd = receive_packet[0];

    switch (cmd)
    {
        case GATEWAY_ADV_CMD:   // gateway broadcast: check slots / join logic
        {
            // Expect at least 7 bytes (0..6) for the list check
            if (size != 7)
            {

                break;
            }

            uint8_t connected = 0;
            uint8_t empty_slot = 0;

            // fields 2..6 are nodes list
            for (uint8_t i = 2; i <= 6; i++)
            {
                if (receive_packet[i] == dev_id)
                {
                	connected = 1;
                	slot = i-2;
                    break; // already listed
                }
                if (receive_packet[i] == 0)
                {
                    empty_slot = 1;

                }
            }

            if (connected && (!connected_to_node))
            {
                uart_printf("[GATEWAY_ADV] node %02X in connection, slot %d\n", dev_id, slot);
                connected_to_gateway = 1;
                connected_to_node = 0;
                gateway_available = 0;

            }
            else if (empty_slot)
            {
                uart_printf("[GATEWAY_ADV] gateway remain slot, add %02X to slot\n", dev_id);
                gateway_available = 1;
                connected_to_gateway = 0;
            }
            else
            {
                uart_printf("[GATEWAY_ADV] gateway no empty slot, cannot join\n");
            }
        }
        break;


        case NODE_ADV_CMD:  // node response to a join (or similar)
        {
            // Expect the packet to include gateway id in receive_packet[1] and the node list in 2..6
            if (size != 7)  break;

            dest = receive_packet[1];

            uint8_t connected = 0;
            uint8_t empty_slot = 0;

            for (int i = 2; i <= 6; i++)
            {
                if (receive_packet[i] == dev_id)
                {
                    connected = 1;
                    slot = i-2;
                    break;
                }
                if (receive_packet[i] == 0)
                {
                    empty_slot = 1;
                }
            }

            if (connected && (!connected_to_gateway))
            {
                uart_printf("[NODE_ADV] node %02X already in connection\n", dev_id);
                connected_to_node = 1;
                node_available = 0;
                dest = receive_packet[1];
                connected_to_gateway = 0;
                node_timeout_check = HAL_GetTick();
            }
            else if (empty_slot)
            {

                uart_printf("[NODE_ADV] dest %02X remain slot, add node %02X to dest\n", dest, dev_id);
                node_available = 1;

            }
            else
            {
                uart_printf("[NODE_ADV] dest %02X have no empty slot, cannot join\n",receive_packet[1]);
            }
        }
        break;
        case GATEWAY_BEACON_CMD:
        	if (connected_to_node) break;
        	if (receive_packet[slot + 3] != dev_id) break;
        	beacon_start = 1;
        	frame = receive_packet[2];
        	HAL_Delay(200);
        	beacon_tick = HAL_GetTick();
        	uart_printf("[GATEWAY_BEACON] beacon start, frame:%d\n",frame);
        	break;
        case NODE_BEACON_CMD:
        			if (connected_to_gateway) break;
                	if (receive_packet[slot + 3] != dev_id) break;
                	beacon_child_start = 1;
                	frame = receive_packet[2];
                	beacon_child_tick = HAL_GetTick();
                	uart_printf("[GATEWAY_BEACON] beacon start, frame:%d\n",frame);
                	break;
        case FORWARD_PACKET_CMD:
        	if (connected_to_node) break;
        	if (receive_packet[1] != dev_id) break;
        	uint8_t child_pos = 0;
        	uint8_t node_id_check = 0;
        	for (uint8_t i = 0; i < 5; i++)
        	{
        		if (connected_node[i] == receive_packet[2])
        			{
        			child_pos = i;
        			node_id_check = 1;
        			break;
        			}
        	}
        	if (!node_id_check) break;
        	uint8_t idx = child_pos*13;
        	for (uint8_t i = idx; i < (idx+13); i++)
        	{
        		forward_sensor_packet[i] = receive_packet[i-idx];
        	}
        	break;
        case CONNECT_NODE_CMD:
                	for (int i = 0; i < 5; i++)
                		        {
                		            if (connected_node[i] == receive_packet[1])
                		            {
                		                uart_printf("Device %02X already connected\n", receive_packet[1]);

                		                break;
                		            }
                		            else if (connected_node[i] == 0)
                		            {
                		                connected_node[i] = receive_packet[1];


                		                uart_printf("Added dev %02X\n", receive_packet[1]);
                		                break;
                		            }
                		        }
                		       break;
        default:
            // unknown command
            break;
    }
}

bool interval_check(uint32_t *target, uint32_t timeout)
{
    uint32_t current = HAL_GetTick();

    if (current - *target >= timeout)
    {
        *target = current;
        return true;
    }
    return false;
}

void frame_timeout_check(uint8_t frame_timeout)
{

}

void node_check(uint32_t timeout_ms)
{
     uint32_t now = HAL_GetTick();

    for (int i = 0; i < 5; i++)
    {
        if (connected_node[i] != 0 )
        {
            if ((now - node_dev_timeout_check[i]) > timeout_ms)
            {
                uart_printf("Device %02X OFFLINE\n", connected_node[i]);

                node_dev_timeout_check[i] = 0;

                connected_node[i] = 0x00;   // optional: remove
            }
        }
    }
}
sensor_typedef m_sensor;

int main(void)
{

  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();


  SX1278_hw.dio0.port =GPIOA;
  SX1278_hw.dio0.pin = GPIO_PIN_2;
  SX1278_hw.nss.port = GPIOA;
  SX1278_hw.nss.pin = GPIO_PIN_4;
  SX1278_hw.reset.port = GPIOA;
  SX1278_hw.reset.pin = GPIO_PIN_3;
  SX1278_hw.spi = &hspi1;

   SX1278.hw = &SX1278_hw;

    	uart_printf("Configuring LoRa module\r\n");
    	SX1278_init(&SX1278, 433000000, SX1278_POWER_20DBM, SX1278_LORA_SF_7,
    	SX1278_LORA_BW_250KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 15);
    	uart_printf("Done configuring LoRaModule\r\n");
    	SX1278_LoRaEntryRx(&SX1278, 16, 2000);
    	uint32_t last_node_adv = 0;
    	uint32_t last_gateway_connect = 0;
    	uint32_t last_node_connect = 0;
    	uint8_t send_node_beacon = 0;
    	while (1)
    	{
    	    /* ---------- RX polling ---------- */
    	    uint8_t rx_size = rx_mode_standby(receive_packet);

    	    if (rx_size > 0)
    	    {
    	        packet_process(receive_packet, rx_size);
    	    }
    	    // node avdvertise
    	    uint8_t node_count = 0;
    	    for (uint8_t i = 0; i < 5; i++)
    	    {
    	    	if (connected_node[i] != 0) node_count ++;
    	    }

    	    if (node_count <5 && connected_to_gateway && interval_check(&last_node_adv, 7000))
    	    {
    	    	uint8_t tx_size = packet_build(NODE_ADV_CMD, transmit_packet);
    	    	    		tx_mode_start(tx_size, 1000);
    	    	    		tx_mode_send(transmit_packet, tx_size, 1000);
    	    	    		rx_mode_start(65, 1000);
    	    }

    	    /* ---------- Connection handling ---------- */
    	    if (gateway_available &&
    	        !connected_to_gateway &&
    	        !connected_to_node &&
				interval_check(&last_gateway_connect, 2000))
    	    {
    	        uint8_t tx_size = packet_build(CONNECT_GATEWAY_CMD, transmit_packet);

    	        tx_mode_start(tx_size, 500);
    	        tx_mode_send(transmit_packet, tx_size, 500);
    	        rx_mode_start(MAX_RX_PACKET_SIZE, 1000);
    	    }
    	    else if (node_available &&
    	             !connected_to_node &&
    	             !connected_to_gateway &&
					 interval_check(&last_node_connect, 2000))
    	    {

    	        uint8_t tx_size = packet_build(CONNECT_NODE_CMD, transmit_packet);

    	        tx_mode_start(tx_size, 500);
    	        tx_mode_send(transmit_packet, tx_size, 500);
    	        rx_mode_start(MAX_RX_PACKET_SIZE, 1000);
    	    }

    	    if (beacon_start && connected_to_gateway && !connected_to_node)
    	    {
    	    	if (slot < 3)
    	    	{
    	    		if (slot_sent && !send_node_beacon)
    	    		{
    	    			uint8_t tx_size = packet_build(NODE_BEACON_CMD, transmit_packet);
    	    			tx_mode_start(8, 500); tx_mode_send(transmit_packet, tx_size, 500);
    	    			rx_mode_start(65, 1000); send_node_beacon = 1;
    	    		}
    	    	}
    	    	else
    	    	{
    	    		if (!send_node_beacon)
    	    		{
    	    			uint8_t tx_size = packet_build(NODE_BEACON_CMD, transmit_packet);
    	    			tx_mode_start(8, 500); tx_mode_send(transmit_packet, tx_size, 500);
    	    			rx_mode_start(65, 1000); send_node_beacon = 1;
    	    		}
    	    	}
    	    }

    	    /* ---------- TDMA parent slot ---------- */
    	    if (beacon_start &&
    	        connected_to_gateway &&
    	        !connected_to_node)
    	    {

    	        uint32_t now = HAL_GetTick() - beacon_tick;

    	        uint32_t slot_start = slot * TDMA_SLOT_TIME;
    	        uint32_t tx_window_start = slot_start + TDMA_GUARD_TIME;
    	        uint32_t tx_window_end   = slot_start + TDMA_SLOT_TIME - TDMA_GUARD_TIME;

    	        if (!slot_sent &&
    	            now >= tx_window_start &&
    	            now < tx_window_end)
    	        {
    	            uint8_t tx_size =
    	                packet_build(SENSOR_PACKET_CMD, transmit_packet);

    	            uint32_t remaining = tx_window_end - now;

    	            tx_mode_start(tx_size, remaining);
    	            uint8_t ret = tx_mode_send(transmit_packet, tx_size, 500);
    	            uart_printf("send sensor: %d\n", ret);
    	            HAL_Delay(300);
    	             tx_size = packet_build(FORWARD_PACKET_CMD, transmit_packet);
    	             tx_mode_start(tx_size, remaining);
    	            ret = tx_mode_send(transmit_packet, tx_size, 500);
    	            uart_printf("send forward: %d\n", ret);
    	            slot_sent = 1;
    	        }

    	        /* end of TDMA frame */
    	        if (now >= TDMA_SLOT_TIME * TDMA_NUM_SLOTS)
    	        {
    	        	send_node_beacon = 0;
    	            beacon_start = 0;
    	            slot_sent    = 0;
    	            rx_mode_start(MAX_RX_PACKET_SIZE, 1000);
    	            uart_printf("[TDMA] parent frame ended\n");
    	        }
    	    }

    	    /* ---------- TDMA child slot ---------- */
    	    if (beacon_child_start &&
    	        !connected_to_gateway &&
    	        connected_to_node)
    	    {
    	        uint32_t now = HAL_GetTick() - beacon_child_tick;

    	        uint32_t slot_start = slot * TDMA_CHILD_SLOT_TIME;
    	        uint32_t tx_window_start = slot_start + TDMA_CHILD_GUARD_TIME;
    	        uint32_t tx_window_end   = slot_start + TDMA_CHILD_SLOT_TIME - TDMA_CHILD_GUARD_TIME;

    	        if (!slot_sent &&
    	            now >= tx_window_start &&
    	            now < tx_window_end)
    	        {
    	            uint8_t tx_size =
    	                packet_build(FORWARD_PACKET_CMD, transmit_packet);

    	            uint32_t remaining = tx_window_end - now;

    	            tx_mode_start(tx_size, remaining / 2);
    	            tx_mode_send(transmit_packet, tx_size, remaining / 2);

    	            slot_sent = 1;
    	        }

    	        /* end of TDMA child frame */
    	        if (now >= TDMA_CHILD_SLOT_TIME * TDMA_CHILD_NUM_SLOTS)
    	        {
    	            beacon_child_start = 0;
    	            slot_sent          = 0;
    	            rx_mode_start(MAX_RX_PACKET_SIZE, 1000);
    	            uart_printf("[TDMA] child frame ended\n");
    	        }
    	    }

    	    HAL_Delay(200);   // cooperative scheduling
    	}

    	}


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
}


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


