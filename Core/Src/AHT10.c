/*
 * AHT10.c
 *
 *  Created on: Nov 6, 2025
 *      Author: Windows
 */

#include "AHT10.h"






void aht10_init(I2C_HandleTypeDef *i2c, uint8_t i2c_addr)
{
	uint8_t init_cmd[3] = {0xAC, 0x33, 0x00}; //init, data0, data1
	HAL_I2C_Master_Transmit(i2c, i2c_addr, init_cmd, sizeof(init_cmd), 100);
	HAL_Delay(80);
}

int aht10_get_data(I2C_HandleTypeDef *i2c, uint8_t i2c_addr, sensor_typedef *m_sensor)
{
	uint8_t receive_buff[6] = {0};
	uint32_t raw_RH;
	uint32_t raw_temp;

	aht10_init(i2c, i2c_addr);

	HAL_I2C_Master_Receive(i2c, i2c_addr, receive_buff, sizeof(receive_buff), 100);
	if ((receive_buff[0] & 0x80)) return 0;
	else
	if (!(receive_buff[0] & 0x80))
	{
		 raw_RH = ((uint16_t)receive_buff[1] << 12) |
		                 ((uint16_t)receive_buff[2] << 4)  |
		                 (receive_buff[3] >> 4);
		     m_sensor->humidity =(uint16_t) (((float)raw_RH * 100.0f / 1048576.0f)*100);

		        raw_temp = (((uint16_t)receive_buff[3] & 0x0F) << 16) |
		                    ((uint16_t)receive_buff[4] << 8)         |
		                     receive_buff[5];
		     m_sensor->temp =(uint16_t) (((float)raw_temp * 200.0f / 1048576.0f - 50.0f)*100);
		     return 1;
	}
}




