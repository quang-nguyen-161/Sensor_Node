/*
 * AHT10.h
 *
 *  Created on: Nov 6, 2025
 *      Author: Windows
 */

#ifndef INC_AHT10_H_
#define INC_AHT10_H_

#include "main.h"
#include <stdint.h>

#define AHT10_ADDRESS 		0X38<<1
#define AHT10_INIT 	  		0xAC
#define AHT10_SOFT_RESET	0xBA

typedef struct
{
	// sensor data = real data *100 to prevent using float
	uint32_t humidity;
	uint32_t temp;
	uint32_t soil_moisture;
	uint32_t battery;
} sensor_typedef;


void aht10_get_data(I2C_HandleTypeDef *i2c, uint8_t i2c_addr, sensor_typedef *m_sensor);
#endif /* INC_AHT10_H_ */
