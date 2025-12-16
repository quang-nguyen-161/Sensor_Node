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
	uint16_t humidity;
	uint16_t temp;
	uint16_t soil_moisture;
	uint16_t battery;
} sensor_typedef;


int aht10_get_data(I2C_HandleTypeDef *i2c, uint8_t i2c_addr, sensor_typedef *m_sensor);
uint16_t read_adc_once(ADC_HandleTypeDef *hadc);
float map_adc_to_percent(uint16_t adc, uint32_t ADC_DRY, uint32_t ADC_WET);

#endif /* INC_AHT10_H_ */
