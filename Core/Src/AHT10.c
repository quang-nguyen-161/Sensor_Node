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
	return 1;
}

uint16_t read_adc_once(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Start(hadc);

    // wait for conversion
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
        return 0; // timeout or error

    uint16_t val = HAL_ADC_GetValue(hadc);

    HAL_ADC_Stop(hadc);

    return val;
}

uint16_t moisture_get(ADC_HandleTypeDef *hadc,
                      uint32_t MOISTURE_DRY,
                      uint32_t MOISTURE_WET, sensor_typedef *m_sensor)
{
    uint32_t sum = 0;

    /* ---- Take 10 ADC samples ---- */
    for (uint8_t i = 0; i < 10; i++)
    {
        sum += read_adc_once(hadc);
        HAL_Delay(100);
    }

    /* ---- Average ADC value ---- */
    uint32_t adc = sum / 10;

    int32_t pct_x100;

    /* ---- ADC → moisture percent ×100 ---- */
    if (MOISTURE_WET < MOISTURE_DRY)
    {
        // Typical soil sensor: wet ADC < dry ADC
        pct_x100 = (int32_t)(MOISTURE_DRY - adc) * 10000L /
                   (int32_t)(MOISTURE_DRY - MOISTURE_WET);
    }
    else
    {
        pct_x100 = (int32_t)(adc - MOISTURE_DRY) * 10000L /
                   (int32_t)(MOISTURE_WET - MOISTURE_DRY);
    }

    /* ---- Clamp to 0–100.00% ---- */
    if (pct_x100 < 0)      pct_x100 = 0;
    if (pct_x100 > 10000)  pct_x100 = 10000;
    m_sensor->soil_moisture = (uint16_t)pct_x100;
    return (uint16_t)pct_x100;
}

#define ADC_MAX     4095UL
#define VREF_MV    3300UL   // ADC reference voltage (mV)

uint16_t battery_get(ADC_HandleTypeDef *hadc,
                     uint16_t MAX_VOLTAGE_MV,
                     uint16_t MIN_VOLTAGE_MV, sensor_typedef *m_sensor)
{
    uint32_t sum = 0;

    /* ---- Take 10 ADC samples ---- */
    for (uint8_t i = 0; i < 10; i++)
    {
        sum += read_adc_once(hadc);
        HAL_Delay(10);
    }

    /* ---- Average ADC value ---- */
    uint32_t adc = sum / 10;

    /* ---- ADC → voltage at pin (mV) ---- */
    uint32_t adc_mv = adc * VREF_MV / ADC_MAX;

    /* ---- Compensate 1/2 voltage divider ---- */
    uint32_t battery_mv = adc_mv * 2;

    int32_t pct_x100;

    /* ---- Clamp battery voltage ---- */
    if (battery_mv < MIN_VOLTAGE_MV)
        battery_mv = MIN_VOLTAGE_MV;
    if (battery_mv > MAX_VOLTAGE_MV)
        battery_mv = MAX_VOLTAGE_MV;

    /* ---- Voltage → percent ×100 ---- */
    pct_x100 = (battery_mv - MIN_VOLTAGE_MV) * 10000L /
               (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV);

    /* ---- Clamp percent ---- */
    if (pct_x100 < 0)      pct_x100 = 0;
    if (pct_x100 > 10000)  pct_x100 = 10000;
    m_sensor->battery = (uint16_t)pct_x100;
    return (uint16_t)pct_x100;
}


