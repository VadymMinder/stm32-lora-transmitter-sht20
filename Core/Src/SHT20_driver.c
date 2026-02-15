/*
 * SHT20_driver.c
 *
 *  Created on: Feb 11, 2026
 *      Author: aboba
 */

#include "SHT20_driver.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t settings = 0;
uint32_t err = 0;


float sht20_Temp_Mess(){
	uint8_t val[3] = {0};
	uint8_t cmd = SHT_TRIGGER_T_HOLD;
	HAL_I2C_Master_Transmit(&hi2c1, SHT_ADDR << 1, &cmd, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, SHT_ADDR << 1, val, 3, 100);

	uint16_t rawTemp = (val[0] << 8) | val[1];
	rawTemp = rawTemp & 0xFFFC;
	float temp = -46.85 + 175.72*((rawTemp)/65536.0);
	return temp;
}

float sht20_Hum_Mess(){
	uint8_t val[3] = {0};
	uint8_t cmd = SHT_TRIGGER_RH_HOLD;
	HAL_I2C_Master_Transmit(&hi2c1, SHT_ADDR << 1, &cmd, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, SHT_ADDR << 1, val, 3, 100);

	uint16_t rawHum = (val[0] << 8) | val[1];
	rawHum = rawHum & 0xFFFC;
	float hum = -6 + 125*((rawHum)/65536.0);
	return hum;
}


