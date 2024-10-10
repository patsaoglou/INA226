/*
 * ina226.c
 *
 *  Created on: Oct 10, 2024
 *      Author: Pantelos
 */

#include "ina226.h"

ina226_status ina226_init(ina226_handle *ina226, I2C_HandleTypeDef *hi2c1, uint16_t configuration)
{
    uint8_t configuration_data[2];

    ina226->hi2c1 = hi2c1;

    if (HAL_I2C_IsDeviceReady(ina226->hi2c1, INA226_I2C_ADDRESS, 10, 100) != HAL_OK)
    {
        return INA_STATUS_I2C_FAIL;
    }

    // software reset
    configuration_data[0] = (RESET_ENABLE & 0xff00) >> 8;
    configuration_data[1] = 0 & 0x00ff;
    if (HAL_I2C_Mem_Write(ina226->hi2c1, INA226_I2C_ADDRESS, (uint16_t) CONFIG_REG, 1, configuration_data, 2, 100) != HAL_OK)
    {
        return INA_STATUS_I2C_FAIL;
    }

    HAL_Delay(100);

    configuration_data[0] = (configuration & 0xff00) >> 8;
    configuration_data[1] = configuration & 0x00ff;

    if (HAL_I2C_Mem_Write(ina226->hi2c1, INA226_I2C_ADDRESS, (uint16_t) CONFIG_REG, 1, configuration_data, 2, 100) != HAL_OK)
    {
        return INA_STATUS_I2C_FAIL;
    }

    return INA_STATUS_OK;
}

ina226_status ina226_set_cal_reg(ina226_handle *ina226)
{
	uint8_t cal_reg_data[2];
	uint16_t calibration_val = CAL_FINAL;
	cal_reg_data[0] = (calibration_val & 0xff00) >> 8;
	cal_reg_data[1] = calibration_val & 0x00ff;
	
	if (HAL_I2C_Mem_Write(ina226->hi2c1, INA226_I2C_ADDRESS, (uint16_t) CAL_REG, 1, cal_reg_data, 2, 100) != HAL_OK)
	{
		return INA_STATUS_I2C_FAIL;
	}
	return INA_STATUS_OK;
}

uint16_t ina226_read_raw_shunt_voltage(ina226_handle *ina226)
{
	uint8_t raw_shunt_reading[2];

	if (HAL_I2C_Mem_Read(ina226->hi2c1, INA226_I2C_ADDRESS,(uint16_t) SHUNT_VOLTAGE, 1, raw_shunt_reading, 2, 100) != HAL_OK)
	{
		return INA_STATUS_I2C_FAIL;
	}

	return raw_shunt_reading[0] << 8 | raw_shunt_reading[1];
}

uint16_t ina226_read_raw_bus_voltage(ina226_handle *ina226)
{
	uint8_t raw_bus_reading[2];

	if (HAL_I2C_Mem_Read(ina226->hi2c1, INA226_I2C_ADDRESS,(uint16_t) BUS_VOLTAGE, 1, raw_bus_reading, 2, 100) != HAL_OK)
	{
		return INA_STATUS_I2C_FAIL;
	}

	return raw_bus_reading[0] << 8 | raw_bus_reading[1];

}

float ina226_read_bus_voltage(ina226_handle *ina226)
{
	uint16_t raw_voltage = ina226_read_raw_bus_voltage(ina226);

	return raw_voltage * BUS_VOL_STEP_VALUE;
}


float ina226_current_via_reg(ina226_handle *ina226)
{
	uint8_t current_reg_data[2];
	int16_t raw_current;

	if (HAL_I2C_Mem_Read(ina226->hi2c1, INA226_I2C_ADDRESS, (uint16_t)CURRENT_REG, 1, current_reg_data, 2, 100) != HAL_OK)
	{
	    return INA_STATUS_I2C_FAIL;
	}

	raw_current = (int16_t)((current_reg_data[0] << 8) | current_reg_data[1]);

	return raw_current * CURRENT_LSB;
}

//	refer page 16
float ina226_power_via_reg(ina226_handle *ina226)
{
	uint8_t power_reg_data[2];
	int16_t raw_power;
	
	if (HAL_I2C_Mem_Read(ina226->hi2c1, INA226_I2C_ADDRESS, (uint16_t)POWER_REG, 1, power_reg_data, 2, 100) != HAL_OK)
	{
	    return INA_STATUS_I2C_FAIL;
	}
	
	raw_power = (int16_t)((power_reg_data[0] << 8) | power_reg_data[1]);
	
	return raw_power * POWER_LSB;
}

