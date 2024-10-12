/*
 * ina226.h
 *
 *  Created on: Oct 10, 2024
 *      Author: Pantelos
 */

#ifndef INC_INA226_H_
#define INC_INA226_H_

//	Define here your system HAL so i2c HAL can be used
#include "stm32f1xx_hal.h"

//	Basic firmware driver for INA 226 power monitor IC. All IO happens in blocking mode.
//	Time out time is defined in case measurement never succeeds, so app will not block.

//	Define driver delay based on you timing application requirements
#define TIME_OUT_MS 		100

//	Define values used in CAL_REG based on your ic hardware configuration (in ohms) (page 15)
//	In my case i set max current to 1A and Rs = 0.1. Some rounding simplification were made.

//	Final calibration value
#define CURRENT_LSB			0.000024
#define POWER_LSB			0.0006
#define CAL_FINAL 			2133

//	Address A0 = GND A1 = GND (page 18)
#define INA226_I2C_ADDRESS (0x40 << 1)

//	Configuration Register Map (page 22)
#define CONFIG_REG 			0x00
#define SHUNT_VOLTAGE 		0x01
#define BUS_VOLTAGE 		0x02
#define POWER_REG 			0x03
#define CURRENT_REG			0x04
#define CAL_REG		 		0x05
#define MASK_EN_REG 		0x06
#define ALERT_LIM_REG 		0x07
#define MANUF_ID	 		0xFE
#define DIE_ID 				0xFF

//  Configuration Register bits

//	Mode bits (page 11)
#define CONTINUOUS_SHUNT_V	0x0005
#define CONTINUOUS_BUS_V    0x0006
#define CONTINUOUS_MODE_ALL 0x0007
#define TRIGGER_SHUNT_V		0x0001
#define TRIGGER_BUS_V		0x0010
#define TRIGGER_BOTH		0x0011

//	Conversion time
//  Shunt voltage measurement CT (page 5 for timings, page 24 for bits)
#define V_SHUNT_140us		(0x0000 << 3)
#define V_SHUNT_204us		(0x0001 << 3)
#define V_SHUNT_332us		(0x0002 << 3)
#define V_SHUNT_588us		(0x0003 << 3)
#define V_SHUNT_1_1ms		(0x0004 << 3)
#define V_SHUNT_2_116ms		(0x0005 << 3)
#define V_SHUNT_4_156ms		(0x0006 << 3)
#define V_SHUNT_8_244ms		(0x0007 << 3)

//  Bus voltage measurement CT (page 5 for timings, page 24 for bits)
#define V_BUS_140us			(0x0000 << 6)
#define V_BUS_204us			(0x0001 << 6)
#define V_BUS_332us			(0x0002 << 6)
#define V_BUS_588us			(0x0003 << 6)
#define V_BUS_1_1ms			(0x0004 << 6)
#define V_BUS_2_116ms		(0x0005 << 6)
#define V_BUS_4_156ms		(0x0006 << 6)
#define V_BUS_8_244ms		(0x0007 << 6)

//	Measurement averaging selection (page 5 for timings, page 24 for bits)
#define TIMES_AVG_OFF		(0x0000 << 9)
#define TIMES_AVG_1			(0x0001 << 9)
#define TIMES_AVG_2			(0x0002 << 9)
#define TIMES_AVG_3			(0x0003 << 9)
#define TIMES_AVG_4			(0x0004 << 9)
#define TIMES_AVG_5			(0x0005 << 9)
#define TIMES_AVG_6			(0x0006 << 9)
#define TIMES_AVG_7			(0x0007 << 9)

//	Write this to configuration register to software reset (page 24 for software reset bit)
#define RESET_ENABLE				(0x0001 <<15)

//	Mask that on the value read by MASK_EN_REG to check when all measurements/calculations are ready
#define	CONVERSION_READY_FLG_MASK 	(0x0008)

//	1 LSB step size for adc readings in bus measurement to get voltage(page 5)
#define	BUS_VOL_STEP_VALUE			0.00125

typedef enum
{
	INA_STATUS_OK,
	INA_STATUS_I2C_FAIL,
	INA_STATUS_TIMEOUT,
	INA_CONVERSION_NOT_READY,
	INA_CONVERSION_READY,
}ina226_status;

typedef struct{
	I2C_HandleTypeDef *hi2c1;

}ina226_handle;

ina226_status ina226_init(ina226_handle *ina226, I2C_HandleTypeDef *hi2c1, uint16_t configuration);
ina226_status ina226_set_cal_reg(ina226_handle *ina226);
ina226_status check_if_conversion_ready(ina226_handle *ina226);

uint16_t ina226_read_raw_shunt_voltage(ina226_handle *ina226);
uint16_t ina226_read_raw_bus_voltage(ina226_handle *ina226);

float ina226_read_bus_voltage(ina226_handle *ina226);
float ina226_current_via_reg(ina226_handle *ina226);
float ina226_power_via_reg(ina226_handle *ina226);

#endif /* INC_INA226_H_ */
