# INA226 Driver for STM32

This repository provides a basic firmware driver for the **INA226** power monitor IC, written for STM32 microcontrollers.

## Usage

- In the code below you can find the basic initialization of the the INA226 handle struct and how to get the basic measurements like current, power and bus voltage using the internal registers. Please take into account your desired conversion times since if you read registers before adc conversion is finished you are going to read the previous value. I you want to read the measurement when it is available implement tick delay mechanism considering your application conversion times

- **Update:** I have added a conversion check function that reads the mask register and masks for the conversion ready bit. So read measurement values when conversion bit is set. On flag bit read the bit is reset automatically by the IC.

```c
/* Initialize INA226 */
ina226_handle ina226;
ina226_init(&ina226, &hi2c1, V_BUS_2_116ms | V_SHUNT_2_116ms | CONTINUOUS_MODE_ALL);

/* Set calibration register */
ina226_set_cal_reg(&ina226);

/* Infinite loop to read data */
while (1)
{
    if (check_if_conversion_ready(&ina226) == INA_CONVERSION_READY)
    {
        float current = ina226_current_via_reg(&ina226);
        float bus_voltage = ina226_read_bus_voltage(&ina226);
        float power = ina226_power_via_reg(&ina226);
    }

    HAL_Delay(500);
    // Use the values as needed
}
```

- Below you find the code related with the hardware system configuration. You have to configure those on your own. Refer the datasheet pages referred in the comments for more information.  

```c
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
}
```

# Disclaimer
This code was written fast for basic initialation, usage and testing of a module i had with this IC. An issue i noticed after some testing was that the current measurement was +10-15mA off the correct value. I don't know if that flactuation is within the ICs specification or it is an issue with the module i have. The module i am using is the INA226 CJMCU-226 IIC from aliexpress. 

**Please** if you find any issues on the code or you want to add code for the stuff i haven't included (like alarm etc.), feel free to raise an issue with a PR so i can test the code and merge it. 

**Thanks in advance**

# License
This driver is provided under the MIT License.

