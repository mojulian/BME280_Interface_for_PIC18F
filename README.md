# BME280 Sensor API
## Introduction 

## Version
File             | Version | Date
-----------------|---------|---------
BME280.c         | 1.0.1   | 18/10/19
BME280.h         | 1.0.1   | 18/10/19

## Integration Details
* Integrate BME280.c and BME280.h into your project
* Two functions has to be modified, such that the api works as expected
* BME_readSPI8bit()
* BME_writeSPI8bit()
you have to set the corresponding SPI registers in your SPI file 
and read/write the buffer in the above mentioned functions!!!

## File information
* bme280.h : This header file contains the declarations of the sensor driver APIs, macros and constants.
* bme280.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interfaces
* SPI 4-wire
* I2C

SPI 3-wire is currently not supported in the API.

## Usage guide
### Initializing the sensor

	BME280_SetDefaultConfiguration();
### Get the Sensor Values!
	BME_ModeInit(); // has to be called each time such that Forced Mode ist Set Before Reading!
	BME_readTemperature();
	BME_readHumidity();
	BME_readPressure();