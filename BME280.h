/* 
 * File:   BME280.h
 * Author: Jason McGuire (j.mcguire.2015@ieee.org)
 * Description: A library for a BME280 Weather Sensor for PIC18F using SPI
 * See example.c for implementation of the library
 * Sample Product: http://www.adafruit.com/products/2652
 * Created on October 13, 2015, 9:08 AM
 
License Information:
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
 
 */
 
#ifndef BME280_H
#define BME280_H
 
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <string.h>
#include <math.h>
#include <pic18f27q10.h>
#include "mcc_generated_files/mcc.h"

#define BME_CSB IO_RA3_LAT // Chip Select
/**
    Section: Register Names 

    MSB mentioned
    for LSB/XLSB increment MSB address by +1/+2
*/
#define BME_HUM_REG    0xFD // BME280 humidity MSB --> has LSB at 0xFE
#define BME_TEMP_REG   0xFA // BME280 temperature reg MSB --> has LSB at 0xFB and XLSB at 0xFC
#define BME_PRESS_REG  0xF7 // BME280 pressure reg MSB --> has LSB at 0xF9 and XLSB at 0xF8
#define BME_CONFIG     0XF5 // BME280 Configure Register
#define BME_CTRL_MEAS  0XF4 // BME280 measurement control
#define BME_STATUS     0XF3 // BME280 Status byte
#define BME_CTRL_HUM   0xF2  // BME280 humidity register settings

#define BME_RESET      0XE0 // BME280 to be reset!
#define BME_ID         0XD0 // BME280 ID register

/**
    Section: Trimming Parameter Temperature
*/
#define BME_DIG_T1_REG 0x88 // BME280 temp calibration coefficients...
#define BME_DIG_T2_REG 0x8A
#define BME_DIG_T3_REG 0x8C
/**
    Section: Trimming Parameter Humidity
*/
#define BME_DIG_H1_REG 0xA1 // BME280 humidity calibration coefficients...
#define BME_DIG_H2_REG 0xE1
#define BME_DIG_H3_REG 0xE3
#define BME_DIG_H4_REG 0xE4
#define BME_DIG_H5_REG 0xE5
#define BME_DIG_H6_REG 0xE7
/**
    Section: Trimming Parameter Pressure
*/
#define BME_DIG_P1_REG 0x8E // BME280 pressure calibration coefficients...
#define BME_DIG_P2_REG 0x90
#define BME_DIG_P3_REG 0x92
#define BME_DIG_P4_REG 0x94
#define BME_DIG_P5_REG 0x96
#define BME_DIG_P6_REG 0x98
#define BME_DIG_P7_REG 0x9A
#define BME_DIG_P8_REG 0x9C
#define BME_DIG_P9_REG 0x9E

/**
  Section: Filter Makros used for Register 0XF5 "CONFIG" bit 4,3,2
*/
#define BME_FILTER_OFF       0X00             // —>  filter off
#define BME_FILTER_2         0X01             //  —>  filter coefficient = 2
#define BME_FILTER_4         0X02             //  —>  filter coefficient = 4
#define BME_FILTER_8         0X03             //  —>  filter coefficient = 8
#define BME_FILTER_16        0X04             //  —>  filter coefficient = 16

#define BME_FILTER_IN_USE    BME_FILTER_2

/**
  Section: Oversampling Makros usef for Register 0XF4 "CTRL_MEAS" bit7,6,5 and/or 4.3.2; Register 0XF2 "CTRL_HUM" bit 2,1,0
*/
#define BME_SAMPLING_SKIPPED 0X00          //  —>  skipped, output set to 0x80000
#define BME_SAMPLING_X1      0X01          //    —>  oversampling x1
#define BME_SAMPLING_X2      0X02          //    —>  oversampling x2
#define BME_SAMPLING_X4      0X03          //    —>  oversampling x4
#define BME_SAMPLING_X8      0X04          //    —>  oversampling x8
#define BME_SAMPLING_X16     0X05          //   —>  oversampling x16

#define BME_SAMPLING_PRESSURE     BME_SAMPLING_X4
#define BME_SAMPLING_TEMPERATURE  BME_SAMPLING_X2
#define BME_SAMPLING_HUMIDITY     BME_SAMPLING_X1

/**
  Section: standby times Makros used for Register 0XF5 "CONFIG" bit 7,6,5
*/
#define BME_STANDBY_0_5      0X00             // —>  standby time = 0.5 ms
#define BME_STANDBY_62_5     0X01             // —>  standby time = 62.5 ms
#define BME_STANDBY_125      0X02             // —>  standby time = 125 ms
#define BME_STANDBY_250      0X03             // —>  standby time = 250 ms
#define BME_STANDBY_500      0X04             // —>  standby time = 500 ms
#define BME_STANDBY_1000     0X05             // —>  standby time = 1000 ms
#define BME_STANDBY_2000     0X06             // —>  standby time = 2000 ms
#define BME_STANDBY_4000     0X07             // —>  standby time = 4000 ms

#define BME_STANDBY_IN_USE   BME_STANDBY_1000

/**
  Section: Modes Makros used for Register 0XF4 "CTRL_MEAS" bit 1,0
  */
#define BME_SleepMode        0X00
#define BME_ForcedMode1      0X01
#define BME_ForcedMode2      0x02
#define BME_NormalMode       0x03

#define BME_MODE_IN_USE      BME_ForcedMode1

signed long int t_fine; // global variable 
/**
    Section: Default Config is as follows:

    BME_CTRL_HUM:       osrs_h[2:0] set to 001 = oversampling x 1
    BME_CTRL_MEAS:      osrs_p[2:0] set to 011 = oversampling x 4
                        osrs_t[2:0] set to 010 = oversampling x 2
                        mode[1:0] set to 01 or 10 = Forced Mode
    BME_CONFIG:         t_sb[2:0] set to 101 = 1000ms standby, only used in normal mode!
                        filter[2:0] set to 001 = Filter Coefficient 2
                        spi3w_en[0] set to 0 = 4Wire SPI used
*/
void BME280_SetDefaultConfiguration(void);

void BME280_Reset(void);

//Forces a sample of the BME280. Also sets oversampling for humidity, temp and press = 1.
//Consult the BME280 Datasheet to change these options. 
  void BME280_Modeinit();

//Write a byte to a register via SPI 
void BME_writeSPI(char, char);

// return a unsigned 16-bit value 
unsigned int BME_readSPI16bit(char);

// return a unsigned 8-bit value
unsigned char BME_readSPI8bit(char);
// returns a unsigned 16-bit (little endian) 
unsigned int BME_readSPI16bit_u_LE(char);

// returns a unsigned 16-bit (little endian)
signed int BME_readSPI16bit_s_LE(char); 

// get temperature and returns it in Celsius
float BME_readTemperature(void); 

// gets RH humidity and returns it as a percentage
float BME_readHumidity(void);

// gets pressure and returns it in kPa.
float BME_readPressure(void); 

// gets device ID
char BME_GetID(void);

// gets device Status
char BME_GetStatus(void);

#endif // BME280_H