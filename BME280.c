/**
  BME280 Interface

  @Company
    Zuend AG

  @File Name
    bme280.c

  @Summary
    This file is used to interface bme280.

  @Description
    This source file provides implementations for Interfacing BME280.
        Device                  :  PIC18F27Q10
        Driver Version          :  2.00
        Compiler                :  XC8 1.45
        MPLAB                   :  MPLAB X 4.15
        MPLAB Code Configurator :  MCC 3.45
 */

/*
    (c) Zuend AG. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY ZuendAG "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH ZuendAG PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL ZuendAG BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF ZuendAG HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, ZuendAG'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO ZuendAG FOR THIS SOFTWARE.

    ZuendAG PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
 */

#include <p18cxxx.h> 
#include "BME280.h"
#include "mcc_generated_files/mcc.h"

// Available Functions:
void BME280_init(); // Starts the BME280. Must use each time you want a new sample. 
void writeSPI(char, char); // Write a byte to a register via SPI
unsigned int readSPI16bit(char); // return a unsigned 16-bit value 
unsigned char readSPI8bit(char); // return a unsigned 8-bit value
unsigned int readSPI16bit_u_LE(char); // returns a unsigned 16-bit (little end)
signed int readSPI16bit_s_LE(char); // returns a unsigned 16-bit (little end)
float readTemp(); // get temperature and returns it in Celsius
float readHumidity(); // gets RH humidity and returns it as a percentage
float readPressure(); // gets pressure and returns it in kPa.

/** 
    Section: To set the configuration for BME280 call this function
*/
void BME280_SetDefaultConfiguration(void){
    BME_CSB = 1; // Chip Select OFF 
    BME280_Reset();
    BME_writeSPI(BME_CTRL_HUM, BME_SAMPLING_HUMIDITY); // set CTRL_HUM register

        // set CTRL_MEAS for the measurement
    uint8_t osrs_t = BME_SAMPLING_TEMPERATURE<<5;
    uint8_t osrs_p = BME_SAMPLING_PRESSURE<<2; 
    uint8_t mode = BME_MODE_IN_USE;
    uint8_t config_measure = (osrs_t|osrs_p)|mode;
    BME_writeSPI(BME_CTRL_MEAS, config_measure);

    // set BME_CONFIG for the measurement
    uint8_t t_sb = BME_STANDBY_IN_USE<<5;
    uint8_t filter = BME_FILTER_IN_USE<<2; 
    uint8_t spi3w_en = 0x00;
    uint8_t configuration = (t_sb|filter)|spi3w_en;
    BME_writeSPI(BME_CONFIG, configuration);
}

void BME280_Reset(void){
    BME_writeSPI(BME_RESET, 0xB6);
    __delay_ms(100);
}

// Starts the BME280 sensor and waits for first sample. MUST be ran each time
// you wish to get a reading. 
void BME280_Modeinit(){
//  BME_writeSPI(BME_CTRL_HUM, BME_SAMPLING_HUMIDITY); // Humidity oversampling = 1
    char ctrl_meas_data= BME_readSPI8bit(BME_CTRL_MEAS);
    char set_mode_and_use_old_data = ctrl_meas_data | BME_MODE_IN_USE;
    BME_writeSPI(BME_CTRL_MEAS, set_mode_and_use_old_data); // Forced mode, Temp/Press oversampling = 1
}

unsigned char BME_readSPI8bit(char reg){
    unsigned char trash;
    BME_CSB = 0;
    SSP1BUF = reg;
    SSP1CON1bits.SSPOV = 0x00;
    while ((SSP1STAT & 0x01) == 0);  // wait for transmission
    trash = SSP1BUF; // clear BF flag
    SSP1BUF = 0; // Send garbage
    while ((SSP1STAT & 0x01) == 0);  // wait for byte
    BME_CSB = 1;
    return SSP1BUF;       // clear flag by reading buffer
}

unsigned int BME_readSPI16bit_u_LE(char reg){ // read 16-bits unsigned little endian
    unsigned int val;
    val = BME_readSPI16bit(reg); 
    return (val >> 8) | (val << 8); // swap upper and lower regs
}

signed int BME_readSPI16bit_s_LE(char reg){ // read 16-bit signed little endian
    return (signed int)BME_readSPI16bit_u_LE(reg);
}



unsigned int BME_readSPI16bit(char reg){
    unsigned int val;
    val = BME_readSPI8bit(reg); // shift in MSB
    val = val << 8 | BME_readSPI8bit(reg+1); // shift in LSB
    return val;
}

void BME_writeSPI(char reg, char data){
    signed char trash;
    BME_CSB = 0;
    SSP1BUF = (reg & ~0x80); // write cmd bit 7 = 0 --> write address which has to be written
    SSP2CON1bits.SSPOV = 0x00;
    while ((SSP1STAT & 0x01) == 0);  // wait for transmission
    trash = SSP1BUF; // clear BF flag
    SSP1BUF = data; // write data to address
    SSP2CON1bits.SSPOV = 0x00;
    while ((SSP1STAT & 0x01) == 0);  // wait for transmission
    BME_CSB = 1;
    trash = SSP1BUF; // clear BF flag 
    return;
}

float BME_readTemperature(void){
    // Calibration Coefficients:
    uint32_t dig_T1 = BME_readSPI16bit_u_LE(BME_DIG_T1_REG); // unsigned long int
    signed long int dig_T2 = BME_readSPI16bit_s_LE(BME_DIG_T2_REG);
    signed long int dig_T3 = BME_readSPI16bit_s_LE(BME_DIG_T3_REG);
    
    // Temperature Raw ADC:
    unsigned long int adc_T = BME_readSPI16bit(BME_TEMP_REG);
    adc_T <<= 8; // move in XLSB register
    adc_T |= BME_readSPI8bit(BME_TEMP_REG + 2);
    adc_T >>= 4; // Only uses top 4 bits of XLSB register 

    // From BME280 data sheet: 
    signed long int var1  = ((((adc_T>>3) - (dig_T1 <<1))) *
	   (dig_T2)) >> 11;
  
    signed long int var2  = (((((adc_T>>4) - (dig_T1)) *
	     ((adc_T>>4) - (dig_T1))) >> 12) *
	     (dig_T3)) >> 14;

    t_fine = var1 + var2;
 
    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float BME_readHumidity(void){
    // Calibration Coefficients:
    unsigned int dig_H1 = BME_readSPI8bit(BME_DIG_H1_REG);
    signed long int dig_H2 = BME_readSPI16bit_s_LE(BME_DIG_H2_REG);
    unsigned int dig_H3 = BME_readSPI8bit(BME_DIG_H3_REG);
    signed long int dig_H4 = (BME_readSPI8bit(BME_DIG_H4_REG) << 4) | (BME_readSPI8bit(BME_DIG_H4_REG+1) & 0xF);
    signed long int dig_H5 = (BME_readSPI8bit(BME_DIG_H5_REG+1) << 4) | (BME_readSPI8bit(BME_DIG_H5_REG) >> 4);
    signed int dig_H6 = (signed int) BME_readSPI8bit(BME_DIG_H6_REG);
    
    // Humidity raw ADC:
    unsigned long int adc_H = BME_readSPI16bit(BME_HUM_REG);
    
    // from BME280 data sheet:
    unsigned long int v_x1_u32r;
    
    v_x1_u32r = t_fine - 76800;
    
    v_x1_u32r = (((((adc_H << 14) - ((dig_H4) << 20) - ((dig_H5) * v_x1_u32r))
            + (16384)) >> 15) * (((((((v_x1_u32r * (dig_H6)) >> 10) *
		    (((v_x1_u32r * (dig_H3)) >> 11) + (32768))) >> 10) +
		  (2097152)) * (dig_H2) + 8192) >> 14));
    
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
			     (dig_H1)) >> 4));
    
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float humidity = (v_x1_u32r>>12);
    return  humidity / 1024.0;
}

float BME_readPressure(void){
    // Calibration Coefficients
    unsigned long int dig_P1 = BME_readSPI16bit_u_LE(BME_DIG_P1_REG);
    signed long int dig_P2 = BME_readSPI16bit_s_LE(BME_DIG_P2_REG);
    signed long int dig_P3 = BME_readSPI16bit_s_LE(BME_DIG_P3_REG);
    signed long int dig_P4 = BME_readSPI16bit_s_LE(BME_DIG_P4_REG);
    signed long int dig_P5 = BME_readSPI16bit_s_LE(BME_DIG_P5_REG);
    signed long int dig_P6 = BME_readSPI16bit_s_LE(BME_DIG_P6_REG);
    signed long int dig_P7 = BME_readSPI16bit_s_LE(BME_DIG_P7_REG);
    signed long int dig_P8 = BME_readSPI16bit_s_LE(BME_DIG_P8_REG);
    signed long int dig_P9 = BME_readSPI16bit_s_LE(BME_DIG_P9_REG);

    // Pressure Raw ADC:
    unsigned long int adc_P = BME_readSPI16bit(BME_PRESS_REG);
    adc_P <<= 8; // move in XLSB register
    adc_P |= BME_readSPI8bit(BME_PRESS_REG + 2);
    adc_P >>= 4; // Only uses top 4 bits of XLSB register 
    
    // from BME280 datasheet 
    signed long int var1 = ((signed long int)t_fine >> 1);
    var1 = var1 - (signed long int)64000;
    signed long int var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0){
        return 0; // avoid exception caused by division by zero
    }
    unsigned long int p = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000){
        p = (p << 1) / ((unsigned long int)var1);
    }
    else
    {
        p = (p / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((signed long int)(p>>2)) * ((signed long int)dig_P8))>>13;
    p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
    float pressure = p/100; // changed from 1000 to 100
    return pressure;
}

char BME_GetID(void){
    return(BME_readSPI8bit(BME_ID));
}

char BME_GetStatus(void){
    return(BME_readSPI8bit(BME_STATUS));
}