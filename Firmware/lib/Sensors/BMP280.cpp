/*
	BMP280.cpp
	
	Bosch BMP280 pressure sensor library for the Arduino microcontroller.
	This library uses I2C connection.

	Uses floating-point equations from BMP280 datasheet.

	modified by mhafuzul islam

	version 1.01		 16/9/2014 initial version
	
	Our example code uses the "pizza-eating" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me italian pizza someday.
*/

#include "BMP280.h"
#include <Wire.h>
#include <stdio.h>
#include <math.h>


BMP280::BMP280(){}
/*
*	Initialize library and coefficient for measurements
*/
char BMP280::begin(int sdaPin, int sclPin)
{
	Wire.begin(sdaPin,sclPin);
	return (readCalibration());
}

char BMP280::begin(TwoWire *i2c) 
{
	i2c_ = i2c;
	return (readCalibration());
}

/*
The BMP280 includes factory calibration data stored on the device.
Each device has different numbers, these must be retrieved and
used in the calculations when taking measurements.
*/
char BMP280::readCalibration() {
	if (    
		readUInt(0x88, dig_T1) &&
		readInt(0x8A, dig_T2)  &&
		readInt(0x8C, dig_T3)  &&
		readUInt(0x8E, dig_P1) &&
		readInt(0x90, dig_P2)  &&
		readInt(0x92, dig_P3)  &&
		readInt(0x94, dig_P4)  &&
		readInt(0x96, dig_P5)  &&
		readInt(0x98, dig_P6)  &&
		readInt(0x9A, dig_P7)  &&
		readInt(0x9C, dig_P8)  &&
		readInt(0x9E, dig_P9))
	{
		return (1);
	}
	else 
		return (0);
}

/*
**	Read a signed integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value   = external variable to store data (function modifies value)
*/
char BMP280::readInt(char address, float &value)

{
	unsigned char data[2];	//char is 4bit,1byte

	data[0] = address;
	if (readBytes(data,2))
	{
		value = (float)(int16_t)(((unsigned int)data[1]<<8)|(unsigned int)data[0]); //
		return(1);
	}
	value = 0;
	return(0);
}
/* 
**	Read an unsigned integer (two bytes) from device
**	@param : address = register to start reading (plus subsequent register)
**	@param : value 	 = external variable to store data (function modifies value)
*/

char BMP280::readUInt(char address, float &value)
{
	unsigned char data[2];	//4bit
	data[0] = address;
	if (readBytes(data,2))
	{
		value = (float)(unsigned int)(((unsigned int)data[1]<<8)|(unsigned int)data[0]);
		return(1);
	}
	value = 0;
	return(0);
}
/*
** Read an array of bytes from device
** @param : value  = external array to hold data. Put starting register in values[0].
** @param : length = number of bytes to read
*/

char BMP280::readBytes(unsigned char *values, char length)
{
	char x;

	i2c_->beginTransmission(BMP280_ADDR);
	i2c_->write(values[0]);
	error = i2c_->endTransmission();
	if (error == 0)
	{
		i2c_->requestFrom(BMP280_ADDR,length);
		while(i2c_->available() != length) ; // wait until bytes are ready
		for(x=0;x<length;x++)
		{
			values[x] = i2c_->read();
		}
		return(1);
	}
	return(0);
}
/*
** Write an array of bytes to device
** @param : values = external array of data to write. Put starting register in values[0].
** @param : length = number of bytes to write
*/
char BMP280::writeBytes(unsigned char *values, char length)
{
	i2c_->beginTransmission(BMP280_ADDR);
	i2c_->write(values,length);
	error = i2c_->endTransmission();
	if (error == 0)
		return(1);
	else
		return(0);
}

short BMP280::getOversampling(void)
{
	return oversampling;
}

char BMP280::setOversampling(short oss)
{
	oversampling = oss;
	return (1);
}
/*
Coefficients (0-4) --> (0,2,4,8,16)
*/
char BMP280::setFilter(unsigned char coeff)
{
    if (coeff > 4) return 0;   // valid values: 0–4

    unsigned char data[2];

    // First read current CONFIG
    data[0] = BMP280_REG_CONFIG;
    if (!readBytes(data, 1)) return 0;

    unsigned char config = data[0];

    config &= ~(0x07 << 2);
    config |= (coeff << 2);

    // Write back
    data[0] = BMP280_REG_CONFIG;
    data[1] = config;

    return writeBytes(data, 2);
}
/*
**	Begin a measurement cycle.
** Oversampling: 0 to 4, higher numbers are slower, higher-res outputs.
** @returns : delay in ms to wait, or 0 if I2C error.
*/
char BMP280::startMeasurment(void)
{
	unsigned char data[2], result, delay;
	
	data[0] = BMP280_REG_CONTROL;

	switch (oversampling)
	{
		case 0:
			data[1] = BMP280_COMMAND_PRESSURE0;     
			oversampling_t = 1;
			delay = 8;			
		break;
		case 1:
			data[1] = BMP280_COMMAND_PRESSURE1;     
			oversampling_t = 1;
			delay = 10;			
		break;
		case 2:
			data[1] = BMP280_COMMAND_PRESSURE2;		
			oversampling_t = 1;
			delay = 15;
		break;
		case 3:
			data[1] = BMP280_COMMAND_PRESSURE3;
			oversampling_t = 1;
			delay = 24;
		break;
		case 4:
			data[1] = BMP280_COMMAND_PRESSURE4;
			oversampling_t = 1;
			delay = 45;
		break;
		case 16:
			data[1] = BMP280_COMMAND_OVERSAMPLING_MAX;
			oversampling_t = 1;
			delay = 80;	//I cannot find any data about timings in datasheet for x16 pressure and x16 temeprature oversampling
		break;				//I guess this is enough time (maybe it can be even smaller ~60ms)
		default:
			data[1] = BMP280_COMMAND_PRESSURE0;
			delay = 9;
		break;
	}
	result = writeBytes(data, 2);
	if (result) // good write?
		return(delay); // return the delay in ms (rounded up) to wait before retrieving data
	else
		return(0); // or return 0 if there was a problem communicating with the BMP
}

/*
**	Get the uncalibrated pressure and temperature value.
**  @param : uP = stores the uncalibrated pressure value.(20bit)
**  @param : uT = stores the uncalibrated temperature value.(20bit)
*/
char BMP280::getUnPT(float &uP, float &uT)
{
	unsigned char data[6];
	char result;
	
	data[0] = BMP280_REG_RESULT_PRESSURE; //0xF7 

	result = readBytes(data, 6); // 0xF7; xF8, 0xF9, 0xFA, 0xFB, 0xFC
	if (result) // good read
	{
		float factor = pow(2, 4);
		uP = (float)(data[0] *4096 + data[1]*16 + data[2]/16) ;	//20bit UP
		uT = (float)(data[3]*4096 + data[4]*16 + data[5]/16) ;		//20bit UT
	}
	return(result);
}
/*
** Retrieve temperature and pressure.
** @param : T = stores the temperature value in degC.
** @param : P = stores the pressure value in mBar.
*/
char BMP280::getTemperatureAndPressure(float &T,float &P)
{
	float uT ;
	float uP;
	char result = getUnPT(uP,uT);
	if(result!=0){
		// calculate the temperature
		result = calcTemperature(T,uT);
		if(result){
			// calculate the pressure
			result = calcPressure(P,uP);
			if(result)return (1);
			else error = 3 ;	// pressure error ;
			return (9);
		}else 
			error = 2;	// temperature error ;
	}
	else 
		error = 1;
	
	return (9);
}
/*
** temperature calculation
** @param : T  = stores the temperature value after calculation.
** @param : uT = the uncalibrated temperature value.
*/
char BMP280::calcTemperature(float &T, float &adc_T)
//
{
	//Serial.print("adc_T = "); Serial.println(adc_T,DEC);
		
	float var1 = (adc_T/16384.0 - dig_T1/1024.0)*dig_T2;
	float var2 = ((adc_T/131072.0 - dig_T1/8192.0)*(adc_T/131072.0 - dig_T1/8192.0))*dig_T3;
	t_fine = var1+var2;
	T = (var1+var2)/5120.0;
	
	if(T>100 || T <-100)return 0;
	
	return (1);
}
/*
**	Pressure calculation from uncalibrated pressure value.
**  @param : P  = stores the pressure value.
**  @param : uP = uncalibrated pressure value. 
*/
char BMP280::calcPressure(float &P,float uP)
{
	float var1 , var2 ;
	
	var1 = (t_fine/2.0) - 64000.0;
	var2 = var1 * (var1 * dig_P6/32768.0);	//not overflow

	var2 = var2 + (var1 * dig_P5 * 2.0);	//overflow		
	var2 = (var2/4.0)+((dig_P4)*65536.0);
		
	var1 = (dig_P3 * var1 * var1/524288.0 + dig_P2 * var1) / 524288.0;
	var1 = (1.0 + var1/32768.0) * dig_P1;
		
	P = 1048576.0- uP;
		
	P = (P-(var2/4096.0))*6250.0/var1 ;		//overflow
		
	var1 = dig_P9*P*P/2147483648.0;			//overflow

	var2 = P*dig_P8/32768.0;
	P = P + (var1+var2+dig_P7)/16.0;
		
	P = P/100.0 ;
	
	if(P>1200.0 || P < 800.0)return (0);
	return (1);
}




float BMP280::sealevel(float P, float A)
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
	return(P/pow(1-(A/44330.0),5.255));
}


float BMP280::altitude(float P, float P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
	return(44330.0*(1-pow(P/P0,1/5.255)));
}


char BMP280::getError(void)
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error
{
	return(error);
}

