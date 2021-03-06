/*
 *	UVR BMP280 MBED Library
 *
 *	@author	 Andres Martinez
 *	@version 0.9a
 *	@date	 8-May-2017
 *
 *	MBED library for Bosch BMP280 sensor fusion board. Adapted from
 *	BME280 Combined humidity and pressure sensor library written by Toyomasa Watarai:
 *	https://developer.mbed.org/users/MACRUM/code/BME280/
 *
 *	Written for UVic Rocketry
 *
 */

#ifndef BMP280_H
#define BMP280_H

#include "mbed.h"

#define DEFAULT_SLAVE_ADDRESS (0x76 << 1)


class BMP280
{
public:

	/** Create a BMP280 instance
	 *	which is connected to specified I2C pins with specified address
	 */
	BMP280(I2C* _i2c_p, char slave_adr = DEFAULT_SLAVE_ADDRESS);

	virtual ~BMP280();

	// Initializa BMP280 sensor
	void initialize(void);

	// read current pressure in Pascals
	uint32_t getPressure(void);

	// read current temperature (divide returned int by 100 to get degrees C)
	int32_t getTemperature(void);

private:

	I2C*		i2c_p;
	char		address;
	uint16_t	dig_T1;
	int16_t		dig_T2, dig_T3;
	uint16_t	dig_P1;
	int16_t		dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	uint16_t	dig_H1, dig_H3;
	int16_t		dig_H2, dig_H4, dig_H5, dig_H6;
	int32_t		t_fine;

};

#endif