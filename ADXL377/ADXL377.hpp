#include "mbed.h"

#ifndef ADXL377_H
#define ADXL377_H

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} ADXL377_readings;

class ADXL377
{
public:

	ADXL377(PinName x_pin, PinName y_pin, PinName z_pin)
	: x_in(AnalogIn(x_pin)),y_in(AnalogIn(y_pin)),z_in(AnalogIn(z_pin))
	{}

	void read(ADXL377_readings &acc)
	{
		acc.x = x_in.read_u16();
		acc.y = y_in.read_u16();
		acc.z = z_in.read_u16();
	}

private:
	ADXL377();
	AnalogIn x_in,y_in,z_in;
};

#endif