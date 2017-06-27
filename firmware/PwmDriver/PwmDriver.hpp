/*
 *	PwmDriver library
 *
 *	@author	 Andres Martinez
 *	@version 0.9b
 *	@date	 14-Mar-2017
 *
 *	Subclass of MBED PwmOut. Used to safely manage PwmOut API calls with
 *	duty cycle presets and defined on/off states. PwmOut.write(float) is only
 *	invoked for non-zero values when driver is set to the "on" state. Signals
 *	are deactivated in the destructor
 *
 *	Using PwmOut and PwmDriver for the same pin(s) is NOT supported - doing so will
 *	negate the state checking features of PwmDriver. Use one or the other!
 *
 *	To-do:
 *		- Track pins already controlled by a PwmDriver object and
 *		  throw an error() if users try to attach multiple drivers
 *		  to a single pin.
 */

#include "mbed.h"

#ifndef PWMDRIVER_H
#define PWMDRIVER_H

class PwmDriver : private PwmOut
{
	public:

	/*
	*	Do not invoke this constructor more than once for a given pin! Doing
	*	so will attach multiple drivers to a pin
	*/
	PwmDriver(PinName _pin, int _period_ms = 1)
	: PwmOut(_pin),duty(0.0),on(false),pin(_pin)
	{
		period_ms(_period_ms);
		write(0.0);
	}

	void turn_on()
	{
		if (!on) // if LED already on, avoids un-necessary write
			write(duty);
		on = true;
	}

	void turn_off()
	{
		on = false;
		write(0.0);
	}

	/*
	*	Duty percentage values must be specified from 0.0 to 1.0.
	*	Duty values less than 0.0 or greater than 1.0 will be clamped
	*	to 0.0 and 1.0, respectively
	*/
	void set_duty(float _duty)
	{
		if (_duty < 0.0)
			duty = 0.0;
		else if (_duty > 1.0)
			duty = 1.0;
		else
			duty = _duty;

		if (on) //update duty if pwm already running
			write(duty);
	}

	float get_duty() const
	{
		return duty;
	}

	// find the actual duty cycle value stored in the board
	float get_api_duty()
	{
		return read();
	}

	bool is_on() const
	{
		return on;
	}

	PinName get_pin() const
	{
		return pin;
	}

	~PwmDriver()
	{
		// turn off pwm signal before destroying the driver
		// don't rely on this if the PwmDriver has global scope
		turn_off();
	}

	PwmDriver(const PwmDriver& other) = delete; // pwm copy not supported
	PwmDriver& operator=(const PwmDriver& other) = delete;
	PwmDriver(PwmDriver&& other)
	: PwmOut(other),duty(other.duty),on(other.on),pin(other.pin)
	{}

	private:

	float duty;
	bool on;
	PinName pin;
	PwmDriver();
};

#endif
