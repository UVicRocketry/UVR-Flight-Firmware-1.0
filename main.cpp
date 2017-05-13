/*
 *	UVR Payload Firmware
 *
 *	@author	 Andres Martinez
 *	@version 0.9a
 *	@date	 8-May-2017
 *
 *	Flight firmware for payload sensors and UV experiment
 *
 *	Written for UVic Rocketry
 *
 *	To-do:
 *	- Finish implementing data queuing
 *	- Implement flight state machine
 *  - Fix BNO055 orientation setting - roll and pitch are currently swapped
 * 	- Implement BNO055 calibration routine if necessary
 *  - Finalize memory storage format - big or little endian???
 *  - Clean up global declarations. Consider making some local scope. Consider putting others in header files.
 */

#include "mbed.h"
#include "transmit_serial.hpp"
#include "BMP280.hpp"
#include "ADXL377.hpp"
#include "BNO055.hpp"
#include "W25Q128.hpp"
#include "PwmDriver.hpp"

#include <queue>

#define TICKER_DELAY 0.01
#define HALL_EFFECT_DURATION_MS 2000

// serial connection to PC. used to recieve data from and send instructions to board
Serial pc(USBTX, USBRX,115200);

// I2C Bus used to communicate with BMP280 and BNO055
auto i2c_p = new I2C(I2C_SDA,I2C_SCL);

// Environmental sensor (temp + pressure)
BMP280 env(i2c_p);
int temperature, pressure;

// 200g x/y/z accelerometer connected to analog input lines 
ADXL377 high_g(A0,A1,A2);
ADXL377_readings high_g_acc;

// 16g accelerometer and orientation sensor
BNO055 imu(i2c_p,NC);
BNO055_QUATERNION_TypeDef quaternion;
BNO055_ACC_short_TypeDef low_g_acc;

// 16Mb Flash memory
W25Q128 flash(D11,D12,D13,D10);

// Debug LED
DigitalOut led0(D2);

// Debug mode jumper
DigitalIn debug_jumper(D7);

// Hall-effect sensor input
DigitalIn hall_effect_input(D4);

// Buzzer output
PwmDriver buzzer(D9);

// Interrupt timer used for data time-stamping
Timer timer0;
int time_elapsed_ms;

// 100Hz interrupt timer
Ticker ticker0;

// Used to store sensor data values before storing them in flash memory
queue<char> data_queue;

// is the board currently sending contents of W25Q128 flash memory to PC?
bool is_sending_data_to_pc = false;
// is the board committing sensor data to W25Q128 flash memory?
bool is_recording_data = false;
// has ticker signalled a pending sensor sampling?
bool sensor_read_pending = false;

// send raw contents of W25Q128 flash memory to PC over serial
void transmit_recorded_data(unsigned int num_pages = W25Q128::NUM_PAGES)
{
	led0 = 0;
	// signal pc that board is now uploading data
	pc.putc('d');
	transmit_int16(pc,0);
	pc.putc(',');
	wait_ms(1000);

	page p;

	for (unsigned int i = 0; i < num_pages; i++)
	{
		if (!is_sending_data_to_pc)
			break;
		flash.read_page(i,p);
		for (auto & c : p)
		{
			pc.putc(c);
		}
	}
	led0 = 1;
}

// ISR to handle instruction tokens sent from PC to NUCLEO
void rx_interrupt()
{
	while (pc.readable())
	{
		char recieved_byte = pc.getc();
		switch (recieved_byte)
		{
			// start uploading flash memory contents to PC
			case 'd':
				is_sending_data_to_pc = true;
			break;
			// abort uploading flash memory contents to PC
			case 'a':
				is_sending_data_to_pc = false;
			break;
			// start recording data to flash memory
			case 'r':
				is_recording_data = true;
			break;
			// stop recording data to flash memory
			case 's':
				is_recording_data = false;
			break;
			default:
			
			break;
		}
	}
}
/*
void push_int16_to_queue(int16_t d)
{
	// little endian
	int index = 0;
	char outbox[2] = {0,0};

	outbox[index] = d & 0xFF;
	data_queue.push(outbox[index]);	
	index++;

	outbox[index] = (d & 0xFF00) >> 8;
	data_queue.push(outbox[index]);
}

void push_int32_to_queue(int32_t d)
{
	// little endian
	int index = 0;
	char outbox[4];

	outbox[index] = d & 0xFF;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = (d & 0xFF00) >> 8;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = (d & 0xFF0000) >> 16;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = (d & 0xFF000000) >> 24;
	data_queue.push(outbox[index]);
}
*/
void push_int16_to_queue(int16_t d)
{
	// big endian
	int index = 0;
	char outbox[2] = {0,0};

	outbox[index] = (d & 0xFF00) >> 8;
	data_queue.push(outbox[index]);	
	index++;

	outbox[index] = d & 0x00FF;
	data_queue.push(outbox[index]);
}

void push_int32_to_queue(int32_t d)
{
	// big endian
	int index = 0;
	char outbox[4];

	outbox[index] = (d & 0xFF000000) >> 24;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = (d & 0xFF0000) >> 16;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = (d & 0xFF00) >> 8;
	data_queue.push(outbox[index]);
	index++;

	outbox[index] = d & 0xFF;
	data_queue.push(outbox[index]);
}

void data_tick()
{
	sensor_read_pending = true;
}

void read_sensors()
{
	// log environmental sensors at slower rate
	static int count = 0;
	if (count >= 100)
	{
		count = 0;
		time_elapsed_ms = timer0.read_ms();
		temperature = env.getTemperature();
		pressure = env.getPressure();
		if (is_recording_data)
		{
			data_queue.push('e');
			push_int32_to_queue(time_elapsed_ms);
			push_int32_to_queue(temperature);
			push_int32_to_queue(pressure);
			data_queue.push(',');
		}
	}
	
	high_g.read(high_g_acc);
	time_elapsed_ms = timer0.read_ms();
	imu.get_quaternion(quaternion);
	imu.get_accel_short(low_g_acc);

	if (is_recording_data)
	{
		data_queue.push('k');
		push_int32_to_queue(time_elapsed_ms);
		push_int16_to_queue(low_g_acc.x);
		push_int16_to_queue(low_g_acc.y);
		push_int16_to_queue(low_g_acc.z);   
		push_int16_to_queue(quaternion.w);
		push_int16_to_queue(quaternion.x);
		push_int16_to_queue(quaternion.y);
		push_int16_to_queue(quaternion.z);   
		data_queue.push(',');
	}
	
	count++;
	sensor_read_pending = false;
}

void process_data_queue()
{
	if (data_queue.size() >= PAGE_SIZE)
	{
		page new_page;
		for (auto & c : new_page)
		{
			c = data_queue.front();
			data_queue.pop();
		}
		flash.push_page(new_page);	
	}
}

void transmit_test_outputs()
{
	pc.putc('e');
	transmit_int32(pc,time_elapsed_ms);
	transmit_int32(pc,temperature);
	transmit_int32(pc,pressure);
	pc.putc(',');

	pc.putc('k');
	transmit_int32(pc,time_elapsed_ms);
	transmit_int16(pc,low_g_acc.x);
	transmit_int16(pc,low_g_acc.y);
	transmit_int16(pc,low_g_acc.z);   
	transmit_int16(pc,quaternion.w);
	transmit_int16(pc,quaternion.x);
	transmit_int16(pc,quaternion.y);
	transmit_int16(pc,quaternion.z);   
	pc.putc(',');

	pc.putc('h');    
	transmit_int16(pc,high_g_acc.x);
	transmit_int16(pc,high_g_acc.y);
	transmit_int16(pc,high_g_acc.z);
	pc.putc(',');
	
	pc.putc('m');
	transmit_int32(pc,flash.get_bytes_pushed());
	pc.putc(',');	
}

int main() 
{
	// rocket flight states
	enum state {STARTUP,DEBUG,IDLE,INIT_FLIGHT,PAD,FLIGHT,LANDED};
	state flight_state = STARTUP;

	buzzer.set_duty(0.5);
	
	debug_jumper.mode(PullDown);
	hall_effect_input.mode(PullDown);
	
	// enable 100Hz sensor sampling interrupt
	ticker0.attach(&data_tick,TICKER_DELAY);

	// polling flag used to check if hall effect sensor has been tripped
	bool hall_effect_detected = false;
	// duration timer to see if hall effect sensor is active for sufficient duration
	Timer hall_effect_timer;
	
	while(1)
	{
		wait_us(1);
		
		switch (flight_state)
		{ 
			case STARTUP:
			// board init procedure. will start here after board reset. goto DEBUG is debug jumper set, else goto IDLE
			{
				
				// if debug jumper is set, then immediately transition to the board debug mode
				if (debug_jumper.read())
				{
					flight_state = DEBUG;
					
					// debug mode indicator light
					led0 = 1;
					
					// remove this erase_sector() for actual firmware!
					flash.erase_sector(0);
					
					// enable serial instruction tokens from PC
					pc.attach(&rx_interrupt,Serial::RxIrq);		

					// start time-stamp clock
					timer0.reset();
					timer0.start();					
				}
				else
				{
					flight_state = IDLE;
				}
			}
			break;
			case DEBUG:
			// output sensor test values over serial and wait for PC instructions. no state transitions here
			{
				// if sensor read flag is set by ticker ISR, then read the sensors
				if (sensor_read_pending)
					read_sensors();

				// if board is currently set to upload memory contents to PC, then do so
				if (is_sending_data_to_pc)
				{
					transmit_recorded_data();
					is_sending_data_to_pc = false;
				}
				// else keep transmitting sensor test values
				else
				{
					transmit_test_outputs();
				}
				
				// if board is currently set to record data, then do so
				if (is_recording_data)
					process_data_queue();
			}
			break;
			case IDLE:
			// start of non-debug sequence. wait for Hall-effect sensor to trip for set duration, then goto INIT_FLIGHT
			{
				if (hall_effect_input.read())
				{
					// if hall effect just detected, then start duration timer
					if (!hall_effect_detected)
					{
						hall_effect_detected = true;
						hall_effect_timer.reset();
						hall_effect_timer.start();
					}
					// if hall effect persists from last function call, then check to see if has lasted for set duration
					else
						// if hall effect detected for set duration or longer, then transition to INIT_FLIGHT
						if (hall_effect_timer.read_ms() >= HALL_EFFECT_DURATION_MS)
						{
							hall_effect_timer.stop();
							buzzer.turn_on();
							flight_state = INIT_FLIGHT;
						}
				}
				// if hall effect doesn't persist for duration, then clear polling flag
				else
					hall_effect_detected = false;
			}
			break;
			case INIT_FLIGHT:
			// initialize board, sensors, and memory for flight. then goto PAD
			{
				//state code goes here
			}
			break;
			case PAD:
			// wait for pressure (altitude) change or z-axis g-force indicative of launch then goto FLIGHT
			{
				//state code goes here				
			}
			break;
			case FLIGHT:
			//  start logging data. goto LANDED if (1) sensors indicate landing or (2) second last page of memory filled
			{
				//state code goes here				
			}
			break;
			case LANDED:
			// stop logging data. shutoff all UV diodes and log time of shutoff.
			{
				//state code goes here				
			}
			break;
			default:
			{}	
			// code should *never* reach this block
		}
	}
	return 0;
}
