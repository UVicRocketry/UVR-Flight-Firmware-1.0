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
 *	- Implement BNO055 calibration routine if necessary
 *	- Set memory storage format to little-endian for competition
 *	- Clean up global declarations. Consider making some local scope. Consider putting others in header files.
 *	- Verify comparison expression of z-axis acceleration threshold
 *	- Verify landing detection delta-pressure threshold
 */

#include "mbed.h"
#include "BMP280.hpp"
#include "ADXL377.hpp"
#include "BNO055.hpp"
#include "W25Q128.hpp"
#include "PwmDriver.hpp"	

#include <queue>
#include <array>

// Enable little-endian flash memory storage here with 1, otherwise enter 0 for big-endian
#define LITTLE_ENDIAN_STORAGE 0

// Enable ground-level testing mode here with 1, otherwise 0 for competition flight mode
#define GROUND_TESTING_MODE 1

constexpr int HALL_EFFECT_TRIGGER_DURATION_MS{500};
constexpr float LAUNCH_Z_ACC_THRESHOLD_MPSQ{9.81 * 3};
constexpr int LAUNCH_Z_ACC_DURATION_MS{2000};

// multiply this value by the referene pressurce to get the pressure at the alititude threashold
// altitude threshold is currently calculated for ***20 meters***
constexpr float LAUNCH_PRESSURE_THRESHOLD_RATIO{0.9976};
// time interval for over which pressure readings are compared to detect LANDED state
constexpr float LANDING_CHECK_TIME_INTERVAL_S{20.0};
// maximum pressure change for which the rocket is considered to be in the LANDED state
constexpr uint32_t LANDING_DETECTION_PRESSURE_CHANGE_THRESHOLD_PA{10};

// serial connection to PC. used to recieve data from and send instructions to board
Serial pc(USBTX, USBRX,115200);

//specify leds and their pin designations here
std::array<DigitalOut,3> leds{DigitalOut(PB_3),DigitalOut(PB_4),DigitalOut(PB_10)};

// I2C Bus used to communicate with BMP280 and BNO055
I2C* i2c_p = new I2C(PB_9,PB_8);

// Environmental sensor (temp + pressure)
BMP280 env(i2c_p,0x77 << 1);
int32_t temperature;
uint32_t pressure;

// 200g x/y/z accelerometer connected to analog input lines
ADXL377 high_g(PA_0,PA_1,PA_4);
ADXL377_readings high_g_acc;

// 16g accelerometer and orientation sensor
BNO055 imu(i2c_p,NC);
constexpr int BNO055_MOUNTING_POSITION{MT_P0};
BNO055_QUATERNION quaternion;
BNO055_ACC_short low_g_acc;

// 16Mb Flash memory
W25Q128 flash(PA_7,PA_6,PA_5,PB_6);

// Status LED
DigitalOut status_led(PB_0);

// Voltage-Regulator ENABLE
DigitalOut voltage_reg_en(PA_10);

// Debug mode jumper
DigitalIn debug_jumper(PA_8);

// Hall-effect sensor input
DigitalIn hall_effect_input(PB_5);

// Buzzer output
PwmDriver buzzer(PC_7);

// NUCLEO USER button
DigitalIn user_button(PC_13);

// Interrupt timer used for data time-stamping
Timer time_stamp;
uint32_t time_elapsed_ms{0};
constexpr uint32_t TIMER_MS_MAX_VALUE = 2147483647 / 1000;

// Used to store sensor data values before storing them in flash memory
queue<char> data_queue;

// is the board currently sending contents of W25Q128 flash memory to PC?
bool is_sending_data_to_pc{false};
// is the board committing sensor data to W25Q128 flash memory?
bool is_recording_data{false};
// is the board currently erasing the W25Q128 flash memory?
bool is_erasing_memory{false};

// transmit a variable over a serial connection as a series of bytes
template <typename T> void transmit_var(Serial &pc, const T& d)
{
	char byte_size = sizeof(d);
	for (int i = 0; i < byte_size; i++)
		pc.putc((d & ( 0xFF << i*8)) >> i*8); 
}

// push a variable to the data queue as a series of bytes (endianess set by LITTLE_ENDIAN_STORAGE 0 or 1)
template <typename T> void push_to_queue(const T& d)
{
	int byte_size = sizeof(d);
	#if LITTLE_ENDIAN_STORAGE
	for (int i = 0; i < byte_size; i++)
	#else
	for (int i = byte_size - 1; i >= 0; i--)
	#endif
		data_queue.push((d & ( 0xFF << i*8)) >> i*8); 
}

// write next available page of queued data to the flash memory
bool process_data_queue()
{
	if (data_queue.size() >= PAGE_SIZE && flash.get_pages_pushed() < flash.MAX_PAGES - 2)
	{
		page new_page;
		for (auto & c : new_page)
		{
			c = data_queue.front();
			data_queue.pop();
		}
		flash.push_page_back(new_page);
		return true;
	}
	return false;
}

// write current contents of data queue to memory, up to a maximum of 256 bytes
// if data queue is smaller than 256 bytes, then write zeros for the corresponding page memory cells
bool force_page_write()
{
	if (flash.get_pages_pushed() < flash.MAX_PAGES)
	{
		page new_page;
		for (auto & c : new_page)
		{
			if (data_queue.empty())
				c = 0;
			else
			{
				c = data_queue.front();
				data_queue.pop();
			}
		}
		flash.push_page_back(new_page);
		return true;
	}
	return false;
}

// send raw contents of W25Q128 flash memory to PC over serial
void transmit_recorded_data(unsigned int num_pages = W25Q128::MAX_PAGES)
{
	// signal pc that board is now uploading data
	pc.putc('d');
	pc.putc(0);
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
			// completely erase the flash memory
			case 'e':
				is_erasing_memory = true;
				is_sending_data_to_pc = false;
				is_recording_data = false;
			break;
			default:

			break;
		}
	}
}

// helper function to manage MBED timer overflow
// assumes update interval is < 30 minutes
void update_time_elapsed()
{
	/*
	static uint32_t old_time_ms{0};
	uint32_t current_time_ms = time_stamp.read_ms();
	
	if (current_time_ms < old_time_ms)
	{
		time_elapsed_ms += (TIMER_MS_MAX_VALUE - old_time_ms);
		old_time_ms = 0;
	}	

	time_elapsed_ms += (current_time_ms - old_time_ms);
	old_time_ms = current_time_ms;
	*/
	time_elapsed_ms = time_stamp.read_ms();
}

void log_data(bool force_env_log = false)
{
	bool record_env_readings{false};
	char data_header = 'k';	
	
	// log environmental sensors at slower rate
	static int count{0};
	if (count >= 100 || force_env_log)
	{
		count = 0;
		record_env_readings = true;
		data_header = 'e';
	}	
	else
		count++;

	data_queue.push(data_header);
	push_to_queue(time_elapsed_ms);

	push_to_queue(low_g_acc.x);
	push_to_queue(low_g_acc.y);
	push_to_queue(low_g_acc.z);
	
	push_to_queue(quaternion.w);
	push_to_queue(quaternion.x);
	push_to_queue(quaternion.y);
	push_to_queue(quaternion.z);
	
	if (record_env_readings)
	{
		push_to_queue(temperature);
		push_to_queue(pressure);
	}
	
	data_queue.push(',');
}

void update_sensor_readings()
{	
	update_time_elapsed();
		
	imu.get_quaternion(quaternion);
	imu.get_accel_short(low_g_acc);
	
	high_g.read(high_g_acc);
	
	temperature = env.getTemperature();
	pressure = env.getPressure();
}

// transmit sensor data packets to be read as input for the UVR-Sensor-Display utility program
void transmit_test_outputs()
{	
	pc.putc('e');
	transmit_var(pc,time_elapsed_ms);
	transmit_var(pc,low_g_acc.x);
	transmit_var(pc,low_g_acc.y);
	transmit_var(pc,low_g_acc.z);
	transmit_var(pc,quaternion.w);
	transmit_var(pc,quaternion.x);
	transmit_var(pc,quaternion.y);	
	transmit_var(pc,quaternion.z);
	transmit_var(pc,temperature);
	transmit_var(pc,pressure);
	pc.putc(',');
	
	pc.putc('h');
	transmit_var(pc,high_g_acc.x);
	transmit_var(pc,high_g_acc.y);
	transmit_var(pc,high_g_acc.z);
	pc.putc(',');
	
	pc.putc('m');
	transmit_var(pc,flash.get_pages_pushed());
	pc.putc(',');
}

int main()
{
	// 100Hz interrupt timer
	Ticker ticker_100_Hz;
	// has ticker signalled a pending sensor sampling?
	bool sensor_read_pending{false};	
	// enable 100Hz sensor sampling interrupt
	ticker_100_Hz.attach([&sensor_read_pending](){sensor_read_pending = true;},0.01);
	
	// rocket flight states
	enum state {STARTUP,DEBUG,IDLE,INIT_FLIGHT,PAD,FLIGHT,LANDED};
	state flight_state{STARTUP};

	// set buzzer duty cycle
	buzzer.set_duty(0.5);
	// set UV leds to off
	for (auto & led : leds)
		led = 0;
	
	debug_jumper.mode(PullDown);
	hall_effect_input.mode(PullUp);
	user_button.mode(PullUp);

	env.initialize();
	imu.set_mounting_position(BNO055_MOUNTING_POSITION);

	while(1)
	{
		switch (flight_state)
		{
			case STARTUP:
			// board init procedure. will start here after board reset. goto DEBUG is debug jumper set, else goto IDLE
			{
				// if debug jumper is set, then immediately transition to the board debug mode
				if (debug_jumper.read())
					flight_state = DEBUG;
				// else start the flight sequence
				else
					flight_state = IDLE;
			}
			break;
			case DEBUG:
			// output sensor test values over serial and wait for PC instructions. no state transitions here
			{
				// debug mode indicator light
				status_led = 1;

				// remove this erase_sector() for actual firmware!
				flash.erase_sector(0);

				// enable serial instruction tokens from PC
				pc.attach(&rx_interrupt,Serial::RxIrq);

				// start time-stamp clock
				time_stamp.reset();
				time_stamp.start();

				while(1)
				{
					// if erase chip flag is set, then erase the chip
					if (is_erasing_memory)
					{
						flash.erase_chip();
						is_erasing_memory = false;
					}
					
					// if sensor read flag is set by ticker ISR, then read the sensors and send updated values to pc
					if (sensor_read_pending)
					{
						// sample all sensors based on current sampling rate
						update_sensor_readings();
						
						if (is_recording_data)
							log_data();
						
						// send debugging data to PC over serial
						transmit_test_outputs();
						
						// reset sensor read flag
						sensor_read_pending = false;						
					}
					
					// if board is currently set to upload memory contents to PC, then do so
					else if (is_sending_data_to_pc)
					{
						transmit_recorded_data();
						is_sending_data_to_pc = false;
					}

					process_data_queue();
				}
			}
			break;
			case IDLE:
			// start of non-debug sequence. wait for Hall-effect sensor to trip for set duration, then goto INIT_FLIGHT
			{
				// polling flag used to check if hall effect sensor has been tripped
				bool hall_effect_detected{false};
				// duration timer to see if hall effect sensor is active for sufficient duration
				Timer hall_effect_timer;

				while(1)
				{
					wait_ms(1);
					if (!hall_effect_input.read() || !user_button.read())
					{
						// if hall effect just detected, then start duration timer
						if (!hall_effect_detected)
						{
							hall_effect_detected = true;
							hall_effect_timer.reset();
							hall_effect_timer.start();
						}
						// if hall effect persists from last function call, then check to see if has lasted for set duration
						else if (hall_effect_timer.read_ms() >= HALL_EFFECT_TRIGGER_DURATION_MS || !user_button.read())
						{
							hall_effect_timer.stop();
							flight_state = INIT_FLIGHT;
							break;
						}
					}
					// if hall effect doesn't persist for duration, then clear polling flag
					else
						hall_effect_detected = false;
				}
			}
			break;
			case INIT_FLIGHT:
			// initialize board, sensors, and memory for flight. then goto PAD
			{
				//erase the memory chip before collecting new data
				//flash.erase_chip();
				buzzer.turn_on();
				wait(2);
				buzzer.turn_off();
				flight_state = PAD;
			}
			break;
			case PAD:
			// wait for pressure (altitude) change or z-axis g-force indicative of launch then goto FLIGHT
			{
				status_led = 1;
				env.getTemperature();
				pressure = env.getPressure();
				float pressure_at_alt_threshold = LAUNCH_PRESSURE_THRESHOLD_RATIO * pressure;
				
				while(1)
				{
					wait_ms(10);
					// check sensors to see if rocket is now in flight
					// check sign of z-axis acceleration - make sure it is correct!!!
					env.getTemperature();
					pressure = env.getPressure();
					imu.get_accel_short(low_g_acc);
					high_g.read(high_g_acc);

					//if (pressure < pressure_at_alt_threshold || low_g_acc.z > LAUNCH_Z_ACC_THRESHOLD_MPSQ || !user_button.read())
					if (pressure < pressure_at_alt_threshold || !user_button.read())
					{					
						flight_state = FLIGHT;	
						break;
					}
				}
			}
			break;
			case FLIGHT:
			//	start logging data. goto LANDED if (1) sensors indicate landing or (2) second last page of memory filled
			{
				buzzer.turn_on();
				time_elapsed_ms = 0;
				time_stamp.reset();
				time_stamp.start();
				
				// activate UV LEDs and log time
				voltage_reg_en = 1;
				wait_ms(5);
				for (auto & led : leds)
					led = 1;
				
				update_sensor_readings();
				data_queue.push('u');
				push_to_queue(time_elapsed_ms);
				data_queue.push(',');
				
				bool check_for_landed{false};
				uint32_t old_pressure = pressure;
				
				Ticker pressure_check_ticker;
				pressure_check_ticker.attach([&]()
				{
					check_for_landed = true;
				},20.0
				);
				
				while(1)
				{
					// if sensor read flag is set by ticker ISR, then read the sensors and send updated values to pc
					if (sensor_read_pending)
					{
						// sample all sensors based on current sampling rate
						update_sensor_readings();
						log_data();
						
						// send debugging data to PC over serial
						#ifdef GROUND_TESTING_MODE
						transmit_test_outputs();
						#endif 
						
						// reset sensor read flag
						sensor_read_pending = false;
					}
					
					process_data_queue();
					
					if (check_for_landed)
					{
						check_for_landed = false;
						
						env.getTemperature();
						pressure = env.getPressure();
						
						uint32_t pressure_change;
						if (old_pressure > pressure)
							pressure_change = old_pressure - pressure;
						else
							pressure_change = pressure - old_pressure;
						if (pressure_change < LANDING_DETECTION_PRESSURE_CHANGE_THRESHOLD_PA)
						{
							flight_state = LANDED;
							break;
						}
						
						old_pressure = pressure;
					}
				}
			}
			break;
			case LANDED:
			// stop logging data. shutoff all UV diodes and log time of shutoff.
			{
				// deactivate UV LEDs
				for (auto & led : leds)
					led = 0;
				voltage_reg_en = 0;
				
				// turn off automatic data sampling
				ticker_100_Hz.detach();
				
				// get any remaining data stored in queue up to remaining page capacity minus two
				while(process_data_queue()){}
				// push partial page to memory
				force_page_write();
				// empty queue of any data that couldn't be stored
				while(!data_queue.empty())
					data_queue.pop();
				
				// take one final data sample and log time of UV shutoff
				update_sensor_readings();
				log_data(true);
				data_queue.push('v');
				push_to_queue(time_elapsed_ms);
				data_queue.push(',');	
				// push final page to memory
				force_page_write();
				
				// wait until retrieval
				while(1)
					wait(10);
			}
			break;
			default:
			// code should *never* reach this block
			{}
		}
	}
	return 0;
}
