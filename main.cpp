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
 *	- !!!VERIFY CHIP ERASE IS IN FINAL INIT_FLIGHT ROUTINE!!!
 *	- !!!Verify FLIGHT->LANDED state transition!!!
 *	- Set memory storage format to little-endian for competition
 *	- Clean up global declarations. Consider making some local scope. Consider putting others in header files.
 */

#include "mbed.h"
#include "BMP280.hpp"
#include "ADXL377.hpp"
#include "BNO055.hpp"
#include "W25Q128.hpp"
#include "PwmDriver.hpp"	

#include <queue>
#include <array>

// Enable little-endian flash memory storage here with 1, otherwise 0 for big-endian
#define LITTLE_ENDIAN_STORAGE 0
// Enable ground-level testing mode here with 1, otherwise 0 for competition flight mode
#define GROUND_TESTING_MODE 1
// Enable BNO055 fusion sensor with 1, otherwise 0
#define BNO055_ENABLED 0

// amount of time that magnetic field must be detected before IDLE->PAD state transition is triggered
constexpr int HALL_EFFECT_TRIGGER_DURATION_MS{500};

// amount g-force required to trigger PAD->FLIGHT state transition
constexpr float LAUNCH_G_FORCE_THRESHOLD{3.0};
// amount of time that positive g-force must be sustained before FLIGHT state is triggered
constexpr int Z_ACC_TRIGGER_DURATION_MS{20};
// ratio of NUCLEO analog input readings to ADXL377 g-force reading
constexpr float ADXL377_INT_PER_G{65535.0/400.0};
// NUCELO analog input reading change corresponding to required g-force change for FLIGHT state triggering
constexpr int ADXL377_Z_ACC_THRESHOLD{(int)(ADXL377_INT_PER_G * LAUNCH_G_FORCE_THRESHOLD)};

// multiply this value by the referene pressurce to get the pressure at the alititude threashold
// altitude threshold is currently calculated for ***20 meters***
constexpr float LAUNCH_PRESSURE_THRESHOLD_RATIO{0.9976};
// time interval for over which pressure readings are compared to detect LANDED state
constexpr float LANDING_CHECK_TIME_INTERVAL_S{20.0};
// maximum pressure change for which the rocket is considered to be in the LANDED state
constexpr uint32_t LANDING_DETECTION_PRESSURE_CHANGE_THRESHOLD_PA{100};

// serial connection to PC. used to recieve data from and send instructions to board
Serial pc(USBTX, USBRX,115200);

// UV led array
std::array<DigitalOut,3> uv_leds{DigitalOut(PB_3),DigitalOut(PB_4),DigitalOut(PB_10)};

// I2C Bus used to communicate with BMP280 and BNO055
I2C* i2c_p = new I2C(PB_9,PB_8);

// BMP280 environmental sensor (temp + pressure)
BMP280 env(i2c_p);
int32_t temperature;
uint32_t pressure;

// ADXL377 200g x/y/z accelerometer connected to analog input lines
ADXL377 high_g(PA_0,PA_1,PA_4);
ADXL377_readings high_g_acc;

// BNO055 16g accelerometer and orientation sensor
#if BNO055_ENABLED
BNO055 imu(i2c_p,NC);
constexpr int BNO055_MOUNTING_POSITION{MT_P0};
BNO055_QUATERNION quaternion;
BNO055_ACC_short low_g_acc;
#endif

// W25Q128 16Mb Flash memory
W25Q128 flash(PA_7,PA_6,PA_5,PB_6);

// Payload board status LED
DigitalOut status_led(PB_0);

// UV diode voltage-regulator ENABLE signal
DigitalOut voltage_reg_en(PA_10);

// Debug mode jumper (can also be used to trigger FLIGHT->LANDED state transition)
DigitalIn debug_jumper(PA_8);

// Hall-effect sensor input
DigitalIn hall_effect_input(PB_5);

// Buzzer output
PwmDriver buzzer(PC_7);

// NUCLEO USER button 
DigitalIn user_button(PC_13);

// Payload board button (can be used to trigger IDLE->PAD and PAD->FLIGHT state transitions)
DigitalIn payload_button(PA_9);

// Interrupt timer used for data time-stamping
Timer time_stamp;
uint32_t time_elapsed_ms{0};
uint32_t sensor_read_time{0};

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
	if (data_queue.size() >= PAGE_SIZE && flash.get_pages_pushed() < flash.MAX_PAGES - 3)
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
	// signal pc that board is now uploading contents
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
	
	// signal pc that board has finish uploading memory contents
	wait_ms(1000);
	pc.putc('f');
	pc.putc(0);
	pc.putc(',');
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
				#if GROUND_TESTING_MODE
				is_recording_data = true;
				#endif
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
 *	Update the time-elapsed (flight T+) clock based on the internal timer.
 *	MBED Timer will only go to ~(2^31 - 1)/1000 before overflow,
 *	therefore it is necessary to account for this if timer values are expected
 *	to go beyond ~30 minutes
 */ 
constexpr uint32_t MAX_MS_COUNT = 1'800'000;
void update_time_elapsed()
{
	static uint8_t overflow_count{0};
	
	uint32_t curr_time_ms = time_stamp.read_ms();
	
	if (curr_time_ms >= MAX_MS_COUNT)
	{
		++overflow_count;
		time_stamp.reset();
		curr_time_ms -= MAX_MS_COUNT;
	}
	
	time_elapsed_ms = curr_time_ms + MAX_MS_COUNT*overflow_count;
}

// push most recent sensor readings to the data queue for later processing (writing to flash)
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

	#if BNO055_ENABLED
	push_to_queue(low_g_acc.x);
	push_to_queue(low_g_acc.y);
	push_to_queue(low_g_acc.z);
	
	push_to_queue(quaternion.w);
	push_to_queue(quaternion.x);
	push_to_queue(quaternion.y);
	push_to_queue(quaternion.z);
	#else
	push_to_queue(high_g_acc.x);
	push_to_queue(high_g_acc.y);
	push_to_queue(high_g_acc.z);
	#endif
	
	if (record_env_readings)
	{
		push_to_queue(temperature);
		push_to_queue(pressure);
	}
	
	data_queue.push(',');
}

// update readings for all sensors
void update_sensor_readings()
{	
	// update time stamp
	update_time_elapsed();

	#if GROUND_TESTING_MODE
	// get sensor read time
	sensor_read_time = time_stamp.read_us();
	#endif
	
	// update 200g accelerometer reading
	high_g.read(high_g_acc);
		
	#if BNO055_ENABLED	
	// get BNO055 16-g and orientation readings
	imu.get_quaternion(quaternion);
	imu.get_accel_short(low_g_acc);
	#endif
	
	// get temperature and pressure readings
	temperature = env.getTemperature();
	pressure = env.getPressure();
	
	#if GROUND_TESTING_MODE
	// get sensor read time
	sensor_read_time = time_stamp.read_us() - sensor_read_time;
	#endif
}

// transmit sensor data packets to be read as input for the UVR-Sensor-Display utility program
void transmit_test_outputs()
{	
	// transmit time elapsed
	pc.putc('c');
	transmit_var(pc,time_elapsed_ms);
	pc.putc(',');
	
	#if BNO055_ENABLED
	// transmit l6-g accelerometer readings
	pc.putc('a');
	transmit_var(pc,low_g_acc.x);
	transmit_var(pc,low_g_acc.y);
	transmit_var(pc,low_g_acc.z);
	pc.putc(',');
	
	pc.putc('q');
	// transmit orientation readings
	transmit_var(pc,quaternion.w);
	transmit_var(pc,quaternion.x);
	transmit_var(pc,quaternion.y);	
	transmit_var(pc,quaternion.z);
	pc.putc(',');
	#endif
	
	// transmit temperature reading
	pc.putc('t');
	transmit_var(pc,temperature);
	pc.putc(',');
	
	// transmit pressure reading
	pc.putc('p');
	transmit_var(pc,pressure);
	pc.putc(',');
	
	// transmit 200g accelerometer reading
	pc.putc('h');
	transmit_var(pc,high_g_acc.x);
	transmit_var(pc,high_g_acc.y);
	transmit_var(pc,high_g_acc.z);
	pc.putc(',');
	
	// transmit number of pages written to the flash memory
	pc.putc('m');
	transmit_var(pc,flash.get_pages_pushed());
	pc.putc(',');

	// transmit current size of the pre-write data queue
	pc.putc('b');
	transmit_var(pc,(uint32_t)data_queue.size());
	pc.putc(',');
	
	// transmit combined read time for all sensors
	pc.putc('s');
	transmit_var(pc,sensor_read_time);
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
	
	// ISR used to pulse the buzzer
	Ticker buzzer_ticker;
	bool buzzer_on{false};
	auto buzzer_pulse = [&]()
	{
			buzzer_on ? buzzer.turn_off() : buzzer.turn_on();
			buzzer_on = !buzzer_on;
	};

	// set UV leds to off
	for (auto & led : uv_leds)
		led = 0;
	
	debug_jumper.mode(PullDown);
	hall_effect_input.mode(PullUp);
	user_button.mode(PullUp);
	payload_button.mode(PullDown);
	
	env.initialize();
	
	#if BNO055_ENABLED
	imu.set_mounting_position(BNO055_MOUNTING_POSITION);
	#endif
	
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

				// enable serial instruction tokens from PC
				pc.attach(&rx_interrupt,Serial::RxIrq);

				// start time-stamp clock
				time_stamp.reset();
				time_stamp.start();

				while(1)
				{
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
					if (!hall_effect_input.read() || payload_button.read())
					{
						// if hall effect just detected, then start duration timer
						if (!hall_effect_detected)
						{
							hall_effect_detected = true;
							hall_effect_timer.reset();
							hall_effect_timer.start();
						}
						// if hall effect persists from last function call, then check to see if has lasted for set duration
						else if (hall_effect_timer.read_ms() >= HALL_EFFECT_TRIGGER_DURATION_MS || payload_button.read())
							break;
						
					}
					// if hall effect doesn't persist for duration, then clear polling flag
					else
						hall_effect_detected = false;
				}
				hall_effect_timer.stop();
				flight_state = INIT_FLIGHT;
			}
			break;
			case INIT_FLIGHT:
			// initialize board, sensors, and memory for flight. then goto PAD
			{
				buzzer_ticker.attach(buzzer_pulse,0.5);
				flash.erase_chip();
				force_page_write();
				buzzer_ticker.detach();
				buzzer.turn_on();
				wait(5);
				buzzer.turn_off();
				flight_state = PAD;
			}
			break;
			case PAD:
			// wait for pressure (altitude) change or z-axis g-force indicative of launch then goto FLIGHT
			{
				env.getTemperature();
				pressure = env.getPressure();
				float pressure_at_alt_threshold = LAUNCH_PRESSURE_THRESHOLD_RATIO * pressure;

				high_g.read(high_g_acc);
				uint16_t resting_z_acc = high_g_acc.z;
				
				#if GROUND_TESTING_MODE
				pc.putc('x');
				transmit_var(pc,(uint32_t)pressure_at_alt_threshold);
				pc.putc(',');
				pc.putc('z');
				transmit_var(pc,resting_z_acc);
				pc.putc(',');
				#endif 
				
				Timer z_acc_timer;
				bool z_acc_detected{false};
				
				while(1)
				{
					wait_ms(2);
					// check sensors to see if rocket is now in flight
					// check sign of z-axis acceleration - make sure it is correct!!!
					env.getTemperature();
					pressure = env.getPressure();
					
					high_g.read(high_g_acc);
					uint16_t curr_z_acc = high_g_acc.z;
					int z_acc_change = curr_z_acc - resting_z_acc;
					
					if (z_acc_change > ADXL377_Z_ACC_THRESHOLD)
					{
						// if threshold z-acceleration just detected, then start duration timer
						if (!z_acc_detected)
						{
							z_acc_detected = true;
							z_acc_timer.reset();
							z_acc_timer.start();
						}
						// if threshold z-acceleration persists for trigger duration, then start flight
						else if (z_acc_timer.read_ms() >= Z_ACC_TRIGGER_DURATION_MS)
							break;
					}
					// if z-acceleration doesn't persist for duration, then clear polling flag
					else
						z_acc_detected = false;
				
					// if current pressure is less than pressure extrapolated for threshold altitude, then start flight
					if (payload_button.read() // || pressure < pressure_at_alt_threshold
						)										
						break;
				}
				
				z_acc_timer.stop();
				flight_state = FLIGHT;
			}
			break;
			case FLIGHT:
			//	start logging data. goto LANDED if (1) sensors indicate landing or (2) second last page of memory filled
			{
				buzzer.turn_on();
				status_led = 1;
				
				time_stamp.reset();
				time_stamp.start();
				
				// activate UV LEDs and log time
				voltage_reg_en = 1;
				wait_ms(5);
				for (auto & led : uv_leds)
					led = 1;
				
				update_sensor_readings();
				data_queue.push('u');
				push_to_queue(time_elapsed_ms);
				data_queue.push(',');
				
				force_page_write();
				
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
						#if GROUND_TESTING_MODE
						transmit_test_outputs();
						#endif 
						
						// reset sensor read flag
						sensor_read_pending = false;
					}
					
					process_data_queue();
					
					if (check_for_landed || debug_jumper.read())
					{
						check_for_landed = false;
						
						env.getTemperature();
						pressure = env.getPressure();
						
						uint32_t pressure_change;
						if (old_pressure > pressure)
							pressure_change = old_pressure - pressure;
						else
							pressure_change = pressure - old_pressure;
						
						if (debug_jumper.read()
							#if GROUND_TESTING_MODE == 0
							|| pressure_change < LANDING_DETECTION_PRESSURE_CHANGE_THRESHOLD_PA
							#endif
							)
							break;

						old_pressure = pressure;
					}
				}
				flight_state = LANDED;
			}
			break;
			case LANDED:
			// stop logging data. shutoff all UV diodes and log time of shutoff.
			{
				// deactivate UV LEDs
				for (auto & led : uv_leds)
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

				// leave a buffer between end of flight data and final landing data
				force_page_write();
				// take one final data sample and log time of UV shutoff
				update_sensor_readings();
				log_data(true);
				data_queue.push('v');
				push_to_queue(time_elapsed_ms);
				data_queue.push(',');
				// push final page to memory
				force_page_write();
				
				// pulse buzzer until rocket is found and debug jumper is placed
				buzzer_ticker.attach(buzzer_pulse,0.5);
				
				// poll debug jumper until retrieval
				while(1)
				{
					wait(1);
					// place the debug jumper to end the flight program (and stop the buzzer)
					if (debug_jumper.read())
						break;
				}
				status_led = 0;
				buzzer_ticker.detach();
				buzzer.turn_off();
				// wait statement needed to ensure buzzer shutoff
				wait(1);
				
				return 0;
			}
			break;
			default:
			// code should *never* reach this block
			{}
		}
	}
	return 0;
}
