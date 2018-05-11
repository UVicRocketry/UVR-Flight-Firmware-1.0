# UVR-Flight-Firmware

STM32 based system for rocket flight data recording and payload UV-sterilization experiment. Used in UVic Rocketry's custom flight hardware at IREC 2017/18.

## Overview
This software is written for the STM32 NUCLEO-L053R8. The NUCLEO interfaces with a custom PCB containing sensor, data-logging, and control hardware. 

![alt text](http://i.imgur.com/UvqYlsa.jpg)

### Sensor hardware
* BMP280 temperature + pressure sensor
* BNO055 low-G accelerometer + orientation sensor
* ADXL377 high-G accelerometer

### Firmware
The hardware control follows a basic state machine model:

[STARTUP]->[DEBUG/IDLE]->[INIT_FLIGHT]->[PAD]->[FLIGHT]->[LANDED]

* [STARTUP] reads the debug jumper pin on power-up, and transitions to [DEBUG] or [IDLE] depending on the reading of the jumper pin [1=DEBUG, 0=IDLE].
* [DEBUG] is a special power-up mode used to test the external hardware and download the contents of the flash memory to PC via the GUI client (see below).  It is not used for flight.
* [IDLE] polls the external hall effect sensor for a magnetic activation signal. Transitions to [INIT_FLIGHT] on detection of the activation signal.
* [INIT_FLIGHT] activates the external sensors and clears the data-log memory. Automatically transitions to [PAD].
* [PAD] polls the sensors for a z-acceleration or altitude change corresponding to launch. Upon detection of launch, activates the UV-sterilization experiment and transitions to [FLIGHT].
* [FLIGHT] continuously writes sensor readings to the flash memory. Upon detection of landing, transitions to [LANDED]
* [LANDED] terminates data-logging and UV-sterilization experiment. 

### Utility program
The custom flight software includes a Windows GUI utility program to (1) view the flight sensor readings in real-time and (2) download the contents of the flash memory to the PC. The flight hardware must be powered up in [DEBUG] mode.

![alt text](http://i.imgur.com/FNl8FKY.jpg)

## Setup

### Firmware

Compile .bin from source. Flash the .bin to the NUCLEO using ST-Link. Power-up with debug jumper removed for flight-ready mode. Power-up with jumper inserted for debugging or data retrieval.

### GUI Utility

Compile .exe from source. Connect NUCLEO via USB and power-up with debug jumper inserted.

## License
GNU LGPL 3.0
