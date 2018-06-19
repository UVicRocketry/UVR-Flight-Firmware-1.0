# UVR-Flight-Firmware

STM32 based system for rocket flight data recording and payload UV-sterilization control. Used in UVic Rocketry's custom flight hardware.

## Overview
This software is written for the STM32 NUCLEO-L053R8. The NUCLEO interfaces with a custom PCB containing sensor, data-logging, and control hardware. 

![alt text](http://i.imgur.com/UvqYlsa.jpg)

### Peripheral hardware
* BMP280 temperature + pressure sensor
* BNO055 low-G accelerometer + orientation fusion sensor
* ADXL377 high-G accelerometer
* W25Q128FV flash memory module

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

### Utility software

The custom flight software includes Windows utility programs to debug the sensors and process the post-flight data.

UVR-Firmware-Utility.exe is used to (1) view the flight sensor readings in real-time and (2) download the contents of the flash memory to the PC. The flight hardware must be powered up in [DEBUG] mode. Memory-Parser.exe is used to convert the downloaded flash memory data to CSV.

![alt text](http://i.imgur.com/FNl8FKY.jpg)

## Setup

### Firmware

Compile .bin from source (GNU ARM - firmware/Makefile). Flash the .bin to the NUCLEO using st-flash (see firmware/flash.sh) or ST-LINK. Power-up with debug jumper removed for flight-ready mode. Power-up with jumper inserted for debugging or data retrieval using UVR-Firmware-Utility.exe.

### Utility software

Compile UVR-Firmware-Utility.exe and Memory-Parser.exe from source (Visual Studio - utility/UVR-Firmware-Utility.sln). Power-up the NUCLEO with the debug jumper inserted and connect the NUCLEO to PC via USB. Start UVR-Firmware-Utility.exe, select the USB COM port from the drop-down menu, and click Connect. Click Start Download to begin downloading the contents of the flash memory module to PC. Use Memory-Parser.exe to convert the generated .mem file to CSV.

## License
GNU LGPL 3.0
