#include "mbed.h"
#include "BMP280.hpp"


BMP280::BMP280(I2C* _i2c_p, char slave_adr)
    :
    i2c_p(_i2c_p), 
    address(slave_adr),
    t_fine(0)
{
}

BMP280::~BMP280()
{
}
    
void BMP280::initialize()
{
    char cmd[18];
 
    cmd[0] = 0xf4; // ctrl_meas
    cmd[1] = 0x47; // Temparature oversampling x1, Pressure oversampling x4, Normal mode
    i2c_p->write(address, cmd, 2);
 
    cmd[0] = 0xf5; // config
    cmd[1] = 0x00; // Standby 0.5ms, Filter off
    i2c_p->write(address, cmd, 2);
 
    cmd[0] = 0x88; // read dig_T regs
    i2c_p->write(address, cmd, 1);
    i2c_p->read(address, cmd, 6);
 
    dig_T1 = (cmd[1] << 8) | cmd[0];
    dig_T2 = (cmd[3] << 8) | cmd[2];
    dig_T3 = (cmd[5] << 8) | cmd[4];
 
    cmd[0] = 0x8E; // read dig_P regs
    i2c_p->write(address, cmd, 1);
    i2c_p->read(address, cmd, 18);
 
    dig_P1 = (cmd[ 1] << 8) | cmd[ 0];
    dig_P2 = (cmd[ 3] << 8) | cmd[ 2];
    dig_P3 = (cmd[ 5] << 8) | cmd[ 4];
    dig_P4 = (cmd[ 7] << 8) | cmd[ 6];
    dig_P5 = (cmd[ 9] << 8) | cmd[ 8];
    dig_P6 = (cmd[11] << 8) | cmd[10];
    dig_P7 = (cmd[13] << 8) | cmd[12];
    dig_P8 = (cmd[15] << 8) | cmd[14];
    dig_P9 = (cmd[17] << 8) | cmd[16];
 
}

int32_t BMP280::getTemperature(void)
{
    int32_t var1, var2, T;
    int32_t adc_T;
    char cmd[4];
    cmd[0] = 0xfa; // temp_msb
    i2c_p->write(address, cmd, 1);
    i2c_p->read(address, &cmd[1], 3);

    //jons slow painful shifting of bits to make it readable
    adc_T = cmd[1];
    adc_T <<= 8;
    adc_T |= cmd[2];
    adc_T <<= 8;
    adc_T |= cmd[3];
     
    //from here more or less same as ada and bosch
 
    adc_T >>= 4;
    
    var1  = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
       ((int32_t)dig_T2)) >> 11;
    
    var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) *
         ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
       ((int32_t)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    
    T  = (t_fine * 5 + 128) >> 8;
    return T;    
}

uint32_t BMP280::getPressure(void)
{
    uint32_t press_raw;
    char cmd[4];
 
    cmd[0] = 0xf7; // press_msb
    i2c_p->write(address, cmd, 1);
    i2c_p->read(address, &cmd[1], 3);
 
    press_raw = (cmd[1] << 12) | (cmd[2] << 4) | (cmd[3] >> 4);
 
    int32_t var1, var2;
    uint32_t press;
 
    var1 = (t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 1);
    var2 = (var2 >> 2) + (dig_P4 << 16);
    var1 = (((dig_P3 * (((var1 >> 2)*(var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
    var1 = ((32768 + var1) * dig_P1) >> 15;
    if (var1 == 0) {
        return 0;
    }
    press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;
    if(press < 0x80000000) {
        press = (press << 1) / var1;
    } else {
        press = (press / var1) * 2;
    }
    var1 = ((int32_t)dig_P9 * ((int32_t)(((press >> 3) * (press >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(press >> 2)) * (int32_t)dig_P8) >> 13;
    press = (press + ((var1 + var2 + dig_P7) >> 4));
    return press;    
}