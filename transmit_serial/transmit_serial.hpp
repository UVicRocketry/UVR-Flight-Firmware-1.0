#include "mbed.h"

#ifndef TRANSMIT_H
#define TRANSMIT_H

void transmit_int16(Serial &pc, int16_t d)
{
	pc.putc(d & 0x00FF);
	pc.putc((d & 0xFF00) >> 8);
}

void transmit_int32(Serial &pc, int32_t d)
{
	pc.putc(d & 0x00FF);
	pc.putc((d & 0xFF00) >> 8);
	pc.putc((d & 0xFF0000) >> 16);
	pc.putc((d & 0xFF000000) >> 24);
}

#endif