#include "mbed.h"

#ifndef TRANSMIT_H
#define TRANSMIT_H

void transmit_int16(Serial &pc, int16_t d)
{
        int index = 0;
        char outbox[2] = {0,0};
        outbox[index] = d & 0x00FF;
        pc.putc(outbox[index]);
        index++;

        outbox[index] = (d & 0xFF00) >> 8;
        pc.putc(outbox[index]);
}

void transmit_int32(Serial &pc, int32_t d)
{
        int index = 0;
        char outbox[4];
        outbox[index] = d & 0x00FF;
        pc.putc(outbox[index]);
        index++;

        outbox[index] = (d & 0xFF00) >> 8;
        pc.putc(outbox[index]);
        index++;

        outbox[index] = (d & 0xFF0000) >> 16;
        pc.putc(outbox[index]);
        index++;

        outbox[index] = (d & 0xFF000000) >> 24;
        pc.putc(outbox[index]);
}

#endif