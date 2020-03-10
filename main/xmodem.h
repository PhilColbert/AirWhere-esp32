/*
 * xmodem.h
 *
 *  Created on: 23 Jan 2018
 *      Author: Fugazi
 */

#ifndef XMODEM_H_
#define XMODEM_H_

#include <Arduino.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <math.h>

#include "AirWare.h"

#define DEBUG_LVL 1

#define X_STX 0x02
#define X_ACK 0x06
#define X_NAK 0x15
#define X_EOF 0x04
#define X_C 0x43

#define min(a,b) ((a)<(b)?(a):(b))

struct xmodem_chunk
{
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[1024];
    uint16_t crc;
}__attribute__((packed));

#define CRC_POLY 0x1021

static uint16_t crc_update(uint16_t crc_in, int incr);
static uint16_t crc16(const uint8_t *data, uint16_t size);
static uint16_t swap16(uint16_t in);
int xmodem_transmit();

extern HardwareSerial flarmSerial;


#endif /* XMODEM_H_ */

