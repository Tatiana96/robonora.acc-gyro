/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include "config.h"

#ifndef I2C_SPEED
  #define I2C_SPEED 100000 // 100kHz or 400kHz
#endif
#ifndef I2C_TIMEOUT
  #define I2C_TIMEOUT 5000 // 5ms
#endif

#define I2C_TIMEOUT_START_BIT 1
#define I2C_TIMEOUT_ADDRESSING_TO_TRANSMIT 2
#define I2C_TIMEOUT_TRANSMITTING 3
#define I2C_TIMEOUT_REPEATED_START 4
#define I2C_TIMEOUT_ADDRESSING_TO_RECEIVE 5
#define I2C_TIMEOUT_RECEIVING 6
#define I2C_TIMEOUT_STOP_BIT 7

#define START 0x08
#define REPEATED_START 0x10
#define MT_SLA_ACK 0x18
#define MT_SLA_NACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NACK 0x30
#define MR_SLA_ACK 0x40
#define MR_SLA_NACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58
#define LOST_ARBTRTN 0x38
#define TWI_STATUS (TWSR & 0xF8)
#define SLA_W(address) (address << 1)
#define SLA_R(address) ((address << 1) + 0x01)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

class I2C
{
  public:
    static void begin();
    static void end();
    static uint8_t write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
    static uint8_t read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
  private:
    static uint8_t start();
    static uint8_t sendAddress(uint8_t i2cAddress);
    static uint8_t sendByte(uint8_t i2cData);
    static uint8_t receiveByte(uint8_t ack);
    static uint8_t stop();
    static void lockUp();
    static uint8_t returnStatus;
    static uint8_t nack;
};

#endif // I2C_H
