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

#include <Arduino.h>
#include "config.h"

#include "I2C.h"

// ================================ Public ================================ //

void I2C::begin()
{
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // Activate internal pull-ups for TWI
    // As per note from ATmega8 manual page 167
    sbi(PORTC, 4);
    sbi(PORTC, 5);
  #else
    // Activate internal pull-ups for TWI
    // As per note from ATmega128 manual page 204
    sbi(PORTD, 0);
    sbi(PORTD, 1);
  #endif
  // Initialise TWI prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;
  // Enable TWI module and ACKs
  TWCR = _BV(TWEN) | _BV(TWEA);
}

void I2C::end()
{
  TWCR = 0;
}

uint8_t I2C::write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size)

{
  returnStatus = 0;
  returnStatus = start();
  if(returnStatus) { return returnStatus; }
  returnStatus = sendAddress(SLA_W(address));
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_ADDRESSING_TO_TRANSMIT; }
    return returnStatus;
  }
  returnStatus = sendByte(reg);
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_TRANSMITTING; }
    return returnStatus;
  }
  for(uint8_t i=0; i<size; i++)
  {
    returnStatus = sendByte(data[i]);
    if(returnStatus)
    {
      if(returnStatus == 1) { return I2C_TIMEOUT_TRANSMITTING; }
      return returnStatus;
    }
  }
  returnStatus = stop();
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_STOP_BIT; }
    return returnStatus;
  }
  return returnStatus;
}

uint8_t I2C::read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size)
{
  if(size == 0) { size++; }
  nack = size - 1;
  returnStatus = 0;
  returnStatus = start();
  if(returnStatus) { return returnStatus; }
  returnStatus = sendAddress(SLA_W(address));
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_ADDRESSING_TO_TRANSMIT; }
    return returnStatus;
  }
  returnStatus = sendByte(reg);
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_TRANSMITTING; }
    return returnStatus;
  }
  returnStatus = start();
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_REPEATED_START; }
    return returnStatus;
  }
  returnStatus = sendAddress(SLA_R(address));
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_ADDRESSING_TO_RECEIVE; }
    return returnStatus;
  }
  for(uint8_t i=0; i<size; i++)
  {
    if(i == nack)
    {
      returnStatus = receiveByte(0);
      if(returnStatus == 1) { return I2C_TIMEOUT_RECEIVING; }
      if(returnStatus != MR_DATA_NACK) { return returnStatus; }
    }
    else
    {
      returnStatus = receiveByte(1);
      if(returnStatus == 1) { return I2C_TIMEOUT_RECEIVING; }
      if(returnStatus != MR_DATA_ACK) { return returnStatus; }
    }
    data[i] = TWDR;
  }
  returnStatus = stop();
  if(returnStatus)
  {
    if(returnStatus == 1) { return I2C_TIMEOUT_STOP_BIT; }
    return returnStatus;
  }
  return returnStatus;
}

// ================================ Private ================================ //

uint8_t I2C::returnStatus;
uint8_t I2C::nack;

uint8_t I2C::start()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  uint32_t startingTime = micros();
  while (!(TWCR & (1 << TWINT)))
  {
    if((micros() - startingTime) >= I2C_TIMEOUT)
    {
      lockUp();
      return 1;
    }
  }
  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))
  {
    return 0;
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    uint8_t bufferedStatus = TWI_STATUS;
    lockUp();
    return bufferedStatus;
  }
  return TWI_STATUS;
}

uint8_t I2C::sendAddress(uint8_t i2cAddress)
{
  TWDR = i2cAddress;
  TWCR = (1 << TWINT) | (1 << TWEN);
  uint32_t startingTime = micros();
  while (!(TWCR & (1 << TWINT)))
  {
    if((micros() - startingTime) >= I2C_TIMEOUT)
    {
      lockUp();
      return 1;
    }
  }
  if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK))
  {
    return 0;
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK))
  {
    stop();
    return bufferedStatus;
  }
  else
  {
    lockUp();
    return bufferedStatus;
  }
}

uint8_t I2C::sendByte(uint8_t i2cData)
{
  TWDR = i2cData;
  TWCR = (1 << TWINT) | (1 << TWEN);
  uint32_t startingTime = micros();
  while (!(TWCR & (1 << TWINT)))
  {
    if((micros() - startingTime) >= I2C_TIMEOUT)
    {
      lockUp();
      return 1;
    }
  }
  if (TWI_STATUS == MT_DATA_ACK)
  {
    return 0;
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (TWI_STATUS == MT_DATA_NACK)
  {
    stop();
    return bufferedStatus;
  }
  else
  {
    lockUp();
    return bufferedStatus;
  }
}

uint8_t I2C::receiveByte(uint8_t ack)
{
  if(ack)
  {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  }
  else
  {
    TWCR = (1 << TWINT) | (1 << TWEN);
  }
  uint32_t startingTime = micros();
  while (!(TWCR & (1 << TWINT)))
  {
    if((micros() - startingTime) >= I2C_TIMEOUT)
    {
      lockUp();
      return 1;
    }
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    uint8_t bufferedStatus = TWI_STATUS;
    lockUp();
    return bufferedStatus;
  }
  return TWI_STATUS;
}

uint8_t I2C::stop()
{
  TWCR = (1 << TWINT) | (1 << TWEN)| (1 << TWSTO);
  uint32_t startingTime = micros();
  while ((TWCR & (1 << TWSTO)))
  {
    if((micros() - startingTime) >= I2C_TIMEOUT)
    {
      lockUp();
      return 1;
    }
  }
  return 0;
}

void I2C::lockUp()
{
  TWCR = 0; // Releases SDA and SCL lines to high impedance
  TWCR = _BV(TWEN) | _BV(TWEA); // Reinitialise TWI
}
