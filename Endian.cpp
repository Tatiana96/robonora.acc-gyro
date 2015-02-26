#include <Arduino.h>
#include "config.h"

#include "Endian.h"

void Endian::swap16(uint8_t* data)
{
  uint8_t swap;
  swap = *data;
  *data = *(data+1);
  *(data+1) = swap;
}

void Endian::swap32(uint8_t* data)
{
  uint8_t swap;
  swap = *data;
  *data = *(data+3);
  *(data+3) = swap;
  swap = *(data+1);
  *(data+1) = *(data+2);
  *(data+2) = swap;
}

void Endian::multiSwap16(uint8_t* data, int count)
{
  for(int i=0; i<count; i++)
  {
    swap16(data);
    data += 2;
  }
}
