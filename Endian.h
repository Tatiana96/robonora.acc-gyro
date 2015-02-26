#ifndef ENDIAN_H
#define ENDIAN_H

#include <Arduino.h>
#include "config.h"

/*! This class provides functions to swap endiannes. */
class Endian
{

  public:

    /*!
     * This function swaps the endianness of a 16-bits value.
     *
     * @param data Pointer to a 16-bits value
     */
    static void swap16(uint8_t* data);

    /*!
     * This function swaps the endianness of a 32-bits value.
     *
     * @param data Pointer to a 32-bits value
     */
    static void swap32(uint8_t* data);

    /*!
     * This function swaps the endianness of multiple 16-bits values.
     *
     * @param Pointer to the first 16-bits value
     * @param count Number of 16-bits values to process
     */
    static void multiSwap16(uint8_t* data, int count);
};

#endif // ENDIAN_H
