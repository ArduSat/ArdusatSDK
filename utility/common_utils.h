/**
 * @file common_utils.h
 * @author Sam Olds (sam@ardusat.com)
 * @date 4-8-2016
 * @brief Common I2C read/write functions used by many drivers
 */

#ifndef COMMON_UTILS_H_
#define COMMON_UTILS_H_

#include "Wire.h"

typedef enum {
  BIG_ENDIAN,
  LITTLE_ENDIAN,
} endian_e;

int readFromRegAddr(uint8_t devAddr, uint8_t reg, void *val, size_t length, endian_e endianness=BIG_ENDIAN);
int writeToRegAddr(uint8_t devAddr, uint8_t reg, void *val, size_t length, endian_e endianness=BIG_ENDIAN);
#endif
