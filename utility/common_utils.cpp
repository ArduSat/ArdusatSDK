/**
 * @file common_utils.cpp
 * @author Sam Olds (sam@ardusat.com)
 * @date 4-8-2016
 * @brief Common I2C read/write functions used by many drivers
 */

#include "common_utils.h"

/**
 * Generic I2C Read function
 *
 * @param devAddr I2C Device address
 * @param reg Register address
 * @param val pointer to byte array to read into
 * @param length number of bytes to read
 *
 * @return 0 on success, other on failure
 */
int readFromRegAddr(uint8_t devAddr, uint8_t reg, void *val, size_t length, endian_e endianness)
{
  uint8_t *byteArray = (uint8_t *) val;
  int ret;
  int readData = 0;

  Wire.beginTransmission(devAddr);

  if (Wire.write(reg) < 1) {
    return -1;
  }

  if ((ret = Wire.endTransmission(false)) != 0) {
    return ret;
  }

  if (byteArray == NULL || length == 0) {
    return 0;
  }

  Wire.beginTransmission(devAddr);
  Wire.requestFrom(devAddr, (uint8_t) length);

  while (Wire.available() > 0 && readData < length) {
    byteArray[endianness == BIG_ENDIAN ? readData : length - 1 - readData] = Wire.read();
    readData++;
  }

  if ((ret = Wire.endTransmission(true)) != 0) {
    return ret;
  }

  return 0;
}

/**
 * Generic I2C Write function
 *
 * @param devAddr I2C Device address
 * @param reg Register address
 * @param val pointer to byte array to write from
 * @param length number of bytes to write
 *
 * @return 0 on success, other on failure
 */
int writeToRegAddr(uint8_t devAddr, uint8_t reg, void *val, size_t length, endian_e endianness)
{
  uint8_t *byteArray = (uint8_t *) val;
  int ret;

  Wire.beginTransmission(devAddr);

  if (Wire.write(reg) < 1) {
    return -1;
  }

  for (int i = 0; i < length; i++) {
    if (Wire.write(byteArray[endianness == BIG_ENDIAN ? i : length - 1 - i]) < 1) {
      return -1;
    }
  }

  if ((ret = Wire.endTransmission()) != 0) {
    return ret;
  }

  return 0;
}
