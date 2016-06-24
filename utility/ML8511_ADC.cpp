/**
 * @file ML8511_ADC.cpp
 * @author Sam Olds (sam@ardusat.com)
 * @date 1-8-2016
 * @brief ADC121xxxx 12 bit I2C ADC for the ML8511 UV sensor
 *
 * http://www.ti.com/product/adc121c027
 * Datasheet: http://www.ti.com/lit/gpn/adc121c027
 */

#include "ML8511_ADC.h"
#include <Arduino.h>
#include "common_utils.h"

#define read16(reg, val) readFromRegAddr(_addr, reg, val, 2, LITTLE_ENDIAN)
#define write8(reg, val) writeToRegAddr(_addr, reg, val, 1, LITTLE_ENDIAN)

// Constructor
ML8511_ADC::ML8511_ADC(uint8_t sensor_addr) :
  _addr(sensor_addr)
{
}

// Initializer
bool ML8511_ADC::init()
{
  uint8_t buf = 0x00;

  Wire.begin();

  buf = ADC121_CONFIG_CYCLE_32 & ADC121_CONFIG_CYCLE_MASK;
  write8(ADC121_CONFIG_REG, &buf);

  return true;
}

// Read value from converter
float ML8511_ADC::read_uv()
{
  uint16_t raw = 0;
  read16(ADC121_CONV_REG, &raw);
  return ((float) raw) / ADC121_MAX_VAL * ADC121_REF_V;
}
