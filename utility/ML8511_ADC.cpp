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

// Constructor
ML8511_ADC::ML8511_ADC(uint8_t sensor_addr)
{
  _addr = sensor_addr;
}

// Deconstructor
ML8511_ADC::~ML8511_ADC()
{
}

// Initializer
bool ML8511_ADC::init()
{
  uint8_t buf = 0x00;

  Wire.begin();

  buf = ADC121_CONFIG_CYCLE_32 & ADC121_CONFIG_CYCLE_MASK;
  write8(ADC121_CONFIG_REG, buf);

  return true;
}

// Read value from converter
float ML8511_ADC::read_uv()
{
  uint16_t raw = read16(ADC121_CONV_REG);
  return ((float) raw) / ADC121_MAX_VAL * ADC121_REF_V;
}

// Generic I2C read registers (two bytes, LSB first)
uint16_t ML8511_ADC::read16(uint8_t reg)
{
  uint16_t data = 0x0000;

  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.beginTransmission(_addr);
  Wire.requestFrom(_addr, (uint8_t)2); // request 2 bytes of data
  data = Wire.read();
  data = (data << 8) | Wire.read();
  Wire.endTransmission();

  return data;
}

// Generic I2C write data to register (single byte)
void ML8511_ADC::write8(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  return;
}
