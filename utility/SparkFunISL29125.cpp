/******************************************************************************
SparkFunISL29125.cpp
Core implementation file for the ISL29125 RGB sensor library.
Jordan McConnell @ SparkFun Electronics
25 Mar 2014
https://github.com/sparkfun/ISL29125_Breakout
This file implements the functions of the SFE_ISL29125 sensor class as well as
providing documentation on what each function does.
Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5
This code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given. 
******************************************************************************/
/*
 * Modified by Sam Olds (Ardusat) to use common i2c read/write utility
 */

#include "SparkFunISL29125.h"
#include "common_utils.h"

#define read8(reg, val) readFromRegAddr(_addr, reg, val, 1, BIG_ENDIAN)
#define read16(reg, val) readFromRegAddr(_addr, reg, val, 2, BIG_ENDIAN)
#define write8(reg, val) writeToRegAddr(_addr, reg, val, 1, BIG_ENDIAN)
#define write16(reg, val) writeToRegAddr(_addr, reg, val, 2, BIG_ENDIAN)


// Constructor - Creates sensor object and sets I2C address
SFE_ISL29125::SFE_ISL29125(uint8_t addr) 
{
  _addr = addr;
}

// Destructor - Deletes sensor object
SFE_ISL29125::~SFE_ISL29125()
{

}

// Initialize - returns true if successful
// Starts Wire/I2C Communication
// Verifies sensor is there by checking its device ID
// Resets all registers/configurations to factory default
// Sets configuration registers for the common use case
bool SFE_ISL29125::init()
{
  bool ret = true;
  uint8_t data = 0x00;
  
  // Start I2C
  Wire.begin();
  
  // Check device ID
  read8(DEVICE_ID, &data);
  if (data != 0x7D)
  {
    ret &= false;
  }
  
  // Reset registers
  ret &= reset();
  
  // Set to RGB mode, 10k lux, and high IR compensation
  ret &= config(CFG1_MODE_RGB | CFG1_10KLUX, CFG2_IR_ADJUST_HIGH, CFG_DEFAULT);
  
  return ret;
}

// Reset all registers - returns true if successful
bool SFE_ISL29125::reset()
{
  uint8_t data = 0x00;
  uint8_t buf = 0x46;
  // Reset registers
  write8(DEVICE_ID, &buf);
  buf = 0; // Reusing buf

  // Check reset
  read8(CONFIG_1, &buf);
  data = buf;
  read8(CONFIG_2, &buf);
  data |= buf;
  read8(CONFIG_3, &buf);
  data |= buf;
  read8(STATUS, &buf);
  data |= buf;
  if (data != 0x00)
  {
    return false;
  }
  return true;
}

// Setup Configuration registers (three registers) - returns true if successful
// Use CONFIG1 variables from SFE_ISL29125.h for first parameter config1, CONFIG2 for config2, 3 for 3
// Use CFG_DEFAULT for default configuration for that register
bool SFE_ISL29125::config(uint8_t config1, uint8_t config2, uint8_t config3)
{
  bool ret = true;
  uint8_t data = 0x00;
  
  // Set 1st configuration register
  write8(CONFIG_1, &config1);
  // Set 2nd configuration register
  write8(CONFIG_2, &config2);
  // Set 3rd configuration register
  write8(CONFIG_3, &config3);
  
  // Check if configurations were set correctly
  read8(CONFIG_1, &data);
  if (data != config1)
  {
    ret &= false;
  }
  read8(CONFIG_2, &data);
  if (data != config2)
  {
    ret &= false;
  }
  read8(CONFIG_3, &data);
  if (data != config3)
  {
    ret &= false;
  }
  return ret;
}

// Sets upper threshold value for triggering interrupts
void SFE_ISL29125::setUpperThreshold(uint16_t data)
{
  write16(THRESHOLD_HL, &data);
}

// Sets lower threshold value for triggering interrupts
void SFE_ISL29125::setLowerThreshold(uint16_t data)
{
  write16(THRESHOLD_LL, &data);
}

// Check what the upper threshold is, 0xFFFF by default
uint16_t SFE_ISL29125::readUpperThreshold()
{
  uint16_t buf = 0;
  read16(THRESHOLD_HL, &buf);
  return buf;
}

// Check what the upper threshold is, 0x0000 by default
uint16_t SFE_ISL29125::readLowerThreshold()
{
  uint16_t buf = 0;
  read16(THRESHOLD_LL, &buf);
  return buf;
}

// Read the latest Sensor ADC reading for the color Red
uint16_t SFE_ISL29125::readRed()
{
  uint16_t buf = 0;
  read16(RED_L, &buf);
  return buf;
}

// Read the latest Sensor ADC reading for the color Green
uint16_t SFE_ISL29125::readGreen()
{
  uint16_t buf = 0;
  read16(GREEN_L, &buf);
  return buf;
}

// Read the latest Sensor ADC reading for the color Blue
uint16_t SFE_ISL29125::readBlue()
{
  uint16_t buf = 0;
  read16(BLUE_L, &buf);
  return buf;
}

// Check status flag register that allows for checking for interrupts, brownouts, and ADC conversion completions
uint8_t SFE_ISL29125::readStatus()
{
  uint8_t buf = 0;
  read8(STATUS, &buf);
  return buf;
}
