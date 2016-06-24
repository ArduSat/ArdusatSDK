/**************************************************************************/
/*!
    @file     Adafruit_TCS34725.cpp
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TCS34725 digital color sensors.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
/*
 * Modified by Sam Olds (Ardusat) to use common i2c read/write utility
 */

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <stdlib.h>
#include <math.h>

#include "Adafruit_TCS34725.h"
#include "common_utils.h"

#define read8(reg, val) readFromRegAddr(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, val, 1, BIG_ENDIAN)
#define read16(reg, val) readFromRegAddr(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, val, 2, BIG_ENDIAN)
#define write8(reg, val) writeToRegAddr(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, val, 1, BIG_ENDIAN)

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Implements missing powf function
*/
/**************************************************************************/
float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Adafruit_TCS34725::enable(void)
{
  uint8_t buf = TCS34725_ENABLE_PON;
  write8(TCS34725_ENABLE, &buf);
  delay(3);

  buf = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
  write8(TCS34725_ENABLE, &buf);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Adafruit_TCS34725::disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  read8(TCS34725_ENABLE, &reg);

  reg = reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  write8(TCS34725_ENABLE, &reg);
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_TCS34725::Adafruit_TCS34725(tcs34725IntegrationTime_t it, tcs34725Gain_t gain) 
{
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
boolean Adafruit_TCS34725::begin(void) 
{
  Wire.begin();
  
  /* Make sure we're actually connected */
  uint8_t x = 0;
  read8(TCS34725_ID, &x);
  if ((x != 0x44) && (x != 0x10))
  {
    return false;
  }
  _tcs34725Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(_tcs34725IntegrationTime);
  setGain(_tcs34725Gain);

  /* Note: by default, the device is in power down mode on bootup */
  enable();

  return true;
}
  
/**************************************************************************/
/*!
    Sets the integration time for the TC34725
*/
/**************************************************************************/
void Adafruit_TCS34725::setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, &it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/**************************************************************************/
/*!
    Adjusts the gain on the TCS34725 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void Adafruit_TCS34725::setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, &gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
void Adafruit_TCS34725::getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised) begin();

  read16(TCS34725_CDATAL, c);
  read16(TCS34725_RDATAL, r);
  read16(TCS34725_GDATAL, g);
  read16(TCS34725_BDATAL, b);
  
  /* Set a delay for the integration time */
  switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      delay(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      delay(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      delay(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      delay(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      delay(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      delay(700);
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725::calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t Adafruit_TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}


void Adafruit_TCS34725::setInterrupt(boolean i) {
  uint8_t r = 0;
  read8(TCS34725_ENABLE, &r);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS34725_ENABLE, &r);
}

void Adafruit_TCS34725::clearInterrupt(void) {
  Wire.beginTransmission(TCS34725_ADDRESS);
  Wire.write(TCS34725_COMMAND_BIT | 0x66);
  Wire.endTransmission();
}


void Adafruit_TCS34725::setIntLimits(uint16_t low, uint16_t high) {
   uint8_t buf = low & 0xFF;
   write8(0x04, &buf);
   buf = low >> 8;
   write8(0x05, &buf);
   buf = high & 0xFF;
   write8(0x06, &buf);
   buf = high >> 8;
   write8(0x07, &buf);
}
