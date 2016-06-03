/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
 * Modified by Ben Peters (Ardusat) to avoid namespace collisions 
 *
 * Modified by Sam Olds (Ardusat) to also support SI1132 sensor
 * https://hackaday.io/project/5684/logs
 * 
 * Modified by Sam Olds (Ardusat) to use common i2c read/write utility
 */

#include "Adafruit_SI1145.h"
#include "common_utils.h"

#define read8(reg, val) readFromRegAddr(_addr, reg, val, 1, BIG_ENDIAN)
#define read16(reg, val) readFromRegAddr(_addr, reg, val, 2, BIG_ENDIAN)
#define write8(reg, val) writeToRegAddr(_addr, reg, val, 1, BIG_ENDIAN)

Adafruit_SI1145::Adafruit_SI1145() {
  _addr = SI1145_ADDR;
}


boolean Adafruit_SI1145::begin(void) {
  Wire.begin();
 
  uint8_t id;
  read8(SI1145_REG_PARTID, &id);
  //if (id != 0x45) return false; // look for SI1145
  if (id != 0x32) return false; // look for SI1132 instead of SI1145
  
  reset();
  

    /***********************************/
  // enable UVindex measurement coefficients!
  uint8_t buf = 0x29;
  write8(SI1145_REG_UCOEFF0, &buf);
  buf = 0x89;
  write8(SI1145_REG_UCOEFF1, &buf);
  buf = 0x02;
  write8(SI1145_REG_UCOEFF2, &buf);
  buf = 0x00;
  write8(SI1145_REG_UCOEFF3, &buf);

  // enable UV sensor
  writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);
  // enable interrupt on every sample
  buf = SI1145_REG_INTCFG_INTOE;
  write8(SI1145_REG_INTCFG, &buf);
  buf = SI1145_REG_IRQEN_ALSEVERYSAMPLE;
  write8(SI1145_REG_IRQEN, &buf);

/****************************** Prox Sense 1 */

  // program LED current
  buf = 0x03; // 20mA for LED 1 only
  write8(SI1145_REG_PSLED21, &buf);
  writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);



  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


/************************/

  // measurement rate for auto
  buf = 0xFF; // 255 * 31.25uS = 8ms
  write8(SI1145_REG_MEASRATE0, &buf);
  
  // auto run
  buf = SI1145_PSALS_AUTO;
  write8(SI1145_REG_COMMAND, &buf);

  return true;
}

void Adafruit_SI1145::reset() {
  uint8_t buf = 0;
  write8(SI1145_REG_MEASRATE0, &buf);
  write8(SI1145_REG_MEASRATE1, &buf);
  write8(SI1145_REG_IRQEN, &buf);
  write8(SI1145_REG_IRQMODE1, &buf);
  write8(SI1145_REG_IRQMODE2, &buf);
  write8(SI1145_REG_INTCFG, &buf);

  buf = 0xFF;
  write8(SI1145_REG_IRQSTAT, &buf);

  buf = SI1145_RESET;
  write8(SI1145_REG_COMMAND, &buf);
  delay(10);

  buf = 0x17;
  write8(SI1145_REG_HWKEY, &buf);
  
  delay(10);
}


//////////////////////////////////////////////////////

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void) {
  uint16_t val;
  read16(0x2C, &val);
  return val;
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void) {
  uint16_t val;
  read16(0x22, &val);
  return val;
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void) {
  uint16_t val;
  read16(0x24, &val);
  return val;
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void) {
  uint16_t val;
  read16(0x26, &val);
  return val;
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);
  
  write8(SI1145_REG_PARAMWR, &v);
  v = p | SI1145_PARAM_SET; // Reusing variable v
  write8(SI1145_REG_COMMAND, &v);

  uint8_t data;
  read8(SI1145_REG_PARAMRD, &data);
  return data;
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) {
  p |= SI1145_PARAM_QUERY;
  write8(SI1145_REG_COMMAND, &p);

  uint8_t data;
  read8(SI1145_REG_PARAMRD, &data);
  return data;
}

/*********************************************************************/
