/***************************************************
  This is a library for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Arduino.h>
#include "config.h"
#include "obcl.h"
#include "cc_wire.h"

#define MLX90614_I2CADDR 0x5A

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F


/*********************************************************************/

uint16_t _mlx90614_read16(uint8_t a) {
  uint16_t ret;

  cc_wire_beginTransmission(MLX90614_I2CADDR); // start transmission to device
  cc_wire_write(a); // sends register address to read from
  cc_wire_endTransmission(false); // end transmission

  cc_wire_requestFrom(MLX90614_I2CADDR, (uint8_t)3);// send data n-bytes read
  ret = cc_wire_read(); // receive DATA
  ret |= cc_wire_read() << 8; // receive DATA

  uint8_t pec = cc_wire_read();

  return ret;
}

//////////////////////////////////////////////////////

float _mlx90614_readTemp(uint8_t reg) {
  float temp;

  temp = _mlx90614_read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

float _mlx90614_readObjectTempF(void) {
  return (_mlx90614_readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}


float _mlx90614_readAmbientTempF(void) {
  return (_mlx90614_readTemp(MLX90614_TA) * 9 / 5) + 32;
}

float _mlx90614_readObjectTempC(void) {
  return _mlx90614_readTemp(MLX90614_TOBJ1);
}


float _mlx90614_readAmbientTempC(void) {
  return _mlx90614_readTemp(MLX90614_TA);
}


boolean mlx90614_init() {
	obcl_begin();
	return(true);
}

float mlx90614_getTempCelsius() {
	return(_mlx90614_readObjectTempC());
}
