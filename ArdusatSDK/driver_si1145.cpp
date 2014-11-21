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

#include <Arduino.h>
#include "config.h"

#include "obcl.h"
#include "cc_wire.h"

#include "driver_si1145.h"
#include "Ardusat_Adafruit_SI1145.h"

Adafruit_SI1145 uv = Adafruit_SI1145(DRIVER_SI1145_ADDR);

boolean si1145_init() {
	obcl_begin();
	return (uv.begin());
}

float si1145_getUVIndex() {
	float UVindex = uv.readUV();
	// the index is multiplied by 100 so to get the
	// integer index, divide by 100!
	UVindex /= 100.0;
	return (UVindex);
}
