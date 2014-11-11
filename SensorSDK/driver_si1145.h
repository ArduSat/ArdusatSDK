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

#ifndef DRIVER_SI1145_H_
#define DRIVER_SI1145_H_

#include <Arduino.h>
#include "config.h"

// ***************************
// *** DRIVER CAPABILITIES ***
// ***************************

#ifndef SENSOR_UVINDEX
#define SENSOR_UVINDEX
#endif


// ************************
// *** DRIVER FUNCTIONS ***
// ************************

#ifdef __cplusplus
extern "C" {
#endif

boolean si1145_init();	// initialize the driver/sensor
float si1145_getUVIndex();	// obtain measure


#ifdef __cplusplus
} // extern "C"
#endif

#endif // DRIVER_SI1145_H_
