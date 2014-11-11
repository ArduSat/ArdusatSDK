/*
 * driver_tsl2561.h
 *
 *  Created on: 23 sept. 2014
 *      Author: JF OMHOVER
 */

#ifndef DRIVER_TSL2561_H_
#define DRIVER_TSL2561_H_

#include <Arduino.h>
#include "config.h"

// ***************************
// *** DRIVER CAPABILITIES ***
// ***************************

#ifndef SENSOR_LUMINOSITY
#define SENSOR_LUMINOSITY
#endif


// ************************
// *** DRIVER FUNCTIONS ***
// ************************

#ifdef __cplusplus
extern "C" {
#endif

boolean tsl2561_init();						// initialize the driver/sensor
float tsl2561_getLux();						// get lux
uint16_t tsl2561_getLuminosity(int8_t ch); 	// get raw luminosity

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* DRIVER_TSL2561_H_ */
