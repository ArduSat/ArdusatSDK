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

#ifdef LOAD_DRIVER_TSL2561

#ifdef __cplusplus
extern "C" {
#endif

boolean tsl2561_init();
float tsl2561_getLux();
uint16_t tsl2561_getLuminosity(int8_t ch);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // LUMINOSITY_TSL2561

#endif /* DRIVER_TSL2561_H_ */
