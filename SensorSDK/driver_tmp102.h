/*
 * driver_tmp102.h
 *
 *  Created on: 25 sept. 2014
 *      Author: JF OMHOVER
 */

#ifndef DRIVER_TMP102_H_
#define DRIVER_TMP102_H_

#include <Arduino.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean tmp102_init();
float tmp102_getTempCelsius();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* DRIVER_TMP102_H_ */
