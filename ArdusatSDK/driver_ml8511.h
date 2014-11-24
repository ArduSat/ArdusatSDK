/**
 * =====================================================================================
 *
 *       Filename:  driver_ml8511.h
 *
 *    Description:  Driver library for ML8511 UV Sensor breakout board, based on the 
 *                  MP8511 UV sensor.
 *
 *                  https://www.sparkfun.com/products/12705
 *
 *        Version:  1.0
 *        Created:  12/03/2014 11:42:16
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *
 * =====================================================================================
 */

#ifndef DRIVER_ML8511_H_
#define DRIVER_ML8511_H_
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

boolean ml8511_init();	// initialize the driver/sensor
float ml8511_getUV();	// obtain measure

#ifdef __cplusplus
} // extern "C"
#endif


#endif 
