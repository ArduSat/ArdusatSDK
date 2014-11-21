#ifndef DRIVER_9DOF_H_
#define DRIVER_9DOF_H_

#include <Arduino.h>
#include "config.h"

// ***************************
// *** DRIVER CAPABILITIES ***
// ***************************

#ifndef SENSOR_ACCELERATION
#define SENSOR_ACCELERATION
#endif

#ifndef SENSOR_ORIENTATION
#define SENSOR_ORIENTATION
#endif

#ifdef __cplusplus
extern "C" {
#endif

boolean adafruit9dof_init();			// initialize the driver/sensor
void adafruit9dof_getRPH(float * roll, float * pitch, float * heading);	// obtain data
void adafruit9dof_getACCEL(float * x, float * y, float * z);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* DRIVER_9DOF_H_ */
