#ifndef DRIVER_9DOF_H_
#define DRIVER_9DOF_H_

#include <Arduino.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean adafruit9dof_init();			// initialize the driver/sensor
void adafruit9dof_getRPH(float * roll, float * pitch, float * heading);	// obtain data

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* DRIVER_9DOF_H_ */
