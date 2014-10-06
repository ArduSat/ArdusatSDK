#ifndef SENSORFUNC_H_
#define SENSORFUNC_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean setupAccelerationSensor();
void readAcceleration(acceleration_t * accel);

boolean setupTemperatureSensor();
void readTemperature(temperature_t * temp);

boolean setupOrientationSensor();
void readOrientation(orientation_t * orient);

boolean setupInfraredTemperatureSensor();
void readInfraredTemperature(temperature_t * temp);

boolean setupLuminositySensor();
void readLuminosity(luminosity_t * lum);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SENSORFUNC_H_ */
