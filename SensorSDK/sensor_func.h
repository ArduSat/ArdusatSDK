// external (basic) functions for obtaining values from the SDK
// to be configured via manifest

#ifndef SENSORFUNC_H_
#define SENSORFUNC_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean setupAccelerationSensor();
void readAcceleration(acceleration_t * accel);

// TODO : returning a struct with the sensor id somehow, to read sensors of several types
boolean setupTemperatureSensor(); // uint16_t id);
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
