// external (basic) functions for obtaining values from the SDK
// to be configured via manifest

#ifndef SENSORFUNC_H_
#define SENSORFUNC_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO : returning a struct with the sensor id somehow, to read sensors of several types
boolean setupTemperatureSensor(); // uint16_t id);
void readTemperature(temperature_t * temp);

boolean setupInfraredTemperatureSensor();
void readInfraredTemperature(temperature_t * temp);

boolean setupLuminositySensor();
void readLuminosity(luminosity_t * lum);

boolean setupUVLightSensor();
void readUVLight(uvlight_t * uv);

boolean setupAccelerationSensor();
void readAcceleration(acceleration_t * accel);

boolean setupOrientationSensor();
void readOrientation(orientation_t * orient);



#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SENSORFUNC_H_ */
