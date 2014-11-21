#ifndef OUTPUT_JSON_H_
#define OUTPUT_JSON_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JSON_TEXTBUFFER_MAXSIZE	256

const char * valueToJSON(const char *sensor_name, uint8_t unit, float value);
const char * accelerationToJSON(const char *sensor_name, acceleration_t * input);
const char * temperatureToJSON(const char *sensor_name, temperature_t * input);
const char * orientationToJSON(const char *sensor_name, orientation_t * input);
const char * luminosityToJSON(const char *sensor_name, luminosity_t * input);
const char * uvlightToJSON(const char *sensor_name, uvlight_t * input);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* OUTPUT_JSON_H_ */
