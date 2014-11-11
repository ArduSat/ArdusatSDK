#ifndef OUTPUT_JSON_H_
#define OUTPUT_JSON_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

const char * dataToJSON(void * input);
const char * headerToJSON(_data_header_t * header);	// warning : resets the internal buffer
const char * accelerationToJSON(acceleration_t * input);
const char * temperatureToJSON(temperature_t * input);
const char * orientationToJSON(orientation_t * input);
const char * luminosityToJSON(luminosity_t * input);
const char * uvlightToJSON(uvlight_t * input);

const char * anythingFloatToJSON(const char * sensorname, const char * unit, float value, int precision);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* OUTPUT_JSON_H_ */
