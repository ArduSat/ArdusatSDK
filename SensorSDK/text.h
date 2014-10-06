#ifndef TEXT_H_
#define TEXT_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

const char * dataToText(void * input);
const char * headerToText(_data_header_t * header);	// warning : resets the internal buffer
const char * accelerationToText(acceleration_t * input);
const char * temperatureToText(temperature_t * input);
const char * orientationToText(orientation_t * input);
const char * luminosityToText(luminosity_t * input);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* TEXT_H_ */
