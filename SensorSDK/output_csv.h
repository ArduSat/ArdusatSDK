#ifndef OUTPUT_CSV_H_
#define OUTPUT_CSV_H_

#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

const char * dataToCSV(void * input);
const char * headerToCSV(_data_header_t * header);	// warning : resets the internal buffer
const char * accelerationToCSV(acceleration_t * input);
const char * temperatureToCSV(temperature_t * input);
const char * orientationToCSV(orientation_t * input);
const char * luminosityToCSV(luminosity_t * input);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* OUTPUT_CSV_H_ */
