#ifndef CONFIG_H_
#define CONFIG_H_

#include "sensor_addr.h"

// TODO : configuration of the intermediary level
// TODO : management of an external "user defined" manifest

// OUTPUT
#define SENSORSDK_OUTPUT_CSV
#define SENSORSDK_OUTPUT_JSON


// ***************
// *** DRIVERS ***
// ***************

#define LOAD_DRIVER_TSL2561
#define LOAD_DRIVER_TMP102
#define LOAD_DRIVER_MLX90614
#define LOAD_DRIVER_ADAFRUIT9DOF
#define LOAD_DRIVER_SI1145


// *****************
// *** ADDRESSES ***
// *****************

#define DRIVER_TSL2561_ADDR		0x39
#define DRIVER_TMP102_ADDR		0x48
#define DRIVER_MLX90614_ADDR	0x5A
#define DRIVER_SI1145_ADDR		0x60

#endif /* CONFIG_H_ */
