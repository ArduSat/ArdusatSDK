#ifndef CONFIG_H_
#define CONFIG_H_

#include "sensor_addr.h"

// TODO : configuration of the intermediary level
// TODO : management of an external "user defined" manifest

// LUMINOSITY
#define LOAD_DRIVER_TSL2561
#define DRIVER_TSL2561_ADDR	0x39

#define LOAD_DRIVER_TMP102
#define DRIVER_TMP102_ADDR	SENSORADDR_TMP102_LL

#define LOAD_DRIVER_MLX90614
#define DRIVER_MLX90614_ADDR	0x5A

// DRIVER_TSL2561

#if defined(LOAD_DRIVER_TSL2561)
#define SENSOR_LUMINOSITY
#endif

#if defined(LOAD_DRIVER_TMP102)
#define SENSOR_TEMPERATURE
#endif

#if defined(LOAD_DRIVER_MLX90614)
#define SENSOR_IRTHERM
#endif

#endif /* CONFIG_H_ */
