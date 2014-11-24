/*
 * config_drivers.h
 *
 *  Created on: 21 oct. 2014
 *      Author: JF OMHOVER
 */

#ifndef CONFIG_DRIVERS_H_
#define CONFIG_DRIVERS_H_

#include "config.h"

#if defined(LOAD_DRIVER_TSL2561)
#include "driver_tsl2561.h"
#endif

#if defined(LOAD_DRIVER_TMP102)
#include "driver_tmp102.h"
#endif

#if defined(LOAD_DRIVER_MLX90614)
#include "driver_mlx90614.h"
#endif

#if defined(LOAD_DRIVER_ADAFRUIT9DOF)
#include "driver_9dof.h"
#endif

#if defined(LOAD_DRIVER_SI1145)
#include "driver_si1145.h"
#endif

#if defined(LOAD_DRIVER_ML8511)
#include "driver_ml8511.h"
#endif

#endif /* CONFIG_DRIVERS_H_ */
