/*
 * driver_mlx90614.h
 *
 *  Created on: 25 sept. 2014
 *      Author: JF OMHOVER
 */

#ifndef DRIVER_MLX90614_H_
#define DRIVER_MLX90614_H_

#include <Arduino.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

boolean mlx90614_init();
float mlx90614_getTempCelsius();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* DRIVER_MLX90614_H_ */
