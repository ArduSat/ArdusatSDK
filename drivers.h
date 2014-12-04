/*
 * =====================================================================================
 *
 *       Filename:  drivers.h
 *
 *    Description:  Implementations of sensor-specific initialization and read driver 
 *                  functions for use by the higher-level Ardusat SDK
 *
 *        Version:  1.0
 *        Created:  12/03/2014 14:47:14
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *
 * =====================================================================================
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <Arduino.h>

#define DRIVER_TSL2561_ADDR		0x39
#define DRIVER_TMP102_ADDR		0x48
#define DRIVER_MLX90614_ADDR	0x5A
#define DRIVER_SI1145_ADDR		0x60
#define DRIVER_ML8511_UV_PIN A0
#define DRIVER_ML8511_REF_PIN A1

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Adafruit 9DOF IMU breakout board features L3GD20H Gyroscope and LSM303 6-axis 
 * magnetometer + accelerometer.
 *
 * http://www.adafruit.com/product/1714
 */
boolean adafruit9dof_init();			// initialize the driver/sensor
void adafruit9dof_getOrientation(float * roll, float * pitch, float * heading);	// obtain data
void adafruit9dof_getAccel(float * x, float * y, float * z);
void adafruit9dof_getMag(float * x, float * y, float * z);

/**
 * ML8511 breakout board contains an MP8511 UV light sensor
 *
 * https://www.sparkfun.com/products/12705
 */
boolean ml8511_init();
float ml8511_getUV();

/**
 * SI1145 breakout board is a UV/Light sensor from Adafruit. It was included in earlier
 * versions of the SpaceKit and is still supported, though not enabled by default.
 *
 * https://learn.adafruit.com/adafruit-si1145-breakout-board-uv-ir-visible-sensor/overview
 */
boolean si1145_init();
float si1145_getUVIndex();

/**
 * MLX90614 IR Thermometer for non-contact temperature sensing.
 *
 * https://www.sparkfun.com/products/9570
 */
boolean mlx90614_init();
float mlx90614_getTempCelsius();

/**
 * TMP102 Temperature sensor
 *
 * https://www.sparkfun.com/products/11931
 */
boolean tmp102_init();
float tmp102_getTempCelsius();		

/**
 * TSL2561 Luminosity Sensor
 *
 * http://www.adafruit.com/product/439
 */
boolean tsl2561_init();
float tsl2561_getLux();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
