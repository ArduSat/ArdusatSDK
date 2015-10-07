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
#include <utility/mlx90614.h>
#include <utility/Adafruit_L3GD20_U.h>
#include <utility/Adafruit_SI1145.h>
#include <utility/BMP180.h>
#include <utility/TSL2561.h>
#include <utility/pololu_LSM303.h>

#define DRIVER_TSL2561_ADDR		          0x39 // 0x49 for lemsens
#define DRIVER_LEMSENS_TSL2561_ADDR		  0x49 // (lemsens)
#define DRIVER_TMP102_ADDR		          0x48 // 0x4B for lemsens
#define DRIVER_LEMSENS_TMP102_1_ADDR		0x4B // (lemsens)
#define DRIVER_MLX90614_ADDR	          0x5A
#define DRIVER_SI1145_ADDR		          0x60
#define DRIVER_BMP180_ADDR              0x77
#define DRIVER_LSM303_DTR_ADDR          0x1E
#define DRIVER_ML8511_UV_PIN            A0
#define DRIVER_ML8511_REF_PIN           A1

/* Constants */
#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Adafruit 9DOF IMU breakout board features L3GD20H Gyroscope and LSM303 6-axis 
 * magnetometer + accelerometer.
 *
 * http://www.adafruit.com/product/1714
 */
typedef enum {
  LSM303_ACCEL_GAIN2G =  0b00000000,
  LSM303_ACCEL_GAIN4G =  0b00001000,
  LSM303_ACCEL_GAIN6G =  0b00010000,
  LSM303_ACCEL_GAIN8G =  0b00011000,
  LSM303_ACCEL_GAIN16G = 0b00100000,
} lsm303_accel_gain_e;

typedef enum {
  LSM303_MAG_SCALE1_3GAUSS = 0b10000000,
  LSM303_MAG_SCALE2GAUSS =   0b00000000,
  LSM303_MAG_SCALE2_5GAUSS = 0b10000001,
  LSM303_MAG_SCALE4GAUSS =   0b00100000,
  LSM303_MAG_SCALE4_7GAUSS = 0b10100000,
  LSM303_MAG_SCALE5_6GAUSS = 0b10100001,
  LSM303_MAG_SCALE8GAUSS =   0b01000000,
  LSM303_MAG_SCALE12GAUSS =  0b01100000,
} lsm303_mag_scale_e;
#define SENSORS_MGAUSS_TO_UTESLA 0.1F

typedef struct
{
  lsm303_accel_gain_e gain;
} config_lsm303_accel_t;

typedef struct
{
  lsm303_mag_scale_e scale;
} config_lsm303_mag_t;

typedef struct {
  float sensitivity;
} config_l3gd20_t;

boolean l3gd20h_init();
void l3gd20h_getOrientation(float *x, float *y, float *z);
void l3gd20h_getRawAngularRates(int16_t *pX, int16_t *pY, int16_t *pZ);
void l3gd20h_getRawTemperature(int8_t *pRawTemperature);

boolean lsm303_accel_init();
boolean lsm303_mag_init();
void lsm303_getAccel(float * x, float * y, float * z);
void lsm303_getRawAcceleration(int16_t *pX, int16_t *pY, int16_t *pZ);
void lsm303_getRawTemperature(int16_t *pRawTemperature);
void lsm303_getMag(float * x, float * y, float * z);
void lsm303_getRawMag(int16_t *pX, int16_t *pY, int16_t *pZ);

/*
 * BMP180 Barometric altimeter is on Adafruit 10DOF, gives atmospheric pressure/altitude.
 *
 * http://www.adafruit.com/product/1604
 */
boolean bmp180_init();
void bmp180_getRawTemperature(uint16_t *temp);
void bmp180_getTemperature(float *temp);
void bmp180_getRawPressure(uint32_t *pressure);
void bmp180_getPressure(float *pressure);

// helper functions for pressure conversions
float pressureToAltitude(float seaLevelPressure, float atmosphericPressure);
float seaLevelPressureForAltitude(float altitude, float atmosphericPressure);

/**
 * ML8511 breakout board contains an MP8511 UV light sensor
 *
 * https://www.sparkfun.com/products/12705
 */
boolean ml8511_init();
float ml8511_getUV(int pin);

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
