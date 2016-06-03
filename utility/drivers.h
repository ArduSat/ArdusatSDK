/**
 * @file   drivers.h
 * @author Ben Peters (ben@ardusat.com)
 * @author Sam Olds (sam@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements sensor-specific driver read/initialization functions
 *
 * These functions are meant to provide a gateway interface that the higher-level
 * generic interface uses to get data from individual sensors.
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include <Arduino.h>
#include <utility/mlx90614.h>
#include <utility/Adafruit_L3GD20_U.h>
#include <utility/Adafruit_SI1145.h>
#include <utility/Adafruit_TCS34725.h>
#include <utility/BMP180.h>
#include <utility/ML8511_ADC.h>
#include <utility/SparkFunISL29125.h>
#include <utility/TSL2561.h>
#include <utility/common_utils.h>
#include <utility/pololu_LSM303.h>


/**
 * I2C addresses for all of the sensors. Some are only available in the space kit,
 * some are only available on the spaceboard, most are available on both but might
 * have different addresses.
 *
 * NOTE: Some of these definitions are not used because the respective drivers take
 *       care of addressing.
 */
/* Spaceboard Specific Addresses */
#define DRIVER_SPACEBOARD_TSL2561_ADDR  0x49  /* TAOS Lux Light Visible/IR sensor */
#define DRIVER_SPACEBOARD_TCS34725_ADDR 0x29  /* TAOS Color Light Sensor */
#define DRIVER_SPACEBOARD_TMP102_ADDR   0x4B  /* Texas Instrument Temperature */
#define DRIVER_SPACEBOARD_SI1132_ADDR   0x14  /* Ultraviolet Index and Ambient light sensor */
#define DRIVER_SPACEBOARD_ISL29125_ADDR 0x44  /* Digital RGB color sensor */

/* Breakout Kit and Spaceboard Sensor Addresses */
#define DRIVER_MLX90614_ADDR            0x5A  /* Melexis Thermopile single pixel */
#define DRIVER_TSL2561_ADDR             0x39  /* TAOS Lux Light Visible/IR sensor */
//#define DRIVER_TSL2591_ADDR           0x39  /* TAOS Lux High DR Visible/IR sensor (Not available in Kit or Spaceboard) */
#define DRIVER_ML8511_UV_PIN            A0
#define DRIVER_ML8511_REF_PIN           A1
#define DRIVER_ML8511_ADDR              0x51  /* Lapis UV light sensor through LTC2451 */
#define DRIVER_LSM303_ADDR              0x1E  /* ST 3-axis accelerometer & magnetometer */
//#define DRIVER_L3GD20_ADDR            0x6B  /* ST 3-axis digital gyroscope (Already defined by 'L3GD20_ADDRESS') */
#define DRIVER_TMP102_ADDR              0x48  /* Texas Instrument Temperature */
#define DRIVER_BMP180_ADDR              0x77  /* Barometric Pressure (Not available on Spaceboard) */
#define DRIVER_SI1132_ADDR              0x60  /* Ultraviolet Index and Ambient light sensor */


/* Constants */
#define SENSORS_GRAVITY_EARTH           (9.80665F)              /* Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON            (1.6F)                  /* The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN             (275.0F)                /* The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD        (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX      (60.0F)                 /* Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN      (30.0F)                 /* Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA    (1013.25F)              /* Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS             (0.017453293F)          /* Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA     (100)                   /* Gauss to micro-Tesla multiplier */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Adafruit 9DOF IMU breakout board features L3GD20H Gyroscope and LSM303 6-axis 
 * magnetometer + accelerometer.
 *
 * http://www.adafruit.com/product/1714
 * NOTE: Actual configuration values vary depending on the variant of the LSM303 (D or DLHC)
 */
typedef enum {
  LSM303_ACCEL_GAIN2G,
  LSM303_ACCEL_GAIN4G,
  LSM303_ACCEL_GAIN6G,
  LSM303_ACCEL_GAIN8G,
  LSM303_ACCEL_GAIN16G,
} lsm303_accel_gain_e;

typedef enum {
  LSM303_MAG_SCALE1_3GAUSS,
  LSM303_MAG_SCALE2GAUSS,
  LSM303_MAG_SCALE2_5GAUSS,
  LSM303_MAG_SCALE4GAUSS,
  LSM303_MAG_SCALE4_7GAUSS,
  LSM303_MAG_SCALE5_6GAUSS,
  LSM303_MAG_SCALE8GAUSS,
  LSM303_MAG_SCALE12GAUSS,
} lsm303_mag_scale_e;
#define SENSORS_MGAUSS_TO_UTESLA 0.1F

typedef struct {
  lsm303_accel_gain_e gain;
} config_lsm303_accel_t;

typedef struct {
  lsm303_mag_scale_e scale;
} config_lsm303_mag_t;

typedef struct {
  float sensitivity;
} config_l3gd20_t;

void catchSpaceboard();

boolean l3gd20h_init(uint8_t range);
void l3gd20h_getOrientation(float *x, float *y, float *z);
void l3gd20h_getRawAngularRates(int16_t *pX, int16_t *pY, int16_t *pZ);
void l3gd20h_getRawTemperature(int8_t *pRawTemperature);

boolean lsm303_accel_init(lsm303_accel_gain_e gain);
boolean lsm303_mag_init(lsm303_mag_scale_e scale);
void lsm303_getAccel(float * x, float * y, float * z);
void lsm303_getRawAcceleration(int16_t *pX, int16_t *pY, int16_t *pZ);
void lsm303_getRawTemperature(int16_t *pRawTemperature);
void lsm303_getMag(float * x, float * y, float * z);
void lsm303_getRawMag(int16_t *pX, int16_t *pY, int16_t *pZ);

/**
 * BMP180 Barometric altimeter is on Adafruit 10DOF, gives atmospheric pressure/altitude.
 *
 * http://www.adafruit.com/product/1604
 */
boolean bmp180_init(bmp085_mode_t mode);
void bmp180_getRawTemperature(uint16_t *temp);
void bmp180_getTemperature(float *temp);
void bmp180_getRawPressure(uint32_t *pressure);
void bmp180_getPressure(float *pressure);

/**
 * ML8511 breakout board contains an MP8511 UV light sensor
 *
 * https://www.sparkfun.com/products/12705
 */
boolean ml8511_init();
float ml8511_getUV(int pin);

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
boolean tsl2561_init(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain);
float tsl2561_getLux();

/**
 * ISL29125 RGB Sensor
 *
 * http://www.sparkfun.com/products/12829
 */
boolean isl29125_init(uint8_t intensity);
void isl29125_getRGB(float * red, float * green, float * blue);

/**
 * TCS34725 RGB Sensor
 *
 * https://learn.adafruit.com/adafruit-color-sensors
 */
boolean tcs34725_init(tcs34725IntegrationTime_t it, tcs34725Gain_t gain);
void tcs34725_getRGB(float * red, float * green, float * blue);

/**
 * SI1132 UV/Light sensor uses the SI1145 driver provided by Adafruit.
 *
 * https://www.silabs.com/Support%20Documents/TechnicalDocs/Si1132.pdf
 * https://learn.adafruit.com/adafruit-si1145-breakout-board-uv-ir-visible-sensor/overview
 */
boolean si1132_init();
float si1132_getUVIndex();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
