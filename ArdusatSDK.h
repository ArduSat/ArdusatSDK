/**
 * @file   ArdusatSDK.h
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor reading and configuration for Space Kit Sensors.
 */
#ifndef ARDUSATSDK_H_
#define ARDUSATSDK_H_

#include <Arduino.h>
#include <utility/drivers.h>
#include <avr/pgmspace.h>
#include <utility/serial.h>

typedef char PROGMEM prog_char;

extern bool ARDUSAT_SHIELD;

/**
 * Unique numeric id for each physical sensor
 */
typedef enum {
	SENSORID_NULL = 0x00,	// no sensor, or unreliable
	SENSORID_TMP102	= 0x01,
	SENSORID_TSL2561 = 0x02,
	SENSORID_MLX90614 = 0x03,
	SENSORID_ADAFRUIT9DOFIMU = 0x04,
	SENSORID_SI1145	= 0x05,
	SENSORID_ML8511	= 0x06,
	SENSORID_BMP180 = 0x07
} sensor_id_t;

/**
 * Unit definitions
 */
typedef enum {
  DATA_UNIT_NONE = 0,
  DATA_UNIT_METER_PER_SECONDSQUARED = 1,
  DATA_UNIT_RADIAN_PER_SECOND = 2,
  DATA_UNIT_MICROTESLA = 3,
  DATA_UNIT_DEGREES_CELSIUS = 4,
  DATA_UNIT_DEGREES_FAHRENHEIT = 5,
  DATA_UNIT_METER_PER_SECOND = 6,
  DATA_UNIT_LUX	= 7,
  DATA_UNIT_RADIAN = 8,
  DATA_UNIT_MILLIWATT_PER_CMSQUARED = 9,
  DATA_UNIT_DEGREES = 10,
  DATA_UNIT_HECTOPASCAL	= 11
} data_unit_t;

/**
 * The data header contains generic information about the data record.
 */
struct _data_header_v1 {
	unsigned char unit;       // unit (standard) of the values (e.g. meter, m/s^2, etc.)
	unsigned char sensor_id;  // id of the sensor that generated this data
	unsigned long timestamp;  // millis for timestamping the data
};

typedef struct _data_header_v1 _data_header_t;

/**
 * Data types are used to store/read sensor data according to the appropriate value type
 */
typedef struct {
	_data_header_t header;
	float x,y,z;
} acceleration_t;

typedef struct {
	_data_header_t header;
	float x,y,z;
} magnetic_t;

typedef struct {
	_data_header_t header;
	float x,y,z;
} gyro_t;

typedef struct {
	_data_header_t header;
	float t;
} temperature_t;

typedef struct {
	_data_header_t header;
	float lux;
} luminosity_t;

typedef struct {
	_data_header_t header;
	float uvindex;
} uvlight_t;

typedef struct {
	_data_header_t header;
	float roll, pitch, heading;
} orientation_t;

typedef struct {
	_data_header_t header;
	float pressure;
} pressure_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Where all of the sensor data is kept before being printed
 */
extern int OUTPUT_BUF_SIZE;
extern char * _output_buffer;
char * _getOutBuf();
void _resetOutBuf();

/**
 * Get a string representation of a unit constant
 */
const char * unit_to_str(unsigned char);

/**
 * setup and read functions are used to initialize sensors and read data from them.
 */
//TODO: expand setup functions with options
boolean beginTemperatureSensor();
void readTemperature(temperature_t & temp);

boolean beginInfraredTemperatureSensor();
void readInfraredTemperature(temperature_t & temp);

boolean beginLuminositySensor();
void readLuminosity(luminosity_t & lum);

boolean beginUVLightSensor();
void readUVLight(uvlight_t & uv, int pin=DRIVER_ML8511_UV_PIN);

boolean beginAccelerationSensor();
void readAcceleration(acceleration_t & accel);

boolean beginGyroSensor();
void readGyro(gyro_t & orient);

boolean beginMagneticSensor();
void readMagnetic(magnetic_t & mag);

boolean beginBarometricPressureSensor();
void readBarometricPressure(pressure_t & pressure);

void calculateOrientation(const acceleration_t & accel, const magnetic_t & mag, orientation_t & orient);

int calculateCheckSum(const char *sensorName, int numValues, ...);

/**
 * toCSV output functions create a string representation of the data in CSV format.
 */
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, ...);
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp=0);
const char * accelerationToCSV(const char *sensorName, acceleration_t & input);
const char * magneticToCSV(const char *sensorName, magnetic_t & input);
const char * gyroToCSV(const char *sensorName, gyro_t & input);
const char * temperatureToCSV(const char *sensorName, temperature_t & input);
const char * luminosityToCSV(const char *sensorName, luminosity_t & input);
const char * uvlightToCSV(const char *sensorName, uvlight_t & input);
const char * orientationToCSV(const char *sensorName, orientation_t & input);
const char * pressureToCSV(const char *sensorName, pressure_t & pressure);

/**
 * toJSON output functions create a string representation of the data in a JSON format
 * that can be used with http://experiments.ardusat.com to visualize and log data.
 *
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valueToJSON(const char *sensorName, unsigned char unit, float value);
const char * accelerationToJSON(const char *sensorName, acceleration_t & input);
const char * magneticToJSON(const char *sensorName, magnetic_t & input);
const char * temperatureToJSON(const char *sensorName, temperature_t & input);
const char * gyroToJSON(const char *sensorName, gyro_t & input);
const char * luminosityToJSON(const char *sensorName, luminosity_t & input);
const char * uvlightToJSON(const char *sensorName, uvlight_t & input);
const char * orientationToJSON(const char *sensorName, orientation_t & input);
const char * pressureToJSON(const char *sensorName, pressure_t & input);

#ifdef __cplusplus
} // extern "C"
#endif

// versions of functions using the default names for each sensor to save memory
const char * accelerationToCSV( acceleration_t & input );
const char * magneticToCSV( magnetic_t & input );
const char * gyroToCSV( gyro_t & input );
const char * temperatureToCSV( temperature_t & input );
const char * luminosityToCSV( luminosity_t & input );
const char * uvlightToCSV( uvlight_t & input );
const char * orientationToCSV( orientation_t & input );
const char * pressureToCSV( pressure_t & pressure );


#endif /* ARDUSATSDK_H_ */
