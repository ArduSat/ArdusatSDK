/**
 * @file   ArdusatSDK.h
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  ArdusatSDK generic sensor reading & configuration for Space Kit Sensors
 */
#ifndef ARDUSATSDK_H_
#define ARDUSATSDK_H_

#include <Arduino.h>

#include <utility/drivers.h>

/**
 * Unique numeric id for each physical sensor
 */
#define SENSORID_NULL			0x00	// no sensor, or unreliable
#define SENSORID_TMP102			0x01
#define SENSORID_TSL2561		0x02
#define SENSORID_MLX90614		0x03
#define SENSORID_ADAFRUIT9DOFIMU	0x04
#define SENSORID_SI1145			0x05
#define SENSORID_ML8511 		0x06 

#define OUTPUT_BUFFER_MAXSIZE 256

/**
 * Unit definitions
 */
#define DATA_UNIT_NONE	0
#define DATA_UNIT_METER_PER_SECONDSQUARED	1	//haccelerometer
#define DATA_UNIT_RADIAN_PER_SECOND		2	// gyroscope
#define DATA_UNIT_MICROTESLA			3	// magnetometer
#define DATA_UNIT_DEGREES_CELSIUS		4	// temperature
#define DATA_UNIT_DEGREES_FAHRENHEIT		5	// temperature
#define DATA_UNIT_METER_PER_SECOND		6	// speed, somehow
#define DATA_UNIT_LUX					7	// luminosity
#define DATA_UNIT_RADIAN				8	// IMU
#define DATA_UNIT_MILLIWATT_PER_CMSQUARED 9

/**
 * Data types for cells in data structures.
 */
// schema for these constants is 			0000XXLL XX for type, LL for len-1
#define DATA_CELLTYPE_HEX	0x00
#define DATA_CELLTYPE_HEX8		0x00	//	00000000
#define DATA_CELLTYPE_HEX16		0x01	//	00000001
#define DATA_CELLTYPE_HEX24		0x02	//	00000010
#define DATA_CELLTYPE_HEX32		0x03	//	00000011

#define DATA_CELLTYPE_INT	0x04
#define DATA_CELLTYPE_INT8		0x04	//	00000100
#define DATA_CELLTYPE_INT16		0x05	//	00000101
#define DATA_CELLTYPE_INT24		0x06	//	00000110
#define DATA_CELLTYPE_INT32		0x07	//	00000111

#define DATA_CELLTYPE_UINT	0x08
#define DATA_CELLTYPE_UINT8		0x08	//	00001000
#define DATA_CELLTYPE_UINT16	0x09	//	00001001
#define DATA_CELLTYPE_UINT24	0x0A	//	00001010
#define DATA_CELLTYPE_UINT32	0x0B	//	00001011

//#define DATA_CELLTYPE_STR		0x0D	//	00001101 EXCEPTION TO THE SCHEMA
#define DATA_CELLTYPE_FLOAT		0x0F	//  00001111

// utility constants
#define DATA_CELLTYPE_LONGINT	0x07	// (INT4)
#define DATA_CELLTYPE_SHORTINT	0x05	// (INT2)
#define DATA_CELLTYPE_INTEGER	0x05	// (SHORTINT) (INT2)
#define DATA_CELLTYPE_ULONGINT	0x0B	// (UINT4)
#define DATA_CELLTYPE_USHORTINT	0x09	// (UINT2)
#define DATA_CELLTYPE_BYTE		0x00	// (HEX1)


/**
 * The data header contains generic information about the data record.
 */
#define SENSORDATA_HEADER_VERSION	 1
struct _data_header_v1 {
	uint8_t version;		// version of the data packet (for compatibility mgt)
	uint8_t length;			// len of the data packet (256 is enough ?)
	uint8_t dimensionality;	// size of the cell array
	uint8_t celltype;		// type of the cell (0 = byte, 1 = float, etc…)
	uint8_t unit;			// unit (standard) of the values (e.g. meter, m/s^2, etc.)
	uint8_t sensor_id;		// id of the sensor that generated this data
	uint32_t timestamp;		// millis for timestamping the data
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
	float roll,pitch,heading;
} orientation_t;

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

#ifdef __cplusplus
extern "C" {
#endif
	
/**
 * Get a string representation of a unit constant
 */
const char * unit_to_str(uint8_t);

/**
 * setup and read functions are used to initialize sensors and read data from them.
 */
//TODO: expand setup functions with options
boolean beginTemperatureSensor(); 
void readTemperature(temperature_t * temp);

boolean beginInfraredTemperatureSensor();
void readInfraredTemperature(temperature_t * temp);

boolean beginLuminositySensor();
void readLuminosity(luminosity_t * lum);

boolean beginUVLightSensor();
void readUVLight(uvlight_t * uv);

boolean beginAccelerationSensor();
void readAcceleration(acceleration_t * accel);

boolean beginOrientationSensor();
void readOrientation(orientation_t * orient);

boolean beginMagneticSensor();
void readMagnetic(magnetic_t * mag);

/**
 * toCSV output functions create a string representation of the data in CSV format.
 */
const char * accelerationToCSV(acceleration_t * input);
const char * magneticToCSV(magnetic_t *input);
const char * temperatureToCSV(temperature_t * input);
const char * orientationToCSV(orientation_t * input);
const char * luminosityToCSV(luminosity_t * input);
const char * uvlightToCSV(uvlight_t * input);

/**
 * toJSON output functions create a string representation of the data in a JSON format
 * that can be used with http://demo.ardusat.com to visualize and log data. 
 * 
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valueToJSON(const char *sensor_name, uint8_t unit, float value);
const char * accelerationToJSON(const char *sensor_name, acceleration_t * input);
const char * magneticToJSON(const char *sensor_name, magnetic_t * input);
const char * temperatureToJSON(const char *sensor_name, temperature_t * input);
const char * orientationToJSON(const char *sensor_name, orientation_t * input);
const char * luminosityToJSON(const char *sensor_name, luminosity_t * input);
const char * uvlightToJSON(const char *sensor_name, uvlight_t * input);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SENSORSDK_H_ */
