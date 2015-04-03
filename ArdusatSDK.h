/**
 * @file   ArdusatSDK.h
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  ArdusatSDK generic sensor reading & configuration for Space Kit Sensors
 */
#ifndef ARDUSATSDK_H_
#define ARDUSATSDK_H_

#include <Arduino.h>
#include <utility/SdFat.h>
#include <utility/drivers.h>
#include <avr/pgmspace.h>
#include <utility/SdVolume.h>
#include <utility/MemoryFree.h>
#include <utility/BinaryDataFmt.h>
#include <utility/serial.h>
#include <utility/RTClib.h>

#define prog_char const char PROGMEM

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

#define OUTPUT_BUFFER_MAXSIZE 500

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
	//uint8_t version;		// version of the data packet (for compatibility mgt)
	//uint8_t length;			// len of the data packet (256 is enough ?)
	//uint8_t dimensionality;	// size of the cell array
	//uint8_t celltype;		// type of the cell (0 = byte, 1 = float, etc…)
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
void readUVLight(uvlight_t * uv, int pin=DRIVER_ML8511_UV_PIN);

boolean beginAccelerationSensor();
void readAcceleration(acceleration_t * accel);

boolean beginGyroSensor();
void readGyro(gyro_t * orient);

boolean beginMagneticSensor();
void readMagnetic(magnetic_t * mag);

boolean beginBarometricPressureSensor();
void readBarometricPressure(pressure_t *pressure);

void calculateOrientation(const acceleration_t *accel, const magnetic_t *mag,
													orientation_t *orient);

/**
 * toCSV output functions create a string representation of the data in CSV format.
 */
const char * valuesToCSV(const char *sensorName, float values[], int numValues, unsigned long timestamp=0);
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp=0);
const char * accelerationToCSV(const char *sensorName, acceleration_t * input);
const char * magneticToCSV(const char *sensorName, magnetic_t *input);
const char * gyroToCSV(const char *sensorName, gyro_t * input);
const char * temperatureToCSV(const char *sensorName, temperature_t * input);
const char * luminosityToCSV(const char *sensorName, luminosity_t * input);
const char * uvlightToCSV(const char *sensorName, uvlight_t * input);
const char * orientationToCSV(const char *sensorName, orientation_t *input);
const char * pressureToCSV(const char *sensorName, pressure_t *pressure);

/**
 * toJSON output functions create a string representation of the data in a JSON format
 * that can be used with http://experiments.ardusat.com to visualize and log data. 
 * 
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valueToJSON(const char *sensorName, uint8_t unit, float value);
const char * accelerationToJSON(const char *sensorName, acceleration_t * input);
const char * magneticToJSON(const char *sensorName, magnetic_t * input);
const char * temperatureToJSON(const char *sensorName, temperature_t * input);
const char * gyroToJSON(const char *sensorName, gyro_t * input);
const char * luminosityToJSON(const char *sensorName, luminosity_t * input);
const char * uvlightToJSON(const char *sensorName, uvlight_t * input);
const char * orientationToJSON(const char *sensorName, orientation_t * input);
const char * pressureToJSON(const char *sensorName, pressure_t *input);

/**
 * Write functions take care of persisting data to an SD card
 *
 * Data can be written in any format, including a custom binary format for 
 * space efficiency, by using the writeBytes function that takes a byte array.
 *
 * The individual helpers write functions automate the process of saving data
 * for each individual value, but write in CSV format, which is easier to consume
 * after the experiment, but takes up more space.
 *
 * Finally, binaryWrite functions write optimized binary representations of the 
 * data. This helps save SD card space, but means that the data must be decoded
 * after download before it can be used.
 */
bool beginDataLog(int chipSelectPin, const char *fileNamePrefix, bool csvData);

int writeBytes(const uint8_t *buffer, uint8_t numBytes);
int writeString(const char *output);
int writeAcceleration(const char *sensorName, acceleration_t *data);
int writeMagnetic(const char *sensorName, magnetic_t *data);
int writeGyro(const char *sensorName, gyro_t *data);
int writeTemperature(const char *sensorName, temperature_t *data);
int writeLuminosity(const char *sensorName, luminosity_t *data);
int writeUVLight(const char *sensorName, uvlight_t *data);
int writeOrientation(const char *sensorName, orientation_t *data);
int writePressure(const char *sensorName, pressure_t *data);

int binaryWriteAcceleration(const uint8_t sensorId, acceleration_t *data);
int binaryWriteMagnetic(const uint8_t sensorId, magnetic_t *data);
int binaryWriteGyro(const uint8_t sensorId, gyro_t *data);
int binaryWriteTemperature(const uint8_t sensorId, temperature_t *data);
int binaryWriteLuminosity(const uint8_t sensorId, luminosity_t *data);
int binaryWriteUVLight(const uint8_t sensorId, uvlight_t *data);
int binaryWriteOrientation(const uint8_t sensorId, orientation_t *data);
int binaryWritePressure(const uint8_t sensorId, pressure_t *data);

/**
 * Setup and use the RTC chip, if found
 *
 * The clock must be set before it can be used. This is achieved by compiling
 * and running a basic script (examples/set_rtc). After this, the library 
 * automatically uses the RTC if available to timestamp.
 */
bool setRTC();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SENSORSDK_H_ */
