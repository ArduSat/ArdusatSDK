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

#ifdef __cplusplus
extern "C" {
#endif

extern bool MANUAL_CONFIG;
extern bool ARDUSAT_SPACEBOARD;

/**
 * Unique numeric id for each physical sensor
 */
typedef enum {
	SENSORID_NULL = 0x00,	// no sensor, or unreliable
	SENSORID_TMP102	= 0x01,
	SENSORID_TSL2561 = 0x02,
	SENSORID_MLX90614 = 0x03,
	SENSORID_ADAFRUIT9DOFIMU = 0x04,
	SENSORID_SI1132	= 0x05,
	SENSORID_ML8511	= 0x06,
	SENSORID_BMP180 = 0x07,
	SENSORID_ISL29125 = 0x08,
	SENSORID_TCS34725 = 0x09,
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
 * Get a string representation of a unit constant
 */
const char * unit_to_str(unsigned char);

/**
 * toCSV output functions create a string representation of the data in CSV format.
 */
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, ...);
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp=0);

/**
 * toJSON output functions create a string representation of the data in a JSON format
 * that can be used with http://experiments.ardusat.com to visualize and log data.
 *
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valueToJSON(const char *sensorName, unsigned char unit, float value);

#ifdef __cplusplus
} // extern "C"
#endif

/**
 * ========================================================================================
 * Acceleration Sensor Class
 * ========================================================================================
 */
class Acceleration {
  private:
    sensor_id_t sensorId;
    lsm303_accel_gain_e gGain = LSM303_ACCEL_GAIN8G;

  public:
    _data_header_t header;
    boolean initialized = false;
    float x;
    float y;
    float z;

    Acceleration(void);
    ~Acceleration();

    boolean begin(void);
    boolean begin(lsm303_accel_gain_e gGain);

    void read(void);
    const char * readToCSV(const char * sensorName = "Acceleration");
    const char * readToJSON(const char * sensorName = "Acceleration");
    const char * toCSV(const char * sensorName = "Acceleration");
    const char * toJSON(const char * sensorName = "Acceleration");
};

/**
 * ========================================================================================
 * Gyroscope Sensor Class
 * ========================================================================================
 */
class Gyro {
  private:
    sensor_id_t sensorId;
    uint8_t range = 0x20;

  public:
    _data_header_t header;
    boolean initialized = false;
    float x;
    float y;
    float z;

    Gyro(void);
    ~Gyro();

    boolean begin(void);
    boolean begin(uint8_t range);

    void read(void);
    const char * readToCSV(const char * sensorName = "Gyro");
    const char * readToJSON(const char * sensorName = "Gyro");
    const char * toCSV(const char * sensorName = "Gyro");
    const char * toJSON(const char * sensorName = "Gyro");
};

/**
 * ========================================================================================
 * Luminosity Sensor Class
 * ========================================================================================
 */
class Luminosity {
  private:
    sensor_id_t sensorId;
    tsl2561IntegrationTime_t intTime = TSL2561_INTEGRATIONTIME_13MS;
    tsl2561Gain_t gain = TSL2561_GAIN_16X;

  public:
    _data_header_t header;
    boolean initialized = false;
    float lux;

    Luminosity(void);
    ~Luminosity();

    boolean begin(void);
    boolean begin(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain);

    void read(void);
    const char * readToCSV(const char * sensorName = "Luminosity");
    const char * readToJSON(const char * sensorName = "Luminosity");
    const char * toCSV(const char * sensorName = "Luminosity");
    const char * toJSON(const char * sensorName = "Luminosity");
};

/**
 * ========================================================================================
 * Magnetic Sensor Class
 * ========================================================================================
 */
class Magnetic {
  private:
    sensor_id_t sensorId;
    lsm303_mag_scale_e gaussScale = LSM303_MAG_SCALE4GAUSS;

  public:
    _data_header_t header;
    boolean initialized = false;
    float x;
    float y;
    float z;

    Magnetic(void);
    ~Magnetic();

    boolean begin(void);
    boolean begin(lsm303_mag_scale_e gaussScale);

    void read(void);
    const char * readToCSV(const char * sensorName = "Magnetic");
    const char * readToJSON(const char * sensorName = "Magnetic");
    const char * toCSV(const char * sensorName = "Magnetic");
    const char * toJSON(const char * sensorName = "Magnetic");
};

/**
 * ========================================================================================
 * Orientation Class
 * ========================================================================================
 */
class Orientation {
  private:
    sensor_id_t sensorId;
    Acceleration * accel;
    Magnetic * mag;

  public:
    _data_header_t header;
    boolean initialized = false;
    float roll;
    float pitch;
    float heading;

    Orientation(void);
    ~Orientation();

    boolean begin(void);
    boolean begin(Acceleration & accel, Magnetic & mag);

    void read(void);
    void read(Acceleration & accel, Magnetic & mag);

    const char * readToCSV(const char * sensorName = "Orientation");
    const char * readToJSON(const char * sensorName = "Orientation");
    const char * readToCSV(Acceleration & accel, Magnetic & mag, const char * sensorName = "Orientation");
    const char * readToJSON(Acceleration & accel, Magnetic & mag, const char * sensorName = "Orientation");
    const char * toCSV(const char * sensorName = "Orientation");
    const char * toJSON(const char * sensorName = "Orientation");
};

/**
 * ========================================================================================
 * Pressure Sensor Class
 * ========================================================================================
 */
class Pressure {
  private:
    sensor_id_t sensorId;
    bmp085_mode_t bmp085_mode = BMP085_MODE_ULTRAHIGHRES;

  public:
    _data_header_t header;
    boolean initialized = false;
    float pressure;

    Pressure(void);
    ~Pressure();

    boolean begin(void);
    boolean begin(bmp085_mode_t mode);

    void read(void);
    float altitudeFromSeaLevelPressure(float seaLevelPressure);
    float seaLevelPressureFromAltitude(float altitude);

    const char * readToCSV(const char * sensorName = "Pressure");
    const char * readToJSON(const char * sensorName = "Pressure");
    const char * toCSV(const char * sensorName = "Pressure");
    const char * toJSON(const char * sensorName = "Pressure");
};

/**
 * ========================================================================================
 * RGB Light Sensor Class
 * ========================================================================================
 */
class RGBLight {
  private:
    sensor_id_t sensorId;
    uint8_t islIntensity = CFG1_10KLUX;
    tcs34725IntegrationTime_t tcsIt = TCS34725_INTEGRATIONTIME_154MS;
    tcs34725Gain_t tcsGain = TCS34725_GAIN_1X;

  public:
    _data_header_t header;
    boolean initialized = false;
    float red;
    float green;
    float blue;

    // Defaults to TCS34725 instead of ISL29125
    RGBLight(sensor_id_t sensor_id=SENSORID_TCS34725);
    ~RGBLight();

    boolean begin(void);
    boolean begin(uint8_t islIntensity);
    boolean begin(tcs34725IntegrationTime_t it, tcs34725Gain_t gain);
    boolean begin(uint8_t islIntensity, tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain);

    void read(void);
    const char * readToCSV(const char * sensorName = "RGBLight");
    const char * readToJSON(const char * sensorName = "RGBLight");
    const char * toCSV(const char * sensorName = "RGBLight");
    const char * toJSON(const char * sensorName = "RGBLight");
};

/**
 * ========================================================================================
 * Temperature Sensor Class
 * ========================================================================================
 */
class Temperature {
  private:
    sensor_id_t sensorId;

  public:
    _data_header_t header;
    boolean initialized = false;
    float t;

    // Defaults to ambient TMP102 instead of MLX90614 for IR
    Temperature(sensor_id_t sensor_id=SENSORID_TMP102);
    ~Temperature();

    boolean begin(void);

    void read(void);
    const char * readToCSV(const char * sensorName = "Temperature");
    const char * readToJSON(const char * sensorName = "Temperature");
    const char * toCSV(const char * sensorName = "Temperature");
    const char * toJSON(const char * sensorName = "Temperature");
};

/**
 * ========================================================================================
 * UV Light Sensor Class
 * ========================================================================================
 */
class UVLight {
  private:
    sensor_id_t sensorId;
    int ML8511_pin = DRIVER_ML8511_UV_PIN;

  public:
    _data_header_t header;
    boolean initialized = false;
    float uvindex;

    // Defaults to ML8511 instead of SI1132
    UVLight(sensor_id_t sensor_id=SENSORID_ML8511);
    ~UVLight();

    boolean begin(void);
    boolean begin(int pin);

    void read(void);
    const char * readToCSV(const char * sensorName = "UVLight");
    const char * readToJSON(const char * sensorName = "UVLight");
    const char * toCSV(const char * sensorName = "UVLight");
    const char * toJSON(const char * sensorName = "UVLight");
};

#endif /* ARDUSATSDK_H_ */
