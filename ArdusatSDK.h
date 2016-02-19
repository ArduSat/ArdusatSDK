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

/*
 * Base Sensor class to be inherited by all other sensors
 * ========================================================================================
 */
class Sensor {
  protected:
    sensor_id_t sensorId;

  public:
    _data_header_t header;

    virtual boolean begin(void) = 0;
    virtual void read(void) = 0;
    virtual const char * readToCSV(const char * sensorName) = 0;
    virtual const char * readToJSON(const char * sensorName) = 0;
    virtual const char * toCSV(const char * sensorName) = 0;
    virtual const char * toJSON(const char * sensorName) = 0;
};

/*
 * UV Light
 * ========================================================================================
 */
class UVLight: public Sensor {
  private:
    int ML8511_pin = DRIVER_ML8511_UV_PIN;

  public:
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

/*
 * RGB Light
 * ========================================================================================
 */
class RGBLight: public Sensor {
  private:

  public:
    float red;
    float green;
    float blue;

    // Defaults to TCS34725 instead of ISL29125
    RGBLight(sensor_id_t sensor_id=SENSORID_TCS34725);
    ~RGBLight();
    boolean begin(void);
    //boolean begin(/* TODO: advanced configs here! */);

    void read(void);
    const char * readToCSV(const char * sensorName = "RGBLight");
    const char * readToJSON(const char * sensorName = "RGBLight");
    const char * toCSV(const char * sensorName = "RGBLight");
    const char * toJSON(const char * sensorName = "RGBLight");
};

/*
 * Temperature
 * ========================================================================================
 */
class Temperature: public Sensor {
  private:

  public:
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

/*
 * Luminosity
 * ========================================================================================
 */
class Luminosity: public Sensor {
  private:

  public:
    float lux;

    Luminosity(void);
    ~Luminosity();
    boolean begin(void);

    void read(void);
    const char * readToCSV(const char * sensorName = "Luminosity");
    const char * readToJSON(const char * sensorName = "Luminosity");
    const char * toCSV(const char * sensorName = "Luminosity");
    const char * toJSON(const char * sensorName = "Luminosity");
};

/*
 * Pressure
 * ========================================================================================
 */
class Pressure: public Sensor {
  private:
    bmp085_mode_t bmp085_mode=BMP085_MODE_ULTRAHIGHRES;

  public:
    float pressure;

    Pressure(void);
    ~Pressure();
    boolean begin(void);
    boolean begin(bmp085_mode_t mode);

    void read(void);
    const char * readToCSV(const char * sensorName = "Pressure");
    const char * readToJSON(const char * sensorName = "Pressure");
    const char * toCSV(const char * sensorName = "Pressure");
    const char * toJSON(const char * sensorName = "Pressure");
};

/*
 * Acceleration
 * ========================================================================================
 */
class Acceleration: public Sensor {
  private:

  public:
    float x;
    float y;
    float z;

    Acceleration(void); // ------ Advanced Configuration?
    ~Acceleration();
    boolean begin(void); // ------ Advanced Configuration?

    void read(void);
    const char * readToCSV(const char * sensorName = "Acceleration");
    const char * readToJSON(const char * sensorName = "Acceleration");
    const char * toCSV(const char * sensorName = "Acceleration");
    const char * toJSON(const char * sensorName = "Acceleration");
};

/*
 * Magnetic
 * ========================================================================================
 */
class Magnetic: public Sensor {
  private:

  public:
    float x;
    float y;
    float z;

    Magnetic(void); // ------ Advanced Configuration?
    ~Magnetic();
    boolean begin(void); // ------ Advanced Configuration?

    void read(void);
    const char * readToCSV(const char * sensorName = "Magnetic");
    const char * readToJSON(const char * sensorName = "Magnetic");
    const char * toCSV(const char * sensorName = "Magnetic");
    const char * toJSON(const char * sensorName = "Magnetic");
};

/*
 * Gyro
 * ========================================================================================
 */
class Gyro: public Sensor {
  private:

  public:
    float x;
    float y;
    float z;

    Gyro(void); // ------ Advanced Configuration?
    ~Gyro();
    boolean begin(void); // ------ Advanced Configuration?

    void read(void);
    const char * readToCSV(const char * sensorName = "Gyro");
    const char * readToJSON(const char * sensorName = "Gyro");
    const char * toCSV(const char * sensorName = "Gyro");
    const char * toJSON(const char * sensorName = "Gyro");
};

/*
 * Orientation
 * ========================================================================================
 */
class Orientation: public Sensor {
  private:
    Acceleration accel;
    Magnetic mag;

  public:
    float roll;
    float pitch;
    float heading;

    Orientation(void); // ------ Advanced Configuration?
    ~Orientation();
    boolean begin(void); // ------ Advanced Configuration?
    boolean begin(Acceleration & accel, Magnetic & mag); // ------ Advanced Configuration?

    void read(void);
    void read(Acceleration & accel, Magnetic & mag);

    const char * readToCSV(const char * sensorName = "Orientation");
    const char * readToJSON(const char * sensorName = "Orientation");
    const char * readToCSV(Acceleration & accel, Magnetic & mag, const char * sensorName = "Orientation");
    const char * readToJSON(Acceleration & accel, Magnetic & mag, const char * sensorName = "Orientation");
    const char * toCSV(const char * sensorName = "Orientation");
    const char * toJSON(const char * sensorName = "Orientation");
};

#endif /* ARDUSATSDK_H_ */
