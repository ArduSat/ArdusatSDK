/**
 * @file    ArdusatSDK.h
 * @author  Ben Peters (ben@ardusat.com)
 * @author  Sam Olds (sam@ardusat.com)
 * @date    December 3, 2014
 * @brief   Implements ArdusatSDK generic sensor reading and configuration for Space Kit Sensors.
 *
 * Provides a unifying wrapper of sensor specific functionality to provide a
 * consistent interface to interact with each type of sensor.
 */

#ifndef ARDUSATSDK_H_
#define ARDUSATSDK_H_

#include <Arduino.h>
#include <utility/drivers.h>
#include <avr/pgmspace.h>
#include <utility/serial.h>

/**
 * Allows the user to manually decide in an Arduino sketch if the SDK should
 * dynamically check if the SpaceBoard is being used or not.
 * Defaults to false
 */
extern boolean MANUAL_CONFIG;

/**
 * Used when dynamically checking if the SpaceBoard is being used or not. If
 * the SpaceBoard is being used, different addresses might be used for each
 * sensor.
 */
extern boolean ARDUSAT_SPACEBOARD;

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
 * Where all of the sensor data is kept before being printed
 */
extern int OUTPUT_BUF_SIZE;
extern char * _output_buffer;
char * _getOutBuf();
void _resetOutBuf();

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
const char * unit_to_str(unsigned char unit);

/**
 * creates a string representation of the data in a CSV format that can be used with
 * http://experiments.ardusat.com to visualize and log data.
 */
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, ...);
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp=0);

/**
 * creates a string representation of the data in a JSON format that can be used with
 * http://experiments.ardusat.com to visualize and log data.
 *
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valuesToJSON(const char *sensorName, unsigned char unit, int numValues, ...);
const char * valueToJSON(const char *sensorName, unsigned char unit, float value);


/**************************************************************************//**
 * @class Sensor
 * @defgroup sensor
 * @brief Base Sensor class that all Sensors inherit from
 *****************************************************************************/
class Sensor {
  protected:
    void initializeHeader(sensor_id_t sensor_id, data_unit_t unit, const char name[] PROGMEM);

    virtual boolean initialize(void) = 0;
    virtual boolean readSensor(void) = 0;

  public:
    const char * name;
    _data_header_t header;
    boolean initialized;

    boolean begin(void);
    boolean read(void);
    const char * readToCSV(const char * sensorName);
    const char * readToJSON(const char * sensorName);

    virtual const char * toCSV(const char * sensorName) = 0;
    virtual const char * toJSON(const char * sensorName) = 0;
};


/**************************************************************************//**
 * @class Acceleration
 * @ingroup sensor
 *
 * @defgroup acceleration
 * @brief Encapsulates all functionality related to the Acceleration Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the LSM303 on the Adafruit 9 Degrees of Freedom
 * Board.
 *
 * Example Usage:
 * @code
 *     Acceleration accel;                        // Instantiate sensor object
 *     accel.begin();                             // Initialize sensor
 *     Serial.println(accel.readToJSON("accel")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Acceleration: public Sensor {
  protected:
    lsm303_accel_gain_e gGain;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float x;
    float y;
    float z;
    Acceleration(void);
    Acceleration(lsm303_accel_gain_e gain);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class Gyro
 * @ingroup sensor
 *
 * @defgroup gyro
 * @brief Encapsulates all functionality related to the Gyro Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the L3GD20 on the Adafruit 9 Degrees of Freedom
 * Board.
 *
 * Example Usage:
 * @code
 *     Gyro gyro;                               // Instantiate sensor object
 *     gyro.begin();                            // Initialize sensor
 *     Serial.println(gyro.readToJSON("gyro")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Gyro: public Sensor {
  protected:
    uint8_t range;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float x;
    float y;
    float z;
    Gyro(void);
    Gyro(uint8_t range);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class Luminosity
 * @ingroup sensor
 *
 * @defgroup luminosity
 * @brief Encapsulates all functionality related to the Luminosity Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the TSL2561 board
 *
 * Example Usage:
 * @code
 *     Luminosity lum;                        // Instantiate sensor object
 *     lum.begin();                           // Initialize sensor
 *     Serial.println(lum.readToJSON("lum")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Luminosity: public Sensor {
  protected:
    tsl2561Gain_t gain;
    tsl2561IntegrationTime_t intTime;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float lux;
    Luminosity(void);
    Luminosity(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain);
    Luminosity(tsl2561Gain_t gain, tsl2561IntegrationTime_t intTime);
    Luminosity(tsl2561IntegrationTime_t intTime);
    Luminosity(tsl2561Gain_t gain);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class Magnetic
 * @ingroup sensor
 *
 * @defgroup magnetic
 * @brief Encapsulates all functionality related to the Magnetic Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the LSM303 on the Adafruit 9 Degrees of Freedom
 * Board.
 *
 * Example Usage:
 * @code
 *     Magnetic mag;                          // Instantiate sensor object
 *     mag.begin();                           // Initialize sensor
 *     Serial.println(mag.readToJSON("mag")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Magnetic: public Sensor {
  protected:
    lsm303_mag_scale_e gaussScale;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float x;
    float y;
    float z;
    Magnetic(void);
    Magnetic(lsm303_mag_scale_e gaussScale);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class Orientation
 * @ingroup sensor
 *
 * @defgroup orientation
 * @brief Encapsulates all functionality related to the Orientation Calculation
 *
 * This class can be used to initialize, read, and print data derived from the
 * Accelerometer and Magnetometer Sensors
 *
 * Example Usage:
 * @code
 *     Orientation orient;                               // Instantiate sensor object
 *     orient.begin();                                   // Initialize sensor
 *     Serial.println(orient.readToJSON("orientation")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Orientation: public Sensor {
  protected:
    Acceleration * accel;
    Magnetic * mag;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float roll;
    float pitch;
    float heading;
    Orientation(Acceleration & accel, Magnetic & mag);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class Pressure
 * @ingroup sensor
 *
 * @defgroup pressure
 * @brief Encapsulates all functionality related to the Pressure Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the BMP180 on the Adafruit 9 Degrees of Freedom
 * Board.
 *
 * @note This Sensor is not available on the SpaceBoard
 *
 * Example Usage:
 * @code
 *     Pressure press;                               // Instantiate sensor object
 *     press.begin();                                // Initialize sensor
 *     Serial.println(press.readToJSON("pressure")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Pressure: public Sensor {
  protected:
    bmp085_mode_t bmp085_mode;

    boolean initialize(void);
    boolean readSensor(void);

  public:
    float pressure;
    Pressure(void);
    Pressure(bmp085_mode_t mode);

    float altitudeFromSeaLevelPressure(float seaLevelPressure);
    float seaLevelPressureFromAltitude(float altitude);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};


/**************************************************************************//**
 * @class RGBLight
 * @ingroup sensor
 *
 * @defgroup rgblight
 * @brief Encapsulates all functionality related to the RGBLight Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from the either the TCS34725 board (default) or the ISL29125
 * board
 *
 * @note No RGBLight Sensors are provided with the Space Kit
 *
 * Example Usage:
 * @code
 *     RGBLight rgb;                          // Instantiate sensor object
 *     rgb.begin();                           // Initialize sensor
 *     Serial.println(rgb.readToJSON("rgb")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     RGBLightTCS rgb;                       // Instantiate sensor object
 *     rgb.begin();                           // Initialize sensor
 *     Serial.println(rgb.readToJSON("rgb")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     RGBLightISL rgb;                       // Instantiate sensor object
 *     rgb.begin();                           // Initialize sensor
 *     Serial.println(rgb.readToJSON("rgb")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class RGBLight: public Sensor {
  protected:
    tcs34725IntegrationTime_t tcsIt;
    tcs34725Gain_t tcsGain;

    boolean initialize(void);
    boolean readSensor(void);
    RGBLight(tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain);

  public:
    float red;
    float green;
    float blue;
    RGBLight(void);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};

/**
 * @class RGBLightTCS
 * @ingroup rgblight
 *
 * @brief Encapsulates all functionality related to the TCS34725 RGBLight Sensor
 */
class RGBLightTCS: public RGBLight {
  public:
    RGBLightTCS(void);
    RGBLightTCS(tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain);
    RGBLightTCS(tcs34725Gain_t tcsGain, tcs34725IntegrationTime_t tcsIt);
    RGBLightTCS(tcs34725IntegrationTime_t tcsIt);
    RGBLightTCS(tcs34725Gain_t tcsGain);
};

/**
 * @class RGBLightISL
 * @ingroup rgblight
 *
 * @brief Encapsulates all functionality related to the ISL29125 RGBLight Sensor
 */
class RGBLightISL: public RGBLight {
  protected:
    uint8_t islIntensity;
    boolean initialize(void);
    boolean readSensor(void);

  public:
    RGBLightISL(void);
    RGBLightISL(uint8_t islIntensity);
};


/**************************************************************************//**
 * @class Temperature
 * @ingroup sensor
 *
 * @defgroup temperature
 * @brief Encapsulates all functionality related to the Temperature Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from either the TMP102 board (default) or the MLX90614 board
 *
 * The TMP102 is an ambient temperature sensor and the MLX90614 is an infrared
 * temperature sensor
 *
 * Example Usage:
 * @code
 *     Temperature temp;                                // Instantiate sensor object
 *     temp.begin();                                    // Initialize sensor
 *     Serial.println(temp.readToJSON("ambient_temp")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     TemperatureTMP temp;                             // Instantiate sensor object
 *     temp.begin();                                    // Initialize sensor
 *     Serial.println(temp.readToJSON("ambient_temp")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     TemperatureMLX temp;                              // Instantiate sensor object
 *     temp.begin();                                     // Initialize sensor
 *     Serial.println(temp.readToJSON("infrared_temp")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Temperature: public Sensor {
  protected:
    boolean initialize(void);
    boolean readSensor(void);

  public:
    float t;
    Temperature(void);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};

/**
 * @class TemperatureTMP
 * @ingroup temperature
 *
 * @brief Encapsulates all functionality related to the TMP102 Ambient Temperature Sensor
 */
class TemperatureTMP: public Temperature {
};

/**
 * @class TemperatureMLX
 * @ingroup temperature
 *
 * @brief Encapsulates all functionality related to the MLX90614 Infrared Temperature Sensor
 */
class TemperatureMLX: public Temperature {
  protected:
    boolean initialize(void);
    boolean readSensor(void);

  public:
    TemperatureMLX(void);
};


/**************************************************************************//**
 * @class UVLight
 * @ingroup sensor
 *
 * @defgroup uvlight
 * @brief Encapsulates all functionality related to the UVLight Sensor
 *
 * This class can be used to initialize, further configure, read, and print
 * data receieved from either the ML8511 board (default) or the SI1132 board
 *
 * Example Usage:
 * @code
 *     UVLight uv;                          // Instantiate sensor object
 *     uv.begin();                          // Initialize sensor
 *     Serial.println(uv.readToJSON("uv")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     UVLightML uv;                        // Instantiate sensor object
 *     uv.begin();                          // Initialize sensor
 *     Serial.println(uv.readToJSON("uv")); // Read and print values in JSON
 * @endcode
 * Or
 * @code
 *     UVLightSI uv;                        // Instantiate sensor object
 *     uv.begin();                          // Initialize senor
 *     Serial.println(uv.readToJSON("uv")); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class UVLight: public Sensor {
  protected:
    int ML8511_pin;

    boolean initialize(void);
    boolean readSensor(void);
    UVLight(int pin);

  public:
    float uvindex;
    UVLight(void);

    const char * toCSV(const char * sensorName);
    const char * toJSON(const char * sensorName);
};

/**
 * @class UVLightML
 * @ingroup uvlight
 *
 * @brief Encapsulates all functionality related to the ML8511 UVLight Sensor
 */
class UVLightML: public UVLight {
  public:
    UVLightML(void);
    UVLightML(int pin);
};

/**
 * @class UVLightSI
 * @ingroup uvlight
 *
 * @brief Encapsulates all functionality related to the SI1132 UVLight Sensor
 */
class UVLightSI: public UVLight {
  protected:
    boolean initialize(void);
    boolean readSensor(void);

  public:
    UVLightSI(void);
};

#endif /* ARDUSATSDK_H_ */
