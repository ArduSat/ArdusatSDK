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
extern bool MANUAL_CONFIG;

/**
 * Used when dynamically checking if the SpaceBoard is being used or not. If
 * the SpaceBoard is being used, different addresses might be used for each
 * sensor.
 */
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
 * creates a string representation of the data in a JSON format that can be used with
 * http://experiments.ardusat.com to visualize and log data.
 *
 * Format is:
 * ~{"sensorName": "name", "unit": "C", "value": 35.3}|
 */
const char * valueToJSON(const char *sensorName, unsigned char unit, float value);

/**
 * @defgroup sensors
 */

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
 *     Acceleration accel = Acceleration(); // Instantiate sensor object
 *     accel.begin();                       // Initialize sensor
 *     Serial.println(accel.readToJSON());  // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Acceleration {
  private:
    sensor_id_t sensorId;
    lsm303_accel_gain_e gGain;

  public:
    _data_header_t header;
    boolean initialized;
    float x;
    float y;
    float z;

    Acceleration(void);
    ~Acceleration(void);

    boolean begin(void);
    boolean begin(lsm303_accel_gain_e gGain);

    void read(void);
    const char * readToCSV(const char * sensorName = "Acceleration");
    const char * readToJSON(const char * sensorName = "Acceleration");
    const char * toCSV(const char * sensorName = "Acceleration");
    const char * toJSON(const char * sensorName = "Acceleration");
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
 *     Gyro gyro = Gyro();                // Instantiate sensor object
 *     gyro.begin();                      // Initialize sensor
 *     Serial.println(gyro.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Gyro {
  private:
    sensor_id_t sensorId;
    uint8_t range;

  public:
    _data_header_t header;
    boolean initialized;
    float x;
    float y;
    float z;

    Gyro(void);
    ~Gyro(void);

    boolean begin(void);
    boolean begin(uint8_t range);

    void read(void);
    const char * readToCSV(const char * sensorName = "Gyro");
    const char * readToJSON(const char * sensorName = "Gyro");
    const char * toCSV(const char * sensorName = "Gyro");
    const char * toJSON(const char * sensorName = "Gyro");
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
 *     Luminosity lum = Luminosity();    // Instantiate sensor object
 *     lum.begin();                      // Initialize sensor
 *     Serial.println(lum.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Luminosity {
  private:
    sensor_id_t sensorId;
    tsl2561IntegrationTime_t intTime;
    tsl2561Gain_t gain;

  public:
    _data_header_t header;
    boolean initialized;
    float lux;

    Luminosity(void);
    ~Luminosity(void);

    boolean begin(void);
    boolean begin(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain);

    void read(void);
    const char * readToCSV(const char * sensorName = "Luminosity");
    const char * readToJSON(const char * sensorName = "Luminosity");
    const char * toCSV(const char * sensorName = "Luminosity");
    const char * toJSON(const char * sensorName = "Luminosity");
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
 *     Magnetic mag = Magnetic();        // Instantiate sensor object
 *     mag.begin();                      // Initialize sensor
 *     Serial.println(mag.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Magnetic {
  private:
    sensor_id_t sensorId;
    lsm303_mag_scale_e gaussScale;

  public:
    _data_header_t header;
    boolean initialized;
    float x;
    float y;
    float z;

    Magnetic(void);
    ~Magnetic(void);

    boolean begin(void);
    boolean begin(lsm303_mag_scale_e gaussScale);

    void read(void);
    const char * readToCSV(const char * sensorName = "Magnetic");
    const char * readToJSON(const char * sensorName = "Magnetic");
    const char * toCSV(const char * sensorName = "Magnetic");
    const char * toJSON(const char * sensorName = "Magnetic");
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
 *     Orientation orient = Orientation();  // Instantiate sensor object
 *     orient.begin();                      // Initialize sensor
 *     Serial.println(orient.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Orientation {
  private:
    sensor_id_t sensorId;
    Acceleration * accel;
    Magnetic * mag;

  public:
    _data_header_t header;
    boolean initialized;
    float roll;
    float pitch;
    float heading;

    Orientation(void);
    ~Orientation(void);

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
 *     Pressure press = Pressure();        // Instantiate sensor object
 *     press.begin();                      // Initialize sensor
 *     Serial.println(press.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Pressure {
  private:
    sensor_id_t sensorId;
    bmp085_mode_t bmp085_mode;

  public:
    _data_header_t header;
    boolean initialized;
    float pressure;

    Pressure(void);
    ~Pressure(void);

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
 *     RGBLight rgb = RGBLight();        // Instantiate sensor object
 *     rgb.begin();                      // Initialize sensor
 *     Serial.println(rgb.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class RGBLight {
  private:
    sensor_id_t sensorId;
    uint8_t islIntensity;
    tcs34725IntegrationTime_t tcsIt;
    tcs34725Gain_t tcsGain;
    boolean begin(uint8_t islIntensity, tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain);

  public:
    _data_header_t header;
    boolean initialized;
    float red;
    float green;
    float blue;

    RGBLight(sensor_id_t sensor_id=SENSORID_TCS34725);
    ~RGBLight(void);

    boolean begin(void);
    boolean begin(uint8_t islIntensity);
    boolean begin(tcs34725IntegrationTime_t it, tcs34725Gain_t gain);

    void read(void);
    const char * readToCSV(const char * sensorName = "RGBLight");
    const char * readToJSON(const char * sensorName = "RGBLight");
    const char * toCSV(const char * sensorName = "RGBLight");
    const char * toJSON(const char * sensorName = "RGBLight");
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
 *     Temperature temp = Temperature();  // Instantiate sensor object
 *     temp.begin();                      // Initialize sensor
 *     Serial.println(temp.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class Temperature {
  private:
    sensor_id_t sensorId;

  public:
    _data_header_t header;
    boolean initialized;
    float t;

    Temperature(sensor_id_t sensor_id=SENSORID_TMP102);
    ~Temperature(void);

    boolean begin(void);

    void read(void);
    const char * readToCSV(const char * sensorName = "Temperature");
    const char * readToJSON(const char * sensorName = "Temperature");
    const char * toCSV(const char * sensorName = "Temperature");
    const char * toJSON(const char * sensorName = "Temperature");
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
 *     UVLight uv = UVLight();          // Instantiate sensor object
 *     uv.begin();                      // Initialize sensor
 *     Serial.println(uv.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
class UVLight {
  private:
    sensor_id_t sensorId;
    int ML8511_pin;

  public:
    _data_header_t header;
    boolean initialized;
    float uvindex;

    UVLight(sensor_id_t sensor_id=SENSORID_ML8511);
    ~UVLight(void);

    boolean begin(void);
    boolean begin(int pin);

    void read(void);
    const char * readToCSV(const char * sensorName = "UVLight");
    const char * readToJSON(const char * sensorName = "UVLight");
    const char * toCSV(const char * sensorName = "UVLight");
    const char * toJSON(const char * sensorName = "UVLight");
};

#endif /* ARDUSATSDK_H_ */
