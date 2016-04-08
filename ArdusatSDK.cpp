/**
 * @file   ArdusatSDK.cpp
 * @author Ben Peters (ben@ardusat.com)
 * @author Sam Olds (sam@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor reading and configuration for Space Kit Sensors.
 *
 * Provides a unifying wrapper of sensor specific functionality to provide a
 * consistent interface to interact with each type of sensor.
 */

#include <stdio.h>
#include <string.h>
#include "ArdusatSDK.h"

bool MANUAL_CONFIG = false;
bool ARDUSAT_SPACEBOARD = false;
int OUTPUT_BUF_SIZE = 256;
char * _output_buffer;
static int _output_buf_len = 0;

// TODO: Change these error messages to be JSON that can easily be caught by the Experiment Platform
const char begin_error_msg[] PROGMEM = "begin%s failed. Check wiring!";
const char unavailable_on_spaceboard_error_msg[] PROGMEM = "%s is not available with %s";

const char spacekit_hardware[] PROGMEM = "Space Kit";
const char spaceboard_hardware[] PROGMEM = "SpaceBoard";

const char acceleration_sensor_name[] PROGMEM = "Acceleration";
const char gyro_sensor_name[] PROGMEM = "Gyro";
const char luminosity_sensor_name[] PROGMEM = "Luminosity";
const char magnetic_sensor_name[] PROGMEM = "Magnetic";
const char pressure_sensor_name[] PROGMEM = "BarometricPressure";
const char temperature_sensor_name[] PROGMEM = "Temperature";
const char irtemperature_sensor_name[] PROGMEM = "IRTemperature";
const char rgblight_sensor_name[] PROGMEM = "RGBLight";
const char uvlight_sensor_name[] PROGMEM = "UVLight";

static char CSV_SEPARATOR = ',';
static char JSON_PREFIX = '~';
static char JSON_SUFFIX = '|';
const char json_format[] PROGMEM = "%c{\"sensorName\":\"%s\",\"unit\":\"%s\",\"value\":%s,\"cs\":%d}%c\n";

/**
 * Gets the output buffer used for storing sensor data, or initializes
 * it if it doesn't yet exist
 *
 * @return the current output buffer
 */
char * _getOutBuf() {
  if (_output_buffer == NULL) {
    _output_buffer = new char[OUTPUT_BUF_SIZE];
  }
  return _output_buffer;
}

void _resetOutBuf() {
  memset(_getOutBuf(), 0, OUTPUT_BUF_SIZE);
  _output_buf_len = 0;
}

/**
 * Convert an enumerated unit code to a string representation.
 *
 * @param unit code (see units.h defines)
 *
 * @return string representation of unit
 */
const char * unit_to_str(unsigned char unit) {
  switch (unit) {
    case (DATA_UNIT_NONE):
      return "";
    case (DATA_UNIT_METER_PER_SECONDSQUARED):
      return "m/s^2";
    case (DATA_UNIT_RADIAN_PER_SECOND):
      return "rad/s";
    case (DATA_UNIT_MICROTESLA):
      return "uT";
    case (DATA_UNIT_DEGREES_CELSIUS):
      return "C";
    case (DATA_UNIT_METER_PER_SECOND):
      return "m/s";
    case (DATA_UNIT_LUX):
      return "lux";
    case (DATA_UNIT_MILLIWATT_PER_CMSQUARED):
      return "mW/cm^2";
    case (DATA_UNIT_HECTOPASCAL):
      return "hPa";
    default:
      return "";
  };
}

/**
 * Prints an error message if beginSensor function fails.
 *
 * This relies on a 128 character output buffer. Make sure that sensorName isn't too long!
 *
 * @param sensorName name of sensor that failed.
 */
void _writeBeginError(const char sensorName[] PROGMEM) {
  char err_msg[55];
  char sensor[50];

  strcpy_P(err_msg, begin_error_msg);
  strcpy_P(sensor, sensorName);

  //Make SURE sensorName isn't too long for the output buffer!!!
  _resetOutBuf();
  sprintf(_getOutBuf(), err_msg, sensor);
  Serial.println(_getOutBuf());
}

/**
 * Prints an error message if desired sensor is not part of spaceboard buildout
 *
 * This relies on a 128 character output buffer. Make sure that sensorName isn't too long!
 *
 * @param sensorName name of sensor that failed.
 */
void _writeUnavailableSensorError(const char sensorName[] PROGMEM, const char hardwareBuild[] PROGMEM) {
  char err_msg[55];
  char sensor[50];
  char hardware[15];

  strcpy_P(err_msg, unavailable_on_spaceboard_error_msg);
  strcpy_P(sensor, sensorName);
  strcpy_P(hardware, hardwareBuild);

  //Make SURE sensorName isn't too long for the output buffer!!!
  _resetOutBuf();
  sprintf(_getOutBuf(), err_msg, sensor, hardware);
  Serial.println(_getOutBuf());
}

/**
 * Accepts initialization functions as a parameter. Tries to initialize functions
 * but prints error messages if fails.
 *
 * @param sensorName name of sensor that failed.
 * @param init_func function to try and initialize sensor
 */
boolean start_sensor_or_err(const char sensorName[] PROGMEM, boolean initialized) {
  if (!initialized) {
    _writeBeginError(sensorName);
    return false;
  } else {
    return true;
  }
}

/*
 * Internal helper to do shared checksum logic
 */
int _calculateCheckSumValue(const char *sensor_name, int num_vals, va_list values) {
  int cs = 0;
  const char *c_ptr = sensor_name;

  for (int i = 0; i < num_vals; ++i)
  {
    cs += lround(va_arg(values, double));
  }

  while (*c_ptr != 0) {
    cs += *c_ptr++;
  }
  return cs;
}

/**
 * Calculates a checksum value for a given sensorName and value
 *
 * @param sensor_name name of sensor
 * @param num_vals Number of values
 * @param values variable number of floats to write
 *
 * @return checksum
 */
int calculateCheckSum(const char *sensor_name, int num_vals, ...) {
  va_list values;
  va_start(values, num_vals);
  int cs = _calculateCheckSumValue(sensor_name, num_vals, values);
  va_end(values);
  return cs;
}

/**
 * Create a CSV string with a generic float value and a sensor name. Optional timestamp
 * argument allows passing in a timestamp; will use millis() otherwise.
 *
 * @param sensorName string sensor name
 * @param value value to write
 * @param timestamp optional timestamp. If 0, millis() will be called.
 *
 * @return pointer to output buffer
 */
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp) {
  return valuesToCSV(sensorName, timestamp, 1, value);
}

/**
 * Create a CSV string with a generic array of float values and a sensor name. Optional timestamp
 * argument allows passing in a timestamp; will use millis() otherwise.
 *
 * @param sensorName string sensor name
 * @param timestamp optional timestamp. If 0, millis() will be called.
 * @param numValues number of float values
 * @param variable float values
 *
 * @return pointer to output buffer
 */
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, ...) {
  int i, name_len;
  va_list args;

  if (timestamp == 0) {
    timestamp = millis();
  }

  _resetOutBuf();

  ultoa(timestamp, _getOutBuf(), 10);
  _output_buf_len = strlen(_getOutBuf());

  if (sensorName != NULL) {
    if ((name_len = strlen(sensorName)) > 50) {
      name_len = 50;
    }
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
    memcpy(&(_getOutBuf()[_output_buf_len]), sensorName, name_len);
    _output_buf_len += name_len;
  }

  va_start(args, numValues);
  for (i = 0; i < numValues; ++i) {
    // We don't know *exactly* how long the floating point value is
    // going to be, so just take a guess here...
    if (_output_buf_len > OUTPUT_BUF_SIZE - 10) {
      break;
    }
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
    dtostrf(va_arg(args, double), 2, 3, _getOutBuf() + _output_buf_len);
    _output_buf_len = strlen(_getOutBuf());
  }
  va_end(args);

  if (_output_buf_len < OUTPUT_BUF_SIZE - 10) {
    va_start(args, numValues);
    int cs = _calculateCheckSumValue(sensorName, numValues, args);
    va_end(args);
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
    itoa(cs, _getOutBuf() + _output_buf_len, 10);
    _output_buf_len = strlen(_getOutBuf());
  }
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

/*
 * Internal helper to build the JSON string at the proper point in the output buffer
 * with the correct values and labels
 */
int _writeJSONValue(char *buf, const char *sensor_name, const char *unit, float value) {
  char num [32];
  char format_str[80];
  // inexact estimate on the number of characters the value will take up...
  if (strlen(sensor_name) + strlen(unit) + 10 + _output_buf_len > OUTPUT_BUF_SIZE) {
    return -1;
  }
  dtostrf(value, 4, 2, num);
  strcpy_P(format_str, json_format);
  _output_buf_len += sprintf(buf, format_str,
			     JSON_PREFIX, sensor_name, unit, num,
			     calculateCheckSum(sensor_name, 1, value), JSON_SUFFIX);
  return _output_buf_len;
}

/*
 * Internal helper to build 3 JSON strings back to back since there are so many sensors
 * that report 3 values
 */
const char * _tripleValueToJSON(const char *sensorName, int labelLength, unsigned char unit,
             char *labelA, float valueA, char *labelB, float valueB, char *labelC, float valueC) {
  int nameLength = strlen(sensorName);
  char nameBuf[nameLength + labelLength];
  _resetOutBuf();

  sprintf(nameBuf, labelA, sensorName);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf, unit_to_str(unit), valueA);

  sprintf(nameBuf, labelB, sensorName);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf, unit_to_str(unit), valueB);

  sprintf(nameBuf, labelC, sensorName);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf, unit_to_str(unit), valueC);
  return _getOutBuf();
}

/**
 * Creates a JSON string with the appropriate values
 *
 * @param sensor_name the label to be given to some data
 * @param unit the unit of measurement used
 * @param value the sensor value
 *
 * @return the output buffer
 */
const char * valueToJSON(const char *sensorName, unsigned char unit, float value) {
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensorName, unit_to_str(unit), value);
  return _getOutBuf();
}


/**************************************************************************//**
 * @brief   Constructs Acceleration sensor object
 * @ingroup acceleration
 *
 * Example Usage:
 * @code
 *     Acceleration accel = Acceleration(); // Instantiate sensor object
 *     accel.begin();                       // Initialize sensor
 *     Serial.println(accel.readToJSON());  // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Acceleration::Acceleration(void) {
  Acceleration::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Acceleration::gGain = LSM303_ACCEL_GAIN8G;
  Acceleration::header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
  Acceleration::initialized = false;
}

Acceleration::~Acceleration() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup acceleration
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Acceleration::begin(void) {
  return Acceleration::begin(Acceleration::gGain);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup acceleration
 *
 * @param gGain Advanced configuration for accelerometer's gain
 *     - LSM303_ACCEL_GAIN2G
 *     - LSM303_ACCEL_GAIN4G
 *     - LSM303_ACCEL_GAIN6G
 *     - LSM303_ACCEL_GAIN8G (Default)
 *     - LSM303_ACCEL_GAIN16G
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Acceleration::begin(lsm303_accel_gain_e gain) {
  catchSpaceboard();
  Acceleration::gGain = gain;
  Acceleration::initialized = start_sensor_or_err(acceleration_sensor_name, lsm303_accel_init(gain));
  return Acceleration::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup acceleration
 */
void Acceleration::read(void) {
  Acceleration::header.timestamp = millis();

  Acceleration::header.sensor_id = Acceleration::sensorId;
  lsm303_getAccel(&(Acceleration::x), &(Acceleration::y), &(Acceleration::z));
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup acceleration
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Acceleration::readToCSV(const char * sensorName) {
  Acceleration::read();
  return Acceleration::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup acceleration
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Acceleration::readToJSON(const char * sensorName) {
  Acceleration::read();
  return Acceleration::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup acceleration
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Acceleration::toCSV(const char * sensorName) {
  if (Acceleration::initialized)
    return valuesToCSV(sensorName, Acceleration::header.timestamp, 3,
                       Acceleration::x, Acceleration::y, Acceleration::z);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup acceleration
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Acceleration::toJSON(const char * sensorName) {
  if (Acceleration::initialized)
    return _tripleValueToJSON(sensorName, 2, Acceleration::header.unit, "%sX", Acceleration::x,
                             "%sY", Acceleration::y, "%sZ", Acceleration::z);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Gyro sensor object
 * @ingroup gyro
 *
 * Example Usage:
 * @code
 *     Gyro gyro = Gyro();                // Instantiate sensor object
 *     gyro.begin();                      // Initialize sensor
 *     Serial.println(gyro.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Gyro::Gyro(void) {
  Gyro::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Gyro::range = 0x20;
  Gyro::header.unit = DATA_UNIT_RADIAN_PER_SECOND;
  Gyro::initialized = false;
}

Gyro::~Gyro() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup gyro
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Gyro::begin(void) {
  return Gyro::begin(Gyro::range);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup gyro
 *
 * @param range Advanced configuration for gyro's range
 * Possible Advanced Configuration Values:
 *     - 0x00  (SENSITIVITY_250DPS)
 *     - 0x10  (SENSITIVITY_500DPS)
 *     - 0x20  (SENSITIVITY_2000DPS) (Default)
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Gyro::begin(uint8_t range) {
  catchSpaceboard();

  Gyro::initialized = start_sensor_or_err(gyro_sensor_name, l3gd20h_init(range));
  return Gyro::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup gyro
 */
void Gyro::read(void) {
  Gyro::header.timestamp = millis();

  Gyro::header.sensor_id = Gyro::sensorId;
  l3gd20h_getOrientation(&(Gyro::x), &(Gyro::y), &(Gyro::z));
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup gyro
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Gyro::readToCSV(const char * sensorName) {
  Gyro::read();
  return Gyro::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup gyro
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Gyro::readToJSON(const char * sensorName) {
  Gyro::read();
  return Gyro::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup gyro
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Gyro::toCSV(const char * sensorName) {
  if (Gyro::initialized)
    return valuesToCSV(sensorName, Gyro::header.timestamp, 3,
                       Gyro::x, Gyro::y, Gyro::z);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup gyro
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Gyro::toJSON(const char * sensorName) {
  if (Gyro::initialized)
    return _tripleValueToJSON(sensorName, 2, Gyro::header.unit, "%sX", Gyro::x,
                             "%sY", Gyro::y, "%sZ", Gyro::z);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Luminosity sensor object
 * @ingroup luminosity
 *
 * Example Usage:
 * @code
 *     Luminosity lum = Luminosity();    // Instantiate sensor object
 *     lum.begin();                      // Initialize sensor
 *     Serial.println(lum.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Luminosity::Luminosity(void) {
  Luminosity::sensorId = SENSORID_TSL2561;
  Luminosity::intTime = TSL2561_INTEGRATIONTIME_13MS;
  Luminosity::gain = TSL2561_GAIN_1X;
  Luminosity::header.unit = DATA_UNIT_LUX;
  Luminosity::initialized = false;
}

Luminosity::~Luminosity() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup luminosity
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Luminosity::begin(void) {
  return Luminosity::begin(Luminosity::intTime, Luminosity::gain);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup luminosity
 *
 * @param intTime Advanced configuration for TSL2561 integration time
 *     - TSL2561_INTEGRATIONTIME_13MS (Default)
 *     - TSL2561_INTEGRATIONTIME_101MS
 *     - TSL2561_INTEGRATIONTIME_402MS
 *     
 * @param gain Advanced configuration for TSL2561 gain
 *     - TSL2561_GAIN_1X (Default)
 *     - TSL2561_GAIN_16X
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Luminosity::begin(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain) {
  catchSpaceboard();
  Luminosity::intTime = intTime;
  Luminosity::gain = gain;
  Luminosity::initialized = start_sensor_or_err(luminosity_sensor_name, tsl2561_init(intTime, gain));
  return Luminosity::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup luminosity
 */
void Luminosity::read(void) {
  Luminosity::header.timestamp = millis();

  Luminosity::header.sensor_id = Luminosity::sensorId;
  Luminosity::lux = tsl2561_getLux();
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup luminosity
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Luminosity::readToCSV(const char * sensorName) {
  Luminosity::read();
  return Luminosity::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup luminosity
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Luminosity::readToJSON(const char * sensorName) {
  Luminosity::read();
  return Luminosity::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup luminosity
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Luminosity::toCSV(const char * sensorName) {
  if (Luminosity::initialized)
    return valueToCSV(sensorName, Luminosity::lux, Luminosity::header.timestamp);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup luminosity
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Luminosity::toJSON(const char * sensorName) {
  if (Luminosity::initialized)
    return valueToJSON(sensorName, Luminosity::header.unit, Luminosity::lux);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Magnetic sensor object
 * @ingroup magnetic
 *
 * Example Usage:
 * @code
 *     Magnetic mag = Magnetic();        // Instantiate sensor object
 *     mag.begin();                      // Initialize sensor
 *     Serial.println(mag.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Magnetic::Magnetic(void) {
  Magnetic::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Magnetic::gaussScale = LSM303_MAG_SCALE4GAUSS;
  Magnetic::header.unit = DATA_UNIT_MICROTESLA;
  Magnetic::initialized = false;
}

Magnetic::~Magnetic() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup magnetic
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Magnetic::begin(void) {
  return Magnetic::begin(Magnetic::gaussScale);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup magnetic
 *
 * @param gaussScale Advanced configuration for magnetometer's scale
 *     - LSM303_MAG_SCALE1_3GAUSS
 *     - LSM303_MAG_SCALE2GAUSS
 *     - LSM303_MAG_SCALE2_5GAUSS
 *     - LSM303_MAG_SCALE4GAUSS (Default)
 *     - LSM303_MAG_SCALE4_7GAUSS
 *     - LSM303_MAG_SCALE5_6GAUSS
 *     - LSM303_MAG_SCALE8GAUSS
 *     - LSM303_MAG_SCALE12GAUSS
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Magnetic::begin(lsm303_mag_scale_e gaussScale) {
  catchSpaceboard();
  Magnetic::gaussScale = gaussScale;
  Magnetic::initialized = start_sensor_or_err(magnetic_sensor_name, lsm303_mag_init(gaussScale));
  return Magnetic::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup magnetic
 */
void Magnetic::read(void) {
  Magnetic::header.timestamp = millis();

  Magnetic::header.sensor_id = Magnetic::sensorId;
  lsm303_getMag(&(Magnetic::x), &(Magnetic::y), &(Magnetic::z));
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup magnetic
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Magnetic::readToCSV(const char * sensorName) {
  Magnetic::read();
  return Magnetic::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup magnetic
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Magnetic::readToJSON(const char * sensorName) {
  Magnetic::read();
  return Magnetic::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup magnetic
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Magnetic::toCSV(const char * sensorName) {
  if (Magnetic::initialized)
    return valuesToCSV(sensorName, Magnetic::header.timestamp, 3,
                       Magnetic::x, Magnetic::y, Magnetic::z);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup magnetic
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Magnetic::toJSON(const char * sensorName) {
  if (Magnetic::initialized)
    return _tripleValueToJSON(sensorName, 2, Magnetic::header.unit, "%sX", Magnetic::x,
                             "%sY", Magnetic::y, "%sZ", Magnetic::z);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Orientation calculation object
 * @ingroup orientation
 *
 * Example Usage:
 * @code
 *     Orientation orient = Orientation();  // Instantiate sensor object
 *     orient.begin();                      // Initialize sensor
 *     Serial.println(orient.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Orientation::Orientation(void) {
  Orientation::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Orientation::header.unit = DATA_UNIT_DEGREES;
  Orientation::initialized = false;
}

Orientation::~Orientation() {
}

/**
 * @brief   Creates new Acceleration and Magnetic Sensors to derive orientation
 * @ingroup orientation
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Orientation::begin(void) {
  Acceleration accel;
  Magnetic mag;
  accel.begin();
  mag.begin();
  return Orientation::begin(accel, mag);
}

/**
 * @brief   Uses provided Acceleration and Magnetic Sensors to derive orientation
 * @ingroup orientation
 *
 * @param accel Existing Acceleration Sensor to use
 * @param mag   Existing Magnetic Sensor to use
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Orientation::begin(Acceleration & accel, Magnetic & mag) {
  catchSpaceboard();
  Orientation::accel = & accel;
  Orientation::mag = & mag;
  Orientation::initialized = accel.initialized && mag.initialized;
  return Orientation::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup orientation
 */
void Orientation::read(void) {
  Orientation::accel->read();
  Orientation::mag->read();
  Orientation::read(*(Orientation::accel), *(Orientation::mag));
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup orientation
 *
 * @param accel Existing Acceleration Sensor to use
 * @param mag   Existing Magnetic Sensor to use
 */
void Orientation::read(Acceleration & acc, Magnetic & mag) {
  float roll;
  float pitch;
  float heading;
  const float PI_F = 3.141592653F;

  // Roll is rotation around x-axis (-180 <= roll <= 180)
  // Positive roll is clockwise rotation wrt positive x axis
  roll = (float) atan2(acc.y, acc.z);

  // Pitch is rotation around y-axis (-180 <= pitch <= 180)
  // Positive pitch is clockwise rotation wrt positive y axis
  if (acc.y * sin(roll) + acc.z * cos(roll) == 0) {
    pitch = acc.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  } else {
    pitch = (float)atan(-acc.x / (acc.y * sin(roll) + acc.z * cos(roll)));
  }

  // Heading is rotation around z-axis
  // Positive heading is clockwise rotation wrt positive z axis
  heading = (float)atan2(mag.z * sin(roll) - mag.y * cos(roll),
			         mag.x * cos(pitch) + mag.y * sin(pitch) * sin(roll) +
				 mag.z * sin(pitch) * cos(roll));

  // Convert radians to degrees
  Orientation::roll = roll * 180 / PI_F;
  Orientation::pitch = pitch * 180 / PI_F;
  Orientation::heading = heading * 180 / PI_F;

  Orientation::header.timestamp = max(acc.header.timestamp, mag.header.timestamp);
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup orientation
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Orientation::readToCSV(const char * sensorName) {
  Orientation::read();
  return Orientation::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup orientation
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Orientation::readToJSON(const char * sensorName) {
  Orientation::read();
  return Orientation::toJSON(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup orientation
 * @param   accel Existing Acceleration Sensor to use
 * @param   mag Existing Magnetic Sensor to use
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Orientation::readToCSV(Acceleration & acc, Magnetic & mag, const char * sensorName) {
  Orientation::read(acc, mag);
  return Orientation::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup orientation
 * @param   accel Existing Acceleration Sensor to use
 * @param   mag Existing Magnetic Sensor to use
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Orientation::readToJSON(Acceleration & acc, Magnetic & mag, const char * sensorName) {
  Orientation::read(acc, mag);
  return Orientation::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup orientation
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Orientation::toCSV(const char * sensorName) {
  if (Orientation::initialized)
    return valuesToCSV(sensorName, Orientation::header.timestamp, 3,
                       Orientation::roll, Orientation::pitch, Orientation::heading);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup orientation
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Orientation::toJSON(const char * sensorName) {
  if (Orientation::initialized)
    return _tripleValueToJSON(sensorName, 8, Orientation::header.unit, "%sRoll", Orientation::roll,
                             "%sPitch", Orientation::pitch, "%sHeading", Orientation::heading);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Pressure object
 * @ingroup pressure
 *
 * Example Usage:
 * @code
 *     Pressure press = Pressure();        // Instantiate sensor object
 *     press.begin();                      // Initialize sensor
 *     Serial.println(press.readToJSON()); // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Pressure::Pressure(void) {
  Pressure::sensorId = SENSORID_BMP180;
  Pressure::bmp085_mode = BMP085_MODE_ULTRAHIGHRES;
  Pressure::header.unit = DATA_UNIT_HECTOPASCAL;
  Pressure::initialized = false;
}

Pressure::~Pressure() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup pressure
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Pressure::begin(void) {
  return Pressure::begin(Pressure::bmp085_mode);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup pressure
 *
 * @param mode Advanced configuration for BMP180's resolution mode
 *     - BMP085_MODE_ULTRALOWPOWER
 *     - BMP085_MODE_STANDARD
 *     - BMP085_MODE_HIGHRES
 *     - BMP085_MODE_ULTRAHIGHRES (Default)
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Pressure::begin(bmp085_mode_t mode) {
  Pressure::bmp085_mode = mode;
  Pressure::initialized = start_sensor_or_err(pressure_sensor_name, bmp180_init(mode));

  catchSpaceboard();
  if (ARDUSAT_SPACEBOARD && !Pressure::initialized) {
    _writeUnavailableSensorError(pressure_sensor_name, spaceboard_hardware);
    return false;
  }

  return Pressure::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup pressure
 */
void Pressure::read(void) {
  Pressure::header.timestamp = millis();

  Pressure::header.sensor_id = Pressure::sensorId;
  bmp180_getPressure(&(Pressure::pressure));
}

/**
 * @brief Calculates current altitude given pressure reading and provided pressure at sea level
 *
 * Equation taken from BMP180 datasheet (page 16):
 * http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
 *
 * Note that using the equation from wikipedia can give bad results at high
 * altitude. See this thread for more information:
 * http://forums.adafruit.com/viewtopic.php?f=22&t=58064
 *
 * @ingroup pressure
 * @param   seaLevelPressure Known pressure at sea level in hPa
 * @return  calculated altitude in meters
 */
float Pressure::altitudeFromSeaLevelPressure(float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(Pressure::pressure / seaLevelPressure, 0.1903));
}

/**
 * @brief Calculate the pressure at sea level given current altitude and pressure reading
 *
 * Equation taken from BMP180 datasheet (page 17):
 * http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
 *
 * Note that using the equation from wikipedia can give bad results at high
 * altitude. See this thread for more information:
 * http://forums.adafruit.com/viewtopic.php?f=22&t=58064
 *
 * @ingroup pressure
 * @param   altitude Known altitude in meters
 * @return  calculated pressure at sea level in hPa
 */
float Pressure::seaLevelPressureFromAltitude(float altitude) {
  return Pressure::pressure / pow(1.0 - (altitude / 44330.0), 5.255);
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup pressure
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Pressure::readToCSV(const char * sensorName) {
  Pressure::read();
  return Pressure::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup pressure
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Pressure::readToJSON(const char * sensorName) {
  Pressure::read();
  return Pressure::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup pressure
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Pressure::toCSV(const char * sensorName) {
  if (Pressure::initialized)
    return valueToCSV(sensorName, Pressure::pressure, Pressure::header.timestamp);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup pressure
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Pressure::toJSON(const char * sensorName) {
  if (Pressure::initialized)
    return valueToJSON(sensorName, Pressure::header.unit, Pressure::pressure);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs RGBLight sensor object, default uses TCS34725 sensor
 * @ingroup rgblight
 *
 * @param   sensor_id Specifies which RGB sensor to use: TCS34725 or ISL29125
 *     - SENSORID_TCS34725
 *     - SENSORID_ISL29125
 *
 * Example Usage:
 * @code
 *     RGBLight rgb = RGBLight(SENSORID_ISL29125); // Instantiate sensor object
 *     rgb.begin();                                // Initialize sensor
 *     Serial.println(rgb.readToJSON());           // Read and print values in JSON
 * @endcode
 *****************************************************************************/
RGBLight::RGBLight(sensor_id_t sensor_id) {
  RGBLight::sensorId = sensor_id;
  RGBLight::islIntensity = CFG1_10KLUX;
  RGBLight::tcsIt = TCS34725_INTEGRATIONTIME_154MS;
  RGBLight::tcsGain = TCS34725_GAIN_1X;
  RGBLight::header.unit = DATA_UNIT_LUX;
  RGBLight::initialized = false;
}

RGBLight::~RGBLight() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup rgblight
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean RGBLight::begin(void) {
  return RGBLight::begin(RGBLight::islIntensity, RGBLight::tcsIt, RGBLight::tcsGain);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup rgblight
 *
 * @param islIntensity Advanced configuration for ISL29125 intensity
 *     - CFG1_375LUX
 *     - CFG1_10KLUX (Default)
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean RGBLight::begin(uint8_t islIntensity) {
  if (RGBLight::sensorId == SENSORID_ISL29125)
    return RGBLight::begin(islIntensity, RGBLight::tcsIt, RGBLight::tcsGain);
  else
    return RGBLight::begin();
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup rgblight
 *
 * @param tcsIt Advanced configuration for TCS34725 integration time
 *     - TCS34725_INTEGRATIONTIME_2_4MS
 *     - TCS34725_INTEGRATIONTIME_24MS
 *     - TCS34725_INTEGRATIONTIME_50MS
 *     - TCS34725_INTEGRATIONTIME_101MS
 *     - TCS34725_INTEGRATIONTIME_154MS (Default)
 *     - TCS34725_INTEGRATIONTIME_700MS
 *
 * @param tcsGain Advanced configuration for TCS34725 gain
 *     - TCS34725_GAIN_1X (Default)
 *     - TCS34725_GAIN_4X
 *     - TCS34725_GAIN_16X
 *     - TCS34725_GAIN_60X
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean RGBLight::begin(tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain) {
  if (RGBLight::sensorId == SENSORID_TCS34725)
    return RGBLight::begin(RGBLight::islIntensity, tcsIt, tcsGain);
  else
    return RGBLight::begin();
}

/**
 * Private function to initialize RGBLight sensor
 */
boolean RGBLight::begin(uint8_t islIntensity, tcs34725IntegrationTime_t tcsIt, tcs34725Gain_t tcsGain) {
  RGBLight::islIntensity = islIntensity;
  RGBLight::tcsIt = tcsIt;
  RGBLight::tcsGain = tcsGain;

  if (RGBLight::sensorId == SENSORID_ISL29125)
    RGBLight::initialized = start_sensor_or_err(rgblight_sensor_name,
                                                isl29125_init(islIntensity));
  else if (RGBLight::sensorId == SENSORID_TCS34725)
    RGBLight::initialized = start_sensor_or_err(rgblight_sensor_name,
                                                tcs34725_init(tcsIt, tcsGain));

  catchSpaceboard();
  if (!ARDUSAT_SPACEBOARD && !RGBLight::initialized) {
    _writeUnavailableSensorError(rgblight_sensor_name, spacekit_hardware);
    return false;
  }

  return RGBLight::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup rgblight
 */
void RGBLight::read(void) {
  RGBLight::header.timestamp = millis();
  RGBLight::header.sensor_id = RGBLight::sensorId;

  if (RGBLight::sensorId == SENSORID_ISL29125)
    isl29125_getRGB(&(RGBLight::red), &(RGBLight::green), &(RGBLight::blue));
  else if (RGBLight::sensorId == SENSORID_TCS34725)
    tcs34725_getRGB(&(RGBLight::red), &(RGBLight::green), &(RGBLight::blue));
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup rgblight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * RGBLight::readToCSV(const char * sensorName) {
  RGBLight::read();
  return RGBLight::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup rgblight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * RGBLight::readToJSON(const char * sensorName) {
  RGBLight::read();
  return RGBLight::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup rgblight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * RGBLight::toCSV(const char * sensorName) {
  if (RGBLight::initialized)
    return valuesToCSV(sensorName, RGBLight::header.timestamp, 3,
                       RGBLight::red, RGBLight::green, RGBLight::blue);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup rgblight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * RGBLight::toJSON(const char * sensorName) {
  if (RGBLight::initialized)
    return _tripleValueToJSON(sensorName, 6, RGBLight::header.unit, "%sRed", RGBLight::red,
                             "%sGreen", RGBLight::green, "%sBlue", RGBLight::blue);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs Temperature sensor object, default uses TMP102 sensor
 * @ingroup temperature
 *
 * @param   sensor_id Specifies which Temperature sensor to use: TMP102 or MLX90614
 *     - SENSORID_TMP102
 *     - SENSORID_MLX90614
 *
 * Example Usage:
 * @code
 *     Temperature temp = Temperature(SENSORID_MLX90614); // Instantiate sensor object
 *     temp.begin();                                      // Initialize sensor
 *     Serial.println(temp.readToJSON());                 // Read and print values in JSON
 * @endcode
 *****************************************************************************/
Temperature::Temperature(sensor_id_t sensor_id) {
  Temperature::sensorId = sensor_id;
  Temperature::header.unit = DATA_UNIT_DEGREES_CELSIUS;
  Temperature::initialized = false;
}

Temperature::~Temperature() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup temperature
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean Temperature::begin(void) {
  catchSpaceboard();
  if (Temperature::sensorId == SENSORID_TMP102) {
    Temperature::initialized = start_sensor_or_err(temperature_sensor_name, tmp102_init());
  } else if (Temperature::sensorId == SENSORID_MLX90614) {
    Temperature::initialized = start_sensor_or_err(irtemperature_sensor_name, mlx90614_init());
  }
  return Temperature::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup temperature
 */
void Temperature::read(void) {
  Temperature::header.timestamp = millis();
  Temperature::header.sensor_id = Temperature::sensorId;

  if (Temperature::sensorId == SENSORID_TMP102)
    Temperature::t = tmp102_getTempCelsius();
  else if (Temperature::sensorId == SENSORID_MLX90614)
    Temperature::t = mlx90614_getTempCelsius();
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup temperature
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Temperature::readToCSV(const char * sensorName) {
  Temperature::read();
  return Temperature::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup temperature
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Temperature::readToJSON(const char * sensorName) {
  Temperature::read();
  return Temperature::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup temperature
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * Temperature::toCSV(const char * sensorName) {
  if (Temperature::initialized)
    return valueToCSV(sensorName, Temperature::t, Temperature::header.timestamp);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup temperature
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * Temperature::toJSON(const char * sensorName) {
  if (Temperature::initialized)
    return valueToJSON(sensorName, Temperature::header.unit, Temperature::t);
  else
    return "";
}


/**************************************************************************//**
 * @brief   Constructs UVLight sensor object, default uses ML8511 sensor
 * @ingroup uvlight
 *
 * @param   sensor_id Specifies which UVLight sensor to use: ML8511 or SI1132
 *     - SENSORID_ML8511
 *     - SENSORID_SI1132
 *
 * Example Usage:
 * @code
 *     UVLight uv = UVLight(SENSORID_SI1132); // Instantiate sensor object
 *     uv.begin();                            // Initialize sensor
 *     Serial.println(uv.readToJSON());       // Read and print values in JSON
 * @endcode
 *****************************************************************************/
UVLight::UVLight(sensor_id_t sensor_id) {
  UVLight::sensorId = sensor_id;
  UVLight::ML8511_pin = DRIVER_ML8511_UV_PIN;
  UVLight::header.unit = DATA_UNIT_MILLIWATT_PER_CMSQUARED;
  UVLight::initialized = false;
}

UVLight::~UVLight() {
}

/**
 * @brief   Initializes the sensor with basic configurations
 * @ingroup uvlight
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean UVLight::begin(void) {
  return UVLight::begin(UVLight::ML8511_pin);
}

/**
 * @brief   Initializes the sensor with advanced configurations
 * @ingroup uvlight
 *
 * @param pin The analog pin the ML8511 board uses on the Arduino
 *     - DRIVER_ML8511_UV_PIN (A0) (Default)
 *
 * @retval true  Successfully initialized
 * @retval false Failed to initialize
 */
boolean UVLight::begin(int pin) {
  catchSpaceboard();
  UVLight::ML8511_pin = pin;
  if (UVLight::sensorId == SENSORID_SI1132)
    UVLight::initialized = start_sensor_or_err(uvlight_sensor_name, si1132_init());
  else if (UVLight::sensorId == SENSORID_ML8511)
    UVLight::initialized = start_sensor_or_err(uvlight_sensor_name, ml8511_init());
  return UVLight::initialized;
}

/**
 * @brief   Takes a reading from the sensor
 * @ingroup uvlight
 */
void UVLight::read(void) {
  UVLight::header.timestamp = millis();

  if (UVLight::sensorId == SENSORID_SI1132) {
    UVLight::header.sensor_id = SENSORID_SI1132;
    UVLight::uvindex = si1132_getUVIndex();
  } else if (UVLight::sensorId == SENSORID_ML8511) {
    UVLight::header.sensor_id = SENSORID_ML8511;
    UVLight::uvindex = ml8511_getUV(UVLight::ML8511_pin);
  }
}

/**
 * @brief   Takes a reading from the sensor and returns value in CSV format
 * @ingroup uvlight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * UVLight::readToCSV(const char * sensorName) {
  UVLight::read();
  return UVLight::toCSV(sensorName);
}

/**
 * @brief   Takes a reading from the sensor and returns value in JSON format
 * @ingroup uvlight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * UVLight::readToJSON(const char * sensorName) {
  UVLight::read();
  return UVLight::toJSON(sensorName);
}

/**
 * @brief   Returns last read value in CSV format
 * @ingroup uvlight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in CSV format or empty string if uninitialized
 */
const char * UVLight::toCSV(const char * sensorName) {
  if (UVLight::initialized)
    return valueToCSV(sensorName, UVLight::uvindex, UVLight::header.timestamp);
  else
    return "";
}

/**
 * @brief   Returns last read value in JSON format
 * @ingroup uvlight
 * @param   sensorName The text to display next to the value
 * @return  sensor readings in JSON format or empty string if uninitialized
 */
const char * UVLight::toJSON(const char * sensorName) {
  if (UVLight::initialized)
    return valueToJSON(sensorName, UVLight::header.unit, UVLight::uvindex);
  else
    return "";
}
