/**
 * @file   ArdusatSDK.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor reading and configuration for Space Kit Sensors.
 */

#include <stdio.h>
#include <string.h>
#include "ArdusatSDK.h"

bool ARDUSAT_SPACEBOARD = false;
int OUTPUT_BUF_SIZE = 256;
char * _output_buffer;
static int _output_buf_len = 0;

const char begin_error_msg[] PROGMEM = "begin%s failed. Check wiring!";
const char gyro_sensor_name[] PROGMEM = "Gyro";
const char acceleration_sensor_name[] PROGMEM = "Acceleration";
const char magnetic_sensor_name[] PROGMEM = "Magnetic";
const char uvlight_sensor_name[] PROGMEM = "UVLight";
const char luminosity_sensor_name[] PROGMEM = "Luminosity";
const char temperature_sensor_name[] PROGMEM = "Temperature";
const char ir_temperature_sensor_name[] PROGMEM = "IRTemperature";
const char pressure_sensor_name[] PROGMEM = "BarometricPressure";
const char rgblight_sensor_name[] PROGMEM = "RGBLight";

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
const char * unit_to_str(unsigned char unit)
{
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
void _writeBeginError(const char sensorName[] PROGMEM)
{
  char err_msg[50];
  char sensor[50];

  strcpy_P(err_msg, begin_error_msg);
  strcpy_P(sensor, sensorName);

  //Make SURE sensorName isn't too long for the output buffer!!!
  _resetOutBuf();
  sprintf(_getOutBuf(), err_msg, sensor);
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
    Serial.println(_getOutBuf());
    return false;
  } else {
    return true;
  }
}

/*
 * Temperature
 */
boolean beginTemperatureSensor(void) {
  return start_sensor_or_err(temperature_sensor_name, tmp102_init());
}

void readTemperature(temperature_t & output) {
  output.header.unit = DATA_UNIT_DEGREES_CELSIUS;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_TMP102;
  output.t = tmp102_getTempCelsius();
}

/*
 * IR Temperature
 */
boolean beginInfraredTemperatureSensor(void) {
  return start_sensor_or_err(ir_temperature_sensor_name, mlx90614_init());
}

void readInfraredTemperature(temperature_t & output) {
  output.header.unit = DATA_UNIT_DEGREES_CELSIUS;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_MLX90614;
  output.t = mlx90614_getTempCelsius();
}

/*
 * Luminosity
 */
boolean beginLuminositySensor(void) {
  return start_sensor_or_err(luminosity_sensor_name, tsl2561_init());
}

void readLuminosity(luminosity_t & output) {
  output.header.unit = DATA_UNIT_LUX;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_TSL2561;
  output.lux = tsl2561_getLux();
}

/*
 * UV Light
 */
sensor_id_t uvLightSensorID;
boolean beginUVLightSensor(sensor_id_t sensor_id) {
  uvLightSensorID = sensor_id;
  if (sensor_id == SENSORID_SI1132)
    return start_sensor_or_err(uvlight_sensor_name, si1132_init());
  else if (sensor_id == SENSORID_ML8511)
    return start_sensor_or_err(uvlight_sensor_name, ml8511_init());
  else
    return false;
}

void readUVLight(uvlight_t & output, int pin) {
  output.header.unit = DATA_UNIT_MILLIWATT_PER_CMSQUARED;
  output.header.timestamp = millis();

  if (uvLightSensorID == SENSORID_SI1132) {
    output.header.sensor_id = SENSORID_SI1132;
    output.uvindex = si1132_getUVIndex();
  } else if (uvLightSensorID == SENSORID_ML8511) {
    output.header.sensor_id = SENSORID_ML8511;
    output.uvindex = ml8511_getUV(pin);
  }
}

/*
 * Acceleration
 */
boolean beginAccelerationSensor() {
  return start_sensor_or_err(acceleration_sensor_name, lsm303_accel_init());
}

void readAcceleration(acceleration_t & output) {
  output.header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  lsm303_getAccel(&(output.x), &(output.y), &(output.z));
}

/*
 * Magnetic Field
 */
boolean beginMagneticSensor() {
  return start_sensor_or_err(magnetic_sensor_name, lsm303_mag_init());
}

void readMagnetic(magnetic_t & output) {
  output.header.unit = DATA_UNIT_MICROTESLA;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  lsm303_getMag(&(output.x), &(output.y), &(output.z));
}

/*
 * Gyroscope
 */
boolean beginGyroSensor() {
  return start_sensor_or_err(gyro_sensor_name, l3gd20h_init());
}

void readGyro(gyro_t & output) {
  output.header.unit = DATA_UNIT_RADIAN_PER_SECOND;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  l3gd20h_getOrientation(&(output.x), &(output.y), &(output.z));
}

/*
 * RGB Light
 */
sensor_id_t rgbLightSensorID;
boolean beginRGBLightSensor(sensor_id_t sensor_id) {
  rgbLightSensorID = sensor_id;
  if (sensor_id == SENSORID_ISL29125) {
    // TODO: Support advance sensor configuration:
    //       https://github.com/sparkfun/ISL29125_Breakout/blob/V_H1.0_L1.0.1/Libraries/
    //               Arduino/examples/ISL29125Interrupts/ISL29125Interrupts.ino
    return start_sensor_or_err(rgblight_sensor_name, isl29125_init());
  } else if (sensor_id ==  SENSORID_TCS34725) {
    // TODO: Support advance sensor configuration:
    //       https://learn.adafruit.com/adafruit-color-sensors/program-it
    return start_sensor_or_err(rgblight_sensor_name, tcs34725_init(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X));
  } else
    return false;
}

void readRGBLight(rgblight_t & output) {
  output.header.unit = DATA_UNIT_LUX;
  output.header.timestamp = millis();

  // TODO: Adjust RGB for relative luminance?
  //       Y = 0.2126 * R + 0.7152 * G + 0.0722 * B

  if (rgbLightSensorID == SENSORID_ISL29125) {
    output.header.sensor_id = SENSORID_ISL29125;
    isl29125_getRGB(&(output.red), &(output.green), &(output.blue));
  } else if (rgbLightSensorID ==  SENSORID_TCS34725) {
    output.header.sensor_id = SENSORID_TCS34725;
    tcs34725_getRGB(&(output.red), &(output.green), &(output.blue));
  }
}

/*
 * Barometric Pressure
 */
boolean beginBarometricPressureSensor(bmp085_mode_t mode) {
  return start_sensor_or_err(pressure_sensor_name, bmp180_init(mode));
}

void readBarometricPressure(pressure_t & output) {
  output.header.unit = DATA_UNIT_HECTOPASCAL;
  output.header.timestamp = millis();

  output.header.sensor_id = SENSORID_BMP180;
  bmp180_getPressure(&output.pressure);
}

/**
 * Calculate the orientation (pitch, roll, heading) from raw magnetometer and
 * accelerometer readings.
 *
 * Calculation based on Adafruit 9DOF library
 * (https://github.com/adafruit/Adafruit_9DOF/)
 *
 * @param accel Acceleration reading to use in calculation
 * @param mag Magnetometer reading to use in calculation
 * @param orient Orientation structure to save calculated orientation in
 */
void calculateOrientation(const acceleration_t & accel, const magnetic_t & mag,
			  orientation_t & orient)
{
  const float PI_F = 3.141592653F;

  // Roll is rotation around x-axis (-180 <= roll <= 180)
  // Positive roll is clockwise rotation wrt positive x axis
  orient.roll = (float) atan2(accel.y, accel.z);

  // Pitch is rotation around y-axis (-180 <= pitch <= 180)
  // Positive pitch is clockwise rotation wrt positive y axis
  if (accel.y * sin(orient.roll) + accel.z * cos(orient.roll) == 0) {
    orient.pitch = accel.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  } else {
    orient.pitch = (float)atan(-accel.x / (accel.y * sin(orient.roll) +
					     accel.z * cos(orient.roll)));
  }

  // Heading is rotation around z-axis
  // Positive heading is clockwise rotation wrt positive z axis
  orient.heading = (float)atan2(mag.z * sin(orient.roll) - mag.y * cos(orient.roll),
			         mag.x * cos(orient.pitch) + mag.y * sin(orient.pitch) * sin(orient.roll) +
				 mag.z * sin(orient.pitch) * cos(orient.roll));

  // Convert radians to degrees
  orient.roll = orient.roll * 180 / PI_F;
  orient.pitch = orient.pitch * 180 / PI_F;
  orient.heading = orient.heading * 180 / PI_F;
  orient.header.unit = DATA_UNIT_DEGREES;
  orient.header.timestamp = accel.header.timestamp > mag.header.timestamp ? accel.header.timestamp :
									       mag.header.timestamp;
}

/*
 * Internal helper to do shared checksum logic
 */
int _calculateCheckSumV(const char *sensor_name, int num_vals, va_list values) {
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
  int cs = _calculateCheckSumV(sensor_name, num_vals, values);
  va_end(values);
  return cs;
}

/*
 * toCSV Output functions
 */
const char * _headerToCSV(_data_header_t * header, const char *sensorName) {
  int name_len;

  if (header == NULL)
    return (NULL);

  _resetOutBuf();

  ultoa(header->timestamp, _getOutBuf(), 10);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  if (sensorName != NULL) {
    if ((name_len = strlen(sensorName)) > 50) {
      name_len = 50;
    }

    memcpy(&(_getOutBuf()[_output_buf_len]), sensorName, name_len);
    _output_buf_len += name_len;
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
  } else {
    utoa(header->sensor_id, _getOutBuf() + _output_buf_len, 10);
    _output_buf_len = strlen(_getOutBuf());
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
  }

  return _getOutBuf();
}

#define _add_checksum_to_csv_buffer(sensor_name, num_vals, ...) \
  do { \
  if (_output_buf_len < OUTPUT_BUF_SIZE - 10) { \
    int cs = calculateCheckSum(sensor_name, num_vals, __VA_ARGS__); \
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR; \
    itoa(cs, _getOutBuf() + _output_buf_len, 10); \
    _output_buf_len = strlen(_getOutBuf()); \
  } } while (0)

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
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp)
{
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
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, ...)
{
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
    int cs = _calculateCheckSumV(sensorName, numValues, args);
    va_end(args);
    _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;
    itoa(cs, _getOutBuf() + _output_buf_len, 10);
    _output_buf_len = strlen(_getOutBuf());
  }
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

// Define a version of a ToCSV function that uses the default name for that
// sensor type
#define _DEFAULT_SENSOR_NAME_TO_CSV_FXN(NAME) \
  const char * NAME ## ToCSV( NAME ## _t &input) { \
    char sensor[50]; \
    strcpy_P(sensor, NAME ## _sensor_name); \
    return NAME ## ToCSV ( sensor, input ); \
  }

int _writeCSVTripleEntry(float a, float b, float c)
{
  dtostrf(a, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(b, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(c, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  return _output_buf_len;
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(acceleration)
const char * accelerationToCSV(const char *sensorName, acceleration_t & input) {
  _headerToCSV(&(input.header), sensorName);

  _writeCSVTripleEntry(input.x, input.y, input.z);

  _add_checksum_to_csv_buffer(sensorName, 3, input.x, input.y, input.z);
  _getOutBuf()[_output_buf_len++] = '\n';
  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(magnetic)
const char * magneticToCSV(const char *sensorName, magnetic_t & input) {
  _headerToCSV(&(input.header), sensorName);

  _writeCSVTripleEntry(input.x, input.y, input.z);

  _add_checksum_to_csv_buffer(sensorName, 3, input.x, input.y, input.z);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(temperature)
const char * temperatureToCSV(const char *sensorName, temperature_t & input) {
  _headerToCSV(&(input.header), sensorName);// warning : resets the internal buffer

  dtostrf(input.t, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 1, input.t);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(gyro)
const char * gyroToCSV(const char *sensorName, gyro_t & input) {
  _headerToCSV(&(input.header), sensorName);// warning : resets the internal buffer

  _writeCSVTripleEntry(input.x, input.y, input.z);

  _add_checksum_to_csv_buffer(sensorName, 3, input.x, input.y, input.z);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(luminosity)
const char * luminosityToCSV(const char *sensorName, luminosity_t & input) {
  _headerToCSV(&(input.header), sensorName);// warning : resets the internal buffer

  dtostrf(input.lux, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 1, input.lux);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(uvlight)
const char * uvlightToCSV(const char *sensorName, uvlight_t & input) {
  _headerToCSV(&(input.header), sensorName);// warning : resets the internal buffer

  dtostrf(input.uvindex, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 1, input.uvindex);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * orientationToCSV(const char *sensorName, orientation_t & input) {
  _headerToCSV(&(input.header), sensorName);

  _writeCSVTripleEntry(input.roll, input.pitch, input.heading);

  _add_checksum_to_csv_buffer(sensorName, 3, input.roll, input.pitch, input.heading);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(rgblight)
const char * rgblightToCSV(const char *sensorName, rgblight_t & input) {
  _headerToCSV(&(input.header), sensorName);

  _writeCSVTripleEntry(input.red, input.green, input.blue);

  _add_checksum_to_csv_buffer(sensorName, 3, input.red, input.green, input.blue);
  _getOutBuf()[_output_buf_len++] = '\n';
  return _getOutBuf();
}

_DEFAULT_SENSOR_NAME_TO_CSV_FXN(pressure)
const char * pressureToCSV(const char *sensorName, pressure_t & input) {
  _headerToCSV(&(input.header), sensorName);

  dtostrf(input.pressure, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 1, input.pressure);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

/*
 * toJSON output functions
 */
int _writeJSONValue(char *buf, const char *sensor_name, const char *unit, float value)
{
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

int _writeFullJSONEntry(char *buf, char *label, const char *sensor, unsigned char unit, float value)
{
  sprintf(buf, label, sensor);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], buf, unit_to_str(unit), value);
  return _output_buf_len;
}

const char * valueToJSON(const char *sensor_name, unsigned char unit, float value)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(unit), value);
  return _getOutBuf();
}

const char * accelerationToJSON(const char *sensor_name, acceleration_t & acceleration)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _resetOutBuf();

  _writeFullJSONEntry(nameBuf, "%sX", sensor_name, acceleration.header.unit, acceleration.x);
  _writeFullJSONEntry(nameBuf, "%sY", sensor_name, acceleration.header.unit, acceleration.y);
  _writeFullJSONEntry(nameBuf, "%sZ", sensor_name, acceleration.header.unit, acceleration.z);
  return _getOutBuf();
}

const char * magneticToJSON(const char *sensor_name, magnetic_t & mag)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _resetOutBuf();

  _writeFullJSONEntry(nameBuf, "%sX", sensor_name, mag.header.unit, mag.x);
  _writeFullJSONEntry(nameBuf, "%sY", sensor_name, mag.header.unit, mag.y);
  _writeFullJSONEntry(nameBuf, "%sZ", sensor_name, mag.header.unit, mag.z);
  return _getOutBuf();
}

const char * gyroToJSON(const char *sensor_name, gyro_t & orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _resetOutBuf();

  _writeFullJSONEntry(nameBuf, "%sX", sensor_name, orient.header.unit, orient.x);
  _writeFullJSONEntry(nameBuf, "%sY", sensor_name, orient.header.unit, orient.y);
  _writeFullJSONEntry(nameBuf, "%sZ", sensor_name, orient.header.unit, orient.z);
  return _getOutBuf();
}

const char * temperatureToJSON(const char *sensor_name, temperature_t & temp)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(temp.header.unit), temp.t);
  return _getOutBuf();
}

const char * luminosityToJSON(const char *sensor_name, luminosity_t & lum)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(lum.header.unit), lum.lux);
  return _getOutBuf();
}

const char * uvlightToJSON(const char *sensor_name, uvlight_t & input)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(input.header.unit), input.uvindex);
  return _getOutBuf();
}

const char * orientationToJSON(const char *sensor_name, orientation_t & orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _resetOutBuf();

  _writeFullJSONEntry(nameBuf, "%sRoll", sensor_name, orient.header.unit, orient.roll);
  _writeFullJSONEntry(nameBuf, "%sPitch", sensor_name, orient.header.unit, orient.pitch);
  _writeFullJSONEntry(nameBuf, "%sHeading", sensor_name, orient.header.unit, orient.heading);
  return _getOutBuf();
}

const char * rgblightToJSON(const char *sensor_name, rgblight_t & rgblight)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _resetOutBuf();

  _writeFullJSONEntry(nameBuf, "%sRed", sensor_name, rgblight.header.unit, rgblight.red);
  _writeFullJSONEntry(nameBuf, "%sGreen", sensor_name, rgblight.header.unit, rgblight.green);
  _writeFullJSONEntry(nameBuf, "%sBlue", sensor_name, rgblight.header.unit, rgblight.blue);
  return _getOutBuf();
}

const char * pressureToJSON(const char *sensor_name, pressure_t & pressure)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(pressure.header.unit), pressure.pressure);
  return _getOutBuf();
}
