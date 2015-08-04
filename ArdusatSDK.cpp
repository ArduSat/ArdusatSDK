/**
 * @file   ArdusatSDK.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor read & configuration functions
 */

#include <stdio.h>
#include <string.h>

#include "ArdusatSDK.h"

int OUTPUT_BUF_SIZE = 256;
int OUTPUT_BUFFER_MAXSIZE = 250;
int _output_buf_len = 0;
bool ARDUSAT_SHIELD = false;
char * _output_buffer;

prog_char begin_error_msg[] = "Uh oh, begin%s failed. Check your wiring!";
prog_char orientation_sensor_name[] = "Orientation";
prog_char accel_sensor_name[] = "Acceleration";
prog_char mag_sensor_name[] = "Magnetic";
prog_char uv_sensor_name[] = "UVLight";
prog_char luminosity_sensor_name[] = "Luminosity";
prog_char temperature_sensor_name[] = "Temperature";
prog_char ir_temperature_sensor_name[] = "IRTemperature";
prog_char pressure_sensor_name[] = "BarometricPressureSensor";

static char CSV_SEPARATOR = ',';
static char JSON_PREFIX = '~';
static char JSON_SUFFIX = '|';
prog_char json_format[] = "%c{\"sensorName\":\"%s\",\"unit\":\"%s\",\"value\":%s,\"cs\":%d}%c\n";

char * _getOutBuf() {
  if (_output_buffer == NULL) {
    _output_buffer = new char[OUTPUT_BUF_SIZE];
  }
  return _output_buffer;
}

/**
 * Convert an enumerated unit code to a string representation.
 *
 * @param unit code (see units.h defines)
 *
 * @return string representation of unit
 */
const char * unit_to_str(uint8_t unit)
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
    case (DATA_UNIT_DEGREES_FAHRENHEIT):
      return "F";
    case (DATA_UNIT_METER_PER_SECOND):
      return "m/s";
    case (DATA_UNIT_LUX):
      return "lux";
    case (DATA_UNIT_MILLIWATT_PER_CMSQUARED):
      return "mW/cm^2";
    case (DATA_UNIT_RADIAN):
      return "rad";
    case (DATA_UNIT_HECTOPASCAL):
      return "hPa";
    default:
      return "";
  };
}

#define start_sensor_or_err(sensor_name, function) if (!function) { _beginError(sensor_name); return false;} \
						   else { return true; }

/**
 * Prints an error message if beginSensor function fails.
 *
 * This relies on a 128 character output buffer. Make sure that sensorName isn't too long!
 *
 * @param sensorName name of sensor that failed.
 */
void _beginError(prog_char *sensorName)
{
  char err_msg[50];
  char sensor[50];
  char output_buffer[100];

  strcpy_P(err_msg, begin_error_msg);
  strcpy_P(sensor, sensorName);

  //Make SURE sensorName isn't too long for the output buffer!!!
  sprintf(output_buffer, err_msg, sensor);
  Serial.println(output_buffer);
}

/*
 * Temperature
 */
boolean beginTemperatureSensor() {
  start_sensor_or_err(temperature_sensor_name, tmp102_init())
}

void readTemperature(temperature_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(temperature_t);
  //output->header.dimensionality = 1;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_DEGREES_CELSIUS;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_TMP102;
  output->t = tmp102_getTempCelsius();
}

/*
 * IR Temperature
 */
boolean beginInfraredTemperatureSensor() {
  start_sensor_or_err(ir_temperature_sensor_name, mlx90614_init())
}

void readInfraredTemperature(temperature_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(temperature_t);
  //output->header.dimensionality = 1;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_DEGREES_CELSIUS;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_MLX90614;
  output->t = mlx90614_getTempCelsius();
}

/*
 * Luminosity
 */
boolean beginLuminositySensor() {
#if defined(TSL2591_LUMINOSITY)
  start_sensor_or_err(luminosity_sensor_name, tsl2591_init())
#else
  start_sensor_or_err(luminosity_sensor_name, tsl2561_init())
#endif
}

void readLuminosity(luminosity_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(temperature_t);
  //output->header.dimensionality = 1;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_LUX;
  output->header.timestamp = millis();

#if defined(TSL2591_LUMINOSITY)
  output->header.sensor_id = SENSORID_TSL2591;
  output->lux = tsl2591_getLux();
#else
  output->header.sensor_id = SENSORID_TSL2561;
  output->lux = tsl2561_getLux();
#endif
}

/*
 * UV Light
 */
boolean beginUVLightSensor() {
#if defined(SI1145_UV_LIGHT)
  start_sensor_or_err(uv_sensor_name, si1145_init())
#else
  start_sensor_or_err(uv_sensor_name, ml8511_init())
#endif
}

void readUVLight(uvlight_t * output, int pin) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(temperature_t);
  //output->header.dimensionality = 1;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_MILLIWATT_PER_CMSQUARED;
  output->header.timestamp = millis();

#if defined(SI1145_UV_LIGHT)
  output->header.sensor_id = SENSORID_SI1145;
  output->uvindex = si1145_getUVIndex();
#else
  output->header.sensor_id = SENSORID_ML8511;
  output->uvindex = ml8511_getUV(pin);
#endif
}

/*
 * Acceleration
 */
boolean beginAccelerationSensor() {
  start_sensor_or_err(accel_sensor_name, lsm303_accel_init())
}

void readAcceleration(acceleration_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(acceleration_t);
  //output->header.dimensionality = 3;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  lsm303_getAccel(&(output->x), &(output->y), &(output->z));
}

/*
 * Magnetic Field
 */
boolean beginMagneticSensor() {
  start_sensor_or_err(mag_sensor_name, lsm303_mag_init())
}

void readMagnetic(magnetic_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(acceleration_t);
  //output->header.dimensionality = 3;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_MICROTESLA;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  lsm303_getMag(&(output->x), &(output->y), &(output->z));
}

/*
 * Gyroscope
 */
boolean beginGyroSensor() {
  start_sensor_or_err(orientation_sensor_name, l3gd20h_init())
}

void readGyro(gyro_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(gyro_t);
  //output->header.dimensionality = 3;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_RADIAN_PER_SECOND;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  l3gd20h_getOrientation(&(output->x), &(output->y), &(output->z));
}

/*
 * Barometric Pressure
 */
boolean beginBarometricPressureSensor() {
  start_sensor_or_err(pressure_sensor_name, bmp180_init())
}

void readBarometricPressure(pressure_t *output)
{
  if (output == NULL)
    return;
  output->header.unit = DATA_UNIT_HECTOPASCAL;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_BMP180;
  bmp180_getPressure(&output->pressure);
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
void calculateOrientation(const acceleration_t *accel, const magnetic_t *mag,
			  orientation_t *orient)
{
  const float PI_F = 3.141592653F;

  // Roll is rotation around x-axis (-180 <= roll <= 180)
  // Positive roll is clockwise rotation wrt positive x axis
  orient->roll = (float) atan2(accel->y, accel->z);

  // Pitch is rotation around y-axis (-180 <= pitch <= 180)
  // Positive pitch is clockwise rotation wrt positive y axis
  if (accel->y * sin(orient->roll) + accel->z * cos(orient->roll) == 0) {
    orient->pitch = accel->x > 0 ? (PI_F / 2) : (-PI_F / 2);
  } else {
    orient->pitch = (float)atan(-accel->x / (accel->y * sin(orient->roll) +
					     accel->z * cos(orient->roll)));
  }

  // Heading is rotation around z-axis
  // Positive heading is clockwise rotation wrt positive z axis
  orient->heading = (float)atan2(mag->z * sin(orient->roll) - mag->y * cos(orient->roll),
			         mag->x * cos(orient->pitch) + mag->y * sin(orient->pitch) * sin(orient->roll) +
				 mag->z * sin(orient->pitch) * cos(orient->roll));

  // Convert radians to degrees
  orient->roll = orient->roll * 180 / PI_F;
  orient->pitch = orient->pitch * 180 / PI_F;
  orient->heading = orient->heading * 180 / PI_F;
  orient->header.unit = DATA_UNIT_DEGREES;
  orient->header.timestamp = accel->header.timestamp > mag->header.timestamp ? accel->header.timestamp :
									       mag->header.timestamp;
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
  int cs = 0;
  const char *c_ptr = sensor_name;

  va_list values;
  va_start(values, num_vals);
  for (int i = 0; i < num_vals; ++i)
  {
    cs += lroundf(va_arg(values, double));
  }
  va_end(values);

  while (*c_ptr != NULL) {
    cs += *c_ptr++;
  }
  return cs;
}

/*
 * toCSV Output functions
 */
void _resetOutBuf() {
  memset(_getOutBuf(), 0, OUTPUT_BUF_SIZE);
  _output_buf_len = 0;
}

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

#define add_float_to_csv_buffer(val) \
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR; \
  dtostrf(val, 2, 3, _getOutBuf() + _output_buf_len);\
  _output_buf_len = strlen(_getOutBuf())

#define _add_checksum_to_csv_buffer(sensor_name, num_vals, ...) \
  int cs = calculateCheckSum(sensor_name, num_vals, __VA_ARGS__); \
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR; \
  itoa(cs, _getOutBuf() + _output_buf_len, 10); \
  _output_buf_len = strlen(_getOutBuf());

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
    if (_output_buf_len > OUTPUT_BUFFER_MAXSIZE - 10) {
      break;
    }
    add_float_to_csv_buffer(va_arg(args, double));
  }
  va_end(args);

  va_start(args, numValues);
  _add_checksum_to_csv_buffer(sensorName, numValues, args);
  va_end(args);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * accelerationToCSV(const char *sensorName, acceleration_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->x, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 3, input->x, input->y, input->z);
  _getOutBuf()[_output_buf_len++] = '\n';
  return _getOutBuf();
}

const char * magneticToCSV(const char *sensorName, magnetic_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->x, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 3, input->x, input->y, input->z);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * temperatureToCSV(const char *sensorName, temperature_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->t, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 1, input->t);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * gyroToCSV(const char *sensorName, gyro_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->x, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());

  _add_checksum_to_csv_buffer(sensorName, 3, input->x, input->y, input->z);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * luminosityToCSV(const char *sensorName, luminosity_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->lux, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _add_checksum_to_csv_buffer(sensorName, 1, input->lux);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char * uvlightToCSV(const char *sensorName, uvlight_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->uvindex, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _add_checksum_to_csv_buffer(sensorName, 1, input->uvindex);
  _getOutBuf()[_output_buf_len++] = '\n';

  return _getOutBuf();
}

const char *orientationToCSV(const char *sensorName, orientation_t *input)
{
  if (input == NULL)
    return NULL;

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->roll, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->pitch, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _getOutBuf()[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->heading, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _add_checksum_to_csv_buffer(sensorName, 3, input->roll, input->pitch, input->heading);
  _getOutBuf()[_output_buf_len++] = '\n';
  return _getOutBuf();
}

const char *pressureToCSV(const char *sensorName, pressure_t *input)
{
  if (input == NULL)
    return NULL;

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->pressure, 2, 3, _getOutBuf() + _output_buf_len);
  _output_buf_len = strlen(_getOutBuf());
  _add_checksum_to_csv_buffer(sensorName, 1, input->pressure);
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
  if (strlen(sensor_name) + strlen(unit) + 10 + _output_buf_len > OUTPUT_BUFFER_MAXSIZE) {
    return -1;
  }
  dtostrf(value, 4, 2, num);
  strcpy_P(format_str, json_format);
  _output_buf_len += sprintf(buf, format_str,
			     JSON_PREFIX, sensor_name, unit, num,
			     calculateCheckSum(sensor_name, 1, value), JSON_SUFFIX);
  return _output_buf_len;
}

const char *valueToJSON(const char *sensor_name, uint8_t unit, float value)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(unit), value);
  return _getOutBuf();
}

const char *accelerationToJSON(const char *sensor_name, acceleration_t *acceleration)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _resetOutBuf();

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_getOutBuf(), nameBuf, unit_to_str(acceleration->header.unit),
		  acceleration->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(acceleration->header.unit), acceleration->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(acceleration->header.unit), acceleration->z);
  return _getOutBuf();
}

const char *magneticToJSON(const char *sensor_name, magnetic_t *mag)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _resetOutBuf();

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_getOutBuf(), nameBuf, unit_to_str(mag->header.unit),
		  mag->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(mag->header.unit), mag->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(mag->header.unit), mag->z);
  return _getOutBuf();
}

const char *gyroToJSON(const char *sensor_name, gyro_t *orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _resetOutBuf();

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_getOutBuf(), nameBuf, unit_to_str(orient->header.unit),
		  orient->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->z);
  return _getOutBuf();
}

const char *temperatureToJSON(const char *sensor_name, temperature_t *temp)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(temp->header.unit), temp->t);
  return _getOutBuf();
}

const char *luminosityToJSON(const char *sensor_name, luminosity_t *lum)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(lum->header.unit), lum->lux);
  return _getOutBuf();
}

const char *uvlightToJSON(const char *sensor_name, uvlight_t *input)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensor_name, unit_to_str(input->header.unit), input->uvindex);
  return _getOutBuf();
}

const char *orientationToJSON(const char *sensor_name, orientation_t *orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _resetOutBuf();

  sprintf(nameBuf, "%sRoll", sensor_name);
  _writeJSONValue(_getOutBuf(), nameBuf, unit_to_str(orient->header.unit),
		  orient->roll);
  sprintf(nameBuf, "%sPitch", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->pitch);
  sprintf(nameBuf, "%sHeading", sensor_name);
  _writeJSONValue(&_getOutBuf()[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->heading);
  return _getOutBuf();
}

const char *pressureToJSON(const char *sensorName, pressure_t *pressure)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensorName, unit_to_str(pressure->header.unit), pressure->pressure);
  return _getOutBuf();
}
