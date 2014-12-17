/**
 * @file   ArdusatSDK.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor read & configuration functions
 */

#include <stdio.h>
#include <string.h>

#include "utility/drivers.h"
#include "ArdusatSDK.h"

char _output_buffer[OUTPUT_BUFFER_MAXSIZE];
int _output_buf_len = 0;

prog_char begin_error_msg[] PROGMEM = "Uh oh, begin%s failed. Check your wiring!";
prog_char orientation_sensor_name[] PROGMEM = "Orientation";
prog_char accel_sensor_name[] PROGMEM = "Acceleration";
prog_char mag_sensor_name[] PROGMEM = "Magnetic";
prog_char uv_sensor_name[] PROGMEM = "UVLight";
prog_char luminosity_sensor_name[] PROGMEM = "Luminosity";
prog_char temperature_sensor_name[] PROGMEM = "Temperature";
prog_char ir_temperature_sensor_name[] PROGMEM = "IRTemperature";

char CSV_SEPARATOR = ',';
char JSON_PREFIX = '~';
char JSON_SUFFIX = '|';
prog_char json_format[] PROGMEM = "%c{\"sensorName\":\"%s\", \"unit\":\"%s\", \"value\": %s}%c\n";

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
void _beginError(const prog_char *sensorName)
{
  char err_msg[50];
  char sensor[50];
  char output_buffer[100];
  int len;

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

void readUVLight(uvlight_t * output) {
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
  output->uvindex = ml8511_getUV();
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
 * Orientation
 */
boolean beginOrientationSensor() {
  start_sensor_or_err(orientation_sensor_name, l3gd20h_init())
}

void readOrientation(orientation_t * output) {
  if (output == NULL)
    return;

  //output->header.version = SENSORDATA_HEADER_VERSION;
  //output->header.length = sizeof(orientation_t);
  //output->header.dimensionality = 3;
  //output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_RADIAN_PER_SECOND;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  l3gd20h_getOrientation(&(output->x), &(output->y), &(output->z));
}

/*
 * toCSV Output functions
 */
void _output_buffer_reset() {
  memset(_output_buffer, 0, OUTPUT_BUFFER_MAXSIZE);
  _output_buf_len = 0;
}

const char * _headerToCSV(_data_header_t * header, const char *sensorName) {
  int name_len;

  if (header == NULL)
    return (NULL);

  _output_buffer_reset();

  ultoa(header->timestamp, _output_buffer, 10);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  if (sensorName != NULL) {
    if ((name_len = strlen(sensorName)) > 50) {
      name_len = 50;
    }

    memcpy(&(_output_buffer[_output_buf_len]), sensorName, name_len);
    _output_buf_len += name_len;
    _output_buffer[_output_buf_len++] = CSV_SEPARATOR;
  } else {
    utoa(header->sensor_id, _output_buffer + _output_buf_len, 10);
    _output_buf_len = strlen(_output_buffer);
    _output_buffer[_output_buf_len++] = CSV_SEPARATOR;
  }

  return _output_buffer;
}

const char * accelerationToCSV(const char *sensorName, acceleration_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->x, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char * magneticToCSV(const char *sensorName, magnetic_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->x, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char * temperatureToCSV(const char *sensorName, temperature_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->t, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char * orientationToCSV(const char *sensorName, orientation_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->x, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char * luminosityToCSV(const char *sensorName, luminosity_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->lux, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char * uvlightToCSV(const char *sensorName, uvlight_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV(&(input->header), sensorName);// warning : resets the internal buffer

  dtostrf(input->uvindex, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

/*
 * toJSON output functions 
 */
int _writeJSONValue(char *buf, const char *sensor_name, const char *unit, float value)
{
  char num [10];
  char format_str[80]; 
  // inexact estimate on the number of characters the value will take up...
  if (strlen(sensor_name) + strlen(unit) + 10 + _output_buf_len > OUTPUT_BUFFER_MAXSIZE) {
    return -1;
  }
  dtostrf(value, 4, 2, num);
  strcpy_P(format_str, json_format);
  _output_buf_len += sprintf(buf, format_str,
			     JSON_PREFIX, sensor_name, unit, num, JSON_SUFFIX);
  return _output_buf_len;
}

const char * valueToJSON(const char *sensor_name, uint8_t unit, float value)
{
  _output_buffer_reset();
  _writeJSONValue(_output_buffer, sensor_name, unit_to_str(unit), value); 
  return _output_buffer;
}

const char * accelerationToJSON(const char *sensor_name, acceleration_t *acceleration)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _output_buffer_reset();	

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_output_buffer, nameBuf, unit_to_str(acceleration->header.unit),
		  acceleration->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(acceleration->header.unit), acceleration->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(acceleration->header.unit), acceleration->z);
  return _output_buffer;
}

const char * magneticToJSON(const char *sensor_name, magnetic_t *mag)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _output_buffer_reset();	

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_output_buffer, nameBuf, unit_to_str(mag->header.unit),
		  mag->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(mag->header.unit), mag->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(mag->header.unit), mag->z);
  return _output_buffer;
}

const char * orientationToJSON(const char *sensor_name, orientation_t *orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _output_buffer_reset();	

  sprintf(nameBuf, "%sX", sensor_name);
  _writeJSONValue(_output_buffer, nameBuf, unit_to_str(orient->header.unit),
		  orient->x);
  sprintf(nameBuf, "%sY", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->y);
  sprintf(nameBuf, "%sZ", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->z);
  return _output_buffer;
}

const char * temperatureToJSON(const char *sensor_name, temperature_t *temp)
{
  _output_buffer_reset();
  _writeJSONValue(_output_buffer, sensor_name, unit_to_str(temp->header.unit), temp->t);
  return _output_buffer;
}

const char * luminosityToJSON(const char *sensor_name, luminosity_t *lum)
{
  _output_buffer_reset();
  _writeJSONValue(_output_buffer, sensor_name, unit_to_str(lum->header.unit), lum->lux);
  return _output_buffer;
}

const char * uvlightToJSON(const char *sensor_name, uvlight_t *input)
{
  _output_buffer_reset();
  _writeJSONValue(_output_buffer, sensor_name, unit_to_str(input->header.unit), input->uvindex);
  return _output_buffer;
}
