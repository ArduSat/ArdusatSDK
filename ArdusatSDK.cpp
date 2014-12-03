/**
 * @file   ArdusatSDK.c
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor read & configuration functions
 */

#include "ArdusatSDK.h"
#include <stdio.h>

#include "Ardusat_Adafruit_Sensor.h"
#include "Ardusat_Adafruit_LSM303_U.h"
#include "Ardusat_Adafruit_L3GD20_U.h"
#include "Ardusat_Adafruit_9DOF.h"
#include "Ardusat_Adafruit_SI1145.h"
#include "drivers.h"

char _output_buffer[OUTPUT_BUFFER_MAXSIZE];
int _output_buf_len = 0;

char CSV_SEPARATOR = ';';
char JSON_PREFIX = '~';
char JSON_SUFFIX = '|';

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

/*
 * Temperature 
 */
boolean beginTemperatureSensor() {
  return tmp102_init();
}

void readTemperature(temperature_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(temperature_t);
  output->header.dimensionality = 1;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_DEGREES_CELSIUS;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_TMP102;
  output->t = tmp102_getTempCelsius();
}

/*
 * IR Temperature
 */
boolean beginInfraredTemperatureSensor() {
  return mlx90614_init();
}

void readInfraredTemperature(temperature_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(temperature_t);
  output->header.dimensionality = 1;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
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
  return(tsl2591_init());
#else
  return tsl2561_init();
#endif
}

void readLuminosity(luminosity_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(temperature_t);
  output->header.dimensionality = 1;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
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
	return si1145_init();
#else
	return ml8511_init();
#endif
}

void readUVLight(uvlight_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(temperature_t);
  output->header.dimensionality = 1;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
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
  return adafruit9dof_init();
}

void readAcceleration(acceleration_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(acceleration_t);
  output->header.dimensionality = 3;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  adafruit9dof_getAccel(&(output->x), &(output->y), &(output->z));
}

/*
 * Magnetic Field
 */
boolean beginMagneticSensor() {
  return adafruit9dof_init();
}

void readMagnetic(magnetic_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(acceleration_t);
  output->header.dimensionality = 3;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_MICROTESLA;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  adafruit9dof_getMag(&(output->x), &(output->y), &(output->z));
}

/*
 * Orientation
 */
boolean beginOrientationSensor() {
  return adafruit9dof_init();
}

void readOrientation(orientation_t * output) {
  if (output == NULL)
    return;

  output->header.version = SENSORDATA_HEADER_VERSION;
  output->header.length = sizeof(orientation_t);
  output->header.dimensionality = 3;
  output->header.celltype = DATA_CELLTYPE_FLOAT;
  output->header.unit = DATA_UNIT_RADIAN;
  output->header.timestamp = millis();

  output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
  adafruit9dof_getOrientation(&(output->roll), &(output->pitch), &(output->heading));
}

/*
 * toCSV Output functions
 */
void _output_buffer_reset() {
  memset(_output_buffer, 0, OUTPUT_BUFFER_MAXSIZE);
  _output_buf_len = 0;
}

const char * _headerToCSV(_data_header_t * header) {
  if (header == NULL)
    return (NULL);

  _output_buffer_reset();

  ultoa(header->timestamp, _output_buffer, 10);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  utoa(header->sensor_id, _output_buffer + _output_buf_len, 10);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return ((const char *) _output_buffer);
}

const char * accelerationToCSV(acceleration_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV((_data_header_t*) input);

  dtostrf(input->x, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->y, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->z, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return (_output_buffer);
}

const char * temperatureToCSV(temperature_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

  dtostrf(input->t, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return (_output_buffer);
}

const char * orientationToCSV(orientation_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

  dtostrf(input->pitch, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->roll, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->heading, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return (_output_buffer);
}

const char * luminosityToCSV(luminosity_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

  dtostrf(input->lux, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return (_output_buffer);
}

const char * uvlightToCSV(uvlight_t * input) {
  if (input == NULL)
    return (NULL);

  _headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

  dtostrf(input->uvindex, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  return (_output_buffer);
}

/*
 * toJSON output functions 
 */
int _writeJSONValue(char *buf, const char *sensor_name, const char *unit, float value)
{
  char num [10];
  // inexact estimate on the number of characters the value will take up...
  if (strlen(sensor_name) + strlen(unit) + 10 + _output_buf_len > OUTPUT_BUFFER_MAXSIZE) {
    return -1;
  }
  dtostrf(value, 4, 2, num);
  _output_buf_len += sprintf(buf, "%c{\"sensorName\":\"%s\", \"unit\":\"%s\", \"value\": %s}%c\n",
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

const char * orientationToJSON(const char *sensor_name, orientation_t *orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 2];
  _output_buffer_reset();	

  sprintf(nameBuf, "%sRoll", sensor_name);
  _writeJSONValue(_output_buffer, "roll", unit_to_str(orient->header.unit),
		  orient->roll);
  sprintf(nameBuf, "%sPitch", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->pitch);
  sprintf(nameBuf, "%sHeading", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->heading);
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
