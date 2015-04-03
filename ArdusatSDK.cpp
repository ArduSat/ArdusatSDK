/**
 * @file   ArdusatSDK.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor read & configuration functions
 */

#include <stdio.h>
#include <string.h>

#include "ArdusatSDK.h"

RTC_DS1307 RTC;

SdVolume vol;
#define _output_buffer vol.cacheAddress()->output_buf
int _output_buf_len = 0;

bool ARDUSAT_SHIELD = false;

prog_char begin_error_msg[] = "Uh oh, begin%s failed. Check your wiring!";
prog_char orientation_sensor_name[] = "Orientation";
prog_char accel_sensor_name[] = "Acceleration";
prog_char mag_sensor_name[] = "Magnetic";
prog_char uv_sensor_name[] = "UVLight";
prog_char luminosity_sensor_name[] = "Luminosity";
prog_char temperature_sensor_name[] = "Temperature";
prog_char ir_temperature_sensor_name[] = "IRTemperature";
prog_char pressure_sensor_name[] = "BarometricPressureSensor";

SdFat sd;
File file;
prog_char sd_card_error[] = "Not enough RAM for SD card sys(free: ";

static char CSV_SEPARATOR = ',';
static char JSON_PREFIX = '~';
static char JSON_SUFFIX = '|';
prog_char json_format[] = "%c{\"sensorName\":\"%s\", \"unit\":\"%s\", \"value\": %s}%c\n";

prog_char csv_header_fmt[] = "timestamp: %lu at millis %lu\n";

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

/*
 * toCSV Output functions
 */
void _output_buffer_reset() {
  memset(_output_buffer, 0, 512);
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

/*
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
  return valuesToCSV(sensorName, &value, 1, timestamp);
}

/*
 * Create a CSV string with a generic array of float values and a sensor name. Optional timestamp
 * argument allows passing in a timestamp; will use millis() otherwise.
 *
 * @param sensorName string sensor name
 * @param value array of float values to write
 * @param numValues number of floats in array
 * @param timestamp optional timestamp. If 0, millis() will be called.
 *
 * @return pointer to output buffer
 */
const char * valuesToCSV(const char *sensorName, float values[], int numValues, unsigned long timestamp)
{
  int i, name_len;

  if (values == NULL)
    return NULL;

  if (timestamp == 0) {
    timestamp = millis();
  }

  _output_buffer_reset();

  ultoa(timestamp, _output_buffer, 10);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  if (sensorName != NULL) {
    if ((name_len = strlen(sensorName)) > 50) {
      name_len = 50;
    }
    memcpy(&(_output_buffer[_output_buf_len]), sensorName, name_len);
    _output_buf_len += name_len;
    _output_buffer[_output_buf_len++] = CSV_SEPARATOR;
  }

  for (i = 0; i < numValues; ++i) {
    if (_output_buf_len > OUTPUT_BUFFER_MAXSIZE - 10) {
      break;
    }
    dtostrf(values[i], 2, 3, _output_buffer + _output_buf_len);
    _output_buf_len = strlen(_output_buffer);
    if (i != numValues - 1) {
      _output_buffer[_output_buf_len++] = CSV_SEPARATOR;
    }
  }

  _output_buffer[_output_buf_len++] = '\n';

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

const char * gyroToCSV(const char *sensorName, gyro_t * input) {
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

const char *orientationToCSV(const char *sensorName, orientation_t *input)
{
  if (input == NULL)
    return NULL;

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->roll, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->pitch, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = CSV_SEPARATOR;

  dtostrf(input->heading, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
}

const char *pressureToCSV(const char *sensorName, pressure_t *input)
{
  if (input == NULL)
    return NULL;

  _headerToCSV(&(input->header), sensorName);

  dtostrf(input->pressure, 2, 3, _output_buffer + _output_buf_len);
  _output_buf_len = strlen(_output_buffer);
  _output_buffer[_output_buf_len++] = '\n';

  return _output_buffer;
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

const char * gyroToJSON(const char *sensor_name, gyro_t *orient)
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

const char *orientationToJSON(const char *sensor_name, orientation_t *orient)
{
  int nameLength = strlen(sensor_name);
  char nameBuf[nameLength + 8];
  _output_buffer_reset();

  sprintf(nameBuf, "%sRoll", sensor_name);
  _writeJSONValue(_output_buffer, nameBuf, unit_to_str(orient->header.unit),
		  orient->roll);
  sprintf(nameBuf, "%sPitch", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->pitch);
  sprintf(nameBuf, "%sHeading", sensor_name);
  _writeJSONValue(&_output_buffer[_output_buf_len], nameBuf,
		  unit_to_str(orient->header.unit), orient->heading);
  return _output_buffer;
}

const char *pressureToJSON(const char *sensorName, pressure_t *pressure)
{
  _output_buffer_reset();
  _writeJSONValue(_output_buffer, sensorName, unit_to_str(pressure->header.unit), pressure->pressure);
  return _output_buffer;
}

#define write_if_init(gen_fn) return writeString(gen_fn);

/**
 * Helper function to write null-terminated output buffer string to file. 
 *
 * @param output_buf pointer to output buffer to write.
 *
 * @return number of bytes written
 */
int writeString(const char *output_buf)
{
  int buf_len = strlen(output_buf);
  return writeBytes((const uint8_t *) output_buf, buf_len);
}

/**
 * Writes the byte array pointed to by buffer to the SD card. Note that there is
 * absolutely no safety checking on numBytes, so use with care.
 *
 * Since we use a shared buffer between the SD card library and many of the output
 * functions, performs a check first to see if we need to allocate a new buffer.
 *
 * @param buffer byte array to write
 * @param number of bytes to write
 * 
 * @return number of bytes written
 */
int writeBytes(const uint8_t *buffer, uint8_t numBytes)
{
  uint8_t *buf;
  int written;
  bool buf_allocated = false;

  if (numBytes > OUTPUT_BUFFER_MAXSIZE) {
    numBytes = OUTPUT_BUFFER_MAXSIZE;
  }

  if (file.isOpen()) {
    if (buffer == (uint8_t *) vol.cacheAddress()->output_buf) {
      buf = (uint8_t *) malloc(numBytes);
      buf_allocated = true;
      memcpy(buf, buffer, numBytes);
    } else {
      buf = (uint8_t *) buffer;
    }

    written = file.write(buf, numBytes);
    file.sync();
    if (buf_allocated)
      free(buf);
    return written;
  } else {
    return 0;
  }
}

/*
 * Helper function to write to the top of the CSV header with the current time 
 */
int _write_csv_time_header(DateTime *now, unsigned long curr_millis)
{
  char fmt_buf[32];

  strcpy_P(fmt_buf, csv_header_fmt);
  _output_buffer_reset();
  sprintf(_output_buffer, fmt_buf, now->unixtime(), curr_millis);
  write_if_init(_output_buffer);
}

int _write_binary_time_header(DateTime *now, unsigned long curr_millis)
{
  uint8_t buf[10];
  uint32_t unixtime = now->unixtime();
  
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  memcpy(buf + 2, &unixtime, 4);
  memcpy(buf + 6, &curr_millis, 4);
  return writeBytes(buf, 10);
}

/**
 * Writes a line of CSV formatted acceleration data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data acceleration_t data to write
 *
 * @return number of bytes written
 */
int writeAcceleration(const char *sensorName, acceleration_t *data)
{
  write_if_init(accelerationToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted magnetic data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data magnetic_t data to write
 *
 * @return number of bytes written
 */
int writeMagnetic(const char *sensorName, magnetic_t *data)
{
  write_if_init(magneticToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted orientation data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data gyro_t data to write
 *
 * @return number of bytes written
 */
int writeGyro(const char *sensorName, gyro_t *data)
{
  write_if_init(gyroToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted tempoerature data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data temperature_t data to write
 *
 * @return number of bytes written
 */
int writeTemperature(const char *sensorName, temperature_t *data)
{
  write_if_init(temperatureToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted luminosity data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data luminosity_t data to write
 *
 * @return number of bytes written
 */
int writeLuminosity(const char *sensorName, luminosity_t *data)
{
  write_if_init(luminosityToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted UV light data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data uvlight_t data to write
 *
 * @return number of bytes written
 */
int writeUVLight(const char *sensorName, uvlight_t *data)
{
  write_if_init(uvlightToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted orientation data to SD card.
 *
 * @param sensorName of this sensor
 * @param data orientation_t data to write
 *
 * @return number of bytes written
 */
int writeOrientation(const char *sensorName, orientation_t *data)
{
  write_if_init(orientationToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted pressure data to SD card.
 *
 * @param sensorName of this sensor
 * @param data pressure_t data to write
 *
 * @return number of bytes written
 */
int writePressure(const char *sensorName, pressure_t *data)
{
  write_if_init(pressureToCSV(sensorName, data))
}

#define init_data_struct(type_def, type_enum) \
  type_def bin_data; \
  bin_data.type = type_enum; \
  bin_data.id = sensorId; \
  bin_data.timestamp = data->header.timestamp;

#define _write_binary_data_struct(type) return writeBytes((uint8_t *) &bin_data, sizeof(type));

int binaryWriteAcceleration(const uint8_t sensorId, acceleration_t *data)
{
  init_data_struct(acceleration_bin_t, ARDUSAT_SENSOR_TYPE_ACCELERATION)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(acceleration_bin_t)
}

int binaryWriteMagnetic(const uint8_t sensorId, magnetic_t *data)
{
  init_data_struct(magnetic_bin_t, ARDUSAT_SENSOR_TYPE_MAGNETIC)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(magnetic_bin_t)
}

int binaryWriteGyro(const uint8_t sensorId, gyro_t *data)
{
  init_data_struct(gyro_bin_t, ARDUSAT_SENSOR_TYPE_GYRO)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(gyro_bin_t)
}

int binaryWriteTemperature(const uint8_t sensorId, temperature_t *data)
{
  init_data_struct(temperature_bin_t, ARDUSAT_SENSOR_TYPE_TEMPERATURE)
  bin_data.temp = data->t;

  _write_binary_data_struct(temperature_bin_t)
}

int binaryWriteLuminosity(const uint8_t sensorId, luminosity_t *data)
{
  init_data_struct(luminosity_bin_t, ARDUSAT_SENSOR_TYPE_LUMINOSITY)
  bin_data.luminosity = data->lux;

  _write_binary_data_struct(luminosity_bin_t)
}

int binaryWriteUVLight(const uint8_t sensorId, uvlight_t *data)
{
  init_data_struct(uv_light_bin_t, ARDUSAT_SENSOR_TYPE_UV)
  bin_data.uv = data->uvindex;

  _write_binary_data_struct(uv_light_bin_t)
}

int binaryWriteOrientation(const uint8_t sensorId, orientation_t *data)
{
  init_data_struct(orientation_bin_t, ARDUSAT_SENSOR_TYPE_ORIENTATION)
  bin_data.roll = data->roll;
  bin_data.pitch = data->pitch;
  bin_data.heading = data->heading;

  _write_binary_data_struct(orientation_bin_t)
}

int binaryWritePressure(const uint8_t sensorId, pressure_t *data)
{
  init_data_struct(pressure_bin_t, ARDUSAT_SENSOR_TYPE_PRESSURE)
  bin_data.pressure = data->pressure;

  _write_binary_data_struct(pressure_bin_t)
}

/**
 * Function starts the SD card service and makes sure that the appropriate directory
 * structure exists, then creates the file to use for logging data. Data will be logged
 * to a file called fileNamePrefix_0.csv or fileNamePrefix_0.bin (number will be incremented
 * for each new log file)
 * 
 * @param chipSelectPin Arduino pin SD card reader CS pin is attached to
 * @param fileNamePrefix string to prefix at beginning of data file
 * @param csvData boolean flag whether we are writing csv data or binary data (used for filename)
 *
 * @return true if successful, false if failed
 */
bool beginDataLog(int chipSelectPin, const char *fileNamePrefix, bool csvData)
{
  bool ret;
  int i = 0;
  int max_len;
  char fileName[19];
  char prefix[8];
  char rootPath[] = "/data";
  unsigned long curr_millis = 0;
  DateTime now;

  // Try to get the current time from the RTC, if available. This will be prepended to the log file
  // to be used to convert relative timestamps to real time values.
  Wire.begin();
  RTC.begin();
  if (RTC.isrunning()) {
    now = RTC.now();
    curr_millis = millis();
  }

  if (freeMemory() < 400) {
    strcpy_P(_output_buffer, sd_card_error);
    Serial.print(_output_buffer);
    Serial.print(freeMemory());
    Serial.println(", need 400)");
    ret = false;
  } else {
    ret = sd.begin(chipSelectPin, SPI_FULL_SPEED);
  }

  //Filenames need to fit the 8.3 filename convention, so truncate down the 
  //given filename if it is too long.
  memcpy(prefix, fileNamePrefix, 7);
  prefix[7] = '\0';

  if (ret) {
    if (!sd.exists(rootPath))
      ret = sd.mkdir(rootPath);
    if (ret) {
      while (true) {
	if (i < 10) {
	  max_len = 7;
	} else if (i < 100) {
	  max_len = 6;
	} else if (i < 1000) {
	  max_len = 5;
	} else {
	  break;
	}

	prefix[max_len - 1] = '\0';
	sprintf(fileName, "%s/%s%d.%s", rootPath, prefix, i, 
		csvData ? "csv" : "bin");
	if (!sd.exists(fileName)) {
	  file = sd.open(fileName, FILE_WRITE);
	  break;
	}
	i++;
      }
    }
  } else {
    sd.initErrorPrint();
  }

  if (RTC.isrunning() && file.isOpen()) {
    if (csvData) {
      _write_csv_time_header(&now, curr_millis);
    } else {
      _write_binary_time_header(&now, curr_millis);
    }
  }

  return file.isOpen();
}

bool setRTC()
{
  Wire.begin();
  RTC.begin();
  // Sets RTC to the date & time sketch was compiled
  RTC.adjust(DateTime(__DATE__, __TIME__));
  return true;
}

