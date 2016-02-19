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

const char * _tripleValueToJSON(const char *sensorName, int labelLength, unsigned char unit,
             char *labelA, float valueA, char *labelB, float valueB, char *labelC, float valueC)
{
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

const char * valueToJSON(const char *sensorName, unsigned char unit, float value)
{
  _resetOutBuf();
  _writeJSONValue(_getOutBuf(), sensorName, unit_to_str(unit), value);
  return _getOutBuf();
}

/*
 * UV Light
 * ========================================================================================
 */
UVLight::UVLight(sensor_id_t sensor_id) {
  UVLight::sensorId = sensor_id;
  UVLight::header.unit = DATA_UNIT_MILLIWATT_PER_CMSQUARED;
}

UVLight::~UVLight() {
}

boolean UVLight::begin(void) {
  return UVLight::begin(UVLight::ML8511_pin);
}

boolean UVLight::begin(int pin) {
  UVLight::ML8511_pin = pin;
  if (UVLight::sensorId == SENSORID_SI1132)
    return start_sensor_or_err(uvlight_sensor_name, si1132_init());
  else if (UVLight::sensorId == SENSORID_ML8511)
    return start_sensor_or_err(uvlight_sensor_name, ml8511_init());
  else
    return false;
}

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

const char * UVLight::readToCSV(const char * sensorName) {
  UVLight::read();
  return UVLight::toCSV(sensorName);
}

const char * UVLight::readToJSON(const char * sensorName) {
  UVLight::read();
  return UVLight::toJSON(sensorName);
}

const char * UVLight::toCSV(const char * sensorName) {
  return valueToCSV(sensorName, UVLight::uvindex, UVLight::header.timestamp);
}

const char * UVLight::toJSON(const char * sensorName) {
  return valueToJSON(sensorName, UVLight::header.unit, UVLight::uvindex);
}


/*
 * RGB Light
 * ========================================================================================
 */
RGBLight::RGBLight(sensor_id_t sensor_id) {
  RGBLight::sensorId = sensor_id;
  RGBLight::header.unit = DATA_UNIT_LUX;
}

RGBLight::~RGBLight() {
}

boolean RGBLight::begin(void) {
  if (RGBLight::sensorId == SENSORID_ISL29125)
    // TODO: Support advance sensor configuration:
    //       https://github.com/sparkfun/ISL29125_Breakout/blob/V_H1.0_L1.0.1/Libraries/
    //               Arduino/examples/ISL29125Interrupts/ISL29125Interrupts.ino
    return start_sensor_or_err(rgblight_sensor_name, isl29125_init());
  else if (RGBLight::sensorId == SENSORID_TCS34725)
    // TODO: Support advance sensor configuration:
    //       https://learn.adafruit.com/adafruit-color-sensors/program-it
    return start_sensor_or_err(rgblight_sensor_name, tcs34725_init(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X));
  else
    return false;
}

void RGBLight::read(void) {
  RGBLight::header.timestamp = millis();
  RGBLight::header.sensor_id = RGBLight::sensorId;

  // TODO: Adjust RGB for relative luminance?
  //       Y = 0.2126 * R + 0.7152 * G + 0.0722 * B

  if (RGBLight::sensorId == SENSORID_ISL29125)
    isl29125_getRGB(&(RGBLight::red), &(RGBLight::green), &(RGBLight::blue));
  else if (RGBLight::sensorId == SENSORID_TCS34725)
    tcs34725_getRGB(&(RGBLight::red), &(RGBLight::green), &(RGBLight::blue));
}

const char * RGBLight::readToCSV(const char * sensorName) {
  RGBLight::read();
  return RGBLight::toCSV(sensorName);
}

const char * RGBLight::readToJSON(const char * sensorName) {
  RGBLight::read();
  return RGBLight::toJSON(sensorName);
}

const char * RGBLight::toCSV(const char * sensorName) {
  return valuesToCSV(sensorName, RGBLight::header.timestamp, 3,
                     RGBLight::red, RGBLight::green, RGBLight::blue);
}

const char * RGBLight::toJSON(const char * sensorName) {
  return _tripleValueToJSON(sensorName, 6, RGBLight::header.unit, "%sRed", RGBLight::red,
                           "%sGreen", RGBLight::green, "%sBlue", RGBLight::blue);
}

/*
 * Temperature
 * ========================================================================================
 */
Temperature::Temperature(sensor_id_t sensor_id) {
  Temperature::sensorId = sensor_id;
  Temperature::header.unit = DATA_UNIT_DEGREES_CELSIUS;
}

Temperature::~Temperature() {
}

boolean Temperature::begin(void) {
  if (Temperature::sensorId == SENSORID_TMP102)
    return start_sensor_or_err(temperature_sensor_name, tmp102_init());
  else if (Temperature::sensorId == SENSORID_MLX90614)
    return start_sensor_or_err(ir_temperature_sensor_name, mlx90614_init());
  else
    return false;
}

void Temperature::read(void) {
  Temperature::header.timestamp = millis();
  Temperature::header.sensor_id = Temperature::sensorId;

  if (Temperature::sensorId == SENSORID_TMP102)
    Temperature::t = tmp102_getTempCelsius();
  else if (Temperature::sensorId == SENSORID_MLX90614)
    Temperature::t = mlx90614_getTempCelsius();
}

const char * Temperature::readToCSV(const char * sensorName) {
  Temperature::read();
  return Temperature::toCSV(sensorName);
}

const char * Temperature::readToJSON(const char * sensorName) {
  Temperature::read();
  return Temperature::toJSON(sensorName);
}

const char * Temperature::toCSV(const char * sensorName) {
  return valueToCSV(sensorName, Temperature::t, Temperature::header.timestamp);
}


const char * Temperature::toJSON(const char * sensorName) {
  return valueToJSON(sensorName, Temperature::header.unit, Temperature::t);
}

/*
 * Luminosity
 * ========================================================================================
 */
Luminosity::Luminosity(void) {
  Luminosity::sensorId = SENSORID_TSL2561;
  Luminosity::header.unit = DATA_UNIT_LUX;
}

Luminosity::~Luminosity() {
}

boolean Luminosity::begin(void) {
  return start_sensor_or_err(luminosity_sensor_name, tsl2561_init());
}

void Luminosity::read(void) {
  Luminosity::header.timestamp = millis();

  Luminosity::header.sensor_id = Luminosity::sensorId;
  Luminosity::lux = tsl2561_getLux();
}

const char * Luminosity::readToCSV(const char * sensorName) {
  Luminosity::read();
  return Luminosity::toCSV(sensorName);
}

const char * Luminosity::readToJSON(const char * sensorName) {
  Luminosity::read();
  return Luminosity::toJSON(sensorName);
}

const char * Luminosity::toCSV(const char * sensorName) {
  return valueToCSV(sensorName, Luminosity::lux, Luminosity::header.timestamp);
}

const char * Luminosity::toJSON(const char * sensorName) {
  return valueToJSON(sensorName, Luminosity::header.unit, Luminosity::lux);
}

/*
 * Pressure
 * ========================================================================================
 */
Pressure::Pressure(void) {
  Pressure::sensorId = SENSORID_BMP180;
  Pressure::header.unit = DATA_UNIT_HECTOPASCAL;
}

Pressure::~Pressure() {
}

boolean Pressure::begin(void) {
  return Pressure::begin(Pressure::bmp085_mode);
}

boolean Pressure::begin(bmp085_mode_t mode) {
  return start_sensor_or_err(pressure_sensor_name, bmp180_init(mode));
}

void Pressure::read(void) {
  Pressure::header.timestamp = millis();

  Pressure::header.sensor_id = Pressure::sensorId;
  bmp180_getPressure(&(Pressure::pressure));
}

const char * Pressure::readToCSV(const char * sensorName) {
  Pressure::read();
  return Pressure::toCSV(sensorName);
}

const char * Pressure::readToJSON(const char * sensorName) {
  Pressure::read();
  return Pressure::toJSON(sensorName);
}

const char * Pressure::toCSV(const char * sensorName) {
  return valueToCSV(sensorName, Pressure::pressure, Pressure::header.timestamp);
}

const char * Pressure::toJSON(const char * sensorName) {
  return valueToJSON(sensorName, Pressure::header.unit, Pressure::pressure);
}

/*
 * Acceleration
 * ========================================================================================
 */
Acceleration::Acceleration(void) {
  Acceleration::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Acceleration::header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
}

Acceleration::~Acceleration() {
}

boolean Acceleration::begin(void) {
  return start_sensor_or_err(acceleration_sensor_name, lsm303_accel_init());
}

void Acceleration::read(void) {
  Acceleration::header.timestamp = millis();

  Acceleration::header.sensor_id = Acceleration::sensorId;
  lsm303_getAccel(&(Acceleration::x), &(Acceleration::y), &(Acceleration::z));
}

const char * Acceleration::readToCSV(const char * sensorName) {
  Acceleration::read();
  return Acceleration::toCSV(sensorName);
}

const char * Acceleration::readToJSON(const char * sensorName) {
  Acceleration::read();
  return Acceleration::toJSON(sensorName);
}

const char * Acceleration::toCSV(const char * sensorName) {
  return valuesToCSV(sensorName, Acceleration::header.timestamp, 3,
                     Acceleration::x, Acceleration::y, Acceleration::z);
}

const char * Acceleration::toJSON(const char * sensorName) {
  return _tripleValueToJSON(sensorName, 2, Acceleration::header.unit, "%sX", Acceleration::x,
                           "%sY", Acceleration::y, "%sZ", Acceleration::z);
}

/*
 * Magnetic
 * ========================================================================================
 */
Magnetic::Magnetic(void) {
  Magnetic::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Magnetic::header.unit = DATA_UNIT_MICROTESLA;
}

Magnetic::~Magnetic() {
}

boolean Magnetic::begin(void) {
  return start_sensor_or_err(magnetic_sensor_name, lsm303_mag_init());
}

void Magnetic::read(void) {
  Magnetic::header.timestamp = millis();

  Magnetic::header.sensor_id = Magnetic::sensorId;
  lsm303_getMag(&(Magnetic::x), &(Magnetic::y), &(Magnetic::z));
}

const char * Magnetic::readToCSV(const char * sensorName) {
  Magnetic::read();
  return Magnetic::toCSV(sensorName);
}

const char * Magnetic::readToJSON(const char * sensorName) {
  Magnetic::read();
  return Magnetic::toJSON(sensorName);
}

const char * Magnetic::toCSV(const char * sensorName) {
  return valuesToCSV(sensorName, Magnetic::header.timestamp, 3,
                     Magnetic::x, Magnetic::y, Magnetic::z);
}

const char * Magnetic::toJSON(const char * sensorName) {
  return _tripleValueToJSON(sensorName, 2, Magnetic::header.unit, "%sX", Magnetic::x,
                           "%sY", Magnetic::y, "%sZ", Magnetic::z);
}

/*
 * Gyro
 * ========================================================================================
 */
Gyro::Gyro(void) {
  Gyro::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Gyro::header.unit = DATA_UNIT_RADIAN_PER_SECOND;
}

Gyro::~Gyro() {
}

boolean Gyro::begin(void) {
  return start_sensor_or_err(gyro_sensor_name, l3gd20h_init());
}

void Gyro::read(void) {
  Gyro::header.timestamp = millis();

  Gyro::header.sensor_id = Gyro::sensorId;
  l3gd20h_getOrientation(&(Gyro::x), &(Gyro::y), &(Gyro::z));
}

const char * Gyro::readToCSV(const char * sensorName) {
  Gyro::read();
  return Gyro::toCSV(sensorName);
}

const char * Gyro::readToJSON(const char * sensorName) {
  Gyro::read();
  return Gyro::toJSON(sensorName);
}

const char * Gyro::toCSV(const char * sensorName) {
  return valuesToCSV(sensorName, Gyro::header.timestamp, 3,
                     Gyro::x, Gyro::y, Gyro::z);
}

const char * Gyro::toJSON(const char * sensorName) {
  return _tripleValueToJSON(sensorName, 2, Gyro::header.unit, "%sX", Gyro::x,
                           "%sY", Gyro::y, "%sZ", Gyro::z);
}

/*
 * Orientation
 * ========================================================================================
 */
Orientation::Orientation(void) {
  Orientation::sensorId = SENSORID_ADAFRUIT9DOFIMU;
  Orientation::header.unit = DATA_UNIT_DEGREES;
}

Orientation::~Orientation() {
}

boolean Orientation::begin(void) {
  Orientation::accel = Acceleration();
  Orientation::mag = Magnetic();
  return Orientation::accel.begin() && Orientation::mag.begin();
}

boolean Orientation::begin(Acceleration & accel, Magnetic & mag) {
  Orientation::accel = accel;
  Orientation::mag = mag;
  return true;
}

void Orientation::read(void) {
  Orientation::accel.read();
  Orientation::mag.read();
  Orientation::read(Orientation::accel, Orientation::mag);
}

void Orientation::read(Acceleration & acc, Magnetic & mag) {
  float roll;
  float pitch;
  float heading;
  unsigned long acctime;
  unsigned long magtime;
  const float PI_F = 3.141592653F;

  acctime = acc.header.timestamp;
  magtime = mag.header.timestamp;

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
  Orientation::header.timestamp = acctime > magtime ? acctime : magtime;
}

const char * Orientation::readToCSV(const char * sensorName) {
  Orientation::read();
  return Orientation::toCSV(sensorName);
}

const char * Orientation::readToJSON(const char * sensorName) {
  Orientation::read();
  return Orientation::toJSON(sensorName);
}

const char * Orientation::readToCSV(Acceleration & acc, Magnetic & mag, const char * sensorName) {
  Orientation::read(acc, mag);
  return Orientation::toCSV(sensorName);
}

const char * Orientation::readToJSON(Acceleration & acc, Magnetic & mag, const char * sensorName) {
  Orientation::read(acc, mag);
  return Orientation::toJSON(sensorName);
}

const char * Orientation::toCSV(const char * sensorName) {
  return valuesToCSV(sensorName, Orientation::header.timestamp, 3,
                     Orientation::roll, Orientation::pitch, Orientation::heading);
}

const char * Orientation::toJSON(const char * sensorName) {
  return _tripleValueToJSON(sensorName, 8, Orientation::header.unit, "%sRoll", Orientation::roll,
                           "%sPitch", Orientation::pitch, "%sHeading", Orientation::heading);
}
