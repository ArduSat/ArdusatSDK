/**
 * @file   drivers.c
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements sensor-specific driver read/initialization functions
 *
 * These functions are meant to provide a gateway interface that the higher-level
 * generic interface uses to get data from individual sensors.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Ardusat_Adafruit_Sensor.h>
#include <Ardusat_Adafruit_LSM303_U.h>
#include <Ardusat_Adafruit_9DOF.h>
#include <Ardusat_Adafruit_SI1145.h>
#include <Ardusat_TSL2561.h>

#include "drivers.h"

/**
 * Generic I2C Read function
 *
 * @param devAddr I2C Device address
 * @param reg Register address
 * @param val pointer to byte array to read into
 * @param length number of bytes to read
 * @param byteOrder Reverse byte order after read
 *
 * @return 0 on success, other on failure
 */
int _readFromRegAddr(uint8_t devAddr, uint8_t reg, uint8_t *val, 
		     int length, boolean byteOrder)
{
  uint8_t *byteArray = (uint8_t *) val;
  int ret;
  int readData = 0;

  Wire.beginTransmission(devAddr);

  if (Wire.write(reg) < 1)
    return -1;

  if ((ret = Wire.endTransmission(false)) != 0) {
    return ret;
  }

  if (byteArray == NULL)
    return -1;

  Wire.beginTransmission(devAddr);
  Wire.requestFrom((byte) devAddr, (byte) length);

  while (Wire.available() > 0) {
    byteArray[readData++] = Wire.read();
  }

  if ((ret = Wire.endTransmission(true)) != 0) {
    return ret;
  }

  if (!byteOrder) {
    uint8_t tmp_b = byteArray[0];
    byteArray[0] = byteArray[1];
    byteArray[1] = tmp_b;
  }
  return 0;
}

/*
 * 9DOF Sensor
 */
// Assign a unique ID to the sensors 
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

boolean adafruit9dof_init() {
  if (!accel.begin()) {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin()) {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  return(true);
}

void adafruit9dof_getOrientation(float * roll, float * pitch, float * heading) {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    *roll = orientation.roll;
    *pitch = orientation.pitch;
    *heading = orientation.heading;
  }
}

void adafruit9dof_getAccel(float * x, float * y, float * z) {
  sensors_event_t accel_event;

  accel.getEvent(&accel_event);
  *x = accel_event.acceleration.x;
  *y = accel_event.acceleration.y;
  *z = accel_event.acceleration.z;
}

void adafruit9dof_getMag(float * x, float * y, float * z) {
  sensors_event_t mag_event;

  accel.getEvent(&mag_event);
  *x = mag_event.magnetic.x;
  *y = mag_event.magnetic.y;
  *z = mag_event.magnetic.z;
}

/*
 * ML8511 UV Light
 */
/*
 * Takes 8 readings from the given pin then averages the values.
 *
 * @return average reading
 */
int average_analog_read(int pinToRead)
{
  short numberOfReadings = 8;
  unsigned int runningValue = 0; 
  int x;

  for(x = 0; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

/*
 * Does a linear transform on the given value to come up with a scaled output.
 *
 * http://forum.arduino.cc/index.php?topic=3922.0
 */
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Inits the ML8511 breakout board UV sensor
 *
 * @return true
 */
boolean ml8511_init()
{
  return true;
}

/**
 * Reads the ML8511 UV Sensor. To do this we take an average analog voltage 
 * read on the UV sensor pin and the 3.3 V power pin, then use that actual 
 * voltage to get a ratio of the read voltage compared to exactly 3.3V. We 
 * then map this value into a properly scaled value.
 *
 * @return calculated UV value in mW / cm^2
 */
float ml8511_getUV()
{
  int uv_v = average_analog_read(DRIVER_ML8511_UV_PIN);
  int ref_v = average_analog_read(DRIVER_ML8511_REF_PIN);
  float scaled_uv_v = 3.3 / ref_v * uv_v;
  return map_float(scaled_uv_v, 0.99, 2.9, 0.0, 15.0);
}

/*
 * SI1145 UV/Light
 */
Adafruit_SI1145 uv = Adafruit_SI1145();

boolean si1145_init() {
  return uv.begin();
}

float si1145_getUVIndex() {
  float UVindex = uv.readUV();

  // the index is multiplied by 100 so to get the integer index, divide by 100
  UVindex /= 100.0;
  return (UVindex);
}

/*
 * MLX90614 IR Temperature
 */
uint16_t _mlx90614_read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(DRIVER_MLX90614_ADDR); // start transmission to device
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(false); // end transmission

  Wire.requestFrom((uint8_t) DRIVER_MLX90614_ADDR, (uint8_t)3);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= Wire.read() << 8; // receive DATA

  uint8_t pec = Wire.read();

  return ret;
}

float _mlx90614_readTemp(uint8_t reg) {
  float temp;

  temp = _mlx90614_read16(reg);
  temp *= .02;
  temp -= 273.15;
  return temp;
}

float _mlx90614_readObjectTempF(void) {
  return (_mlx90614_readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}

float _mlx90614_readAmbientTempF(void) {
  return (_mlx90614_readTemp(MLX90614_TA) * 9 / 5) + 32;
}

float _mlx90614_readObjectTempC(void) {
  return _mlx90614_readTemp(MLX90614_TOBJ1);
}

float _mlx90614_readAmbientTempC(void) {
  return _mlx90614_readTemp(MLX90614_TA);
}

boolean mlx90614_init() {
  Wire.begin();
  return true;
}

float mlx90614_getTempCelsius() {
  return _mlx90614_readObjectTempC();
}

/*
 * TMP102 Temperature
 */
byte _tmp102_buff[2];

boolean tmp102_init() {
  Wire.begin();
  return true;
}

float tmp102_getTempCelsius() {
  uint16_t val;
  float tmp;

  _readFromRegAddr(DRIVER_TMP102_ADDR, 0x00, (uint8_t *) &val, 2, false);

  tmp = (val >> 4) * 0.0625;
  return tmp;
}

/*
 * TSL2561 Luminosity
 */
TSL2561 tsl2561 = TSL2561(DRIVER_TSL2561_ADDR);

boolean tsl2561_init() {
  return tsl2561.begin();
}

float tsl2561_getLux() {
  uint16_t ch0 = tsl2561.getLuminosity(0);
  uint16_t ch1 = tsl2561.getLuminosity(1);
  return tsl2561.calculateLux(ch0, ch1);
}
