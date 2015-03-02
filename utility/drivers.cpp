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
#include "drivers.h"

/**
 * Generic I2C Read function
 *
 * @param devAddr I2C Device address
 * @param reg Register address
 * @param val pointer to byte array to read into
 * @param length number of bytes to read
 *
 * @return 0 on success, other on failure
 */
int _readFromRegAddr(uint8_t devAddr, uint8_t reg, void *val, int length)
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

  if (byteArray == NULL || length == 0)
    return 0;

  Wire.beginTransmission(devAddr);
  Wire.requestFrom((byte) devAddr, (byte) length);

  while (Wire.available() > 0) {
    byteArray[readData++] = Wire.read();
  }

  if ((ret = Wire.endTransmission(true)) != 0) {
    return ret;
  }

  return 0;
}

/**
 * Generic I2C Write function
 *
 * @param devAddr I2C Device address
 * @param reg Register address
 * @param val pointer to byte array to write from
 * @param length number of bytes to write
 *
 * @return 0 on success, other on failure
 */
int _writeToRegAddr(uint8_t devAddr, uint8_t reg, void *val, int length)
{
  uint8_t *byteArray = (uint8_t *) val;
  int ret;

  Wire.beginTransmission(devAddr);

  if (Wire.write(reg) < 1)
    return -1;

  for (int i=0; i < length; ++i) {
    if (Wire.write(byteArray[i]) < 1)
      return -1;
  }

  if ((ret = Wire.endTransmission()) != 0) {
    return ret;
  }

  return 0;
}

/*
 * L3GD20 Gyro Sensor
 */
boolean l3gd20h_init() {
  uint8_t buf;
  Wire.begin();

  // Check WHO_AM_I register
  if (_readFromRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_WHO_AM_I, &buf, 1) ||
      ((buf != L3GD20_ID) && buf != L3GD20H_ID)) {
    return false;
  }

  // Sets switch to normal mode & enables 3 channels
  buf = 0x0F;
  if (_writeToRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_CTRL_REG1, &buf, 1)) {
    return false;
  }

  //Adjust gyro range
  //0x00 = 250DPS
  //0x10 = 500DPS
  //0x20 = 2000DPS
  buf = 0x00;
  if (_writeToRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_CTRL_REG4, &buf, 1))
    return false;

  return true;
}

int _2bit_xyz_read(uint8_t addr, uint8_t reg, int16_t *x, int16_t *y, int16_t *z,
                   bool reverse_bits) {
  uint8_t buf[6];
  uint8_t *hi;
  uint8_t *low;
  int ret;

  if (!(ret = _readFromRegAddr(addr, reg, buf, 6))) {
    if (reverse_bits) {
      low = buf + 1;
      hi = buf;
    } else {
      low = buf;
      hi = low + 1;
    }
    *x = (int16_t)(*low | (((int16_t) *hi) << 8));
    low = low + 2;
    hi = hi + 2;
    *y = (int16_t)((*low) | (((int16_t) *hi) << 8));
    low = low + 2;
    hi = hi + 2;
    *z = (int16_t)((*low) | (((int16_t) *hi) << 8));
    return 0;
  }

  return ret;
}

void l3gd20h_getOrientation(float *x, float *y, float *z) {
  int16_t vals[3];
  if (_2bit_xyz_read(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_OUT_X_L | 0x80,
                     &vals[0], &vals[1], &vals[2], false) == 0) {

    // Need different compensation values here if we give ability to adjust gyro range
    *x = vals[0] * L3GD20_GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
    *y = vals[1] * L3GD20_GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
    *z = vals[2] * L3GD20_GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
  }
}

/**
 * Get raw 16 bit readings from l3gd20h gyroscope. These raw values can then be converted
 * into engineering values.
 *
 * @param pX value to store x-axis data in
 * @param pY value to store y-axis data in
 * @param pZ value to store z-axis data in
 */
void l3gd20h_getRawAngularRates(int16_t *pX, int16_t *pY, int16_t *pZ)
{
  if((NULL != pX) && (NULL != pY) && (NULL != pZ))
  {
    _2bit_xyz_read(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_OUT_X_L | 0x80,
                     pX, pY, pZ, false);
  }
}

/**
 * Gets the raw 8-bit temperature reading from the L3GD20H on-die thermometer. This
 * value has a scale factor of -1 LSB/degree C.
 *
 * @param pRawTemperature location to store temp reading in
 */
void l3gd20h_getRawTemperature(int8_t *pRawTemperature)
{
  if(NULL != pRawTemperature)
  {
    _readFromRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_OUT_TEMP, pRawTemperature, sizeof(*pRawTemperature));
  }
}

/*
 * LSM303 Accel + Mag Sensor
 */
int _lsm303_set_mag_gain(lsm303MagGain gain)
{
  int new_xy;
  int new_z;
  int ret;

  switch(gain) {
    case LSM303_MAGGAIN_1_3:
      new_xy = 1100;
      new_z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      new_xy = 855;
      new_z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      new_xy = 670;
      new_z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      new_xy = 450;
      new_z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      new_xy = 400;
      new_z  = 255;
      break;
    case LSM303_MAGGAIN_5_6:
      new_xy = 330;
      new_z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      new_xy = 230;
      new_z  = 205;
      break;
    default:
      return -1;
  }

  if ((ret = _writeToRegAddr(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M,
                             (uint8_t *) &gain, 1)) == 0) {
    _lsm303Mag_Gauss_LSB_XY = new_xy;
    _lsm303Mag_Gauss_LSB_Z = new_z;
    return 0;
  } else {
    return ret;
  }
}

boolean lsm303_accel_init() {
  uint8_t res = 0x57;

  Wire.begin();
  if (_writeToRegAddr(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, &res, 1)) {
    return false; }

  //verify connected to sensor
  if (_readFromRegAddr(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, &res, 1) ||
      res != 0x57) {
    return false;
  }

  // set full scale range to +/- 2g, enable high resolution mode
  res = 8;
  if (_writeToRegAddr(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, &res, 1)) {
    return false; }

  return true;
}

boolean lsm303_mag_init() {
  uint8_t res;

  Wire.begin();
  // configure mode select for continuous conversion
  res = 0x00;
  if (_writeToRegAddr(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, &res, 1)) {
    return false;
  }

  // configure for 15 Hz ODR, enable temperature sensor
  res = 0x90;
  if (_writeToRegAddr(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, &res, 1)) {
    return false;
  }

  //verify connected to sensor
  if (_readFromRegAddr(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, &res, 1) ||
      res != 0x90) {
    return false;
  }

  return _lsm303_set_mag_gain(LSM303_MAGGAIN_1_3) == 0;
}

void lsm303_getAccel(float *x, float *y, float *z)
{
  int16_t vals[3];
  if (_2bit_xyz_read(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80,
                     &vals[0], &vals[1], &vals[2], false) == 0) {
    *x = (vals[0] >> 4) * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    *y = (vals[1] >> 4) * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    *z = (vals[2] >> 4) * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  }
}

/**
 * Get raw 16 bit readings from LSM303 accelerometer.
 *
 * @param pX value to store x-axis data in
 * @param pY value to store y-axis data in
 * @param pZ value to store z-axis data in
 */
void lsm303_getRawAcceleration(int16_t *pX, int16_t *pY, int16_t *pZ)
{
  if((NULL != pX) && (NULL != pY) && (NULL != pZ))
  {
    _2bit_xyz_read(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80,
                     pX, pY, pZ, false);

	 // since the accelerometer readings are 12 bit (in high resolution mode) and left justified, we neet to right shift by 4
    (*pX) >>= 4;
    (*pY) >>= 4;
    (*pZ) >>= 4;
  }
}

/**
 * Gets 16-bit temperature fro on-die thermometer on the magnetometer. This appears to 
 * be appropriately scaled & offset in deg C.
 *
 * @param pRawTemperature location to store temp reading in
 */
void lsm303_getRawTemperature(int16_t *pRawTemperature)
{
  if(NULL != pRawTemperature)
  {
    uint8_t ReadingFromRegisters[2];

    _readFromRegAddr(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_TEMP_OUT_H_M, ReadingFromRegisters, sizeof(ReadingFromRegisters));

    // Since the _readFromRegAddr() call reads the temperature out in big endian format (MSB first),
    // we need to convert it to little endian
    *pRawTemperature = ReadingFromRegisters[0] << 8;
    *pRawTemperature |= ReadingFromRegisters[1];
    // Since the temperature value appears to be 10 bits and left justified, we need to right shift by 6
    *pRawTemperature >>= 6;
  }
}

void lsm303_getMag(float *x, float *y, float *z)
{
  int16_t vals[3];

  //For some reason, y & z are "flipped" in the raw data coming out of the mag.
  //(z values are read before y values)
  //use indexes to flip them back
  if (_2bit_xyz_read(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M,
                     &vals[0], &vals[2], &vals[1], true) == 0) {
    *x = vals[0] / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
    *y = vals[1] / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
    *z = vals[2] / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
  }
}

/**
 * Get raw 16 bit readings from LSM303 magnetometer.
 *
 * @param pX value to store x-axis data in
 * @param pY value to store y-axis data in
 * @param pZ value to store z-axis data in
 */
void lsm303_getRawMag(int16_t *pX, int16_t *pY, int16_t *pZ)
{
  if((NULL != pX) && (NULL != pY) && (NULL != pZ))
  {
    //NOTE: the magnetometer output registers are not in X, Y, Z order; instead,
    //they are in X, Z, Y order
    _2bit_xyz_read(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M,
                     pX, pZ, pY, true);
  }
}


/*
 * BMP180 Barometeric Altimeter
 */
// BMP state variables
static bmp085_calibration _bmp180_calibration;
static uint8_t _bmp180_mode;

/*
 * Reads the factory configuration from the BMP180
 */
#define _bmp180_read_unsigned_config_val(reg, config_location) \
  if (_readFromRegAddr(DRIVER_BMP180_ADDR, reg, buf, 2)) { \
    return -1; \
  } else { \
    config_location = (buf[0] << 8) + buf[1]; \
  }

#define _bmp180_read_signed_config_val(reg, config_location) \
  if (_readFromRegAddr(DRIVER_BMP180_ADDR, reg, buf, 2)) { \
    return -1; \
  } else { \
    config_location = (int16_t) ((buf[0] << 8) + buf[1]); \
  }

static int _bmp180_read_configuration(void)
{
  uint8_t buf[2];
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_AC1, _bmp180_calibration.ac1)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_AC2, _bmp180_calibration.ac2)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_AC3, _bmp180_calibration.ac3)
  _bmp180_read_unsigned_config_val(BMP085_REGISTER_CAL_AC4, _bmp180_calibration.ac4)
  _bmp180_read_unsigned_config_val(BMP085_REGISTER_CAL_AC5, _bmp180_calibration.ac5)
  _bmp180_read_unsigned_config_val(BMP085_REGISTER_CAL_AC6, _bmp180_calibration.ac6)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_B1, _bmp180_calibration.b1)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_B2, _bmp180_calibration.b2)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_MB, _bmp180_calibration.mb)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_MC, _bmp180_calibration.mc)
  _bmp180_read_signed_config_val(BMP085_REGISTER_CAL_MD, _bmp180_calibration.md)

  return 0;
}

/**
 * Compute B5 coefficient from datasheet for temp & pressure calculations
 *
 * @param ut raw temp measured from BMP180 chip
 */
int32_t _bmp180_compute_b5(int32_t ut)
{
  int32_t X1 = (ut - (int32_t) _bmp180_calibration.ac6) * ((int32_t) _bmp180_calibration.ac5) >> 15;
  int32_t X2 = ((int32_t) _bmp180_calibration.mc << 11) / (X1 + (int32_t) _bmp180_calibration.md);
  return X1 + X2;
}

/**
 * Initialize the BMP180. Sets ultra high res mode.
 */
boolean bmp180_init()
{
  uint8_t res;
  bmp085_mode_t mode = BMP085_MODE_ULTRAHIGHRES;

  Wire.begin();

  if (_readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CHIPID, &res, 1) ||
      res != 0x55) {
    return false;
  }

  if (_bmp180_read_configuration()) {
    return false;
  }

  _bmp180_mode = mode;
  return true;
}

/**
 * Gets raw 16 bit temperature from the BMP180. See datasheet for instructions on
 * calculating calibrated value.
 *
 * @param temp location to write temperature to.
 */
void bmp180_getRawTemperature(uint16_t *temp)
{
  uint8_t reg[2];
  reg[0] = BMP085_REGISTER_READTEMPCMD;
  if (_writeToRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CONTROL, reg, 1)) {
    return;
  }

  delay(5);

  if (_readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_TEMPDATA, reg, 2)) {
    return;
  }

  *temp = (reg[0] << 8) + reg[1];
}

/**
 * Get calibrated temperature from BMP180.
 *
 * @param temp location to write temp to
 */
void bmp180_getTemperature(float *temp)
{
  uint16_t raw_temp = 0;
  int32_t UT, X1, X2, B5;

  bmp180_getRawTemperature(&raw_temp);
  if (raw_temp != 0) {
    UT = (int32_t) raw_temp;
    B5 = _bmp180_compute_b5(UT);
    *temp = (B5 + 8) >> 4;
    *temp /= 10;
  }
}

/**
 * Gets raw 24 bit pressure value from BMP180. See datasheet for instructions on 
 * calculating calibrated value.
 *
 * @param pressure location to write pressure to
 */
void bmp180_getRawPressure(uint32_t *pressure)
{
  uint8_t reg[3];
  uint32_t temp32;
  reg[0] = BMP085_REGISTER_READPRESSURECMD + (_bmp180_mode << 6);

  if (_writeToRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CONTROL, &reg, 1)) {
    return;
  }

  // read delay is based on resolution mode set in begin
  switch(_bmp180_mode) {
    case BMP085_MODE_ULTRALOWPOWER:
      delay(5);
      break;
    case BMP085_MODE_STANDARD:
      delay(8);
      break;
    case BMP085_MODE_HIGHRES:
      delay(14);
      break;
    case BMP085_MODE_ULTRAHIGHRES:
    default:
      delay(26);
      break;
  }

  if (_readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_PRESSUREDATA, reg, 3)) {
    return;
  }

  temp32 = (uint32_t) reg[0] << 16;
  temp32 += reg[1] << 8;
  temp32 += reg[2];
  temp32 >>= (8 - _bmp180_mode);

  *pressure = temp32;
}

/**
 * Get calibrated pressure from BMP180.
 *
 * @param pressure location to write to
 */
void bmp180_getPressure(float *pressure) {
  int32_t ut = 0;
  int32_t up = 0;
  int32_t pressure_i = 0;
  int32_t x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;
  uint16_t raw_temp;

  bmp180_getRawTemperature(&raw_temp);
  ut = (int32_t) raw_temp;
  bmp180_getRawPressure((uint32_t *) &up);
  up = (int32_t) up;

  // Temp compensation
  b5 = _bmp180_compute_b5(ut);

  // Pressure compensation
  b6 = b5 - 4000;
  x1 = (_bmp180_calibration.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp180_calibration.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) _bmp180_calibration.ac1) * 4 + x3) << _bmp180_mode) + 2) >> 2;
  x1 = (_bmp180_calibration.ac3 * b6) >> 13;
  x2 = (_bmp180_calibration.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp180_calibration.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> _bmp180_mode));

  if (b7 < 0x80000000) {
    p = (b7 << 1) / b4;
  } else {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure_i = p + ((x1 + x2 + 3791) >> 4);

  *pressure = pressure_i / 100.0F;
}

/**
 * Calculates the altitude in meters from the specified measured atmospheric pressure (hPa) 
 * and sea-level pressure (hPa)
 *
 * Equation taken from BMP180 datasheet (page 16):
 * http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
  
 * Note that using the equation from wikipedia can give bad results
 * at high altitude.  See this thread for more information:
 * http://forums.adafruit.com/viewtopic.php?f=22&t=58064
 *
 * @param seaLevelPressure Known pressure at sea-level (hPa)
 * @param atmosphericPressure Pressure at altitude (hPa)
 */
float pressureToAltitude(float seaLevelPressure, float atmosphericPressure)
{
  return 44330.0 * (1.0 - pow(atmosphericPressure / seaLevelPressure, 0.1903));
}

/**
 * Calculates the pressure at sea level (in hPa) from the specified altitude (meters)
 * and measured atmospheric pressure (hPa)
 * 
 * Equation taken from BMP180 datasheet (page 17):
 * http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
 *
 * Note that using the equation from wikipedia can give bad results
 * at high altitude.  See this thread for more information:
 * http://forums.adafruit.com/viewtopic.php?f=22&t=58064
 *
 * @param altitude Known altitude (meters)
 * @param atmosphericPressure Measured atmospheric pressure (hPa)
 */
float seaLevelPressureForAltitude(float altitude, float atmosphericPressure)
{
  return atmosphericPressure / pow(1.0 - (altitude / 44330.0), 5.255);
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
  pinMode(DRIVER_ML8511_UV_PIN, INPUT);
  pinMode(DRIVER_ML8511_REF_PIN, INPUT);

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

  //TODO: re-write to leverage _read function
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
  int16_t val;
  uint8_t * bytes = (uint8_t *) &val;
  uint8_t temp_byte;
  float tmp;

  _readFromRegAddr(DRIVER_TMP102_ADDR, 0x00, bytes, 2);

  temp_byte = bytes[0];
  bytes[0] = bytes[1];
  bytes[1] = temp_byte;

  tmp = (val >> 4) * 0.0625;
  return tmp;
}

/*
 * TSL2561 Luminosity
 */
TSL2561 tsl2561 = TSL2561(DRIVER_TSL2561_ADDR);

boolean tsl2561_init() {
  boolean result = tsl2561.begin();

  if(result)
  {
    tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    tsl2561.setGain(TSL2561_GAIN_1X);
    tsl2561.enableAutoRange(false);
  }

  return result;

}

float tsl2561_getLux() {
  uint16_t broadband, ir;

  tsl2561.getLuminosity(&broadband, &ir);
  return tsl2561.calculateLux(broadband, ir);
}
