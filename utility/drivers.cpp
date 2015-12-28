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
#include "pololu_LSM303.h"
#include "ArdusatSDK.h"

static config_lsm303_accel_t _lsm303_d_accel_config;
static config_lsm303_mag_t _lsm303_d_mag_config;
static config_l3gd20_t _l3gd20_config;

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
  // Ctrl register 4 selects scale
  // BDU BLE FS1 FS0 - 0 0 SIM
  //  0   0   0   0  0 0 0  0
  //
  // FS1/FS 0 Values:
  // 0x00 = 250 DPS
  // 0x10 = 500 DPS
  // 0x20 = 2000 DPS
  // 0x30 = 2000 DPS
  buf = 0x20;
  _l3gd20_config.sensitivity = L3GD20_GYRO_SENSITIVITY_2000DPS;

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

    *x = vals[0] * _l3gd20_config.sensitivity * SENSORS_DPS_TO_RADS;
    *y = vals[1] * _l3gd20_config.sensitivity * SENSORS_DPS_TO_RADS;
    *z = vals[2] * _l3gd20_config.sensitivity * SENSORS_DPS_TO_RADS;
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
LSM303 lsm;

void _lsm303_set_sdk_options() {
  // update accelerometer gain
  if (lsm.getDeviceType() == LSM303::device_D) {
    lsm.writeReg(LSM303::CTRL2, LSM303_ACCEL_GAIN8G);
    _lsm303_d_accel_config.gain = LSM303_ACCEL_GAIN8G;
  } else {
    // Ctrl register FS1/FS0 for gain
    // 00 +/- 2
    // 01 +/- 4
    // 10 +/- 8
    // 11 +/- 16
    // BDU BLE FS1 FS0 HR 0 0 SIM
    // 0    0   0   0   1 0 0  0
    // 0b00011000 = 0x18 (High res, +/- 4g)
    lsm.writeAccReg(LSM303::CTRL_REG4_A, 0x38);
    _lsm303_d_accel_config.gain = LSM303_ACCEL_GAIN16G;
  }

  // update mag scaling
  if (lsm.getDeviceType() == LSM303::device_D) {
    lsm.writeReg(LSM303::CTRL6, LSM303_MAG_SCALE4GAUSS);
    _lsm303_d_mag_config.scale = LSM303_MAG_SCALE4GAUSS;
    // Ctrl register for magnetic resolution, data rate, temp enable
    // TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
    //    0/1    1      1      1      1      0      0   0
    // 0xF8 = 0b11111000 - temp enable, mag high res, defaults ODR, LIR
    // enable onboard temp sensor
    lsm.writeReg(LSM303::CTRL5, 0xf8);
  } else {
    // Ctrl register for mag scaling
    // 0b001 +-1.3
    // 0b010 +-1.9
    // 0b011 +-2.5
    // 0b100 +-4.0
    // 0b101 +-4.7
    // 0b110 +-5.6
    // ob111 +-8.1
    // GN2 GN1 GN0  0 -->
    // 0b10000000 = 0x80 (+/- 4 gauss)
    lsm.writeMagReg(LSM303::CRB_REG_M, 0x80);
    _lsm303_d_mag_config.scale = LSM303_MAG_SCALE4GAUSS;

    // Ctrl register for Data Output Rate & Temp sensor
    // TEMP_EN 0 0 DO2 DO1 DO0 0 0
    //    1    0 0  1   0   0  0 0
    // 0x90 = 0b10010000 - temp enable, default data rate
    lsm.writeMagReg(LSM303::CRA_REG_M, 0x90);
  }
}

boolean lsm303_accel_init() {
  Wire.begin();
  lsm.init();
  lsm.enableDefault();
  _lsm303_set_sdk_options();
  return true;
}

boolean lsm303_mag_init() {
  Wire.begin();
  lsm.init();
  lsm.enableDefault();
  _lsm303_set_sdk_options();
  return true;
}

void lsm303_getAccel(float *x, float *y, float *z)
{
  float sensitivity;

  lsm.readAcc();

  if (lsm.getDeviceType() == LSM303::device_D) {
    switch(_lsm303_d_accel_config.gain) {
        case LSM303_ACCEL_GAIN2G:
            sensitivity = 0.061;
            break;
        case LSM303_ACCEL_GAIN4G:
            sensitivity = 0.122;
            break;
        case LSM303_ACCEL_GAIN6G:
            sensitivity = 0.183;
            break;
        case LSM303_ACCEL_GAIN8G:
            sensitivity = 0.244;
            break;
        case LSM303_ACCEL_GAIN16G:
            sensitivity = 0.732;
            break;
    }
    *x = lsm.a.x * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
    *y = lsm.a.y * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
    *z = lsm.a.z * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
  } else {
    switch(_lsm303_d_accel_config.gain) {
        case LSM303_ACCEL_GAIN2G:
            sensitivity = 1.0;
            break;
        case LSM303_ACCEL_GAIN4G:
            sensitivity = 2.0;
            break;
        case LSM303_ACCEL_GAIN8G:
            sensitivity = 4.0;
            break;
        case LSM303_ACCEL_GAIN16G:
            sensitivity = 12.0;
            break;
    }
    // need to right shift 4 b/c we've got 12 bit resolution, left-justified
    *x = (lsm.a.x >> 4) * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
    *y = (lsm.a.y >> 4) * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
    *z = (lsm.a.z >> 4) * sensitivity / 1000 * SENSORS_GRAVITY_STANDARD;
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
  if((NULL != pX) && (NULL != pY) && (NULL != pZ)) {
    lsm.readAcc();
    *pX = lsm.a.x;
    *pY = lsm.a.y;
    *pZ = lsm.a.z;
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
  uint8_t raw_temp[2];

  if(NULL != pRawTemperature) {
    if (lsm.getDeviceType() == LSM303::device_D) {
      raw_temp[0] = lsm.readReg(LSM303::TEMP_OUT_L);
      raw_temp[1] = lsm.readReg(LSM303::TEMP_OUT_H);
      *pRawTemperature = raw_temp[0] | (raw_temp[1] << 8);
    } else {
      raw_temp[0] = lsm.readMagReg(LSM303::TEMP_OUT_L_M);
      raw_temp[1] = lsm.readMagReg(LSM303::TEMP_OUT_H_M);
      *pRawTemperature = (raw_temp[0] | (raw_temp[1] << 8)) >> 4;
    }
  }
}

void lsm303_getMag(float *x, float *y, float *z)
{
  float sensitivity, z_sensitivity;

  lsm.readMag();

  if (lsm.getDeviceType() == LSM303::device_D) {
    switch(_lsm303_d_mag_config.scale) {
        case LSM303_MAG_SCALE2GAUSS:
            sensitivity = 0.080;
            break;
        case LSM303_MAG_SCALE4GAUSS:
            sensitivity = 0.160;
            break;
        case LSM303_MAG_SCALE8GAUSS:
            sensitivity = 0.320;
            break;
        case LSM303_MAG_SCALE12GAUSS:
            sensitivity = 0.479;
            break;
    }
    *x = lsm.m.x * sensitivity * SENSORS_MGAUSS_TO_UTESLA;
    *y = lsm.m.y * sensitivity * SENSORS_MGAUSS_TO_UTESLA;
    *z = lsm.m.z * sensitivity * SENSORS_MGAUSS_TO_UTESLA;
  } else {
    switch(_lsm303_d_mag_config.scale) {
        case LSM303_MAG_SCALE1_3GAUSS:
            sensitivity = 1100;
            z_sensitivity = 980;
            break;
        case LSM303_MAG_SCALE2GAUSS:
            sensitivity = 855;
            z_sensitivity = 760;
            break;
        case LSM303_MAG_SCALE2_5GAUSS:
            sensitivity = 670;
            z_sensitivity = 600;
            break;
        case LSM303_MAG_SCALE4GAUSS:
            sensitivity = 450;
            z_sensitivity = 400;
            break;
        case LSM303_MAG_SCALE4_7GAUSS:
            sensitivity = 400;
            z_sensitivity = 355;
            break;
        case LSM303_MAG_SCALE5_6GAUSS:
            sensitivity = 330;
            z_sensitivity = 295;
            break;
        case LSM303_MAG_SCALE8GAUSS:
            sensitivity = 230;
            z_sensitivity = 205;
            break;
    }
    *x = lsm.m.x / sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
    *y = lsm.m.y / sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
    *z = lsm.m.z / sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
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
  if((NULL != pX) && (NULL != pY) && (NULL != pZ)) {
    lsm.readMag();
    *pX = lsm.m.x;
    *pY = lsm.m.y;
    *pZ = lsm.m.z;
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
boolean bmp180_init(bmp085_mode_t mode)
{
  uint8_t res;

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
ML8511_ADC ml8511_uv_adc = ML8511_ADC(DRIVER_ML8511_ADDR);

boolean ml8511_init()
{
  if (ARDUSAT_SPACEBOARD) {
    return ml8511_uv_adc.init();
  } else {
    pinMode(DRIVER_ML8511_UV_PIN, INPUT);
    pinMode(DRIVER_ML8511_REF_PIN, INPUT);
    return true;
  }
}

/**
 * Reads the ML8511 UV Sensor. To do this we take an average analog voltage 
 * read on the UV sensor pin and the 3.3 V power pin, then use that actual 
 * voltage to get a ratio of the read voltage compared to exactly 3.3V. We 
 * then map this value into a properly scaled value.
 * 
 * @param pin to read
 *
 * @return calculated UV value in mW / cm^2
 */
float ml8511_getUV(int pin)
{
  float scaled_uv_v;
  if (ARDUSAT_SPACEBOARD) {
    scaled_uv_v = ml8511_uv_adc.read_uv();
  } else {
    int uv_v = average_analog_read(pin);
    int ref_v = average_analog_read(DRIVER_ML8511_REF_PIN);
    scaled_uv_v = 3.3 / ref_v * uv_v;
  }
  return map_float(scaled_uv_v, 0.99, 2.9, 0.0, 15.0);
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

  if (ARDUSAT_SPACEBOARD) {
    temp_byte = DRIVER_LEMSENS_TMP102_1_ADDR;
  } else {
    temp_byte = DRIVER_TMP102_ADDR;
  }

  _readFromRegAddr(temp_byte, 0x00, bytes, 2);
  temp_byte = bytes[0];
  bytes[0] = bytes[1];
  bytes[1] = temp_byte;

  tmp = (val >> 4) * 0.0625;
  return tmp;
}

/*
 * TSL2561 Luminosity
 */
//TSL2561 tsl2561 = TSL2561(DRIVER_TSL2561_ADDR);
TSL2561 *tsl2561;

boolean tsl2561_init() {
  uint8_t addr;
  if (!tsl2561) {
    if (ARDUSAT_SPACEBOARD) {
      addr = DRIVER_LEMSENS_TSL2561_ADDR;
    } else {
      addr = DRIVER_TSL2561_ADDR;
    }
    tsl2561 = new TSL2561(addr);
  }
  boolean result = tsl2561->begin();

  if(result)
  {
    tsl2561->setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    tsl2561->setGain(TSL2561_GAIN_16X);
    tsl2561->enableAutoRange(true);
  }

  return result;
}

float tsl2561_getLux() {
  uint16_t broadband, ir;

  tsl2561->getLuminosity(&broadband, &ir);
  return tsl2561->calculateLux(broadband, ir);
}


/*
 * ISL29125 RGB Light Sensor
 */
SFE_ISL29125 isl29125;

boolean isl29125_init() {
  return isl29125.init();
}

void isl29125_getRGB(float *red, float *green, float *blue) {
  *red = isl29125.readRed();
  *green = isl29125.readGreen();
  *blue = isl29125.readBlue();
}


/*
 * TCS34725 RGB Light Sensor
 */
Adafruit_TCS34725 tcs34725 = Adafruit_TCS34725();

boolean tcs34725_init(tcs34725IntegrationTime_t it, tcs34725Gain_t gain) {
  boolean init = tcs34725.begin();
  tcs34725.setIntegrationTime(it);
  tcs34725.setGain(gain);
  return init;
}

void tcs34725_getRGB(float *red, float *green, float *blue) {
  uint16_t r, g, b, clear;

  tcs34725.getRawData(&r, &g, &b, &clear);
  *red = r;
  *green = g;
  *blue = b;
}


/*
 * SI1132 UV Light Sensor
 */
Adafruit_SI1145 si1132_uv = Adafruit_SI1145();

boolean si1132_init() {
  return si1132_uv.begin();
}

float si1132_getUVIndex() {
  float UVindex = si1132_uv.readUV();

  // the index is multiplied by 100 so to get the integer index, divide by 100
  UVindex /= 100.0;
  return UVindex;
}
