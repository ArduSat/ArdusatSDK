/**
 * @file   drivers.cpp
 * @author Ben Peters (ben@ardusat.com)
 * @author Sam Olds (sam@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements sensor-specific driver read/initialization functions
 *
 * These functions are meant to provide a gateway interface that the higher-level
 * generic interface uses to get data from individual sensors.
 */

#include "ArdusatSDK.h"
#include "drivers.h"
#include <Wire.h>

static config_lsm303_accel_t _lsm303_d_accel_config;
static config_lsm303_mag_t _lsm303_d_mag_config;
static config_l3gd20_t _l3gd20_config;

/**
 * Check to see if both the ISL29125 and TCS34725 RGB Light Sensors
 * exists. They are only included with the spaceboard, not the space
 * kit, so if they do exist, the user has a spaceboard.
 */
void catchSpaceboard() {
  // Only need to check once, as `catchSpaceboard` is called by all
  // sensor `begin` functions
  if (!ARDUSAT_SPACEBOARD && !MANUAL_CONFIG) {
    uint8_t islData;
    uint8_t tcsData;
    Wire.begin();

    // Here, we are checking to see if the sensor replies with the
    // correct response at its expected address on the spaceboard
    readFromRegAddr(DRIVER_SPACEBOARD_ISL29125_ADDR, 0x00, &islData, 1, BIG_ENDIAN);
    readFromRegAddr(DRIVER_SPACEBOARD_TCS34725_ADDR, 0x80 | 0x12, &tcsData, 1, BIG_ENDIAN);

    // Checking the responses from the identify registers of each
    // sensor to make sure it's what we expect for both
    if (islData == 0x7D && (tcsData == 0x44 || tcsData == 0x10)) {
      ARDUSAT_SPACEBOARD = true;
    }
  }
}

/*
 * L3GD20 Gyro Sensor
 */
boolean l3gd20h_init(uint8_t range) {
  uint8_t buf;
  Wire.begin();

  // Check WHO_AM_I register
  if (readFromRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_WHO_AM_I, &buf, 1) ||
      ((buf != L3GD20_ID) && buf != L3GD20H_ID)) {
    return false;
  }

  // Sets switch to normal mode & enables 3 channels
  buf = 0x0F;
  if (writeToRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_CTRL_REG1, &buf, 1)) {
    return false;
  }

  // Adjust gyro range
  // Ctrl register 4 selects scale
  // BDU BLE FS1 FS0 - 0 0 SIM
  //  0   0   0   0  0 0 0  0
  //
  // FS1/FS 0 Values:
  // 0x00 = 250 DPS
  // 0x10 = 500 DPS
  // 0x20 = 2000 DPS

  switch(range) {
    case 0x00:
      _l3gd20_config.sensitivity = L3GD20_GYRO_SENSITIVITY_250DPS;
      break;
    case 0x10:
      _l3gd20_config.sensitivity = L3GD20_GYRO_SENSITIVITY_500DPS;
      break;
    case 0x20:
      _l3gd20_config.sensitivity = L3GD20_GYRO_SENSITIVITY_2000DPS;
      break;
    default:
      range = 0x20;
      _l3gd20_config.sensitivity = L3GD20_GYRO_SENSITIVITY_2000DPS;
  }

  if (writeToRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_CTRL_REG4, &range, 1))
    return false;

  return true;
}

int _2bit_xyz_read(uint8_t addr, uint8_t reg, int16_t *x, int16_t *y, int16_t *z,
                   bool reverse_bits) {
  uint8_t buf[6];
  uint8_t *hi;
  uint8_t *low;
  int ret;

  if (!(ret = readFromRegAddr(addr, reg, buf, 6))) {
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
    readFromRegAddr(L3GD20_ADDRESS, L3GD20_GYRO_REGISTER_OUT_TEMP, pRawTemperature, sizeof(*pRawTemperature));
  }
}

/*
 * LSM303 Accel + Mag Sensor
 *
 * LSM303_D Datasheet:
 * http://www.st.com/web/en/resource/technical/document/datasheet/DM00057547.pdf
 *
 * LSM303_DLHC Datasheet:
 * https://www.adafruit.com/datasheets/LSM303DLHC.PDF
 */
LSM303 lsm;

// LSM303 Accelerometer Configurations
void _lsm303_accel_config(lsm303_accel_gain_e gGain) {
  uint8_t gain;
  _lsm303_d_accel_config.gain = gGain;

  // LSM303 D (Spaceboard variant)
  if (lsm.getDeviceType() == LSM303::device_D) {
    // Adjustable gain:
    //   AFS = 0b00000000 (+/-  2 g full scale) = 0x00
    //   AFS = 0b00001000 (+/-  4 g full scale) = 0x08
    //   AFS = 0b00010000 (+/-  6 g full scale) = 0x10
    //   AFS = 0b00011000 (+/-  8 g full scale) = 0x18 (Default)
    //   AFS = 0b00100000 (+/- 16 g full scale) = 0x20
    switch (gGain) {
      case LSM303_ACCEL_GAIN2G:
        gain = 0x00;
        break;
      case LSM303_ACCEL_GAIN4G:
        gain = 0x08;
        break;
      case LSM303_ACCEL_GAIN6G:
        gain = 0x10;
        break;
      case LSM303_ACCEL_GAIN8G:
        gain = 0x18;
        break;
      case LSM303_ACCEL_GAIN16G:
        gain = 0x20;
        break;
    }
    lsm.writeReg(CTRL2, gain);

    // 0x57 = 0b01010111
    // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    lsm.writeReg(CTRL1, 0x57);
  }
  else // LSM303 DLHC (Breakout Space Kit variant)
  {
    // Adustable gain:
    //   FS = 0b00001000 (+/-  2 g full scale; high resolution enable) = 0x08
    //   FS = 0b00011000 (+/-  4 g full scale; high resolution enable) = 0x18
    //   FS = 0b00101000 (+/-  8 g full scale; high resolution enable) = 0x28 (Default)
    //   FS = 0b00111000 (+/- 16 g full scale; high resolution enable) = 0x38
    switch (gGain) {
      case LSM303_ACCEL_GAIN2G:
        gain = 0x08;
        break;
      case LSM303_ACCEL_GAIN4G:
        gain = 0x18;
        break;
      case LSM303_ACCEL_GAIN6G:
        _lsm303_d_accel_config.gain = LSM303_ACCEL_GAIN8G;
        // Rollthrough because DLHC doesn't have 6G Gain
      case LSM303_ACCEL_GAIN8G:
        gain = 0x28;
        break;
      case LSM303_ACCEL_GAIN16G:
        gain = 0x38;
        break;
    }
    lsm.writeAccReg(CTRL_REG4_A, gain);

    // 0x47 = 0b01000111
    // ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
    lsm.writeAccReg(CTRL_REG1_A, 0x47);
  }
}

// LSM303 Magnetometer Configurations
void _lsm303_mag_config(lsm303_mag_scale_e gaussScale) {
  uint8_t scale;
  _lsm303_d_mag_config.scale = gaussScale;

  // LSM303 D (Spaceboard variant)
  if (lsm.getDeviceType() == LSM303::device_D) {
    // 0x64 = 0b01100100
    // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR)
    lsm.writeReg(CTRL5, 0x64);

    // Adjustable gauss scale:
    //   MFS = 0b00000000 (+/-  2 gauss full scale) = 0x00
    //   MFS = 0b00100000 (+/-  4 gauss full scale) = 0x20 (Default)
    //   MFS = 0b01000000 (+/-  8 gauss full scale) = 0x40
    //   MFS = 0b01100000 (+/- 12 gauss full scale) = 0x60
    switch (gaussScale) {
      case LSM303_MAG_SCALE1_3GAUSS: // Rollthrough because D doesn't have 1.3 Gauss
      case LSM303_MAG_SCALE2GAUSS:
        _lsm303_d_mag_config.scale = LSM303_MAG_SCALE2GAUSS;
        scale = 0x00;
        break;
      case LSM303_MAG_SCALE2_5GAUSS: // Rollthrough because D doesn't have 2.5 Gauss
      case LSM303_MAG_SCALE4GAUSS:
        _lsm303_d_mag_config.scale = LSM303_MAG_SCALE4GAUSS;
        scale = 0x20;
        break;
      case LSM303_MAG_SCALE4_7GAUSS: // Rollthrough because D doesn't have 4.7 Gauss
      case LSM303_MAG_SCALE5_6GAUSS: // Rollthrough because D doesn't have 5.6 Gauss
      case LSM303_MAG_SCALE8GAUSS:
        _lsm303_d_mag_config.scale = LSM303_MAG_SCALE8GAUSS;
        scale = 0x40;
        break;
      case LSM303_MAG_SCALE12GAUSS:
        _lsm303_d_mag_config.scale = LSM303_MAG_SCALE12GAUSS;
        scale = 0x60;
        break;
    }
    lsm.writeReg(CTRL6, scale);

    // 0x00 = 0b00000000
    // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)
    lsm.writeReg(CTRL7, 0x00);
  }
  else // LSM303 DLHC (Breakout Space Kit variant)
  {
    // 0x0C = 0b00001100
    // DO = 011 (7.5 Hz ODR)
    lsm.writeMagReg(CRA_REG_M, 0x0C);

    // Adjustable gauss scale:
    //   MFS = 0b00100000 (+/- 1.3 gauss full scale) = 0x20
    //   MFS = 0b01000000 (+/- 1.9 gauss full scale) = 0x40
    //   MFS = 0b01100000 (+/- 2.5 gauss full scale) = 0x60
    //   MFS = 0b10000000 (+/- 4.0 gauss full scale) = 0x80 (Default)
    //   MFS = 0b10100000 (+/- 4.7 gauss full scale) = 0xA0
    //   MFS = 0b11000000 (+/- 5.6 gauss full scale) = 0xC0
    //   MFS = 0b11100000 (+/- 8.1 gauss full scale) = 0xE0
    switch (gaussScale) {
      case LSM303_MAG_SCALE1_3GAUSS:
        scale = 0x20;
        break;
      case LSM303_MAG_SCALE2GAUSS:
        scale = 0x40;
        break;
      case LSM303_MAG_SCALE2_5GAUSS:
        scale = 0x60;
        break;
      case LSM303_MAG_SCALE4GAUSS:
        scale = 0x80;
        break;
      case LSM303_MAG_SCALE4_7GAUSS:
        scale = 0xA0;
        break;
      case LSM303_MAG_SCALE5_6GAUSS:
        scale = 0xC0;
        break;
      case LSM303_MAG_SCALE12GAUSS: // Rollthrough because DLHC doesn't have 12 Gauss
      case LSM303_MAG_SCALE8GAUSS:
        _lsm303_d_mag_config.scale = LSM303_MAG_SCALE8GAUSS;
        scale = 0xE0;
        break;
    }
    lsm.writeMagReg(CRB_REG_M, scale);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    lsm.writeMagReg(MR_REG_M, 0x00);
  }
}

boolean lsm303_accel_init(lsm303_accel_gain_e gain) {
  Wire.begin();
  lsm.init();
  _lsm303_accel_config(gain);
  return true;
}

boolean lsm303_mag_init(lsm303_mag_scale_e scale) {
  Wire.begin();
  lsm.init();
  _lsm303_mag_config(scale);
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
      raw_temp[0] = lsm.readReg(TEMP_OUT_L);
      raw_temp[1] = lsm.readReg(TEMP_OUT_H);
      *pRawTemperature = raw_temp[0] | (raw_temp[1] << 8);
    } else {
      raw_temp[0] = lsm.readMagReg(TEMP_OUT_L_M);
      raw_temp[1] = lsm.readMagReg(TEMP_OUT_H_M);
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
  if (readFromRegAddr(DRIVER_BMP180_ADDR, reg, buf, 2)) { \
    return -1; \
  } else { \
    config_location = (buf[0] << 8) + buf[1]; \
  }

#define _bmp180_read_signed_config_val(reg, config_location) \
  if (readFromRegAddr(DRIVER_BMP180_ADDR, reg, buf, 2)) { \
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

  if (readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CHIPID, &res, 1) ||
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
  if (writeToRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CONTROL, reg, 1)) {
    return;
  }

  delay(5);

  if (readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_TEMPDATA, reg, 2)) {
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

  if (writeToRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_CONTROL, &reg, 1)) {
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

  if (readFromRegAddr(DRIVER_BMP180_ADDR, BMP085_REGISTER_PRESSUREDATA, reg, 3)) {
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
float _mlx90614_readTemp(uint8_t reg) {
  uint32_t data;

  // Reads 3 bytes, but the third byte is a Packet Error Code (PEC)
  // and is unused.
  readFromRegAddr(DRIVER_MLX90614_ADDR, reg, &data, 3, LITTLE_ENDIAN);
  float temp = (data & 0xFFFF); // Get rid of PEC byte from read

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
    temp_byte = DRIVER_SPACEBOARD_TMP102_ADDR;
  } else {
    temp_byte = DRIVER_TMP102_ADDR;
  }

  readFromRegAddr(temp_byte, 0x00, bytes, 2);
  temp_byte = bytes[0];
  bytes[0] = bytes[1];
  bytes[1] = temp_byte;

  tmp = (val >> 4) * 0.0625;
  return tmp;
}

/*
 * TSL2561 Luminosity
 */
TSL2561 *tsl2561;

boolean tsl2561_init(tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain) {
  uint8_t addr;
  if (!tsl2561) {
    if (ARDUSAT_SPACEBOARD) {
      addr = DRIVER_SPACEBOARD_TSL2561_ADDR;
    } else {
      addr = DRIVER_TSL2561_ADDR;
    }
    tsl2561 = new TSL2561(addr);
  }
  boolean result = tsl2561->begin();

  if(result)
  {
    tsl2561->setIntegrationTime(intTime);
    tsl2561->setGain(gain);
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
SFE_ISL29125 isl29125 = SFE_ISL29125(DRIVER_SPACEBOARD_ISL29125_ADDR);

// intensity == CFG1_375LUX if dark
//              CFG1_10KLUX if bright (default)
boolean isl29125_init(uint8_t intensity) {
  boolean initialized = isl29125.init();
  if (initialized && (intensity == CFG1_375LUX || intensity == CFG1_10KLUX)) {
    initialized = isl29125.config(CFG1_MODE_RGB | intensity, CFG2_IR_ADJUST_HIGH, CFG_DEFAULT);
  }
  return initialized;
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
