/***************************************************************************
 This is a library for the LSM303 Accelerometer and magnentometer/compass

 Designed specifically to work with the Adafruit LSM303DLHC Breakout

 These displays use I2C to communicate, 2 pins are required to interface.

 Adafruit invests time and resources providing this open source code,
 please support Adafruit andopen-source hardware by purchasing products
 from Adafruit!

 Written by Kevin Townsend for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "Ardusat_Adafruit_LSM303_U.h"
#include "obcl.h"

static float _lsm303Accel_MG_LSB = 0.001F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z = 980.0F;   // Varies with gain

/***************************************************************************
 ACCELEROMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Abstract away platform differences in Arduino wire library
 */
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::write8(byte address, byte reg, byte value) {
	obcl_writeByteToRegAddr(address, reg, value);
	/*
	 Wire.beginTransmission(address);
	 #if ARDUINO >= 100
	 Wire.write((uint8_t)reg);
	 Wire.write((uint8_t)value);
	 #else
	 Wire.send(reg);
	 Wire.send(value);
	 #endif
	 Wire.endTransmission();
	 */
}

/**************************************************************************/
/*!
 @brief  Abstract away platform differences in Arduino wire library
 */
/**************************************************************************/
byte Adafruit_LSM303_Accel_Unified::read8(byte address, byte reg) {
	uint8_t ret = 0;
	obcl_readByteFromRegAddr(address, reg, &ret);
	return (ret);
	/*
	 Wire.beginTransmission(address);
	 #if ARDUINO >= 100
	 Wire.write((uint8_t)reg);
	 #else
	 Wire.send(reg);
	 #endif
	 Wire.endTransmission();
	 Wire.requestFrom(address, (byte)1);
	 #if ARDUINO >= 100
	 value = Wire.read();
	 #else
	 value = Wire.receive();
	 #endif
	 Wire.endTransmission();

	 return value;
	 */
}

/**************************************************************************/
/*!
 @brief  Reads the raw data from the sensor
 */
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::read() {
	uint8_t recdlen = 0;
	uint8_t _data[6];

	obcl_readByteArrayFromRegAddr( LSM303_ADDRESS_ACCEL,
			(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80), (uint8_t*) &_data, 6,
			&recdlen);

//	uint8_t * ptr = (uint8_t *)&_accelData;

	// Shift values to create properly formed integer (low byte first)
	_accelData.x = (int16_t) (_data[0] | (_data[1] << 8)) >> 4;
	_accelData.y = (int16_t) (_data[2] | (_data[3] << 8)) >> 4;
	_accelData.z = (int16_t) (_data[4] | (_data[5] << 8)) >> 4;
	/*

	 // Read the accelerometer
	 Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
	 #if ARDUINO >= 100
	 Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
	 #else
	 Wire.send(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
	 #endif
	 Wire.endTransmission();
	 Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);

	 // Wait around until enough data is available
	 while (Wire.available() < 6);

	 #if ARDUINO >= 100
	 uint8_t xlo = Wire.read();
	 uint8_t xhi = Wire.read();
	 uint8_t ylo = Wire.read();
	 uint8_t yhi = Wire.read();
	 uint8_t zlo = Wire.read();
	 uint8_t zhi = Wire.read();
	 #else
	 uint8_t xlo = Wire.receive();
	 uint8_t xhi = Wire.receive();
	 uint8_t ylo = Wire.receive();
	 uint8_t yhi = Wire.receive();
	 uint8_t zlo = Wire.receive();
	 uint8_t zhi = Wire.receive();
	 #endif

	 // Shift values to create properly formed integer (low byte first)
	 _accelData.x = (int16_t)(xlo | (xhi << 8)) >> 4;
	 _accelData.y = (int16_t)(ylo | (yhi << 8)) >> 4;
	 _accelData.z = (int16_t)(zlo | (zhi << 8)) >> 4;
	 */
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates a new Adafruit_LSM303 class
 */
/**************************************************************************/
Adafruit_LSM303_Accel_Unified::Adafruit_LSM303_Accel_Unified(int32_t sensorID) {
	_sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool Adafruit_LSM303_Accel_Unified::begin() {
	// Enable I2C
	obcl_begin();
//  Wire.begin();

	// Enable the accelerometer (100Hz)
	write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
	// if we are connected or not
	uint8_t reg1_a = read8(LSM303_ADDRESS_ACCEL,
			LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	if (reg1_a != 0x57) {
		return false;
	}

	return true;
}

/**************************************************************************/
/*! 
 @brief  Gets the most recent sensor event
 */
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::getEvent(sensors_event_t *event) {
	/* Clear the event */
	memset(event, 0, sizeof(sensors_event_t));

	/* Read new data */
	read();

	event->version = sizeof(sensors_event_t);
	event->sensor_id = _sensorID;
	event->type = SENSOR_TYPE_ACCELEROMETER;
	event->timestamp = millis();
	event->acceleration.x = _accelData.x * _lsm303Accel_MG_LSB
			* SENSORS_GRAVITY_STANDARD;
	event->acceleration.y = _accelData.y * _lsm303Accel_MG_LSB
			* SENSORS_GRAVITY_STANDARD;
	event->acceleration.z = _accelData.z * _lsm303Accel_MG_LSB
			* SENSORS_GRAVITY_STANDARD;
}

/**************************************************************************/
/*! 
 @brief  Gets the sensor_t data
 */
/**************************************************************************/
void Adafruit_LSM303_Accel_Unified::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "LSM303", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_ACCELEROMETER;
	sensor->min_delay = 0;
	sensor->max_value = 0.0F; // TBD
	sensor->min_value = 0.0F; // TBD
	sensor->resolution = 0.0F; // TBD
}

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Abstract away platform differences in Arduino wire library
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::write8(byte address, byte reg, byte value) {
	obcl_writeByteToRegAddr(address, reg, value);
	/*
	 Wire.beginTransmission(address);
	 #if ARDUINO >= 100
	 Wire.write((uint8_t)reg);
	 Wire.write((uint8_t)value);
	 #else
	 Wire.send(reg);
	 Wire.send(value);
	 #endif
	 Wire.endTransmission();
	 */
}

/**************************************************************************/
/*!
 @brief  Abstract away platform differences in Arduino wire library
 */
/**************************************************************************/
byte Adafruit_LSM303_Mag_Unified::read8(byte address, byte reg) {
	uint8_t ret = 0;
	obcl_readByteFromRegAddr(address, reg, &ret);
	return (ret);
	/*
	 byte value;

	 Wire.beginTransmission(address);
	 #if ARDUINO >= 100
	 Wire.write((uint8_t)reg);
	 #else
	 Wire.send(reg);
	 #endif
	 Wire.endTransmission();
	 Wire.requestFrom(address, (byte)1);
	 #if ARDUINO >= 100
	 value = Wire.read();
	 #else
	 value = Wire.receive();
	 #endif
	 Wire.endTransmission();

	 return value;
	 */
}

/**************************************************************************/
/*!
 @brief  Reads the raw data from the sensor
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::read() {
	uint8_t recdlen = 0;
	uint8_t _data[6];

	obcl_readByteArrayFromRegAddr( LSM303_ADDRESS_MAG,
			LSM303_REGISTER_MAG_OUT_X_H_M, (uint8_t*) &_data, 6, &recdlen);

	/*
	 // Read the magnetometer
	 Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
	 #if ARDUINO >= 100
	 Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
	 #else
	 Wire.send(LSM303_REGISTER_MAG_OUT_X_H_M);
	 #endif
	 Wire.endTransmission();
	 Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);

	 // Wait around until enough data is available
	 while (Wire.available() < 6);

	 // Note high before low (different than accel)
	 #if ARDUINO >= 100
	 uint8_t xhi = Wire.read();
	 uint8_t xlo = Wire.read();
	 uint8_t zhi = Wire.read();
	 uint8_t zlo = Wire.read();
	 uint8_t yhi = Wire.read();
	 uint8_t ylo = Wire.read();
	 #else
	 uint8_t xhi = Wire.receive();
	 uint8_t xlo = Wire.receive();
	 uint8_t zhi = Wire.receive();
	 uint8_t zlo = Wire.receive();
	 uint8_t yhi = Wire.receive();
	 uint8_t ylo = Wire.receive();
	 #endif
	 */

	// Shift values to create properly formed integer (low byte first)
	_magData.x = (int16_t) (_data[1] | ((int16_t) _data[0] << 8));
	_magData.y = (int16_t) (_data[3] | ((int16_t) _data[2] << 8));
	_magData.z = (int16_t) (_data[5] | ((int16_t) _data[4] << 8));

	// ToDo: Calculate orientation
	_magData.orientation = 0.0;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates a new Adafruit_LSM303 class
 */
/**************************************************************************/
Adafruit_LSM303_Mag_Unified::Adafruit_LSM303_Mag_Unified(int32_t sensorID) {
	_sensorID = sensorID;
	_autoRangeEnabled = false;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool Adafruit_LSM303_Mag_Unified::begin() {
	// Enable I2C
	obcl_begin();
//  Wire.begin();

	// Enable the magnetometer
	write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
	// the default value (0b00010000/0x10)
	uint8_t reg1_a = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
	if (reg1_a != 0x10) {
		return false;
	}

	// Set the gain to a known level
	setMagGain(LSM303_MAGGAIN_1_3);

	return true;
}

/**************************************************************************/
/*! 
 @brief  Enables or disables auto-ranging
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::enableAutoRange(bool enabled) {
	_autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
 @brief  Sets the magnetometer's gain
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::setMagGain(lsm303MagGain gain) {
	write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte) gain);

	_magGain = gain;

	switch (gain) {
	case LSM303_MAGGAIN_1_3:
		_lsm303Mag_Gauss_LSB_XY = 1100;
		_lsm303Mag_Gauss_LSB_Z = 980;
		break;
	case LSM303_MAGGAIN_1_9:
		_lsm303Mag_Gauss_LSB_XY = 855;
		_lsm303Mag_Gauss_LSB_Z = 760;
		break;
	case LSM303_MAGGAIN_2_5:
		_lsm303Mag_Gauss_LSB_XY = 670;
		_lsm303Mag_Gauss_LSB_Z = 600;
		break;
	case LSM303_MAGGAIN_4_0:
		_lsm303Mag_Gauss_LSB_XY = 450;
		_lsm303Mag_Gauss_LSB_Z = 400;
		break;
	case LSM303_MAGGAIN_4_7:
		_lsm303Mag_Gauss_LSB_XY = 400;
		_lsm303Mag_Gauss_LSB_Z = 255;
		break;
	case LSM303_MAGGAIN_5_6:
		_lsm303Mag_Gauss_LSB_XY = 330;
		_lsm303Mag_Gauss_LSB_Z = 295;
		break;
	case LSM303_MAGGAIN_8_1:
		_lsm303Mag_Gauss_LSB_XY = 230;
		_lsm303Mag_Gauss_LSB_Z = 205;
		break;
	}
}

/**************************************************************************/
/*! 
 @brief  Gets the most recent sensor event
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::getEvent(sensors_event_t *event) {
	bool readingValid = false;

	/* Clear the event */
	memset(event, 0, sizeof(sensors_event_t));

	while (!readingValid) {
		/* Read new data */
		read();

		/* Make sure the sensor isn't saturating if auto-ranging is enabled */
		if (!_autoRangeEnabled) {
			readingValid = true;
		} else {
			Serial.print(_magData.x);
			Serial.print(" ");
			Serial.print(_magData.y);
			Serial.print(" ");
			Serial.print(_magData.z);
			Serial.println(" ");
			/* Check if the sensor is saturating or not */
			if ((_magData.x >= 4090) | (_magData.x <= -4090)
					| (_magData.y >= 4090) | (_magData.y <= -4090)
					| (_magData.z >= 4090) | (_magData.z <= -4090)) {
				/* Saturating .... increase the range if we can */
				switch (_magGain) {
				case LSM303_MAGGAIN_5_6:
					setMagGain(LSM303_MAGGAIN_8_1);
					readingValid = false;
					Serial.println("Changing range to +/- 8.1");
					break;
				case LSM303_MAGGAIN_4_7:
					setMagGain(LSM303_MAGGAIN_5_6);
					readingValid = false;
					Serial.println("Changing range to +/- 5.6");
					break;
				case LSM303_MAGGAIN_4_0:
					setMagGain(LSM303_MAGGAIN_4_7);
					readingValid = false;
					Serial.println("Changing range to +/- 4.7");
					break;
				case LSM303_MAGGAIN_2_5:
					setMagGain(LSM303_MAGGAIN_4_0);
					readingValid = false;
					Serial.println("Changing range to +/- 4.0");
					break;
				case LSM303_MAGGAIN_1_9:
					setMagGain(LSM303_MAGGAIN_2_5);
					readingValid = false;
					Serial.println("Changing range to +/- 2.5");
					break;
				case LSM303_MAGGAIN_1_3:
					setMagGain(LSM303_MAGGAIN_1_9);
					readingValid = false;
					Serial.println("Changing range to +/- 1.9");
					break;
				default:
					readingValid = true;
					break;
				}
			} else {
				/* All values are withing range */
				readingValid = true;
			}
		}
	}

	event->version = sizeof(sensors_event_t);
	event->sensor_id = _sensorID;
	event->type = SENSOR_TYPE_MAGNETIC_FIELD;
	event->timestamp = millis();
	event->magnetic.x = _magData.x
			/ _lsm303Mag_Gauss_LSB_XY* SENSORS_GAUSS_TO_MICROTESLA;
	event->magnetic.y = _magData.y
			/ _lsm303Mag_Gauss_LSB_XY* SENSORS_GAUSS_TO_MICROTESLA;
	event->magnetic.z = _magData.z
			/ _lsm303Mag_Gauss_LSB_Z* SENSORS_GAUSS_TO_MICROTESLA;
}

/**************************************************************************/
/*! 
 @brief  Gets the sensor_t data
 */
/**************************************************************************/
void Adafruit_LSM303_Mag_Unified::getSensor(sensor_t *sensor) {
	/* Clear the sensor_t object */
	memset(sensor, 0, sizeof(sensor_t));

	/* Insert the sensor name in the fixed length char array */
	strncpy(sensor->name, "LSM303", sizeof(sensor->name) - 1);
	sensor->name[sizeof(sensor->name) - 1] = 0;
	sensor->version = 1;
	sensor->sensor_id = _sensorID;
	sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
	sensor->min_delay = 0;
	sensor->max_value = 0.0F; // TBD
	sensor->min_value = 0.0F; // TBD
	sensor->resolution = 0.0F; // TBD
}
