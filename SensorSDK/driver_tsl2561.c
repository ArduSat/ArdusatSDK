#include <Arduino.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "sensor_addr.h"
#include "obcl.h"
#include "driver_tsl2561.h"
#include "config.h"

#ifdef LOAD_DRIVER_TSL2561

/**************************************************************************/
/*!
 @file     tsl2561.h
 @author   K. Townsend (microBuilder.eu)

 @section LICENSE

 Software License Agreement (BSD License)

 Copyright (c) 2010, microBuilder SARL
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
//#include <Wire.h>

#define TSL2561_VISIBLE 2                   // channel 0 - channel 1#define TSL2561_INFRARED 1                  // channel 1#define TSL2561_FULLSPECTRUM 0              // channel 0
// 3 i2c address options!
#define TSL2561_ADDR_LOW  0x29
#define TSL2561_ADDR_FLOAT 0x39
#define TSL2561_ADDR_HIGH 0x49

// Lux calculations differ slightly for CS package
//#define TSL2561_PACKAGE_CS
#define TSL2561_PACKAGE_T_FN_CL

#define TSL2561_READBIT           (0x01)

#define TSL2561_COMMAND_BIT       (0x80)    // Must be 1#define TSL2561_CLEAR_BIT         (0x40)    // Clears any pending interrupt (write 1 to clear)#define TSL2561_WORD_BIT          (0x20)    // 1 = read/write word (rather than byte)#define TSL2561_BLOCK_BIT         (0x10)    // 1 = using block read/write
#define TSL2561_CONTROL_POWERON   (0x03)
#define TSL2561_CONTROL_POWEROFF  (0x00)

#define TSL2561_LUX_LUXSCALE      (14)      // Scale by 2^14#define TSL2561_LUX_RATIOSCALE    (9)       // Scale ratio by 2^9#define TSL2561_LUX_CHSCALE       (10)      // Scale channel values by 2^10#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE
// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE#define TSL2561_LUX_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE#define TSL2561_LUX_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE#define TSL2561_LUX_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE#define TSL2561_LUX_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE#define TSL2561_LUX_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE#define TSL2561_LUX_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE#define TSL2561_LUX_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE#define TSL2561_LUX_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE#define TSL2561_LUX_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE#define TSL2561_LUX_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE#define TSL2561_LUX_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE#define TSL2561_LUX_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE#define TSL2561_LUX_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE#define TSL2561_LUX_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE#define TSL2561_LUX_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE#define TSL2561_LUX_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE#define TSL2561_LUX_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE#define TSL2561_LUX_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE#define TSL2561_LUX_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE#define TSL2561_LUX_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE#define TSL2561_LUX_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE#define TSL2561_LUX_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE#define TSL2561_LUX_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE
// CS package values
#define TSL2561_LUX_K1C           (0x0043)  // 0.130 * 2^RATIO_SCALE#define TSL2561_LUX_B1C           (0x0204)  // 0.0315 * 2^LUX_SCALE#define TSL2561_LUX_M1C           (0x01ad)  // 0.0262 * 2^LUX_SCALE#define TSL2561_LUX_K2C           (0x0085)  // 0.260 * 2^RATIO_SCALE#define TSL2561_LUX_B2C           (0x0228)  // 0.0337 * 2^LUX_SCALE#define TSL2561_LUX_M2C           (0x02c1)  // 0.0430 * 2^LUX_SCALE#define TSL2561_LUX_K3C           (0x00c8)  // 0.390 * 2^RATIO_SCALE#define TSL2561_LUX_B3C           (0x0253)  // 0.0363 * 2^LUX_SCALE#define TSL2561_LUX_M3C           (0x0363)  // 0.0529 * 2^LUX_SCALE#define TSL2561_LUX_K4C           (0x010a)  // 0.520 * 2^RATIO_SCALE#define TSL2561_LUX_B4C           (0x0282)  // 0.0392 * 2^LUX_SCALE#define TSL2561_LUX_M4C           (0x03df)  // 0.0605 * 2^LUX_SCALE#define TSL2561_LUX_K5C           (0x014d)  // 0.65 * 2^RATIO_SCALE#define TSL2561_LUX_B5C           (0x0177)  // 0.0229 * 2^LUX_SCALE#define TSL2561_LUX_M5C           (0x01dd)  // 0.0291 * 2^LUX_SCALE#define TSL2561_LUX_K6C           (0x019a)  // 0.80 * 2^RATIO_SCALE#define TSL2561_LUX_B6C           (0x0101)  // 0.0157 * 2^LUX_SCALE#define TSL2561_LUX_M6C           (0x0127)  // 0.0180 * 2^LUX_SCALE#define TSL2561_LUX_K7C           (0x029a)  // 1.3 * 2^RATIO_SCALE#define TSL2561_LUX_B7C           (0x0037)  // 0.00338 * 2^LUX_SCALE#define TSL2561_LUX_M7C           (0x002b)  // 0.00260 * 2^LUX_SCALE#define TSL2561_LUX_K8C           (0x029a)  // 1.3 * 2^RATIO_SCALE#define TSL2561_LUX_B8C           (0x0000)  // 0.000 * 2^LUX_SCALE#define TSL2561_LUX_M8C           (0x0000)  // 0.000 * 2^LUX_SCALE
enum {
	TSL2561_REGISTER_CONTROL = 0x00,
	TSL2561_REGISTER_TIMING = 0x01,
	TSL2561_REGISTER_THRESHHOLDL_LOW = 0x02,
	TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03,
	TSL2561_REGISTER_THRESHHOLDH_LOW = 0x04,
	TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05,
	TSL2561_REGISTER_INTERRUPT = 0x06,
	TSL2561_REGISTER_CRC = 0x08,
	TSL2561_REGISTER_ID = 0x0A,
	TSL2561_REGISTER_CHAN0_LOW = 0x0C,
	TSL2561_REGISTER_CHAN0_HIGH = 0x0D,
	TSL2561_REGISTER_CHAN1_LOW = 0x0E,
	TSL2561_REGISTER_CHAN1_HIGH = 0x0F
};

typedef enum {
	TSL2561_INTEGRATIONTIME_13MS = 0x00,    // 13.7ms
	TSL2561_INTEGRATIONTIME_101MS = 0x01,    // 101ms
	TSL2561_INTEGRATIONTIME_402MS = 0x02     // 402ms
} tsl2561IntegrationTime_t;

typedef enum {
	TSL2561_GAIN_0X = 0x00,    // No gain
	TSL2561_GAIN_16X = 0x10,    // 16x gain
} tsl2561Gain_t;

// CC version
int8_t _tsl2561_addr;
tsl2561IntegrationTime_t _tsl2561_integration;
tsl2561Gain_t _tsl2561_gain;
boolean _tsl2561_initialized;

void _tsl2561_construct(uint8_t addr);
boolean _tsl2561_begin(void);
void _tsl2561_enable(void);
void _tsl2561_disable(void);
void _tsl2561_write8(uint8_t r, uint8_t v);
uint16_t _tsl2561_read16(uint8_t reg);

uint32_t _tsl2561_calculateLux(uint16_t ch0, uint16_t ch1);
void _tsl2561_setTiming(tsl2561IntegrationTime_t integration);
void _tsl2561_setGain(tsl2561Gain_t gain);
uint16_t _tsl2561_getLuminosity(uint8_t channel);
uint32_t _tsl2561_getFullLuminosity();

// CLASS
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

void _tsl2561_construct(uint8_t addr) {
	_tsl2561_addr = addr;
	_tsl2561_initialized = false;
	_tsl2561_integration = TSL2561_INTEGRATIONTIME_13MS;
	_tsl2561_gain = TSL2561_GAIN_16X;

	// we cant do wire initialization till later, because we havent loaded Wire yet
}

boolean _tsl2561_begin(void) {
	uint8_t x = 0;
	obcl_begin();

	// Initialise I2C
	obcl_readByteFromRegAddr(_tsl2561_addr, TSL2561_REGISTER_ID, &x);

	//Serial.print("0x"); Serial.println(x, HEX);
	if (x & 0x0A) {
		//Serial.println("Found TSL2561");
	} else {
		return false;
	}
	_tsl2561_initialized = true;

	// Set default integration time and gain
	_tsl2561_setTiming (_tsl2561_integration);
	_tsl2561_setGain (_tsl2561_gain);
	// Note: by default, the device is in power down mode on bootup
	_tsl2561_disable();

	return true;
}

void _tsl2561_enable(void) {
	if (!_tsl2561_initialized)
		_tsl2561_begin();

	// Enable the device by setting the control bit to 0x03
	_tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
			TSL2561_CONTROL_POWERON);
}

void _tsl2561_disable(void) {
	if (!_tsl2561_initialized)
		_tsl2561_begin();

	// Disable the device by setting the control bit to 0x03
	_tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
			TSL2561_CONTROL_POWEROFF);
}

void _tsl2561_setGain(tsl2561Gain_t gain) {
	if (!_tsl2561_initialized)
		_tsl2561_begin();

	_tsl2561_enable();
	_tsl2561_gain = gain;
	_tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
			_tsl2561_integration | _tsl2561_gain);
	_tsl2561_disable();
}

void _tsl2561_setTiming(tsl2561IntegrationTime_t integration) {
	if (!_tsl2561_initialized)
		_tsl2561_begin();

	_tsl2561_enable();
	_tsl2561_integration = integration;
	_tsl2561_write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
			_tsl2561_integration | _tsl2561_gain);
	_tsl2561_disable();
}

uint32_t _tsl2561_calculateLux(uint16_t ch0, uint16_t ch1) {
	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;

	switch (_tsl2561_integration) {
	case TSL2561_INTEGRATIONTIME_13MS:
		chScale = TSL2561_LUX_CHSCALE_TINT0;
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		chScale = TSL2561_LUX_CHSCALE_TINT1;
		break;
	default: // No scaling ... integration time = 402ms
		chScale = (1 << TSL2561_LUX_CHSCALE);
		break;
	}

	// Scale for gain (1x or 16x)
	if (!_tsl2561_gain)
		chScale = chScale << 4;

	// scale the channel values
	channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

	// find the ratio of the channel values (Channel1/Channel0)
	unsigned long ratio1 = 0;
	if (channel0 != 0)
		ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE + 1)) / channel0;

	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;

	unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
	{	b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
	else if (ratio <= TSL2561_LUX_K2C)
	{	b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
	else if (ratio <= TSL2561_LUX_K3C)
	{	b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
	else if (ratio <= TSL2561_LUX_K4C)
	{	b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
	else if (ratio <= TSL2561_LUX_K5C)
	{	b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
	else if (ratio <= TSL2561_LUX_K6C)
	{	b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
	else if (ratio <= TSL2561_LUX_K7C)
	{	b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
	else if (ratio > TSL2561_LUX_K8C)
	{	b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T)) {
		b = TSL2561_LUX_B1T;
		m = TSL2561_LUX_M1T;
	} else if (ratio <= TSL2561_LUX_K2T) {
		b = TSL2561_LUX_B2T;
		m = TSL2561_LUX_M2T;
	} else if (ratio <= TSL2561_LUX_K3T) {
		b = TSL2561_LUX_B3T;
		m = TSL2561_LUX_M3T;
	} else if (ratio <= TSL2561_LUX_K4T) {
		b = TSL2561_LUX_B4T;
		m = TSL2561_LUX_M4T;
	} else if (ratio <= TSL2561_LUX_K5T) {
		b = TSL2561_LUX_B5T;
		m = TSL2561_LUX_M5T;
	} else if (ratio <= TSL2561_LUX_K6T) {
		b = TSL2561_LUX_B6T;
		m = TSL2561_LUX_M6T;
	} else if (ratio <= TSL2561_LUX_K7T) {
		b = TSL2561_LUX_B7T;
		m = TSL2561_LUX_M7T;
	} else if (ratio > TSL2561_LUX_K8T) {
		b = TSL2561_LUX_B8T;
		m = TSL2561_LUX_M8T;
	}
#endif

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));

	// do not allow negative lux value
	if (temp < 0)
		temp = 0;

	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (TSL2561_LUX_LUXSCALE - 1));

	// strip off fractional portion
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

	// Signal I2C had no errors
	return lux;
}

uint32_t _tsl2561_getFullLuminosity(void) {
	uint32_t x;
	if (!_tsl2561_initialized)
		_tsl2561_begin();

	// Enable the device by setting the control bit to 0x03
	_tsl2561_enable();

	// Wait x ms for ADC to complete
	switch (_tsl2561_integration) {
	case TSL2561_INTEGRATIONTIME_13MS:
		delay(14);
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		delay(102);
		break;
	default:
		delay(403);
		break;
	}

	x = _tsl2561_read16(
			TSL2561_COMMAND_BIT | TSL2561_WORD_BIT
					| TSL2561_REGISTER_CHAN1_LOW);
	x <<= 16;
	x |= _tsl2561_read16(
			TSL2561_COMMAND_BIT | TSL2561_WORD_BIT
					| TSL2561_REGISTER_CHAN0_LOW);

	_tsl2561_disable();

	return x;
}
uint16_t _tsl2561_getLuminosity(uint8_t channel) {

	uint32_t x = _tsl2561_getFullLuminosity();

	if (channel == 0) {
		// Reads two byte value from channel 0 (visible + infrared)
		return (x & 0xFFFF);
	} else if (channel == 1) {
		// Reads two byte value from channel 1 (infrared)
		return (x >> 16);
	} else if (channel == 2) {
		// Reads all and subtracts out just the visible!
		return ((x & 0xFFFF) - (x >> 16));
	}

	// unknown channel!
	return 0;
}

uint16_t _tsl2561_read16(uint8_t reg) {
	int8_t t_ret;
	uint16_t out = 0;	// output value of the function

	t_ret = obcl_readWordFromRegAddr(_tsl2561_addr, reg, &out, false);
	return(out);
}

void _tsl2561_write8(uint8_t reg, uint8_t value) {
	int8_t t_ret = obcl_writeByteToRegAddr(_tsl2561_addr, reg, value);
//	return(t_ret);
}




boolean tsl2561_init() {
	_tsl2561_construct(DRIVER_TSL2561_ADDR);
	return(_tsl2561_begin());
}

float tsl2561_getLux() {
	uint16_t ch0 = _tsl2561_getLuminosity(0);
	uint16_t ch1 = _tsl2561_getLuminosity(1);
	return (_tsl2561_calculateLux(ch0, ch1));
}

uint16_t tsl2561_getLuminosity(int8_t ch) {
	return (_tsl2561_getLuminosity(0));
}

#endif
