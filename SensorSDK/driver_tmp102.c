/** @brief  Library to query the on board temperature sensors.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    @author Jorge Ortiz and NanoSatisfi, Inc.
    @date May 2013
*/

#include <Arduino.h>
#include "obcl.h"	// for OBCL
#include "config.h"

uint8_t _tmp102_temp_i2c_addr;
byte _tmp102_buff[2];

void _tmp102_construct(uint8_t addr);
void _tmp102_init(); // jfomhover on 07/08/2013 : uint8_t nodeid not used anywhere
float _tmp102_getTemp();    //returns temperature in celcius
int16_t _tmp102_getRawTemp(); // returns the raw value from the sensor (12bits)


void _tmp102_construct(uint8_t addr) {
	_tmp102_temp_i2c_addr = addr;
}

void _tmp102_init() {
	obcl_begin();
}

float _tmp102_getTemp() {
	int16_t TemperatureSum = _tmp102_getRawTemp();
	float celsius = TemperatureSum * 0.0625;

	return celsius;
}

int16_t _tmp102_getRawTemp() {
	int16_t TemperatureSum = 0;
	int8_t t_ret = 0;

	t_ret = obcl_readWord(_tmp102_temp_i2c_addr, (uint16_t*) (&TemperatureSum), true);

	TemperatureSum = TemperatureSum >> 4;

	return (TemperatureSum);
}


boolean tmp102_init() {
	_tmp102_construct(DRIVER_TMP102_ADDR);
	_tmp102_init();
	return(true);
//	return(_tmp102_begin());
}

float tmp102_getTempCelsius() {
	return (_tmp102_getTemp());
}
