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
#include "obcl.h"	// for OBCL#include "config.h"
#include "sensor_addr.h"

uint8_t _tmp102_temp_i2c_addr;
byte _tmp102_buff[2];

void _tmp102_construct(uint8_t addr);
void _tmp102_init(); // jfomhover on 07/08/2013 : uint8_t nodeid not used anywhere

void _tmp102_construct(uint8_t addr) {
	_tmp102_temp_i2c_addr = addr;
}

void _tmp102_init() {
	obcl_begin();
}

boolean tmp102_init() {
	_tmp102_construct(DRIVER_TMP102_ADDR);
	_tmp102_init();
	return (true);
//	return(_tmp102_begin());
}

float tmp102_getTempCelsius() {
	uint16_t valDEUX;
	float tmp;
	obcl_readWordFromRegAddr(0x48, 0x00, (uint16_t *) &valDEUX, false);
	tmp = (valDEUX >> 4)*0.0625;
	return (tmp);
}
