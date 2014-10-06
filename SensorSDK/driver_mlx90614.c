#include <Arduino.h>
#include "config.h"
#include "obcl.h"

#define MLX90614_FACTOR		0.02 // InfraTherm has a resolution of .02

boolean mlx90614_init() {
	obcl_begin();
	return(true);
}

float mlx90614_getTempCelsius() {
	int8_t data_t[3];
	uint8_t recdSize = 0;
	int8_t t_ret = 0;
	int8_t data_low, data_high, pec;
	int16_t rawData = 0;
	float tempData;

	t_ret = obcl_readByteArrayFromRegAddr(DRIVER_MLX90614_ADDR, 0x07, (uint8_t*)data_t, 3, &recdSize);

	if (t_ret < 0)
		return 0.0;

	// jfomhover 08/09/2013 : below, I didn't want to ruin the clarity of the formula, but these variables can be skipped
	data_low = data_t[0]; //read first byte
	data_high = data_t[1]; //read second byte
	pec = data_t[2]; //read checksum

	// This masks off the error bit of the high byte,
	// then moves it left 8 bits and adds the low byte.
	// Taken from bildr forum on InfraTherm
	rawData = (((data_high & 0x007F) << 8) + data_low);
	tempData = (float) (rawData);
	//multiply by resolution and account for error to convert to Kelvin
	tempData = (tempData * MLX90614_FACTOR) - 0.01;
	tempData = tempData - 273.15;	//convert from Kelvin to Celsius
	return(tempData);
}
