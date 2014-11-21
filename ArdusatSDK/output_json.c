#include <Arduino.h>
#include <stdio.h>

#include "output_json.h"
#include "units.h"

char JSON_PREFIX = '~';
char JSON_SUFFIX = '|';

char _json_buffer[JSON_TEXTBUFFER_MAXSIZE];
int len;

void _json_buffer_reset() {
	memset(_json_buffer, 0, JSON_TEXTBUFFER_MAXSIZE);
	len = 0;
}

int _writeJSONValue(char *buf, const char *sensor_name, const char *unit, float value)
{
	char num [10];
	// inexact estimate on the number of characters the value will take up...
	if (strlen(sensor_name) + strlen(unit) + 10 + len > JSON_TEXTBUFFER_MAXSIZE) {
		return -1;
	}
	dtostrf(value, 4, 2, num);
	len += sprintf(buf, "%c{\"sensorName\":\"%s\", \"unit\":\"%s\", \"value\": %s}%c\n",
								 JSON_PREFIX, sensor_name, unit, num, JSON_SUFFIX);
	return len;
}

const char * valueToJSON(const char *sensor_name, uint8_t unit, float value)
{
	_json_buffer_reset();
	_writeJSONValue(_json_buffer, sensor_name, unit_to_str(unit), value); 
	return _json_buffer;
}

const char * accelerationToJSON(const char *sensor_name, acceleration_t *acceleration)
{
	int nameLength = strlen(sensor_name);
	char nameBuf[nameLength + 2];
	_json_buffer_reset();	

	sprintf(nameBuf, "%sX", sensor_name);
	_writeJSONValue(_json_buffer, nameBuf, unit_to_str(acceleration->header.unit), acceleration->x);
	sprintf(nameBuf, "%sY", sensor_name);
	_writeJSONValue(&_json_buffer[len], nameBuf, unit_to_str(acceleration->header.unit), acceleration->y);
	sprintf(nameBuf, "%sZ", sensor_name);
	_writeJSONValue(&_json_buffer[len], nameBuf, unit_to_str(acceleration->header.unit), acceleration->z);
	return _json_buffer;
}

const char * orientationToJSON(const char *sensor_name, orientation_t *orient)
{
	int nameLength = strlen(sensor_name);
	char nameBuf[nameLength + 2];
	_json_buffer_reset();	

	sprintf(nameBuf, "%sRoll", sensor_name);
	_writeJSONValue(_json_buffer, "roll", unit_to_str(orient->header.unit), orient->roll);
	sprintf(nameBuf, "%sPitch", sensor_name);
	_writeJSONValue(&_json_buffer[len], nameBuf, unit_to_str(orient->header.unit), orient->pitch);
	sprintf(nameBuf, "%sHeading", sensor_name);
	_writeJSONValue(&_json_buffer[len], nameBuf, unit_to_str(orient->header.unit), orient->heading);
	return _json_buffer;
}

const char * temperatureToJSON(const char *sensor_name, temperature_t *temp)
{
	_json_buffer_reset();
	_writeJSONValue(_json_buffer, sensor_name, unit_to_str(temp->header.unit), temp->t);
	return _json_buffer;
}

const char * luminosityToJSON(const char *sensor_name, luminosity_t *lum)
{
	_json_buffer_reset();
	_writeJSONValue(_json_buffer, sensor_name, unit_to_str(lum->header.unit), lum->lux);
	return _json_buffer;
}

const char * uvlightToJSON(const char *sensor_name, uvlight_t *input)
{
	_json_buffer_reset();
	_writeJSONValue(_json_buffer, sensor_name, unit_to_str(input->header.unit), input->uvindex);
	return _json_buffer;
}
