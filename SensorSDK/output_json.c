#include <Arduino.h>
#include "output_csv.h"

// ~{ "sensorName":"accelerometerX", "unit":"m/s^2", "value": 5.3 }|

#define JSON_PREFIX		'~'
#define JSON_BEGIN		'{'
#define JSON_VALUESEP	':'
#define JSON_FIELDSEP	','
#define JSON_STR		'\"'
#define JSON_END		'}'
#define JSON_SUFFIX		'|'

#define JSON_TEXTBUFFER_MAXSIZE	256

char _json_buffer[JSON_TEXTBUFFER_MAXSIZE];
int len;


const char * dataToJSON(void * input) {
	// TODO : detect structure within, switch for the proper function
	return(NULL);
}


boolean _output_json_appendChar(const char c) {
	if (len < JSON_TEXTBUFFER_MAXSIZE) {
		_json_buffer[len++] = c;
		return(true);
	} else {
		return(false);
	}
}

boolean _output_json_appendStr(const char * str) {
	if (str == NULL)
		return(true);

	if ((len+strlen(str)) > JSON_TEXTBUFFER_MAXSIZE)
		return(false);

	memcpy(_json_buffer+len, str, strlen(str));
	len += strlen(str);
	return(true);
}

boolean _output_json_appendQuoteStr(const char * str) {
	if (str == NULL)
		return(true);

	if ((len+strlen(str)+2) > JSON_TEXTBUFFER_MAXSIZE)
		return(false);

	_json_buffer[len++] = JSON_STR;
	memcpy(_json_buffer+len, str, strlen(str));
	len += strlen(str);
	_json_buffer[len++] = JSON_STR;

	return(true);
}

boolean _output_json_appendFloat(float f, int precision) {
	if ((len + 10) >= JSON_TEXTBUFFER_MAXSIZE)
		return(false);

	dtostrf(f, 2, precision, _json_buffer + len);
	len = strlen(_json_buffer);
	return(true);
}

// MODEL  ~{ "sensorName":"accelerometerX", "unit":"m/s^2", "value": 5.3 }|
boolean _output_json_appendValue(const char * sensorname, const char * unit, float value, int precision) {
	if (!_output_json_appendChar(JSON_PREFIX))
		return(false);
	if (!_output_json_appendChar(JSON_BEGIN))
		return(false);

	if (!_output_json_appendQuoteStr("sensorName"))
		return(false);
	if (!_output_json_appendChar(JSON_VALUESEP))
		return(false);
	if (!_output_json_appendQuoteStr(sensorname))
		return(false);

	if (!_output_json_appendChar(JSON_FIELDSEP))
		return(false);

	if (!_output_json_appendQuoteStr("unit"))
		return(false);
	if (!_output_json_appendChar(JSON_VALUESEP))
		return(false);
	if (!_output_json_appendQuoteStr(unit))
		return(false);

	if (!_output_json_appendChar(JSON_FIELDSEP))
		return(false);

	if (!_output_json_appendQuoteStr("value"))
		return(false);
	if (!_output_json_appendChar(JSON_VALUESEP))
		return(false);
	if (!_output_json_appendFloat(value, precision))
		return(false);

	if (!_output_json_appendChar(JSON_END))
		return(false);
	if (!_output_json_appendChar(JSON_SUFFIX))
		return(false);
}


const char * accelerationToJSON(acceleration_t * input) {
	if (input == NULL)
		return (NULL);

	_json_buffer_reset();

	// TODO : use buffer exception ?
	_output_json_appendValue("accelerationX", "m/s^2", input->x, 3);
	_output_json_appendValue("accelerationY", "m/s^2", input->y, 3);
	_output_json_appendValue("accelerationZ", "m/s^2", input->z, 3);

	return (_json_buffer);
}

const char * temperatureToJSON(temperature_t * input) {
	if (input == NULL)
		return (NULL);

	_json_buffer_reset();

	// TODO : use buffer exception ?
	_output_json_appendValue("temperature", "°C", input->t, 2);	// TODO : use precision in config file ?

	return (_json_buffer);
}

const char * orientationToJSON(orientation_t * input) {
	if (input == NULL)
		return (NULL);

	_json_buffer_reset();

	// TODO : use buffer exception ?
	_output_json_appendValue("orientationROLL", "degrees", input->roll, 3);
	_output_json_appendValue("orientationPITCH", "degrees", input->pitch, 3);
	_output_json_appendValue("orientationHEADING", "degrees", input->heading, 3);

	return (_json_buffer);
}

const char * luminosityToJSON(luminosity_t * input) {
	if (input == NULL)
		return (NULL);

	_json_buffer_reset();

	// TODO : use buffer exception ?
	_output_json_appendValue("luminosity", "lux", input->lux, 2);

	return (_json_buffer);
}

void _json_buffer_reset() {
	memset(_json_buffer, 0, JSON_TEXTBUFFER_MAXSIZE);
	len = 0;
}


/*

 class PrintBuffer {
 private:
 char _buffer[16];

 public:
 void reset() {
 memset(_buffer, '\0', 16);
 }
 void print(int8_t val) {
 reset();
 itoa(val, _buffer, 10);
 }
 void print(int16_t val) {
 reset();
 itoa(val, _buffer, 10);
 }
 void print(int32_t val) {
 reset();
 ltoa(val, _buffer, 10);
 }
 void print(uint8_t val) {
 reset();
 utoa(val, _buffer, 10);
 }
 void print(uint16_t val) {
 reset();
 utoa(val, _buffer, 10);
 }
 void print(uint32_t val) {
 reset();
 ultoa(val, _buffer, 10);
 }
 void print(float val) {
 reset();
 dtostrf(val, 2, 3, _buffer);
 }
 void print(double val) {
 reset();
 dtostrf(val, 2, 3, _buffer);
 }
 int getLength() {
 return(strlen(_buffer));
 }
 const char * get() {
 return(_buffer);
 }
 };
 */
