#include <Arduino.h>
#include "output_csv.h"

#define CSV_SEPARATOR	';'
#define CSV_TEXTBUFFER_MAXSIZE	64

char _csv_buffer[CSV_TEXTBUFFER_MAXSIZE];
int len;


const char * dataToCSV(void * input) {
	// TODO : detect structure within, switch for the proper function
	return(NULL);
}

// warning : resets the internal buffer
const char * headerToCSV(_data_header_t * header) {
	if (header == NULL)
		return (NULL);

	_csv_buffer_reset();

	ultoa(header->timestamp, _csv_buffer, 10);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	utoa(header->sensor_id, _csv_buffer + len, 10);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return ((const char *) _csv_buffer);
}

const char * accelerationToCSV(acceleration_t * input) {
	if (input == NULL)
		return (NULL);

	headerToCSV((_data_header_t*) input);

	dtostrf(input->x, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	dtostrf(input->y, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	dtostrf(input->z, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return (_csv_buffer);
}

const char * temperatureToCSV(temperature_t * input) {
	if (input == NULL)
		return (NULL);

	headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

	dtostrf(input->t, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return (_csv_buffer);
}

const char * orientationToCSV(orientation_t * input) {
	if (input == NULL)
		return (NULL);

	headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

	dtostrf(input->pitch, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	dtostrf(input->roll, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	dtostrf(input->heading, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return (_csv_buffer);
}

const char * luminosityToCSV(luminosity_t * input) {
	if (input == NULL)
		return (NULL);

	headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

	dtostrf(input->lux, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return (_csv_buffer);
}

const char * uvlightToCSV(uvlight_t * input) {
	if (input == NULL)
		return (NULL);

	headerToCSV((_data_header_t*) input);// warning : resets the internal buffer

	dtostrf(input->uvindex, 2, 3, _csv_buffer + len);
	len = strlen(_csv_buffer);
	_csv_buffer[len++] = CSV_SEPARATOR;

	return (_csv_buffer);
}

void _csv_buffer_reset() {
	memset(_csv_buffer, 0, CSV_TEXTBUFFER_MAXSIZE);
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
