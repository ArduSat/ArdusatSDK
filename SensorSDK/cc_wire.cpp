/*
 * cc_wire.cpp
 *
 *  Created on: 25 sept. 2014
 *      Author: JF OMHOVER
 */
#include <Arduino.h>
#include <Wire.h>
#include "cc_wire.h"


void cc_wire_begin() {
	Wire.begin();
}

void cc_wire_beginAs(uint8_t a) {
	Wire.begin(a);
}
void cc_wire_beginTransmission(uint8_t a) {
		Wire.beginTransmission(a);
}

uint8_t cc_wire_endTransmission(boolean b) {
	Wire.endTransmission(b);
}

uint8_t cc_wire_requestFrom(uint8_t a, uint8_t l) {
	Wire.requestFrom(a,l);
}

size_t cc_wire_write(uint8_t v) {
	Wire.write(v);
}

size_t cc_wire_writeArray(const uint8_t * a, size_t s) {
	Wire.write(a,s);
}

int16_t cc_wire_available(void) {
	return(Wire.available());
}

int16_t cc_wire_read(void) {
	return(Wire.read());
}

int16_t cc_wire_peek(void) {
	return(Wire.peek());
}

void cc_wire_flush(void) {
	Wire.flush();
}

void cc_serial_printFloat(float f) {
	Serial.print("float: ");
	Serial.println(f);
}

