/*
 * c_wire.h
 *
 *  Created on: 25 sept. 2014
 *      Author: JF OMHOVER
 */

#ifndef C_WIRE_H_
#define C_WIRE_H_

#ifdef __cplusplus
extern "C" {
#endif

void cc_wire_begin();
void cc_wire_beginAs(uint8_t);
void cc_wire_beginTransmission(uint8_t);

uint8_t cc_wire_endTransmission(void);
uint8_t cc_wire_requestFrom(uint8_t, uint8_t);
size_t cc_wire_write(uint8_t);
size_t cc_wire_writeArray(const uint8_t *, size_t);

int16_t cc_wire_available(void);
int16_t cc_wire_read(void);
int16_t cc_wire_peek(void);
void cc_wire_flush(void);


#ifdef __cplusplus
} // extern "C"
#endif

#endif /* C_WIRE_H_ */
