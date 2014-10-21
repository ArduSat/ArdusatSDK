// encapsulating Arduino Wire library functions into CC functions

#ifndef C_WIRE_H_
#define C_WIRE_H_

#ifdef __cplusplus
extern "C" {
#endif

void cc_wire_begin();								// Wire.begin
void cc_wire_beginAs(uint8_t);						// Wire.begin(address)
void cc_wire_beginTransmission(uint8_t);			// Wire.beginTransmission(address)

uint8_t cc_wire_endTransmission(boolean);				// Wire.endTransmission()
uint8_t cc_wire_requestFrom(uint8_t, uint8_t);		// Wire.requestfrom(addr, reg);
size_t cc_wire_write(uint8_t);						// Wire.write(char)
size_t cc_wire_writeArray(const uint8_t *, size_t);	// Wire.write several times

int16_t cc_wire_available(void);					// Wire.available()
int16_t cc_wire_read(void);							// Wire.read()
int16_t cc_wire_peek(void);							// Wire.peek()
void cc_wire_flush(void);							// Wire.flush()

void cc_serial_printFloat(float);


#ifdef __cplusplus
} // extern "C"
#endif

#endif /* C_WIRE_H_ */
