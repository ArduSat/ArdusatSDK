#ifndef _OBCL_H_
#define _OBCL_H_

#include <Arduino.h>
#include "sensor_addr.h"
#include "nanosat_message.h"

// uncomment the #define below to activate the reading of localAddress_ in EEPROM addr 0x00
//#define I2CCOMM_REALNODE

#define I2C_COMM_BEGINASMASTER	0	// use that if you want to begin() as the master of the I2C bus
#define I2C_COMM_BLOCKTIMEOUT	-1	// use that to block read functions until the full expected data is received#define I2C_COMM_INSTANTTIMEOUT	0	// by default, don't wait, data is available or die !// possible return values#define I2C_COMM_OK					0	// everything's perfect !#define I2C_COMM_ERROR				-1	// generic error (allo Houston ?)#define I2C_COMM_ERRORTIMEOUT		-2	// timeout reached before receiving values#define I2C_COMM_ERRORNOWRITE		-3	// writing doesn't seem to work (Wire.write() return values mismatch)#define I2C_COMM_ERRORTOOMUCHDATA	-4	// data given to the transmission exceeds the internal buffer#define I2C_COMM_ERRORNACKADDR		-5	// typical return value when there's no OnboardCommLayer listening / connected#define I2C_COMM_ERRORNACKDATA		-6	// ???#define I2C_COMM_NOTENOUGHDATA		-7	// received less data than expected
#ifdef __cplusplus
extern "C" {
#endif

// *** used in : AppStorage + Camera sketches
uint8_t obcl_sendMessage(nanosat_message_t msg);
// *** used in : example sketches
uint8_t obcl_sendExit();

void obcl_flushWrite();	// *** (needed in geiger_sensor_poller.ino, dunno why)
void obcl_flushRead();	// *** (needed in geiger_sensor_poller.ino, dunno why)
int8_t obcl_sendBuffer(uint8_t devAddr, uint8_t * buffer, int size);

void obcl_scan();	// scans the I2C bus
boolean obcl_begin(); // by default, begin as master of the I2C bus
boolean obcl_beginAs(uint8_t localAddr);
uint8_t obcl_getAddress();
int8_t obcl_switchAddress(uint8_t destinationAddr);

// sends a byte, and expect one from destinationAddr
int8_t obcl_readByteFromRegAddr(uint8_t devaddr, uint8_t regaddr,
		uint8_t * receivedBytePtr);

// write a byte into the device's register at specified address
int8_t obcl_writeByteToRegAddr(uint8_t devaddr, uint8_t regaddr, uint8_t value);

// calls on the sourceAddr and request a 16 bits int from a REG
int8_t obcl_readWordFromRegAddr(uint8_t devaddr, uint8_t reg,
		uint16_t * receivedIntPtr, // a pointer to the byte that will be filled by the received value
		boolean order);				// true for MSB-LSB, false for LSB-MSB

// calls on the devaddr, request a byte array from register
int8_t obcl_readByteArrayFromRegAddr(uint8_t devaddr,// address of the I2C device
		uint8_t regaddr,		// memory address of the register
		byte * recBuffer,		// buffer for the received values
		uint8_t expectedLen,	// length of the expected data array
		uint8_t * receivedLen);	// where to put the length actually received

// calls on the sourceAddr and request 2 bytes
int8_t obcl_readWord(uint8_t devaddr, uint16_t * receivedIntPtr,// a pointer to the int that will be filled by the received value
		boolean order);				// true for MSB-LSB, false for LSB-MSB

// calls on the sourceAddr and request a compile a MSB and LSB kind of int
int8_t obcl_readWordFromMSBLSBAddr(uint8_t devaddr, uint8_t msb_reg,
		uint8_t lsb_reg, uint16_t * receivedIntPtr);// a pointer to the byte that will be filled by the received value

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* _OBCL_H_ */
