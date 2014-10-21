/*
 *   @file nanosat_message.h
 *   @version 1.0
 *   @name NanoSatisfi Inc.
 *
 *   @section LICENSE
 *
 *   see LICENSE and NOTICE files.
 *
 *   @section DESCRIPTION
 *   Message passing schema between OnboardCommLayer (I2C wrapper) and Payload
 *   supervisor. This interface exposes system calls to user space and sensor
 *   access. The main interaction players are Payload Supervisor(assv) and
 *   experiment nodes.
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <Arduino.h>
#include "obcl.h"
#include "cc_wire.h"
#include "nanosat_message.h"
#include "sensor_addr.h"

#define _CURRENT_NODE_ADDR	0x01

static uint8_t _obcl_transmitBuffer[sizeof(nanosat_message_t)];
static uint8_t _obcl_assvAddress_= I2C_ADD_ASSV_1; // jfomhover on 07/08/2013 : should be the address of the master arduino in the Sat
static uint8_t _obcl_localAddress_= I2C_COMM_BEGINASMASTER;	// the local address of the I2C node
static boolean _obcl_hasBegun_= false;// true if the comm has already been Wire.begin() somehow
static int _obcl_filed;


// provides the error value for the kind of errors happening at Wire.endTransmission()
int8_t obcl_endTransmissionErrorCode(byte retValue, int sizeSent) {
	switch (retValue) {
	case 0:	// success
		return ((int8_t) sizeSent);
	case 1: // data too long to fit in transmit buffer
		return (I2C_COMM_ERRORTOOMUCHDATA);
	case 2: // received NACK on transmit of address
		return (I2C_COMM_ERRORNACKADDR);
	case 3: // received NACK on transmit of data
		return (I2C_COMM_ERRORNACKDATA);
	case 4: // other error
	default:
		return (I2C_COMM_ERROR);
	}
}

// use buff = NULL for no answer
int8_t obcl_sendReg_recvBufLen(int filed, uint8_t reg, uint8_t * buffer, uint8_t len,
		uint8_t * recdlen) {

	cc_wire_beginTransmission(filed);
	if (cc_wire_write(reg) < 1)
		return (I2C_COMM_ERRORNOWRITE);
	int8_t t_ret = cc_wire_endTransmission(false);

	if (t_ret < 0) {
		return (obcl_endTransmissionErrorCode(t_ret, 1));
	}

	if (buffer == NULL)
		return (I2C_COMM_OK);

	delayMicroseconds(2);

	cc_wire_beginTransmission(filed);
	cc_wire_requestFrom((byte) filed, (byte) len);

	// reading data loop

	uint8_t obtainedData = 0;

	while (cc_wire_available() > 0) {
		buffer[obtainedData++] = cc_wire_read();
	}

	t_ret = cc_wire_endTransmission(true);         // end transmission

	if (recdlen != NULL)
		recdlen[0] = obtainedData;

	if (obtainedData < len)
		return (I2C_COMM_NOTENOUGHDATA);

	return (obcl_endTransmissionErrorCode(t_ret, obtainedData));
}

int8_t obcl_recvBufLen(int filed, uint8_t * buffer, uint8_t len,
		uint8_t * recdlen) {
	int8_t t_ret = 0;

	if (buffer == NULL)
		return (I2C_COMM_OK);

	cc_wire_beginTransmission(filed);
	cc_wire_requestFrom((byte) filed, (byte) len);

	// reading data loop

	uint8_t obtainedData = 0;

	while (cc_wire_available() > 0) {
		buffer[obtainedData++] = cc_wire_read();
	}

	t_ret = cc_wire_endTransmission(true);         // end transmission

	if (recdlen != NULL)
		recdlen[0] = obtainedData;

	if (obtainedData < len)
		return (I2C_COMM_NOTENOUGHDATA);

	return (obcl_endTransmissionErrorCode(t_ret, obtainedData));
}

int8_t obcl_sendCmdVal_recvBufLen(int filed, uint8_t cmd, uint8_t val,
		uint8_t * buffer, uint8_t len, uint8_t * recdlen) {

	cc_wire_beginTransmission(filed);
	cc_wire_write(cmd);
	cc_wire_write(val);
	int8_t t_ret = cc_wire_endTransmission(false);

	if (t_ret < 0) {
		return (obcl_endTransmissionErrorCode(t_ret, 2));
	}

	if (buffer == NULL)
		return (I2C_COMM_OK);

	delayMicroseconds(2);

	cc_wire_beginTransmission(filed);
	cc_wire_requestFrom((byte) filed, (byte) len);

	// reading data loop

	uint8_t obtainedData = 0;

	while (cc_wire_available() > 0) {
		buffer[obtainedData++] = cc_wire_read();
	}

	t_ret = cc_wire_endTransmission(true);         // end transmission

	if (recdlen != NULL)
		recdlen[0] = obtainedData;

	if (obtainedData < len)
		return (I2C_COMM_NOTENOUGHDATA);

	return (obcl_endTransmissionErrorCode(t_ret, obtainedData));
}


// *** AppStorage + Camera sketches
uint8_t obcl_sendMessage(nanosat_message_t msg) {
	memcpy(_obcl_transmitBuffer, &msg, sizeof(nanosat_message_t)); // jfomhover on 2013/08/12 : is this necessary since we're not multitasking ?

	if (obcl_sendBuffer(_obcl_assvAddress_, _obcl_transmitBuffer,
			sizeof(nanosat_message_t)) < 0)
		return (0);
	else
		return (sizeof(nanosat_message_t));
}

// *** AppStorage + example sketches
uint8_t obcl_sendExit() {
	nanosat_message_t msg = {
	NODE_COMM_MESSAGE_PREFIX, EXIT, _CURRENT_NODE_ADDR };
	return (obcl_sendMessage(msg));
}

// *** everywhere
boolean obcl_begin() {
	return(obcl_beginAs(I2C_COMM_BEGINASMASTER));
}

boolean obcl_beginAs(uint8_t localAddr) {
	if (_obcl_hasBegun_)
		return (true); // if is already begun, don't do it again (aren't you supposed not to ?)

	_obcl_localAddress_ = _CURRENT_NODE_ADDR;

	if (localAddr == I2C_COMM_BEGINASMASTER)
		cc_wire_begin();
	else
		cc_wire_beginAs(_obcl_localAddress_);

	_obcl_hasBegun_ = true;
	_obcl_filed = 0;
	return (_obcl_hasBegun_);
}

// returns the local address
uint8_t obcl_getAddress() {
	return (_obcl_localAddress_);	// useless on RPI
}

// switch the target device's address
int8_t obcl_switchAddress(uint8_t destinationAddr) {
	// no need for nothing here, set filed for compatibility
	_obcl_filed = destinationAddr;
//	Wire.beginTransmission(destinationAddr);
	return (I2C_COMM_OK);
}

// flush the Wire (needed in geiger_sensor_poller.ino, dunno why)
void obcl_flushWrite() {
	cc_wire_flush();
}

// flush the Wire (needed in geiger_sensor_poller.ino, dunno why)
void obcl_flushRead() {
	while (cc_wire_available())
		cc_wire_read();
}

// sends the whole buffer of values to destinationAddr,
// and if the sending fails (you never know), try again until if works or timeout reached
int8_t obcl_sendBuffer(uint8_t devAddr, uint8_t * buffer,
		int size) {
	int i;
	cc_wire_beginTransmission(devAddr);

	int sentCount = 0;

	for (i = 0; i < size; i++) {
		sentCount += cc_wire_write(buffer[i]);
	}
	// TODO : i don't know what to do here... use the timeout or whatever ?
	/*	if (sentCount < size) {

	 }
	 */

	byte t_ret = cc_wire_endTransmission(true);
	return (obcl_endTransmissionErrorCode(t_ret, sentCount));
}

// *** Lum
// sends a byte (register probably), and expect one from destinationAddr
int8_t obcl_readByteFromRegAddr(uint8_t devAddr, uint8_t regaddr,
		uint8_t * receivedBytePtr) {

	int8_t t_ret = 0;
	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	return (obcl_sendReg_recvBufLen(_obcl_filed, regaddr, receivedBytePtr, 1, NULL));
}

// *** Accel, Gyro, Lum, Mag
// write a byte into the device's register at specified address
int8_t obcl_writeByteToRegAddr(uint8_t devAddr, uint8_t regaddr,
		uint8_t value) {

	int8_t t_ret = 0;
	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	return (obcl_sendCmdVal_recvBufLen(_obcl_filed, regaddr, value, NULL, 0, NULL));
}

// *** Lum
// calls on the sourceAddr and request a 16 bits int from a REG
int8_t obcl_readWordFromRegAddr(uint8_t devAddr, uint8_t regaddr,
		uint16_t * receivedIntPtr, // a pointer to the byte that will be filled by the received value
		boolean order) {

	int8_t t_ret = 0;
	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	uint8_t * array = (uint8_t *)receivedIntPtr;
	t_ret = obcl_sendReg_recvBufLen(_obcl_filed, regaddr, (uint8_t *) array, 2, NULL);
	if (!order) {
		uint8_t t_b = array[0];
		array[0] = array[1];
		array[1] = t_b;
	}
	return(t_ret);
}

// *** Accel, Gyro, InfraTherm, Geiger
// calls on the devaddr, request a byte array from register
int8_t obcl_readByteArrayFromRegAddr(uint8_t devAddr,// address of the I2C device
		uint8_t regaddr,		// memory address of the register
		byte * recBuffer,		// buffer for the received values
		uint8_t expectedLen,	// length of the expected data array
		uint8_t * receivedLen)	// where to put the length actually received
		{

	int8_t t_ret = 0;
	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	return (obcl_sendReg_recvBufLen(_obcl_filed, regaddr, recBuffer, expectedLen, receivedLen));
}

// *** Temp
// calls on the sourceAddr and request 2 bytes
int8_t obcl_readWord(uint8_t devAddr, uint16_t * receivedIntPtr,// a pointer to the int that will be filled by the received value
		boolean order)
		{
	int8_t t_ret = 0;

	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	t_ret = obcl_recvBufLen(_obcl_filed, (uint8_t *) receivedIntPtr, 2, NULL);
//	t_ret = obcl_sendReg_recvBufLen(_obcl_filed, 0x00, (uint8_t *) receivedIntPtr, 2, NULL);
	if (t_ret < 0)
		return (t_ret);

	if (order) {
		uint8_t * t_ptr = (uint8_t *) receivedIntPtr;
		uint8_t b = t_ptr[0];
		t_ptr[0] = t_ptr[1];
		t_ptr[1] = b;
	}
	return (I2C_COMM_OK);
}

// *** Mag
// calls on the sourceAddr and request a compile a MSB and LSB kind of int
int8_t obcl_readWordFromMSBLSBAddr(uint8_t devAddr,
		uint8_t msb_reg, uint8_t lsb_reg, uint16_t * receivedIntPtr)
		{

	int8_t t_ret = 0;
	if ((t_ret = obcl_switchAddress(devAddr)) < 0)
		return (I2C_COMM_ERRORNACKADDR);

	uint8_t t_array[2];

	if ((t_ret = obcl_sendReg_recvBufLen(_obcl_filed, msb_reg, t_array + 1, 1, NULL)) < 0)
		return (t_ret);
	if ((t_ret = obcl_sendReg_recvBufLen(_obcl_filed, lsb_reg, t_array, 1, NULL)) < 0)
		return (t_ret);

	memcpy((void*) receivedIntPtr, (void*) t_array, 2);
	return (I2C_COMM_OK);
}

