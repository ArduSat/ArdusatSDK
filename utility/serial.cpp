/**
 * @file   serial.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   January 7, 2015
 * @brief  Ardusat SDK Unified serial library to wrap software and hardware 
 *         serial libraries under one interface.
 */

#include <stdlib.h>

#include <ArdusatSDK.h>
#include "serial.h"

const char no_software_params_err_msg[] PROGMEM = "Uh oh, you specified a software serial mode but didn't specify transmit/recieve pins! Halting program...";
const char xbee_cmd_mode[] PROGMEM = "+++";
const char xbee_cmd_ack[] PROGMEM = "OK";
const char xbee_cmd_baud[] PROGMEM = "ATBD ";
const char xbee_cmd_write[] PROGMEM = "ATWR";
const char xbee_cmd_close[] PROGMEM = "ATCN";
const char xbee_baud_success[] PROGMEM = "Set XBEE baud rate to ";

#define send_to_serial(function) \
  if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) { \
    Serial.function; \
  } \
  if (_soft_serial && (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE)) { \
    _soft_serial->function; \
  }

#define return_serial_function(fxn) \
  if (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) { \
    return _soft_serial->fxn; \
  } else { \
    return Serial.fxn; \
  }

/**
 * If only a mode is specified to the constructor, we need to be using hardware serial.
 *
 * Prints error message and halts execution if software serial with no options is requested.
 */
ArdusatSerial::ArdusatSerial(serialMode mode)
{
  // If no other arguments (pins) are given, we can't initialize a software serial 
  // connection.
  _mode = SERIAL_MODE_HARDWARE;
}

/**
 * Constructor with serial mode and connection params for software serial.
 */
ArdusatSerial::ArdusatSerial(serialMode mode, unsigned char softwareReceivePin,
                             unsigned char softwareTransmitPin, bool softwareInverseLogic)
{
  if (mode == SERIAL_MODE_SOFTWARE || mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    _soft_serial = new SoftwareSerial(softwareReceivePin, softwareTransmitPin,
                                      softwareInverseLogic);
  }

  _mode = mode;
}

ArdusatSerial::~ArdusatSerial()
{
  if (_soft_serial != NULL) {
    delete _soft_serial;
  }
}

/**
 * Begin serial communications at the specified baud rate. Optionally attempts to
 * set the Xbee unit attached to the SoftwareSerial port to the specified baud rate.
 *
 * Note that baud rates above ~57600 are not well-supported by SoftwareSerial, and 
 * even 57600 may cause some bugs.
 *
 * @param baud rate for serial communications
 * @param setXbeeSpeed flag to optionally attempt to set the Xbee unit to the baud rate 
 */
void ArdusatSerial::begin(unsigned long baud, bool setXbeeSpeed)
{
  if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    Serial.begin(baud);
  }

  if (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    if (setXbeeSpeed) {
      set_xbee_baud_rate(_soft_serial, baud);
    }
    _soft_serial->end();
    _soft_serial->begin(baud);
  }
}

void ArdusatSerial::end()
{
  send_to_serial(end())
}

int ArdusatSerial::peek()
{
  return_serial_function(peek())
}

int ArdusatSerial::read()
{
  return_serial_function(read())
}

int ArdusatSerial::available()
{
  return_serial_function(available())
}

void ArdusatSerial::flush()
{
  send_to_serial(flush())
}

size_t ArdusatSerial::write(unsigned char b) {
  size_t ret = 1;

  if (_soft_serial != NULL && 
      ( _mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE)) {
    ret = ret & _soft_serial->write(b);
  }

  if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    ret = ret & Serial.write(b);
  }

  return ret;
}

/**
 * Enters command mode on the xbee by trying various speeds until one is found that
 * works.
 *
 * @returns 0 if successful, -1 if unsuccessful
 */
int _enter_xbee_cmd_mode(SoftwareSerial *serial, unsigned long speed)
{
  char buf[5];
  int i;
  // These are all the rates supported by xbee hardware. To save time on boot,
  // just check for some of the most commonly used
  //unsigned long rates[] = {  9600, 19200, 38400, 57600, 115200, 4800, 2400, 1200 };
  unsigned long rates[] = {  9600, 57600, 115200, 19200, 38400 };
  strcpy_P(buf, xbee_cmd_mode);

  serial->end();
  // Give the Xbee time to boot after reset
  delay(1200);
  serial->begin(speed);
  serial->print(buf);
  delay(1200);
  if (!serial->available()) {
    for (i = 0; i < 5; ++i) {
      serial->end();
      serial->begin(rates[i]);
      serial->print(buf);
      delay(1200);
      if (serial->available()) {
        speed = rates[i];
        break;
      }

      // if we get here we have no more speeds to try...
      if (i == 4) {
        return -1;
      }
    }
  }

  for (i = 0; i < 2; ++i) {
    buf[i] = serial->read();
    if (buf[i] == -1) {
      break;
    }
  }

  strcpy_P(buf+2, xbee_cmd_ack);
  //HACK: at higher baud rates (57600+), we don't get proper "OK" ack codes
  //      back from the xbee chip. But, we can still successfully write to the 
  //      xbee. For now, ignore check on ack, and just go ahead with trying to
  //      change the speed if we get serial->available() at the given speed.
  //return strncmp(buf, buf+2, 2);
  return 0;
}

void set_xbee_baud_rate(Stream *serial, unsigned long speed)
{
  char buf [30];
  unsigned char rate;

  if (_enter_xbee_cmd_mode((SoftwareSerial *) serial, speed) == 0) {
    switch(speed) {
      case 1200:
        rate = 0;
        break;
      case 2400:
        rate = 1;
        break;
      case 4800:
        rate = 2;
        break;
      case 9600:
        rate = 3;
        break;
      case 19200:
        rate = 4;
        break;
      case 38400:
        rate = 5;
        break;
      case 57600:
        rate = 6;
        break;
      case 115200:
        rate = 7;
        break;
      default:
        //default to 57600 if no valid rate specified
        //(fastest bug-free software serial rate)
        rate = 6;
        break;
    }
    strcpy_P(buf, xbee_cmd_baud);
    serial->print(buf);
    serial->println(rate);
    strcpy_P(buf, xbee_cmd_write);
    serial->println(buf);
    delay(1200);
    strcpy_P(buf, xbee_cmd_close);
    serial->println(buf);
    delay(1200);
    strcpy_P(buf, xbee_baud_success);
    Serial.print(buf);
    Serial.println(speed);
  }
}
