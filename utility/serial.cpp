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

const char no_software_params_err_msg[] PROGMEM = "You specified a software serial mode but didn't specify transmit/recieve pins!";

#ifdef __AVR__
  #define send_to_serial(function) \
    if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) { \
      Serial.function; \
    } \
    if (_soft_serial && (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE)) { \
      _soft_serial->function; \
    }
#elif defined(ESP32)
    #define send_to_serial(function) Serial.function;
#endif


#ifdef __AVR__
  #define return_serial_function(fxn) \
    if (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) { \
      return _soft_serial->fxn; \
    } else { \
      return Serial.fxn; \
    }
#elif defined(ESP32)
  #define return_serial_function(fxn) return Serial.fxn;
#endif

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
#ifdef __AVR__
  if (mode == SERIAL_MODE_SOFTWARE || mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    _soft_serial = new SoftwareSerial(softwareReceivePin, softwareTransmitPin,
                                      softwareInverseLogic);
  }
#endif
  _mode = mode;
}

ArdusatSerial::~ArdusatSerial()
{
#ifdef __AVR__
  if (_soft_serial != NULL) {
    delete _soft_serial;
  }
#endif
}

/**
 * Begin serial communications at the specified baud rate.
 *
 * Note that baud rates above ~57600 are not well-supported by SoftwareSerial, and 
 * even 57600 may cause some bugs.
 *
 * @param baud rate for serial communications
 */
void ArdusatSerial::begin(unsigned long baud)
{
#ifdef __AVR__
  if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    Serial.begin(baud);
  }

  if (_mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    _soft_serial->end();
    _soft_serial->begin(baud);
  }
#elif defined(ESP32)
  Serial.begin(baud);
#endif
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

#ifdef __AVR__
  if (_soft_serial != NULL && 
      ( _mode == SERIAL_MODE_SOFTWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE)) {
    ret = ret & _soft_serial->write(b);
  }

  if (_mode == SERIAL_MODE_HARDWARE || _mode == SERIAL_MODE_HARDWARE_AND_SOFTWARE) {
    ret = ret & Serial.write(b);
  }
#elif defined(ESP32)
  ret = ret & Serial.write(b);
#endif
  return ret;
}
