/**
 * @file   ArdusatLogging.h
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  ArdusatLogging logs sensor readings
 */

#ifndef ARDUSATLOGGING_H_
#define ARDUSATLOGGING_H_

#include <Arduino.h>
#include <utility/SdFat.h>
#include <utility/drivers.h>
#include <avr/pgmspace.h>
#include <utility/SdVolume.h>
#include <utility/MemoryFree.h>
#include <utility/BinaryDataFmt.h>
#include <utility/serial.h>
#include <utility/RTClib.h>

#include "ArdusatSensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Write functions take care of persisting data to an SD card
 *
 * Data can be written in any format, including a custom binary format for
 * space efficiency, by using the writeBytes function that takes a byte array.
 *
 * The individual helpers write functions automate the process of saving data
 * for each individual value, but write in CSV format, which is easier to consume
 * after the experiment, but takes up more space.
 *
 * Finally, binaryWrite functions write optimized binary representations of the
 * data. This helps save SD card space, but means that the data must be decoded
 * after download before it can be used.
 */
bool beginDataLog(int chipSelectPin, const char *fileNamePrefix, bool csvData);

int writeBytes(const uint8_t *buffer, uint8_t numBytes);
int writeString(const char *output);
int writeAcceleration(const char *sensorName, acceleration_t *data);
int writeMagnetic(const char *sensorName, magnetic_t *data);
int writeGyro(const char *sensorName, gyro_t *data);
int writeTemperature(const char *sensorName, temperature_t *data);
int writeLuminosity(const char *sensorName, luminosity_t *data);
int writeUVLight(const char *sensorName, uvlight_t *data);
int writeOrientation(const char *sensorName, orientation_t *data);
int writePressure(const char *sensorName, pressure_t *data);

int binaryWriteAcceleration(const uint8_t sensorId, acceleration_t *data);
int binaryWriteMagnetic(const uint8_t sensorId, magnetic_t *data);
int binaryWriteGyro(const uint8_t sensorId, gyro_t *data);
int binaryWriteTemperature(const uint8_t sensorId, temperature_t *data);
int binaryWriteLuminosity(const uint8_t sensorId, luminosity_t *data);
int binaryWriteUVLight(const uint8_t sensorId, uvlight_t *data);
int binaryWriteOrientation(const uint8_t sensorId, orientation_t *data);
int binaryWritePressure(const uint8_t sensorId, pressure_t *data);

/**
 * Setup and use the RTC chip, if found
 *
 * The clock must be set before it can be used. This is achieved by compiling
 * and running a basic script (examples/set_rtc). After this, the library
 * automatically uses the RTC if available to timestamp.
 */
bool setRTC();

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ARDUSATLOGGING_H_ */
