/**
 * @file   ArdusatLogging.cpp
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 3, 2014
 * @brief  Implements ArdusatSDK generic sensor read & configuration functions
 */

#include <stdio.h>
#include <string.h>

#include "ArdusatLogging.h"

RTC_DS1307 RTC;

SdVolume vol;
SdFat sd;
File file;
prog_char sd_card_error[] = "Not enough RAM for SD card sys(free: ";
prog_char csv_header_fmt[] = "timestamp: %lu at millis %lu\n";

#define write_if_init(gen_fn) return writeString(gen_fn);

/**
 * Helper function to write null-terminated output buffer string to file.
 *
 * @param output_buf pointer to output buffer to write.
 *
 * @return number of bytes written
 */
int writeString(const char *output_buf)
{
  int buf_len = strlen(output_buf);
  return writeBytes((const uint8_t *) output_buf, buf_len);
}

/**
 * Writes the byte array pointed to by buffer to the SD card. Note that there is
 * absolutely no safety checking on numBytes, so use with care.
 *
 * Since we use a shared buffer between the SD card library and many of the output
 * functions, performs a check first to see if we need to allocate a new buffer.
 *
 * @param buffer byte array to write
 * @param number of bytes to write
 *
 * @return number of bytes written
 */
int writeBytes(const uint8_t *buffer, uint8_t numBytes)
{
  uint8_t *buf;
  int written;
  bool buf_allocated = false;

  if (numBytes > OUTPUT_BUFFER_MAXSIZE) {
    numBytes = OUTPUT_BUFFER_MAXSIZE;
  }

  if (file.isOpen()) {
    if (buffer == (uint8_t *) _getOutBuf()) {
      buf = (uint8_t *) malloc(numBytes);
      buf_allocated = true;
      memcpy(buf, buffer, numBytes);
    } else {
      buf = (uint8_t *) buffer;
    }

    written = file.write(buf, numBytes);
    file.sync();
    if (buf_allocated)
      free(buf);
    return written;
  } else {
    return 0;
  }
}

/**
 * Writes a line of CSV formatted acceleration data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data acceleration_t data to write
 *
 * @return number of bytes written
 */
int writeAcceleration(const char *sensorName, acceleration_t *data)
{
  write_if_init(accelerationToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted magnetic data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data magnetic_t data to write
 *
 * @return number of bytes written
 */
int writeMagnetic(const char *sensorName, magnetic_t *data)
{
  write_if_init(magneticToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted orientation data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data gyro_t data to write
 *
 * @return number of bytes written
 */
int writeGyro(const char *sensorName, gyro_t *data)
{
  write_if_init(gyroToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted tempoerature data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data temperature_t data to write
 *
 * @return number of bytes written
 */
int writeTemperature(const char *sensorName, temperature_t *data)
{
  write_if_init(temperatureToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted luminosity data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data luminosity_t data to write
 *
 * @return number of bytes written
 */
int writeLuminosity(const char *sensorName, luminosity_t *data)
{
  write_if_init(luminosityToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted UV light data to the SD card.
 *
 * @param sensorName of this sensor
 * @param data uvlight_t data to write
 *
 * @return number of bytes written
 */
int writeUVLight(const char *sensorName, uvlight_t *data)
{
  write_if_init(uvlightToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted orientation data to SD card.
 *
 * @param sensorName of this sensor
 * @param data orientation_t data to write
 *
 * @return number of bytes written
 */
int writeOrientation(const char *sensorName, orientation_t *data)
{
  write_if_init(orientationToCSV(sensorName, data))
}

/**
 * Writes a line of CSV formatted pressure data to SD card.
 *
 * @param sensorName of this sensor
 * @param data pressure_t data to write
 *
 * @return number of bytes written
 */
int writePressure(const char *sensorName, pressure_t *data)
{
  write_if_init(pressureToCSV(sensorName, data))
}

#define init_data_struct(type_def, type_enum) \
  type_def bin_data; \
  bin_data.type = type_enum; \
  bin_data.id = sensorId; \
  bin_data.timestamp = data->header.timestamp;

#define _write_binary_data_struct(type) return writeBytes((uint8_t *) &bin_data, sizeof(type));

int binaryWriteAcceleration(const uint8_t sensorId, acceleration_t *data)
{
  init_data_struct(acceleration_bin_t, ARDUSAT_SENSOR_TYPE_ACCELERATION)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(acceleration_bin_t)
}

int binaryWriteMagnetic(const uint8_t sensorId, magnetic_t *data)
{
  init_data_struct(magnetic_bin_t, ARDUSAT_SENSOR_TYPE_MAGNETIC)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(magnetic_bin_t)
}

int binaryWriteGyro(const uint8_t sensorId, gyro_t *data)
{
  init_data_struct(gyro_bin_t, ARDUSAT_SENSOR_TYPE_GYRO)
  bin_data.x = data->x;
  bin_data.y = data->y;
  bin_data.z = data->z;

  _write_binary_data_struct(gyro_bin_t)
}

int binaryWriteTemperature(const uint8_t sensorId, temperature_t *data)
{
  init_data_struct(temperature_bin_t, ARDUSAT_SENSOR_TYPE_TEMPERATURE)
  bin_data.temp = data->t;

  _write_binary_data_struct(temperature_bin_t)
}

int binaryWriteLuminosity(const uint8_t sensorId, luminosity_t *data)
{
  init_data_struct(luminosity_bin_t, ARDUSAT_SENSOR_TYPE_LUMINOSITY)
  bin_data.luminosity = data->lux;

  _write_binary_data_struct(luminosity_bin_t)
}

int binaryWriteUVLight(const uint8_t sensorId, uvlight_t *data)
{
  init_data_struct(uv_light_bin_t, ARDUSAT_SENSOR_TYPE_UV)
  bin_data.uv = data->uvindex;

  _write_binary_data_struct(uv_light_bin_t)
}

int binaryWriteOrientation(const uint8_t sensorId, orientation_t *data)
{
  init_data_struct(orientation_bin_t, ARDUSAT_SENSOR_TYPE_ORIENTATION)
  bin_data.roll = data->roll;
  bin_data.pitch = data->pitch;
  bin_data.heading = data->heading;

  _write_binary_data_struct(orientation_bin_t)
}

int binaryWritePressure(const uint8_t sensorId, pressure_t *data)
{
  init_data_struct(pressure_bin_t, ARDUSAT_SENSOR_TYPE_PRESSURE)
  bin_data.pressure = data->pressure;

  _write_binary_data_struct(pressure_bin_t)
}


/*
 * Helper function to write to the top of the CSV header with the current time
 */
int _write_csv_time_header(DateTime *now, unsigned long curr_millis)
{
  char fmt_buf[32];

  strcpy_P(fmt_buf, csv_header_fmt);

  memset(_getOutBuf(), 0, OUTPUT_BUF_SIZE);

  sprintf(_getOutBuf(), fmt_buf, now->unixtime(), curr_millis);
  write_if_init(_getOutBuf());
}

int _write_binary_time_header(DateTime *now, unsigned long curr_millis)
{
  uint8_t buf[10];
  uint32_t unixtime = now->unixtime();

  buf[0] = 0xFF;
  buf[1] = 0xFF;
  memcpy(buf + 2, &unixtime, 4);
  memcpy(buf + 6, &curr_millis, 4);
  return writeBytes(buf, 10);
}

/**
 * Function starts the SD card service and makes sure that the appropriate directory
 * structure exists, then creates the file to use for logging data. Data will be logged
 * to a file called fileNamePrefix_0.csv or fileNamePrefix_0.bin (number will be incremented
 * for each new log file)
 *
 * @param chipSelectPin Arduino pin SD card reader CS pin is attached to
 * @param fileNamePrefix string to prefix at beginning of data file
 * @param csvData boolean flag whether we are writing csv data or binary data (used for filename)
 *
 * @return true if successful, false if failed
 */
bool beginDataLog(int chipSelectPin, const char *fileNamePrefix, bool csvData)
{
  bool ret;
  int i = 0;
  int max_len;
  char fileName[19];
  char prefix[8];
  char rootPath[] = "/data";
  unsigned long curr_millis = 0;
  DateTime now;

  if (_output_buffer != NULL) {
    delete []_output_buffer;
  }
  OUTPUT_BUF_SIZE = 512;
  OUTPUT_BUFFER_MAXSIZE = 500;
  _output_buffer = vol.cacheAddress()->output_buf;

  // Try to get the current time from the RTC, if available. This will be prepended to the log file
  // to be used to convert relative timestamps to real time values.
  Wire.begin();
  RTC.begin();
  if (RTC.isrunning()) {
    now = RTC.now();
    curr_millis = millis();
  }

  if (freeMemory() < 400) {
    strcpy_P(_getOutBuf(), sd_card_error);
    Serial.print(_getOutBuf());
    Serial.print(freeMemory());
    Serial.println(", need 400)");
    ret = false;
  } else {
    ret = sd.begin(chipSelectPin, SPI_FULL_SPEED);
  }

  //Filenames need to fit the 8.3 filename convention, so truncate down the
  //given filename if it is too long.
  memcpy(prefix, fileNamePrefix, 7);
  prefix[7] = '\0';

  if (ret) {
    if (!sd.exists(rootPath))
      ret = sd.mkdir(rootPath);
    if (ret) {
      while (true) {
	if (i < 10) {
	  max_len = 7;
	} else if (i < 100) {
	  max_len = 6;
	} else if (i < 1000) {
	  max_len = 5;
	} else {
	  break;
	}

	prefix[max_len - 1] = '\0';
	sprintf(fileName, "%s/%s%d.%s", rootPath, prefix, i,
		csvData ? "csv" : "bin");
	if (!sd.exists(fileName)) {
	  file = sd.open(fileName, FILE_WRITE);
	  break;
	}
	i++;
      }
    }
  } else {
    sd.initErrorPrint();
  }

  if (RTC.isrunning() && file.isOpen()) {
    if (csvData) {
      _write_csv_time_header(&now, curr_millis);
    } else {
      _write_binary_time_header(&now, curr_millis);
    }
  }

  return file.isOpen();
}

bool setRTC()
{
  Wire.begin();
  RTC.begin();
  // Sets RTC to the date & time sketch was compiled
  RTC.adjust(DateTime(__DATE__, __TIME__));
  return true;
}
