/**
 * @file   BinaryDataFmt.h
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 19, 2014
 * @brief  Definitions for binary data format types. These definitions are used
 *         to encode/decode binary data.
 */

#ifndef BINARY_DATA_FMT_H_
#define BINARY_DATA_FMT_H_

/**
 * Binary data structures are designed to be as space-efficient as possible, 
 * given the sensor's native resolution.
 */
#define _bin_data_header uint8_t length; uint8_t id; uint32_t timestamp;

typedef struct {
	_bin_data_header
	uint16_t x;
	uint16_t y;
	uint16_t z;
} acceleration_bin_t;

typedef struct {
	_bin_data_header
	uint16_t x;
	uint16_t y;
	uint16_t z;
} magnetic_bin_t;

typedef struct {
	_bin_data_header
	uint16_t x;
	uint16_t y;
	uint16_t z;
} orientation_bin_t;

typedef struct {
	_bin_data_header
	uint16_t temp;
} temperature_bin_t;

typedef struct {
	_bin_data_header
	uint16_t luminosity;
} luminosity_bin_t;

typedef struct {
	_bin_data_header
	uint16_t uv;
} uv_light_bin_t;

typedef struct {
	float min;
	float max;
} bin_range_t;

//To translate from floating point representations into
//2 byte integer representations we need to choose an effective
//measurement range. When decoding binary data after logging, these
//min/max numbers are required to get floating point values.
#define BIN_ACCEL_MIN -160.0F
#define BIN_ACCEL_MAX 160.0F

#define BIN_MAG_MIN -2000.0F
#define BIN_MAG_MAX 2000.0F

#define BIN_ORIENTATION_MIN -60.0F
#define BIN_ORIENTATION_MAX 60.0F

#define BIN_TEMPERATURE_MIN -150.0F
#define BIN_TEMPERATURE_MAX 1000.0F

#define BIN_LUMINOSITY_MIN 0.0F
#define BIN_LUMINOSITY_MAX 2000.0F

#define BIN_UV_MIN 0.0F
#define BIN_UV_MAX 15.0F

#endif
