#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#define SENSORDATA_HEADER_VERSION	 1

// the data header is encapsulated at the beginning of every "packet" structure
// it provides the information necessary to analyse the content of the structure
// when read in its binary form (useful for sending/receiving data)

struct _data_header_v1 {
	uint8_t version;		// version of the data packet (for compatibility mgt)
	uint8_t length;			// len of the data packet (256 is enough ?)
	uint8_t dimensionality;	// size of the cell array
	uint8_t celltype;		// type of the cell (0 = byte, 1 = float, etc…)
	uint8_t unit;			// unit (standard) of the values (e.g. meter, m/s^2, etc.)
	uint8_t sensor_id;		// id of the sensor that generated this data
	uint32_t timestamp;		// millis for timestamping the data
};

typedef struct _data_header_v1 _data_header_t;

typedef struct {
	_data_header_t header;
	float x,y,z;
} acceleration_t;

typedef struct {
	_data_header_t header;
	float x,y,z;
} magnetic_t;

typedef struct {
	_data_header_t header;
	float roll,pitch,heading;
} orientation_t;

typedef struct {
	_data_header_t header;
	float t;
} temperature_t;

typedef struct {
	_data_header_t header;
	float lux;
} luminosity_t;

typedef struct {
	_data_header_t header;
	float uvindex;
} uvlight_t;


#ifdef USELESS
typedef struct {
	_data_header_t header;
	float x,y,z;
} magnetic_t;

/** FROM ADAFRUIT : struct sensors_color_s is used to return color data in a common format. */
typedef struct {
	_data_header_t header;
	union {
		float c[3];
		/* RGB color space */
		struct {
			float r; /**< Red component */
			float g; /**< Green component */
			float b; /**< Blue component */
		};
	};
	uint32_t rgba; /**< 24-bit RGBA value */
} color_t;
#endif

#endif /* SENSORDATA_H_ */
