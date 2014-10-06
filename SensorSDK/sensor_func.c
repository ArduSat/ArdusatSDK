#include <Arduino.h>
#include "sensor_func.h"
#include "sensor_ids.h"
#include "units.h"
#include "driver_tsl2561.h"
#include "config.h"

boolean setupAccelerationSensor() {
	return(false);
}

void readAcceleration(acceleration_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(acceleration_t);
	output->header.dimensionality = 3;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_METERPERSECONDSQUARE;
	output->header.timestamp = millis();
	output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU; // TODO : LOCAL_*_SENSOR_ID;

	// TODO : real sensor
	output->x = 9.7;
	output->y = 0.2;
	output->z = 0.1;
}

boolean setupTemperatureSensor() {
#if defined(LOAD_DRIVER_TMP102)
	return(tmp102_init());
#else
	return(false);
#endif
}

void readTemperature(temperature_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(temperature_t);
	output->header.dimensionality = 1;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_DEGREESCELSIUS;
	output->header.timestamp = millis();

#if defined(LOAD_DRIVER_TMP102)
	output->header.sensor_id = SENSORID_TMP102;
	output->t = tmp102_getTempCelsius();
#else
	output->header.sensor_id = SENSORID_NULL;
	output->t = 0.0;
#endif
}

boolean setupOrientationSensor() {
	return(false);
}

void readOrientation(orientation_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(orientation_t);
	output->header.dimensionality = 3;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_RADIAN;
	output->header.timestamp = millis();
	output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU; // TODO : LOCAL_*_SENSOR_ID;

	// TODO : real sensor
	output->yaw = 0.1;
	output->pitch = 0.2;
	output->roll = 0.3;
}

boolean setupInfraredTemperatureSensor() {
#if defined(LOAD_DRIVER_MLX90614)
	return(mlx90614_init());
#else
	return(false);
#endif
}

void readInfraredTemperature(temperature_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(temperature_t);
	output->header.dimensionality = 1;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_DEGREESCELSIUS;
	output->header.timestamp = millis();

#if defined(LOAD_DRIVER_MLX90614)
	output->header.sensor_id = SENSORID_MLX90614;
	output->t = mlx90614_getTempCelsius();
#else
	output->header.sensor_id = SENSORID_NULL;
	output->t = 0.0;
#endif
}


boolean setupLuminositySensor() {
#ifdef SENSOR_LUMINOSITY
#if defined(LOAD_DRIVER_TSL2561)
	return(tsl2561_init());
#elif defined(LOAD_DRIVER_TSL2591)
	return(tsl2591_init());
#endif
#else
	return(false);
#endif
}

void readLuminosity(luminosity_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(temperature_t);
	output->header.dimensionality = 1;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_LUX;
	output->header.timestamp = millis();

#ifdef SENSOR_LUMINOSITY
#if defined(LOAD_DRIVER_TSL2561)
	output->header.sensor_id = SENSORID_TSL2561;
	output->lux = tsl2561_getLux();
#elif defined(LOAD_DRIVER_TSL2591)
	output->header.sensor_id = SENSORID_TSL2591;
	output->lux = tsl2591_getLux();
#endif
#else
	output->header.sensor_id = SENSORID_NULL;
	output->lux = 0.0;
#endif
}

