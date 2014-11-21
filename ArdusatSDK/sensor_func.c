#include <Arduino.h>
#include "sensor_func.h"
#include "sensor_ids.h"
#include "units.h"
#include "cc_wire.h"
#include "config.h"
#include "config_incdrivers.h"


// ***************************
// *** TEMPERATURE (SOLID) ***
// ***************************

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
	output->header.unit = DATA_UNIT_DEGREES_CELSIUS;
	output->header.timestamp = millis();

#if defined(LOAD_DRIVER_TMP102)
	output->header.sensor_id = SENSORID_TMP102;
	output->t = tmp102_getTempCelsius();
#else
	output->header.sensor_id = SENSORID_NULL;
	output->t = 0.0;
#endif
}




// ************************
// *** TEMPERATURE (IR) ***
// ************************

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
	output->header.unit = DATA_UNIT_DEGREES_CELSIUS;
	output->header.timestamp = millis();

#if defined(LOAD_DRIVER_MLX90614)
	output->header.sensor_id = SENSORID_MLX90614;
	output->t = mlx90614_getTempCelsius();
#else
	output->header.sensor_id = SENSORID_NULL;
	output->t = 0.0;
#endif
}


// ******************
// *** LUMINOSITY ***
// ******************

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


// *******************
// *** ULTRAVIOLET ***
// *******************

boolean setupUVLightSensor() {
#ifdef SENSOR_UVINDEX
#if defined(LOAD_DRIVER_SI1145)
	return(si1145_init());
#endif
#else	// SENSOR_UVINDEX
	return(false);
#endif
}

void readUVLight(uvlight_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(temperature_t);
	output->header.dimensionality = 1;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_MILLIWATT_PER_CMSQUARED;
	output->header.timestamp = millis();

#ifdef SENSOR_UVINDEX
#if defined(LOAD_DRIVER_TSL2561)
	output->header.sensor_id = SENSORID_SI1145;
	output->uvindex = si1145_getUVIndex();
#endif
#else	// SENSOR_UVINDEX
	output->header.sensor_id = SENSORID_NULL;
	output->uvindex = 0.0;
#endif
}


// ********************
// *** ACCELERATION ***
// ********************

boolean setupAccelerationSensor() {
#if defined(LOAD_DRIVER_ADAFRUIT9DOF)
	return(adafruit9dof_init());
#else
	return(false);
#endif
}

void readAcceleration(acceleration_t * output) {
	if (output == NULL)
		return;

	output->header.version = SENSORDATA_HEADER_VERSION;
	output->header.length = sizeof(acceleration_t);
	output->header.dimensionality = 3;
	output->header.celltype = DATA_CELLTYPE_FLOAT;
	output->header.unit = DATA_UNIT_METER_PER_SECONDSQUARED;
	output->header.timestamp = millis();

	// TODO : real sensor
#if defined(LOAD_DRIVER_ADAFRUIT9DOF)
	output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
	adafruit9dof_getACCEL(&(output->x),&(output->y),&(output->z));
#else
	output->header.sensor_id = SENSORID_NULL;
#endif
}


// *******************
// *** ORIENTATION ***
// *******************

boolean setupOrientationSensor() {
#if defined(LOAD_DRIVER_ADAFRUIT9DOF)
	return(adafruit9dof_init());
#else
	return(false);
#endif
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

	// TODO : real sensor
#if defined(LOAD_DRIVER_ADAFRUIT9DOF)
	output->header.sensor_id = SENSORID_ADAFRUIT9DOFIMU;
	adafruit9dof_getRPH(&(output->roll),&(output->pitch),&(output->heading));
	//output->t = mlx90614_getTempCelsius();
#else
	output->header.sensor_id = SENSORID_NULL;
#endif
}
