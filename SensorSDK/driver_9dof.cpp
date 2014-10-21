#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include "config.h"
#include "driver_9dof.h"

/* Assign a unique ID to the sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;


boolean adafruit9dof_init() {
	if (!accel.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
		while (1)
			;
	}
	if (!mag.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		while (1)
			;
	}
	return(true);
}

void adafruit9dof_getRPH(float * roll, float * pitch, float * heading) {
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_vec_t orientation;

	/* Calculate pitch and roll from the raw accelerometer data */
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation)) {
		/* 'orientation' should have valid .roll and .pitch fields */
		*roll = orientation.roll;
		*pitch = orientation.pitch;
	}

	/* Calculate the heading using the magnetometer */
	mag.getEvent(&mag_event);
	if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
		*heading = orientation.heading;
	}
}


