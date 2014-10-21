/***************************************************************************
 This is a library for the Adafruit 9DOF Breakout

 Designed specifically to work with the Adafruit 9DOF Breakout:
 http://www.adafruit.com/products/1714

 These displays use I2C to communicate, 2 pins are required to interface.

 Adafruit invests time and resources providing this open source code,
 please support Adafruit andopen-source hardware by purchasing products
 from Adafruit!

 Written by Kevin Townsend for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>
#include <math.h>

#include "Adafruit_9DOF.h"
#include "obcl.h"

#define PI  (3.14159265F);

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates a new Adafruit_9DOF class
 */
/**************************************************************************/
Adafruit_9DOF::Adafruit_9DOF(void) {
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Setups the HW
 */
/**************************************************************************/
bool Adafruit_9DOF::begin() {
	// Enable I2C
	obcl_begin();
//  Wire.begin();

	return true;
}

/**************************************************************************/
/*!
 @brief  Populates the .pitch/.roll fields in the sensors_vec_t struct
 with the right angular data (in degree)

 @param  event         The sensors_event_t variable containing the
 data from the accelerometer
 @param  orientation   The sensors_vec_t object that will have it's
 .pitch and .roll fields populated
 @return Returns true if the operation was successful, false if there
 was an error

 @code

 bool error;
 sensors_event_t event;
 sensors_vec_t orientation;
 ...
 lsm303accelGetSensorEvent(&event);
 error = accelGetOrientation(&event, &orientation);

 @endcode
 */
/**************************************************************************/
bool Adafruit_9DOF::accelGetOrientation(sensors_event_t *event,
		sensors_vec_t *orientation) {
	/* Make sure the input is valid, not null, etc. */
	if (event == NULL)
		return false;
	if (orientation == NULL)
		return false;

	float t_pitch;
	float t_roll;
	float signOfZ = event->acceleration.z >= 0 ? 1.0F : -1.0F;

	/* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
	/* roll is positive and increasing when moving downward                                     */
	/*                                                                                          */
	/*                                 y                                                        */
	/*             roll = atan(-----------------)                                               */
	/*                          sqrt(x^2 + z^2)                                                 */
	/* where:  x, y, z are returned value from accelerometer sensor                             */

	t_roll = event->acceleration.x * event->acceleration.x
			+ event->acceleration.z * event->acceleration.z;
	orientation->roll = (float) atan2(event->acceleration.y, sqrt(t_roll)) * 180
			/ PI
	;

	/* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
	/* pitch is positive and increasing when moving upwards                                     */
	/*                                                                                          */
	/*                                 x                                                        */
	/*             roll = atan(-----------------)                                               */
	/*                          sqrt(y^2 + z^2)                                                 */
	/* where:  x, y, z are returned value from accelerometer sensor                             */

	t_pitch = event->acceleration.y * event->acceleration.y
			+ event->acceleration.z * event->acceleration.z;
	orientation->pitch = (float) atan2(event->acceleration.x,
			signOfZ * sqrt(t_pitch)) * 180 / PI
	;

	return true;
}

/**************************************************************************/
/*!
 @brief  Utilize the sensor data from an accelerometer to compensate
 the magnetic sensor measurements when the sensor is tilted
 (the pitch and roll angles are not equal 0�)

 @param  axis          The given axis (SENSOR_AXIS_X/Y/Z) that is
 parallel to the gravity of the Earth

 @param  mag_event     The raw magnetometer data to adjust for tilt

 @param  accel_event   The accelerometer event data to use to determine
 the tilt when compensating the mag_event values

 @code

 // Perform tilt compensation with matching accelerometer data
 sensors_event_t accel_event;
 error = lsm303accelGetSensorEvent(&accel_event);
 if (!error)
 {
 magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
 }

 @endcode
 */
/**************************************************************************/
bool Adafruit_9DOF::magTiltCompensation(sensors_axis_t axis,
		sensors_event_t *mag_event, sensors_event_t *accel_event) {
	/* Make sure the input is valid, not null, etc. */
	if (mag_event == NULL)
		return false;
	if (accel_event == NULL)
		return false;

	float accel_X, accel_Y, accel_Z;
	float *mag_X, *mag_Y, *mag_Z;

	switch (axis) {
	case SENSOR_AXIS_X:
		/* The X-axis is parallel to the gravity */
		accel_X = accel_event->acceleration.y;
		accel_Y = accel_event->acceleration.z;
		accel_Z = accel_event->acceleration.x;
		mag_X = &(mag_event->magnetic.y);
		mag_Y = &(mag_event->magnetic.z);
		mag_Z = &(mag_event->magnetic.x);
		break;

	case SENSOR_AXIS_Y:
		/* The Y-axis is parallel to the gravity */
		accel_X = accel_event->acceleration.z;
		accel_Y = accel_event->acceleration.x;
		accel_Z = accel_event->acceleration.y;
		mag_X = &(mag_event->magnetic.z);
		mag_Y = &(mag_event->magnetic.x);
		mag_Z = &(mag_event->magnetic.y);
		break;

	case SENSOR_AXIS_Z:
		/* The Z-axis is parallel to the gravity */
		accel_X = accel_event->acceleration.x;
		accel_Y = accel_event->acceleration.y;
		accel_Z = accel_event->acceleration.z;
		mag_X = &(mag_event->magnetic.x);
		mag_Y = &(mag_event->magnetic.y);
		mag_Z = &(mag_event->magnetic.z);
		break;

	default:
		return false;
	}

	float t_roll = accel_X * accel_X + accel_Z * accel_Z;
	float rollRadians = (float) atan2(accel_Y, sqrt(t_roll));

	float t_pitch = accel_Y * accel_Y + accel_Z * accel_Z;
	float pitchRadians = (float) atan2(accel_X, sqrt(t_pitch));

	float cosRoll = (float) cos(rollRadians);
	float sinRoll = (float) sin(rollRadians);
	float cosPitch = (float) cos(pitchRadians);
	float sinPitch = (float) sin(pitchRadians);

	/* The tilt compensation algorithm                            */
	/* Xh = X.cosPitch + Z.sinPitch                               */
	/* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
	*mag_X = (*mag_X) * cosPitch + (*mag_Z) * sinPitch;
	*mag_Y = (*mag_X) * sinRoll * sinPitch + (*mag_Y) * cosRoll
			- (*mag_Z) * sinRoll * cosPitch;

	return true;
}

/**************************************************************************/
/*!
 @brief  Populates the .heading fields in the sensors_vec_t
 struct with the right angular data (0-359�)

 Heading increases when measuring clockwise

 @param  axis          The given axis (SENSOR_AXIS_X/Y/Z)

 @param  event         The raw magnetometer sensor data to use when
 calculating out heading

 @param  orientation   The sensors_vec_t object where we will
 assign an 'orientation.heading' value

 @code

 magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);

 @endcode
 */
/**************************************************************************/
bool Adafruit_9DOF::magGetOrientation(sensors_axis_t axis,
		sensors_event_t *event, sensors_vec_t *orientation) {
	/* Make sure the input is valid, not null, etc. */
	if (event == NULL)
		return false;
	if (orientation == NULL)
		return false;

	switch (axis) {
	case SENSOR_AXIS_X:
		/* Sensor rotates around X-axis                                                                 */
		/* "heading" is the angle between the 'Y axis' and magnetic north on the horizontal plane (Oyz) */
		/* heading = atan(Mz / My)                                                                      */
		orientation->heading = (float) atan2(event->magnetic.z,
				event->magnetic.y) * 180 / PI
		;
		break;

	case SENSOR_AXIS_Y:
		/* Sensor rotates around Y-axis                                                                 */
		/* "heading" is the angle between the 'Z axis' and magnetic north on the horizontal plane (Ozx) */
		/* heading = atan(Mx / Mz)                                                                      */
		orientation->heading = (float) atan2(event->magnetic.x,
				event->magnetic.z) * 180 / PI
		;
		break;

	case SENSOR_AXIS_Z:
		/* Sensor rotates around Z-axis                                                                 */
		/* "heading" is the angle between the 'X axis' and magnetic north on the horizontal plane (Oxy) */
		/* heading = atan(My / Mx)                                                                      */
		orientation->heading = (float) atan2(event->magnetic.y,
				event->magnetic.x) * 180 / PI
		;
		break;

	default:
		return false;
	}

	/* Normalize to 0-359� */
	if (orientation->heading < 0) {
		orientation->heading = 360 + orientation->heading;
	}

	return true;
}

/**************************************************************************/
/*!
 @brief  Populates the .roll/.pitch/.heading fields in the sensors_vec_t
 struct with the right angular data (in degree).

 The starting position is set by placing the object flat and
 pointing northwards (Z-axis pointing upward and X-axis pointing
 northwards).

 The orientation of the object can be modeled as resulting from
 3 consecutive rotations in turn: heading (Z-axis), pitch (Y-axis),
 and roll (X-axis) applied to the starting position.


 @param  accel_event   The sensors_event_t variable containing the
 data from the accelerometer

 @param  mag_event     The sensors_event_t variable containing the
 data from the magnetometer

 @param  orientation   The sensors_vec_t object that will have it's
 .roll, .pitch and .heading fields populated
 */
/**************************************************************************/
bool Adafruit_9DOF::fusionGetOrientation(sensors_event_t *accel_event,
		sensors_event_t *mag_event, sensors_vec_t *orientation) {
	/* Make sure the input is valid, not null, etc. */
	if (accel_event == NULL)
		return false;
	if (mag_event == NULL)
		return false;
	if (orientation == NULL)
		return false;

	float const PI_F = 3.14159265F;

	/* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
	/* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
	/*                                                                                                */
	/*                    y                                                                           */
	/*      roll = atan2(---)                                                                         */
	/*                    z                                                                           */
	/*                                                                                                */
	/* where:  y, z are returned value from accelerometer sensor                                      */
	orientation->roll = (float) atan2(accel_event->acceleration.y,
			accel_event->acceleration.z);

	/* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
	/* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
	/*                                                                                                */
	/*                                 -x                                                             */
	/*      pitch = atan(-------------------------------)                                             */
	/*                    y * sin(roll) + z * cos(roll)                                               */
	/*                                                                                                */
	/* where:  x, y, z are returned value from accelerometer sensor                                   */
	if (accel_event->acceleration.y * sin(orientation->roll)
			+ accel_event->acceleration.z * cos(orientation->roll) == 0)
		orientation->pitch =
				accel_event->acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
	else
		orientation->pitch = (float) atan(
				-accel_event->acceleration.x
						/ (accel_event->acceleration.y * sin(orientation->roll)
								+ accel_event->acceleration.z
										* cos(orientation->roll)));

	/* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
	/* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
	/*                                                                                                */
	/*                                       z * sin(roll) - y * cos(roll)                            */
	/*   heading = atan2(--------------------------------------------------------------------------)  */
	/*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
	/*                                                                                                */
	/* where:  x, y, z are returned value from magnetometer sensor                                    */
	orientation->heading = (float) atan2(
			mag_event->magnetic.z * sin(orientation->roll)
					- mag_event->magnetic.y * cos(orientation->roll),
			mag_event->magnetic.x * cos(orientation->pitch)
					+ mag_event->magnetic.y * sin(orientation->pitch)
							* sin(orientation->roll)
					+ mag_event->magnetic.z * sin(orientation->pitch)
							* cos(orientation->roll));

	/* Convert angular data to degree */
	orientation->roll = orientation->roll * 180 / PI_F;
	orientation->pitch = orientation->pitch * 180 / PI_F;
	orientation->heading = orientation->heading * 180 / PI_F;

	return true;
}
