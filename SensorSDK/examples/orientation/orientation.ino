#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!setupOrientationSensor()) {
	  Serial.println("can't init IMU");
  }
}

void loop(void)
{
	orientation_t orient;

	readOrientation(&orient);

	Serial.println(orientationToJSON(&orient));

	delay(1000);
}
