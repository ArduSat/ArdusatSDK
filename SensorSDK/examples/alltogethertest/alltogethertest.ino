#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>
#include <util_freeMemory.h> // within SensorSDK library

void setup(void)
{
  Serial.begin(9600);

  obcl_begin();	// starts on board communication layer
  obcl_scan();	// scans the available I2C sensors

  if (!setupOrientationSensor()) {
    Serial.println("can't init 9dof");
  }

  if (!setupTemperatureSensor()) {
    Serial.println("can't init temperature sensor");
  }

  if (!setupLuminositySensor()) {
    Serial.println("can't init lum sensor");
  }

  if (!setupInfraredTemperatureSensor()) {
    Serial.println("can't init IR sensor");
  }

  if (!setupUVLightSensor()) {
    Serial.println("can't init UV sensor");
  }
}

void loop(void)
{
  temperature_t temp;
  readTemperature(&temp);
  Serial.println(temperatureToJSON(&temp));

  temperature_t irtemp;
  readInfraredTemperature(&irtemp);
  Serial.println(temperatureToJSON(&irtemp));

  luminosity_t lum;
  readLuminosity(&lum);
  Serial.println(luminosityToJSON(&lum));

  uvlight_t uv;
  readUVLight(&uv);
  Serial.println(uvlightToJSON(&uv));

  acceleration_t accel;
  readAcceleration(&accel);
  Serial.println(accelerationToJSON(&accel));

  orientation_t orient;
  readOrientation(&orient);
  Serial.println(orientationToJSON(&orient));

  Serial.println(anythingFloatToJSON("memory", "bytes", util_freeMemory(), 0));

  delay(1000);
}
