#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

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

  Serial.println(orientationToJSON("orientation", &orient));

  delay(1000);
}
