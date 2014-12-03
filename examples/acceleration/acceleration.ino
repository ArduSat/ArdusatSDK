#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginAccelerationSensor()) {
    Serial.println("can't init IMU");
  }
}

void loop(void)
{
  acceleration_t accel;

  readAcceleration(&accel);

  Serial.println(accelerationToJSON("acceleration", &accel));

  delay(1000);
}
