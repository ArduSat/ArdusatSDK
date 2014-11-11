#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!setupAccelerationSensor()) {
    Serial.println("can't init IMU");
  }
}

void loop(void)
{
  acceleration_t accel;

  readAcceleration(&accel);

  Serial.println(accelerationToJSON(&accel));

  delay(1000);
}
