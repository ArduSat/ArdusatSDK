#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginGyroSensor()) {
    Serial.println("can't init IMU");
  }
}

void loop(void)
{
  gyro_t orient;

  readGyro(&orient);

  Serial.println(gyroToJSON("orientation", &orient));

  delay(1000);
}
