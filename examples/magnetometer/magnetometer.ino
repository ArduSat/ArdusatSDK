/**
 * @file magnetometer.ino
 * @author Ben Peters (ben@ardusat.com)
 * @date 05-29-2015
 * @brief Basic example of using the LSM303DHLC magnetometer with the Ardusat SDK
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

magnetic_t mag;

void setup(void)
{
  Serial.begin(9600);

  if (!beginMagneticSensor()) {
    Serial.println("can't init IMU");
  }
}

void loop(void)
{
  readMagnetic(&mag);

  Serial.println(magneticToJSON("magnetic", &mag));
  delay(2000);
}
