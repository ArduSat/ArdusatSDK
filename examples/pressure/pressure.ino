/**
 * @file   pressure.ino
 * @Author Ben Peters (ben@ardusat.com)
 * @date   February 12, 2015
 * @brief  Example of reading in barometric pressure from the Adafruit 10DOF IMU chip.
 *
 *         Set current altitude in feet and/or current sea level atmospheric pressure
 *         to get meaningful altitude/pressure calculations.
 */
#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

#define MY_ALTITUDE_FEET 4300
float myAltitude;
float currentSeaLevelPressure = 1026.8;

void setup(void)
{
  Serial.begin(9600);

  if (!beginBarometricPressureSensor()) {
    Serial.println("can't init barometric pressure");
  }

  myAltitude = MY_ALTITUDE_FEET * 0.3048;
}

void loop(void)
{
  pressure_t pressure;
  uint16_t rawTemp;
  uint32_t rawPressure;
  float temp;

  readBarometricPressure(&pressure);
  bmp180_getRawTemperature(&rawTemp);
  bmp180_getTemperature(&temp);
  bmp180_getRawPressure(&rawPressure);

  Serial.println("Raw Values");
  Serial.print("  - Temp: ");
  Serial.println(rawTemp);
  Serial.print("  - Pressure: ");
  Serial.println(rawPressure);

  Serial.println("");

  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println("C");

  Serial.print("");

  Serial.print("Sea level pressure at altitude ");
  Serial.print(myAltitude * 3.28084);
  Serial.print("ft: ");
  Serial.print(seaLevelPressureForAltitude(myAltitude, pressure.pressure));
  Serial.println("hPa");

  Serial.println("");

  Serial.print("Altitude assuming sea level pressure ");
  Serial.print(currentSeaLevelPressure);
  Serial.print("hPa: ");
  Serial.print(pressureToAltitude(currentSeaLevelPressure, pressure.pressure) * 3.28084);
  Serial.println("ft");

  Serial.println("");

  Serial.println(pressureToJSON("pressure", &pressure));
  delay(1000);
}
