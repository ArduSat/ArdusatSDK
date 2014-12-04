#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginInfraredTemperatureSensor()) {
    Serial.println("can't init IR temperature sensor");
  }
}

void loop(void)
{
  temperature_t temp;

  readInfraredTemperature(&temp);

  Serial.print("temp: \t");
  Serial.println(temperatureToJSON("infared", &temp));

  delay(1000);
}
