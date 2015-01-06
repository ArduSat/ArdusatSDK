#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginTemperatureSensor()) {
    Serial.println("can't init temperature sensor");
  }
}

void loop(void)
{
  temperature_t temp;

  readTemperature(&temp);

  Serial.print("temp: \t");
  Serial.println(temperatureToJSON("ambient_temp", &temp));

  delay(1000);
}
