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

  readTemperature("ambient_temp", &temp);

  Serial.print("temp: \t");
  Serial.println(temperatureToJSON(&temp));

  delay(1000);
}
