#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!setupInfraredTemperatureSensor()) {
	  Serial.println("can't init IR temperature sensor");
  }
}

void loop(void)
{
	temperature_t temp;

	readInfraredTemperature(&temp);

	Serial.print("temp: \t");
	Serial.println(temperatureToText(&temp));

	delay(1000);
}
