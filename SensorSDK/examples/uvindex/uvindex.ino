#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!setupUVLightSensor()) {
	  Serial.println("can't init UV sensor");
  }
}

void loop(void)
{
	uvlight_t uv;

	readUVLight(&uv);

	Serial.println(uvlightToJSON(&uv));

	delay(1000);
}
