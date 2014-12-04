#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginUVLightSensor()) {
	  Serial.println("can't init UV sensor");
  }
}

void loop(void)
{
	uvlight_t uv;

	readUVLight(&uv);

	Serial.println(uvlightToJSON("uv", &uv));

	delay(1000);
}
