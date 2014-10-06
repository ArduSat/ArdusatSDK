#include <Arduino.h>
#include <Wire.h>
#include <SensorSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!setupLuminositySensor()) {
	  Serial.println("can't init luminosity sensor");
  }
}

void loop(void)
{
	luminosity_t lum;

	readLuminosity(&lum);

	Serial.print("lum: \t");
	Serial.println(luminosityToText(&lum));

	delay(1000);
}

