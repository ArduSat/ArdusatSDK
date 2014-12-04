#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup(void)
{
  Serial.begin(9600);

  if (!beginLuminositySensor()) {
    Serial.println("can't init luminosity sensor");
  }
}

void loop(void)
{
  luminosity_t lum;

  readLuminosity(&lum);

  Serial.print("lum: \t");
  Serial.println(luminosityToJSON("luminosity", &lum));

  delay(1000);
}
