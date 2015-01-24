/**
 * @file   set_rtc.ino
 * @Author Ben Peters (ben@ardusat.com)
 * @date   January 23, 2015
 * @brief  Sets the RTC to the datetime that this script was compiled.
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

void setup()
{
  Serial.begin(9600);
  setRTC();

  Serial.println("\nRTC set!");
}

void loop()
{
}
