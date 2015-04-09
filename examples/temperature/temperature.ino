#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

static char LOG_FILE_PREFIX[] = "TEST";
static bool LOG_CSV_DATA = false;

#define SD_CS_PIN 10 //CS pin used for SD card reader. Reader should be wired to DIO
                     //10, 11, 12, 13 

temperature_t temp;
void setup(void)
{
  Serial.begin(9600);
  while(!Serial);

  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);

  if (!beginDataLog(SD_CS_PIN, LOG_FILE_PREFIX, LOG_CSV_DATA)) {
    Serial.println("Failed to initialize SD card...");
  }

  if (!beginTemperatureSensor()) {
    Serial.println("can't init temperature sensor");
  }
}

void loop(void)
{

  readTemperature(&temp);
  binaryWriteTemperature(0, &temp);

  Serial.print("temp: \t");
  Serial.println(temp.t);

  delay(1000);
}
