/**
 * @file   sd_card.c
 * @Author Ben Peters (ben@ardusat.com)
 * @date   December 21, 2014
 * @brief  Example saving sensor data to a SD card.
 *
 *         This utility reads data from all the sensors in the space kit, then logs it
 *         it to the SD card in either CSV or binary format (see define flags).
 *
 *         Data is stored in the /DATA directory on the FAT32 formated SD card. The filename
 *         of a datalog is /DATA/PREFIX0.CSV for CSV data or /DATA/PREFIX0.BIN for binary
 *         data. See the docs for more information about binary formats. Note that 
 *         prefix will be truncated to 7 character or less if a longer filename is given.
 */
/*-----------------------------------------------------------------------------
 *  Includes
 *-----------------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <ArdusatSDK.h>

/*-----------------------------------------------------------------------------
 *  Constant Definitions
 *-----------------------------------------------------------------------------*/
#define READ_INTERVAL 500 // interval, in ms, to wait between readings

// Pin Definitions
// 3 LEDs can be turned on/off based on sensor values. These constants define the pins
// they are attached to
#define LED_TMP102 3 // Light turns on based on temp sensor
#define LED_MLX90614 2 // Light turns on based on infrared temp
#define LED_TSL2561 4 // Light turns on based on low light conditions
#define LED_SERIAL 5

// These pins are used for measurements
#define LDR_Pin A2 // analog pin 2 for LDR photo sensor
#define UVOUT0 A0 //Output from the UV sensor
#define UVOUT1 A2 //Output from the UV sensor
#define UVOUT2 A3 //Output from the UV sensor
#define REF_3V3 A1 // 3.3V power on the Arduino board

#define SD_CS_PIN 10 //CS pin used for SD card reader. Reader should be wired to DIO
                     //10, 11, 12, 13 
static char LOG_FILE_PREFIX[] = "HAB";
static bool LOG_CSV_DATA = false;

temperature_t temp;
luminosity_t luminosity;
uvlight_t uv_light;
acceleration_t accel;
magnetic_t mag;
gyro_t orientation;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  setup
 *  Description:  This function runs when the Arduino first turns on/resets. This is 
 *                our chance to take care of all one-time configuration tasks to get
 *                the program ready to begin logging data.
 * =====================================================================================
 */
void setup() {
  //Serial.begin(9600);
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);

  if (!beginDataLog(SD_CS_PIN, LOG_FILE_PREFIX, LOG_CSV_DATA)) {
    //Serial.println("Failed to initialize SD card...");
    while (true);
  }

  beginAccelerationSensor();
  beginTemperatureSensor();
  beginInfraredTemperatureSensor();
  beginLuminositySensor();
  beginUVLightSensor();
  beginGyroSensor();
  beginMagneticSensor();

  // More pin initializations
  pinMode(UVOUT0, INPUT);
  pinMode(UVOUT1, INPUT);
  pinMode(UVOUT2, INPUT);
  pinMode(REF_3V3, INPUT);
  pinMode(13, INPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);

  /* We're ready to go! */
  //Serial.println("");
  digitalWrite(9, HIGH);
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  loop
 *  Description:  After setup runs, this loop function runs until the Arduino loses 
 *                power or resets. We go through and update each of the attached sensors,
 *                write out the updated values in JSON format, then delay before repeating
 *                the loop again.
 * =====================================================================================
 */
void loop() {
  // Read Accelerometer
  readAcceleration(&accel);
  //Serial.println(accelerationToJSON("accelerometer", &accel));

  // Read Magnetometer
  readMagnetic(&mag);
  //Serial.println(magneticToJSON("magnetic", &mag));

  // Read Gyro
  readGyro(&orientation);
  //Serial.println(gyroToJSON("gyro", &orientation));

  // Read Temp from TMP102 (default in celcius)
  readTemperature(&temp);
  //Serial.println(temperatureToJSON("temp", &temp));

  // Read MLX Infrared temp sensor
  readInfraredTemperature(&temp);
  //Serial.println(temperatureToJSON("infraredTemp", &temp));

  // Read TSL2561 Luminosity
  readLuminosity(&luminosity);
  //Serial.println(luminosityToJSON("luminosity", &luminosity));

  // Read MP8511 UV 
  readUVLight(&uv_light, UVOUT0);
  binaryWriteUVLight(5, &uv_light);
  //Serial.println(uvlightToJSON("uv0", &uv_light));
  readUVLight(&uv_light, UVOUT1);
  binaryWriteUVLight(6, &uv_light);
  //Serial.println(uvlightToJSON("uv1", &uv_light));
  readUVLight(&uv_light, UVOUT2);
  binaryWriteUVLight(7, &uv_light);
  //Serial.println(uvlightToJSON("uv2", &uv_light));

  binaryWriteAcceleration(0, &accel);
  binaryWriteMagnetic(1, &mag);
  binaryWriteGyro(2, &orientation);
  binaryWriteTemperature(3, &temp);
  binaryWriteLuminosity(4, &luminosity);

  //Serial.print("Wrote ");
  //Serial.print(bytesWritten);
  //Serial.println(" bytes.");

  //Serial.print("Free Memory: ");
  //Serial.println(freeMemory());

  delay(READ_INTERVAL);
}
