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
#define READ_INTERVAL 10000 // interval, in ms, to wait between readings

// Pin Definitions
// 3 LEDs can be turned on/off based on sensor values. These constants define the pins
// they are attached to
#define LED_TMP102 3 // Light turns on based on temp sensor
#define LED_MLX90614 2 // Light turns on based on infrared temp
#define LED_TSL2561 4 // Light turns on based on low light conditions
#define LED_SERIAL 5

// These pins are used for measurements
#define LDR_Pin A2 // analog pin 2 for LDR photo sensor
#define UVOUT A0 //Output from the UV sensor
#define REF_3V3 A1 // 3.3V power on the Arduino board

#define SD_CS_PIN 10 //CS pin used for SD card reader. Reader should be wired to DIO
                     //10, 11, 12, 13 
static char LOG_FILE_PREFIX[] = "MYLOG";
static bool LOG_CSV_DATA = true;

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  setup
 *  Description:  This function runs when the Arduino first turns on/resets. This is 
 *                our chance to take care of all one-time configuration tasks to get
 *                the program ready to begin logging data.
 * =====================================================================================
 */
void setup() {
  Serial.begin(9600);

  if (!beginDataLog(SD_CS_PIN, LOG_FILE_PREFIX, LOG_CSV_DATA)) {
    Serial.println("Failed to initialize SD card...");
    while (true);
  }

  beginAccelerationSensor();
  beginTemperatureSensor();
  beginInfraredTemperatureSensor();
  beginLuminositySensor();
  beginUVLightSensor();
  beginGyroSensor();
  beginMagneticSensor();
  
  // initialize the digital pins as outputs for the LEDs
  pinMode(LED_TMP102, OUTPUT);
  pinMode(LED_MLX90614, OUTPUT);       
  pinMode(LED_TSL2561, OUTPUT);
  pinMode(LED_SERIAL, OUTPUT); 
  
  // More pin initializations
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  
  /* We're ready to go! */
  Serial.println("");
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
  temperature_t temp;
  luminosity_t luminosity;
  uvlight_t uv_light;
  acceleration_t accel;
  magnetic_t mag;
  gyro_t orientation;
  byte byteRead;
  float temp_val;
  float infrared_temp;
  int bytesWritten;

  // To test sending serial data from the computer, we can turn the Serial Read
  // LED on or off
  // Entering 1 will turn ON, 0 or any other number turns OFF
  if (Serial.available()) {
    byteRead = Serial.read();

    // Echo the value read back on the serial port
    Serial.write(byteRead);
    if (byteRead == 49) // Equals 1 / ON
    {
      digitalWrite(LED_SERIAL, HIGH);   // turn the LED on (HIGH is the voltage level)
    }
    else
    {
      digitalWrite(LED_SERIAL, LOW);    // turn the LED off by making the voltage LOW
    }  
  }

  // Read Accelerometer
  readAcceleration(&accel);
  Serial.println(accelerationToJSON("accelerometer", &accel));

  // Read Magnetometer
  readMagnetic(&mag);
  Serial.println(magneticToJSON("magnetic", &mag));
  
  // Read Gyro
  readGyro(&orientation);
  Serial.println(gyroToJSON("gyro", &orientation));

  // Read Temp from TMP102 (default in celcius)
  readTemperature(&temp);
  Serial.println(temperatureToJSON("temp", &temp));
  temp_val = temp.t;

  // Logic to turn on the temperature LED based on detected temp above ~29.5 C / 85 F
  if (temp_val > 29.5) {
    digitalWrite(LED_TMP102, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_TMP102, LOW);    // turn the LED off by making the voltage LOW
  }
  
  // Read MLX Infrared temp sensor
  readInfraredTemperature(&temp);
  Serial.println(temperatureToJSON("infraredTemp", &temp));
  infrared_temp = temp.t;

  if (infrared_temp > 29.5) {
    digitalWrite(LED_MLX90614, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_MLX90614, LOW);    // turn the LED off by making the voltage LOW
  }
        
  // Read TSL2561 Luminosity
  readLuminosity(&luminosity);
  Serial.println(luminosityToJSON("luminosity", &luminosity));

  if (luminosity.lux) {
    if (luminosity.lux < 100) {
      digitalWrite(LED_TSL2561, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else {
      digitalWrite(LED_TSL2561, LOW);    // turn the LED off by making the voltage LOW
    }
  } 
  
  // Read MP8511 UV 
  readUVLight(&uv_light);
  Serial.println(uvlightToJSON("uv", &uv_light));

if (LOG_CSV_DATA) {
  bytesWritten = writeAcceleration("accelerometer", &accel);
  bytesWritten += writeMagnetic("magnetic", &mag);
  bytesWritten += writeGyro("gyro", &orientation);
  bytesWritten += writeTemperature("temp", &temp);
  bytesWritten += writeLuminosity("luminosity", &luminosity);
  bytesWritten += writeUVLight("uv", &uv_light);
} else {
  bytesWritten = binaryWriteAcceleration(0, &accel);
  bytesWritten += binaryWriteMagnetic(1, &mag);
  bytesWritten += binaryWriteGyro(2, &orientation);
  bytesWritten += binaryWriteTemperature(3, &temp);
  bytesWritten += binaryWriteLuminosity(4, &luminosity);
  bytesWritten += binaryWriteUVLight(5, &uv_light);
}

  Serial.print("Wrote ");
  Serial.print(bytesWritten);
  Serial.println(" bytes.");

  Serial.print("Free Memory: ");
  Serial.println(freeMemory());

  delay(READ_INTERVAL);
}
