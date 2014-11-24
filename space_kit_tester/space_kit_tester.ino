/*
 * =====================================================================================
 *
 *       Filename:  space_kit_tester.ino
 *
 *    Description:  Simple driver for all the sensors included in the Ardusat
 *                  Space Kit. Outputs all sensor values at a configurable 
 *                  interval in JSON format that can be read by the Ardusat 
 *                  demo app (https://demo.ardusat.com).
 *
 *                  This example uses many third-party libraries available from
 *                  Adafruit (https://github.com/adafruit). These libraries are
 *                  mostly under an Apache License, Version 2.0.
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *        Version:  1.0
 *        Created:  10/29/2014 02:50:13
 *       Revision:  none
 *       Compiler:  Arduino
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *   Organization:  Ardusat
 *
 * =====================================================================================
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
#define READ_INTERVAL 250 // interval, in ms, to wait between readings

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
  Serial.println("Ardusat Space Kit tester"); 

  setupAccelerationSensor();
  setupTemperatureSensor();
  setupInfraredTemperatureSensor();
  setupLuminositySensor();
  setupUVLightSensor();
  setupOrientationSensor();
  setupMagneticSensor();
  
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
  orientation_t orientation;
  byte byteRead;
  float temp_val;
  float infrared_temp;

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
  //Serial.println(accelerationToJSON("accelerometer", &accel));
  Serial.println(valueToJSON("accelerometerX", accel.header.unit, accel.x));
  Serial.println(valueToJSON("accelerometerY", accel.header.unit, accel.y));
  Serial.println(valueToJSON("accelerometerZ", accel.header.unit, accel.z));

  // Read Magnetometer
  readMagnetic(&mag);
  Serial.println(valueToJSON("magneticX", mag.header.unit, mag.x));
  Serial.println(valueToJSON("magneticY", mag.header.unit, mag.y));
  Serial.println(valueToJSON("magneticZ", mag.header.unit, mag.z));
  
  // Read Gyro
  readOrientation(&orientation);
  //Serial.println(orientationToJSON("gyro", &orientation));
  Serial.println(valueToJSON("gyroRoll", orientation.header.unit, orientation.roll));
  Serial.println(valueToJSON("gyroPitch", orientation.header.unit, orientation.pitch));
  Serial.println(valueToJSON("gyroHeading", orientation.header.unit, orientation.heading));

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

  delay(READ_INTERVAL);
}
