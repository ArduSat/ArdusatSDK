/*
 * =====================================================================================
 *
 *       Filename:  space_kit_tester.ino
 *
 *    Description:  Simple driver for all the sensors included in the Ardusat
 *                  Space Kit. Outputs all sensor values at a configurable
 *                  interval in JSON format that can be read by the Ardusat
 *                  Experiment Platform (http://experiments.ardusat.com).
 *
 *                  This example uses many third-party libraries available from
 *                  Adafruit (https://github.com/adafruit). These libraries are
 *                  mostly under an Apache License, Version 2.0.
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *        Version:  1.1
 *        Created:  10/29/2014
 *       Revision:  none
 *       Compiler:  Arduino
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *   Organization:  Ardusat
 *         Edited:  8/25/2015
 *      Edited By:  Sam Olds (sam@ardusat.com)
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
 *  Setup Software Serial to allow for both RF communication and USB communication
 *    RX is digital pin 10 (connect to TX/DOUT of RF Device)
 *    TX is digital pin 11 (connect to RX/DIN of RF Device)
 *-----------------------------------------------------------------------------*/
ArdusatSerial serialConnection(SERIAL_MODE_HARDWARE_AND_SOFTWARE, 8, 9);

/*-----------------------------------------------------------------------------
 *  Constant Definitions
 *-----------------------------------------------------------------------------*/
const int READ_INTERVAL = 250; // interval, in ms, to wait between readings

// Pin Definitions
// 3 LEDs can be turned on/off based on sensor values. These constants define the pins
// they are attached to
const int LED_TMP102 = 3; // Light turns on based on temp sensor
const int LED_MLX90614 = 2; // Light turns on based on infrared temp
const int LED_TSL2561 = 4; // Light turns on based on low light conditions
const int LED_SERIAL = 5;

// These pins are used for measurements
const int LDR_Pin = A2; // analog pin 2 for LDR photo sensor
const int UVOUT = A0; //Output from the UV sensor
const int REF_3V3 = A1; // 3.3V power on the Arduino board

// Barometer settings
const int MY_ALTITUDE_FEET = 4300;
float myAltitude;
float currentSeaLevelPressure = 1026.8;

temperature_t temp;
luminosity_t luminosity;
uvlight_t uv_light;
acceleration_t accel;
magnetic_t mag;
gyro_t gyro;
orientation_t orientation;
pressure_t pressure;

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  setup
 *  Description:  This function runs when the Arduino first turns on/resets. This is
 *                our chance to take care of all one-time configuration tasks to get
 *                the program ready to begin logging data.
 * =====================================================================================
 */
void setup(void)
{
  serialConnection.begin(9600);

  beginAccelerationSensor();
  beginTemperatureSensor();
  beginInfraredTemperatureSensor();
  beginLuminositySensor();
  beginUVLightSensor();
  beginGyroSensor();
  beginMagneticSensor();
  beginBarometricPressureSensor();
  myAltitude = MY_ALTITUDE_FEET * 0.3048;

  // initialize the digital pins as outputs for the LEDs
  pinMode(LED_TMP102, OUTPUT);
  pinMode(LED_MLX90614, OUTPUT);
  pinMode(LED_TSL2561, OUTPUT);
  pinMode(LED_SERIAL, OUTPUT);

  // More pin initializations
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);

  /* We're ready to go! */
  serialConnection.println("Ardusat Space Kit tester");
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  loop
 *  Description:  After setup runs, this loop function runs until the Arduino loses
 *                power or resets. We go through and update each of the attached
 *                sensors, write out the updated values in JSON format, then delay
 *                before repeating the loop again.
 * =====================================================================================
 */
void loop(void)
{
  byte byteRead;
  float temp_val;
  float infrared_temp;

  // To test sending serial data from the computer, we can turn the serialConnection Read
  // LED on or off
  // Entering 1 will turn ON, 0 or any other number turns OFF
  if (serialConnection.available()) {
    byteRead = serialConnection.read();

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
  readAcceleration(accel);
  serialConnection.println(accelerationToJSON("accelerometer", accel));

  // Read Magnetometer
  readMagnetic(mag);
  serialConnection.println(magneticToJSON("magnetic", mag));

  // Read Gyro
  readGyro(gyro);
  serialConnection.println(gyroToJSON("gyro", gyro));

  // Calculate Orientation from Accel + Magnet data
  calculateOrientation(accel, mag, orientation);
  serialConnection.println(orientationToJSON("orientation", orientation));

  // Read Temp from TMP102 (default in celcius)
  readTemperature(temp);
  serialConnection.println(temperatureToJSON("temp", temp));
  temp_val = temp.t;

  // Logic to turn on the temperature LED based on detected temp above ~29.5 C / 85 F
  if (temp_val > 29.5) {
    digitalWrite(LED_TMP102, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_TMP102, LOW);    // turn the LED off by making the voltage LOW
  }

  // Read MLX Infrared temp sensor
  readInfraredTemperature(temp);
  serialConnection.println(temperatureToJSON("infraredTemp", temp));
  infrared_temp = temp.t;

  if (infrared_temp > 29.5) {
    digitalWrite(LED_MLX90614, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_MLX90614, LOW);    // turn the LED off by making the voltage LOW
  }

  // Read TSL2561 Luminosity
  readLuminosity(luminosity);
  serialConnection.println(luminosityToJSON("luminosity", luminosity));

  if (luminosity.lux) {
    if (luminosity.lux < 100) {
      digitalWrite(LED_TSL2561, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else {
      digitalWrite(LED_TSL2561, LOW);    // turn the LED off by making the voltage LOW
    }
  }

  // Read MP8511 UV
  readUVLight(uv_light);
  serialConnection.println(uvlightToJSON("uv", uv_light));

  //  Read Barometer BMP180
  readBarometricPressure(pressure);
  Serial.println(pressureToJSON("pressure", pressure));
  delay(READ_INTERVAL);
}
