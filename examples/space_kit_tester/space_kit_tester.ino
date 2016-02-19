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

Temperature ambient;
Temperature infrared = Temperature(SENSORID_MLX90614);
Luminosity lum;
UVLight uv;
Acceleration accel;
Magnetic mag;
Gyro gyro;
Orientation orient;
Pressure pressure;

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
  //ARDUSAT_SPACEBOARD = true;
  serialConnection.begin(9600);

  ambient.begin();
  infrared.begin();
  lum.begin();
  uv.begin();
  accel.begin();
  mag.begin();
  gyro.begin();
  orient.begin(accel, mag);
  pressure.begin();

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
  serialConnection.println(accel.readToJSON("accelerometer"));

  // Read Magnetometer
  serialConnection.println(mag.readToJSON("magnetic"));

  // Read Gyro
  serialConnection.println(gyro.readToJSON("gyro"));

  // Calculate Orientation from Accel + Magnet data
  serialConnection.println(orient.readToJSON(accel, mag, "orientation"));

  // Read Temp from TMP102 (default in celcius)
  serialConnection.println(ambient.readToJSON("ambientTemp"));

  // Logic to turn on the temperature LED based on detected temp above ~29.5 C / 85 F
  if (ambient.t > 29.5) {
    digitalWrite(LED_TMP102, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_TMP102, LOW);    // turn the LED off by making the voltage LOW
  }

  // Read MLX Infrared temp sensor
  serialConnection.println(infrared.readToJSON("infraredTemp"));

  if (infrared.t > 29.5) {
    digitalWrite(LED_MLX90614, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_MLX90614, LOW);    // turn the LED off by making the voltage LOW
  }

  // Read TSL2561 Luminosity
  serialConnection.println(lum.readToJSON("luminosity"));

  if (lum.lux) {
    if (lum.lux < 100) {
      digitalWrite(LED_TSL2561, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else {
      digitalWrite(LED_TSL2561, LOW);    // turn the LED off by making the voltage LOW
    }
  }

  // Read MP8511 UV
  serialConnection.println(uv.readToJSON("uv"));

  //  Read Barometer BMP180
  serialConnection.println(pressure.readToJSON("pressure"));

  delay(READ_INTERVAL);
}
