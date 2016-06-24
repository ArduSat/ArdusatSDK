/*
 * =====================================================================================
 *
 *       Filename:  all_sensors.ino
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
 *        Version:  1.2
 *        Created:  10/29/2014
 *       Revision:  none
 *       Compiler:  Arduino
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *   Organization:  Ardusat
 *         Edited:  3/25/2016
 *      Edited By:  Kevin Cocco (kevin@ardusat.com)
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
 *    RX is digital pin 8 (connect to TX/DOUT of RF Device)
 *    TX is digital pin 9 (connect to RX/DIN of RF Device)
 *-----------------------------------------------------------------------------*/
ArdusatSerial serialConnection(SERIAL_MODE_HARDWARE_AND_SOFTWARE, 8, 9);

/*-----------------------------------------------------------------------------
 *  Constant Definitions
 *-----------------------------------------------------------------------------*/
const int READ_INTERVAL = 0; // interval, in ms, to wait between readings

Acceleration accel;
TemperatureTMP ambient;
Gyro gyro;
TemperatureMLX infrared;
Luminosity lum;
Magnetic mag;
Pressure pressure;
RGBLightTCS rgb;
RGBLightISL rgb_ISL29125;
UVLightML uv;
UVLightSI uv_SI1132;

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

  /* We're ready to go! */
  serialConnection.println("");

  accel.begin();
  ambient.begin();
  gyro.begin();
  infrared.begin();
  lum.begin();
  mag.begin();
  pressure.begin();
  rgb.begin();
  rgb_ISL29125.begin();
  uv.begin();
  uv_SI1132.begin();
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
  // Read Accelerometer
  serialConnection.println(accel.readToJSON("accelerometer"));

  // Read Temp from TMP102 (default in celcius)
  serialConnection.println(ambient.readToJSON("ambientTemp"));

  // Read Gyro
  serialConnection.println(gyro.readToJSON("gyro"));

  // Read MLX Infrared temp sensor
  serialConnection.println(infrared.readToJSON("infraredTemp"));

  // Read TSL2561 Luminosity
  serialConnection.println(lum.readToJSON("luminosity"));

  // Read Magnetometer
  serialConnection.println(mag.readToJSON("magnetic"));

  // Read Barometric Pressure
  serialConnection.println(pressure.readToJSON("pressure"));

  // Read TCS34725 RGB (Default)
  serialConnection.println(rgb.readToJSON("rgb"));

  // Read ISL29125 RGB
  serialConnection.println(rgb_ISL29125.readToJSON("rgb_ISL29125"));

  // Read ML8511 UV (Default)
  serialConnection.println(uv.readToJSON("uv"));

  // Read SI1132 UV
  serialConnection.println(uv_SI1132.readToJSON("uv_SI1132"));

  delay(READ_INTERVAL);
}
