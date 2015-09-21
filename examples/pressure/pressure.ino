/*
 * =====================================================================================
 *
 *       Filename:  pressure.ino
 *
 *    Description:  Outputs the barometric pressure sensor readings in a JSON format
 *                  that can be read by the Ardusat Experiment Platform
 *                  (http://experiments.ardusat.com).
 *
 *                  Set current altitude in feet and/or current sea level atmospheric
 *                  pressure to get meaningful altitude/pressure calculations.
 *
 *                  This example uses many third-party libraries available from
 *                  Adafruit (https://github.com/adafruit). These libraries are
 *                  mostly under an Apache License, Version 2.0.
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *        Version:  1.0
 *        Created:  02/12/2015
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
const int MY_ALTITUDE_FEET = 4300;
float currentSeaLevelPressure = 1026.8;
float myAltitude;
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

  if (!beginBarometricPressureSensor()) {
    serialConnection.println("Can't initialize barometric pressure sensor! Check your wiring.");
  }

  myAltitude = MY_ALTITUDE_FEET * 0.3048;

  /* We're ready to go! */
  serialConnection.println("");
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
  readBarometricPressure(pressure);

  serialConnection.print("Sea level pressure at altitude ");
  serialConnection.print(myAltitude * 3.28084);
  serialConnection.print("ft: ");
  serialConnection.print(seaLevelPressureForAltitude(myAltitude, pressure.pressure));
  serialConnection.println("hPa");

  serialConnection.println("");

  serialConnection.print("Altitude assuming sea level pressure ");
  serialConnection.print(currentSeaLevelPressure);
  serialConnection.print("hPa: ");
  serialConnection.print(pressureToAltitude(currentSeaLevelPressure, pressure.pressure) * 3.28084);
  serialConnection.println("ft");

  serialConnection.println("");

  serialConnection.println(pressureToJSON("pressure", pressure));
  delay(1000);
}
