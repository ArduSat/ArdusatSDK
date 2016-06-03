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
 *    RX is digital pin 8 (connect to TX/DOUT of RF Device)
 *    TX is digital pin 9 (connect to RX/DIN of RF Device)
 *-----------------------------------------------------------------------------*/
ArdusatSerial serialConnection(SERIAL_MODE_HARDWARE_AND_SOFTWARE, 8, 9);

/*-----------------------------------------------------------------------------
 *  Constant Definitions
 *-----------------------------------------------------------------------------*/
float MY_ALTITUDE_FEET = 4300.0;    // My altitude in feet
float seaLevelPressure = 1026.8;    // The assumed pressure (in hPa) at sea level
float altitude;                     // My altitude in meters
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
  serialConnection.begin(9600);

  pressure.begin();

  altitude = convFeetToMeters(MY_ALTITUDE_FEET);

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
  pressure.read();
  float calculatedSeaLevelPressure = pressure.seaLevelPressureFromAltitude(altitude);
  float calculatedAltitude = pressure.altitudeFromSeaLevelPressure(seaLevelPressure);

  serialConnection.print("Sea level pressure, calculated from current pressure (");
  serialConnection.print(pressure.pressure);
  serialConnection.print(" hPa) and altitude (");
  serialConnection.print(convMetersToFeet(altitude));
  serialConnection.print(" ft)  = ");
  serialConnection.print(calculatedSeaLevelPressure);
  serialConnection.println(" hPa");

  serialConnection.print("Altitude, calculated from current pressure (");
  serialConnection.print(pressure.pressure);
  serialConnection.print(" hPa) and sea level pressure (");
  serialConnection.print(seaLevelPressure);
  serialConnection.print(" hPa) = ");
  serialConnection.print(convMetersToFeet(calculatedAltitude));
  serialConnection.println(" ft");

  serialConnection.println("");

  serialConnection.println(pressure.toJSON("pressure"));
  delay(1000);
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  convFeetToMeters
 *   Parameters:  feet
 *  Description:  This function converts the value of the parameter (in feet) to meters
 * =====================================================================================
 */
float convFeetToMeters(float feet) {
  return feet * 0.3048;
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  convMetersToFeet
 *   Parameters:  meters
 *  Description:  This function converts the value of the parameter (in meters) to feet
 * =====================================================================================
 */
float convMetersToFeet(float meters) {
  return meters * 3.28084;
}
