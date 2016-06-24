/*
 * =====================================================================================
 *
 *       Filename:  csv.ino
 *
 *    Description:  Outputs the values from two different sensors into text properly
 *                  formatted for a CSV file.
 *
 *                  This example uses many third-party libraries available from
 *                  Adafruit (https://github.com/adafruit). These libraries are
 *                  mostly under an Apache License, Version 2.0.
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *        Version:  1.0
 *        Created:  02/16/2016
 *       Revision:  none
 *       Compiler:  Arduino
 *
 *         Author:  Sam Olds (sam@ardusat.com)
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
 *  Setup Software Serial to allow for both RF communication and USB communication
 *    RX is digital pin 8 (connect to TX/DOUT of RF Device)
 *    TX is digital pin 9 (connect to RX/DIN of RF Device)
 *-----------------------------------------------------------------------------*/
ArdusatSerial serialConnection(SERIAL_MODE_HARDWARE_AND_SOFTWARE, 8, 9);

/*-----------------------------------------------------------------------------
 *  Constant Definitions
 *-----------------------------------------------------------------------------*/
Acceleration accel;
Temperature temp;

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

  accel.begin();
  temp.begin();

  /* We're ready to go! */
  serialConnection.println("");
  serialConnection.print("timestamp (millis), name, ");
  serialConnection.print("temperature (C), x acceleration (m/s^2), y acceleration (m/s^2), z acceleration (m/s^2), ");
  serialConnection.println("checksum");
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
  accel.read();
  temp.read();
  
  serialConnection.println(valuesToCSV("reading", accel.header.timestamp, 4, temp.t, accel.x, accel.y, accel.z));

  delay(1000);
}
