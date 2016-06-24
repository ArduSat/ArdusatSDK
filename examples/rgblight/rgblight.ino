/*
 * =====================================================================================
 *
 *       Filename:  rgblight.ino
 *
 *    Description:  Outputs the RGB light sensor readings in a JSON format that
 *                  can be read by the Ardusat Experiment Platform
 *                  (http://experiments.ardusat.com).
 *
 *                  This example uses many third-party libraries available from
 *                  Adafruit (https://github.com/adafruit). These libraries are
 *                  mostly under an Apache License, Version 2.0.
 *
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *
 *        Version:  1.0
 *        Created:  11/30/2015
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
/* Default Sensor Configurations - To use different configuration, place a
                                   "//" at the beginning of the next line and
                                   remove the "//" at the beginning of the
                                   configuration you want to use */
RGBLight rgb; // => TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X

/* TCS34725 - Useful outside or in very bright room */
//RGBLightTCS rgb(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/* TCS34725 - Useful at night or in dark room */
//RGBLightTCS rgb(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_60X);

/* ISL29125 - Useful in bright area */
//RGBLightISL rgb(CFG1_10KLUX);

/* ISL29125 - Useful in dark area */
//RGBLightISL rgb(CFG1_375LUX);


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

  rgb.begin();

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
  serialConnection.println(rgb.readToJSON("rgb"));

  delay(1000);
}
