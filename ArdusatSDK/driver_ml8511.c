/**
 * =====================================================================================
 *
 *       Filename:  driver_ml8511.c
 *
 *    Description:  Implementation of driver for ML8511 UV Sensor breakout board, which
 *    	 	    uses the MP8511 UV sensor.
 *
 *                  https://www.sparkfun.com/products/12705
 *
 *        Version:  1.0
 *        Created:  12/03/2014 11:47:28
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *
 * =====================================================================================
 */

#include "driver_ml8511.h"

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  average_analog_read
 *  Description:  Takes 8 readings from the given pin then averages the values.
 * =====================================================================================
 */
int average_analog_read(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
  int x;

  for(x = 0; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  map_float
 *  Description:  Does a linear transform on the given value to come up with a scaled
 *                output.
 *                http://forum.arduino.cc/index.php?topic=3922.0
 * =====================================================================================
 */
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Inits the ML8511 breakout board UV sensor
 *
 * @return true
 */
boolean ml8511_init()
{
  return true;
}

/**
 * Reads the ML8511 UV Sensor. To do this we take an average analog voltage 
 * read on the UV sensor pin and the 3.3 V power pin, then use that actual 
 * voltage to get a ratio of the read voltage compared to exactly 3.3V. We 
 * then map this value into a properly scaled value.
 *
 * @return calculated UV value in mW / cm^2
 */
float ml8511_getUV()
{
  int uv_v = average_analog_read(DRIVER_ML8511_UV_PIN);
  int ref_v = average_analog_read(DRIVER_ML8511_REF_PIN);
  float scaled_uv_v = 3.3 / ref_v * uv_v;
  return map_float(scaled_uv_v, 0.99, 2.9, 0.0, 15.0);
}
