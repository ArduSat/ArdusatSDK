/*
 * =====================================================================================
 *
 *       Filename:  units.c
 *
 *    Description:  Unit utilities.
 *
 *        Version:  1.0
 *        Created:  11/24/2014 09:16:56
 *
 *         Author:  Ben Peters (ben@ardusat.com)
 *
 * =====================================================================================
 */
#include <Arduino.h>

#include "units.h"

/**
 * Convert an enumerated unit code to a string representation.
 *
 * @param unit code (see units.h defines)
 * 
 * @return string representation of unit
 */
const char * unit_to_str(uint8_t unit) 
{
  switch (unit) {
    case (DATA_UNIT_NONE):
      return "";
    case (DATA_UNIT_METER_PER_SECONDSQUARED):
      return "m/s^2";
    case (DATA_UNIT_RADIAN_PER_SECOND):
      return "rad/s";
    case (DATA_UNIT_MICROTESLA):
      return "uT";
    case (DATA_UNIT_DEGREES_CELSIUS):
      return "C";
    case (DATA_UNIT_DEGREES_FAHRENHEIT):
      return "F";
    case (DATA_UNIT_METER_PER_SECOND):
      return "m/s";
    case (DATA_UNIT_LUX):
      return "lux";
    case (DATA_UNIT_MILLIWATT_PER_CMSQUARED):
      return "mW/cm^2";
    case (DATA_UNIT_RADIAN):
      return "rad";
    default:
      return "";
  };
}
