Ardusat Sensor SDK
==================

The Ardusat Sensor SDK is a software package designed to make interacting with the sensors found in
the [Ardusat Space Kit](http://www.ardusat.com/products) as easy as possible, while also providing
a powerful unified interface to use the same code to interact with ground-based sensors as well as
satellite systems. It builds on top of popular open source libraries provided by
[Adafruit](https://github.com/adafruit) and others.

# Installing the SDK
Installing the SDK is easy - it works like any other third party Arduino library. Just download 
the SDK at https://s3-us-west-2.amazonaws.com/ardusatweb/ArdusatSDK.zip or clone this repository
to your hard drive, then open the Arduino IDE, go to Sketch -> Import Library -> Add Library and
navigate to your download (zip file or the directory cloned with git). You now should be able
to use the SDK in your sketches.

# Using the SDK
The first step to using the SDK is to import it into your sketch. This can be done with a simple
import statement:
```
#import <ArdusatSDK.h>
```

After the SDK is imported, the basic I/O functions and sensor drivers should be available.

## Initializing Sensors

Sensors in the SDK should be initialized before use. To initialize sensors, a number of `begin`
functions are provided, which should be called in the Arduino `setup` function. These functions are
listed below.

Function | Sensor | Config Arguments
--- | --- | ---
**beginTemperatureSensor** | TMP102 | None
**beginInfraredTemperatureSensor** | MLX90614 | None
**beginLuminositySensor** | TSL2561 | None
**beginAccelerationSensor** | LSM303 (9DOF breakout) | None
**beginOrientationSensor** | L3GD20 (9DOF breakout) | None
**beginUVLightSensor** | SI1145 | None

`begin` functions return `true` on success or `false` on failure.

## Reading Sensor Data

Sensor data is read by calling the appropriate `read` functions and providing a pointer to a data 
structure to read data into. Data structures and read functions are listed below.

Read Function | Data Structure | Data Elements | Sensor
--- | --- | --- | ---
`readTemperature` | `temperature_t` | `t` | TMP102
`readInfraredTemperature` | `temperature_t` | `t` | MLX90614
`setupLuminositySensor` | `luminosity_t` | `lux` | TSL2561
`setupAccelerationSensor` | `acceleration_t` | `x, y, z` | LSM303 (9DOF breakout)
`setupOrientationSensor` | `orientation_t` | `roll, pitch, heading` | L3GD20 (9DOF breakout)
`setupUVLightSensor` | `uvlight_t` | `uvindex` | SI1145

Usage example:
```
#import ArdusatSDK.H

Serial.begin(9600);
temperature_t temp_data;
if (!beginTemperatureSensor()) {
  Serial.println("There was a problem initializing the temperature sensor.");
  while (1);
}
readTemperature(&temp_data);
Serial.println(temp_data.t);
```

## Different Output Formats
The Ardusat SDK can output sensor data in both JSON and CSV format to allow interfacing with
external systems such as the [Demo Ardusat App](http://demo.ardusat.com). To use these functions,
call the `ToJSON` or `ToCSV` family of functions:

Function | Arguments
--- | ---
**valueToJSON** | `const char *sensor_name, uint8_t unit_definition, float value`
**accelerationToJSON** | `const char *sensor_name, acceleration_t *acceleration_data`
**orientationToJSON** | `const char *sensor_name, orientation_t *orientation_data`
**temperatureToJSON** | `const char *sensor_name, temperature_t *temp_data`
**luminosityToJSON** | `const char *sensor_name, luminosity_t *luminosity_data`
**uvlightToJSON** | `const char *sensor_name, uvlight_t *uvlight_data`

Example:
```
  temperature_t temp_data;
  readTemperature(&temp_data);
  Serial.println(temperatureToJSON("temperature", &temp_data));
```

# Getting Help
If you're having trouble running the examples, chances are something is messed up with the external
library locations in your Arduino IDE. Double check that the ArdusatSDK library is imported into
your Arduino libraries (Sketch -> Import Libraries -> Contributed). If the sketches are compiling 
and uploading but not behaving as expected, make sure you double check your wiring, it's always easy
to accidentally plug something in wrong! 

If you get really stuck, feel free to reach out at <support@ardusat.com>, or the "Issues" section of
this repository.
