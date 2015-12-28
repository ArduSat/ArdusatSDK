# Ardusat SDK

The Ardusat Sensor SDK is a software package designed to make interacting with the sensors found in
the [Ardusat Space Kit](http://www.ardusat.com/products) as easy as possible, while also providing
a powerful unified interface to use the same code to interact with ground-based sensors as well as
satellite systems. It builds on top of popular open source libraries provided by
[Adafruit](https://github.com/adafruit) and others.

## Installing the SDK
The first step to getting going is to install the Arduino Integrated Development Environment (IDE).
Downloads for all major OS versions can be found at
[http://arduino.cc/en/Main/Software](http://arduino.cc/en/Main/Software).

Once you have the IDE, installing the SDK is easy - it works like any other third party Arduino
library. Just [download the SDK](https://www.ardusat.com/downloads/ArdusatSDK.zip) or
clone this repository to your hard drive, then open the Arduino IDE, go to 'Sketch -> Import Library
-> Add Library' and navigate to your download (zip file or the directory cloned with Git). You now
should be able to use the SDK in your sketches.

If you're interested in using the SDK for logging data to an SD card, you'll need to do this same
thing with our other
[Ardusat Logging SDK Library](http://github.com/ardusat/ardusatsdk-logging)

## Using the SDK
The first step to using the SDK is to import it into your sketch. This can be done with a simple
import statement:

```
#include <ArdusatSDK.h>
```

After the SDK is imported, the basic I/O functions and sensor drivers should be available.

### Have one of our new Spaceboard Sensors?
If you're using one of our new Spaceboards instead of the break out Space Kit, just add
`ARDUSAT_SPACEBOARD = true;` inside the `setup()` function. It should look something like
this:
```
void setup(void)
{
  ARDUSAT_SPACEBOARD = true;

  Serial.begin(9600);
  if (!beginTemperatureSensor()) {
    Serial.println("There was a problem initializing the temperature sensor.");
    while (1);
  }
}
```

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
**beginMagneticSensor** | LSM303 (9DOF breakout) | None
**beginOrientationSensor** | L3GD20 (9DOF breakout) | None
**beginUVLightSensor** | ML8511 (Default) or SI1132 | Optional `SENSORID_ML8511` or `SENSORID_SI1132`
**beginBarometricPressureSensor** | BMP180 | None
**beginRGBLightSensor** | TCS34725 (Default) or ISL29125 | Optional `SENSORID_TCS34725` or `SENSORID_ISL29125`

`begin` functions return `true` on success or `false` on failure.

## Reading Sensor Data

Sensor data is read by calling the appropriate `read` functions and providing a pointer to a data 
structure to read data into. Data structures and read functions are listed below.

Read Function | Data Structure | Data Elements | Sensor
--- | --- | --- | ---
`readTemperature` | `temperature_t` | `t` | TMP102
`readInfraredTemperature` | `temperature_t` | `t` | MLX90614
`readLuminosity` | `luminosity_t` | `lux` | TSL2561
`readAcceleration` | `acceleration_t` | `x, y, z` | LSM303 (10DOF breakout)
`readMagnetic` | `magnetic_t` | `x, y, z` | LSM303 (10DOF breakout)
`readGyro` | `orientation_t` | `x, y, z` | L3GD20 (10DOF breakout)
`readUVLight` | `uvlight_t` | `uvindex` | ML8511
`readBarometricPressure` | `pressure_t` | `pressure` | BMP180 (10DOF breakout)
`readRGBLight` | `rgblight_t` | `red, green, blue` | TCS34725

In addition to these `read` functions, a convenience function `calculateOrientation` is provided
to calculate the 3-axis orientation from raw data from the accelerometer and magnetometer. This 
function calculates `roll` (rotation about `x` axis), `pitch` (rotation about `y` axis), and 
`heading` (rotation about `z` axis), and has the following signature:

```
void calculateOrientation(const acceleration_t, const magnetic_t, orientation_t);
```

Usage example:
```
#import <ArdusatSDK.h>

temperature_t temp_data;

void setup(void)
{
  Serial.begin(9600);
  if (!beginTemperatureSensor()) {
    Serial.println("There was a problem initializing the temperature sensor.");
    while (1);
  }
}

void loop(void)
{
  readTemperature(temp_data);
  Serial.println(temp_data.t);
}
```

The Barometric Pressure sensor has two additional convenience functions to calculate altitude (which
requires knowing the current sea level barometric pressure, a value that's easily available from
weather observation data), or current sea level barometric pressure (which requires knowing the
current altitude). For these convenience functions, altitude should be provided in meters, and
pressure values should be provided in hPa.

```
float pressureToAltitude(float knownSeaLevelPressure, float measuredAtmosphericPressure);
float seaLevelPressureForAltitude(float knownAltitude, float measuredAtmosphericPressure);
```

To translate meters to feet, multiply the meter value by `3.28084`. To translate feet to meters,
multiply the feet value by `0.3084`.

## Different Output Formats
The Ardusat SDK can output sensor data in both JSON and CSV format to allow interfacing with
external systems such as the [Ardusat Experiment Platform](http://experiments.ardusat.com). To use these functions,
call the `ToJSON` or `ToCSV` family of functions:

### Checksum
Both JSON and CSV output formats optionally include checksum values to verify that the data remains
uncorrupted through transmission. This is an integer value that is calculated when the data packet
is first written in a known way. The client receiving the data packet can then re-calculate the
checksum using the same algorithm, then verify that the calculated value matches the transmitted
checksum. The algorithm used to calculate the checksum sums the characters in the `sensorName` field
as integers, then rounds the value (s) and adds this to the sum of the name characters.

Example checksum calculation:

```
int calculate_checksum(const char *sensorName, float value) {
  int checksum = 0;
  int i, len;

  len = strlen(sensorName);
  for (i = 0; i < len; ++i) {
    checksum += sensorName[i];
  }
  checksum += lroundf(value);
  return checksum;
}

```

### JSON Format
Function | Arguments
--- | ---
**valueToJSON** | `const char *sensor_name, unsigned char unit_definition, float value`
**accelerationToJSON** | `const char *sensor_name, acceleration_t acceleration_data`
**magneticToJSON** | `const char *sensor_name, magnetic_t magnetic_data`
**orientationToJSON** | `const char *sensor_name, orientation_t orientation_data`
**temperatureToJSON** | `const char *sensor_name, temperature_t temp_data`
**luminosityToJSON** | `const char *sensor_name, luminosity_t luminosity_data`
**uvlightToJSON** | `const char *sensor_name, uvlight_t uvlight_data`
**rgblightToJSON** | `const char *sensor_name, rgblight_t rgblight_data`

Example:
```
  temperature_t temp_data;
  readTemperature(temp_data);
  Serial.println(temperatureToJSON("temperature", temp_data));
  >> ~{"sensorName": "temperature", "unit": "C", "value": 23.5, "cs": 43}|
```

### CSV Format
The CSV format includes a simple timestamp, in milliseconds since the Arduino began running,
followed by a sensor name and a list of sensor values. The last value is an integer checksum.

Function | Arguments
--- | ---
**accelerationToCSV** | `const char *sensor_name, acceleration_t acceleration_data`
**magneticToCSV** | `const char *sensor_name, magnetic_t magnetic_data`
**orientationToCSV** | `const char *sensor_name, orientation_t orientation_data`
**temperatureToCSV** | `const char *sensor_name, temperature_t temp_data`
**luminosityToCSV** | `const char *sensor_name, luminosity_t luminosity_data`
**uvlightToCSV** | `const char *sensor_name, uvlight_t uvlight_data`
**rgblightToCSV** | `const char *sensor_name, rgblight_t rgblight_data`

Example:
```
  temperature_t temp_data;
  readTemperature(temp_data);
  Serial.println(temperatureToCSV("temperature", temp_data));
  >> 123,temperature,23.5,43
```

### Serial Library
The regular Arduino `Serial` library works fine with the SDK. However, sometimes when connecting to
XBee or other wireless modules, it is useful to be able to wire serial connections that leave the
main Arduino USB Serial free for programming. To do this, the SDK leverages the `SoftwareSerial`
library in a new class called `ArdusatSerial`. This class has the same interface as both `Serial`
and `SoftwareSerial`, but can be configured to output on hardware serial (`Serial`), software
serial, or both.

If a software serial mode is selected, the `ArdusatSerial` constructor must be supplied with
arguments for `softwareReceivePin` and `softwareTransmitPin`:

```
ArdusatSerial(serialMode mode, unsigned char softwareReceivePin, unsigned char softwareTransmitPin);
```

Usage:
```
ArdusatSerial serialConnection(SERIAL_MODE_HARDWARE_AND_SOFTWARE, 10, 11);

void setup()
{
  serialConnection.begin(9600);
  serialConnection.println("This message will go out on hardware serial and software serial!");
  beginTemperatureSensor();
}

void loop()
{
  temperature_t temp_data;
  readTemperature(temp_data);
  serialConnection.println(temperatureToCSV("temperature", temp_data));
}

```

#### Serial Modes
Name | Description
--- | ---
**SERIAL_MODE_HARDWARE** | Output serial data on built-in USB hardware serial
**SERIAL_MODE_SOFTWARE** | Output serial data on software serial (must specify transmit and receive pins in the constructor, see arguments in example above)
**SERIAL_MODE_HARDWARE_AND_SOFTWARE** | Output to both hardware and software serial interfaces

#### Limitations
SoftwareSerial does not appear to work reliably above 57600 baud. 

# Getting Help
If you're having trouble running the examples, chances are something is messed up with the external
library locations in your Arduino IDE. Double check that the ArdusatSDK library is imported into
your Arduino libraries (Sketch -> Import Libraries -> Contributed). If the sketches are compiling 
and uploading but not behaving as expected, make sure you double check your wiring, it's always easy
to accidentally plug something in wrong! 

If you get really stuck, feel free to reach out at <support@ardusat.com>, or the "Issues" section of
this repository.
