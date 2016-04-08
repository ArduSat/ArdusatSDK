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
thing with our other [Ardusat Logging SDK Library](http://github.com/ardusat/ardusatsdk-logging)



## Using the SDK
The first step to using the SDK is to include it into your sketch. This can be done with a simple
include statement:

```
#include <ArdusatSDK.h>
```

After the SDK is included, the basic I/O functions and sensor drivers should be available.



## Sensor Overview and Reference
Every Sensor in the SDK is accessible with a Sensor class that helps wrap up internal functionality.
Each Sensor Class supports reading data from one or more physical sensors.

#### Overview
Sensor Class | Sensors | `SENSORIDS`
--- | --- | ---
Acceleration | LSM303 (9DOF breakout)             | `SENSORID_ADAFRUIT9DOFIMU`
Gyro         | L3GD20 (9DOF breakout)             | `SENSORID_ADAFRUIT9DOFIMU`
Luminosity   | TSL2561                            | `SENSORID_TSL2561`
Magnetic     | LSM303 (9DOF breakout)             | `SENSORID_ADAFRUIT9DOFIMU`
Orientation  | Derived from Acc and Mag           | `SENSORID_ADAFRUIT9DOFIMU`
Pressure     | BMP180                             | `SENSORID_BMP180`
RGBLight     | TCS34725 (Default) and ISL29125    | `SENSORID_TCS34725` or `SENSORID_ISL29125`
Temperature  | TMP102 (Default) and MLX90614 (IR) | `SENSORID_TMP102` or `SENSORID_MLX90614`
UVLight      | ML8511 (Default) and SI1132        | `SENSORID_ML8511` or `SENSORID_SI1132`

To use a non-default sensor, simply provide the `SENSORID_` of the desired sensor to the
corresponding class when instantiating a sensor object. For example:

```
UVLight uv;                            // --> uv is now a sensor object that reads from the ML8511 Sensor
UVLight uv = UVLight(SENSORID_ML8511); // --> uv is now a sensor object that reads from the ML8511 Sensor
UVLight uv = UVLight(SENSORID_SI1132); // --> uv is now a sensor object that reads from the SI1132 Sensor
```

#### Common Functions
Every Sensor in the SDK has the following functions that can be used to initialize, configure, read,
and print data:

Available Functions | Required Parameters | Optional Parameters | Description
--- | --- | --- | ---
`begin()`      | None | (Sensor Class Dependent)  | Initializes sensors with any optional advanced configurations
`read()`       | None | None                      | Takes a reading from the sensor
`readToCSV()`  | None | `const char * sensorName` | Takes a reading and prints it in a CSV format
`readToJSON()` | None | `const char * sensorName` | Takes a reading and prints it in a JSON format
`toCSV()`      | None | `const char * sensorName` | Prints the last reading taken in a CSV format
`toJSON()`     | None | `const char * sensorName` | Prints the last reading taken in a JSON format

* CSV  format: `timestamp,sensorName,values...,checksum`
* JSON format: `~{"sensorName": "NAME", "unit": "UNIT", "value": 123.4, "cs": 123}|`

NOTE: The `toCSV` functions will not guarantee that you are outputting data in a format that is acceptable for
      importing CSV logs into the [Ardusat Experiment Platform](http://experiments.ardusat.com). These functions
      are for live STREAMING data into the Experiment Platform.

      If you are calling the `toCSV` functions on exactly one type of Sensor Class per sketch, this is fine for
      STREAMING data and LOGGING data to be imported.

      If you are calling the `toCSV` functions on more than one type of Sensor Class per sketch, this is fine for
      only STREAMING data, LOGGING data to be important will not be formatted correctly. If you want to LOG more
      than one type of Sensor Class, see the CSV Logging Example with `valuesToCSV`.

There is a full example in the CSV Examples of how to use `valuesToCSV`, but this function is available to make
it easier to format data properly:
```
const char * valuesToCSV(const char *sensorName, unsigned long timestamp, int numValues, float values...);
const char * valueToCSV(const char *sensorName, float value, unsigned long timestamp); // if timestamp == 0, it will just calculate a timestamp using millis()
```

#### Sensor Specifics
This is an overview of the sensor specific fields and advanced configuration parameters

Sensor Class | Data Fields | Optional `begin` Parameters (for advanced configuration)
--- | --- | ---
Acceleration | `float x`, `float y`, `float z`              | `lsm303_accel_gain_e gGain`
Gyro         | `float x`, `float y`, `float z`              | `uint8_t range`
Luminosity   | `float lux`                                  | `tsl2561IntegrationTime_t intTime, tsl2561Gain_t gain`
Magnetic     | `float x`, `float y`, `float z`              | `lsm303_mag_scale_e gaussScale`
Orientation  | `float roll`, `float pitch`, `float heading` | `Acceleration & accel, Magnetic & mag` (Existing Accel and Mag objects)
Pressure     | `float pressure`                             | `bmp085_mode_t mode`
RGBLight     | `float red`, `float green`, `float blue`     | TCS34725: `tcs34725IntegrationTime_t it, tcs34725Gain_t gain` ISL29125: `uint8_t islIntensity`
Temperature  | `float t`                                    | TMP102: `None` MLX90614: `None`
UVLight      | `float uvindex`                              | ML8511: `int pin` SI1132: `None`


These are the specific values avaliable for each of the advanced configurations

`begin` Parameter Type | Acceptable Values | Sensor Class
--- | --- | ---
`lsm303_accel_gain_e gGain` | `LSM303_ACCEL_GAIN2G`, `LSM303_ACCEL_GAIN4G`, `LSM303_ACCEL_GAIN6G`, `LSM303_ACCEL_GAIN8G` (Default), `LSM303_ACCEL_GAIN16G` | Acceleration
`uint8_t range` | `0x00` (SENSITIVITY_250DPS), `0x10` (SENSITIVITY_500DPS), `0x20` (SENSITIVITY_2000DPS) (Default) | Gyro
`tsl2561IntegrationTime_t intTime` | `TSL2561_INTEGRATIONTIME_13MS` (Default), `TSL2561_INTEGRATIONTIME_101MS`, `TSL2561_INTEGRATIONTIME_402MS` | Luminosity
`tsl2561Gain_t gain` | `TSL2561_GAIN_1X` (Default), `TSL2561_GAIN_16X` | Luminosity
`lsm303_mag_scale_e gaussScale` | `LSM303_MAG_SCALE1_3GAUSS`, `LSM303_MAG_SCALE2GAUSS`, `LSM303_MAG_SCALE2_5GAUSS`, `LSM303_MAG_SCALE4GAUSS` (Default), `LSM303_MAG_SCALE4_7GAUSS`, `LSM303_MAG_SCALE5_6GAUSS`, `LSM303_MAG_SCALE8GAUSS`, `LSM303_MAG_SCALE12GAUSS` | Magnetic
`bmp085_mode_t mode` | `BMP085_MODE_ULTRALOWPOWER`, `BMP085_MODE_STANDARD`, `BMP085_MODE_HIGHRES`, `BMP085_MODE_ULTRAHIGHRES` (Default) | Pressure
`tcs34725IntegrationTime_t it` | `TCS34725_INTEGRATIONTIME_2_4MS`, `TCS34725_INTEGRATIONTIME_24MS`, `TCS34725_INTEGRATIONTIME_50MS`, `TCS34725_INTEGRATIONTIME_101MS`, `TCS34725_INTEGRATIONTIME_154MS` (Default), `TCS34725_INTEGRATIONTIME_700MS` | RGBLight - TCS34725
`tcs34725Gain_t gain` | `TCS34725_GAIN_1X` (Default), `TCS34725_GAIN_4X`, `TCS34725_GAIN_16X`, `TCS34725_GAIN_60X` | RGBLight - TCS34725
`uint8_t islIntensity` | `CFG1_375LUX`, `CFG1_10KLUX` (Default) | RGBLight - ISL29125
`int pin` | `DRIVER_ML8511_UV_PIN` (A0) (Default), or any other Arduino Pin | UVLight - ML8511


#### Sensor Notes
Because the Orientation "Sensor" is really just a derived value from the sensor readings from the
Accelerometer and Magnetometer, the Orientation sensor can use existing Accelerometer and Magnetic
sensor objects. Accelerometer and Magnetic Sensor objects can be provided to the Orientations `begin`
function so that it just uses those sensors, otherwise it will create its own. There are also extra
read and print functions so that the Orientation will be calculated without necessarily calling read
on the Accelerometer or Magnetic Sensors:

```
void Orientation::read(Acceleration & accel, Magnetic & mag);
const char * Orientation::readToCSV(Acceleration & accel, Magnetic & mag, const char * sensorName);
const char * Orientation::readToJSON(Acceleration & accel, Magnetic & mag, const char * sensorName);
```


The Barometric Pressure sensor has two additional convenience functions to calculate altitude (which
requires knowing the current sea level barometric pressure, a value that's easily available from
weather observation data), or current sea level barometric pressure (which requires knowing the
current altitude). For these convenience functions, altitude should be provided in meters, and
pressure values should be provided in hPa.

```
float Pressure::altitudeFromSeaLevelPressure(float seaLevelPressure);
float Pressure::seaLevelPressureFromAltitude(float altitude);
```

To translate meters to feet, multiply the meter value by `3.28084`. To translate feet to meters,
multiply the feet value by `0.3084`.



## Sensor Usage Examples
```
#include <ArdusatSDK.h>

Luminosity lum;

void setup(void) {
  Serial.begin(9600);
  lum.begin();
}

void loop(void) {
  Serial.println(lum.readToJSON());       // >> ~{"sensorName": "Luminosity", "unit": "lux", "value": 123.5, "cs": 43}|
  Serial.println(lum.readToJSON("lum2")); // >> ~{"sensorName": "lum2", "unit": "lux", "value": 121.9, "cs": 30}|
  Serial.println(lum.readToCSV());        // >> 243,Luminosity,128.2,49
  Serial.println(lum.readToCSV("lum4"));  // >> 311,lum4,124.6,29
  Serial.println(lum.lux);                // >> 124.6
  Serial.println(lum.lux);                // >> 124.6
  lum.read();
  Serial.println(lum.lux);                // >> 127.5
}
```

```
#include <ArdusatSDK.h>

RGBLight rgb = RGBLight(SENSORID_TSL2561);

void setup(void) {
  Serial.begin(9600);
  rgb.begin(CFG1_375LUX);
}

void loop(void) {
  Serial.println(rgb.readToJSON()); // >> ~{"sensorName": "RGBLight", "unit": "lux", "red": 89.2, "green": 177.9, "blue": 183.2, "cs": 83}|
}
```


## CSV Examples
As mentioned in the *NOTE* in "Sensor Overview and Reference", the Experiment Platform reads reads two different types of CSV
formats. The `toCSV` family of functions work fine on multiple types of Sensor Classes at a time when you want to STREAM data,
but if you want to LOG data to be imported, you need to have properly formatted data.

####CSV STREAMING example:
```
#include <ArdusatSDK.h>

Magnetic mag;
Luminosity lum;

void setup(void) {
  Serial.begin(9600);
  mag.begin();
  lum.begin();
}

void loop(void) {
  Serial.println(mag.readToCSV());
  Serial.println(lum.readToCSV());
}
```
Output: (not proper CSV format) Will fail to be imported to Experiment Platform
        However, this is fine if you're streaming CSV data to the Experiment PLatform.
```
89,Magnetic,0.78,9.11,0.02,123
123,Luminosity,128.2,234
145,Magnetic,0.74,9.01,0.03,234
168,Luminosity,126.7,423
...
```


####CSV LOGGING example:
```
#include <ArdusatSDK.h>

Acceleration accel;
Temperature temp;

void setup(void) {
  Serial.begin(9600);
  accel.begin();
  temp.begin();

  // CSV Headers to print once at the top of the data
  Serial.print("timestamp (millis), name, ");
  Serial.print("temperature (C), x acceleration (m/s^2), y acceleration (m/s^2), z acceleration (m/s^2), ");
  Serial.println("checksum");
}

void loop(void) {
  accel.read();
  temp.read();

                            //Log Name, Time Stamp, Number of Values, Temp, Accel X, Accel Y, Accel Z
  Serial.println(valuesToCSV("reading", accel.header.timestamp, 4, temp.t, accel.x, accel.y, accel.z));
}
```
Output: Proper CSV format. This will be successfully imported into the Experiment Platform
        However, this will not work if you're streaming CSV data to the Experiment PLatform.
```
timestamp (millis), name, temperature (C), x acceleration (m/s^2), y acceleration (m/s^2), z acceleration (m/s^2), checksum
89,reading,27.3,9.11,0.02,0.78,123
123,reading,27.2,9.10,0.08,0.73,120
145,reading,27.3,9.02,0.11,0.88,121
198,reading,27.2,9.08,0.03,0.80,128
211,reading,27.3,9.13,0.08,0.75,119
229,reading,27.5,9.19,0.01,0.78,127
...
```


## Different Output Formats
The Ardusat SDK can output sensor data in both JSON and CSV format to allow interfacing with
external systems such as the [Ardusat Experiment Platform](http://experiments.ardusat.com). To use these functions,
call the `toJSON` or `toCSV` family of functions:



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
Temperature temp;

void setup(void) {
  serialConnection.begin(9600);
  temp.begin();

  serialConnection.println("This message will go out on hardware serial and software serial!");
}

void loop(void) {
  temp.read();
  serialConnection.println("The recorded temperature is ");
  serialConnection.println(temp.t);
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
library locations in your Arduino IDE. Double check that the ArdusatSDK library is included into
your Arduino libraries (Sketch -> Import Libraries -> Contributed). If the sketches are compiling 
and uploading but not behaving as expected, make sure you double check your wiring, it's always easy
to accidentally plug something in wrong! 

If you get really stuck, feel free to reach out at <support@ardusat.com>, or the "Issues" section of
this repository.
