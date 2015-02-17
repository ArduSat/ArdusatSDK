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
**beginMagneticSensor** | LSM303 (9DOF breakout) | None
**beginOrientationSensor** | L3GD20 (9DOF breakout) | None
**beginUVLightSensor** | ML8511 | None
**beginBarometricPressureSensor** | BMP180 | None

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

In addition to these `read` functions, a convenience function `calculateOrientation` is provided
to calculate the 3-axis orientation from raw data from the accelerometer and magnetometer. This 
function calculates `roll` (rotation about `x` axis), `pitch` (rotation about `y` axis), and 
`heading` (rotation about `z` axis), and has the following signature:

```
void calculateOrientation(const acceleration_t *, const magnetic_t *, orientation_t *);
```

Usage example:
```
#import <ArdusatSDK.h>

Serial.begin(9600);
temperature_t temp_data;
if (!beginTemperatureSensor()) {
  Serial.println("There was a problem initializing the temperature sensor.");
  while (1);
}
readTemperature(&temp_data);
Serial.println(temp_data.t);
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

### JSON Format
Function | Arguments
--- | ---
**valueToJSON** | `const char *sensor_name, uint8_t unit_definition, float value`
**accelerationToJSON** | `const char *sensor_name, acceleration_t *acceleration_data`
**magneticToJSON** | `const char *sensor_name, magnetic_t *magnetic_data`
**orientationToJSON** | `const char *sensor_name, orientation_t *orientation_data`
**temperatureToJSON** | `const char *sensor_name, temperature_t *temp_data`
**luminosityToJSON** | `const char *sensor_name, luminosity_t *luminosity_data`
**uvlightToJSON** | `const char *sensor_name, uvlight_t *uvlight_data`

Example:
```
  temperature_t temp_data;
  readTemperature(&temp_data);
  Serial.println(temperatureToJSON("temperature", &temp_data));
  >> ~{"sensorName": "temperature", "unit": "C", "value": 23.5}|
```

### CSV Format
Function | Arguments
--- | ---
**accelerationToCSV** | `const char *sensor_name, acceleration_t *acceleration_data`
**magneticToCSV** | `const char *sensor_name, magnetic_t *magnetic_data`
**orientationToCSV** | `const char *sensor_name, orientation_t *orientation_data`
**temperatureToCSV** | `const char *sensor_name, temperature_t *temp_data`
**luminosityToCSV** | `const char *sensor_name, luminosity_t *luminosity_data`
**uvlightToCSV** | `const char *sensor_name, uvlight_t *uvlight_data`

Example:
```
  temperature_t temp_data;
  readTemperature(&temp_data);
  Serial.println(temperatureToCSV("temperature", &temp_data));
  >> 123,acceleration,0.1,0.3,9.81
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
ArdusatSerial(serialMode mode, uint8_t softwareReceivePin, uint8_t softwareTransmitPin);
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
  readTemperature(&temp_data);
  serialConnection.println(temperatureToCSV("temperature", &temp_data));
}

```

#### Serial Modes
Name | Description
--- | ---
**SERIAL_MODE_HARDWARE** | Output serial data on built-in USB hardware serial
**SERIAL_MODE_SOFTWARE** | Output serial data on software serial (must specify transmit & receive pins in the constructor, see arguments in example above)
**SERIAL_MODE_HARDWARE_AND_SOFTWARE** | Output to both hardware and software serial interfaces

#### Limitations
SoftwareSerial does not appear to work reliably above 57600 baud. 

## Logging Sensor Data
Data can be logged to an SD card to allow gathering data without an active connection to a computer.
To do this, an external SD card adapter breakout board is required; these are available from
[Adafruit](http://www.adafruit.com/product/254) and
[SparkFun](https://www.sparkfun.com/products/12941). These boards use an SPI protocol that uses
digital pins 11, 12, and 13 on most Arduinos, and additionally requires a configurable "chip select"
pin that is often pin 10. 

**External SD Breakout Wiring**

SD Breakout Pin | Arduino Pin
--- | ---
CLK | DIO13
DO | DIO12
DI | DIO11
CS | DIO10

If using the [SparkFun SD Shield](https://www.sparkfun.com/products/12761), chip select (`CS`) is
set to Arduino pin 8, rather than pin 10, so sketches must reflect that (see below)

The exact SD cards supported might vary from board to board, but most should support MicroSD and
MicroSDHC cards formatted in FAT32 (or FAT16) formats.

In order to log SD data, the function `beginDataLog` must be called. `beginDataLog` has three
arguments: 

`bool beginDataLog(int chipSelectPin, const char *fileNamePrefix, bool logCSVData)`

The chip select pin argument must correspond to the pin connected to CS on the SD breakout board.
`fileNamePrefix` specifies the filename to be used for data log files. Log files will be placed in a
subdirectory `/DATA` on the SD card with sequential filenames up to 8 characters long (any longer
filename will be truncated as appropriate). Thus log files will be `/DATA/MYLOGFI0.CSV` to
`/DATA/MYLOGFI9.CSV`, followed by `/DATA/MYLOGF10.CSV`, etc. Finally, the `logCSVData` argument
specifies whether binary-format data logging (more space efficient, but must be decoded before use)
or CSV format (can be read by a wide variety of software, but takes up more space) will be used. 
Binary formatted logs end in `.BIN`, CSV formatted logs end in `.CSV`. `beginDataLog` will return
`true` if the log system was started successfully, `false` otherwise.

If a Real Time Clock (RTC) chip is connected to the Arduino and set properly, the logging system
will automatically read the current POSIX time (seconds since 1/1/1970) from the RTC, as well as the
milliseconds since the Arduino started (`millis()`) and log both these values to the beginning of
the log file. Since all subsequent values log milliseconds since the Arduino started (again, using
`millis()`), this offset line can be used to translate these relative timestamps into absolute
datetimes. The RTC chip used is the DS1307, and should be wired up on the I2C bus using `SDA` and
`SCL` pins.

After the logging system is started, the `writeX` and `binaryWriteX` functions can be used to
actually write the binary data much like the `ToJSON` and `ToCSV` functions listed above. Binary and
CSV formats are described below.

### CSV Log Format
CSV data will be logged exactly as output from `ToCSV` functions appear on the Serial output
display. The format is: `timestamp (ms),sensorName,values`. Note that unless a real time clock chip
is used, the Arduino has no ability to know the actual time, so `timestamp` will be the time in MS
since the Arduino chip was started.

### Logging Other Sensor Data
If you have a custom sketch that includes data which doesn't fit into any of the ArdusatSDK-provided
data structures, two convenience functions `valueToCSV` and `valuesToCSV` are provided to format
generic `float` values into CSV strings. `valueToCSV` writes a single `float` to the CSV string,
while `valuesToCSV` writes an array of strings. The output of these functions can be given to the
`writeString` function to log the data to an SD card. Both `toCSV` functions take an optional
`timestamp` argument - if this is not provided, the current time will be added to the CSV string
using the `millis()` function.

Usage example:

```
float mySingleValue;
typedef struct {
  float myValue1;
  float myValue2;
  float myValue3;
} myValues_t;

myValues_t myValues;

mySingleValue = someMeasurementFunction();
long currTime = millis();

myValues.myValue1 = someOtherMeasurementFunction();
myValues.myValue2 = another();
myValues.myValue3 = thirdMeasurementFunction();

writeString(valueToCSV("first_sensor", mySingleValue, currTime));
writeString(valuesToCSV("my_vector_data", (float *) &myValues, 3));
```

### Binary Data Format
The binary data format allows data to be logged with less space on the SD card, enabling more data
to be logged in the same space allotment. However, these binary data stamps must be decoded before
they can be used by external data programs. To do this, a simple Python-based utility,
`decode_binary.py` has been provided with the SDK. On Mac OS X and Linux systems, Python should be
installed already, but might need to be upgraded for pre Python 2.5 versions (fairly uncommon). On
Windows, Python must be installed before the script can be used - go to 
[Python Downloads](https://www.python.org/downloads/) to download an installer or newer version of
Python. We will also be adding conversion support on the Ardusat website at some point.

Once Python is installed, the utility can be used (run `python ./decode_binary.py --help` for usage 
information) to translate the binary data into regular CSV data with the format. If no output path 
is given, the output path will default to the filename of the binary file, saved to the current 
working directory.

**decode_binary.py usage options**
```
>> python ./decode_binary.py --help
usage: decode_binary.py [-h] [-o,--output-file OUTPUT_FILE]
                        [-s,--stop-on-error]
                        input_file

Decodes a binary data file created using the ArdusatSDK.

positional arguments:
  input_file            Binary data file to decode

optional arguments:
  -h, --help            show this help message and exit
  -o,--output-file OUTPUT_FILE
                        CSV file to write decoded data to
  -s,--stop-on-error    Stop decoding if unexpected bytes encountered
```

**Decode a binary data file (Mac OS X/Linux)**
```
>> python ./decode_binary.py -o my_data.csv MYDATA0.BIN
Decoding MYDATA0.BIN (24516 bytes) and saving data to my_data.csv...
Finished decoding MYDATA0.BIN, saved 1362 data observations to my_data.csv
```

**Decode a binary data file (Windows)**
```
>> C:\Python27\python.exe C:\ArdusatSDK\decode_binary.py -o C:\ArdusatSDK\my_data.csv
C:\ArdusatSDK\MYDATA0.BIN
Decoding C:\ArdusatSDK\MYDATA0.BIN (24516 bytes) and saving data to C:\ArdusatSDK\my_data.csv...
Finished decoding C:\ArdusatSDK\MYDATA0.BIN, saved 1362 data observations to C:\ArdusatSDK\my_data.csv
```

The actual data format for each time of data is described below, along with the number of bytes for
each reading, which can be used to calculate the total amount of space required by data.

Every data log has an identical header that identifies the sensor, the type of sensor, and the
timestamp the measurement was taken.
```
uint8_t type;
uint8_t id;
uint32_t timestamp;
```

After this header, the length of the data structure depends on the type of data. Data is written
using 4 byte floating point format.

#### Acceleration Data (18 bytes total)
```
header (6 bytes)
float x;
float y;
float z;
```

#### Magnetic Data (18 bytes total)
```
header (6 bytes)
float x;
float y;
float z;
```

#### Orientation Data (18 bytes total)
```
header (6 bytes)
float x;
float y;
float z;
```

#### Temperature Data (10 bytes total)
```
header (6 bytes)
float temp;
```

#### Luminosity Data (10 bytes total)
```
header (6 bytes)
float lux;
```

#### UV Light Data (10 bytes total)
```
header (6 bytes)
float uv;
```

See `examples/sd_card/sd_card.ino` for a usage example.

# Getting Help
If you're having trouble running the examples, chances are something is messed up with the external
library locations in your Arduino IDE. Double check that the ArdusatSDK library is imported into
your Arduino libraries (Sketch -> Import Libraries -> Contributed). If the sketches are compiling 
and uploading but not behaving as expected, make sure you double check your wiring, it's always easy
to accidentally plug something in wrong! 

If you get really stuck, feel free to reach out at <support@ardusat.com>, or the "Issues" section of
this repository.
