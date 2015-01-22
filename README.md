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
`setupMagneticSensor` | `magnetic_t` | `x, y, z` | LSM303 (9DOF breakout)
`setupOrientationSensor` | `orientation_t` | `x, y, z` | L3GD20 (9DOF breakout)
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

After the logging system is started, the `writeX` and `binaryWriteX` functions can be used to
actually write the binary data much like the `ToJSON` and `ToCSV` functions listed above. Binary and
CSV formats are described below.

### CSV Log Format
CSV data will be logged exactly as output from `ToCSV` functions appear on the Serial output
display. The format is: `timestamp (ms),sensorName,values`. Note that unless a real time clock chip
is used, the Arduino has no ability to know the actual time, so `timestamp` will be the time in MS
since the Arduino chip was started.

### Binary Data Format
The binary data format allows data to be logged with less space on the SD card, enabling more data
to be logged in the same space allotment. However, these binary data stamps must be decoded before
they can be used by external data programs. To do this, a simple utility, `decode_binary.c` has been
provided with the SDK. It must be compiled before use. On Mac OS X, this requires command line build
utilities (`gcc`), on Linux the `gcc` compiler is usually included by default. This utility currently
does not support Windows systems, but Cygwin can likely be used for compatibility, and we will be
adding conversion support on the Ardusat website shortly.

**Compile decode_binary with gcc (Mac OS X/Linux)**
```
cd /path/to/ArdusatSDK
gcc decode_binary.c -o decode_binary
```

Once the binary is compiled, it can be used (run ./decode_binary --help for usage information) to
translate the binary data into regular CSV data with the format. If no output path is given, the output path will
default to the filename of the binary file, saved to the current working directory.
```
>> ./decode_binary --help
Decodes a binary data file created using the ArdusatSDK.
usage: ./decode_binary [options] FILE
Options:
  -o,--output-file PATH          CSV file to write decoded data to
  -h,--help                      Print this usage info.
>> ./decode_binary -o ./my-data.csv /Volumes/SD\ Card/DATA/MYLOG0.BIN
Decoding file /Volumes/SD Card/DATA/MYLOG0.BIN and saving data to
/Users/ardusat/Downloads/ArdusatSDK/my-data.csv...
Finished decoding /Volumes/SD Card/DATA/MYLOG0.BIN. Saved 132 data observations to
/Users/ardusat/Downloads/ArdusatSDK/my-data.csv.
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
