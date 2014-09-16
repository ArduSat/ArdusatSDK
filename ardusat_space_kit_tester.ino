/*************************************************** 
  This is a library example for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
// ------------------------------------------
// Serial Read LED on/off
/* Use a variable called byteRead to temporarily store
   the data coming from the computer */
byte byteRead;

// ------------------------------------------
//Photoresistor
int LDR_Pin = A2; //analog pin 2

// ------------------------------------------
//IMU LSM303/l3GD20
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

// ------------------------------------------
//TMP102 Temp
//#include <Wire.h>
#include <tmp102.h>
#define TMP102_ADDR 0x48
int led_tmp102 = 3;
Tmp102 tmp102(&Wire, TMP102_ADDR);

// ------------------------------------------
//MLX90614 IR Thermopile
//#include <Wire.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int led_mlx90614 = 2;
float objectTemp = 0;

// ------------------------------------------
//TSL2561 Lux
//#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
int led_tsl2561 = 4;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// ------------------------------------------
//MP8511 UV SparkFun
//Hardware pin definitions
int UVOUT = A0; //Output from the sensor
int REF_3V3 = A1; //3.3V power on the Arduino board
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led_mp8511 = 5;
int ledon = 0;


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}


void setup() {
        Serial.begin(9600);
        Serial.println("Ardusat Space Kit tester"); 
        
        // ------------------------------------------
        //IMU LSM303/l3GD20
        /* Initialise the sensors */
        if(!accel.begin())
        {
          /* There was a problem detecting the ADXL345 ... check your connections */
          Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
          while(1);
        }
        if(!mag.begin())
        {
          /* There was a problem detecting the LSM303 ... check your connections */
          Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
          while(1);
        }
        if(!gyro.begin())
        {
          /* There was a problem detecting the L3GD20 ... check your connections */
          Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
          while(1);
        }
        
        // ------------------------------------------
        //TMP102 Temp
        // initialize the digital pin as an output.
        pinMode(led_tmp102, OUTPUT);
        
        // ------------------------------------------
        //MLX90614 IR Thermopile
        pinMode(led_mlx90614, OUTPUT);       
        mlx.begin(); 
        
        // ------------------------------------------
        //TSL2561 Lux
        pinMode(led_tsl2561, OUTPUT);
        
        /* Initialise the sensor */
        if(!tsl.begin())
        {
          /* There was a problem detecting the ADXL345 ... check your connections */
          Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
          while(1);
        }
  
        /* Display some basic information on this sensor */
        displaySensorDetails();
  
        /* Setup the sensor gain and integration time */
        configureSensor();
        
        // ------------------------------------------
        //MP8511 UV SparkFun
        pinMode(UVOUT, INPUT);
        pinMode(REF_3V3, INPUT);
        //Serial.println("MP8511 example");
        // LED initialize the digital pin as an output.
        pinMode(led_mp8511, OUTPUT); 
        
        
        /* We're ready to go! */
        Serial.println("");
}

void loop(){
        // ------------------------------------------
        // Serial Read LED on/off
        // Entering 1 will turn ON, 0 or any other number turns OFF
        /*  check if data has been sent from the computer: */
        if (Serial.available()) {
          /* read the most recent byte */
          byteRead = Serial.read();
          /*ECHO the value that was read, back to the serial port. */
          Serial.write(byteRead);
          if (byteRead == 49) // Equals 1 / ON
          {
            digitalWrite(led_mp8511, HIGH);   // turn the LED on (HIGH is the voltage level)
          }
          else
          {
            digitalWrite(led_mp8511, LOW);    // turn the LED off by making the voltage LOW
          }  
        }
  
        // ------------------------------------------
        //Photoresistor
        int LDRReading = analogRead(LDR_Pin);
        Serial.print(F("Photoresistor:\t"));
        Serial.println(LDRReading);
  
        // ------------------------------------------
        //IMU LSM303/l3GD20
        /* Get a new sensor event */
        sensors_event_t event;
   
        /* Display the results (acceleration is measured in m/s^2) */
        accel.getEvent(&event);
        Serial.print(F("Accelerometer:\t"));
        Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

        /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
        mag.getEvent(&event);
        Serial.print(F("Magnetometer:\t"));
        Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

        /* Display the results (gyrocope values in rad/s) */
        gyro.getEvent(&event);
        Serial.print(F("Gyroscope:\t"));
        Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
        Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
        Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s "); 
  
        // ------------------------------------------
        // tmp102 
        float celsius;
        float fahrenheit;
        celsius = tmp102.read();
        // Celsius to Fahrenheit: Multiply by 9, then divide by 5, then add 32
        fahrenheit = celsius * 9 / 5 + 32;
	Serial.print("tmp102:\t\t"); Serial.print("Ambient = "); Serial.print(fahrenheit); Serial.println("*F");      
        if (fahrenheit > 85)
        {
          digitalWrite(led_tmp102, HIGH);   // turn the LED on (HIGH is the voltage level)
        }
        else
        {
          digitalWrite(led_tmp102, LOW);    // turn the LED off by making the voltage LOW
        }
	
        // ------------------------------------------
        // mlx90614
        //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
        //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
        Serial.print("mlx9014:\t"); Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
        objectTemp = mlx.readObjectTempF();
        Serial.print("*F\tObject = "); Serial.print(objectTemp); Serial.println("*F");
        if (objectTemp > 85)
        {
          digitalWrite(led_mlx90614, HIGH);   // turn the LED on (HIGH is the voltage level)
        }
        else
        {
          digitalWrite(led_mlx90614, LOW);    // turn the LED off by making the voltage LOW
        }
        
        // ------------------------------------------
        //TSL2561 Lux
        /* Get a new sensor event */ 
        //sensors_event_t event;
        tsl.getEvent(&event);
        Serial.print("tsl2561:\t");Serial.print("Luminosity = ");
        /* Display the results (light is measured in lux) */
        if (event.light)
        {
          int lux_tsl2561 = event.light;
          Serial.print(lux_tsl2561); Serial.println(" lux");
          
          if (lux_tsl2561 < 100)
          {
            digitalWrite(led_tsl2561, HIGH);   // turn the LED on (HIGH is the voltage level)
          }
          else
          {
            digitalWrite(led_tsl2561, LOW);    // turn the LED off by making the voltage LOW
          }
          int last_lux_tsl2561 = lux_tsl2561;
        }
        else
        {
        /* If event.light = 0 lux the sensor is probably saturated
         and no reliable data could be generated! */
          Serial.println("Sensor overload");
        }
        
        // ------------------------------------------
        //MP8511 UV SparkFun       
        int uvLevel = averageAnalogRead(UVOUT);
        int refLevel = averageAnalogRead(REF_3V3);
  
        //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
        float outputVoltage = 3.3 / refLevel * uvLevel;
  
        float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);

        Serial.print("MP8511: \t");
        Serial.print("UV Level = "); 
        Serial.print(uvLevel);

        Serial.print("\t voltage =  ");
        Serial.print(outputVoltage);

        Serial.print("\tUV Intensity (mW/cm^2) = ");
        Serial.println(uvIntensity);
        
        // ---------------------------------------------
        Serial.println("----------------------------------");      
        delay(1000);
}


// ------------------------------------------
//MP8511 UV SparkFun
        
//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
