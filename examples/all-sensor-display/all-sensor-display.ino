#include <ArdusatSDK.h>

#define SHOW_LOGO_DELAY_MS 3000

#define OLED_RESET 4
#define OLED_I2C 0x3C
Display display;

Acceleration acc;
Gyro         gyr;
Luminosity   lum;
RGBLightTCS  rgb(TCS34725_INTEGRATIONTIME_24MS);
Magnetic     mag;
Temperature  tmp;
UVLight      uvl;

enum SENSORSTATE {
  ACCELERATION, // accelerometer  (DRIVER_LSM303_ADDR = 0x1E, same as magnetometer)

  GYRO,         // gyroscope      (L3GD20_ADDRESS = 0x6B)

  LUMINOSITY,   // luminosity     (DRIVER_SPACEBOARD_TSL2561_ADDR = 0x49  pre v1.5 spaceboard)

  RGBLIGHT,     // red, green, blue light sensor
                //                (DRIVER_SPACEBOARD_TCS34725_ADDR = 0x29)
                // Secondary RGB: (DRIVER_SPACEBOARD_ISL29125_ADDR = 0x44)

  MAGNETIC,     // magnetometer   (same I2C as accelerometer)

  TEMPERATURE,  // infrared       (DRIVER_SPACEBOARD_TMP102_ADDR = 0x4B)
                // Secondary IR:  (DRIVER_MLX90614_ADDR = 0x5A)

  UVLIGHT,      // ultraviolet light
                //                (DRIVER_SI1132_ADDR = 0x60)
                // Secondary UV:  (DRIVER_ML8511_ADDR = 0x51)
  SENSOR_STATE_END
};

const char msg_sensor_not_found[] PROGMEM = "Sensor Not Found";
const char msg_o_spaceboard[] PROGMEM = "O Spaceboard...";
const char msg_where_art_thou[] PROGMEM = "Where Art Thou?";
const char msg_cant_start[] PROGMEM = "Check board is 3.3V";
const char msg_please_set_3v3[] PROGMEM = "and power off for 5s";

float barValue;
float gForce;
enum SENSORSTATE sensorState = ACCELERATION;

void nextSensorState() {
  sensorState = ((int)sensorState) + 1;
  if (sensorState == SENSOR_STATE_END) {
    sensorState = 0;
  }
}

bool i2cDeviceFoundAt(uint8_t deviceAddress) {
  Wire.beginTransmission(deviceAddress);
  return (Wire.endTransmission() == 0);
}

void dieBlinking(int pulses) {
  int i;
  pinMode(LED_BUILTIN, OUTPUT);
  while(1) {
    for (i = 0; i < pulses; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
    delay(750);
  }
}

void printCentered(const char* text, int line, int lineHeight = 12) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(display.width()/2 - w/2, line * lineHeight);
  display.print(text);
}

char buf[60];
// print text string that is stored in program space
void printCentered_P(const char* text, int line, int lineHeight = 12) {
  strcpy_P(buf, (PGM_P)text);
  printCentered(buf, line, lineHeight);
}

void printMessage2_P(PGM_P text1, PGM_P text2) {
  display.clearDisplay();
  printCentered_P(text1, 0);
  printCentered_P(text2, 1);
  display.display();
}

float getGravForce(Acceleration* a) {
  return sqrt(sq((*a).x)+sq((*a).y)+sq((*a).z)) / 9.8;
}

void setup() {
  bool oledFound,
       accMagFound,
       lumFound,
       rgbFound,
       gyrFound,
       tmpFound,
       uvlFound,
       allSensorsFound,
       noSensorFound;
  bool accelerometerOk = false;
  unsigned long time = millis();

  Wire.begin();
  Serial.begin(9600);

  // First thing, check for expected I2C spaceboard sensors, and OLED display
  oledFound = i2cDeviceFoundAt(OLED_I2C);
  accMagFound = i2cDeviceFoundAt(DRIVER_LSM303_ADDR);
  rgbFound = i2cDeviceFoundAt(DRIVER_SPACEBOARD_TCS34725_ADDR);
  gyrFound = i2cDeviceFoundAt(L3GD20_ADDRESS);
  tmpFound = i2cDeviceFoundAt(DRIVER_SPACEBOARD_TMP102_ADDR);
  uvlFound = i2cDeviceFoundAt(DRIVER_ML8511_ADDR);

  allSensorsFound =
    accMagFound && rgbFound &&
    gyrFound && tmpFound && uvlFound;

  noSensorFound =
    !accMagFound && !rgbFound &&
    !gyrFound && !tmpFound && !uvlFound;

  if (oledFound) {
    // Initialize OLED
    display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C);
  }

  if (allSensorsFound) {
    // Initialize Sensors
    gyr.begin();
    lum.begin();
    rgb.begin();
    mag.begin();
    tmp.begin();
    uvl.begin();
    acc.begin();
  }


  if (oledFound) {
    if (allSensorsFound) {
      // Show Logo
      display.display();

      // Check for case where Arduino is set for 5V and the
      // Accelerometer/Magnetometer doesn't function as a result

      // Wait 3 seconds, and while we're waiting, check that the
      // accelerometer is not "stuck" at a crazy value. If it is,
      // it indicates the board is in 5V mode (and should be set
      // to 3.3V) OR the board was quickly re-powered (see
      // https://www.pololu.com/product/2127/faqs for a wild ride)

      while (millis() < time + SHOW_LOGO_DELAY_MS) {
        delay(1);
        // TODO: check accel
        acc.read();
        gForce = getGravForce(&acc);
        if (gForce > 0.1 && gForce < 4.0) accelerometerOk = true;
      }
      
      if (!accelerometerOk) {
        printMessage2_P(msg_cant_start, msg_please_set_3v3);
        
        // 1-blink code means something's wrong with the spaceboard
        dieBlinking(1);
      }

      display.clearDisplay();

    } else if (noSensorFound) {
      // Most likely, the sensor board is not attached
      printMessage2_P(msg_o_spaceboard, msg_where_art_thou);
      
      // 1-blink code means something's wrong with the spaceboard
      dieBlinking(1);
    } else {
      // At least one sensor was not found
      if (!accMagFound) {
        printMessage2_P(acceleration_sensor_name, msg_sensor_not_found);
      }
      if (!rgbFound) {
        printMessage2_P(rgblight_sensor_name, msg_sensor_not_found);
      }
      if (!gyrFound) {
        printMessage2_P(gyro_sensor_name, msg_sensor_not_found);
      }
      if (!tmpFound) {
        printMessage2_P(irtemperature_sensor_name, msg_sensor_not_found);
      }
      if (!uvlFound) {
        printMessage2_P(uvlight_sensor_name, msg_sensor_not_found);
      }
      
      // 1-blink code means something's wrong with the spaceboard
      dieBlinking(1);
    }
  } else { // OLED wasn't found
    
    // Since the OLED wasn't found, we'll need to show errors
    // via blinky lights instead of on the display
    if (allSensorsFound) {
      // 2-blink code means something's wrong with the OLED display
      dieBlinking(2);
    } else {
      // Most likely, the sensor board is not attached
      // 3-blink code means something's wrong with both the spaceboard AND the OLED display
      dieBlinking(3);
    }
  }


}

void drawBar(float percent, int x = 0, int w = display.width()) {
  int width = (int)(percent / 100 * w);
  
  display.fillRect(x, 12, width, 24, WHITE);
}

void drawCenteredBar(float delta) {
  if (delta > 0) {
    display.fillRect(display.width()/2, 12, delta, 24, WHITE);
  } else {
    display.fillRect(display.width()/2 + delta, 12, -delta, 24, WHITE);
  }
}

bool acceptUserInput = true;
int waitUserInput = 0;
int w;

void loop() {
  display.clearDisplay();

  acc.read();
  gForce = getGravForce(&acc);

  if (acceptUserInput) {
    if (gForce > 1.5) {
      nextSensorState();
      acceptUserInput = false;
      waitUserInput = 10;
    }
  } else {
    if (waitUserInput > 0) {
      waitUserInput--;
    } else {
      acceptUserInput = true;
    }
  }

  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  switch(sensorState) {
    case ACCELERATION:
      drawBar(gForce * 10);
    
      // text display tests
      display.print("Acceleration: ");
      display.print(gForce);
    break;

    case GYRO:
      gyr.read();

      drawCenteredBar(gyr.x * 50);
      
      // text display tests
      display.print("Gyro: ");
      display.print(gyr.x);
    break;
    
    case LUMINOSITY:
      lum.read();
    
      drawBar(lum.lux / 50);
    
      // text display tests
      display.print("Luminosity: ");
      display.print((int)lum.lux);
    break;
    
    case MAGNETIC:
      mag.read();

      drawBar(mag.x);
      
      display.print("Magnetic: ");
      display.print(mag.x);
    break;
    
    case RGBLIGHT:
      w = display.width()/3;
      rgb.read();
      
      drawBar(rgb.red, w * 0, w);
      drawBar(rgb.green, w * 1, w);
      drawBar(rgb.blue, w * 2, w);
      
      display.setCursor(w*0,0);
      display.print("R: ");
      display.print((int)rgb.red);

      display.setCursor(w*1,0);
      display.print("G: ");
      display.print((int)rgb.green);

      display.setCursor(w*2,0);
      display.print("B: ");
      display.print((int)rgb.blue);
    break;
    
    case TEMPERATURE:
      tmp.read();

      drawBar(tmp.t);
      
      display.print("Temperature: ");
      display.print(tmp.t);
    break;
    
    case UVLIGHT:
      uvl.read();

      drawBar(uvl.uvindex * 10);
      
      display.print("UltraViolet: ");
      display.print(uvl.uvindex);
    break;
  }


  display.display();

  delay(10);
}
