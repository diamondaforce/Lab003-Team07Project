#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

void doHeartbeat();
long degreesToDutyCycle(int deg);

const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cTrigPin           = 38;                    // GPIO pin for ultrasonic trigger
const int cEchoPin           = 39;                    // GPIO pin for ultrasonic echo
const int cServo1Pin         = 41;                    // GPIO pin for servo motor 1
const int cServo2Pin         = 42;                    // GPIO pin for servo motor 2
const int cServo1Channel     = 5;                     // PWM channel used for the first RC servo motor
const int cServo2Channel     = 6;                     // PWM channel used for the second RC servo motor
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725    

Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED

  pinMode(cServo1Pin, OUTPUT);                      // configure servo 1 GPIO for output
  ledcSetup(cServo1Channel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServo1Pin, cServo1Channel);         // assign servo pin to servo channel
  
  pinMode(cServo2Pin, OUTPUT);                      // configure servo 2 GPIO for output
  ledcSetup(cServo2Channel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServo2Pin, cServo2Channel);         // assign servo pin to servo channel

  pinMode(cTrigPin, OUTPUT);  
	pinMode(cEchoPin, INPUT);  

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}
