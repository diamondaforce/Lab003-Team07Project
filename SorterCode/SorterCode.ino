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

// Variables
boolean timeUp1sec = false;                                                    // 1 second timer elapsed flag
boolean timeUp10milli = false;                                                 // 10 millisecond timer elapsed flag
boolean timeUp100milli = false;                                                // 100 millisecond timer elapsed flag
boolean timeUp500milli = false; 
unsigned char sortIndex = 0;
unsigned long timerCount1sec = 0;                                              // 1 second timer count in milliseconds
unsigned long timerCount10milli = 0;
unsigned long timerCount100milli = 0;
unsigned long timerCount500milli = 0;
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count
float duration, distance;                                                      // duration and distance measured from HC-SR04

float threshold = 2;

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

void loop() {
  currentMicros = micros();                                                    // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {                              // enter when 1 ms has elapsed
      previousMicros = currentMicros;                                          // record current time in microseconds

      // 1 second timer, counts 1000 milliseconds
      timerCount1sec = timerCount1sec + 1;                                     // increment 1 second timer count
      if (timerCount1sec > 1000) {                                             // if 1 second has elapsed
        timerCount1sec = 0;                                                    // reset 1 second timer count
        timeUp1sec = true;                                                     // indicate that 1 second has elapsed
      }

      // 50 millisecond timer
      timerCount500milli = timerCount500milli + 1;                             // increment 50 millisecond timer count
      if (timerCount500milli > 500) {                                          // if 50 milliseconds have elapsed
        timerCount500milli = 0;                                                // reset 50 millisecond timer count
        timeUp500milli = true;                                                 // indicate that 50 milliseconds have elapsed
      }

      // 100 millisecond timer
      timerCount100milli = timerCount100milli + 1;                             // increment 100 millisecond timer count
      if (timerCount100milli > 100) {                                          // if 100 milliseconds have elapsed
        timerCount100milli = 0;                                                // reset 100 millisecond timer count
        timeUp100milli = true;                                                 // indicate that 100 milliseconds have elapsed
      }

      // 10 millisecond timer
      timerCount10milli = timerCount10milli + 1;                               // increment 10 millisecond timer count
      if (timerCount10milli > 10) {                                           // if 10 milliseconds have elapsed
        timerCount10milli = 0;                                                 // reset 10 millisecond timer count
        timeUp10milli = true;                                                  // indicate that 10 milliseconds have elapsed
      }

      switch (sorterIndex) {
        case 0:
          ledcWrite(cServo2Channel, degreesToDutyCycle(90));
          digitalWrite(cTrigPin, HIGH);
          if (timeUp10milli) {
            digitalWrite(cTrigPin, LOW);
            duration = pulseIn(cEchoPin, HIGH);
            distance = (duration * 0.0343) / 2;
            if (distance < threshold) {
              ledcWrite(cServo1Channel, degreesToDutyCycle(45));
              timerCount100milli = 0;
              timerCount1sec = 0;
              timeUp100milli = false;
              timeUp1sec = false;
              sorterIndex++;
            } else {
              timeUp10milli = false;
              timerCount10milli = 0;
            }
          }
          break;
        case 1:
          if (timeUp100milli) {
            ledcWrite(cServo1Channel, degreesToDutyCycle(0));
          }

          if (timeUp1sec) {
            uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
            digitalWrite(cTCSLED, HIGH);                        // turn on onboard LED
            if (tcsFlag) {                                      // if colour sensor initialized
              tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values

              if () {
                ledcWrite(cServo2Channel, degreesToDutyCycle(0));
                timerCount500milli = 0;
                timeUp500milli = false;
                sorterIndex++; 
              } else if () {
                ledcWrite(cServo2Channel, degreesToDutyCycle(180));
                timerCount500milli = 0;
                timeUp500milli = false;
                sorterIndex++; 
              }
            }
          }
          break;
        case 2:
          if (timeUp500milli) {
            ledcWrite(cServo2Channel, degreesToDutyCycle(90));
            timerCount100milli = 0;
            timerCount10milli = 0;
            timeUp100milli = false;
            timeUp10milli = false;
            sorterIndex = 0;
          }
      }
  } 
}

long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}