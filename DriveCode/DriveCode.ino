//#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use

const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = 255;                                                       // PWM value for maximum speed
const int cLeftAdjust = 0;                                                     // Amount to slow down left motor relative to right
const int cRightAdjust = 0;                                                    // Amount to slow down right motor relative to left

boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
boolean timeUp200msec = false;                                                 // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;                                                  // motor drive speed (0-255)
unsigned char rightDriveSpeed;                                                 // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int  robotModeIndex = 0;                                              // robot operational state
unsigned int modePBDebounce;                                                   // pushbutton debounce timer count
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;                                           // 200 millisecond timer count in milliseconds
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

Motion Bot = Motion();                                                         // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();                                             // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();                                            // Instance of Encoders for right encoder data

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif
   
  // Set up motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // set up right encoder

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
  modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop() {
  long pos[] = {0, 0};                                                         // current motor positions
  int pot = 0;                                                                 // raw ADC value from pot

  currentMicros = micros();                                                    // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
      previousMicros = currentMicros;                                          // record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
      if (timerCount3sec > 3000) {                                             // if 3 seconds have elapsed
        timerCount3sec = 0;                                                    // reset 3 second timer count
        timeUp3sec = true;                                                     // indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      timerCount2sec = timerCount2sec + 1;                                     // increment 2 second timer count
      if (timerCount2sec > 2000) {                                             // if 2 seconds have elapsed
         timerCount2sec = 0;                                                   // reset 2 second timer count
         timeUp2sec = true;                                                    // indicate that 2 seconds have elapsed
      }
   
      // 200 millisecond timer, counts 200 milliseconds
      timerCount200msec = timerCount200msec + 1;                               // Increment 200 millisecond timer count
      if(timerCount200msec > 200) {                                            // If 200 milliseconds have elapsed
         timerCount200msec = 0;                                                // Reset 200 millisecond timer count
         timeUp200msec = true;                                                 // Indicate that 200 milliseconds have elapsed
      }

      // Mode pushbutton debounce and toggle
      if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
        // Start debounce
        if (modePBDebounce <= 25) {                                           // 25 millisecond debounce time
          modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
          if (modePBDebounce > 25) {                                         // if held for at least 25 mS
            modePBDebounce = 1000;                                          // change debounce timer count to 1 second
          }
        }
        if (modePBDebounce >= 1000) {                                         // maintain 1 second timer count until release
          modePBDebounce = 1000;
        }
      }
      else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
        if(modePBDebounce <= 26) {                                            // if release occurs within debounce interval
          modePBDebounce = 0;                                                // reset debounce timer count
        }
        else {
          modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
          if(modePBDebounce >= 1025) {                                       // if pushbutton was released for 25 mS
            modePBDebounce = 0;                                             // reset debounce timer count
            robotModeIndex++;                                               // switch to next mode
            robotModeIndex = robotModeIndex & 1;                            // keep mode index between 0 and 1
            timerCount3sec = 0;                                             // reset 3 second timer count
            timeUp3sec = false;                                             // reset 3 second timer
          }
        }
      }

      // check if drive motors should be powered
      motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled

      switch(robotModeIndex) {
        case 0: // Robot stopped
          Bot.Stop("D1");    
          LeftEncoder.clearEncoder();                                        // clear encoder counts
          RightEncoder.clearEncoder();
          driveIndex = 0;                                                    // reset drive index
          timeUp2sec = false;                                                // reset 2 second timer
          break;

        case 1: // Run robot
          if (timeUp3sec) {                                                  // pause for 3 sec before running case 1 code
            // Read pot to update drive motor speed
            pot = analogRead(POT_R1);
            leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
            rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;
#ifdef DEBUG_DRIVE_SPEED 
            Serial.print(F(" Left Drive Speed: Pot R1 = "));
            Serial.print(pot);
            Serial.print(F(", mapped = "));
            Serial.println(leftDriveSpeed);
#endif
#ifdef DEBUG_ENCODER_COUNT
            if (timeUp200msec) {
              timeUp200msec = false;                                       // reset 200 ms timer
              LeftEncoder.getEncoderRawCount();                            // read left encoder count 
              RightEncoder.getEncoderRawCount();                           // read right encoder count
              Serial.print(F("Left Encoder count = "));
              Serial.print(LeftEncoder.lRawEncoderCount);
              Serial.print(F("  Right Encoder count = "));
              Serial.print(RightEncoder.lRawEncoderCount);
              Serial.print("\n");
            }
#endif
            if (motorsEnabled) {                                            // run motors only if enabled
              switch(driveIndex) {                                      // cycle through drive states
                case 0: // Drive forward
                  Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                  if (RightEncoder.lRawEncoderCount > 5000) {
                    driveIndex++;                                       // next state: turn left
                  }
                  break;

                case 1: // Turn left
                  Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                  if (RightEncoder.lRawEncoderCount > 5000) {
                    driveIndex++;                                       // next state: drive forward
                  }
                  break;

                case 2: // Drive forward
                  Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed
                  if (RightEncoder.lRawEncoderCount > 5000) {
                    driveIndex++;                                       // next state: turn right
                  }
                  break;

                case 3: // Turn right
                  Bot.Right("D1", leftDriveSpeed, rightDriveSpeed);    // drive ID, left speed, right speed
                  if (LeftEncoder.lRawEncoderCount > 5000) {
                    driveIndex = 0;                                       // next state: drive forward
                  }
                  break;
              }
            }
          }
          else {                                                             // stop when motors are disabled
            Bot.Stop("D1");  
          }
      }
    }
}
          
