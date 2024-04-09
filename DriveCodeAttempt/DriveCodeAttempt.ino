// MSE 2202 Lab003-Team 7 - Zachary, Shady, Abdullah
//#define DEBUG_DRIVE_SPEED    1 // Uncomment to enable drive speed debugging via serial output
//#define DEBUG_ENCODER_COUNT  1 // Uncomment to enable encoder count debugging via serial output

#include <Arduino.h> // Include for basic Arduino functions
#include <Adafruit_NeoPixel.h> // Include for controlling Adafruit NeoPixel LEDs
#include <MSE2202_Lib.h> // Include custom library for specific motor and encoder functions

// Motor and Encoder GPIO pin definitions
#define LEFT_MOTOR_A        35 // GPIO pin for left motor input A
#define LEFT_MOTOR_B        36 // GPIO pin for left motor input B
#define RIGHT_MOTOR_A       37 // GPIO pin for right motor input A
#define RIGHT_MOTOR_B       38 // GPIO pin for right motor input B
#define ENCODER_LEFT_A      15 // GPIO pin for left encoder input A
#define ENCODER_LEFT_B      16 // GPIO pin for left encoder input B
#define ENCODER_RIGHT_A     11 // GPIO pin for right encoder input A
#define ENCODER_RIGHT_B     12 // GPIO pin for right encoder input B
#define MODE_BUTTON         0  // GPIO pin for the mode selection button
#define MOTOR_ENABLE_SWITCH 3  // GPIO pin for motor enable switch
#define POT_R1              1  // GPIO pin for potentiometer input for speed control
#define SMART_LED           21 // GPIO pin for controlling smart LED
#define SMART_LED_COUNT     1  // Number of smart LEDs being used
#define IR_DETECTOR         14 // GPIO pin for IR Detector

// Constants for motor speed control
const int cMinPWM = 150;    // Minimum PWM value to ensure motor movement
const int cMaxPWM = 255;    // Maximum PWM value for fastest motor speed
const int cLeftAdjust = 0;  // Adjustment value for left motor speed, used for fine tuning
const int cRightAdjust = 0; // Adjustment value for right motor speed, used for fine tuning

//movement
const float wheelRadius = 0.018; // Meters
const int cCountsRev = 1096; // Encoder counts per revolution
const float distancePerCount = (2 * PI * wheelRadius) / cCountsRev; // Meters per encoder count
const float baseStationDetectionThreshold = 0.005; // Meters (5 mm)

// Global variables
boolean motorsEnabled = true;         // Flag to track motor enabled state
boolean timeUp3sec = false;           // Flag for 3-second timer
boolean timeUp2sec = false;           // Flag for 2-second timer
boolean timeUp200msec = false;        // Flag for 200-millisecond timer
unsigned char leftDriveSpeed;         // Variable to store left motor drive speed
unsigned char rightDriveSpeed;        // Variable to store right motor drive speed
unsigned char driveIndex;             // Index to control current drive action
unsigned int  robotModeIndex = 0;     // Index to control robot operational mode
unsigned int modePBDebounce;          // Counter for mode button debounce logic
unsigned long timerCount3sec = 0;     // Counter for 3-second timer
unsigned long timerCount2sec = 0;     // Counter for 2-second timer
unsigned long timerCount200msec = 0;  // Counter for 200-millisecond timer
unsigned long previousMicros;         // Variable to store the last microsecond timestamp
unsigned long currentMicros;          // Variable to store the current microsecond timestamp
int duty = 5;
int locIndex = 12;

float DriveAmnt;
float DriveAmn;
float RotateAmnt;

double threemeter = 3/distancePerCount; 
IR Scan = IR(); 

Motion Bot = Motion();                // Instance of Motion class for motor control
Encoders LeftEncoder = Encoders();    // Instance of Encoders class for left encoder data
Encoders RightEncoder = Encoders();   // Instance of Encoders class for right encoder data

void setup() {
  // Initialize serial communication for debugging if enabled
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200); // Start serial communication at 115200 baud rate
#endif
   
  // Set up motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // Initialize motor control with specified pins
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning ); // Initialize left encoder with specified pins
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning ); // Initialize right encoder with specified pins

  Scan.Begin(IR_DETECTOR, 1200);

  // Configure input pins with internal pull-up resistors
  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP); // Configure motor enable switch as input with internal pull-up
  pinMode(MODE_BUTTON, INPUT_PULLUP);         // Configure mode selection button as input with internal pull-up
  modePBDebounce = 0;                         // Initialize debounce counter for mode button


  //set up ledc
  ledcSetup(1,50,8); 
  ledcAttachPin(41,1);
}
void loop() {
  long pos[] = {0, 0};                // Array to store current motor positions, not used in this snippet
  int pot = 0;                        // Variable to store raw ADC value from potentiometer

  currentMicros = micros();           // Update current time in microseconds

  // Check if 1 ms has elapsed to perform timing operations
  if ((currentMicros - previousMicros) >= 1000) {
    previousMicros = currentMicros;   // Update the last microsecond timestamp for next iteration

    // Timer logic for 3 seconds
    timerCount3sec += 1;              // Increment 3-second timer
    if (timerCount3sec > 3000) {      // Check if 3 seconds have elapsed
      timerCount3sec = 0;             // Reset 3-second timer
      timeUp3sec = true;              // Set flag indicating 3 seconds have elapsed
    }

    // Timer logic for 2 seconds
    timerCount2sec += 1;              // Increment 2-second timer
    if (timerCount2sec > 2000) {      // Check if 2 seconds have elapsed
      timerCount2sec = 0;             // Reset 2-second timer
      timeUp2sec = true;              // Set flag indicating 2 seconds have elapsed
    }

    // Timer logic for 200 milliseconds
    timerCount200msec += 1;           // Increment 200 millisecond timer
    if (timerCount200msec > 200) {    // Check if 200 milliseconds have elapsed
      timerCount200msec = 0;          // Reset 200 millisecond timer
      timeUp200msec = true;           // Set flag indicating 200 milliseconds have elapsed
    }

    // Mode pushbutton debounce logic
    if (!digitalRead(MODE_BUTTON)) {  // If mode button is pressed
      if (modePBDebounce <= 25) {     // Debounce time of 25 ms not yet passed
        modePBDebounce += 1;          // Increment debounce counter
        if (modePBDebounce > 25) {    // Once debounce time has passed
          modePBDebounce = 1000;      // Set counter to large number to wait for button release
        }
      }
    } else {                          // If mode button is released
      if (modePBDebounce > 25) {      // If button was pressed long enough for debounce
        modePBDebounce = 0;           // Reset debounce counter
        robotModeIndex++;             // Change robot mode
        robotModeIndex &= 1;          // Ensure mode index alternates between 0 and 1
        // Reset timers and flags associated with mode change
        timerCount3sec = 0;           
        timeUp3sec = false;           
      }
    }

    // Check motor enable switch and update motor enabled state
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH); // Motors are enabled if switch is ON (connected to ground)
/*
    // Robot operational mode handling
    switch (robotModeIndex) {
      case 0: // Robot stopped mode
        Bot.Stop("D1");                 // Stop both motors
        LeftEncoder.clearEncoder();     // Clear left encoder counts
        RightEncoder.clearEncoder();    // Clear right encoder counts
        driveIndex = 0;                 // Reset drive index for next operation
        timeUp2sec = false;             // Reset 2-second timer flag
        break;

      case 1: // Robot run mode
        if (timeUp3sec) {               // Check if 3-second wait time has elapsed before executing run mode
          timeUp3sec = false;           // Reset 3-second timer flag

          // Read potentiometer to set motor speeds
          pot = analogRead(POT_R1);     // Read potentiometer value
          leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust; // Map pot value to PWM range for left motor
          rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust; // Map pot value to PWM range for right motor

          // Debugging output for drive speeds
#ifdef DEBUG_DRIVE_SPEED
          Serial.print("Left Drive Speed: Pot R1 = ");
          Serial.print(pot);
          Serial.print(", mapped = ");
          Serial.println(leftDriveSpeed);
#endif

          // Debugging output for encoder counts, updated every 200 ms

#ifdef DEBUG_ENCODER_COUNT
          if (timeUp200msec) {
            timeUp200msec = false; // Reset the 200 ms timer flag
            // Fetch and print encoder counts for left and right encoders
            LeftEncoder.getEncoderRawCount();                            
            RightEncoder.getEncoderRawCount();                           
            Serial.print("Left Encoder count = ");
            Serial.print(LeftEncoder.lRawEncoderCount);
            Serial.print("  Right Encoder count = ");
            Serial.println(RightEncoder.lRawEncoderCount);
          }
#endif

          // Motor control logic based on robot operational mode
          if (motorsEnabled) { // Check if motor enable switch is ON
            // Drive logic based on current drive index (state)*/
            switch(driveIndex) {
              /*case 0: // Drive forward state
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > 5000) { // Check if right encoder count exceeds threshold
                  driveIndex++; // Move to next state (turn left)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 1: // Turn left state
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  driveIndex++; // Move to next state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 2: // Drive forward state (repeated)
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward again
                if (RightEncoder.lRawEncoderCount > 5000) { // Check if right encoder count exceeds threshold again
                  driveIndex++; // Move to next state (turn right)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 3: // Turn right state
                Bot.Right("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn right
                if (LeftEncoder.lRawEncoderCount > 1800) { // Check if left encoder count exceeds threshold
                  driveIndex = 0; // Reset to first state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder(); 
                }
                break; */
              case 0:
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > threemeter ) { // Check if right encoder count exceeds threshold
                  driveIndex++; // Move to next state (turn left)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 1:
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  driveIndex++; // Move to next state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 2:
                //drive back
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > threemeter ) { // Check if right encoder count exceeds threshold
                  driveIndex++; // Move to next state (turn left)
                  if(locIndex= 12){driveIndex=driveIndex+2;}
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                break;

              case 3:
                //potentially relocate with IR or else skip to case 6
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (Scan.Available()){
                  driveIndex++;
                  break;
                }
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  driveIndex++; // Move to next state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }

                break;

              case 4:
                //rotate the angle needed and drive distance needed and rotate towards sorter

                 RotateAmnt = (1900/360)/90;/*US-angle*/ 
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < RotateAmnt) { // Check if right encoder count goes below negative threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                } 
                 DriveAmnt = /*US-Xvalue*/2/distancePerCount;
                Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > DriveAmnt ) { // Check if right encoder count exceeds threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                driveIndex++;
                break;

                case 5:
                {
                //turn around again
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  driveIndex++; // Move to next state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                }
                break;

               case 6:
               {
                //turn to right or left depending on index
                if (locIndex >= 7){
                  //turn right
                  Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                }
                else {
                  //turn left
                  Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                }
                locIndex--;
                driveIndex++;
               }
                break;

               case 7:
               {
                //drive depending on index and length of test area
                if(locIndex>=7){
                 DriveAmn = (locIndex-6)*0.25; // convert into encoder
                  Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > 5000 ) { // Check if right encoder count exceeds threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                }
                else {
                  int index2 = map(locIndex, 1, 6, 6, 1);
                  DriveAmn = (index2)*0.25; //convert to encoder
                  Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to drive forward
                if (RightEncoder.lRawEncoderCount > 5000 ) { // Check if right encoder count exceeds threshold
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
                }
                driveIndex++;
               }
                break;

               case 8:
               {
                //turn back
                Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // Command motors to turn left
                if (RightEncoder.lRawEncoderCount < -1900) { // Check if right encoder count goes below negative threshold
                  driveIndex++; // Move to next state (drive forward)
                  // Clear encoder counts for next movement
                  LeftEncoder.clearEncoder();                          
                  RightEncoder.clearEncoder();
                }
               }
                break;

               case 9:
               {
                //back to case 0
                driveIndex=0;
               }
                break;
            }
          } else {
            // If motors are disabled (e.g., via switch), stop all motor activity
            Bot.Stop("D1");  
          }
        }
        // End of motor control logic
      
      // End of operational mode handling
    
  // End of timing and control loop

// End of main program loop