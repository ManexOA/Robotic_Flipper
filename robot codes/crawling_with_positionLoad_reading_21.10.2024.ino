/* This is the code for controlling the Crawling Gait of the robotic flipper. 
The the code generates the movement of crawling and it also reads Position and 
Load data from the motors' sensors. [Last edited: 21/10/2024]*/


#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID1 = 1; 
const uint8_t DXL_ID2 = 2;
const uint8_t DXL_ID3 = 3;

const float DXL_PROTOCOL_VERSION = 1.0; // set to 1.0 if using AX-12A model

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

// Timing intervals
const unsigned long motorMoveInterval = 800; // Interval for each motor movement (in milliseconds)
const unsigned long dataReadInterval = 500;   // Interval for reading data (in milliseconds)

// Timing variables
unsigned long previousMotorMillis = 0;
unsigned long previousDataMillis = 0;
unsigned long currentMillis = 0;

unsigned long case16StartMillis = 0; // The delay variable for allowing the flipper to get to the initial position

// State variables
int step = 0;
bool moveMotor = true; // Flag to control motor movement
unsigned long cycleCounter = 0; // Counter for the cycles

// Define the pins for the LEDs (LEDs will be used as START and END reference times to match video and arduino program movements time series)
int led1 = 52;  // LED 1 (START) connected to pin 52 of the Dynamixel Shield board
int led2 = 53;  // LED 2 (END) connected to pin 53 of the Dynamixel Shield board

void setup() {
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);

  dxl.torqueOff(DXL_ID1);
  dxl.torqueOff(DXL_ID2);
  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  // Set the LED pins as outputs
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  // Set velocity value for the motors
  dxl.setGoalVelocity(DXL_ID1, 50); // 0-255 is the range of values for an 8-bit unsigned integer.
  dxl.setGoalVelocity(DXL_ID2, 50);
  dxl.setGoalVelocity(DXL_ID3, 50);

  dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 180, UNIT_DEGREE);
  delay(3000); // wait until all motors are in HOME position

  DEBUG_SERIAL.println("MOTORS IN HOME POSITION"); // Print message when motors are in Home position
  handleDataReading(); // Print initial Load and Position values.

  previousMotorMillis = millis(); // Start motor movement timing
}

void loop() {
  currentMillis = millis();

  // Call functions for cooperative multitasking
  handleMotorMovement();
  handleDataReading();
}

void handleMotorMovement() {
  if (moveMotor && currentMillis - previousMotorMillis >= motorMoveInterval) {
    previousMotorMillis = currentMillis;

    switch (step) {
      case 0:
        DEBUG_SERIAL.println("Flipper movement START");
        // Turn LED 2 off
        digitalWrite(led2, LOW);
        // Turn LED 1 on
        digitalWrite(led1, HIGH);
        dxl.setGoalPosition(DXL_ID1, 130, UNIT_DEGREE); // Move the flipper UP
        dxl.setGoalPosition(DXL_ID2, 170, UNIT_DEGREE); // Move the flipper FORWARD
        dxl.setGoalPosition(DXL_ID3, 120, UNIT_DEGREE); // Twist the flipper slightly to touch with flipper edge.
        step = 1;
        break;

      case 1:
        dxl.setGoalPosition(DXL_ID1, 130, UNIT_DEGREE); // Move the flipper UP
        dxl.setGoalPosition(DXL_ID2, 170, UNIT_DEGREE); // Move the flipper FORWARD
        step = 2;
        break;

      case 2:
        dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 170, UNIT_DEGREE); // Move the flipper FORWARD
        step = 3;
        break;

      case 3:
        dxl.setGoalPosition(DXL_ID1, 220, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 180, UNIT_DEGREE); // Move the flipper FORWARD
        step = 4;
        break;

      case 4:
        dxl.setGoalPosition(DXL_ID1, 270, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 210, UNIT_DEGREE); // Move the flipper FORWARD
        delay(2000); // Wait until flipper is dug into sand
        step = 5;
        break;

      case 5: // Flipper dug into sand. // SWEEP STROKE START
        // Turn LED 1 off
        digitalWrite(led1, LOW);
        DEBUG_SERIAL.println("SWEEP STROKE START");
        DEBUG_SERIAL.println("DATA MEASUREMENT [START]");
        DEBUG_SERIAL.print("Cycle Count: ");
        DEBUG_SERIAL.println(cycleCounter);
        dxl.setGoalPosition(DXL_ID1, 240, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 210, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 6;
        break;

      case 6: 
        dxl.setGoalPosition(DXL_ID1, 220, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 230, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 7;
        break;

      case 7:
        dxl.setGoalPosition(DXL_ID1, 200, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 270, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 8;
        break;

      case 8:
        dxl.setGoalPosition(DXL_ID1, 195, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 290, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 9;
        break;

      case 9: // SWEEP STROKE END
        dxl.setGoalPosition(DXL_ID1, 190, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 320, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 10;
        break;

      case 10: // Lift FLIPPER UP
        DEBUG_SERIAL.println("SWEEP STROKE END");
        DEBUG_SERIAL.println("DATA MEASUREMENT [FINISH]");
        DEBUG_SERIAL.println("LIFT FLIPPER UP");
        dxl.setGoalPosition(DXL_ID1, 150, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 330, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 11;
        break;

      case 11:
        dxl.setGoalPosition(DXL_ID1, 140, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 330, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 12;
        break;

      case 12:
        dxl.setGoalPosition(DXL_ID1, 130, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 320, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 13; 
        break;

      case 13:
        dxl.setGoalPosition(DXL_ID1, 100, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 310, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 14; 
        break;

      case 14:
        dxl.setGoalPosition(DXL_ID1, 80, UNIT_DEGREE); // Move the flipper DOWN
        dxl.setGoalPosition(DXL_ID2, 300, UNIT_DEGREE); // Move the flipper BACKWARDS
        step = 15; 
        break;

      case 15:
        dxl.setGoalPosition(DXL_ID1, 130, UNIT_DEGREE); // Move the flipper UP
        dxl.setGoalPosition(DXL_ID2, 200, UNIT_DEGREE); // Move the flipper FORWARD
        //delay(3000); // wait until it finishes the trayectory.
        DEBUG_SERIAL.println("Flipper movement END");

        // Turn LED 2 on
        digitalWrite(led2, HIGH);
        step = 16; 
        break;

       case 16: // Back to the START position
        if (case16StartMillis == 0) {
          DEBUG_SERIAL.println("Flipper moving to the initial position");

          // Set the goal positions
          dxl.setGoalPosition(DXL_ID1, 130, UNIT_DEGREE); // Move the flipper UP
          dxl.setGoalPosition(DXL_ID2, 170, UNIT_DEGREE); // Move the flipper FORWARD
          dxl.setGoalPosition(DXL_ID3, 120, UNIT_DEGREE); // Twist the flipper slightly to touch with flipper edge.

          // Record the start time
          case16StartMillis = currentMillis;
        }

        // Check if 3000 milliseconds have passed
        if (currentMillis - case16StartMillis >= 3000) {

          DEBUG_SERIAL.println("Flipper in the START position");
          // Increment cycle counter when one full cycle is completed
         
          cycleCounter++;
          DEBUG_SERIAL.print("Cycle Count: ");
          DEBUG_SERIAL.println(cycleCounter);

          // Reset step and timer
          step = 0;
          case16StartMillis = 0;
        }
        break;
    }
  }
}

void handleDataReading() {
  if (currentMillis - previousDataMillis >= dataReadInterval) {
    previousDataMillis = currentMillis;

    // Read LOAD data
    int load1 = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID1);
    int load2 = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID2);
    int load3 = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID3);

    // Print LOAD data for the Serial Plotter
    DEBUG_SERIAL.print("MOTOR1_load: ");
    DEBUG_SERIAL.println(load1);
    DEBUG_SERIAL.print("MOTOR2_load: ");
    DEBUG_SERIAL.println(load2);
    DEBUG_SERIAL.print("MOTOR3_load: ");
    DEBUG_SERIAL.println(load3);
    DEBUG_SERIAL.println(); // New line for better readability

    // Read and Print POSITION data for the Serial Plotter
    DEBUG_SERIAL.print("MOTOR1_Position(Z): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.print("MOTOR2_Position(X): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.print("MOTOR3_Position(Twist): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.println(); // New line for better readability
  }
}
