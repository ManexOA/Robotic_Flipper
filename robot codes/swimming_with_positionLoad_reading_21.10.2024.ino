/* This code is for the locomotion control of a robotic flipper, 
mimicking the original swimming locomotion pattern of sea turtles.
The data of the animal's flipper motion during swimming was obtained from this paper: https://doi.org/10.1038/s41598-022-21459-y.
We filtered the data and formatted as we needed, for our robotic model.
The data we are using in this code, can be found in the text file included in the same 
directory of this code.*/


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

// Position DATA (from swimming geometry plot)
// Define the angle values for both motors
const float motor1_angles[29] = {
    248.9, 259.7, 265.2, 270.0, 269.3, 266.1, 251.8, 243.7, 
    220.0, 189.6, 162.2, 162.2, 147.2, 109.2, 98.9, 80.2, 
    80.0, 80.4, 90.5, 95.6, 108.4, 127.8, 142.8, 160.2, 
    179.3, 195.8, 210.3, 225.6, 239.0
};

const float motor2_angles[29] = {
    100.0, 100.1, 104.8, 119.7, 122.7, 133.5, 178.5, 199.5, 
    245.0, 258.3, 270.0, 270.0, 268.4, 255.2, 246.9, 229.3, 
    226.6, 224.5, 225.6, 228.7, 233.6, 232.6, 223.0, 206.3, 
    186.7, 168.2, 140.7, 118.8, 100.6
};

// Timing intervals
const unsigned long motorMoveInterval = 500; // Interval for each motor movement
const unsigned long dataReadInterval = 500;   // Interval for reading data

// Timing variables
unsigned long previousMotorMillis = 0;
unsigned long previousDataMillis = 0;
unsigned long currentMillis = 0;

// State variables
int step = 0;
bool moveMotor = true; // Flag to control motor movement
unsigned long cycleCounter = 0; // Counter for the cycles

// Function prototypes
float convertLoadToNm(int load);

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

  // Set velocity value for the motors
  dxl.setGoalVelocity(DXL_ID1, 50);
  dxl.setGoalVelocity(DXL_ID2, 50);
  dxl.setGoalVelocity(DXL_ID3, 50);

  dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 180, UNIT_DEGREE);
  delay(5000); // wait until all the motors are in Home position

  // Set initial positions of the motors
  dxl.setGoalPosition(DXL_ID1, motor1_angles[0], UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, motor2_angles[0], UNIT_DEGREE);
  delay(3000); //delay to reach the position

  DEBUG_SERIAL.println("MOTORS IN HOME POSITION"); // Print message when motors are in Home position
  
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

    // Execute motor movement steps based on the current step
    // Figure-of-eight locomotion using switch/case structure

    // Set motors' velocity for SWIMMING
    dxl.setGoalVelocity(DXL_ID1, 50);
    dxl.setGoalVelocity(DXL_ID2, 50);
    dxl.setGoalVelocity(DXL_ID3, 50);

    // Use switch statement to select the step
switch (step) {
    case 0:
        DEBUG_SERIAL.print("Cycle Count: ");
        DEBUG_SERIAL.println(cycleCounter);
        DEBUG_SERIAL.println("Flipper movement START");
        dxl.setGoalPosition(DXL_ID1, motor1_angles[0], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[0], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID3, 160, UNIT_DEGREE); // Twist the flipper slightly for thrust
        //delay(300); //delay to reach the position
        step = 1;
        break;
    case 1:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[1], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[1], UNIT_DEGREE);
        //delay(100); //delay to reach the position
        step = 2;
        break;
    case 2:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[2], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[2], UNIT_DEGREE);
        //delay(100); //delay to reach the position
        step = 3;
        break;
    case 3:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[3], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[3], UNIT_DEGREE);
        //delay(300); //delay to reach the position
        step = 4;
        break;
    case 4:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[4], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[4], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 5;
        break;
    case 5:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[5], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[5], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 6;
        break;
    case 6:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[6], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[6], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 7;
        break;
    case 7:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[7], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[7], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 8;
        break;
    case 8:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[8], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[8], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 9;
        break;
    case 9:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[9], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[9], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 10;
        break;
    case 10:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[10], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[10], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 11;
        break;
    case 11:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[11], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[11], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 12;
        break;
    case 12:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[12], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[12], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 13;
        break;
    case 13:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[13], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[13], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 14;
        break;
    case 14:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[14], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[14], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 15;
        break;
    case 15:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[15], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[15], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID3, 180, UNIT_DEGREE); // Twist the flipper slightly for thrust
        //delay(500); //delay to reach the position
        step = 16;
        break;
    case 16:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[16], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[16], UNIT_DEGREE);
        //delay(200); //delay to reach the position
        step = 17;
        break;
    case 17:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[17], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[17], UNIT_DEGREE);
        //delay(200); //delay to reach the position
        step = 18;
        break;
    case 18:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[18], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[18], UNIT_DEGREE);
        //delay(200); //delay to reach the position
        step = 19;
        break;
    case 19:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[19], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[19], UNIT_DEGREE);
        //delay(200); //delay to reach the position
        step = 20;
        break;
    case 20:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[20], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[20], UNIT_DEGREE);
        //delay(200); //delay to reach the position
        step = 21;
        break;
    case 21:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[21], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[21], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 22;
        break;
    case 22:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[22], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[22], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 23;
        break;
    case 23:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[23], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[23], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 24;
        break;
    case 24:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[24], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[24], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 25;
        break;
    case 25:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[25], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[25], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 26;
        break;
    case 26:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[26], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[26], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 27;
        break;
    case 27:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[27], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[27], UNIT_DEGREE);
        //delay(500); //delay to reach the position
        step = 28;
        break;
    case 28:
        dxl.setGoalPosition(DXL_ID1, motor1_angles[28], UNIT_DEGREE);
        dxl.setGoalPosition(DXL_ID2, motor2_angles[28], UNIT_DEGREE);

        delay(1000); // wait until it finishes the trayectory.
        DEBUG_SERIAL.println("Flipper movement END");

        // Increment cycle counter when one full cycle is completed
        cycleCounter++;
        DEBUG_SERIAL.print("Cycle Count: ");
        DEBUG_SERIAL.println(cycleCounter);
        step = 0; // Loop back to the start
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

     // Print data for the Serial Plotter
     DEBUG_SERIAL.print("Motor1_Load: ");
     DEBUG_SERIAL.println(load1);
     DEBUG_SERIAL.print(" ");
     DEBUG_SERIAL.print("Motor2_Load: ");
     DEBUG_SERIAL.print(load2);
     DEBUG_SERIAL.print(" ");
     DEBUG_SERIAL.print("Motor3_Load: ");
     DEBUG_SERIAL.println(load3);

    // Read and Print POSITION data for the Serial Plotter
    DEBUG_SERIAL.print("MOTOR1_Position (X): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.print("MOTOR2_Position (Z): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.print("MOTOR3_Position (twist angle): ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE)); //use angle in degree
    DEBUG_SERIAL.println(); // New line for better readability
   }
}

/* The motor's load values (raw data, bits) without any external force is as follows:

CW direction: 1023-2047 bits
CCW direction: 0-1023 bits
*/
