/*
 * SortArm-6DOF: Interactive Joint Calibration & Serial Angle Sweep
 * Course: Robotics (EEP 433) - Faculty of Engineering, Alexandria University
 * Author: Abd El-Rhman Muhammad Saad (ID: 19017058)
 * 
 * Hardware Pinout:
 *   S1 (Base Yaw):       Pin 3  (PWM)
 *   S2 (Shoulder Pitch): Pin 5  (PWM)
 *   S3 (Elbow Pitch):    Pin 6  (PWM)
 *   S4 (Wrist Pitch):    Pin 9  (PWM)
 *   S5 (Wrist Roll):     Pin 10 (PWM)
 *   S6 (Gripper):        Pin 11 (PWM)
 * 
 * Serial Syntax:
 *   Enter joint commands over 9600 baud serial, e.g.:
 *   "A 90 B 120 C 60 D 50 E 90 F 120"
 */

#include <Servo.h>

// Servo objects for each kinematic joint
Servo servoA;  // S1: Base Yaw
Servo servoB;  // S2: Shoulder Pitch
Servo servoC;  // S3: Elbow Pitch
Servo servoD;  // S4: Wrist Pitch
Servo servoE;  // S5: Wrist Roll
Servo servoF;  // S6: End-Effector Gripper

// Current tracked joint angles
int angleA = 90;
int angleB = 90;
int angleC = 90;
int angleD = 90;
int angleE = 90;
int angleF = 90;

String inputString = "";
bool stringComplete = false;

// Smooth servo trajectory interpolation to minimize mechanical jerk & current spikes
void moveSmooth(Servo &servo, int targetAngle) {
  int currentAngle = servo.read();
  if (currentAngle < targetAngle) {
    for (int i = currentAngle; i <= targetAngle; i++) {
      servo.write(i);
      delay(10);
    }
  } else {
    for (int i = currentAngle; i >= targetAngle; i--) {
      servo.write(i);
      delay(10);
    }
  }
}

void processCommand(String command) {
  command.trim();
  command += " ";
  int index = 0;
  
  while (index < command.length()) {
    char servoChar = toupper(command.charAt(index));
    index++;
    
    while (index < command.length() && command.charAt(index) == ' ') index++;
    String angleStr = "";
    while (index < command.length() && isDigit(command.charAt(index))) {
      angleStr += command.charAt(index);
      index++;
    }
    
    if (angleStr.length() > 0) {
      int angle = angleStr.toInt();
      angle = constrain(angle, 0, 180);
      
      switch (servoChar) {
        case 'A':
          moveSmooth(servoA, angle);
          angleA = angle;
          Serial.print(F("[ACK] Joint S1 (Base) -> ")); Serial.println(angle);
          break;
        case 'B':
          moveSmooth(servoB, angle);
          angleB = angle;
          Serial.print(F("[ACK] Joint S2 (Shoulder) -> ")); Serial.println(angle);
          break;
        case 'C':
          moveSmooth(servoC, angle);
          angleC = angle;
          Serial.print(F("[ACK] Joint S3 (Elbow) -> ")); Serial.println(angle);
          break;
        case 'D':
          moveSmooth(servoD, angle);
          angleD = angle;
          Serial.print(F("[ACK] Joint S4 (Wrist Pitch) -> ")); Serial.println(angle);
          break;
        case 'E':
          moveSmooth(servoE, angle);
          angleE = angle;
          Serial.print(F("[ACK] Joint S5 (Wrist Roll) -> ")); Serial.println(angle);
          break;
        case 'F':
          moveSmooth(servoF, angle);
          angleF = angle;
          Serial.print(F("[ACK] Joint S6 (Gripper) -> ")); Serial.println(angle);
          break;
        default:
          Serial.print(F("[ERR] Invalid joint identifier: "));
          Serial.println(servoChar);
      }
    }
    while (index < command.length() && command.charAt(index) == ' ') index++;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  
  Serial.println(F("===================================================="));
  Serial.println(F("SortArm-6DOF: Interactive Joint Calibration Utility"));
  Serial.println(F("Enter commands like: A 90 B 120 C 60 D 50 E 90 F 120"));
  Serial.println(F("===================================================="));
  
  // Attach PWM pins
  servoA.attach(3);
  servoB.attach(5);
  servoC.attach(6);
  servoD.attach(9);
  servoE.attach(10);
  servoF.attach(11);
  
  // Initialize to neutral pose smoothly
  moveSmooth(servoA, angleA);
  moveSmooth(servoB, angleB);
  moveSmooth(servoC, angleC);
  moveSmooth(servoD, angleD);
  moveSmooth(servoE, angleE);
  moveSmooth(servoF, angleF);
  
  inputString.reserve(64);
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '
' || inChar == '') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
