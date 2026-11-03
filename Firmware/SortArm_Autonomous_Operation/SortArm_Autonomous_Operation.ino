/*
 * SortArm-6DOF: Autonomous Sorting Cell Firmware
 * Course: Robotics (EEP 433) - Faculty of Engineering, Alexandria University
 * Author: Abd El-Rhman Muhammad Saad (ID: 19017058)
 * 
 * System Architecture:
 *   - IR Sensor: Digital Pin 2 (Active LOW on colored/black box detection)
 *   - Conveyor DC Motor Driver: Digital Pin 7 (NPN base switching)
 *   - Status Buzzer: Digital Pin 8 (Audible alert on detection)
 *   - 6-DOF Servos:
 *       servo_1 (Base Yaw):       Pin 3
 *       servo_2 (Shoulder Pitch): Pin 5
 *       servo_3 (Elbow Pitch):    Pin 6
 *       servo_4 (Wrist Pitch):    Pin 9
 *       servo_5 (Wrist Roll):     Pin 10
 *       servo_6 (Gripper):        Pin 11
 */

#include <Servo.h>

// Pin Definitions
const int irPin       = 2;
const int conveyorPin = 7;
const int buzzerPin   = 8;

// Servo Objects
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;

// Motion Pose Functions
void homePosition(int delay_time) {
  servo_1.write(90);
  servo_2.write(105);
  servo_3.write(120);
  servo_4.write(30);
  servo_5.write(90);
  servo_6.write(120);
  delay(delay_time);
}

void pickPosition_1(int delay_time) {
  // Pre-pick trajectory positioning
  servo_1.write(80);
  servo_2.write(140);
  servo_3.write(110);
  servo_4.write(80);
  servo_5.write(160);
  servo_6.write(110); // Gripper open
  delay(delay_time);
}

void pickPosition_2(int delay_time) {
  // Descent onto conveyor payload
  servo_1.write(80);
  servo_2.write(110);
  servo_3.write(110);
  servo_4.write(80);
  servo_5.write(160);
  servo_6.write(110);
  delay(delay_time);
}

void gripPosition(int delay_time) {
  // Securely close end-effector jaws around box
  servo_1.write(80);
  servo_2.write(110);
  servo_3.write(110);
  servo_4.write(80);
  servo_5.write(160);
  servo_6.write(150); // Gripper clamped
  delay(delay_time);
}

void releasePosition(int delay_time) {
  // Rotate base to sorting bin location
  servo_1.write(150);
  servo_2.write(110);
  servo_3.write(110);
  servo_4.write(80);
  servo_5.write(160);
  servo_6.write(150);
  delay(delay_time);
}

void ungripPosition(int delay_time) {
  // Release payload into destination bin
  servo_1.write(150);
  servo_2.write(110);
  servo_3.write(110);
  servo_4.write(80);
  servo_5.write(160);
  servo_6.write(110); // Gripper open
  delay(delay_time);
}

void setup() {
  Serial.begin(9600);
  
  pinMode(irPin, INPUT);
  pinMode(conveyorPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  // Attach all six joint servos
  servo_1.attach(3);
  servo_2.attach(5);
  servo_3.attach(6);
  servo_4.attach(9);
  servo_5.attach(10);
  servo_6.attach(11);
  
  Serial.println(F("===================================================="));
  Serial.println(F("SortArm-6DOF: Autonomous Sorting Cell Initialized"));
  Serial.println(F("Monitoring IR Sensor at Conveyor Checkpoint..."));
  Serial.println(F("===================================================="));
  
  // Move to home pose and start conveyor
  homePosition(1500);
  digitalWrite(conveyorPin, HIGH);
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  int irVal = digitalRead(irPin);
  
  // Object Detection Trigger (Active LOW for reflective threshold)
  if (irVal == LOW) {
    Serial.println(F("[EVENT] Target Box Detected! Halting Conveyor..."));
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(conveyorPin, LOW);  // Stop conveyor belt immediately
    delay(200);
    digitalWrite(buzzerPin, LOW);
    
    // Execute deterministic 6-DOF pick-and-place sequence
    Serial.println(F("[STATE] Moving to Pre-Pick Pose (pickPosition_1)"));
    pickPosition_1(1500);
    
    Serial.println(F("[STATE] Descending to Payload (pickPosition_2)"));
    pickPosition_2(1500);
    
    Serial.println(F("[STATE] Actuating Gripper (gripPosition)"));
    gripPosition(1500);
    
    Serial.println(F("[STATE] Transferring to Sorting Bin (releasePosition)"));
    releasePosition(1500);
    
    Serial.println(F("[STATE] Releasing Payload (ungripPosition)"));
    ungripPosition(1500);
    
    Serial.println(F("[STATE] Returning to Neutral Pose (homePosition)"));
    homePosition(1500);
    
    Serial.println(F("[EVENT] Sorting Cycle Complete. Resuming Conveyor..."));
    digitalWrite(conveyorPin, HIGH); // Restart conveyor
    delay(1000);                     // Debounce delay to prevent re-triggering on exit
  } else {
    // Normal running state
    digitalWrite(conveyorPin, HIGH);
    homePosition(100);
  }
}
