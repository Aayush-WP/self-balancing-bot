#include <Wire.h>
#include <MPU6050_light.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <Servo.h>

// Create Servo objects
Servo servo1;
Servo servo2;

bool isServo1Moving = false;
bool isServo2Moving = false;
unsigned long servo1Timer = 0;
unsigned long servo2Timer = 0;
const unsigned long servoInterval = 30; // Time interval for servo updates
int targetServo1Angle = 90;
int targetServo2Angle = 45;


// Define SoftwareSerial pins for Bluetooth
SoftwareSerial bluetooth(A1, A0); // RX, TX

// MPU6050 setup
MPU6050 mpu(Wire);

// Tilt PID variables
float Kp_tilt = 50.0, Ki_tilt = 0.0, Kd_tilt = 0.0;
float targetAngle = 0.0;
float error_tilt, previousError_tilt = 0;
float errorSum_tilt = 0;
float errorRate_tilt;
float PID_output_tilt;

// Motor control pins
const int motorA1 = 9;
const int motorA2 = 4;
const int motorB1 = A3;
const int motorB2 = A2;
const int EN1 = 6;
const int EN2 = 5;

// Encoder setup
#define encodPinAR 2
#define encodPinBR 7
#define encodPinAL 3
#define encodPinBL 8

Encoder encoderLeft(encodPinAL, encodPinBL);
Encoder encoderRight(encodPinAR, encodPinBR);

// Time variables
unsigned long previousTime, currentTime;
float elapsedTime;
char command; // Initialize command to stop motors

//Servo variables
float Servo1_angle = 0;
float Servo2_angle = 0;

void setup() {
  Serial.begin(9600);      // Serial communication for debugging
  bluetooth.begin(9600);   // Start Bluetooth communication with HC-05

  Serial.println("Bluetooth is ready!");

  Wire.begin();

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.calcOffsets();
  Serial.println("MPU6050 ready!");

  // Initialize motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  // Attach the servos to the respective pins
  servo2.attach(10);
  servo1.attach(11);
  /*
  for (int angle = servo2.read(); angle <= 90; angle++) {
    servo1.write(angle);
    servo2.write(angle);
    delay(15); // Small delay for smooth movement
  }
  */
  servo1.write(90);
  delay(100);
  servo2.write(45);
  delay(100);
  Servo1_angle = servo1.read();
  Servo2_angle = servo2.read();
  servo1.detach();
  servo2.detach();
  previousTime = millis();
}

void loop() {
  // Read Bluetooth commands
  if (bluetooth.available()) {
    command = bluetooth.read();
    processCommand(command);
  }

  // Update MPU6050 readings
  mpu.update();
  float angle = mpu.getAngleY();

  // Tilt PID Calculation
  error_tilt = targetAngle - angle;
  errorSum_tilt += error_tilt; // Accumulate error for integral term
  errorRate_tilt = (error_tilt - previousError_tilt) / elapsedTime;
  PID_output_tilt = (Kp_tilt * error_tilt) + (Ki_tilt * errorSum_tilt) + (Kd_tilt * errorRate_tilt);
  PID_output_tilt = constrain(PID_output_tilt, -100, 100);
  previousError_tilt = error_tilt;

  // Adjust motor speeds based on tilt PID output
  int motorSpeed = abs(PID_output_tilt);
  motorSpeed = constrain(motorSpeed, 0, 255);

  if (PID_output_tilt < 0) {
    moveForward(motorSpeed);
  } else if (PID_output_tilt > 0) {
    moveBackward(motorSpeed);
  } else {
    stopMotors();
  }

 // Update Servo1
  if (isServo1Moving && millis() - servo1Timer >= servoInterval) {
    if (Servo1_angle != targetServo1Angle) {
      if (Servo1_angle < targetServo1Angle) {
        Servo1_angle++;
      } else if (Servo1_angle > targetServo1Angle) {
        Servo1_angle--;
      }
      servo1.write(Servo1_angle); // Move servo to new position
      servo1Timer = millis(); // Reset the timer
    } else {
      isServo1Moving = false; // Stop moving when target is reached
      servo1.detach(); // Detach to save power
    }
  }

  // Update Servo2
  if (isServo2Moving && millis() - servo2Timer >= servoInterval) {
    if (Servo2_angle != targetServo2Angle) {
      if (Servo2_angle < targetServo2Angle) {
        Servo2_angle++;
      } else if (Servo2_angle > targetServo2Angle) {
        Servo2_angle--;
      }
      servo2.write(Servo2_angle);
      servo2Timer = millis();
    } else {
      isServo2Moving = false;
      servo2.detach();
    }
  }

  // Timing
  currentTime = millis();
  elapsedTime = (currentTime - previousTime)/1000.0f; // Convert to seconds
  previousTime = currentTime;

  // Debugging information to Serial Monitor
  Serial.print("Angle: "); Serial.print(angle);
  //Serial.print(" | Tilt PID: "); Serial.println(PID_output_tilt);
  Serial.print("Servo1_angle: "); Serial.println(Servo1_angle);
  Serial.print("Servo2_angle: "); Serial.println(Servo2_angle);
  delay(30);  // Small delay for responsiveness
}

// Motor Control Functions
void moveForward(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void moveBackward(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void stopMotors() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}

// Bluetooth Command Processing
void processCommand(char command) {
  Serial.print("Processing Command: ");
  switch (command) {
    case 'F': // Forward
      moveForward(150);
      bluetooth.println("Moving Forward");
      break;
    case 'B': // Backward
      moveBackward(150);
      bluetooth.println("Moving Backward");
      break;
    case 'L': // Left rotation
      yawLeft(150);
      bluetooth.println("Yawing Left");
      break;
    case 'R': // Right rotation
      yawRight(150);
      bluetooth.println("Yawing Right");
      break;
    case 'P': // Stop
      stopMotors();
      bluetooth.println("Stopping Motors");
      break;
    case 'S': // Stop
      if (Servo2_angle == targetServo2Angle && Servo1_angle == targetServo1Angle){
      //Servo1_angle+=5;
      gripperOpen(Servo1_angle+5);
      bluetooth.println("Gripper_Opening");}
      break;
    case 'C': // Stop
      if (Servo2_angle == targetServo2Angle && Servo1_angle == targetServo1Angle){
      //Servo1_angle-=5;
      gripperClose(Servo1_angle-5);
      bluetooth.println("Gripper Closing");}
      break;
    case 'X': // Stop
      if (Servo2_angle == targetServo2Angle && Servo1_angle == targetServo1Angle){
      //Servo2_angle-=5;
      gripperDown(Servo2_angle-5);
      bluetooth.println("Gripper Down");}
      break;
    case 'T': // Stop
      if (Servo2_angle == targetServo2Angle && Servo1_angle == targetServo1Angle){
      //Servo2_angle+=5;
      gripperUp(Servo2_angle+5);
      bluetooth.println("Gripper Up");}
      break;
    default:
      bluetooth.println("Unknown command received!");
      break;
  }
  command = '\0'; // Reset the command after processing
}

// Add missing yaw functions
void yawLeft(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void yawRight(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}


void gripperOpen(int newAngle) {
  targetServo1Angle = constrain(newAngle, 50, 150);
  isServo1Moving = true;
  servo1.attach(11); // Attach the servo
  servo1Timer = millis(); // Start the timer
}

void gripperClose(int newAngle) {
  targetServo1Angle = constrain(newAngle, 50, 150);
  isServo1Moving = true;
  servo1.attach(11);
  servo1Timer = millis(); 
}

void gripperUp(int newAngle) {
  targetServo2Angle = constrain(newAngle, 20, 135);;
  isServo2Moving = true;
  servo2.attach(10);
  servo2Timer = millis();
}

void gripperDown(int newAngle) {
  targetServo2Angle = constrain(newAngle, 20, 135);
  isServo2Moving = true;
  servo2.attach(10);
  servo2Timer = millis();
}