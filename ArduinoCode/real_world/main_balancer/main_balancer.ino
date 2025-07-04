// necessary libraries for MPU6050 and motor control amd to setup I2C
#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

// MPU6050 setup
MPU6050 mpu(Wire); // Create MPU6050 object with I2C connection

// PID variables
float Kp = 76.0, Ki = 25.0, Kd = 0.8; // Tuned Proportional(Kp), Integral(Ki), Derivative(Kd) gains
float targetAngle = 0.0;             // Desired upright angle (setpoint) in degrees
float error, previousError = 0;      // Current and previous errors
float errorSum = 0;                  // Integral term
float errorRate;                     // Derivative term
float PID_output;                    // PID output

// Motor control pins
const int motorA1 = 9;  //Pin to control motor A direction in forward direction
const int motorA2 = 4; //Pin to control motor A direction in backward direction
const int motorB1 = A3; //Pin to control motor B direction in forward direction
const int motorB2 = A2; //Pin to control motor B direction in backward direction
const int EN1 = 6; // PWM speed control pin for motor A
const int EN2 = 5; // PWM speed control pin for motor B

// Time variables
unsigned long previousTime, currentTime;
float elapsedTime; // Time between PID updates

void setup() {
  Serial.begin(9600); // Initialize serial for debugging
  Wire.begin();       // Start I2C communication

  // Initialize MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("MPU6050 initialization failed!"); //Prints the message to warn that MPU6050 could not be initialized properly
    while (1);
  }
  mpu.calcOffsets(); // For Calibrating MPU6050 sensor
  Serial.println("MPU6050 ready!"); //Prints the message to indicate that MPU6050 is ready for use

  // Initialize motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  previousTime = millis(); // Start time tracking
}

void loop() {
  // Update MPU6050 readings
  mpu.update();
  float angle = mpu.getAngleY(); // Get tilt angle about axis parallel to axis of wheels, in this case Y-axis

  // Calculate PID terms
  error = targetAngle - angle;  // Calculate error term for angle tilt
  errorSum = constrain(errorSum + error, -50, 50); // Integral term, constrained to prevent integral windup
  errorRate = (error - previousError) / elapsedTime; // Derivative term

  PID_output = (Kp * error) + (Ki * errorSum) + (Kd * errorRate); // PID calculation
  PID_output = constrain(PID_output, -200, 200); // Constrain PID output to safe, useful values

  // Adjust motor speeds based on PID output
  int motorSpeed = abs(PID_output);
  motorSpeed = constrain(motorSpeed, 0, 250); // Limit motor speed to prevent damage due to high speed
  //Calling required function for balancing
  if (PID_output < 0) {
    moveForward(motorSpeed); // Move forward if PID > 0
  } else if (PID_output > 0) {
    moveBackward(motorSpeed); // Move backward if PID < 0
  } else {
    stopMotors(); // Stop if PID output is zero
  }

  previousError = error; // Store current error for next loop
  elapsedTime = (millis() - previousTime) / 1000.0; // Calculate elapsed time in seconds
  previousTime = millis(); // Update previous time

  // Debugging information
  Serial.print("Angle: "); Serial.print(angle); //Prints angle tilt value
  Serial.print(" | PID Output: "); Serial.println(PID_output); //Prints calculated PID value
  delay(10); // Short delay for loop
}

//Motor Control Functions: moveForward(), moveBackward(), stopMotors()

// Function: moveForward
//Input: speed <int> - Speed value (0 to 250) used to control motor speed in forward direction
//Output: None
//Logic: Used by the robot for balancing itself when tilted in forward direction
//Example Call: moveForward(155)
void moveForward(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

// Function: moveBackward
//Input: speed <int> - Speed value (0 to 250) used to control motor speed in backward direction
//Output: None
//Logic: Used by the robot for balancing itself when tilted in backward direction
//Example Call: moveBackward(155)
void moveBackward(int speed) {
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

// Function: stopMotors
//Input: None
//Output: None
//Logic: Used to stop motors when robot has achieved equilibrium point
//Example Call: stopMotors()
void stopMotors() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
}