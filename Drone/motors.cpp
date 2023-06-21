#include <Arduino.h>
#include "Functions.h"
#include "Constants.h"
#include "Structs.h"
#include "Servo.h"

// Define motor objects
Servo frontLeftMotor;
Servo frontRightMotor;
Servo rearLeftMotor;
Servo rearRightMotor;

// Initialize motors
void initializeMotors() {
  // Attach motor pins and set pulse width limits
  frontLeftMotor.attach(FRONT_LEFT_MOTOR_PIN, MIN_MOTOR_PULSE_WIDTH, MAX_MOTOR_PULSE_WIDTH);
  frontRightMotor.attach(FRONT_RIGHT_MOTOR_PIN, MIN_MOTOR_PULSE_WIDTH, MAX_MOTOR_PULSE_WIDTH);
  rearLeftMotor.attach(REAR_LEFT_MOTOR_PIN, MIN_MOTOR_PULSE_WIDTH, MAX_MOTOR_PULSE_WIDTH);
  rearRightMotor.attach(REAR_RIGHT_MOTOR_PIN, MIN_MOTOR_PULSE_WIDTH, MAX_MOTOR_PULSE_WIDTH);

  // Stop motors
  stopMotors();
}

// Spin motors based on motor powers
void spinMotors(struct MotorPowers motorPowers) {
  frontLeftMotor.write(motorPowers.frontLeftMotorPower);
  frontRightMotor.write(motorPowers.frontRightMotorPower);
  rearLeftMotor.write(motorPowers.rearLeftMotorPower);
  rearRightMotor.write(motorPowers.rearRightMotorPower);
}

// Stop motors by setting motor powers to 0
void stopMotors() {
  frontLeftMotor.write(0);
  frontRightMotor.write(0);
  rearLeftMotor.write(0);
  rearRightMotor.write(0);
}
