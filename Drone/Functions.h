#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Functions used in the drone project
// Each function is defined in a separate file

// Motor-related functions (MotorControl.ino)
void initializeMotors(); 
// Initializes the motors

void stopMotors(); 
// Stops the motors

void spinMotors(struct MotorPowers motorPowers); 
// Spins the motors based on the provided MotorPowers struct


// Output signals (OutputSignals.ino)
void initializeOutputSignals(); 
// Initializes the output signals

void syncOutputSignals(); 
// Synchronizes the output signals

void blink_led(); 
// Blinks the LED


// IMU functions (IMU.ino)
void initializeIMU(); 
// Initializes the IMU

struct IMU_Values GetIMUvalues(); 
// Gets IMU values


// Receiver functions (Receiver.ino)
void initializeReceiver(); 
// Initializes the receiver

void handle_interrupt(); 
// Handles receiver interrupt

struct ReceiverCommands GetReceiverCommands(); 
// Gets receiver commands

struct ReceiverCommands getFailureReceiverCommand(); 
// Gets receiver command for failure cases


// Arming and stick position checks (Arming.ino)
bool getArmStatus(); 
// Gets the arm status of the drone

bool isArming(); 
// Checks if the drone is being armed

bool isDisarming(); 
// Checks if the drone is being disarmed

bool isThrottleStickPositonAtFullDown(); 
// Checks if the throttle stick is at the full down position

bool isYawStickPositionAtFullLeft(); 
// Checks if the yaw stick is at the full left position

bool isYawStickPositionAtFullRight(); 
// Checks if the yaw stick is at the full right position


// Utility function (Utilities.ino)
double map_double(double x, double in_min, double in_max, double out_min, double out_max); 
// Maps a value from one range to another


// PID control and motor power calculation (PIDControl.ino)
void InitializePIDConstants(); 
// Initializes PID constants

struct MotorPowers calculateMotorPowers(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values); 
// Calculates motor powers based on receiver commands and IMU values

double calculateYawError(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values); 
// Calculates yaw error

struct MotorPowers reduceMotorPowers(MotorPowers motorPowers); 
// Reduces motor powers

void resetPidVariables(); 
// Resets PID variables

double fix360degrees(double val); 
// Fixes the angle value to a 0-360 range

struct ConstantsPID getPIDValues(); 
// Gets the current PID values

void tunePID(struct ReceiverCommands cmd); 
// Tunes the PID controller based on receiver commands

double getControlSignal(double error, double kp, double ki, double kd, double& pid_i, double& last_error, double delta_time_in_seconds); 
// Gets control signal from PID controller


// Armed and disarmed status (Arming.ino)
void quadcopter_armed(); 
// Sets the quadcopter to the armed state

void quadcopter_disarmed(); 
// Sets the quadcopter to the disarmed state

#endif
