// Include required header files
#include <Arduino.h>
#include "Functions.h"
#include "Constants.h"
#include "Structs.h"

// Setup function, executed once at the beginning
void setup() {
  // Initialize motors, output signals, IMU, receiver, and PID constants
  initializeMotors();
  initializeOutputSignals();
  initializeIMU();
  initializeReceiver();
  InitializePIDConstants();

  // Begin Serial communication at 9600 baud
  Serial.begin(9600);
  // Uncomment the following line to add a 5-second delay before starting the main loop
  // delay(5000);
}

// Main loop function, executed repeatedly
void loop() {
  // Synchronize output signals
  syncOutputSignals();

  // Get commands from the receiver
  struct ReceiverCommands commands = GetReceiverCommands();

  // Get IMU values
  struct IMU_Values imu_values = GetIMUvalues();

  // Tune the PID controller using the received commands
  tunePID(commands);

  // Get PID values
  struct ConstantsPID pid = getPIDValues();

  // Check for command errors, throttle below start point, not armed, or IMU errors
  if (commands.Error || commands.Throttle < THROTTLE_START_POINT || !commands.Armed || imu_values.Error) {
    // If any of the above conditions are true, stop the motors and reset PID variables
    stopMotors();
    resetPidVariables();
    return;
  }

  // If new IMU data is available
  if (imu_values.NewDataAvailable) {
    // Calculate motor powers based on commands and IMU values
    struct MotorPowers motorPowers = calculateMotorPowers(commands, imu_values);

    // Spin motors with the calculated powers
    spinMotors(motorPowers);
  }
}
