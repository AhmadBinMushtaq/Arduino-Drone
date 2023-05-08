#include <Arduino.h>
#include "Functions.h"
#include "Constants.h"
#include "Structs.h"

void setup() {

  initializeMotors();
  initializeOutputSignals();
  initializeIMU();
  initializeReceiver();
  InitializePIDConstants();

  Serial.begin(9600);
//  delay(5000);
    
}

void loop() {

  syncOutputSignals();
  
  struct ReceiverCommands commands = GetReceiverCommands();

  ////  Serial.print("Roll: ");
//  Serial.print(commands.RollAngle);
//  Serial.print(",");
//  Serial.print("\t");
////  Serial.print("Pitch: ");
  Serial.print(commands.PitchAngle);
  Serial.print(",");
////  Serial.print("Throttle: ");
//  Serial.print(commands.Throttle);
//  Serial.print("\t");
////  Serial.print("del_Yaw: ");
//  Serial.print(commands.YawAngleChange);
//  Serial.print("\t");
//  Serial.print("Aux 1: ");
//  Serial.print(commands.Aux_1);
//  Serial.print("\t");
////  Serial.print("Aux 2: ");
//  Serial.print(commands.Aux_2);
//  Serial.print("\t");
////  Serial.print("Error: ");
//  Serial.print(commands.Error);
//  Serial.print("\t");
////  Serial.print("Armed: ");
//  Serial.println(commands.Armed);
  
  struct IMU_Values imu_values = GetIMUvalues();
//
//  Serial.print("Pitch: ");
  Serial.print(imu_values.CurrentOrientation.PitchAngle);
  Serial.print(",");
////  Serial.print("Roll: ");
//  Serial.print(imu_values.CurrentOrientation.RollAngle);
//  Serial.print(",");
////  Serial.print("Yaw: ");
//  Serial.print(imu_values.CurrentOrientation.YawAngle);
//  Serial.print(",");
////  Serial.print("Error: ");
//  Serial.print(imu_values.Error);
//  Serial.print(",");
////  Serial.print("New Data: ");
//  Serial.print(imu_values.NewDataAvailable);
//  Serial.print(",");
////  Serial.print("del_T: ");
//  Serial.println(imu_values.DeltaTimeInSeconds);



  tunePID(commands);
//
  struct ConstantsPID pid = getPIDValues();

//  Serial.print("Roll, Pitch values: P: ");
  Serial.print(pid.KP_roll_pitch);
  Serial.print(",");
//  Serial.print("I: ");
  Serial.print(pid.KI_roll_pitch);
  Serial.print(",");
//  Serial.print("D: ");
  Serial.println(pid.KD_roll_pitch);
  

  if (commands.Error || commands.Throttle < THROTTLE_START_POINT || !commands.Armed || imu_values.Error)
  {
    stopMotors();
    resetPidVariables();
    return;
  }

  if (imu_values.NewDataAvailable) {
    struct MotorPowers motorPowers = calculateMotorPowers(commands, imu_values);
    spinMotors(motorPowers);
  }
}
