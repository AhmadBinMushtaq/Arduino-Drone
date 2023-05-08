#include<Arduino.h>
#include"Structs.h"
#include"Constants.h"
#include"Functions.h"

//----------- PID CONFIGURATION-----------

struct ConstantsPID pid;



//----------- PID CONTROL LIMITS-----------
double ROLL_PITCH_CONTROL_SIGNAL_LIMIT = pid.KP_roll_pitch * QUADCOPTER_MAX_TILT_ANGLE * 2;

double roll_pid_i, roll_last_error, pitch_pid_i, pitch_last_error, yaw_pid_i, yaw_last_error;
double roll_control_signal, pitch_control_signal, yaw_control_signal;

void InitializePIDConstants(){
  pid.KP_roll_pitch = 0.28;
  pid.KI_roll_pitch = 0.48;
  pid.KD_roll_pitch = 0.20;

  pid.KP_yaw = 0.05;
  pid.KI_yaw = 0.06;
  pid.KD_yaw = 0.01;

  ROLL_PITCH_CONTROL_SIGNAL_LIMIT = pid.KP_roll_pitch * QUADCOPTER_MAX_TILT_ANGLE * 2;
  }

struct MotorPowers calculateMotorPowers(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values) {
  // calculate orientation errors (error: difference between desired orientation and actual orientation)
  double rollError = receiverCommands.RollAngle - imu_values.CurrentOrientation.RollAngle;
  double pitchError = receiverCommands.PitchAngle - imu_values.CurrentOrientation.PitchAngle;
  double yawError = calculateYawError(receiverCommands, imu_values);

  // calculate control gains based on errors
  roll_control_signal = getControlSignal(rollError, pid.KP_roll_pitch, pid.KI_roll_pitch, pid.KD_roll_pitch, roll_pid_i, roll_last_error, imu_values.DeltaTimeInSeconds);
  pitch_control_signal = getControlSignal(pitchError, pid.KP_roll_pitch, pid.KI_roll_pitch, pid.KD_roll_pitch, pitch_pid_i, pitch_last_error, imu_values.DeltaTimeInSeconds);
  yaw_control_signal = getControlSignal(yawError, pid.KP_yaw, pid.KI_yaw, pid.KD_yaw, yaw_pid_i, yaw_last_error, imu_values.DeltaTimeInSeconds);

  // limit roll-pitch control signals
  roll_control_signal = constrain(roll_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);
  pitch_control_signal = constrain(pitch_control_signal, -ROLL_PITCH_CONTROL_SIGNAL_LIMIT, ROLL_PITCH_CONTROL_SIGNAL_LIMIT);

  // calculate power for each motor
  struct MotorPowers motorPowers;
  motorPowers.frontLeftMotorPower = round(receiverCommands.Throttle + roll_control_signal - pitch_control_signal - yaw_control_signal);
  motorPowers.frontRightMotorPower = round(receiverCommands.Throttle - roll_control_signal - pitch_control_signal + yaw_control_signal);
  motorPowers.rearLeftMotorPower = round(receiverCommands.Throttle + roll_control_signal + pitch_control_signal + yaw_control_signal);
  motorPowers.rearRightMotorPower = round(receiverCommands.Throttle - roll_control_signal + pitch_control_signal - yaw_control_signal);

  motorPowers = reduceMotorPowers(motorPowers);

  return motorPowers;
}

double calculateYawError(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values) {
  double imuYawAngleChangeInDeltaTime = fix360degrees(imu_values.CurrentOrientation.YawAngle - imu_values.PreviousOrientation.YawAngle);
  double imuYawAngleChangePerSecond = imuYawAngleChangeInDeltaTime / imu_values.DeltaTimeInSeconds;
  double yawError = receiverCommands.YawAngleChange - imuYawAngleChangePerSecond;
  yawError = constrain(yawError, -QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND, QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND);
  return yawError;
}

struct MotorPowers reduceMotorPowers(MotorPowers motorPowers) { // to preserve balance if throttle limit exceeds the max value (180)
  int maxMotorPower = max(max(motorPowers.frontLeftMotorPower, motorPowers.frontRightMotorPower), max(motorPowers.rearLeftMotorPower, motorPowers.rearRightMotorPower));
  if (maxMotorPower > 180) {
    double power_reduction_rate = (double)maxMotorPower / (double)180;
    motorPowers.frontLeftMotorPower = round((double)motorPowers.frontLeftMotorPower / power_reduction_rate);
    motorPowers.frontRightMotorPower = round((double)motorPowers.frontRightMotorPower / power_reduction_rate);
    motorPowers.rearLeftMotorPower = round((double)motorPowers.rearLeftMotorPower / power_reduction_rate);
    motorPowers.rearRightMotorPower = round((double)motorPowers.rearRightMotorPower / power_reduction_rate);
  }
  return motorPowers;
}

void resetPidVariables() {
  roll_pid_i = 0;
  roll_last_error = 0;
  pitch_pid_i = 0;
  pitch_last_error = 0;
  yaw_pid_i = 0;
  yaw_last_error = 0;
}

double fix360degrees(double val) {
  if (val > 180) {
    return val - 360;
  } else if (val < -180) {
    return val + 360;
  } else {
    return val;
  }
}

void tunePID(struct ReceiverCommands cmd){
  if(cmd.Aux_1 < 40){
    pid.KP_roll_pitch = (double) cmd.Aux_2 /100;
    ROLL_PITCH_CONTROL_SIGNAL_LIMIT = pid.KP_roll_pitch * QUADCOPTER_MAX_TILT_ANGLE * 2;
    }
  else if (cmd.Aux_1 < 80){
    pid.KI_roll_pitch = (double) cmd.Aux_2 /100;
    }
  else{
    pid.KD_roll_pitch = (double) cmd.Aux_2 /100;
    }
  }

struct ConstantsPID getPIDValues(){
  return pid;
  }
