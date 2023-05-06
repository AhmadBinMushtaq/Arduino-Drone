#ifndef STRUCTS_H
#define STRUCTS_H

struct ReceiverRawValues {
  volatile bool TransmitterCommunicationFailure;
  volatile int ChannelValues[8];
};
struct ReceiverCommands {
  bool Armed;
  bool Error;
  int Throttle;
  double YawAngleChange;
  double PitchAngle;
  double RollAngle;
  int Aux_1;
  int Aux_2;
};
struct ConstantsPID{
  double KP_roll_pitch;
  double KI_roll_pitch;
  double KD_roll_pitch;

  double KP_yaw;
  double KI_yaw;
  double KD_yaw;
};

struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};
struct IMU_Values {
  bool Error;
  bool NewDataAvailable;
  double DeltaTimeInSeconds;
  struct Orientation CurrentOrientation;
  struct Orientation PreviousOrientation;
};
struct MotorPowers {
  int frontLeftMotorPower;
  int frontRightMotorPower;
  int rearLeftMotorPower;
  int rearRightMotorPower;
};

#endif
