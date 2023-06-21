#ifndef STRUCTS_H
#define STRUCTS_H

// Struct definitions used in the drone project

// Raw values received from the transmitter (Receiver.ino)
struct ReceiverRawValues {
  volatile bool TransmitterCommunicationFailure;
  volatile int ChannelValues[8];
};

// Parsed commands received from the transmitter (Receiver.ino)
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

// PID constants for roll, pitch, and yaw (PIDControl.ino)
struct ConstantsPID {
  double KP_roll_pitch;
  double KI_roll_pitch;
  double KD_roll_pitch;

  double KP_yaw;
  double KI_yaw;
  double KD_yaw;
};

// Orientation angles (IMU.ino)
struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};

// IMU values including current and previous orientation (IMU.ino)
struct IMU_Values {
  bool Error;
  bool NewDataAvailable;
  double DeltaTimeInSeconds;
  struct Orientation CurrentOrientation;
  struct Orientation PreviousOrientation;
};

// Motor powers for each motor (MotorControl.ino)
struct MotorPowers {
  int frontLeftMotorPower;
  int frontRightMotorPower;
  int rearLeftMotorPower;
  int rearRightMotorPower;
};

#endif
