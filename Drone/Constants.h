#ifndef CONSTANTS_H
#define CONSTANTS_H

// Constants and settings for the drone

// Flight parameters
// Average flight time is 13 minutes with 10x5 props, 3000 mAh 3S lipo battery

// Pin assignments
// ----------------
#define FRONT_LEFT_MOTOR_PIN 5
#define FRONT_RIGHT_MOTOR_PIN 6
#define REAR_LEFT_MOTOR_PIN 11
#define REAR_RIGHT_MOTOR_PIN 10
#define LED_PIN 12
#define BUZZER_PIN 8
#define IMU_INTERRUPT_PIN 3
#define RX_INTERRUPT_PIN 2

// IMU calibration values
// -----------------------
#define ACCEL_OFFSET_X 90
#define ACCEL_OFFSET_Y 1611
#define ACCEL_OFFSET_Z 1788
#define GYRO_OFFSET_X 220
#define GYRO_OFFSET_Y 76
#define GYRO_OFFSET_Z -86

// Receiver and transmitter settings
// ---------------------------------
#define TRANSMITTER_JOYSTICK_MIN_VALUE 600
#define TRANSMITTER_JOYSTICK_MAX_VALUE 1600
#define TRANSMITTER_ARMING_DURATION_IN_MILLISECONDS 2000
#define TRANSMITTER_ARMING_JOYSTICK_TOLERANCE 100

// ESC settings
// ------------
#define MIN_MOTOR_PULSE_WIDTH 1000
#define MAX_MOTOR_PULSE_WIDTH 2000

// Communication timeouts
// ----------------------
#define IMU_COMMUNICATION_TIMEOUT_IN_MILLISECONDS 100
#define RECEIVER_COMMUNICATION_TIMEOUT_IN_MILLISECONDS 500

// Flight limits
// -------------
#define THROTTLE_START_POINT 10  // between 0-180
#define THROTTLE_LIMIT_POINT 180 // between 0-180
#define QUADCOPTER_MAX_TILT_ANGLE 10.00 // roll, pitch tilt angle limit in degrees
#define QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND 45.00

#endif
