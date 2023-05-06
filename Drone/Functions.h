#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void initializeMotors();
void initializeOutputSignals();
void syncOutputSignals();
void blink_led();
void initializeIMU();
void initializeReceiver();
void handle_interrupt();
struct ReceiverCommands GetReceiverCommands();
struct ReceiverCommands getFailureReceiverCommand();
bool getArmStatus();
bool isArming();
bool isDisarming();
bool isThrottleStickPositonAtFullDown();
bool isYawStickPositionAtFullLeft();
bool isYawStickPositionAtFullRight();
double map_double(double x, double in_min, double in_max, double out_min, double out_max);
struct IMU_Values GetIMUvalues();
void stopMotors();
void spinMotors(struct MotorPowers motorPowers);
void InitializePIDConstants();
struct MotorPowers calculateMotorPowers(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values);
double calculateYawError(struct ReceiverCommands receiverCommands, struct IMU_Values imu_values);
struct MotorPowers reduceMotorPowers(MotorPowers motorPowers);
void resetPidVariables();
double fix360degrees(double val);
struct ConstantsPID getPIDValues();
void tunePID(struct ReceiverCommands cmd);
double getControlSignal(double error, double kp, double ki, double kd, double& pid_i, double& last_error, double delta_time_in_seconds);
void quadcopter_armed();
void quadcopter_disarmed();

#endif
