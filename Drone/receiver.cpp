#include <Arduino.h>
#include "Constants.h"
#include "Functions.h"
#include "Structs.h"

volatile struct ReceiverRawValues receiverRawValues;
bool armed = false;

volatile unsigned long receiver_last_communication_time = millis();
volatile unsigned long pulse_start = 0;
volatile byte current_channel = 0;

void initializeReceiver() {
  attachInterrupt(digitalPinToInterrupt(RX_INTERRUPT_PIN), handle_interrupt, CHANGE);
}

void handle_interrupt() {

  if(millis()-receiver_last_communication_time < RECEIVER_COMMUNICATION_TIMEOUT_IN_MILLISECONDS){
    receiverRawValues.TransmitterCommunicationFailure = false;
    }
  else{
    receiverRawValues.TransmitterCommunicationFailure = true;
    }
  
  if(micros()-pulse_start>2050){
    pulse_start = micros();
    current_channel = 0;
    return;
    }

  if(micros() - pulse_start < 450){
    pulse_start = micros();
    return;
    }
    
    if(current_channel>6){
      pulse_start = micros();
      receiver_last_communication_time = millis();
      receiverRawValues.ChannelValues[current_channel] = micros() - pulse_start;
      current_channel = 0;
      return;
      }
    receiverRawValues.ChannelValues[current_channel] = micros() - pulse_start;
    current_channel++;
    pulse_start = micros();

  
}

struct ReceiverCommands GetReceiverCommands() {
  if (receiverRawValues.TransmitterCommunicationFailure) {
    return getFailureReceiverCommand();
  }
  if(millis()-receiver_last_communication_time > RECEIVER_COMMUNICATION_TIMEOUT_IN_MILLISECONDS){
    return getFailureReceiverCommand();
    }
    struct ReceiverCommands cmd;
    cmd.RollAngle = map_double(constrain(receiverRawValues.ChannelValues[0], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_TILT_ANGLE, QUADCOPTER_MAX_TILT_ANGLE);
    cmd.PitchAngle = map_double(constrain(receiverRawValues.ChannelValues[1], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_TILT_ANGLE, QUADCOPTER_MAX_TILT_ANGLE);
    cmd.Throttle = map_double(constrain(receiverRawValues.ChannelValues[2], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, 0, THROTTLE_LIMIT_POINT);
    cmd.YawAngleChange = map_double(constrain(receiverRawValues.ChannelValues[3], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, -QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND, QUADCOPTER_MAX_YAW_ANGLE_CHANGE_PER_SECOND);
    cmd.Aux_1 = map(constrain(receiverRawValues.ChannelValues[4], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, 0, 100);
    cmd.Aux_2 = map(constrain(receiverRawValues.ChannelValues[5], TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE), TRANSMITTER_JOYSTICK_MIN_VALUE, TRANSMITTER_JOYSTICK_MAX_VALUE, 0, 100);
    cmd.Armed = getArmStatus();
    cmd.Error = false;

    return cmd;
}

struct ReceiverCommands getFailureReceiverCommand() {
  struct ReceiverCommands cmd;
  cmd.Error = true;
  cmd.RollAngle = 0;
  cmd.PitchAngle = 0;
  cmd.YawAngleChange = 0;
  cmd.Armed = false;
  cmd.Throttle = 0;
  return cmd;
}

bool hasArmingStarted = false;
bool hasDisarmingStarted = false;
unsigned long armingStartTime;
unsigned long disarmingStartTime;

bool getArmStatus() {
  if (receiverRawValues.TransmitterCommunicationFailure) {
    return armed;
  }

  if (isArming() && !armed) {
    hasDisarmingStarted = false;
    if (!hasArmingStarted) {
      armingStartTime = millis();
      hasArmingStarted = true;
    } else {
      if (millis() - armingStartTime >= TRANSMITTER_ARMING_DURATION_IN_MILLISECONDS) {
        armed = true;
        quadcopter_armed();
      }
    }
  } else if (isDisarming() && armed) {
    hasArmingStarted = false;
    if (!hasDisarmingStarted) {
      disarmingStartTime = millis();
      hasDisarmingStarted = true;
    } else {
      if (millis() - disarmingStartTime >= TRANSMITTER_ARMING_DURATION_IN_MILLISECONDS) {
        armed = false;
        quadcopter_disarmed();
      }
    }
  } else {
    hasArmingStarted = false;
    hasDisarmingStarted = false;
  }

  return armed;
}

bool isArming() {
  if (isThrottleStickPositonAtFullDown() && isYawStickPositionAtFullRight()) {
    return true;
  }
  return false;
}

bool isDisarming() {
  if (isThrottleStickPositonAtFullDown() && isYawStickPositionAtFullLeft()) {
    return true;
  }
  return false;
}

bool isThrottleStickPositonAtFullDown() {
  if (abs(receiverRawValues.ChannelValues[2] - TRANSMITTER_JOYSTICK_MIN_VALUE) < TRANSMITTER_ARMING_JOYSTICK_TOLERANCE) {
    return true;
  }
  return false;
}

bool isYawStickPositionAtFullLeft() {
  if (abs(receiverRawValues.ChannelValues[3] - TRANSMITTER_JOYSTICK_MIN_VALUE) < TRANSMITTER_ARMING_JOYSTICK_TOLERANCE) {
    return true;
  }
  return false;
}

bool isYawStickPositionAtFullRight() {
  if (abs(receiverRawValues.ChannelValues[3] - TRANSMITTER_JOYSTICK_MAX_VALUE) < TRANSMITTER_ARMING_JOYSTICK_TOLERANCE) {
    return true;
  }
  return false;
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
