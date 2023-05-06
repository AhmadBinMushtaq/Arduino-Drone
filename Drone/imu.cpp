#include <Arduino.h>
#include "Constants.h"
#include "Structs.h"
#include "Functions.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool blinkState = false;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

struct Orientation previousOrientation;
unsigned long last_time = 0;

void initializeIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(IMU_INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(GYRO_OFFSET_X);
  mpu.setYGyroOffset(GYRO_OFFSET_Y);
  mpu.setZGyroOffset(GYRO_OFFSET_Z);
  mpu.setZAccelOffset(ACCEL_OFFSET_Z);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_BUILTIN, OUTPUT);
}

struct IMU_Values GetIMUvalues() {
  struct IMU_Values o;
  o.NewDataAvailable = false;

  if (!mpu.testConnection()) {
    o.Error = true;
    return o;
  }

  if (!dmpReady)
    return o;

  unsigned long current_time = millis();
  unsigned long delta_time_in_milliseconds = current_time - last_time;
  double delta_time_in_seconds = (double)delta_time_in_milliseconds / 1000.0;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    o.CurrentOrientation.YawAngle = ypr[0] * 180 / M_PI;
    o.CurrentOrientation.RollAngle = ypr[1] * 180 / M_PI;
    o.CurrentOrientation.PitchAngle = ypr[2] * 180 / M_PI; // -1 is for changing rotation in order to align with receiver values
    o.PreviousOrientation = previousOrientation;
    o.NewDataAvailable = true;
    o.DeltaTimeInSeconds = delta_time_in_seconds;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    previousOrientation = o.CurrentOrientation;
    if (last_time == 0) {
      last_time = current_time;
      o.Error = true;
      return o;
    }
    last_time = current_time;
  }

  if (delta_time_in_milliseconds > IMU_COMMUNICATION_TIMEOUT_IN_MILLISECONDS) {
    o.Error = true;
  } else {
    o.Error = false;
  }

  return o;
}
