#include <Servo.h>

#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 10
#define MOTOR_PIN_4 11

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

volatile unsigned long pulse_start = 0;
volatile unsigned int channel_values[8];
volatile byte current_channel = 0;

int control_command[4];

double Kp_roll = 1;
double Ki_roll = 1;
double Kd_roll = 1;

double Kp_pitch = 1;
double Ki_pitch = 1;
double Kd_pitch = 1;

double Kp_yaw = 1;
double Ki_yaw = 1;
double Kd_yaw = 1;

double roll_err = 0;
double pitch_err = 0;
double yaw_err = 0;

double prev_roll_err = 0;
double prev_pitch_err = 0;
double prev_yaw_err = 0;

double roll_i = 0;
double pitch_i = 0;
double yaw_i = 0;

byte M1 = 0;
byte M2 = 0;
byte M3 = 0;
byte M4 = 0;
double PID_frequency = 0.005;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), handle_interrupt, CHANGE);
  setupMPU();
  updateMPU();
  while(!mpuReady());
  attachMotors();
  delay(10000);
  
}

void loop() {
  updateCommand();
  updateMPU();
  PID_Loop();
}

void updateCommand(){
  control_command[0] = map(channel_values[0], 600, 1600, -100, 100);
  control_command[1] = map(channel_values[1], 600, 1600, -100, 100);
  control_command[2] = map(channel_values[2], 600, 1600, 0, 180);
  control_command[3] = map(channel_values[3], 600, 1600, -100, 100);
  }

void attachMotors(){
  motor1.attach(MOTOR_PIN_1, 1000, 2000);
  motor2.attach(MOTOR_PIN_2, 1000, 2000);
  motor3.attach(MOTOR_PIN_3, 1000, 2000);
  motor4.attach(MOTOR_PIN_4, 1000, 2000);
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
  }
  
void PID_Loop(){
  calculateErrors();
  double roll_PID = getPID(roll_err, Kp_roll, Ki_roll, Kd_roll, roll_i, prev_roll_err, PID_frequency);
  double pitch_PID = getPID(pitch_err, Kp_pitch, Ki_pitch, Kd_pitch, pitch_i, prev_pitch_err, PID_frequency);
  double yaw_PID = getPID(yaw_err, Kp_yaw, Ki_yaw, Kd_yaw, yaw_i, prev_yaw_err, PID_frequency);
  
//  Serial.print("Roll PID: ");
//  Serial.print(roll_PID);
//  Serial.print("Pitch PID: ");
//  Serial.print(pitch_PID);
//  Serial.print("Yaw PID: ");
//  Serial.println(yaw_PID);
  
  calculateMotorSpeeds(roll_PID, pitch_PID, yaw_PID);
  updateSpeed();
}

void calculateErrors(){
//  Serial.print("Roll: ");
//  Serial.print(getRoll());
//  Serial.print("Pitch: ");
//  Serial.print(getPitch());
//  Serial.print("Yaw: ");
//  Serial.println(getYaw());
  
  roll_err = control_command[0] - getRoll();
  pitch_err = control_command[1] - getPitch();
  yaw_err = control_command[3] - getYaw();
  }

void calculateMotorSpeeds(double rol, double pit, double yaw){
  M1 = control_command[2] + rol + pit - yaw;
  M2 = control_command[2] - rol + pit + yaw;
  M3 = control_command[2] - rol - pit - yaw;
  M4 = control_command[2] + rol - pit + yaw;

  M1 = constrain(M1, 0, 180);
  M2 = constrain(M2, 0, 180);
  M3 = constrain(M3, 0, 180);
  M4 = constrain(M4, 0, 180);
  }

void updateSpeed(){
  Serial.print("Motors: ");
  Serial.print(M1);
  Serial.print("    ");
  Serial.print(M2);
  Serial.print("    ");
  Serial.print(M3);
  Serial.print("    ");
  Serial.println(M4);
  
  motor1.write(M1);
  motor2.write(M2);
  motor3.write(M3);
  motor4.write(M4);
  }

double getPID(double error, double kp, double ki, double kd, double& pid_i, double& last_error, double delta_time_in_seconds) {
  double pid_p = error;
  double pid_d = (error - last_error) / delta_time_in_seconds;
  pid_i += error * delta_time_in_seconds;

  double control_signal = (kp * pid_p) + (ki * pid_i) + (kd * pid_d);
  last_error = error;
  return control_signal;
}

void handle_interrupt() {
  if(micros()-pulse_start>2050){
    pulse_start = micros();
    current_channel = 0;
    return;
    }

  if(micros() - pulse_start < 450){
    pulse_start = micros();
    return;
    }
    
    if(current_channel>8){
      pulse_start = micros();
      current_channel = 0;
      }
    channel_values[current_channel] = micros() - pulse_start;
    current_channel++;
    pulse_start = micros();

  
}
