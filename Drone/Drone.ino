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

double yaw = 0;
double pitch = 0;
double roll = 0;

double acc_x = 0;
double acc_y = 0;
double acc_z = 0;

double Kp_roll = 1;
double Ki_roll = 0;
double Kd_roll = 0;

double Kp_pitch = 0;
double Ki_pitch = 0;
double Kd_pitch = 0;

double Kp_yaw = 0;
double Ki_yaw = 0;
double Kd_yaw = 0;

double roll_err = 0;
double pitch_err = 0;
double yaw_err = 0;

double prev_roll_err = 0;
double prev_pitch_err = 0;
double prev_yaw_err = 0;

double roll_i = 0;
double pitch_i = 0;
double yaw_i = 0;

int M1 = 0;
int M2 = 0;
int M3 = 0;
int M4 = 0;
double PID_frequency = 0.005;

void setup() {
  Serial.begin(9600);
  attachMotors();
  delay(5000);
  motorTest();
  
  attachInterrupt(digitalPinToInterrupt(2), handle_interrupt, CHANGE);
  setupMPU();
  updateMPU(yaw, pitch, roll, acc_x, acc_y, acc_z);
  while(!mpuReady());
  
}

void loop() {
  updateCommand();
  updateMPU(yaw, pitch, roll, acc_x, acc_y, acc_z);

//  Serial.print("areal\t");
//  Serial.print(acc_x);
//  Serial.print("\t");
//  Serial.print(acc_y);
//  Serial.print("\t");
//  Serial.println(acc_z);

//  Serial.print("ypr: ");
//  Serial.print(yaw);
//  Serial.print("\t");
//  Serial.print(pitch);
//  Serial.print("\t");
//  Serial.println(roll);
  
  PID_Loop();
}

void updateCommand(){
  control_command[0] = constrain(channel_values[0], 600, 1600);
  control_command[1] = constrain(channel_values[1], 600, 1600);
  control_command[2] = constrain(channel_values[2], 600, 1600);
  control_command[3] = constrain(channel_values[3], 600, 1600);
  
  control_command[0] = map(control_command[0], 600, 1600, -100, 100);
  control_command[1] = map(control_command[1], 600, 1600, -100, 100);
  control_command[2] = map(control_command[2], 600, 1600, 0, 180);
  control_command[3] = map(control_command[3], 600, 1600, -100, 100);

//  Serial.print("Control Commands: ");
//  Serial.print(control_command[0]);
//  Serial.print("\t");
//  Serial.print(control_command[1]);
//  Serial.print("\t");
//  Serial.print(control_command[2]);
//  Serial.print("\t");
//  Serial.println(control_command[3]);
  control_command[0] = 0;
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

void motorTest(){
  for(int i = 0; i<30; i++){
    motor1.write(i);
    motor2.write(i);
    motor3.write(i);
    motor4.write(i);
    delay(20);
    }
  for(int i = 30; i>0; i--){
    motor1.write(i);
    motor2.write(i);
    motor3.write(i);
    motor4.write(i);
    delay(50);
    }
  }

void PID_Loop(){
  calculateErrors();
  double roll_PID = getPID(roll_err, Kp_roll, Ki_roll, Kd_roll, roll_i, prev_roll_err, PID_frequency);
  double pitch_PID = getPID(pitch_err, Kp_pitch, Ki_pitch, Kd_pitch, pitch_i, prev_pitch_err, PID_frequency);
  double yaw_PID = getPID(yaw_err, Kp_yaw, Ki_yaw, Kd_yaw, yaw_i, prev_yaw_err, PID_frequency);
  
  Serial.print("Roll PID: ");
  Serial.print(roll_PID);
  Serial.print("\t");

//  Serial.print("Pitch PID: ");
//  Serial.print(pitch_PID);
//  Serial.print("Yaw PID: ");
//  Serial.println(yaw_PID);
  
  calculateMotorSpeeds(roll_PID, pitch_PID, yaw_PID);
  updateSpeed();
}

void calculateErrors(){
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("\t");
  
//  Serial.print("Pitch: ");
//  Serial.print(getPitch());
//  Serial.print("Yaw: ");
//  Serial.println(getYaw());
  
  roll_err = control_command[0] - roll;
  pitch_err = control_command[1] - pitch;
  yaw_err = control_command[3] - yaw;

  Serial.print("Error: ");
  Serial.print(roll_err);
  Serial.print("\t");
  }

void calculateMotorSpeeds(double rol, double pit, double yaw){
  M1 = control_command[2] + rol + pit - yaw;
  M2 = control_command[2] - rol + pit + yaw;
  M3 = control_command[2] - rol - pit - yaw;
  M4 = control_command[2] + rol - pit + yaw;

  M1<0?M1=0:M1=M1;
  M2<0?M2=0:M2=M2;
  M3<0?M3=0:M3=M3;
  M4<0?M4=0:M4=M4;

  M1 = constrain(M1, 0, 180);
  M2 = constrain(M2, 0, 180);
  M3 = constrain(M3, 0, 180);
  M4 = constrain(M4, 0, 180);
  }

void updateSpeed(){
  Serial.print("Motors: ");
  Serial.print(M1);
  Serial.print("\t");
  Serial.print(M2);
  Serial.print("\t");
  Serial.print(M3);
  Serial.print("\t");
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
