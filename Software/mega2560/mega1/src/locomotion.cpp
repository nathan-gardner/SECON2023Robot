#ifndef LOCOMOTION_CPP
#define LOCOMOTION_CPP

#include <Arduino.h>
#include <ros.h>
#include <util/atomic.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <locomotion.h>

namespace locomotion
{
// Locomotion Data
geometry_msgs::Twist t_stateMotorLocomotion;
std_msgs::Int32MultiArray i32_motorPosData;
// array format front_left, front_right, rear_left, rear_right
int32_t enc_pos[4] = { 0, 0, 0, 0 };
volatile int32_t enc_pos_i[4] = { 0, 0, 0, 0 };

float enc_vel[4] = { 0, 0, 0, 0 };
volatile float enc_vel_i[4] = { 0, 0, 0, 0 };
volatile float xyz[4] = { 0, 0, 0, 0 };

int error_count[4] = { 0, 0, 0, 0 };
bool startCheck = false;
bool newDir = false;

float deltaT = 0;

std_msgs::Float32MultiArray af32_velocity;
ros::Publisher velocity("/locomotion/velocity", &af32_velocity);

// Locomotion Pub/Sub
ros::Publisher motorState("/locomotion/motorState", &t_stateMotorLocomotion);
ros::Publisher encoder("/locomotion/encoder", &i32_motorPosData);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("/locomotion/cmd_vel", &cmdVelCallback);

void set_motor_speed(int motor_pin1, int motor_pin2, int speed_pin, int motor_speed)
{
  if (motor_speed > 0)
  {
    digitalWrite(motor_pin1, HIGH);  // forward direction
    digitalWrite(motor_pin2, LOW);
  }
  else if (motor_speed < 0)
  {
    digitalWrite(motor_pin1, LOW);  // reverse direction
    digitalWrite(motor_pin2, HIGH);
    motor_speed = -motor_speed;
  }
  else
  {
    digitalWrite(motor_pin1, LOW);  // stop movement
    digitalWrite(motor_pin2, LOW);
  }
  analogWrite(speed_pin, motor_speed);  // set motor speed (ignoring for now)
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  if(cmd_vel.linear.x != t_stateMotorLocomotion.linear.x ||
    cmd_vel.linear.y != t_stateMotorLocomotion.linear.y ||
    cmd_vel.angular.z != t_stateMotorLocomotion.angular.z){
    newDir = true;
  }
  t_stateMotorLocomotion = cmd_vel;
  // calculate motor speeds from twist message
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float z = cmd_vel.angular.z;

  float front_left_speed = y + x + z;
  float front_right_speed = y - x - z;
  float rear_left_speed = y - x + z;
  float rear_right_speed = y + x - z;
  
  xyz[0] = front_left_speed;
  xyz[1] = front_right_speed;
  xyz[2] = rear_left_speed;
  xyz[3] = rear_right_speed;

  startCheck = false;

  error_count[0] = 0;
  error_count[1] = 0;
  error_count[2] = 0;
  error_count[3] = 0;
}

void set_locomotion_speed()
{
  // variable which will hold the pwr (-255 to 255) which will be written to the motors, (front left, front right, rear left, rear right)
  int pwr[4];
  pi_control(enc_vel, pwr, xyz);
  
  // Check that we are making progress towards goal
  for (int i = 0; i<4; i++){
    if(abs(xyz[i] - enc_vel[i]) < 5){
      startCheck = true;
    }
    if(startCheck){
      if(abs(xyz[i] - enc_vel[i]) > 8){
        error_count[i]++;
      }
    }
    if(error_count[i] > MAX_ERROR_COUNT){
      xyz[0] = 0;
      xyz[1] = 0;
      xyz[2] = 0;
      xyz[3] = 0;
    }
  }
  // set motor speeds
  set_motor_speed(FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, FRONT_LEFT_SPEED_PIN, pwr[0]);
  set_motor_speed(FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, FRONT_RIGHT_SPEED_PIN, pwr[1]);
  set_motor_speed(REAR_LEFT_PIN1, REAR_LEFT_PIN2, REAR_LEFT_SPEED_PIN, pwr[2]);
  set_motor_speed(REAR_RIGHT_PIN1, REAR_RIGHT_PIN2, REAR_RIGHT_SPEED_PIN, pwr[3]);
}

void updateEncoder()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    for (int i = 0; i < 4; i++)
    {
      enc_pos[i] = enc_pos_i[i];
    }
  }
  i32_motorPosData.data_length = sizeof(enc_pos) / sizeof(enc_pos[0]);
  i32_motorPosData.data = enc_pos;
}

void updateVelocity()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    for (int i = 0; i < 4; i++)
    {
      enc_vel[i] = enc_vel_i[i];
    }
  }
  af32_velocity.data_length = 4;
  af32_velocity.data = enc_vel;
}

void readFrontLeftEncoder()
{
  int b = digitalRead(FRONT_LEFT_ENCB);
  if (b > 0)
  {
    enc_pos_i[0]++;
  }
  else
  {
    enc_pos_i[0]--;
  }
}

void readFrontRightEncoder()
{
  int b = digitalRead(FRONT_RIGHT_ENCB);
  if (b > 0)
  {
    enc_pos_i[1]--;
  }
  else
  {
    enc_pos_i[1]++;
  }
}

void readRearLeftEncoder()
{
  int b = digitalRead(REAR_LEFT_ENCB);
  if (b > 0)
  {
    enc_pos_i[2]++;
  }
  else
  {
    enc_pos_i[2]--;
  }
}

void readRearRightEncoder()
{
  int b = digitalRead(REAR_RIGHT_ENCB);
  if (b > 0)
  {
    enc_pos_i[3]--;
  }
  else
  {
    enc_pos_i[3]++;
  }
}

void computeVelocity(volatile float* vel)
{
  static int32_t enc_posPrev[4];
  static long prevT;
  long currT = micros();
  deltaT = ((float)(currT - prevT)) / 1.0e6;
  for (int i = 0; i < 4; i++)
  {
    *(vel + i) = (enc_pos[i] - enc_posPrev[i]) / deltaT;
    // convert from clicks per cycle to rpm
    *(vel + i) = *(vel + i) / CLICKS_PER_ROTATION * 60.0;

    enc_posPrev[i] = enc_pos[i];
  }
  prevT = currT;
}

void lowPassFilter(volatile float* vel)
{
  static float velocityFilter[4];
  static float velocityPrev[4];
  for (int i = 0; i < 4; i++)
  {
    // predefined low pass filter constaints
    velocityFilter[i] = 0.854 * velocityFilter[i] + 0.0728 * (*(vel + i)) + 0.0728 * velocityPrev[i];
    velocityPrev[i] = *(vel + i);
  }
  // update velocity to low pass filter version
  vel = velocityFilter;
}

void pi_control(float* vel, int* pwr, volatile float* xyz)
{
  static float eintegral[4];

  if(newDir){
    eintegral[0] = 0;
    eintegral[1] = 0;
    eintegral[2] = 0;
    eintegral[3] = 0;
    newDir = false;
  }

  float kp = 1.5;
  float ki = 3;

  
  for (int i = 0; i < 4; i++)
  {
    float vt = *(xyz + i);
    /*
    // Decrease current draw for large changes and decrease voltage rise time
    if(vt - *(vel + i) > 50){
      vt = *(vel + i) + 50;
    }
    else if(vt - *(vel + i) < -50){
      vt = *(vel + i) - 50;
    }
    */
    float e = vt - *(vel + i);
    eintegral[i] = eintegral[i] + e * deltaT;
    float u = e * kp + eintegral[i] * ki;
    *(pwr + i) = (int)u;
    // cap pwr at -255 or 255
    if (*(pwr + i) > 255)
    {
      *(pwr + i) = 255;
    }
    else if (*(pwr + i) < -255)
    {
      *(pwr + i) = -255;
    }
  }
}

void e_stop(){
  set_motor_speed(FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, FRONT_LEFT_SPEED_PIN, 0);
  set_motor_speed(FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, FRONT_RIGHT_SPEED_PIN, 0);
  set_motor_speed(REAR_LEFT_PIN1, REAR_LEFT_PIN2, REAR_LEFT_SPEED_PIN, 0);
  set_motor_speed(REAR_RIGHT_PIN1, REAR_RIGHT_PIN2, REAR_RIGHT_SPEED_PIN, 0);
}

void init(ros::NodeHandle* nh)
{
  // Locomotion
  pinMode(FRONT_LEFT_PIN1, OUTPUT);
  pinMode(FRONT_LEFT_PIN2, OUTPUT);
  pinMode(FRONT_RIGHT_PIN1, OUTPUT);
  pinMode(FRONT_RIGHT_PIN2, OUTPUT);
  pinMode(REAR_LEFT_PIN1, OUTPUT);
  pinMode(REAR_LEFT_PIN2, OUTPUT);
  pinMode(REAR_RIGHT_PIN1, OUTPUT);
  pinMode(REAR_RIGHT_PIN2, OUTPUT);

  pinMode(FRONT_LEFT_SPEED_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_SPEED_PIN, OUTPUT);
  pinMode(REAR_LEFT_SPEED_PIN, OUTPUT);
  pinMode(REAR_RIGHT_SPEED_PIN, OUTPUT);

  pinMode(FRONT_LEFT_ENCA, INPUT);
  pinMode(FRONT_LEFT_ENCB, INPUT);
  pinMode(FRONT_RIGHT_ENCA, INPUT);
  pinMode(FRONT_RIGHT_ENCB, INPUT);
  pinMode(REAR_LEFT_ENCA, INPUT);
  pinMode(REAR_LEFT_ENCB, INPUT);
  pinMode(REAR_RIGHT_ENCA, INPUT);
  pinMode(REAR_RIGHT_ENCB, INPUT);

  // locomotion
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCA), readFrontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCA), readFrontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCA), readRearLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCA), readRearRightEncoder, RISING);

  // locomotion
  //nh->advertise(motorState);
  nh->advertise(encoder);
  nh->subscribe(cmd_vel);
  nh->advertise(velocity);
}

}  // namespace locomotion

#endif