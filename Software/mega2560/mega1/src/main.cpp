/**
 * @file main.cpp
 * @author Nathan Gardner
 * @brief ROS node P/S to /consumption/motorState and consumption/cmdMotorState for consumption control. ROS node P/S Twist message type on topics /locomotion/motorState and /locomotion/cmd_vel for locomotion motor control. 
 * @version 0.2
 * @date 2023-01-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MAIN_CPP
#define MAIN_CPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>

// Consumption Defs
#define PWM1 23
#define consumptionMotorOff analogWrite(PWM1, 0)

// Locomotion Defs 
#define FRONT_LEFT_PIN1 26
#define FRONT_LEFT_PIN2 28
#define FRONT_RIGHT_PIN1 22
#define FRONT_RIGHT_PIN2 24
#define REAR_LEFT_PIN1 30
#define REAR_LEFT_PIN2 32
#define REAR_RIGHT_PIN1 34
#define REAR_RIGHT_PIN2 36

#define FRONT_LEFT_SPEED_PIN 10
#define FRONT_RIGHT_SPEED_PIN 11
#define REAR_LEFT_SPEED_PIN 12
#define REAR_RIGHT_SPEED_PIN 13

// ENCA always need to be at an interrupt pin
#define FRONT_LEFT_ENCA 19
#define FRONT_LEFT_ENCB 5
#define FRONT_RIGHT_ENCA 2
#define FRONT_RIGHT_ENCB 6
#define REAR_LEFT_ENCA 18
#define REAR_LEFT_ENCB 7
#define REAR_RIGHT_ENCA 3
#define REAR_RIGHT_ENCB 4

void consumptionCallback(const std_msgs::UInt8& msg);
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);

ros::NodeHandle nh;

// Consumption Data
std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
ros::Publisher _consumption_motorState("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::UInt8> _consumption_cmdMotorState("/consumption/cmdMotorState", &consumptionCallback);

// Locomotion Data
geometry_msgs::Twist t_stateMotorLocomotion;
std_msgs::UInt32MultiArray u32_motorPosData;

// array format front_left, front_right, rear_left, rear_right
uint32_t enc_pos[4] = {0, 0, 0, 0};

// Locomotion Pub/Sub
ros::Publisher _locomotion_motorState("/locomotion/motorState", &t_stateMotorLocomotion);
ros::Publisher _locomotion_encoder("/locomotion/encoder", &u32_motorPosData);
ros::Subscriber<geometry_msgs::Twist> _locomotion_cmd_vel("/locomotion/cmd_vel", &cmdVelCallback);

/**
 * @brief Callback for ros::Subscriber /locomotion/cmd_vel
 * Analog write to PWM pin for locomotion motor speed control
 * 
 * @param msg Updated std_msgs::UInt8 value published to /consumption/cmdMotorState
 */
void consumptionCallback(const std_msgs::UInt8& msg){
  // update motor state and analog pin
  u8_stateMotorConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

/**
 * @brief Digital write to motor_pin1 and motor_pin2 to command direction and speed. Used for the locomotion subsystem.
 * 
 * @param motor_pin1 Pin1 (digital value) for locomotion motor for which the direction is being set
 * @param motor_pin2 Pin2 (digital value) for locomotion motor for which the direction is being set
 * @param speed_pin Speed pin (analog value) for locomotion motor for which the speed is being set
 * @param motor_speed [-1.0,1.0] range for from Twist message which encodes cmd direction and speed
 */
void set_motor_speed(int motor_pin1, int motor_pin2, int speed_pin, float motor_speed) {
  // set motor speed
  int set_speed = 255 * motor_speed;
  if (set_speed > 0) {
    digitalWrite(motor_pin1, HIGH); // forward direction
    digitalWrite(motor_pin2, LOW);
  } 
  else if (set_speed < 0) {
    digitalWrite(motor_pin1, LOW); // reverse direction
    digitalWrite(motor_pin2, HIGH);
    set_speed = -set_speed;
  }
  else {
    digitalWrite(motor_pin1, LOW); // stop movement
    digitalWrite(motor_pin2, LOW);
  }
  analogWrite(speed_pin, set_speed); // set motor speed (ignoring for now)
}

/**
 * @brief Decodes geometry_msgs::Twist message to get linear x and y and angular z. Calculates each wheel speed and directions and then writes values to pins. 
 * 
 * @param cmd_vel Updated geometry_msgs::Twist value published to /locomotion/cmd_vel 
 */
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  t_stateMotorLocomotion = cmd_vel;
  // calculate motor speeds from twist message
  float x = cmd_vel.linear.x;
  float y = cmd_vel.linear.y;
  float z = cmd_vel.angular.z;

  float front_left_speed = y + x + z;
  float front_right_speed = y - x - z;
  float rear_left_speed = y - x + z;
  float rear_right_speed = y + x - z;

  // set motor speeds
  set_motor_speed(FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, FRONT_LEFT_SPEED_PIN, front_left_speed);
  set_motor_speed(FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, FRONT_RIGHT_SPEED_PIN, front_right_speed);
  set_motor_speed(REAR_LEFT_PIN1, REAR_LEFT_PIN2, REAR_LEFT_SPEED_PIN, rear_left_speed);
  set_motor_speed(REAR_RIGHT_PIN1, REAR_RIGHT_PIN2, REAR_RIGHT_SPEED_PIN, rear_right_speed);
}

/**
 * @brief Update published encoder value
 * 
 */

void updateEncoder(){
  u32_motorPosData.data_length = sizeof(enc_pos)/sizeof(enc_pos[0]);
  u32_motorPosData.data = enc_pos;
}

/**
 * @brief interrupt service routine for front left encoder , increments position + or - based on direction
 * 
 */
void readFrontLeftEncoder(){
  int b = digitalRead(FRONT_LEFT_ENCB);
  if(b>0){
    enc_pos[0]++;
  }
  else{
    enc_pos[0]--;
  }
}

/**
 * @brief interrupt service routine for front right encoder , increments position + or - based on direction
 * 
 */
void readFrontRightEncoder(){
  int b = digitalRead(FRONT_RIGHT_ENCB);
  if(b>0){
    enc_pos[1]--;
  }
  else{
    enc_pos[1]++;
  }
}

/**
 * @brief interrupt service routine for rear left encoder , increments position + or - based on direction
 * 
 */
void readRearLeftEncoder(){
  int b = digitalRead(REAR_LEFT_ENCB);
  if(b>0){
    enc_pos[2]++;
  }
  else{
    enc_pos[2]--;
  }
}

/**
 * @brief interrupt service routine for rear right encoder , increments position + or - based on direction
 * 
 */
void readRearRightEncoder(){
  int b = digitalRead(REAR_RIGHT_ENCB);
  if(b>0){
    enc_pos[3]--;
  }
  else{
    enc_pos[3]++;
  }
}

/**
 * @brief Setup code for Arduino boot
 * 
 */
void setup()
{
  //Consumption
  pinMode(PWM1, OUTPUT);

  //Locomotion
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

  nh.initNode();
  // consumption
  nh.advertise(_consumption_motorState);
  nh.subscribe(_consumption_cmdMotorState);
  // locomotion
  nh.advertise(_locomotion_motorState);
  nh.advertise(_locomotion_encoder);
  nh.subscribe(_locomotion_cmd_vel);

  // initialize consumption motor state to false and motor state to off
  u8_stateMotorConsumption.data = 0;
  consumptionMotorOff;

  // locomotion
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCA), readFrontLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCA), readFrontRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCA), readRearLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCA), readRearRightEncoder, RISING);
}

/**
 * @brief Loop that is run continually while the Arduino is powered on
 * 
 */
void loop()
{
  updateEncoder();
  _locomotion_encoder.publish( &u32_motorPosData );
  _consumption_motorState.publish( &u8_stateMotorConsumption );
  _locomotion_motorState.publish ( &t_stateMotorLocomotion );
  nh.spinOnce();
  delay(10);
}

#endif
