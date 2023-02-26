/**
 * @file main.cpp
 * @author Nathan Gardner
 * @brief ROS node P/S to /consumption/motorState and consumption/cmdMotorState for motor remote control
 * @version 0.1
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
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>

// Consumption Defs
#define PWM1 2
#define consumptionMotorOff analogWrite(PWM1, 0)

// Locomotion Defs ()
#define FRONT_LEFT_PIN1 5
#define FRONT_LEFT_PIN2 6
#define FRONT_RIGHT_PIN1 3
#define FRONT_RIGHT_PIN2 4
#define REAR_LEFT_PIN1 7
#define REAR_LEFT_PIN2 8
#define REAR_RIGHT_PIN1 9
#define REAR_RIGHT_PIN2 10

// Prototypes
void consumptionCallback(const std_msgs::UInt8& msg);
void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);

ros::NodeHandle nh;

// Consumption Data
std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
ros::Publisher consumption_pub("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::UInt8> consumption_sub("/consumption/cmdMotorState", &consumptionCallback);

// Locomotion Data
geometry_msgs::Twist t_stateMotorLocomotion;

// Locomotion Pub/Sub
ros::Publisher locomotion_pub("/locomotion/motorState", &t_stateMotorLocomotion);
ros::Subscriber<geometry_msgs::Twist> locomotion_sub("/locomotion/cmd_vel", &cmdVelCallback);


void consumptionCallback(const std_msgs::UInt8& msg){
  // update motor state and analog pin
  u8_stateMotorConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

//---------------------------------------------------------------------------

void set_motor_speed(int motor_pin1, int motor_pin2, int motor_speed) {
  // set motor speed
  if (motor_speed > 0) {
    digitalWrite(motor_pin1, HIGH); // forward direction
    digitalWrite(motor_pin2, LOW);
  } 
  else if (motor_speed < 0) {
    digitalWrite(motor_pin1, LOW); // reverse direction
    digitalWrite(motor_pin2, HIGH);
  }
  else {
    digitalWrite(motor_pin1, LOW); // stop movement
    digitalWrite(motor_pin2, LOW);
  }
  //analogWrite(motor_pin, motor_speed); // set motor speed (ignoring for now)
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  t_stateMotorLocomotion = cmd_vel;
  // calculate motor speeds from twist message
  volatile float x = cmd_vel.linear.x;
  volatile float y = cmd_vel.linear.y;
  volatile float z = cmd_vel.angular.z;

  int front_left_speed = y + x + z;
  int front_right_speed = y - x - z;
  int rear_left_speed = y - x + z;
  int rear_right_speed = y + x - z;

  // set motor speeds
  set_motor_speed(FRONT_LEFT_PIN1, FRONT_LEFT_PIN2, front_left_speed);
  set_motor_speed(FRONT_RIGHT_PIN1, FRONT_RIGHT_PIN2, front_right_speed);
  set_motor_speed(REAR_LEFT_PIN1, REAR_LEFT_PIN2, rear_left_speed);
  set_motor_speed(REAR_RIGHT_PIN1, REAR_RIGHT_PIN2, rear_right_speed);
}

//---------------------------------------------------------------------------

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
  
  //pinMode(FRONT_RIGHT_PIN, OUTPUT);
  //pinMode(REAR_LEFT_PIN, OUTPUT);
  //pinMode(REAR_RIGHT_PIN, OUTPUT);

  nh.initNode();
  // consumption
  nh.advertise(consumption_pub);
  nh.subscribe(consumption_sub);
  // locomotion
  nh.advertise(locomotion_pub);
  nh.subscribe(locomotion_sub);

  // initialize consumption motor state to false and motor state to off
  u8_stateMotorConsumption.data = 0;
  consumptionMotorOff;

  //au8_stateMotorLocomotion.data = 0;
  //analogWrite(PWMX, 0);
}

void loop()
{
  consumption_pub.publish( &u8_stateMotorConsumption );
  locomotion_pub.publish ( &t_stateMotorLocomotion );
  nh.spinOnce();
  delay(10);
}

#endif
