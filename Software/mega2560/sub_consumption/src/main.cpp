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
#include <std_msgs/UInt8MultiArray.h>

// Consumption Defs
#define PWM1 2
#define consumptionMotorOff analogWrite(PWM1, 0)

// Locomotion Defs
#define PWMX 0 // 1 PWM pin ADD LATER
#define LOCOMOTIONDIGITAL1 0 // 3 digital pins ADD LATER
#define LOCOMOTIONDIGITAL2 0
#define LOCOMOTIONDIGITAL3 0

// Prototypes
void consumptionCallback(const std_msgs::UInt8& msg);
void locomotionCallback(const std_msgs::UInt8MultiArray& msg);

ros::NodeHandle nh;

// Consumption Data
std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
ros::Publisher consumption_pub("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::UInt8> consumption_sub("/consumption/cmdMotorState", &consumptionCallback);

// Locomotion Data
std_msgs::UInt8MultiArray au8_stateMotorLocomotion;

// Locomotion Pub/Sub
ros::Publisher locomotion_pub("/locomotion/motorState", &au8_stateMotorLocomotion);
ros::Subscriber<std_msgs::UInt8MultiArray> locomotion_sub("/locomotion/cmdMotorState", &locomotionCallback);

void consumptionCallback(const std_msgs::UInt8& msg){
  // update motor state and analog pin
  u8_stateMotorConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

void locomotionCallback(const std_msgs::UInt8MultiArray& msg){
  // insert callback here

}

void setup()
{
  pinMode(PWM1, OUTPUT);

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
  locomotion_pub.publish ( &au8_stateMotorLocomotion );
  nh.spinOnce();
  delay(1000);
}

#endif
