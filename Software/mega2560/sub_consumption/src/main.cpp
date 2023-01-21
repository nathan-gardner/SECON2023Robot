/**
 * @file main.cpp
 * @author Nathan Gardner
 * @brief ROS node P/S to stateConsumption for motor remote control
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

#define PWM1 2
#define consumptionMotorOff analogWrite(PWM1, 0)

// Prototypes
void consumptionCallback(const std_msgs::UInt8& msg);

ros::NodeHandle nh;
std_msgs::UInt8 u8_stateConsumption;

ros::Publisher consumption_pub("stateConsumption", &u8_stateConsumption);
ros::Subscriber<std_msgs::UInt8> consumption_sub("consumption_sub", &consumptionCallback);

void consumptionCallback(const std_msgs::UInt8& msg){
  // update motor state and analog pin
  u8_stateConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

void setup()
{
  pinMode(PWM1, OUTPUT);

  nh.initNode();
  nh.advertise(consumption_pub);
  nh.subscribe(consumption_sub);

  // initialize motor state to false and motor state to off
  u8_stateConsumption.data = 0;
  consumptionMotorOff;
}

void loop()
{
  consumption_pub.publish( &u8_stateConsumption );
  nh.spinOnce();
  delay(1000);
}

#endif