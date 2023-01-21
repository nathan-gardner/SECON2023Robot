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
#include <std_msgs/Bool.h>

#define PWM1 2
#define consumptionMotorOn analogWrite(PWM1, 255)
#define consumptionMotorOff analogWrite(PWM1, 0)

// Prototypes
void consumptionCallback(const std_msgs::Bool& msg);

ros::NodeHandle nh;
std_msgs::Bool b_stateConsumption;

ros::Publisher consumption_pub("stateConsumption", &b_stateConsumption);
ros::Subscriber<std_msgs::Bool> consumption_sub("consumption_sub", &consumptionCallback);

void consumptionCallback(const std_msgs::Bool& msg){
  if(msg.data == true){
    b_stateConsumption.data = true;
    consumptionMotorOn;
  }
  else{
    b_stateConsumption.data = false;
    consumptionMotorOff;
  }
}

void setup()
{
  pinMode(PWM1, OUTPUT);

  nh.initNode();
  nh.advertise(consumption_pub);
  nh.subscribe(consumption_sub);

  // initialize motor state to false and motor state to off
  b_stateConsumption.data = false;
  consumptionMotorOff;
}

void loop()
{
  consumption_pub.publish( &b_stateConsumption );
  nh.spinOnce();
  delay(1000);
}

#endif