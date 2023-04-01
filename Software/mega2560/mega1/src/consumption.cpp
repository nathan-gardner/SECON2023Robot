#ifndef CONSUMPTION_CPP
#define CONSUMPTION_CPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

#include <consumption.h>

namespace consumption
{

// Consumption Data
std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
ros::Publisher motorState("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::UInt8> cmdMotorState("/consumption/cmdMotorState", &consumptionCallback);

void consumptionCallback(const std_msgs::UInt8& msg)
{
  // update motor state and analog pin
  u8_stateMotorConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

void init(ros::NodeHandle* nh)
{
  // Consumption
  pinMode(PWM1, OUTPUT);
  // consumption
  //nh->advertise(motorState);
  //nh->subscribe(cmdMotorState);

  // initialize consumption motor state to false and motor state to off
  u8_stateMotorConsumption.data = 0;
  consumptionMotorOff;
}


}

#endif