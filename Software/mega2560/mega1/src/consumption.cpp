#ifndef CONSUMPTION_CPP
#define CONSUMPTION_CPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>

#include <consumption.h>

namespace consumption
{

// Consumption Data
//std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
//ros::Publisher motorState("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::Bool> cmdMotorState("/consumption/cmdMotorState", &consumptionCallback);

void consumptionCallback(const std_msgs::Bool& msg)
{
  if(msg.data == false){
    digitalWrite(CONSUMPTIONPIN1, LOW);
    digitalWrite(CONSUMPTIONPIN2, LOW);
  }
  else{
    digitalWrite(CONSUMPTIONPIN1, LOW);
    digitalWrite(CONSUMPTIONPIN2, HIGH);
  }
  
}

void init(ros::NodeHandle* nh)
{
  // Consumption
  //pinMode(PWM1, OUTPUT);
  pinMode(CONSUMPTIONPIN1, OUTPUT);
  pinMode(CONSUMPTIONPIN2, OUTPUT);
  // consumption
  //nh->advertise(motorState);
  nh->subscribe(cmdMotorState);

  // initialize consumption motor state to false and motor state to off
  //u8_stateMotorConsumption.data = 0;
  //consumptionMotorOff;
}


}

#endif