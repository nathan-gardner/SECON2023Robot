#ifndef CONSUMPTION_H
#define CONSUMPTION_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

// Consumption Defs
#define PWM1 23
#define consumptionMotorOff analogWrite(PWM1, 0)

namespace consumption
{
void consumptionCallback(const std_msgs::UInt8& msg);

// Consumption Data
std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
ros::Publisher motorState("/consumption/motorState", &u8_stateMotorConsumption);
ros::Subscriber<std_msgs::UInt8> cmdMotorState("/consumption/cmdMotorState", &consumptionCallback);

/**
 * @brief Callback for ros::Subscriber /locomotion/cmd_vel
 * Analog write to PWM pin for locomotion motor speed control
 *
 * @param msg Updated std_msgs::UInt8 value published to /consumption/cmdMotorState
 */
void consumptionCallback(const std_msgs::UInt8& msg)
{
  // update motor state and analog pin
  u8_stateMotorConsumption.data = msg.data;
  analogWrite(PWM1, msg.data);
}

/**
 * @brief Initialization for the consumption namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh)
{
  // Consumption
  pinMode(PWM1, OUTPUT);
  // consumption
  nh->advertise(motorState);
  nh->subscribe(cmdMotorState);

  // initialize consumption motor state to false and motor state to off
  u8_stateMotorConsumption.data = 0;
  consumptionMotorOff;
}

}  // namespace consumption

#endif