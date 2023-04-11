#ifndef CONSUMPTION_H
#define CONSUMPTION_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

// Consumption Defs
#define PWM1 23 // depricated
#define CONSUMPTIONPIN1 8
#define CONSUMPTIONPIN2 9
#define consumptionMotorOff analogWrite(PWM1, 0)

namespace consumption
{
// Consumption Data
//extern std_msgs::UInt8 u8_stateMotorConsumption;

// Consumption Pub/Sub
//extern ros::Publisher motorState;
extern ros::Subscriber<std_msgs::UInt8> cmdMotorState;

/**
 * @brief Callback for ros::Subscriber /locomotion/cmd_vel
 * Analog write to PWM pin for locomotion motor speed control
 *
 * @param msg Updated std_msgs::UInt8 value published to /consumption/cmdMotorState
 */
void consumptionCallback(const std_msgs::UInt8& msg);

/**
 * @brief Initialization for the consumption namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh);

}  // namespace consumption

#endif