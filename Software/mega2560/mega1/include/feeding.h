#ifndef FEEDING_H
#define FEEDING_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <PololuMaestro.h>

// Servo defs
#define maestroSerial Serial2

namespace feeding
{
extern MicroMaestro maestro;
extern uint8_t u8_feedingServoPos;

// Feeding Sub
extern ros::Subscriber<std_msgs::String> servo_pos;

/**
 * @brief Decodes std_msgs::String and update feeding servo position. The exact value of the positions will need to be
 * found when the feeding system has been designed.
 *
 * @param msg std_msgs::String& value for the changed position of the feeding servo
 */
void cmdPosServo(const std_msgs::String& msg);

/**
 * @brief Initialization for the feeding namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh);

}  // namespace feeding

#endif