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
extern uint8_t u8_rightFeedingServoPos;
extern uint8_t u8_leftFeedingServoPos;

// Feeding Sub
extern ros::Subscriber<std_msgs::String> right_servo_pos;
extern ros::Subscriber<std_msgs::String> left_servo_pos;

/**
 * @brief Decodes std_msgs::String and update feeding servo position. The exact value of the positions will need to be
 * found when the feeding system has been designed.
 *
 * @param msg std_msgs::String& value for the changed position of the feeding servo
 */
void cmdRightPosServo(const std_msgs::String& msg);

/**
 * @brief Decodes std_msgs::String and update feeding servo position. The exact value of the positions will need to be
 * found when the feeding system has been designed.
 *
 * @param msg std_msgs::String& value for the changed position of the feeding servo
 */
void cmdLeftPosServo(const std_msgs::String& msg);

/**
 * @brief Initialization for the feeding namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh);

}  // namespace feeding

#endif