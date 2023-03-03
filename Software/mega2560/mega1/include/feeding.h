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
void cmdPosServo(const std_msgs::String& msg);

MicroMaestro maestro(maestroSerial);
uint8_t u8_feedingServoPos = 0;

// Feeding Sub
ros::Subscriber<std_msgs::String> servo_pos("/feeding/cmd_servo_pos", &cmdPosServo);

/**
 * @brief Decodes std_msgs::String and update feeding servo position. The exact value of the positions will need to be
 * found when the feeding system has been designed.
 *
 * @param msg std_msgs::String& value for the changed position of the feeding servo
 */
void cmdPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "LEFT") == 0)
  {
    u8_feedingServoPos = 254;
  }
  else if (strcmp(msg.data, "RIGHT") == 0)
  {
    u8_feedingServoPos = 0;
  }
  else
  {
    u8_feedingServoPos = 254;
  }
}

/**
 * @brief Initialization for the feeding namespace
 *
 * @param nh Pointer to the ROS node handle
 */
void init(ros::NodeHandle* nh)
{
  // feeding
  nh->subscribe(servo_pos);
  // Setup servo for feeding
  maestroSerial.begin(9600);
  maestro.setTarget(0, 6000);
}

}  // namespace feeding

#endif