#ifndef FEEDING_CPP
#define FEEDING_CPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <PololuMaestro.h>

#include <feeding.h>

namespace feeding
{
MicroMaestro maestro(maestroSerial);
uint8_t u8_rightFeedingServoPos = 254;
uint8_t u8_leftFeedingServoPos = 254;

// Feeding Sub
ros::Subscriber<std_msgs::String> right_servo_pos("/feeding/cmd_right_servo_pos", &cmdRightPosServo);
ros::Subscriber<std_msgs::String> left_servo_pos("/feeding/cmd_left_servo_pos", &cmdLeftPosServo);

void cmdRightPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "CLOSE") == 0)
  {
    u8_rightFeedingServoPos = 254;
  }
  else if (strcmp(msg.data, "OPEN") == 0)
  {
    u8_rightFeedingServoPos = 0;
  }
  else
  {
    u8_rightFeedingServoPos = 254;
  }
}

void cmdLeftPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "CLOSE") == 0)
  {
    u8_leftFeedingServoPos = 254;
  }
  else if (strcmp(msg.data, "OPEN") == 0)
  {
    u8_leftFeedingServoPos = 0;
  }
  else
  {
    u8_leftFeedingServoPos = 254;
  }
}

void init(ros::NodeHandle* nh)
{
  // feeding
  nh->subscribe(right_servo_pos);
  nh->subscribe(left_servo_pos);
  // Setup servo for feeding
  maestroSerial.begin(9600);
  maestro.setTarget(0, 6000);
}
}

#endif