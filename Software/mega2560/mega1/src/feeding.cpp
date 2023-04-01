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
uint8_t u8_redFeedingServoPos = 0;
uint8_t u8_greenFeedingServoPos = 0;

// Feeding Sub
ros::Subscriber<std_msgs::String> red_servo_pos("/feeding/cmd_red_servo_pos", &cmdRedPosServo);
ros::Subscriber<std_msgs::String> blue_servo_pos("/feeding/cmd_blue_servo_pos", &cmdBluePosServo);

void cmdRedPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "LEFT") == 0)
  {
    u8_redFeedingServoPos = 254;
  }
  else if (strcmp(msg.data, "RIGHT") == 0)
  {
    u8_redFeedingServoPos = 0;
  }
  else
  {
    u8_redFeedingServoPos = 254;
  }
}

void cmdBluePosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "LEFT") == 0)
  {
    u8_greenFeedingServoPos = 254;
  }
  else if (strcmp(msg.data, "RIGHT") == 0)
  {
    u8_greenFeedingServoPos = 0;
  }
  else
  {
    u8_greenFeedingServoPos = 254;
  }
}

void init(ros::NodeHandle* nh)
{
  // feeding
  nh->subscribe(red_servo_pos);
  nh->subscribe(blue_servo_pos);
  // Setup servo for feeding
  maestroSerial.begin(9600);
  maestro.setTarget(0, 6000);
}
}

#endif