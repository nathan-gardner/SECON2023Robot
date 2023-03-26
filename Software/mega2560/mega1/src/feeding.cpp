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
uint8_t u8_feedingServoPos = 0;

// Feeding Sub
ros::Subscriber<std_msgs::String> servo_pos("/feeding/cmd_servo_pos", &cmdPosServo);

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

void init(ros::NodeHandle* nh)
{
  // feeding
  //nh->subscribe(servo_pos);
  // Setup servo for feeding
  maestroSerial.begin(9600);
  maestro.setTarget(0, 6000);
}
}

#endif