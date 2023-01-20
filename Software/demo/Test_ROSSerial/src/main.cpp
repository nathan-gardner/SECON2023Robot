/**
 * @file main.cpp
 * @author Nathan Gardner
 * @brief ROS demo for Arduino Mega 2560: set up node, advertise topic,
 * and counts up on topic
 * Ensure ROS is intalled correctly
 * @version 0.1
 * @date 2023-01-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
   
std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

uint32_t counter = 0;

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = counter;
  chatter.publish( &str_msg );
  nh.spinOnce();
  counter++;
  delay(1000);
}