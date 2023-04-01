#ifndef DUCKSTORAGE_H
#define  DUCKSTORAGE_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <PololuMaestro.h>

#define SOLENOIDPIN1 15
#define SOLENOIDPIN2 14

#define maestroSerial Serial1

namespace DuckStorage{
extern MicroMaestro maestro;
extern uint8_t u8_duckStorageServoPos;

extern ros::Subscriber<std_msgs::String> servo_pos;
extern ros::Subscriber<std_msgs::Bool> solenoid_pos;

void cmdPosServo(const std_msgs::String& msg);
void cmdPosSolenoid(const std_msgs::Bool& msg);
void init(ros::NodeHandle* nh);
}

#endif