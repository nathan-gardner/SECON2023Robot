#ifndef DUCKSTORAGE_H
#define  DUCKSTORAGE_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <PololuMaestro.h>

#include <duckstorage.h>

#define solenoidPin 7
#define solenoidPin2 8

#define maestroSerial Serial1

namespace DuckStorage{
void cmdPosServo(const std_msgs::String& msg);
void cmdPosSolenoid(const std_msgs::String& msg);

MicroMaestro maestro(maestroSerial);
uint8_t u8_duckStorageServoPos = 0;

ros::Subscriber<std_msgs::String> servo_pos("/duckstorage/cmd_servo_pos", &cmdPosServo);
ros::Subscriber<std_msgs::String> solenoid_pos("/duckstorage/cmd_solenoide_pos", &cmdPosSolenoid);

void cmdPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "LEFT") == 0)
  {
    u8_duckStorageServoPos = 254;
  }
  else if (strcmp(msg.data, "RIGHT") == 0)
  {
    u8_duckStorageServoPos = 0;
  }
  else
  {
    u8_duckStorageServoPos = 254;
  }
}

void cmdPosSolenoid(const std_msgs::String& msg)
{
    if(strcmp(msg.data, "IN") == 0)
    {
        digitalWrite(solenoidPin, HIGH);
        digitalWrite(solenoidPin2, HIGH);
    }
    else if(strcmp(msg.data, "OUT") == 0)
    {
        digitalWrite(solenoidPin, LOW);
        digitalWrite(solenoidPin2, LOW);
    }
}

void init(ros::NodeHandle* nh)
{
    nh->subscribe(servo_pos);

    pinMode(solenoidPin, OUTPUT);
    pinMode(solenoidPin2, OUTPUT);
    maestroSerial.begin(9600);
    DuckStorage::maestro.setTarget(0, 6000);
}
}

#endif