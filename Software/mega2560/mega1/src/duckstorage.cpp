#ifndef DUCKSTORAGE_H
#define  DUCKSTORAGE_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <PololuMaestro.h>

#include <duckstorage.h>

#define SOLENOIDPIN1 15
#define SOLENOIDPIN2 14

#define maestroSerial Serial2

namespace DuckStorage{
void cmdPosServo(const std_msgs::String& msg);
void cmdPosSolenoid(const std_msgs::Bool& msg);

MicroMaestro maestro(maestroSerial);
uint8_t u8_duckStorageServoPos = 0;

ros::Subscriber<std_msgs::String> servo_pos("/duckstorage/cmd_servo_pos", &cmdPosServo);
ros::Subscriber<std_msgs::Bool> solenoid_pos("/duckstorage/cmd_solenoid_pos", &cmdPosSolenoid);

void cmdPosServo(const std_msgs::String& msg)
{
  if (strcmp(msg.data, "EXTEND") == 0)
  {
    maestro.restartScript(0);
  }
  else if (strcmp(msg.data, "RETRACT") == 0)
  {
    maestro.restartScript(1);
  }
}

void cmdPosSolenoid(const std_msgs::Bool& msg)
{
    if(msg.data == true) // retract solenoid
    {
        digitalWrite(SOLENOIDPIN1, HIGH);
        digitalWrite(SOLENOIDPIN2, HIGH);
    }
    else if(msg.data == false) // extend solenoid
    {
        digitalWrite(SOLENOIDPIN1, LOW);
        digitalWrite(SOLENOIDPIN2, LOW);
    }
}

void init(ros::NodeHandle* nh)
{
    nh->subscribe(servo_pos);
    nh->subscribe(solenoid_pos);

    pinMode(SOLENOIDPIN1, OUTPUT);
    pinMode(SOLENOIDPIN2, OUTPUT);
    digitalWrite(SOLENOIDPIN1, LOW);
    digitalWrite(SOLENOIDPIN2, LOW);
    maestroSerial.begin(9600);
    DuckStorage::maestro.setTarget(0, 6000);
}
}

#endif