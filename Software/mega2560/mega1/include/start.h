#ifndef START_H
#define START_H

#include <Arduino.h>
#include <ros.h>

#include <std_msgs/Bool.h>

#define START_PIN 52

namespace start{
    extern std_msgs::Bool b_start;
    extern ros::Publisher start;

    extern bool b;

    void read();
    void updateStart();
    void init(ros::NodeHandle* nh);
}

#endif