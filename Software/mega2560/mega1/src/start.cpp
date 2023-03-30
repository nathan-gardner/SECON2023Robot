#ifndef START_CPP
#define START_CPP

#include <Arduino.h>
#include <ros.h>
#include <start.h>

#include <std_msgs/Bool.h>

namespace start{
    std_msgs::Bool b_start;
    ros::Publisher start("/start", &b_start);

    bool b;

    void read(){
        b = digitalRead(START_PIN);
    }

    void updateStart(){
        b_start.data = b;
    }


    void init(ros::NodeHandle* nh){
        pinMode(START_PIN, INPUT);
        nh->advertise(start);
    }
}


#endif