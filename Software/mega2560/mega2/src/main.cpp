/**
 * @file main.cpp
 * @author Nathan Gardner
 * @brief Main driver file
 * @version 0.2
 * @date 2023-01-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef MAIN_CPP
#define MAIN_CPP

#include <Arduino.h>
#include <ros.h>
#include <PololuMaestro.h>

#include <duckstorage.h>

ros::NodeHandle nh;

void setup() {
  nh.initNode();
  DuckStorage::init(&nh);
}

void loop() {
  DuckStorage::maestro.setTargetMiniSSC(0, DuckStorage::u8_duckStorageServoPos);
  nh.spinOnce();
  delay(10);
}



#endif
