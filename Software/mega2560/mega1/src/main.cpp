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

#include <feeding.h>
#include <locomotion.h>
#include <consumption.h>

ros::NodeHandle nh;

/**
 * @brief Setup code for Arduino boot
 *
 */
void setup()
{
  nh.initNode();

  locomotion::init(&nh);
  feeding::init(&nh);
  consumption::init(&nh);
}

/**
 * @brief Loop that is run continually while the Arduino is powered on
 *
 */
void loop()
{
  feeding::maestro.setTargetMiniSSC(0, feeding::u8_feedingServoPos);
  locomotion::updateEncoder();
  locomotion::encoder.publish(&locomotion::u32_motorPosData);
  consumption::motorState.publish(&consumption::u8_stateMotorConsumption);
  locomotion::motorState.publish(&locomotion::t_stateMotorLocomotion);
  nh.spinOnce();
  delay(10);
}

#endif