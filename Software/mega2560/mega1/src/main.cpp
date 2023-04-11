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

#include <locomotion.h>
#include <feeding.h>
#include <consumption.h>
#include <duckstorage.h>
#include <start.h>

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
  DuckStorage::init(&nh);
  start::init(&nh);

  Serial.begin(115200);
}

/**
 * @brief Loop that is run continually while the Arduino is powered on
 *
 */
void loop()
{
  start::read();
  locomotion::updateEncoder();
  locomotion::updateVelocity();
  start::updateStart();
  locomotion::computeVelocity(locomotion::enc_vel);
  locomotion::lowPassFilter(locomotion::enc_vel);
  locomotion::set_locomotion_speed();
  feeding::maestro.setTargetMiniSSC(2, feeding::u8_rightFeedingServoPos);
  feeding::maestro.setTargetMiniSSC(3, feeding::u8_leftFeedingServoPos);
  //locomotion::velocity.publish(&locomotion::af32_velocity);
  locomotion::encoder.publish(&locomotion::i32_motorPosData);
  start::start.publish(&start::b_start);
  //consumption::motorState.publish(&consumption::u8_stateMotorConsumption);
  //locomotion::motorState.publish(&locomotion::t_stateMotorLocomotion);
  nh.spinOnce();
  delay(150);
}

#endif
