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

#include <PololuMaestro.h>

#define solenoidPin 7
#define solenoidPin2 8

void setup() {
  pinMode(solenoidPin, OUTPUT);
  pinMode(solenoidPin2, OUTPUT);

}

void loop() {
  digitalWrite(solenoidPin, HIGH);
  digitalWrite(solenoidPin2, HIGH);
  delay(2000);

  digitalWrite(solenoidPin, LOW);
  digitalWrite(solenoidPin2, LOW);
  delay(7000);
}

#endif
