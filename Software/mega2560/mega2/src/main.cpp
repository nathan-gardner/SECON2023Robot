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
#define maestroSerial Serial1

MicroMaestro maestro(maestroSerial);

void setup() {
  pinMode(solenoidPin, OUTPUT);
  pinMode(solenoidPin2, OUTPUT);
  maestroSerial.begin(9600);
  maestro.setTarget(0, 6000);
}

void loop() {
  digitalWrite(solenoidPin, HIGH);
  digitalWrite(solenoidPin2, HIGH);
  maestro.setTargetMiniSSC(0, 254);
  delay(500);

  digitalWrite(solenoidPin, LOW);
  digitalWrite(solenoidPin2, LOW);
  maestro.setTargetMiniSSC(0, 0);
  delay(7000);
}



#endif
