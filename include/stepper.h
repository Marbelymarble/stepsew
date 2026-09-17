#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include "main.h"


void stepperSetup();
void disableMotor();
void enableMotorMin();
void enableMotor(int rpm);
int calculateRPM(float speedFactor);

#endif