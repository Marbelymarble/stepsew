#ifndef PEDAL_H
#define PEDAL_H

#include <Arduino.h>
#include "main.h"
#include "stepper.h"


void pedalSetup();
int pedalRead();
float mapPedalValue(int pedalValue);
#endif
