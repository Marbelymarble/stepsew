#include <Arduino.h>
#include "stepper.h"
#include "pedal.h"
#include "main.h"


unsigned long lastStart = 0;
static bool isMinSpeedMode = false;
static bool isMotorEnabled = false;
movingAvg pedalAvg(movingAVGWindowSize);


void setup() {
Serial.begin(115200);
  stepperSetup();
  pedalSetup();
  lastStart = millis() -MIN_PERIOD_MS; // so that the first loop iteration runs immediately
  pedalAvg.begin();
}

void loop() {
  unsigned long now = millis();
  pedalAvg.reading(pedalRead());
  if ((long)(now - lastStart) >= (long)MIN_PERIOD_MS) { // cast to long to avoid overflow issues
    lastStart = now;
    programLoop();
  }
}

void programLoop() {
  // entry stuff (i know this all is giga spaghetti)


  int pedalImmediate = pedalRead();
  Serial.println(pedalAvg.getAvg());
  const int enterNormalModeThreshold = pedalSpeedVariationThreshold + pedalSpeedVariationHysteresis;
  const int enterMinModeThreshold = pedalSpeedVariationThreshold - pedalSpeedVariationHysteresis;
  const int disableMotorThreshold = pedalEnableThreshold - pedalEnableHysteresis;
  const int enableMotorThreshold = pedalEnableThreshold + pedalEnableHysteresis;

  if (isMotorEnabled) {
    if (pedalAvg.getAvg() <= disableMotorThreshold) {
      disableMotor();
      isMotorEnabled = false;
      isMinSpeedMode = false;
      return;
    }
  } else {
    if (pedalAvg.getAvg() < enableMotorThreshold) {
      disableMotor();
      isMinSpeedMode = false;
      return;
    }
    isMotorEnabled = true;
  }

  if (isMinSpeedMode) {
    if (pedalAvg.getAvg() >= enterNormalModeThreshold) {
      isMinSpeedMode = false;
    }
  } else {
    if (pedalAvg.getAvg() <= enterMinModeThreshold) {
      isMinSpeedMode = true;
    }
  }

  if (isMinSpeedMode) {
    enableMotorMin();
    return;
  }

  float pedalMapped = mapPedalValue(pedalAvg.getAvg());
  enableMotor(calculateRPM(powf(pedalMapped, pedalPower)));
}