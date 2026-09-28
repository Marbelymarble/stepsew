#include <Arduino.h>
#include "stepper.h"
#include "pedal.h"
#include "main.h"


unsigned long lastStart = 0;
static bool isMinSpeedMode = false;
static bool isMotorEnabled = false;
movingAvg pedalAvg(movingAVGWindowSize);

long lastInterruptUs = 0;
long interruptDelay = 0;
long rotationCount = 0;


bool bdc_trigger = false;


void setup() {
Serial.begin(115200);
  stepperSetup();
  pedalSetup();
  lastStart = millis() -MIN_PERIOD_MS; // so that the first loop iteration runs immediately
  pedalAvg.begin();
  pinMode(33, INPUT_PULLUP); // Set pin 33 as input with pull-up resistor
  attachInterrupt(33, bdc_callback, FALLING);
}

void loop() {
  unsigned long now = millis();
  pedalAvg.reading(pedalRead());
  if ((long)(now - lastStart) >= (long)MIN_PERIOD_MS) { // cast to long to avoid overflow issues
    lastStart = now;
    programLoop();
  }


  if(isMotorRunning()) {
    interruptDelay = (returnCurrentSpeed() * motorStepsPerRevolution) - 5;
  } else {
    interruptDelay = 500000;
  }
}

void programLoop() {
  // entry stuff (i know this all is giga spaghetti)
  Serial.println(returnCurrentPosition());

  int pedalImmediate = pedalRead();
  //Serial.println(pedalAvg.getAvg());
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

void ARDUINO_ISR_ATTR bdc_callback() {
    uint32_t now = micros();
    // Ignore pulses occurring within 5 ms of the previous one
    if (now - lastInterruptUs > interruptDelay) {
      rotationCount++;
      lastInterruptUs = now;
      //Serial.println(rotationCount);
      setCurrentPosition(0);
    }

}
