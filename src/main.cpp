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


unsigned char machineState = stopped;
bool request_homing = false;


void setup() {
Serial.begin(115200);
  stepperSetup();
  pedalSetup();
  lastStart = millis() -MIN_PERIOD_MS; // so that the first loop iteration runs immediately
  pedalAvg.begin();
  pinMode(33, INPUT_PULLUP); // Set pin 33 as input with pull-up resistor
  attachInterrupt(33, bdc_callback, FALLING);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);
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
  bool homeStarted = false;

  switch (machineState) {
    case stopped: 
      if (isMotorRunning) {
        disableMotor();
        // break;
      }
    if (pedalAvg.getAvg() > (pedalEnableThreshold + pedalHysteresis)) {
      machineState = minSpeed;
    } else {
      break;
    }

    case minSpeed:
      enableMotorMin();
      if (pedalAvg.getAvg() < (pedalEnableThreshold - pedalHysteresis)) {
        machineState = homing;
        break;
    } else if (pedalAvg.getAvg() > (pedalSpeedVariationThreshold + pedalHysteresis)) {
      machineState = speedVariation;
      break;
    } else {
      break;
    }

    case speedVariation:
      if(pedalAvg.getAvg() < (pedalSpeedVariationThreshold - pedalHysteresis)) {
        machineState = minSpeed;
        break;
      } else {
        float pedalMapped = mapPedalValue(pedalAvg.getAvg());
        enableMotor(calculateRPM(powf(pedalMapped, pedalPower)));
        break;
      }

    case homing:
      if (!homeStarted) {
        homeStarted = true;
        goToHome();
      }
      
      if ((pedalAvg.getAvg() > (pedalSpeedVariationThreshold + pedalHysteresis))) {
        machineState = speedVariation;
        break;
      }
      
      if(returnCurrentPosition() < 100) {
        disableMotor();
        homeStarted = false;
        machineState = stopped;
        break;
      }
  }



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

long returnPedalAvg() {
  return pedalAvg.getAvg();
}
