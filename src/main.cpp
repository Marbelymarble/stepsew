#include <Arduino.h>
#include "stepper.h"
#include "pedal.h"
#include "main.h"


unsigned long lastStart = 0;

void setup() {
Serial.begin(115200);
  stepperSetup();
  pedalSetup();

  lastStart = millis() -MIN_PERIOD_MS; // so that the first loop iteration runs immediately
}

void loop() {
  unsigned long now = millis();
  if ((long)(now - lastStart) >= (long)MIN_PERIOD_MS) { // cast to long to avoid overflow issues
    lastStart = now;
    programLoop();
  }
}

void programLoop() {
  // read pedal value
  int pedalValue = analogRead(pedalPin);

  // calculate speed based on pedal value
  int speed = calculateSpeed(pedalValue);

  // set motor speed
  setMotorSpeed(speed);
}