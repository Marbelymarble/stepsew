#include "pedal.h"

void pedalSetup() {
  pinMode(pedalPin, INPUT);
}

void pedalRead() {
  int pedalValue = analogRead(pedalPin);
  Serial.print(pedalValue);
  Serial.print(" - ");

  if(pedalValue < pedalEnableThreshold) {
    disableMotor();
    Serial.println("Pedal not pressed - motor disabled");
  }else if (pedalValue >= pedalEnableThreshold && pedalValue < pedalSpeedVariationThreshold) {
    enableMotorMin();
    Serial.println("Pedal pressed - minimum speed");
  } else {
    float pedalMapped = static_cast<float>(pedalValue - pedalSpeedVariationThreshold) / static_cast<float>(pedalSpeedVariationMax - pedalSpeedVariationThreshold);
    
    enableMotor(calculateRPM(powf(pedalMapped, pedalPower)));

    Serial.println(pedalMapped);
  }
}