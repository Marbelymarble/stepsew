#include "pedal.h"

void pedalSetup() {
  pinMode(pedalPin, INPUT);
}

int pedalRead() {
  return analogRead(pedalPin);
}

float mapPedalValue(int pedalValue) {
  float pedalMapped = static_cast<float>(pedalValue - pedalSpeedVariationThreshold) / static_cast<float>(pedalSpeedVariationMax - pedalSpeedVariationThreshold);
  return pedalMapped;
}