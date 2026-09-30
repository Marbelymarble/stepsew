#include "pedal.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace {

int readings[Config::kPedalSamples] = {};
size_t nextReading = 0;
size_t readingCount = 0;
uint32_t readingSum = 0;

}  // namespace

namespace Pedal {

void begin() {
  pinMode(Config::kPedalPin, INPUT);
  analogReadResolution(12);
}

int sample() {
  const int reading = analogRead(Config::kPedalPin);
  readingSum -= readings[nextReading];
  readings[nextReading] = reading;
  readingSum += reading;
  nextReading = (nextReading + 1) % Config::kPedalSamples;
  if (readingCount < Config::kPedalSamples) {
    ++readingCount;
  }
  return static_cast<int>((readingSum + readingCount / 2) / readingCount);
}

uint32_t rpmForReading(int reading) {
  if (reading <= Config::kPedalVariableThreshold) {
    return Config::kMinRpm;
  }
  if (reading >= Config::kPedalMax) {
    return Config::kMaxRpm;
  }
  const float fraction = static_cast<float>(reading - Config::kPedalVariableThreshold) /
                         (Config::kPedalMax - Config::kPedalVariableThreshold);
  return Config::kMinRpm + static_cast<uint32_t>(
      (Config::kMaxRpm - Config::kMinRpm) * powf(fraction, Config::kPedalExponent));
}

}  // namespace Pedal
