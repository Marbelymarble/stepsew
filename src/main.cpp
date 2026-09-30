#include <Arduino.h>

#include "config.h"
#include "bdc_sensor.h"
#include "machine.h"
#include "pedal.h"

namespace {

uint32_t lastSampleMs = 0;

}  // namespace

void setup() {
  Serial.begin(Config::kSerialBaud);
  Pedal::begin();
  Machine::begin();
  lastSampleMs = millis() - Config::kControlPeriodMs;
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastSampleMs) < Config::kControlPeriodMs) {
    return;
  }
  // Do not insert back-to-back samples to catch up after a delayed iteration.
  lastSampleMs = now;
  Machine::update(Pedal::sample(), now);
  BdcSensor::reportDiagnostics(now);
}
