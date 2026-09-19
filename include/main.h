#include <movingAvg.h>
#pragma once

// stupid pins stuff
static constexpr uint8_t stepPin = 14;
static constexpr uint8_t dirPin = 27;
static constexpr uint8_t enablePin = 26;
static constexpr uint8_t pedalPin = 2;

// stupid analog pedal bullshit
#define pedalResistor 10000 // 10k ohm resistor for pedal input
#define pedalEnableThreshold 1750
#define pedalEnableHysteresis 20 // deadband to prevent chatter between enabled and disabled states
#define pedalSpeedVariationThreshold 2000 //threshold where we rise above minimum speed and start using the pedal value for speed regulation
#define pedalSpeedVariationHysteresis 20 // deadband to prevent chatter between minimum-speed and normal-speed modes
#define pedalSpeedVariationMax 4095 //maximum pedal value
#define pedalPower 1.2 //power to which the pedal value is raised

// Motor definition stuff
#define motorStepsPerRevolution 3200 //mathematical max is 4000 (at 200k pulses per second), 3200 gives us a bit of a buffer, 4k may be tested later
#define motorMaxRPM 2500 // max motor speed in RPM
#define motorMinRPM 5 // minimum motor speed in RPM so that the Machine still works

// stupid program specific stuff
const unsigned long MIN_PERIOD_MS = 100; // minimum period in milliseconds for the loop to run, to avoid oversampling and in general get a more consistent behaviour
const unsigned int movingAVGWindowSize = 1000; // size of the moving average window for pedal readings

void programLoop();