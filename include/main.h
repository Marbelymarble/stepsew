#include <movingAvg.h>
#pragma once

// stupid pins stuff
static constexpr uint8_t stepPin = 14;
static constexpr uint8_t dirPin = 27;
static constexpr uint8_t enablePin = 26;
static constexpr uint8_t pedalPin = 2;

// stupid analog pedal bullshit
#define pedalResistor 10000 // 10k ohm resistor for pedal input
#define pedalEnableThreshold 2000
#define pedalHysteresis 20 // deadband to prevent chatter between enabled and disabled states
#define pedalSpeedVariationThreshold 2250 //threshold where we rise above minimum speed and start using the pedal value for speed regulation
#define pedalSpeedVariationMax 4095 //maximum pedal value
#define pedalPower 1.2 //power to which the pedal value is raised

// Motor definition stuff
#define motorStepsPerRevolution 2000 //mathematical max is 4000 (at 200k pulses per second), 3200 gives us a bit of a buffer, 4k may be tested later
#define motorMaxRPM 3000 // max motor speed in RPM
#define motorMinRPM 60 // minimum motor speed in RPM so that the Machine still works

// stupid program specific stuff
const unsigned long MIN_PERIOD_MS = 1; // minimum period in milliseconds for the loop to run, to avoid oversampling and in general get a more consistent behaviour
const unsigned int movingAVGWindowSize = 500; // size of the moving average window for pedal readings


enum {stopped, minSpeed, speedVariation, homing};

void programLoop();
void ARDUINO_ISR_ATTR bdc_callback();
bool returnBdcTrigger();
void setBdcTrigger(bool);
void goToHome();
long returnPedalAvg();