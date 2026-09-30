#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Config {

constexpr uint8_t kStepPin = 14;
constexpr uint8_t kDirectionPin = 27;
constexpr uint8_t kEnablePin = 26;  // Active low.
constexpr uint8_t kPedalPin = 2;
constexpr uint8_t kBdcPin = 33;  // Active low; falling edge marks BDC.

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kControlPeriodMs = 1;
constexpr size_t kPedalSamples = 20;
constexpr int kPedalEnableThreshold = 2000;
constexpr int kPedalHysteresis = 20;
constexpr int kPedalVariableThreshold = 2250;
constexpr int kPedalMax = 4095;
constexpr float kPedalExponent = 1.2f;
constexpr uint32_t kPedalTapMinMs = 20;
constexpr uint32_t kPedalTapMaxMs = 250;
// Reject brief pedal rebound while homing, without imposing the tap hold delay.
constexpr uint32_t kHomingResumePressMs = 50;

constexpr uint32_t kMotorStepsPerRevolution = 2000;
// Approximately 2.33 motor revolutions per needle cycle (about 7:3).
// Confirm this rounded value against the sensor's measured crossing interval.
constexpr uint32_t kStepsPerNeedleCycle = 4667;
// Forward distance from physical BDC to TDC; adjust for the machine's linkage.
constexpr uint32_t kBdcToTdcSteps = kStepsPerNeedleCycle / 2;
// Calibrate the sensor's falling edge at BDC. A positive offset stops this many
// steps beyond that edge and must remain inside the sensor's active region.
constexpr uint32_t kBdcOffsetSteps = 0;
// Approach the expected BDC edge early, then scan at the needle-toggle speed.
// Stop 2% of a nominal cycle early; the final scan can cover a full cycle.
constexpr uint32_t kBdcApproachLeadSteps = kStepsPerNeedleCycle / 50;
constexpr uint32_t kBdcSearchTimeoutMs = 3000;
constexpr uint32_t kMinRpm = 60;
constexpr uint32_t kMaxRpm = 3000;
constexpr int32_t kAccelerationStepsPerSecondSquared = 400000;
constexpr uint32_t kHomingRpm = 150;
// At/below this actual, queued, and requested rate, scan directly on release.
constexpr uint32_t kPositionFeedbackMaxRpm = 150;
constexpr uint32_t kPositionFeedbackMaxHz =
    kPositionFeedbackMaxRpm * kMotorStepsPerRevolution / 60;
constexpr uint32_t kHomingHz = kHomingRpm * kMotorStepsPerRevolution / 60;
constexpr uint32_t kNeedleMoveHz = kMinRpm * kMotorStepsPerRevolution / 60;
// Allow a full needle cycle plus time for acceleration and final verification.
constexpr uint32_t kNeedleMoveTimeoutMs =
    1000 + (UINT64_C(1000) * kStepsPerNeedleCycle + kNeedleMoveHz - 1) / kNeedleMoveHz;

// PCNT 6 and 7 are free on the classic ESP32 with FastAccelStepper's IDF 4 driver.
constexpr uint8_t kPositionCounterUnit = 7;
constexpr uint32_t kSensorEventMaxAgeUs = 50000;
// Edge lockout, not a minimum LOW duration: short Hall pulses still count.
constexpr uint32_t kSensorDebouncePercent = 50;
constexpr uint32_t kSensorDebounceMinUs = 1000;
constexpr uint32_t kSensorDebounceMaxUs = 2000000;
constexpr uint32_t kSensorStoppedDebounceUs = 500000;
// Reject additional crossings within the same revolution, without demanding
// an exact interval from an approximate pulley ratio.
constexpr uint32_t kSensorMinSeparationSteps = kStepsPerNeedleCycle * 3 / 4;
constexpr uint32_t kReferenceMaxTravelSteps = kStepsPerNeedleCycle * 5 / 4;
constexpr uint32_t kHomingSpeedSettleMs = 50;
constexpr uint32_t kStopSettleMs = 10;
// Give the mechanism/input time to settle after STEP pulses end. During this
// window HIGH resets LOW qualification instead of immediately rejecting BDC.
constexpr uint32_t kBdcStopQualificationMs = 50;
constexpr uint32_t kFaultStopTimeoutMs = 1000;
// FastAccelStepper refills about every 4 ms. Two cycles of lookahead reduce
// sensor-to-stop latency while retaining a refill margin. Set only at startup.
constexpr uint32_t kForwardPlanningMs = 8;
// Bound capture-to-stop travel: queued motion, control/engine latency, braking.
// The final Hall LOW check remains mandatory even within this pulse budget.
constexpr uint32_t bdcCaptureToleranceSteps(uint32_t speedHz) {
  return (speedHz * (kForwardPlanningMs + kControlPeriodMs + 10) + 999) / 1000 +
         (UINT64_C(1) * speedHz * speedHz + 2 * kAccelerationStepsPerSecondSquared - 1) /
             (2 * kAccelerationStepsPerSecondSquared) + 1;
}
constexpr uint32_t kBdcCaptureToleranceSteps = bdcCaptureToleranceSteps(kNeedleMoveHz);
constexpr uint32_t kPlanningMarginSteps = kHomingHz / 20;  // 50 ms at homing speed.

// Up to two needle cycles to approach BDC, followed by the bounded edge search.
// Scale the deadline with the transmission ratio, not just the motor RPM.
constexpr uint32_t kHomingTravelMs =
    (UINT64_C(1000) * (UINT64_C(2) * kStepsPerNeedleCycle + kBdcOffsetSteps) +
     kHomingHz - 1) / kHomingHz;
constexpr uint32_t kMaxDecelerationMs =
    (UINT64_C(1000) * kMaxRpm * kMotorStepsPerRevolution / 60 +
     kAccelerationStepsPerSecondSquared - 1) / kAccelerationStepsPerSecondSquared;
constexpr uint32_t kHomingTimeoutMs = kHomingTravelMs + kMaxDecelerationMs +
                                    kHomingSpeedSettleMs + kStopSettleMs + 1000 +
                                    kBdcSearchTimeoutMs;

static_assert(kPedalSamples > 0, "The pedal filter needs at least one sample");
static_assert(kPedalVariableThreshold < kPedalMax, "Invalid pedal range");
static_assert(kPedalTapMinMs > 0 && kPedalTapMinMs < kPedalTapMaxMs,
              "Invalid pedal tap duration");
static_assert(kMinRpm > 0 && kMinRpm <= kMaxRpm, "Invalid motor speed range");
static_assert(kHomingRpm >= kMinRpm && kHomingRpm <= kMaxRpm, "Invalid homing speed");
static_assert(kForwardPlanningMs >= 8 && kForwardPlanningMs <= UINT8_MAX,
              "Retain at least two 4 ms queue-refill cycles");
static_assert(kPositionFeedbackMaxRpm >= kMinRpm && kPositionFeedbackMaxRpm <= kHomingRpm,
              "Reusable positioning feedback must stay within the homing speed limit");
static_assert(kSensorDebouncePercent > 0 && kSensorDebouncePercent < 100,
              "Leave time for the next legitimate BDC crossing");
static_assert(kSensorDebounceMaxUs >= kSensorDebounceMinUs,
              "Invalid debounce limits");
static_assert(kSensorMinSeparationSteps > 0, "Invalid sensor separation");
static_assert(kSensorDebounceMinUs > 0 &&
                  UINT64_C(1) * kSensorDebounceMinUs * kMaxRpm * kMotorStepsPerRevolution <
                      UINT64_C(60000000) * kStepsPerNeedleCycle,
              "Sensor debounce must be shorter than a needle cycle at maximum speed");
static_assert(UINT64_C(1) * kMaxRpm * kMotorStepsPerRevolution / 60 *
                      kSensorEventMaxAgeUs / 1000000 < INT16_MAX,
              "Sensor event age must allow unambiguous pulse-counter reconstruction");
static_assert(kBdcOffsetSteps < kStepsPerNeedleCycle, "Invalid BDC offset");
static_assert(kBdcToTdcSteps > 0 && kBdcToTdcSteps < kStepsPerNeedleCycle,
              "TDC must be between successive BDC positions");
static_assert(bdcCaptureToleranceSteps(kPositionFeedbackMaxHz) < kBdcToTdcSteps,
              "Braking allowance must leave a forward move to TDC");
static_assert(kBdcApproachLeadSteps > 0 && kBdcApproachLeadSteps < kStepsPerNeedleCycle / 2,
              "The homing approach lead must be less than half a needle cycle");

}  // namespace Config
