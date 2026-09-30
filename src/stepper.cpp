#include "stepper.h"

#include <Arduino.h>
#include <FastAccelStepper.h>

#include "config.h"

#if !CONFIG_IDF_TARGET_ESP32 || ESP_IDF_VERSION_MAJOR != 4
#error "Position capture requires the classic ESP32 and the pinned Arduino/IDF 4 platform"
#endif

namespace {

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;
uint32_t commandedRpm = 0;
uint32_t commandedHz = Config::kHomingHz;

int64_t forwardDistance(uint32_t from, uint32_t to) {
  const uint32_t difference = to - from;
  return difference <= INT32_MAX ? static_cast<int64_t>(difference)
                                 : static_cast<int64_t>(difference) - (INT64_C(1) << 32);
}

}  // namespace

namespace Motor {

bool begin() {
  digitalWrite(Config::kEnablePin, HIGH);
  pinMode(Config::kEnablePin, OUTPUT);
  digitalWrite(Config::kDirectionPin, LOW);
  pinMode(Config::kDirectionPin, OUTPUT);

  engine.init();
  stepper = engine.stepperConnectToPin(Config::kStepPin);
  if (stepper == nullptr) {
    return false;
  }
  // Keep the physical direction LOW, as in the original wiring. Omitting a
  // direction pin from the library also prevents accidental reverse moves.
  stepper->setEnablePin(Config::kEnablePin, true);
  stepper->setAutoEnable(false);
  stepper->setForwardPlanningTimeInMs(Config::kForwardPlanningMs);
  return stepper->disableOutputs() &&
         stepper->setAcceleration(Config::kAccelerationStepsPerSecondSquared) == 0 &&
         stepper->setSpeedInHz(Config::kHomingHz) == 0 &&
         stepper->attachToPulseCounter(Config::kPositionCounterUnit, 0, 0);
}

bool run(uint32_t rpm) {
  if (stepper == nullptr || rpm < Config::kMinRpm || rpm > Config::kMaxRpm) {
    return false;
  }
  if (rpm == commandedRpm && stepper->isRunningContinuously()) {
    return true;
  }
  const uint32_t frequency = rpm * Config::kMotorStepsPerRevolution / 60;
  if (stepper->setSpeedInHz(frequency) != 0 || !stepper->enableOutputs() ||
      stepper->runForward() != MOVE_OK) {
    return false;
  }
  commandedHz = frequency;
  commandedRpm = rpm;
  return true;
}

bool isRunning() {
  return stepper != nullptr && stepper->isRunning();
}

bool isAtHomingSpeed() {
  if (!isRunning()) {
    return false;
  }
  const int32_t actualSpeed = stepper->getCurrentSpeedInMilliHz();
  const int32_t plannedSpeed = stepper->getCurrentSpeedInMilliHz(false);
  const int32_t limit = Config::kHomingHz * 1000;
  return actualSpeed > 0 && actualSpeed <= limit && plannedSpeed > 0 && plannedSpeed <= limit;
}

uint32_t debounceSpeedHz() {
  if (!isRunning()) {
    return 0;
  }
  const int32_t actual = stepper->getCurrentSpeedInMilliHz();
  const int32_t planned = stepper->getCurrentSpeedInMilliHz(false);
  const int32_t fastest = actual > planned ? actual : planned;
  const uint32_t measuredHz = fastest > 0 ? (static_cast<uint32_t>(fastest) + 999) / 1000 : 0;
  // Include the requested rate so acceleration cannot leave a slow-speed
  // lockout in effect when the next crossing arrives sooner.
  return measuredHz > commandedHz ? measuredHz : commandedHz;
}

void requestStop() {
  commandedRpm = 0;
  if (stepper != nullptr) {
    stepper->stopMove();
  }
}

void abort() {
  // Only used for a latched fault if normal deceleration fails to finish.
  // The position becomes invalid and a controller reset is required.
  if (stepper != nullptr) {
    stepper->forceStopAndNewPosition(0);
  }
}

bool release() {
  if (stepper == nullptr) {
    digitalWrite(Config::kEnablePin, HIGH);
    return true;
  }
  return !stepper->isRunning() && stepper->disableOutputs();
}

uint32_t position() {
  return stepper == nullptr ? 0 : static_cast<uint32_t>(stepper->getCurrentPosition());
}

uint16_t pulseCount() {
  return stepper == nullptr ? 0 : static_cast<uint16_t>(stepper->readPulseCounter());
}

uint32_t positionAtPulseCount(uint16_t pulseCount) {
  // The free-running PCNT and the stepper share an initial zero and are never
  // rebased while moving. Reconstruct the edge's 32-bit position from its low
  // 16 bits. Only fresh edges within kSensorEventMaxAgeUs may be passed here.
  const uint32_t nearbyPosition = position();
  const uint16_t difference = pulseCount - static_cast<uint16_t>(nearbyPosition);
  const int32_t offset = difference <= INT16_MAX ? static_cast<int32_t>(difference)
                                                : static_cast<int32_t>(difference) - 65536;
  return nearbyPosition + static_cast<uint32_t>(offset);
}

bool moveToBdcApproach(uint32_t reference, uint32_t& target) {
  if (!isAtHomingSpeed()) {
    return false;
  }
  // Stop before the estimated edge; the machine controller finds actual BDC.
  target = reference - Config::kBdcApproachLeadSteps;
  const uint32_t queuedPosition = stepper->getPositionAfterCommandsCompleted();
  const int64_t lead = forwardDistance(queuedPosition, target);
  const int64_t requiredLead = stepper->stepsToStop() + Config::kPlanningMarginSteps;
  if (lead <= requiredLead) {
    const uint32_t extraCycles = static_cast<uint32_t>(
        (requiredLead - lead) / Config::kStepsPerNeedleCycle + 1);
    target += extraCycles * Config::kStepsPerNeedleCycle;
  }
  commandedRpm = 0;
  return stepper->moveTo(static_cast<int32_t>(target)) == MOVE_OK;
}

bool isAtTarget(uint32_t target) {
  return stepper != nullptr && !stepper->isRunning() && position() == target &&
         static_cast<uint16_t>(stepper->readPulseCounter()) == static_cast<uint16_t>(target);
}

bool scanForward(uint32_t steps, uint32_t& start, uint32_t& target) {
  if (!isAtHomingSpeed() || debounceSpeedHz() > Config::kPositionFeedbackMaxHz ||
      steps == 0 || steps > INT32_MAX) {
    return false;
  }
  start = position();
  target = start + steps;
  const uint32_t queuedPosition = stepper->getPositionAfterCommandsCompleted();
  if (forwardDistance(queuedPosition, target) <= stepper->stepsToStop()) {
    return false;
  }
  commandedRpm = 0;
  commandedHz = Config::kNeedleMoveHz;
  return stepper->setSpeedInHz(Config::kNeedleMoveHz) == 0 &&
         stepper->moveTo(static_cast<int32_t>(target)) == MOVE_OK;
}

bool moveForward(uint32_t steps, uint32_t& target) {
  if (stepper == nullptr || steps == 0 || steps > INT32_MAX || !isAtTarget(position())) {
    return false;
  }
  target = position() + steps;
  commandedRpm = 0;
  commandedHz = Config::kNeedleMoveHz;
  return stepper->setSpeedInHz(Config::kNeedleMoveHz) == 0 && stepper->enableOutputs() &&
         stepper->moveTo(static_cast<int32_t>(target)) == MOVE_OK;
}

}  // namespace Motor
