#include "machine.h"

#include <Arduino.h>

#include "bdc_sensor.h"
#include "config.h"
#include "pedal.h"
#include "stepper.h"

namespace {

enum class State {
  Stopped,
  MinimumSpeed,
  VariableSpeed,
  Slowing,
  SeekingBdc,
  Positioning,
  HomingBdcSearch,
  HomingBdcOffset,
  Settling,
  MovingNeedle,
  SeekingNeedleBdc,
  SettlingNeedle,
  Fault
};

enum class NeedlePosition { Unknown, Bottom, Top };

State state = State::Stopped;
NeedlePosition needlePosition = NeedlePosition::Unknown;
NeedlePosition needleDestination = NeedlePosition::Unknown;
uint32_t homingStartedMs = 0;
uint32_t slowSinceMs = 0;
uint32_t settledSinceMs = 0;
uint32_t faultStartedMs = 0;
uint32_t referencePosition = 0;
uint32_t targetPosition = 0;
uint32_t needleMoveStartedMs = 0;
uint32_t bdcParkOverrunSteps = 0;
uint32_t bdcSearchStart = 0;
uint32_t bdcSearchRemaining = 0;
uint32_t bdcSearchLimit = 0;
uint32_t bdcEdgePosition = 0;
uint32_t bdcCaptureTolerance = Config::kBdcCaptureToleranceSteps;
bool bdcAcceptEntryEdge = false;
bool bdcSearchStarted = false;
bool bdcSearchStopping = false;
bool bdcStopObserved = false;
uint32_t bdcStoppedSinceMs = 0;
bool haveBdcEdge = false;
bool bdcCandidateLow = false;
uint32_t bdcCandidateLowSinceMs = 0;
uint32_t pedalPressedMs = 0;
bool slowSpeedReached = false;
bool haveReference = false;
bool sensorSettled = false;
bool pedalDown = false;
bool tapEligible = false;
bool delayStartForTap = false;

struct ReferenceDiagnostics {
  uint32_t rawEdgeStart = 0;
  uint32_t debounceRejectedStart = 0;
  uint32_t edges = 0;
  uint32_t staleEdges = 0;
  uint32_t ignoredEdges = 0;
  uint32_t acceptedEdges = 0;
  uint32_t lastIntervalSteps = 0;
  uint32_t lastAcceptedIntervalSteps = 0;
};

ReferenceDiagnostics referenceDiagnostics;

struct SearchDiagnostics {
  uint32_t observed = 0;
  uint32_t stale = 0;
  uint32_t outsideWindow = 0;
  uint32_t returnedHigh = 0;
  uint32_t minRejectedTravel = UINT32_MAX;
  uint32_t maxRejectedTravel = 0;
};

SearchDiagnostics searchDiagnostics;

const char* stateName(State value) {
  switch (value) {
    case State::Stopped:
      if (needlePosition == NeedlePosition::Bottom) {
        return "Stopped: needle at BDC";
      }
      if (needlePosition == NeedlePosition::Top) {
        return "Stopped: needle at TDC";
      }
      return "Stopped";
    case State::MinimumSpeed: return "Sewing: minimum speed";
    case State::VariableSpeed: return "Sewing: pedal speed";
    case State::Slowing: return "Homing: settling at homing speed";
    case State::SeekingBdc: return "Homing: acquiring BDC reference";
    case State::Positioning: return "Homing: approaching BDC";
    case State::HomingBdcSearch: return "Homing: locating the BDC sensor edge";
    case State::HomingBdcOffset: return "Homing: applying BDC offset";
    case State::Settling: return "Homing: checking final position";
    case State::MovingNeedle: return "Tap: moving needle to the opposite position";
    case State::SeekingNeedleBdc: return "Tap: locating the BDC sensor edge";
    case State::SettlingNeedle: return "Tap: checking final position";
    case State::Fault: return "FAULT: reset controller to resume";
  }
  return "Unknown state";
}

void changeState(State next) {
  if (state != next) {
    state = next;
    Serial.println(stateName(state));
  }
}

void fault(const char* reason, uint32_t nowMs) {
  const bool wasSearching = state == State::HomingBdcSearch || state == State::SeekingNeedleBdc;
  needlePosition = NeedlePosition::Unknown;
  tapEligible = false;
  haveReference = false;
  Motor::requestStop();
  faultStartedMs = nowMs;
  changeState(State::Fault);
  Serial.println(reason);
  if (wasSearching) {
    Serial.printf("BDC search edges: observed=%lu stale=%lu outside=%lu returned_HIGH=%lu latched=%s\n",
                  static_cast<unsigned long>(searchDiagnostics.observed),
                  static_cast<unsigned long>(searchDiagnostics.stale),
                  static_cast<unsigned long>(searchDiagnostics.outsideWindow),
                  static_cast<unsigned long>(searchDiagnostics.returnedHigh),
                  haveBdcEdge ? "yes" : "no");
    if (searchDiagnostics.returnedHigh > 0) {
      Serial.printf("BDC rejected edges: edge_to_stop_min=%lu edge_to_stop_max=%lu allowed=%lu\n",
                    static_cast<unsigned long>(searchDiagnostics.minRejectedTravel),
                    static_cast<unsigned long>(searchDiagnostics.maxRejectedTravel),
                    static_cast<unsigned long>(bdcCaptureTolerance));
    }
  }
}

bool isHoming() {
  return state == State::Slowing || state == State::SeekingBdc ||
         state == State::Positioning || state == State::HomingBdcSearch ||
         state == State::HomingBdcOffset || state == State::Settling;
}

bool isMovingNeedle() {
  return state == State::MovingNeedle || state == State::SeekingNeedleBdc ||
         state == State::SettlingNeedle;
}

void updateSensorDebounce() {
  const uint32_t speedHz = Motor::debounceSpeedHz();
  uint32_t intervalUs = Config::kSensorStoppedDebounceUs;
  if (speedHz > 0) {
    const uint64_t cycleFractionUs = UINT64_C(1000000) * Config::kStepsPerNeedleCycle *
                                     Config::kSensorDebouncePercent / (UINT64_C(100) * speedHz);
    intervalUs = cycleFractionUs > Config::kSensorDebounceMaxUs
                     ? Config::kSensorDebounceMaxUs : static_cast<uint32_t>(cycleFractionUs);
    if (intervalUs < Config::kSensorDebounceMinUs) {
      intervalUs = Config::kSensorDebounceMinUs;
    }
  }
  BdcSensor::setDebounceUs(intervalUs);
}

bool sensorMatches(NeedlePosition position) {
  return position != NeedlePosition::Unknown &&
         BdcSensor::isActive() == (position == NeedlePosition::Bottom);
}

bool updatePedalGesture(bool pressed, bool released, uint32_t nowMs) {
  if (!pedalDown && pressed) {
    pedalDown = true;
    pedalPressedMs = nowMs;
    tapEligible = state == State::Stopped && needlePosition != NeedlePosition::Unknown;
    delayStartForTap = tapEligible || isMovingNeedle();
  }
  if (pedalDown && released) {
    pedalDown = false;
    const uint32_t duration = nowMs - pedalPressedMs;
    const bool tapped = tapEligible && duration >= Config::kPedalTapMinMs &&
                        duration <= Config::kPedalTapMaxMs;
    tapEligible = false;
    return tapped;
  }
  return false;
}

void run(State next, int reading, uint32_t nowMs) {
  if (state == State::Stopped || isMovingNeedle()) {
    // A new sewing session must not reuse a reference from before a manual
    // handwheel adjustment or an interrupted needle toggle.
    haveReference = false;
    referenceDiagnostics = {};
    referenceDiagnostics.rawEdgeStart = BdcSensor::edgeCount();
    referenceDiagnostics.debounceRejectedStart = BdcSensor::debounceRejectedCount();
  }
  needlePosition = NeedlePosition::Unknown;
  tapEligible = false;
  const uint32_t rpm = next == State::VariableSpeed ? Pedal::rpmForReading(reading) : Config::kMinRpm;
  if (!Motor::run(rpm)) {
    fault("Motor rejected the run command", nowMs);
    return;
  }
  changeState(next);
}

void prepareBdcSearch(uint32_t start, uint32_t travelLimit) {
  searchDiagnostics = {};
  bdcSearchStart = start;
  bdcSearchLimit = travelLimit;
  bdcSearchRemaining = travelLimit;
  haveBdcEdge = false;
  bdcSearchStarted = false;
  bdcSearchStopping = false;
  bdcStopObserved = false;
  bdcCandidateLow = false;
  bdcCaptureTolerance = Config::kBdcCaptureToleranceSteps;
  bdcAcceptEntryEdge = false;
}

void startNeedleToggle(uint32_t nowMs) {
  if (needlePosition == NeedlePosition::Unknown || !Motor::isAtTarget(targetPosition) ||
      !sensorMatches(needlePosition)) {
    fault("Needle position is no longer valid; acquire a new BDC reference", nowMs);
    return;
  }
  const bool raising = needlePosition == NeedlePosition::Bottom;
  needleDestination = raising ? NeedlePosition::Top : NeedlePosition::Bottom;
  needlePosition = NeedlePosition::Unknown;
  needleMoveStartedMs = nowMs;
  if (raising) {
    haveBdcEdge = false;
    const uint32_t steps = Config::kBdcToTdcSteps - bdcParkOverrunSteps;
    if (!Motor::moveForward(steps, targetPosition)) {
      fault("Motor rejected the needle toggle", nowMs);
      return;
    }
    changeState(State::MovingNeedle);
  } else {
    // TDC has no Hall signal. Watch the entire forward return, rather than
    // using the approximate cycle length to choose where capture may begin.
    prepareBdcSearch(targetPosition, Config::kStepsPerNeedleCycle);
    BdcSensor::resetCapture();
    changeState(State::SeekingNeedleBdc);
  }
}

void trackReference(const BdcSensor::Edge& edge) {
  ++referenceDiagnostics.edges;
  if (static_cast<uint32_t>(micros() - edge.timeUs) > Config::kSensorEventMaxAgeUs) {
    ++referenceDiagnostics.staleEdges;
    return;
  }
  const uint32_t position = Motor::positionAtPulseCount(edge.pulseCount);
  if (haveReference) {
    const uint32_t distance = position - referencePosition;
    referenceDiagnostics.lastIntervalSteps = distance;
    if (distance < Config::kSensorMinSeparationSteps) {
      ++referenceDiagnostics.ignoredEdges;
      return;  // Additional edges near the same sensor crossing are bounce.
    }
    referenceDiagnostics.lastAcceptedIntervalSteps = distance;
  }
  // One captured falling edge establishes BDC. Further edges refresh it;
  // the approximate transmission ratio is not an exact-spacing acceptance test.
  referencePosition = position;
  haveReference = true;
  ++referenceDiagnostics.acceptedEdges;
}

bool referenceIsRecent() {
  return haveReference &&
         static_cast<uint32_t>(Motor::position() - referencePosition) <= Config::kReferenceMaxTravelSteps;
}

void positionFromReference(uint32_t nowMs) {
  if (!Motor::moveToBdcApproach(referencePosition, targetPosition)) {
    fault("Motor rejected the BDC target", nowMs);
    return;
  }
  // A noisy reference may put the approach on the wrong part of the cycle.
  // Search forward for a full cycle, as with the return tap, rather than
  // requiring the real edge to lie within +/- 93 steps of that prediction.
  prepareBdcSearch(targetPosition, Config::kStepsPerNeedleCycle);
  changeState(State::Positioning);
}

void startHoming(uint32_t nowMs) {
  homingStartedMs = nowMs;
  const uint32_t speedHz = Motor::debounceSpeedHz();
  if (speedHz > 0 && speedHz <= Config::kPositionFeedbackMaxHz && Motor::isAtHomingSpeed()) {
    // Begin capture at release, without a predicted approach point or a
    // prerequisite reference-acquisition stitch. The finite endpoint prevents
    // endless motion if the next Hall crossing is missing.
    const bool useCurrentEdge = referenceIsRecent() && BdcSensor::isActive() &&
        static_cast<uint32_t>(Motor::position() - referencePosition) <=
            Config::bdcCaptureToleranceSteps(speedHz);
    const uint32_t currentEdge = referencePosition;
    // Do not erase an interrupt that arrived after this update's first read.
    BdcSensor::resetCapture(false);
    uint32_t scanStart = 0;
    if (!Motor::scanForward(Config::kStepsPerNeedleCycle, scanStart, targetPosition)) {
      fault("Motor rejected the direct low-speed BDC scan", nowMs);
      return;
    }
    prepareBdcSearch(scanStart, Config::kStepsPerNeedleCycle);
    bdcSearchStarted = true;
    // Already queued motion may still run at the release speed for a short
    // time. Bound braking using that speed, rather than assuming 60 RPM yet.
    bdcCaptureTolerance = Config::bdcCaptureToleranceSteps(speedHz);
    bdcAcceptEntryEdge = true;
    if (useCurrentEdge) {
      bdcEdgePosition = currentEdge;
      haveBdcEdge = true;
      Motor::requestStop();
      bdcSearchStopping = true;
    }
    changeState(State::HomingBdcSearch);
    Serial.println("Homing: direct low-speed BDC scan");
    return;
  }
  if (!Motor::run(Config::kHomingRpm)) {
    fault("Motor rejected the homing speed", nowMs);
    return;
  }
  slowSpeedReached = false;
  changeState(State::Slowing);
}

void homingTimedOut(uint32_t nowMs) {
  const State timedOutState = state;
  const uint32_t position = Motor::position();
  const uint16_t pulses = Motor::pulseCount();
  const bool sensorActive = BdcSensor::isActive();
  const uint32_t rawEdges = BdcSensor::edgeCount() - referenceDiagnostics.rawEdgeStart;
  const uint32_t debounced = BdcSensor::debounceRejectedCount() - referenceDiagnostics.debounceRejectedStart;
  const char* reason = "Homing timed out";
  switch (timedOutState) {
    case State::Slowing:
      reason = "Homing timed out before the motor settled at homing speed";
      break;
    case State::SeekingBdc:
      reason = rawEdges == 0 ? "Homing timed out: no BDC falling-edge interrupts received"
                            : "Homing timed out: BDC interrupts received, but no recent reference available";
      break;
    case State::HomingBdcSearch:
      reason = "Homing timed out while locating a stable BDC sensor edge";
      break;
    case State::HomingBdcOffset:
    case State::Positioning:
      reason = "Homing timed out before the BDC move completed";
      break;
    case State::Settling:
      reason = "Homing reached its pulse target, but the BDC sensor did not stay LOW";
      break;
    default:
      break;
  }
  fault(reason, nowMs);
  // Print only after requesting a stop, so diagnostics cannot delay capture.
  Serial.printf("Homing diagnostics (since sewing started): GPIO=%u level=%s raw=%lu debounced=%lu captured=%lu stale=%lu ignored=%lu accepted=%lu\n",
                Config::kBdcPin, sensorActive ? "LOW" : "HIGH",
                static_cast<unsigned long>(rawEdges),
                static_cast<unsigned long>(debounced),
                static_cast<unsigned long>(referenceDiagnostics.edges),
                static_cast<unsigned long>(referenceDiagnostics.staleEdges),
                static_cast<unsigned long>(referenceDiagnostics.ignoredEdges),
                static_cast<unsigned long>(referenceDiagnostics.acceptedEdges));
  Serial.printf("BDC tracking: last_interval=%lu accepted_interval=%lu nominal_cycle=%lu; position=%lu pulse_count=%u\n",
                static_cast<unsigned long>(referenceDiagnostics.lastIntervalSteps),
                static_cast<unsigned long>(referenceDiagnostics.lastAcceptedIntervalSteps),
                static_cast<unsigned long>(Config::kStepsPerNeedleCycle),
                static_cast<unsigned long>(position), pulses);
}

void updatePositioning(State settlingState, uint32_t nowMs) {
  if (Motor::isRunning()) {
    return;
  }
  if (!Motor::isAtTarget(targetPosition)) {
    fault("Motor did not finish at the commanded needle position", nowMs);
    return;
  }
  sensorSettled = false;
  changeState(settlingState);
}

void rememberBdcEdge(const BdcSensor::Edge& edge) {
  ++searchDiagnostics.observed;
  if (haveBdcEdge) {
    return;
  }
  if (static_cast<uint32_t>(micros() - edge.timeUs) > Config::kSensorEventMaxAgeUs) {
    ++searchDiagnostics.stale;
    return;
  }
  const uint32_t position = Motor::positionAtPulseCount(edge.pulseCount);
  const bool inScan = static_cast<uint32_t>(position - bdcSearchStart) <= bdcSearchLimit;
  const bool atEntry = bdcAcceptEntryEdge && BdcSensor::isActive() &&
      static_cast<uint32_t>(bdcSearchStart - position) <= bdcCaptureTolerance;
  if (inScan || atEntry) {
    bdcEdgePosition = position;
    haveBdcEdge = true;
    bdcCandidateLow = false;
  } else {
    ++searchDiagnostics.outsideWindow;
  }
}

void updateBdcSearch(State offsetState, State settlingState, uint32_t nowMs) {
  BdcSensor::Edge edge = {};
  if (BdcSensor::takeEdge(edge)) {
    rememberBdcEdge(edge);
  }
  if (haveBdcEdge && Motor::isRunning() && !bdcSearchStopping) {
    Motor::requestStop();
    bdcSearchStopping = true;
  }
  if (Motor::isRunning()) {
    bdcStopObserved = false;
    return;
  }
  // A sensor-triggered stop ends before the finite scan's original endpoint.
  // Preserve its actual coordinate; never rebase either pulse counter.
  if (bdcSearchStopping) {
    targetPosition = Motor::position();
    bdcSearchStopping = false;
  }
  if (!Motor::isAtTarget(targetPosition)) {
    fault("BDC search stopped at an unexpected pulse count", nowMs);
    return;
  }
  const uint32_t travel = targetPosition - bdcSearchStart;
  if (travel > bdcSearchLimit) {
    fault("BDC search exceeded its travel limit", nowMs);
    return;
  }
  bdcSearchRemaining = bdcSearchLimit - travel;
  if (!bdcStopObserved) {
    bdcStopObserved = true;
    bdcStoppedSinceMs = nowMs;
  }
  if (haveBdcEdge && !BdcSensor::isActive()) {
    bdcCandidateLow = false;
    if (static_cast<uint32_t>(nowMs - bdcStoppedSinceMs) < Config::kBdcStopQualificationMs) {
      return;
    }
    ++searchDiagnostics.returnedHigh;
    const uint32_t traveled = targetPosition - bdcEdgePosition;
    if (traveled < searchDiagnostics.minRejectedTravel) {
      searchDiagnostics.minRejectedTravel = traveled;
    }
    if (traveled > searchDiagnostics.maxRejectedTravel) {
      searchDiagnostics.maxRejectedTravel = traveled;
    }
    haveBdcEdge = false;
    bdcCandidateLow = false;
    bdcSearchStarted = false;
  }
  if (haveBdcEdge) {
    if (!bdcCandidateLow) {
      bdcCandidateLow = true;
      bdcCandidateLowSinceMs = nowMs;
      return;
    }
    if (static_cast<uint32_t>(nowMs - bdcCandidateLowSinceMs) < Config::kStopSettleMs) {
      return;
    }
    const uint32_t traveled = targetPosition - bdcEdgePosition;
    const int64_t remaining = static_cast<int64_t>(Config::kBdcOffsetSteps) - traveled;
    if (remaining < -static_cast<int64_t>(bdcCaptureTolerance)) {
      fault("BDC capture exceeded the allowed braking travel", nowMs);
      Serial.printf("BDC capture: edge=%lu stopped=%lu offset=%lu tolerance=%lu\n",
                    static_cast<unsigned long>(bdcEdgePosition),
                    static_cast<unsigned long>(targetPosition),
                    static_cast<unsigned long>(Config::kBdcOffsetSteps),
                    static_cast<unsigned long>(bdcCaptureTolerance));
    } else if (remaining > 0) {
      if (!Motor::moveForward(static_cast<uint32_t>(remaining), targetPosition)) {
        fault("Motor rejected the BDC calibration offset", nowMs);
      } else {
        changeState(offsetState);
      }
    } else {
      sensorSettled = false;
      changeState(settlingState);
    }
    return;
  }
  if (bdcSearchRemaining == 0 || bdcSearchStarted) {
    fault("No stable BDC edge in the positioning search window; check reference and sensor", nowMs);
    Serial.printf("BDC search: start=%lu stopped=%lu limit=%lu level=%s accepted_interval=%lu nominal_cycle=%lu raw=%lu debounced=%lu\n",
                  static_cast<unsigned long>(bdcSearchStart),
                  static_cast<unsigned long>(targetPosition),
                  static_cast<unsigned long>(bdcSearchLimit),
                  BdcSensor::isActive() ? "LOW" : "HIGH",
                  static_cast<unsigned long>(referenceDiagnostics.lastAcceptedIntervalSteps),
                  static_cast<unsigned long>(Config::kStepsPerNeedleCycle),
                  static_cast<unsigned long>(BdcSensor::edgeCount() - referenceDiagnostics.rawEdgeStart),
                  static_cast<unsigned long>(BdcSensor::debounceRejectedCount() - referenceDiagnostics.debounceRejectedStart));
    return;
  }
  // One finite scan at the same speed as a BDC/TDC toggle. Its endpoint bounds
  // travel even if the Hall signal is missing; capture requests normal braking.
  if (!Motor::moveForward(bdcSearchRemaining, targetPosition)) {
    fault("Motor rejected the BDC search move", nowMs);
    return;
  }
  bdcStopObserved = false;
  bdcSearchStarted = true;
}

void updateSettling(NeedlePosition destination, uint32_t nowMs) {
  if (!Motor::isAtTarget(targetPosition)) {
    fault("Position verification failed; the final pulse count changed", nowMs);
  } else if (!sensorMatches(destination)) {
    sensorSettled = false;
  } else if (!sensorSettled) {
    sensorSettled = true;
    settledSinceMs = nowMs;
  } else if (static_cast<uint32_t>(nowMs - settledSinceMs) >= Config::kStopSettleMs) {
    if (!Motor::release()) {
      fault("Motor outputs could not be released", nowMs);
    } else {
      if (destination == NeedlePosition::Bottom) {
        bdcParkOverrunSteps = targetPosition - (bdcEdgePosition + Config::kBdcOffsetSteps);
        Serial.printf("BDC stop: edge_to_stop=%lu offset=%lu\n",
                      static_cast<unsigned long>(targetPosition - bdcEdgePosition),
                      static_cast<unsigned long>(Config::kBdcOffsetSteps));
      }
      needlePosition = destination;
      changeState(State::Stopped);
    }
  }
}

void needleMoveTimedOut(uint32_t nowMs) {
  const State timedOutState = state;
  const bool sensorActive = BdcSensor::isActive();
  const uint32_t position = Motor::position();
  const uint16_t pulses = Motor::pulseCount();
  const char* reason = "Needle toggle timed out before motion completed";
  if (timedOutState == State::SeekingNeedleBdc) {
    reason = "Needle toggle timed out while searching for the BDC edge";
  } else if (timedOutState == State::SettlingNeedle) {
    reason = needleDestination == NeedlePosition::Bottom
                 ? "Needle reached its target, but the BDC sensor did not stay LOW"
                 : "Needle reached its target, but the BDC sensor did not stay HIGH at TDC";
  }
  fault(reason, nowMs);
  Serial.printf("Tap diagnostics: GPIO=%u level=%s position=%lu target=%lu pulse_count=%u BDC_edge=%s search_remaining=%lu\n",
                Config::kBdcPin, sensorActive ? "LOW" : "HIGH",
                static_cast<unsigned long>(position),
                static_cast<unsigned long>(targetPosition), pulses,
                haveBdcEdge ? "captured" : "none",
                static_cast<unsigned long>(bdcSearchRemaining));
}

void updateHoming(uint32_t nowMs) {
  switch (state) {
    case State::Slowing:
      if (!Motor::isAtHomingSpeed()) {
        slowSpeedReached = false;
      } else if (!slowSpeedReached) {
        slowSpeedReached = true;
        slowSinceMs = nowMs;
      } else if (static_cast<uint32_t>(nowMs - slowSinceMs) >= Config::kHomingSpeedSettleMs) {
        // Pulse positions captured during fast motion/deceleration are not
        // used for final planning. Start one fresh acquisition at settled speed.
        haveReference = false;
        BdcSensor::resetCapture();
        changeState(State::SeekingBdc);
      }
      break;

    case State::SeekingBdc:
      if (referenceIsRecent()) {
        positionFromReference(nowMs);
      }
      break;

    case State::Positioning:
      updatePositioning(State::HomingBdcSearch, nowMs);
      break;

    case State::HomingBdcSearch:
      updateBdcSearch(State::HomingBdcOffset, State::Settling, nowMs);
      break;

    case State::HomingBdcOffset:
      updatePositioning(State::Settling, nowMs);
      break;

    case State::Settling:
      updateSettling(NeedlePosition::Bottom, nowMs);
      break;

    default:
      break;
  }
}

}  // namespace

namespace Machine {

void begin() {
  if (!Motor::begin()) {
    fault("Motor or pulse counter initialization failed", millis());
  } else if (!BdcSensor::begin()) {
    fault("BDC sensor initialization failed", millis());
  } else {
    Serial.println("Ready; tracking BDC pulses while sewing for the next stop");
  }
}

static void updateState(int reading, uint32_t nowMs) {
  BdcSensor::Edge edge = {};
  const bool haveEdge = BdcSensor::takeEdge(edge);

  if (state == State::Fault) {
    if (Motor::isRunning() && static_cast<uint32_t>(nowMs - faultStartedMs) >= Config::kFaultStopTimeoutMs) {
      Motor::abort();
    }
    Motor::release();
    return;
  }

  if (haveEdge && (state == State::MinimumSpeed || state == State::VariableSpeed ||
                   state == State::Slowing || state == State::SeekingBdc)) {
    trackReference(edge);
  }
  if (haveEdge && (state == State::SeekingNeedleBdc || state == State::Positioning ||
                   state == State::HomingBdcSearch)) {
    rememberBdcEdge(edge);
  }

  const bool pressed = reading > Config::kPedalEnableThreshold + Config::kPedalHysteresis;
  const bool released = reading < Config::kPedalEnableThreshold - Config::kPedalHysteresis;
  const bool variableSpeed = reading > Config::kPedalVariableThreshold + Config::kPedalHysteresis;

  // A handwheel crossing or a changed sensor level invalidates a parked
  // reference. STEP pulses alone cannot detect manual shaft movement.
  if (state == State::Stopped && needlePosition != NeedlePosition::Unknown &&
      (haveEdge || !Motor::isAtTarget(targetPosition) || !sensorMatches(needlePosition))) {
    needlePosition = NeedlePosition::Unknown;
    tapEligible = false;
    Serial.println("Needle reference lost; sew and release the pedal to reacquire BDC");
  }

  const bool tapped = updatePedalGesture(pressed, released, nowMs);
  const bool held = pedalDown &&
                    static_cast<uint32_t>(nowMs - pedalPressedMs) >= Config::kPedalTapMaxMs;

  if (isMovingNeedle()) {
    const uint32_t timeout = Config::kNeedleMoveTimeoutMs +
                            (needleDestination == NeedlePosition::Bottom
                                 ? Config::kBdcSearchTimeoutMs : 0);
    if (static_cast<uint32_t>(nowMs - needleMoveStartedMs) >= timeout) {
      needleMoveTimedOut(nowMs);
    } else if (held) {
      run(variableSpeed ? State::VariableSpeed : State::MinimumSpeed, reading, nowMs);
    } else if (state == State::MovingNeedle) {
      updatePositioning(State::SettlingNeedle, nowMs);
    } else if (state == State::SeekingNeedleBdc) {
      updateBdcSearch(State::MovingNeedle, State::SettlingNeedle, nowMs);
    } else {
      updateSettling(needleDestination, nowMs);
    }
    return;
  }

  if (isHoming()) {
    if (static_cast<uint32_t>(nowMs - homingStartedMs) >= Config::kHomingTimeoutMs) {
      homingTimedOut(nowMs);
    } else if (pressed && pedalDown &&
               static_cast<uint32_t>(nowMs - pedalPressedMs) >= Config::kHomingResumePressMs) {
      run(variableSpeed ? State::VariableSpeed : State::MinimumSpeed, reading, nowMs);
    } else {
      updateHoming(nowMs);
    }
    return;
  }

  switch (state) {
    case State::Stopped:
      if (tapped && needlePosition != NeedlePosition::Unknown) {
        startNeedleToggle(nowMs);
      } else if (pedalDown && (!delayStartForTap || held)) {
        run(variableSpeed ? State::VariableSpeed : State::MinimumSpeed, reading, nowMs);
      }
      break;

    case State::MinimumSpeed:
      if (released) {
        startHoming(nowMs);
      } else if (variableSpeed) {
        run(State::VariableSpeed, reading, nowMs);
      }
      break;

    case State::VariableSpeed:
      if (released) {
        startHoming(nowMs);
      } else if (reading < Config::kPedalVariableThreshold - Config::kPedalHysteresis) {
        run(State::MinimumSpeed, reading, nowMs);
      } else {
        run(State::VariableSpeed, reading, nowMs);
      }
      break;

    default:
      fault("Unexpected machine state", nowMs);
      break;
  }
}

void update(int reading, uint32_t nowMs) {
  updateSensorDebounce();
  updateState(reading, nowMs);
  // Publish new speed/state limits immediately after a motor command.
  updateSensorDebounce();
}

}  // namespace Machine
