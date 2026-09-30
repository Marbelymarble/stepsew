#pragma once

#include <stdint.h>

namespace Motor {

bool begin();
bool run(uint32_t rpm);
bool isRunning();
bool isAtHomingSpeed();
// Highest actual, queued, or requested pulse rate; zero at rest.
uint32_t debounceSpeedHz();
void requestStop();
void abort();
bool release();

// Positions use unsigned arithmetic so forward distances survive 32-bit wrap.
uint32_t position();
uint16_t pulseCount();
uint32_t positionAtPulseCount(uint16_t pulseCount);
// At homing speed, plan a reachable stop before the estimated next BDC edge.
bool moveToBdcApproach(uint32_t reference, uint32_t& target);
// Convert low-speed continuous motion into a bounded scan at needle-move speed.
// Returns both coordinates so capture and travel limits use the same origin.
bool scanForward(uint32_t steps, uint32_t& start, uint32_t& target);
// A finite forward move at the minimum sewing speed, starting from standstill.
bool moveForward(uint32_t steps, uint32_t& target);
bool isAtTarget(uint32_t target);

}  // namespace Motor
