#pragma once

#include <stdint.h>

namespace BdcSensor {

struct Edge {
  uint16_t pulseCount;
  uint32_t timeUs;
};

bool begin();
// Restart debounce; optionally retain an edge already captured during motion.
// Diagnostic counters are retained in either case.
void resetCapture(bool discardPending = true);
// Change the active lockout without clearing the original edge or its timestamp.
void setDebounceUs(uint32_t intervalUs);
// Returns the first captured edge outside the lockout, regardless of pulse width.
bool takeEdge(Edge& edge);
// Raw falling-edge interrupts since startup, including coalesced bounce.
uint32_t edgeCount();
// Falling edges suppressed by the time-based lockout since startup.
uint32_t debounceRejectedCount();
bool isActive();
// Once-per-second status; skips output when the serial buffer is full.
void reportDiagnostics(uint32_t nowMs);

}  // namespace BdcSensor
