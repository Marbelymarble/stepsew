#pragma once

#include <stdint.h>

namespace Machine {

void begin();
void update(int pedalReading, uint32_t nowMs);

}  // namespace Machine
