#pragma once

#include <stdint.h>

namespace Pedal {

void begin();
// Call once per control interval. Returns the filtered 12-bit ADC value.
int sample();
uint32_t rpmForReading(int reading);

}  // namespace Pedal
