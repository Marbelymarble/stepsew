#include "bdc_sensor.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_timer.h>
#include <soc/pcnt_struct.h>

#include "config.h"

namespace {

portMUX_TYPE edgeMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool edgePending = false;
volatile uint32_t fallingEdgeCount = 0;
volatile uint32_t debounceRejectedEdges = 0;
volatile uint32_t deliveredEdges = 0;
volatile uint32_t coalescedEdges = 0;
volatile uint32_t lastAdmittedEdgeUs = 0;
volatile bool haveAdmittedEdge = false;
volatile uint32_t debounceUs = Config::kSensorStoppedDebounceUs;
volatile uint32_t lastAdmittedDebounceUs = Config::kSensorStoppedDebounceUs;
volatile uint16_t edgePulseCount = 0;
volatile uint32_t edgeTimeUs = 0;

void IRAM_ATTR onFallingEdge(void*) {
  // Read hardware directly: no motor-library calls or position resets in the ISR.
  const uint16_t pulseCount = PCNT.cnt_unit[Config::kPositionCounterUnit].cnt_val;
  // Arduino's micros() resides in flash in this build; esp_timer_get_time() is
  // in IRAM and remains callable when the flash cache is disabled.
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  portENTER_CRITICAL_ISR(&edgeMux);
  ++fallingEdgeCount;
  // Rejected bounce does not extend the lockout. Preserve the first edge's
  // position instead of delaying capture until the burst finishes.
  // Acceleration may shorten an active lockout; deceleration must not extend
  // one that started at a faster speed and hide the next real crossing.
  const uint32_t lockoutUs = debounceUs < lastAdmittedDebounceUs
                                 ? debounceUs : lastAdmittedDebounceUs;
  if (haveAdmittedEdge &&
      static_cast<uint32_t>(now - lastAdmittedEdgeUs) < lockoutUs) {
    ++debounceRejectedEdges;
    portEXIT_CRITICAL_ISR(&edgeMux);
    return;
  }
  haveAdmittedEdge = true;
  lastAdmittedEdgeUs = now;
  lastAdmittedDebounceUs = debounceUs;
  // Keep the first edge until the loop consumes it. Subsequent bounce must
  // not replace its timestamp or pulse count. No pulse-width assumption:
  // even an input that has returned HIGH before this ISR runs is captured.
  if (!edgePending) {
    edgePulseCount = pulseCount;
    edgeTimeUs = now;
    edgePending = true;
  } else {
    ++coalescedEdges;
  }
  portEXIT_CRITICAL_ISR(&edgeMux);
}

}  // namespace

namespace BdcSensor {

bool begin() {
  gpio_config_t config = {};
  config.pin_bit_mask = UINT64_C(1) << Config::kBdcPin;
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_NEGEDGE;
  if (gpio_config(&config) != ESP_OK) {
    return false;
  }
  const esp_err_t installed = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (installed != ESP_OK && installed != ESP_ERR_INVALID_STATE) {
    return false;
  }
  const gpio_num_t pin = static_cast<gpio_num_t>(Config::kBdcPin);
  return gpio_isr_handler_add(pin, onFallingEdge, nullptr) == ESP_OK;
}

void resetCapture(bool discardPending) {
  portENTER_CRITICAL(&edgeMux);
  if (discardPending) {
    edgePending = false;
  }
  haveAdmittedEdge = false;
  portEXIT_CRITICAL(&edgeMux);
}

void setDebounceUs(uint32_t intervalUs) {
  portENTER_CRITICAL(&edgeMux);
  debounceUs = intervalUs;
  portEXIT_CRITICAL(&edgeMux);
}

bool takeEdge(Edge& edge) {
  portENTER_CRITICAL(&edgeMux);
  const bool pending = edgePending;
  if (pending) {
    ++deliveredEdges;
  }
  edge.pulseCount = edgePulseCount;
  edge.timeUs = edgeTimeUs;
  edgePending = false;
  portEXIT_CRITICAL(&edgeMux);
  return pending;
}

uint32_t edgeCount() {
  portENTER_CRITICAL(&edgeMux);
  const uint32_t count = fallingEdgeCount;
  portEXIT_CRITICAL(&edgeMux);
  return count;
}

uint32_t debounceRejectedCount() {
  portENTER_CRITICAL(&edgeMux);
  const uint32_t count = debounceRejectedEdges;
  portEXIT_CRITICAL(&edgeMux);
  return count;
}

bool isActive() {
  return digitalRead(Config::kBdcPin) == LOW;
}

void reportDiagnostics(uint32_t nowMs) {
  static uint32_t lastReportMs = 0;
  if (static_cast<uint32_t>(nowMs - lastReportMs) < 1000) {
    return;
  }
  lastReportMs = nowMs;
  // Snapshot together, then format outside the ISR lock. Raw counts include
  // every interrupt, even if debounce or pending-edge coalescing rejects it.
  portENTER_CRITICAL(&edgeMux);
  const uint32_t raw = fallingEdgeCount;
  const uint32_t rejected = debounceRejectedEdges;
  const uint32_t delivered = deliveredEdges;
  const uint32_t merged = coalescedEdges;
  const uint32_t interval = debounceUs;
  portEXIT_CRITICAL(&edgeMux);
  char line[128];
  const int length = snprintf(line, sizeof(line),
      "BDC pin=%u %s raw=%lu db=%lu read=%lu merged=%lu us=%lu\n",
      Config::kBdcPin, isActive() ? "LOW" : "HIGH",
      static_cast<unsigned long>(raw), static_cast<unsigned long>(rejected),
      static_cast<unsigned long>(delivered), static_cast<unsigned long>(merged),
      static_cast<unsigned long>(interval));
  if (length > 0 && static_cast<size_t>(length) < sizeof(line) &&
      Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(length));
  }
}

}  // namespace BdcSensor
