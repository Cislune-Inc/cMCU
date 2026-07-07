#include <Arduino.h>

#include "config.h"
#include "control_logic.h"
#include "host_link.h"
#include "radio_link.h"
#include "roboclaw_link.h"
#include "system_state.h"

namespace {

SystemState g_state = {};

}  // namespace

void setup() {
  g_state.startup_ms = millis();

  setup_host_link();
  setup_radio_link();
  setup_roboclaw_link(g_state);
}

void loop() {
  const uint32_t now_ms = millis();

  update_radio_link(g_state, now_ms);
  update_host_link(g_state, now_ms);
  update_roboclaw_telemetry(g_state, now_ms);
  control_logic::update_state(g_state, now_ms);
  apply_motor_outputs(g_state, now_ms);
  send_telemetry(g_state, now_ms);

  delay(config::kLoopDelayMs);
}
