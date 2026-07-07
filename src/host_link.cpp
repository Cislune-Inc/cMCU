#include "host_link.h"

#include <Arduino.h>

#include "config.h"
#include "protocol.h"

namespace {

constexpr size_t kBufferLen = 96;
char g_rx_buffer[kBufferLen];
size_t g_rx_len = 0;

void handle_line(SystemState& state, const char* line, uint32_t now_ms) {
  const protocol::ParseResult result = protocol::parse_host_line(line, now_ms);
  if (!result.ok) {
    Serial.println("ERR,H");
    return;
  }

  if (result.is_ping) {
    state.host.last_pong_ms = now_ms;
    state.host.linked = true;
    state.host.handshake_complete = true;
    Serial.println("ACK,PONG");
    return;
  }

  if (!state.host.handshake_complete) {
    Serial.println("ERR,WAIT_PONG");
    return;
  }

  state.host.last_packet_ms = now_ms;
  state.host.linked = true;

  if (result.is_mode_select) {
    state.host.selected_mode = result.selected_mode;
    Serial.print("ACK,MODE,");
    Serial.println(static_cast<char>(state.host.selected_mode));
    return;
  }
  if (result.is_drive_mode) {
    state.host.requested_drive_mode = result.drive_mode;
    Serial.print("ACK,DRIVE,");
    Serial.println(static_cast<char>(state.host.requested_drive_mode));
    return;
  }
  if (!result.is_motion) {
    return;
  }

  result.motion.source_mode = state.host.selected_mode;

  if (state.host.selected_mode == OperatingMode::KEYBOARD) {
    state.host.keyboard_command = result.motion;
  } else if (state.host.selected_mode == OperatingMode::AUTONOMOUS) {
    state.host.autonomous_command = result.motion;
  }
  Serial.println("ACK,CMD");
}

}  // namespace

void setup_host_link() {
  Serial.begin(config::kHostBaud);
}

void update_host_link(SystemState& state, uint32_t now_ms) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c != '\n') {
      if (g_rx_len + 1 < kBufferLen) {
        g_rx_buffer[g_rx_len++] = c;
      } else {
        g_rx_len = 0;
      }
      continue;
    }

    g_rx_buffer[g_rx_len] = '\0';
    if (g_rx_len > 0) {
      handle_line(state, g_rx_buffer, now_ms);
    }
    g_rx_len = 0;
  }
}

void send_telemetry(SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.last_telemetry_ms) < config::kTelemetryPeriodMs) {
    return;
  }

  state.last_telemetry_ms = now_ms;
  ++state.telemetry_sequence;

  char buffer[160];
  protocol::format_telemetry(state, buffer, sizeof(buffer));
  Serial.println(buffer);
}
