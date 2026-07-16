#include "host_link.h"

#include <Arduino.h>

#include "board_pins.h"
#include "config.h"
#include "protocol.h"

namespace {

constexpr size_t kBufferLen = 128;
char g_rx_buffer[kBufferLen];
size_t g_rx_len = 0;

bool sequence_is_newer(uint32_t sequence, const HostState& host) {
  return !host.has_accepted_sequence ||
         static_cast<int32_t>(sequence - host.last_accepted_sequence) > 0;
}

void accept_sequence(HostState& host, uint32_t sequence) {
  host.last_accepted_sequence = sequence;
  host.has_accepted_sequence = true;
}

void handle_line(SystemState& state, const char* line, uint32_t now_ms) {
  board::debug_uart.print("RX: ");
  board::debug_uart.println(line);

  const protocol::ParseResult result = protocol::parse_host_line(line, now_ms);
  if (!result.ok) {
    board::host_uart.println("ERR,INVALID");
    return;
  }

  if (result.is_hello) {
    state.host.handshake_complete = true;
    state.host.has_accepted_sequence = false;
    state.host.velocity_command = {};
    board::host_uart.print("ACK,HELLO,");
    board::host_uart.println(config::kProtocolVersion);
    return;
  }

  if (!state.host.handshake_complete) {
    board::host_uart.println("ERR,NO_HELLO");
    return;
  }

  if (!result.is_velocity) {
    board::host_uart.println("ERR,INVALID");
    return;
  }

  const uint32_t sequence = result.velocity.sequence;
  if (!sequence_is_newer(sequence, state.host)) {
    board::host_uart.println("ERR,SEQUENCE");
    return;
  }

  state.host.velocity_command = result.velocity;

  accept_sequence(state.host, sequence);
  board::host_uart.print("ACK,CMD,");
  board::host_uart.println(sequence);
}

}  // namespace

void setup_host_link() {
  board::debug_uart.begin(config::kHostBaud);
  board::host_uart.begin(config::kHostBaud);
}

void update_host_link(SystemState& state, uint32_t now_ms) {
  while (board::host_uart.available() > 0) {
    const char c = static_cast<char>(board::host_uart.read());
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

  char buffer[320];
  protocol::format_telemetry(state, now_ms, buffer, sizeof(buffer));
  board::host_uart.println(buffer);
}
