#include "radio_link.h"

#include <Arduino.h>

#include "config.h"
#include "protocol.h"

namespace {

constexpr size_t kBufferLen = 96;
char g_radio_buffer[kBufferLen];
size_t g_radio_len = 0;

}  // namespace

void setup_radio_link() {
  Serial3.begin(config::kRadioBaud);
}

void update_radio_link(SystemState& state, uint32_t now_ms) {
  while (Serial3.available() > 0) {
    const char c = static_cast<char>(Serial3.read());
    if (c == '\r') {
      continue;
    }
    if (c != '\n') {
      if (g_radio_len + 1 < kBufferLen) {
        g_radio_buffer[g_radio_len++] = c;
      } else {
        g_radio_len = 0;
      }
      continue;
    }

    g_radio_buffer[g_radio_len] = '\0';
    if (g_radio_len > 0) {
      protocol::parse_radio_line(g_radio_buffer, state.radio, now_ms);
    }
    g_radio_len = 0;
  }
}
