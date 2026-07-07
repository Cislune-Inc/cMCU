#pragma once

#include <stddef.h>

#include "system_state.h"

namespace protocol {

struct ParseResult {
  bool ok = false;
  bool is_ping = false;
  bool is_mode_select = false;
  bool is_drive_mode = false;
  bool is_motion = false;
  OperatingMode selected_mode = OperatingMode::KEYBOARD;
  DriveMode drive_mode = DriveMode::DUTY;
  MotionCommand motion = {};
};

ParseResult parse_host_line(const char* line, uint32_t now_ms);
bool parse_radio_line(const char* line, RadioState& radio, uint32_t now_ms);
void format_telemetry(const SystemState& state, char* buffer, size_t buffer_len);

}  // namespace protocol
