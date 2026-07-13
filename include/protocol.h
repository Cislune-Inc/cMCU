#pragma once

#include <stddef.h>

#include "system_state.h"

namespace protocol {

struct ParseResult {
  bool ok = false;
  bool is_hello = false;
  bool is_velocity = false;
  bool is_duty_test = false;
  MotionCommand velocity = {};
  DutyTestCommand duty_test = {};
};

ParseResult parse_host_line(const char* line, uint32_t now_ms);
void format_telemetry(const SystemState& state, uint32_t now_ms,
                      char* buffer, size_t buffer_len);

}  // namespace protocol
