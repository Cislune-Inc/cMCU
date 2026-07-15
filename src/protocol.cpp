#include "protocol.h"

#include <math.h>
#include <stdio.h>

#include "config.h"

namespace protocol {
namespace {

const char* mode_name(OperatingMode mode) {
  switch (mode) {
    case OperatingMode::DISABLED:
      return "DISABLED";
    case OperatingMode::BODY_VELOCITY:
      return "VELOCITY";
    case OperatingMode::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

const char* fault_name(FaultCode fault) {
  switch (fault) {
    case FaultCode::OK:
      return "NONE";
    case FaultCode::STARTUP:
      return "STARTUP";
    case FaultCode::HOST_TIMEOUT:
      return "HOST";
    case FaultCode::BATTERY:
      return "BATTERY";
    case FaultCode::ROBOCLAW:
      return "ROBOCLAW";
    case FaultCode::FEEDBACK:
      return "FEEDBACK";
  }
  return "UNKNOWN";
}

}  // namespace

ParseResult parse_host_line(const char* line, uint32_t now_ms) {
  ParseResult result = {};
  unsigned long version = 0;
  int end = 0;
  if (sscanf(line, "HELLO,%lu%n", &version, &end) == 1 && line[end] == '\0') {
    result.ok = version == config::kProtocolVersion;
    result.is_hello = result.ok;
    return result;
  }

  unsigned long sequence = 0;
  float linear_mps = 0.0f;
  float angular_radps = 0.0f;
  if (sscanf(line, "CMD_VEL,%lu,%f,%f%n", &sequence, &linear_mps,
             &angular_radps, &end) == 3 &&
      line[end] == '\0' && isfinite(linear_mps) && isfinite(angular_radps)) {
    result.ok = true;
    result.is_velocity = true;
    result.velocity.linear_mps = linear_mps;
    result.velocity.angular_radps = angular_radps;
    result.velocity.sequence = static_cast<uint32_t>(sequence);
    result.velocity.received_ms = now_ms;
    result.velocity.valid = true;
    return result;
  }

  return result;
}

void format_telemetry(const SystemState& state, uint32_t now_ms,
                      char* buffer, size_t buffer_len) {
  uint8_t encoder_flags = 0;
  uint8_t speed_flags = 0;
  for (size_t i = 0; i < kWheelCount; ++i) {
    if (state.telemetry.encoder_valid[i]) {
      encoder_flags |= static_cast<uint8_t>(1U << i);
    }
    if (state.telemetry.speed_valid[i]) {
      speed_flags |= static_cast<uint8_t>(1U << i);
    }
  }

  snprintf(
      buffer, buffer_len,
      "TLM,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.1f,%u,%u,%u,%u,%u,%lu,%lu,%u,%s,%s",
      static_cast<unsigned long>(config::kProtocolVersion),
      static_cast<unsigned long>(state.telemetry_sequence),
      static_cast<unsigned long>(now_ms),
      static_cast<unsigned long>(state.host.last_accepted_sequence),
      static_cast<long>(state.telemetry.encoder_counts[0]),
      static_cast<long>(state.telemetry.encoder_counts[1]),
      static_cast<long>(state.telemetry.encoder_counts[2]),
      static_cast<long>(state.telemetry.encoder_counts[3]),
      static_cast<long>(state.telemetry.measured_qpps[0]),
      static_cast<long>(state.telemetry.measured_qpps[1]),
      static_cast<long>(state.telemetry.measured_qpps[2]),
      static_cast<long>(state.telemetry.measured_qpps[3]),
      static_cast<long>(state.wheel_outputs.qpps[0]),
      static_cast<long>(state.wheel_outputs.qpps[1]),
      static_cast<long>(state.wheel_outputs.qpps[2]),
      static_cast<long>(state.wheel_outputs.qpps[3]),
      static_cast<double>(state.telemetry.battery_voltage),
      state.telemetry.battery_valid ? 1U : 0U, encoder_flags, speed_flags,
      state.front_roboclaw.healthy ? 1U : 0U,
      state.rear_roboclaw.healthy ? 1U : 0U,
      static_cast<unsigned long>(state.front_roboclaw.error_status),
      static_cast<unsigned long>(state.rear_roboclaw.error_status),
      state.fault == FaultCode::HOST_TIMEOUT ? 1U : 0U,
      mode_name(state.mode), fault_name(state.fault));
}

}  // namespace protocol
