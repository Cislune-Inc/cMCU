#include "protocol.h"

#include <stdio.h>
#include <string.h>

#include "control_logic.h"

namespace protocol {

static const char* mode_name(OperatingMode mode) {
  switch (mode) {
    case OperatingMode::SAFE:
      return "SAFE";
    case OperatingMode::JOYSTICK:
      return "JOY";
    case OperatingMode::KEYBOARD:
      return "KEY";
    case OperatingMode::AUTONOMOUS:
      return "AUTO";
    default:
      return "UNKNOWN";
  }
}

static const char* drive_mode_name(DriveMode drive_mode) {
  switch (drive_mode) {
    case DriveMode::DUTY:
      return "DUTY";
    case DriveMode::VELOCITY:
      return "VELOCITY";
    default:
      return "UNKNOWN";
  }
}

static const char* fault_name(FaultCode fault) {
  switch (fault) {
    case FaultCode::OK:
      return "None";
    case FaultCode::STARTUP:
      return "Startup";
    case FaultCode::ESTOP:
      return "Estop";
    case FaultCode::RADIO_TIMEOUT:
      return "Radio";
    case FaultCode::HOST_TIMEOUT:
      return "Host";
    case FaultCode::BATTERY:
      return "Battery";
    case FaultCode::ROBOCLAW:
      return "RoboClaw";
    default:
      return "Unknown";
  }
}

static bool parse_mode_char(char mode_char, OperatingMode& mode) {
  switch (mode_char) {
    case 'J':
      mode = OperatingMode::JOYSTICK;
      return true;
    case 'K':
      mode = OperatingMode::KEYBOARD;
      return true;
    case 'A':
      mode = OperatingMode::AUTONOMOUS;
      return true;
    case 'S':
      mode = OperatingMode::SAFE;
      return true;
    default:
      return false;
  }
}

static bool parse_drive_mode_char(char drive_char, DriveMode& drive_mode) {
  switch (drive_char) {
    case 'U':
      drive_mode = DriveMode::DUTY;
      return true;
    case 'V':
      drive_mode = DriveMode::VELOCITY;
      return true;
    default:
      return false;
  }
}

static MotionCommand make_motion(OperatingMode mode, float throttle, float turn, int flags,
                                 uint32_t now_ms) {
  MotionCommand motion = {};
  motion.source_mode = mode;
  motion.throttle = control_logic::clamp_norm(throttle);
  motion.turn = control_logic::clamp_norm(turn);
  motion.estop = (flags & 0x1) != 0;
  motion.sequence = 0;
  motion.received_ms = now_ms;
  motion.valid = true;
  return motion;
}

ParseResult parse_host_line(const char* line, uint32_t now_ms) {
  ParseResult result = {};
  if (strcmp(line, "PONG") == 0) {
    result.ok = true;
    result.is_ping = true;
    return result;
  }

  char mode_char = '\0';
  if (sscanf(line, "M,%c", &mode_char) == 1) {
    OperatingMode mode;
    if (parse_mode_char(mode_char, mode) &&
        (mode == OperatingMode::KEYBOARD || mode == OperatingMode::AUTONOMOUS)) {
      result.ok = true;
      result.is_mode_select = true;
      result.selected_mode = mode;
    }
    return result;
  }

  char drive_char = '\0';
  if (sscanf(line, "D,%c", &drive_char) == 1) {
    DriveMode drive_mode;
    if (parse_drive_mode_char(drive_char, drive_mode)) {
      result.ok = true;
      result.is_drive_mode = true;
      result.drive_mode = drive_mode;
    }
    return result;
  }

  float throttle = 0.0f;
  float turn = 0.0f;
  int flags = 0;
  if (sscanf(line, "C,%f,%f,%d", &throttle, &turn, &flags) == 3) {
    result.ok = true;
    result.is_motion = true;
    result.motion = make_motion(OperatingMode::SAFE, throttle, turn, flags, now_ms);
  }
  return result;
}

bool parse_radio_line(const char* line, RadioState& radio, uint32_t now_ms) {
  char mode_char = '\0';
  float throttle = 0.0f;
  float turn = 0.0f;
  int flags = 0;
  if (sscanf(line, "R,%c,%f,%f,%d", &mode_char, &throttle, &turn, &flags) != 4) {
    return false;
  }

  OperatingMode mode;
  if (!parse_mode_char(mode_char, mode) ||
      (mode != OperatingMode::JOYSTICK && mode != OperatingMode::AUTONOMOUS)) {
    return false;
  }

  radio.selected_mode = mode;
  radio.last_packet_ms = now_ms;
  radio.linked = true;
  radio.estop = (flags & 0x1) != 0;

  if (mode == OperatingMode::JOYSTICK) {
    radio.joystick_command = make_motion(mode, throttle, turn, flags, now_ms);
  } else {
    radio.joystick_command.valid = false;
    radio.joystick_command.estop = radio.estop;
    radio.joystick_command.received_ms = now_ms;
  }
  return true;
}

void format_telemetry(const SystemState& state, char* buffer, size_t buffer_len) {
  snprintf(buffer, buffer_len,
           "mode=%s drive=%s host=%s thr=%.3f trn=%.3f enc=%ld|%ld|%ld|%ld batt=%.1f fault=%s",
           mode_name(state.mode),
           drive_mode_name(state.drive_mode),
           mode_name(state.host.selected_mode),
           static_cast<double>(state.active_command.throttle),
           static_cast<double>(state.active_command.turn),
           static_cast<long>(state.telemetry.encoder_counts[1]),
           static_cast<long>(state.telemetry.encoder_counts[0]),
           static_cast<long>(state.telemetry.encoder_counts[3]),
           static_cast<long>(state.telemetry.encoder_counts[2]),
           static_cast<double>(state.telemetry.battery_voltage),
           fault_name(state.fault));
}

}  // namespace protocol
