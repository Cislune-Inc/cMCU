#include <assert.h>
#include <string.h>

#include "control_logic.h"
#include "protocol.h"

static void test_mix_drive_scales() {
  const MixedDrive drive = control_logic::mix_drive(1.0f, 1.0f);
  assert(drive.left == 0.0f);
  assert(drive.right == 1.0f);
}

static void test_host_protocol_parses_motion() {
  const protocol::ParseResult result = protocol::parse_host_line("C,0.5,-0.25,1", 123U);
  assert(result.ok);
  assert(result.is_motion);
  assert(result.motion.estop);
  assert(result.motion.received_ms == 123U);
}

static void test_host_protocol_requires_literal_pong() {
  const protocol::ParseResult pong = protocol::parse_host_line("PONG", 10U);
  assert(pong.ok);
  assert(pong.is_ping);

  const protocol::ParseResult ping = protocol::parse_host_line("PING", 10U);
  assert(!ping.ok);
}

static void test_radio_protocol_updates_selected_mode() {
  RadioState radio = {};
  const bool ok = protocol::parse_radio_line("R,A,0.0,0.0,0", radio, 99U);
  assert(ok);
  assert(radio.selected_mode == OperatingMode::AUTONOMOUS);
  assert(radio.linked);
}

static void test_fault_for_missing_host_command() {
  SystemState state = {};
  state.startup_ms = 0U;
  state.roboclaw_ok = true;
  state.radio.selected_mode = OperatingMode::AUTONOMOUS;
  state.radio.linked = true;
  state.radio.last_packet_ms = 1900U;
  state.host.handshake_complete = true;
  state.telemetry.battery_valid = true;
  state.telemetry.battery_voltage = 24.0f;

  control_logic::update_state(state, 2000U);
  assert(state.mode == OperatingMode::SAFE);
  assert(state.fault == FaultCode::HOST_TIMEOUT);
}

static void test_fault_for_missing_radio_command() {
  SystemState state = {};
  state.startup_ms = 0U;
  state.roboclaw_ok = true;
  state.radio.selected_mode = OperatingMode::AUTONOMOUS;
  state.telemetry.battery_valid = true;
  state.telemetry.battery_voltage = 24.0f;

  control_logic::update_state(state, 2000U);
  assert(state.mode == OperatingMode::SAFE);
  assert(state.fault == FaultCode::RADIO_TIMEOUT);
}

static void test_host_protocol_selects_mode() {
  const protocol::ParseResult result = protocol::parse_host_line("M,A", 55U);
  assert(result.ok);
  assert(result.is_mode_select);
  assert(result.selected_mode == OperatingMode::AUTONOMOUS);
}

static void test_auto_radio_uses_selected_host_mode() {
  SystemState state = {};
  state.startup_ms = 0U;
  state.roboclaw_ok = true;
  state.radio.selected_mode = OperatingMode::AUTONOMOUS;
  state.radio.linked = true;
  state.radio.last_packet_ms = 1990U;
  state.host.handshake_complete = true;
  state.host.selected_mode = OperatingMode::KEYBOARD;
  state.host.keyboard_command.source_mode = OperatingMode::KEYBOARD;
  state.host.keyboard_command.throttle = 0.5f;
  state.host.keyboard_command.valid = true;
  state.host.keyboard_command.received_ms = 1990U;
  state.telemetry.battery_valid = true;
  state.telemetry.battery_voltage = 24.0f;

  control_logic::update_state(state, 2000U);
  assert(state.mode == OperatingMode::KEYBOARD);
  assert(state.fault == FaultCode::OK);
  assert(state.active_command.source_mode == OperatingMode::KEYBOARD);
}

static void test_host_requires_handshake_before_motion() {
  SystemState state = {};
  state.startup_ms = 0U;
  state.roboclaw_ok = true;
  state.radio.selected_mode = OperatingMode::AUTONOMOUS;
  state.radio.linked = true;
  state.radio.last_packet_ms = 1990U;
  state.host.selected_mode = OperatingMode::KEYBOARD;
  state.host.keyboard_command.source_mode = OperatingMode::KEYBOARD;
  state.host.keyboard_command.throttle = 0.5f;
  state.host.keyboard_command.valid = true;
  state.host.keyboard_command.received_ms = 1990U;
  state.telemetry.battery_valid = true;
  state.telemetry.battery_voltage = 24.0f;

  control_logic::update_state(state, 2000U);
  assert(state.mode == OperatingMode::SAFE);
  assert(state.fault == FaultCode::HOST_TIMEOUT);
}

static void test_telemetry_is_readable_and_reordered() {
  SystemState state = {};
  state.mode = OperatingMode::KEYBOARD;
  state.fault = FaultCode::HOST_TIMEOUT;
  state.drive_mode = DriveMode::DUTY;
  state.host.selected_mode = OperatingMode::AUTONOMOUS;
  state.active_command.throttle = 0.4f;
  state.active_command.turn = 0.1f;
  state.telemetry.battery_voltage = 24.0f;
  state.telemetry.encoder_counts[0] = 11;  // FL
  state.telemetry.encoder_counts[1] = 22;  // FR
  state.telemetry.encoder_counts[2] = 33;  // RL
  state.telemetry.encoder_counts[3] = 44;  // RR

  char buffer[256];
  protocol::format_telemetry(state, buffer, sizeof(buffer));

  assert(strcmp(buffer,
                "mode=KEY drive=DUTY host=AUTO thr=0.400 trn=0.100 enc=22|11|44|33 batt=24.0 fault=Host") == 0);
}

int main() {
  test_mix_drive_scales();
  test_host_protocol_parses_motion();
  test_host_protocol_requires_literal_pong();
  test_radio_protocol_updates_selected_mode();
  test_fault_for_missing_host_command();
  test_fault_for_missing_radio_command();
  test_host_protocol_selects_mode();
  test_auto_radio_uses_selected_host_mode();
  test_host_requires_handshake_before_motion();
  test_telemetry_is_readable_and_reordered();
  return 0;
}
