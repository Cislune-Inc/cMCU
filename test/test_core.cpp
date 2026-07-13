#include <assert.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "control_logic.h"
#include "protocol.h"

static bool near(int32_t actual, int32_t expected, int32_t tolerance = 1) {
  return actual >= expected - tolerance && actual <= expected + tolerance;
}

static SystemState ready_state(uint32_t command_ms) {
  SystemState state = {};
  state.startup_ms = 0;
  state.front_roboclaw.healthy = true;
  state.rear_roboclaw.healthy = true;
  state.telemetry.battery_valid = true;
  state.telemetry.battery_voltage = 24.0f;
  state.host.handshake_complete = true;
  state.host.velocity_command.valid = true;
  state.host.velocity_command.received_ms = command_ms;
  return state;
}

static void test_protocol_handshake_and_velocity() {
  const protocol::ParseResult hello = protocol::parse_host_line("HELLO,1", 10);
  assert(hello.ok && hello.is_hello);
  assert(!protocol::parse_host_line("HELLO,2", 10).ok);

  const protocol::ParseResult velocity =
      protocol::parse_host_line("CMD_VEL,42,0.25,-0.5", 123);
  assert(velocity.ok && velocity.is_velocity);
  assert(velocity.velocity.sequence == 42);
  assert(velocity.velocity.linear_mps == 0.25f);
  assert(velocity.velocity.angular_radps == -0.5f);
  assert(velocity.velocity.received_ms == 123);
  assert(!protocol::parse_host_line("CMD_VEL,42,nan,0", 123).ok);
  assert(!protocol::parse_host_line("CMD_VEL,42,0,0,junk", 123).ok);
}

static void test_duty_is_disabled_by_default() {
  const protocol::ParseResult duty =
      protocol::parse_host_line("CMD_DUTY,1,0.1,0,0,0", 10);
  assert(!config::kDutyTestEnabled);
  assert(!duty.ok);
}

static void test_body_velocity_kinematics() {
  MotionCommand command = {};
  command.linear_mps = 0.2f;
  command.angular_radps = 0.4f;
  const WheelOutputs output = control_logic::body_velocity_targets(command);
  const float qpps_per_mps = config::kEncoderCountsPerWheelRevolution /
                             (2.0f * static_cast<float>(M_PI) *
                              config::kWheelRadiusMeters);
  const int32_t expected_left = static_cast<int32_t>(lroundf(0.1f * qpps_per_mps));
  const int32_t expected_right = static_cast<int32_t>(lroundf(0.3f * qpps_per_mps));
  assert(near(output.qpps[0], expected_left));
  assert(near(output.qpps[1], expected_right));
  assert(output.qpps[0] == output.qpps[2]);
  assert(output.qpps[1] == output.qpps[3]);
}

static void test_saturation_preserves_ratio() {
  MotionCommand command = {};
  command.linear_mps = 100.0f;
  command.angular_radps = 100.0f;
  const WheelOutputs output = control_logic::body_velocity_targets(command);
  for (size_t i = 0; i < kWheelCount; ++i) {
    assert(abs(output.qpps[i]) <= config::kMaxQpps);
  }
}

static void test_slew_and_reversal_cross_zero() {
  WheelOutputs current = {};
  WheelOutputs desired = {};
  current.qpps[0] = 1000;
  desired.qpps[0] = -1000;
  const WheelOutputs first =
      control_logic::slew_velocity_targets(current, desired, 20);
  assert(first.qpps[0] == 800);

  WheelOutputs value = current;
  for (int i = 0; i < 5; ++i) {
    value = control_logic::slew_velocity_targets(value, desired, 20);
  }
  assert(value.qpps[0] == 0);
}

static void test_state_requires_fresh_host_command() {
  SystemState state = ready_state(1900);
  state.host.velocity_command.linear_mps = 0.2f;
  control_logic::update_state(state, 2000);
  assert(state.mode == OperatingMode::BODY_VELOCITY);
  assert(state.fault == FaultCode::OK);

  control_logic::update_state(state, 2200);
  assert(state.mode == OperatingMode::FAULT);
  assert(state.fault == FaultCode::HOST_TIMEOUT);
  assert(state.wheel_outputs.qpps[0] == 0);
}

static void test_state_requires_both_controllers() {
  SystemState state = ready_state(1990);
  state.rear_roboclaw.healthy = false;
  control_logic::update_state(state, 2000);
  assert(state.mode == OperatingMode::FAULT);
  assert(state.fault == FaultCode::ROBOCLAW);
}

static void test_telemetry_fields_are_ordered() {
  SystemState state = ready_state(100);
  state.mode = OperatingMode::BODY_VELOCITY;
  state.fault = FaultCode::OK;
  state.telemetry_sequence = 7;
  state.host.last_accepted_sequence = 6;
  state.telemetry.encoder_counts[0] = 11;
  state.telemetry.encoder_counts[1] = 22;
  state.telemetry.encoder_counts[2] = 33;
  state.telemetry.encoder_counts[3] = 44;
  for (size_t i = 0; i < kWheelCount; ++i) {
    state.telemetry.encoder_valid[i] = true;
    state.telemetry.speed_valid[i] = true;
  }

  char buffer[320];
  protocol::format_telemetry(state, 1234, buffer, sizeof(buffer));
  const char* prefix = "TLM,1,7,1234,6,11,22,33,44,";
  assert(strncmp(buffer, prefix, strlen(prefix)) == 0);
  assert(strstr(buffer, ",24.0,1,15,15,1,1,0,0,0,VELOCITY,NONE") != nullptr);
}

int main() {
  test_protocol_handshake_and_velocity();
  test_duty_is_disabled_by_default();
  test_body_velocity_kinematics();
  test_saturation_preserves_ratio();
  test_slew_and_reversal_cross_zero();
  test_state_requires_fresh_host_command();
  test_state_requires_both_controllers();
  test_telemetry_fields_are_ordered();
  return 0;
}
