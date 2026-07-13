#include "control_logic.h"

#include <math.h>

#include "config.h"

namespace control_logic {

float clamp_norm(float value) {
  if (value < -1.0f) {
    return -1.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

float apply_deadband(float value) {
  value = clamp_norm(value);
  return fabsf(value) < config::kInputDeadband ? 0.0f : value;
}

MixedDrive mix_drive(float throttle, float turn) {
  throttle = apply_deadband(throttle);
  turn = apply_deadband(turn);

  MixedDrive drive = {};
  drive.left = throttle - turn;
  drive.right = throttle + turn;

  const float max_mag = fmaxf(fabsf(drive.left), fabsf(drive.right));
  if (max_mag > 1.0f) {
    drive.left /= max_mag;
    drive.right /= max_mag;
  }

  return drive;
}

WheelOutputs build_wheel_outputs(const MixedDrive& drive, DriveMode mode) {
  WheelOutputs outputs = {};

  const int16_t left_duty =
      static_cast<int16_t>(lroundf(drive.left * config::kMaxDuty));
  const int16_t right_duty =
      static_cast<int16_t>(lroundf(drive.right * config::kMaxDuty));
  const int32_t left_qpps =
      static_cast<int32_t>(lroundf(drive.left * config::kMaxQpps));
  const int32_t right_qpps =
      static_cast<int32_t>(lroundf(drive.right * config::kMaxQpps));

  outputs.duty[static_cast<size_t>(WheelId::FL)] = left_duty;
  outputs.duty[static_cast<size_t>(WheelId::FR)] = right_duty;
  outputs.duty[static_cast<size_t>(WheelId::RL)] = left_duty;
  outputs.duty[static_cast<size_t>(WheelId::RR)] = right_duty;

  if (mode == DriveMode::VELOCITY) {
    outputs.qpps[static_cast<size_t>(WheelId::FL)] = left_qpps;
    outputs.qpps[static_cast<size_t>(WheelId::FR)] = right_qpps;
    outputs.qpps[static_cast<size_t>(WheelId::RL)] = left_qpps;
    outputs.qpps[static_cast<size_t>(WheelId::RR)] = right_qpps;
  }

  return outputs;
}

static MotionCommand select_active_command(const SystemState& state) {
  switch (state.mode) {
    case OperatingMode::JOYSTICK: {
      MotionCommand motion = state.radio.joystick_command;
      motion.throttle = -motion.throttle;
      return motion;
    }
    case OperatingMode::KEYBOARD:
      return state.host.keyboard_command;
    case OperatingMode::AUTONOMOUS:
      return state.host.autonomous_command;
    case OperatingMode::SAFE:
    default:
      return {};
  }
}

static OperatingMode select_operating_mode(const SystemState& state) {
  return state.host.selected_mode;
}

static FaultCode compute_fault(const SystemState& state, uint32_t now_ms) {
  if ((now_ms - state.startup_ms) < config::kStartupLockoutMs) {
    return FaultCode::STARTUP;
  }
  if (!state.roboclaw_ok) {
    return FaultCode::ROBOCLAW;
  }
  if (state.telemetry.battery_valid &&
      state.telemetry.battery_voltage < config::kBatteryCriticalVolts) {
    return FaultCode::BATTERY;
  }
  if (state.mode == OperatingMode::JOYSTICK) {
    if (!state.radio.linked) {
      return FaultCode::RADIO_TIMEOUT;
    }
    if (state.radio.estop) {
      return FaultCode::ESTOP;
    }
    if (!state.radio.joystick_command.valid ||
        state.radio.joystick_command.received_ms == 0U) {
      return FaultCode::RADIO_TIMEOUT;
    }
    return FaultCode::OK;
  }
  if (state.mode == OperatingMode::KEYBOARD || state.mode == OperatingMode::AUTONOMOUS) {
    if (!state.host.handshake_complete) {
      return FaultCode::HOST_TIMEOUT;
    }
    if (state.active_command.estop) {
      return FaultCode::ESTOP;
    }
    if (!state.active_command.valid ||
        (now_ms - state.active_command.received_ms) > config::kHostTimeoutMs) {
      return FaultCode::HOST_TIMEOUT;
    }
    return FaultCode::OK;
  }
  return FaultCode::OK;
}

void update_state(SystemState& state, uint32_t now_ms) {
  state.mode = select_operating_mode(state);
  state.drive_mode = state.host.requested_drive_mode;
  state.active_command = select_active_command(state);
  state.fault = compute_fault(state, now_ms);

  if (state.fault != FaultCode::OK) {
    state.mode = OperatingMode::SAFE;
    state.active_command = {};
    state.mixed_drive = {};
    state.wheel_outputs = {};
    return;
  }

  state.mixed_drive = mix_drive(state.active_command.throttle, state.active_command.turn);
  state.wheel_outputs = build_wheel_outputs(state.mixed_drive, state.drive_mode);
}

}  // namespace control_logic
