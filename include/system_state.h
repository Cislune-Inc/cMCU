#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum class OperatingMode : uint8_t {
  SAFE = 'S',
  JOYSTICK = 'J',
  KEYBOARD = 'K',
  AUTONOMOUS = 'A',
};

enum class DriveMode : uint8_t {
  DUTY = 'U',
  VELOCITY = 'V',
};

enum class FaultCode : uint8_t {
  OK = '0',
  STARTUP = 'B',
  ESTOP = 'E',
  RADIO_TIMEOUT = 'R',
  HOST_TIMEOUT = 'H',
  BATTERY = 'T',
  ROBOCLAW = 'C',
};

enum class WheelId : uint8_t {
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};

struct MotionCommand {
  OperatingMode source_mode = OperatingMode::SAFE;
  float throttle = 0.0f;
  float turn = 0.0f;
  bool estop = false;
  uint32_t sequence = 0;
  uint32_t received_ms = 0;
  bool valid = false;
};

struct RadioState {
  OperatingMode selected_mode = OperatingMode::JOYSTICK;
  MotionCommand joystick_command = {};
  bool estop = false;
  uint32_t last_packet_ms = 0;
  bool linked = false;
};

struct HostState {
  OperatingMode selected_mode = OperatingMode::KEYBOARD;
  MotionCommand keyboard_command = {};
  MotionCommand autonomous_command = {};
  DriveMode requested_drive_mode = DriveMode::DUTY;
  uint32_t last_packet_ms = 0;
  uint32_t last_pong_ms = 0;
  bool linked = false;
  bool handshake_complete = false;
};

struct MixedDrive {
  float left = 0.0f;
  float right = 0.0f;
};

struct WheelOutputs {
  int16_t duty[4] = {0, 0, 0, 0};
  int32_t qpps[4] = {0, 0, 0, 0};
};

struct TelemetryState {
  float battery_voltage = 0.0f;
  bool battery_valid = false;
  int32_t encoder_counts[4] = {0, 0, 0, 0};
  bool encoder_valid[4] = {false, false, false, false};
};

struct SystemState {
  OperatingMode mode = OperatingMode::SAFE;
  DriveMode drive_mode = DriveMode::DUTY;
  FaultCode fault = FaultCode::STARTUP;
  bool roboclaw_ok = false;

  RadioState radio = {};
  HostState host = {};
  MotionCommand active_command = {};
  MixedDrive mixed_drive = {};
  WheelOutputs wheel_outputs = {};
  TelemetryState telemetry = {};

  uint32_t startup_ms = 0;
  uint32_t last_telemetry_ms = 0;
  uint32_t last_motor_update_ms = 0;
  uint32_t telemetry_sequence = 0;
};

struct MotorChannel {
  uint8_t address;
  bool channel_m2;
  WheelId wheel;
  int8_t direction;
};

constexpr size_t kWheelCount = 4;

constexpr MotorChannel kMotorMap[kWheelCount] = {
    {0x80, false, WheelId::FL, 1},
    {0x80, true, WheelId::FR, 1},
    {0x81, false, WheelId::RL, 1},
    {0x81, true, WheelId::RR, 1},
};
