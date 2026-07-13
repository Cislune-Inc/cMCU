#pragma once

#include <stdint.h>

namespace config {

constexpr uint8_t kEstopOutPin = 6;
constexpr uint8_t kRadioCePin = 2;
constexpr uint8_t kRadioCsnPin = 10;

constexpr uint32_t kHostBaud = 115200;
constexpr uint32_t kRadioBaud = 115200;
constexpr uint32_t kRoboclawBaud = 38400;

constexpr uint32_t kLoopDelayMs = 5;
constexpr uint32_t kTelemetryPeriodMs = 50;
constexpr uint32_t kMotorUpdatePeriodMs = 20;
constexpr uint32_t kRoboclawTimeoutUs = 100000;
constexpr uint32_t kRoboclawSetupDelayMs = 500;

constexpr uint32_t kRadioTimeoutMs = 250;
constexpr uint32_t kHostTimeoutMs = 250;
constexpr uint32_t kStartupLockoutMs = 1000;

constexpr float kBatteryCriticalVolts = 18.8f;
constexpr int16_t kMaxDuty = 16384;
constexpr int32_t kMaxQpps = 4000;
constexpr float kInputDeadband = 0.05f;

constexpr uint8_t kFrontRoboclawAddress = 0x80;
constexpr uint8_t kRearRoboclawAddress = 0x81;
constexpr uint8_t kBatteryRoboclawAddress = 0x81;

}  // namespace config
