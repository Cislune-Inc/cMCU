#pragma once

#include <stdint.h>

namespace config {

constexpr uint32_t kProtocolVersion = 1;
constexpr uint32_t kHostBaud = 115200;
constexpr uint32_t kRoboclawBaud = 38400;

constexpr uint32_t kLoopDelayMs = 5;
constexpr uint32_t kTelemetryPeriodMs = 50;
constexpr uint32_t kMotorUpdatePeriodMs = 20;
constexpr uint32_t kRoboclawTimeoutUs = 10000;
constexpr uint32_t kRoboclawSetupDelayMs = 500;

constexpr uint32_t kHostTimeoutMs = 250;
constexpr uint32_t kRoboclawHealthTimeoutMs = 250;
constexpr uint32_t kStartupLockoutMs = 1000;

constexpr float kBatteryCriticalVolts = 18.0f;
constexpr int32_t kMaxQpps = 4000;
constexpr int32_t kMaxQppsChangePerSecond = 10000;
constexpr float kMaxLinearMps = 0.15f;
constexpr float kMaxAngularRadps = 0.4f;

// Small-rover baselines recovered from carve_ws. Measure and update these,
// kMaxQpps, the slew limit, and kMotorMap before the first ground test.
constexpr float kWheelRadiusMeters = 0.09f;
constexpr float kEffectiveTrackWidthMeters = 0.108f;
constexpr float kEncoderCountsPerWheelRevolution = 36200.0f;

constexpr uint32_t kRoboclawCriticalErrorMask = 0x000FFF;

constexpr uint8_t kFrontRoboclawAddress = 0x80;
constexpr uint8_t kRearRoboclawAddress = 0x81;

}  // namespace config
