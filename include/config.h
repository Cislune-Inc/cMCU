#pragma once

#include <stdint.h>

#ifndef CMCU_ENABLE_DUTY_TEST
#define CMCU_ENABLE_DUTY_TEST 0
#endif

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

constexpr float kBatteryCriticalVolts = 18.8f;
constexpr int16_t kMaxDebugDuty = 4096;
constexpr int32_t kMaxQpps = 4000;
constexpr int32_t kMaxQppsChangePerSecond = 10000;
constexpr float kMaxLinearMps = 0.5f;
constexpr float kMaxAngularRadps = 1.0f;

// Provisional values. Measure the rover before a ground test.
constexpr float kWheelRadiusMeters = 0.10f;
constexpr float kEffectiveTrackWidthMeters = 0.50f;
constexpr float kEncoderCountsPerWheelRevolution = 1000.0f;

constexpr bool kDutyTestEnabled = CMCU_ENABLE_DUTY_TEST != 0;
constexpr uint32_t kRoboclawCriticalErrorMask = 0x000FFF;

constexpr uint8_t kFrontRoboclawAddress = 0x80;
constexpr uint8_t kRearRoboclawAddress = 0x81;

}  // namespace config
