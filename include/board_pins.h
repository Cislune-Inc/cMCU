#pragma once

#include <Arduino.h>

namespace board {

inline auto& debug_uart = Serial;
inline HardwareSerial& host_uart = Serial1;
inline HardwareSerial& front_roboclaw_uart = Serial2;
inline HardwareSerial& rear_roboclaw_uart = Serial3;

constexpr uint8_t kHostRxPin = 0;
constexpr uint8_t kHostTxPin = 1;
constexpr uint8_t kFrontRoboclawRxPin = 7;
constexpr uint8_t kFrontRoboclawTxPin = 8;
constexpr uint8_t kRearRoboclawRxPin = 15;
constexpr uint8_t kRearRoboclawTxPin = 14;

}  // namespace board
