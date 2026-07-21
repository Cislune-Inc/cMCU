#include <Arduino.h>

#include "RoboClaw.h"
#include "board_pins.h"
#include "config.h"

namespace {

constexpr uint8_t kFirstPacketAddress = 0x80;
constexpr uint8_t kLastPacketAddress = 0x87;
constexpr uint32_t kScanPeriodMs = 5000;

RoboClaw g_front(&board::front_roboclaw_uart, config::kRoboclawTimeoutUs);
RoboClaw g_rear(&board::rear_roboclaw_uart, config::kRoboclawTimeoutUs);
uint32_t g_last_scan_ms = 0;

size_t scan_port(const char* port_name, RoboClaw& roboclaw) {
  size_t matches = 0;
  for (uint8_t address = kFirstPacketAddress;
       address <= kLastPacketAddress; ++address) {
    char version[64] = {};
    if (!roboclaw.ReadVersion(address, version)) {
      continue;
    }

    ++matches;
    Serial.print("ROBOCLAW_FOUND,");
    Serial.print(port_name);
    Serial.print(",0x");
    Serial.print(address, HEX);
    Serial.print(",");
    Serial.println(version);
  }
  return matches;
}

void scan_addresses() {
  Serial.println("ROBOCLAW_SCAN_BEGIN,38400,0x80-0x87");
  const size_t front_matches = scan_port("front", g_front);
  const size_t rear_matches = scan_port("rear", g_rear);
  Serial.print("ROBOCLAW_SCAN_END,front=");
  Serial.print(front_matches);
  Serial.print(",rear=");
  Serial.println(rear_matches);
}

}  // namespace

void setup() {
  Serial.begin(config::kHostBaud);
  g_front.begin(config::kRoboclawBaud);
  g_rear.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);
  scan_addresses();
  g_last_scan_ms = millis();
}

void loop() {
  const uint32_t now_ms = millis();
  if ((now_ms - g_last_scan_ms) >= kScanPeriodMs) {
    scan_addresses();
    g_last_scan_ms = now_ms;
  }
  delay(config::kLoopDelayMs);
}
