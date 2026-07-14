#include <Arduino.h>

#include "RoboClaw.h"
#include "board_pins.h"
#include "config.h"
#include "system_state.h"

namespace {

// Raw duty is intentional here: this test must not depend on a tuned velocity
// PID or valid encoder feedback. Keep the rover's wheels off the ground.
constexpr int16_t kTestDuty = 2048;  // 6.25% of RoboClaw's full duty range.
constexpr uint32_t kFeedbackPeriodMs = 250;

RoboClaw g_front_roboclaw(&board::front_roboclaw_uart,
                          config::kRoboclawTimeoutUs);
RoboClaw g_rear_roboclaw(&board::rear_roboclaw_uart,
                         config::kRoboclawTimeoutUs);
uint32_t g_last_feedback_ms = 0;

void print_speed(const char* wheel, RoboClaw& roboclaw, uint8_t address,
                 bool channel_m2) {
  uint8_t status = 0;
  bool valid = false;
  const uint32_t qpps = channel_m2
                            ? roboclaw.ReadISpeedM2(address, &status, &valid)
                            : roboclaw.ReadISpeedM1(address, &status, &valid);

  Serial.print(wheel);
  Serial.print("=");
  if (valid) {
    Serial.print(qpps);
    Serial.print("(status=0x");
    Serial.print(status, HEX);
    Serial.print(")");
  } else {
    Serial.print("INVALID");
  }
}

void print_feedback(uint32_t now_ms) {
  if ((now_ms - g_last_feedback_ms) < kFeedbackPeriodMs) {
    return;
  }
  g_last_feedback_ms = now_ms;

  Serial.print("RAW_QPPS,");
  print_speed("FL", g_front_roboclaw, config::kFrontRoboclawAddress, false);
  Serial.print(",");
  print_speed("FR", g_front_roboclaw, config::kFrontRoboclawAddress, true);
  Serial.print(",");
  print_speed("RL", g_rear_roboclaw, config::kRearRoboclawAddress, false);
  Serial.print(",");
  print_speed("RR", g_rear_roboclaw, config::kRearRoboclawAddress, true);
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  g_front_roboclaw.begin(config::kRoboclawBaud);
  g_rear_roboclaw.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);
}

void loop() {
  const int16_t fl_duty = kTestDuty * kMotorMap[0].command_direction;
  const int16_t fr_duty = kTestDuty * kMotorMap[1].command_direction;
  const int16_t rl_duty = kTestDuty * kMotorMap[2].command_direction;
  const int16_t rr_duty = kTestDuty * kMotorMap[3].command_direction;

  g_front_roboclaw.DutyM1M2(config::kFrontRoboclawAddress,
                            static_cast<uint16_t>(fl_duty),
                            static_cast<uint16_t>(fr_duty));
  g_rear_roboclaw.DutyM1M2(config::kRearRoboclawAddress,
                           static_cast<uint16_t>(rl_duty),
                           static_cast<uint16_t>(rr_duty));

  print_feedback(millis());
  delay(config::kMotorUpdatePeriodMs);
}
