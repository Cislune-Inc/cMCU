#include <Arduino.h>

#include "RoboClaw.h"
#include "board_pins.h"
#include "config.h"
#include "system_state.h"

namespace {

// Start with the rover's wheels off the ground. Increase only after confirming
// that the motor wiring and direction signs are correct.
constexpr int32_t kTestQpps = 500;

RoboClaw g_front_roboclaw(&board::front_roboclaw_uart,
                          config::kRoboclawTimeoutUs);
RoboClaw g_rear_roboclaw(&board::rear_roboclaw_uart,
                         config::kRoboclawTimeoutUs);

}  // namespace

void setup() {
  g_front_roboclaw.begin(config::kRoboclawBaud);
  g_rear_roboclaw.begin(config::kRoboclawBaud);
  delay(config::kRoboclawSetupDelayMs);
}

void loop() {
  const int32_t fl_qpps = kTestQpps * kMotorMap[0].command_direction;
  const int32_t fr_qpps = kTestQpps * kMotorMap[1].command_direction;
  const int32_t rl_qpps = kTestQpps * kMotorMap[2].command_direction;
  const int32_t rr_qpps = kTestQpps * kMotorMap[3].command_direction;

  g_front_roboclaw.SpeedM1M2(config::kFrontRoboclawAddress,
                             static_cast<uint32_t>(fl_qpps),
                             static_cast<uint32_t>(fr_qpps));
  g_rear_roboclaw.SpeedM1M2(config::kRearRoboclawAddress,
                            static_cast<uint32_t>(rl_qpps),
                            static_cast<uint32_t>(rr_qpps));
  delay(config::kMotorUpdatePeriodMs);
}
