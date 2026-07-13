#include "radio_link.h"

#include <Arduino.h>
#include <RH_NRF24.h>
#include <SPI.h>

#include "config.h"
#include "control_logic.h"

namespace {

enum class ControllerMode : uint8_t {
  MANUAL = 1,
  AUTO = 2,
};

struct RcPacket {
  float joyy;
  float joyx;
  float fwd_drum;
  float aft_drum;
  float fwd_lift;
  float aft_lift;
  bool estop;
  ControllerMode mode;
} __attribute__((packed, aligned(1)));

RH_NRF24 g_radio(config::kRadioCePin, config::kRadioCsnPin);
bool g_radio_ready = false;

MotionCommand to_motion(const RcPacket& packet, uint32_t now_ms) {
  MotionCommand motion = {};
  motion.source_mode = OperatingMode::JOYSTICK;
  motion.throttle = control_logic::clamp_norm(packet.joyy);
  motion.turn = control_logic::clamp_norm(packet.joyx);
  motion.estop = packet.estop;
  motion.received_ms = now_ms;
  motion.valid = true;
  return motion;
}

}  // namespace

void setup_radio_link() {
  g_radio_ready = g_radio.init() &&
                  g_radio.setChannel(1) &&
                  g_radio.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPowerm18dBm);
}

void update_radio_link(SystemState& state, uint32_t now_ms) {
  if (!g_radio_ready || !g_radio.available()) {
    return;
  }

  uint8_t len = sizeof(RcPacket);
  RcPacket packet = {};
  if (!g_radio.recv(reinterpret_cast<uint8_t*>(&packet), &len) || len != sizeof(RcPacket)) {
    return;
  }

  state.radio.last_packet_ms = now_ms;
  state.radio.linked = true;
  state.radio.estop = packet.estop;
  state.radio.selected_mode = OperatingMode::JOYSTICK;
  state.radio.joystick_command = to_motion(packet, now_ms);
}
