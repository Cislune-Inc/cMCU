# cMCU

## `test` branch

This branch replaces the normal `main.cpp` with a direct RoboClaw bring-up
program. It continuously commands all four mapped motors at `kTestQpps = 500`
and does not wait for Jetson, ROS, or host-link commands. Keep the wheels off
the ground and remove motor power to stop it.

Teensy 4.0 firmware for a four-wheel skid-steer rover. The Jetson sends one
authoritative body-velocity stream; the Teensy converts it to wheel QPPS and
uses two RoboClaws for closed-loop wheel velocity control.

## Wiring

All UART signals are 3.3 V logic. Teensy 4.0 pins are not 5 V tolerant. Connect
the Jetson, Teensy, and both RoboClaw logic grounds together.

| Link | Source | Destination |
| --- | --- | --- |
| Jetson to Teensy | Jetson UART TX | Teensy pin 0 (RX1) |
| Teensy to Jetson | Teensy pin 1 (TX1) | Jetson UART RX |
| Front RoboClaw command | Teensy pin 8 (TX2) | Front RoboClaw S1 |
| Front RoboClaw response | Front RoboClaw S2 | Teensy pin 7 (RX2) |
| Rear RoboClaw command | Teensy pin 14 (TX3) | Rear RoboClaw S1 |
| Rear RoboClaw response | Rear RoboClaw S2 | Teensy pin 15 (RX3) |

Configure both RoboClaws for packet serial at 38400 baud. The firmware expects
front address `0x80` and rear address `0x81`. Motor mapping is:

- Front RoboClaw M1: front-left
- Front RoboClaw M2: front-right
- Rear RoboClaw M1: rear-left
- Rear RoboClaw M2: rear-right

Use the signal contact on each RoboClaw S1/S2 header. If the Teensy has its own
power supply, do not connect either RoboClaw's 5 V BEC contact to the Teensy.

There is no E-stop input/output or radio connection in this firmware. Use an
independent hardware power disconnect during commissioning.

## Host protocol version 1

The link is line-oriented ASCII at 115200 baud. End every packet with `\n`.

1. Jetson sends `HELLO,1` and waits for `ACK,HELLO,1`.
2. Jetson continuously sends
   `CMD_VEL,<sequence>,<linear_mps>,<angular_radps>`.
3. The Teensy replies `ACK,CMD,<sequence>` for accepted commands and stops if a
   fresh command is not received within 250 ms.

Sequence numbers must increase; a new `HELLO,1` resets sequence tracking.
Malformed, stale, non-finite, or wrong-version commands are rejected.

Telemetry is:

```text
TLM,<version>,<tlm_seq>,<mcu_ms>,<last_cmd_seq>,
    <enc_fl>,<enc_fr>,<enc_rl>,<enc_rr>,
    <measured_fl>,<measured_fr>,<measured_rl>,<measured_rr>,
    <target_fl>,<target_fr>,<target_rl>,<target_rr>,
    <battery_v>,<battery_valid>,<encoder_valid_bits>,<speed_valid_bits>,
    <front_ok>,<rear_ok>,<front_error>,<rear_error>,<watchdog_expired>,<mode>,<fault>
```

The actual packet is one line without spaces. Encoder and speed validity use
bits 0..3 for FL, FR, RL, and RR.

## Required calibration

Before a ground test, replace the provisional values in `include/config.h`:

- `kWheelRadiusMeters`
- `kEffectiveTrackWidthMeters`
- `kEncoderCountsPerWheelRevolution`
- velocity, QPPS, battery, and slew limits as appropriate for the rover

Independent command and encoder direction corrections are in `kMotorMap` in
`include/system_state.h`.

## Duty-cycle debugging

Normal builds do not compile in a duty command. For a wheels-off-ground
commissioning build only, add `-DCMCU_ENABLE_DUTY_TEST=1` to `build_flags`.
That enables:

```text
CMD_DUTY,<sequence>,<fl>,<fr>,<rl>,<rr>
```

Each value is clamped to `[-1, 1]` and then limited by `kMaxDebugDuty`. Remove
the build flag before normal operation.

## Build and test

```bash
pio run
g++ -std=gnu++17 -Iinclude test/test_core.cpp src/control_logic.cpp src/protocol.cpp -o /tmp/cMCU-tests
/tmp/cMCU-tests
```
