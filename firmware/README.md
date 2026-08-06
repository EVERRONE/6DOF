# 6DOF Robot Arm - Teensy Firmware

## Overview
This firmware controls a 6-axis robot arm using a Teensy 4.1 microcontroller with TMC2209 stepper drivers.

## Hardware Requirements
- Teensy 4.1
- 6Ã— TMC2209 stepper drivers (1/16 microstepping)
- 6Ã— NEMA stepper motors
- 4Ã— Omron D2F-L endstops (J2-J5)
- 24V power supply

## Pin Configuration

### Motor Connections
| Joint | STEP Pin | DIR Pin |
|-------|----------|---------|
| J1    | 2        | 5       |
| J2    | 3        | 6       |
| J3    | 4        | 7       |
| J4    | 22       | 23      |
| J5    | 24       | 25      |
| J6    | 26       | 27      |

### Other Connections
- Enable Pin 1: 8 (Active LOW - EN net of driver board 1, drives J1/J2/J3)
- Enable Pin 2: 9 (Active LOW - EN net of driver board 2, drives J4/J5/J6)
- Endstops: J2(30), J3(31), J4(32), J5(33)

The drivers are split across two boards, and each board has its own shared EN
net. This grouping matters when diagnosing faults: J1/J2/J3 share every supply
and logic net on board 1, and J4/J5/J6 share them on board 2. A fault that
takes out exactly one of those groups is a board-level fault, not a per-motor
fault. See "All joints on one board lose torque together" under Troubleshooting.

## Uploading Firmware

### Prerequisites
1. Install Arduino IDE 2.x or PlatformIO
2. Install Teensyduino from https://www.pjrc.com/teensy/td_download.html

### Steps
1. Open `firmware.ino` in Arduino IDE
2. Select **Tools â†’ Board â†’ Teensy 4.1**
3. Select **Tools â†’ USB Type â†’ Serial**
4. Select **Tools â†’ CPU Speed â†’ 600 MHz**
5. Click **Upload**
6. Wait for Teensy Loader to complete

## Serial Communication

### Baud Rate
115200

### Command Protocol
Commands are ASCII strings terminated with `\n` (newline).

**Move to Joint Angles:**
```
J <j1> <j2> <j3> <j4> <j5> <j6> <speed>
```
Example: `J 10 20 30 40 50 60 30\n`

**Home Joints:**
```
H <joints>
```
Examples:
- `H 2\n` - Home joint 2 only
- `H 2345\n` - Home joints 2, 3, 4, 5 in sequence
- `H ALL\n` - Home all joints with endstops

**Enable/Disable Motors:**
```
E <1|0>
```
- `E 1\n` - Enable motors
- `E 0\n` - Disable motors

**Emergency Stop:**
```
S
```
- `S\n` - Immediately stop all motion

**Query Position:**
```
Q
```
- `Q\n` - Request current position (triggers immediate POS response)

**Trajectory Queue (Cartesian execution):**
```
TQ CLEAR
TQ PT <t_ms> <q1> <q2> <q3> <q4> <q5> <q6> <qd1> <qd2> <qd3> <qd4> <qd5> <qd6>
TQ RUN
TQ STOP
TQ?
```
- `TQ` is intended for high-quality Cartesian trajectory playback.
- Queue points use logical degrees (`q*`) and logical deg/s (`qd*`).
- Queue playback is executed through timer-driven stepping (motion kernel v2).

**Motion Kernel Diagnostics:**
```
MQ?
MQ RESET
```
- `MQ?` returns `MQ STAT <tick_jitter_us> <queue_underrun> <step_overrun>`.
- `MQ RESET` clears diagnostic counters.

### Response Format

**Position Updates (sent every 100ms):**
```
POS <j1> <j2> <j3> <j4> <j5> <j6>
```

**Endstop State (sent every 100ms):**
```
ENDSTOP <j1> <j2> <j3> <j4> <j5> <j6>
```
Each value is `0` (not triggered) or `1` (triggered).

**Homing Complete:**
```
HOMED <joint_number>
```

**Acknowledgments:**
```
OK <message>
```

**Errors:**
```
ERROR <message>
```

**Trajectory Queue Events:**
```
TQ READY <count>
TQ PROG <point_index> <elapsed_ms>
TQ DONE
```

**Motion Kernel Event:**
```
MQ STAT <tick_jitter_us> <queue_underrun> <step_overrun>
```

## Testing

### Test 1: Connection
1. Upload firmware
2. Open Serial Monitor (115200 baud)
3. You should see: `OK Robot arm ready`
4. Position and endstop updates should appear every 100ms

### Test 2: Enable Motors
Send: `E 1`
Expected: `OK Motors enabled`

### Test 3: Move Single Joint
Send: `J 10 0 0 0 0 0 20`
Expected: J1 moves to 10 degrees at 20Â°/s

### Test 4: Home a Joint
Send: `H 2`
Expected:
- J2 moves toward endstop
- Stops on the endstop
- Sets logical angle to home zero (0 deg by default)
- Response: `HOMED 2`

### Test 5: Emergency Stop
1. Send: `J 30 0 0 0 0 0 10` (slow move)
2. While moving, send: `S`
3. Motor should stop immediately
4. Expected: `OK Emergency stop`

## Calibration

### Microsteps per Degree
Default values are in `config.h`:
- J1: 55.556
- J2: 222.222
- J3: 55.556
- J4: 33.333
- J5: 17.778
- J6: 8.889

**To calibrate:**
1. Home the joint
2. Command a 360Â° rotation: `J 360 0 0 0 0 0 20`
3. Measure actual rotation with a protractor
4. Adjust `USTEPS_PER_DEG` in `config.h` if error > 5Â°
5. Re-upload firmware

### Calibration for Manual Joint Control (Angle Accuracy)
If slider moves do not match actual joint motion, calibrate `USTEPS_PER_DEG` per joint:
1. Home the joint.
2. Command a known angle (e.g. 90Ã‚Â°): `J 0 0 0 90 0 0 20` (adjust for the joint you are testing).
3. Measure the actual rotation.
4. Compute: `newUSTEPS = oldUSTEPS * (commanded / actual)`.
5. Update `USTEPS_PER_DEG` in `config.h` and re-upload firmware.

### Homing Sequence
The homing sequence is:
1. **Fast approach** - Move toward endstop at HOMING_SPEED (10Â°/s)
2. **Set home angle** - Define current position as `HOME_LOGICAL_DEG` in `config.h`
3. **Stay on endstop** - No backoff or post-home move

## Troubleshooting

**Problem:** Motors don't move
- Check enable pin is LOW (8 â†’ GND through Teensy)
- Verify driver Vref is set correctly (~0.6-1.0V depending on motor)
- Check 24V power supply

**Problem:** All joints on one board lose torque together (J1/J2/J3, or J4/J5/J6)

Three motors do not fail at the same instant by coincidence. That group is
exactly one driver board, so the fault is in something all three share on that
board, not in a motor, a coil wire, or a single driver. If flexing or moving
the board provokes it, it is an intermittent mechanical connection. Candidates,
all shared per board:

- EN net (pin 8 for board 1, pin 9 for board 2). On TMC2209 the ENN input is
  pulled high internally, so an open EN wire floats to "disabled" and kills all
  drivers on that board instantly. This is the opposite of A4988/DRV8825, which
  default to enabled when EN floats. Prime suspect on this hardware.
- 24V motor supply (VMOT) at that board's screw terminal or its trace.
- Common GND between the Teensy and that board.
- Logic supply (VCC_IO) to that board's drivers.

To discriminate, measure against that board's GND while flexing the board:
EN should sit steady at 0V while enabled, VMOT steady at 24V, and Teensy GND to
board GND steady at 0.00V. Whichever one moves is the fault. A temporary jumper
from that board's EN pin straight to GND bypasses the Teensy EN wire; if the
dropouts stop, the EN net was the fault. Note that this jumper also defeats the
`E 0` software disable and the emergency stop's ability to de-energize that
board, so use it only as a bench test, never for normal operation.

The firmware cannot see any of this. There is no current or voltage sense on
the driver boards, so `StepperController` keeps counting steps it believes it
issued, and `POS` keeps reporting angles for joints that are actually limp. The
`HOMED` flag also stays true. After any dropout the reported pose is wrong and
the web app will plan Cartesian moves against it. Re-home (`H ALL`) after every
dropout; re-enabling with `E 1` does not recover the lost steps.

**Problem:** Position drift
- Verify USTEPS_PER_DEG calibration
- Check for mechanical binding
- Ensure step pulses are 5Âµs wide

**Problem:** Endstop not detected
- Verify wiring: C â†’ Teensy pin, NO â†’ GND
- Endstop should read LOW when triggered
- Check INPUT_PULLUP is enabled

**Problem:** Jerky motion
- Prefer queue-based Cartesian execution (`TQ`) over many high-rate `J` commands
- Increase TMC2209 microstepping interpolation
- Reduce speed
- Check for mechanical resistance

## Files Description

- `firmware.ino` - Main sketch, setup() and loop()
- `config.h` - Hardware constants (pins, limits, calibration)
- `types.h` - Data structures (JointAngles, RobotState, etc.)
- `StepperController.h/.cpp` - Motor control and step generation
- `HomingController.h/.cpp` - Endstop handling and homing sequences
- `SerialProtocol.h/.cpp` - Command parsing and response formatting

## Safety Notes

âš ï¸ **IMPORTANT:**
- Always test with motors disabled first
- Keep E-stop circuit functional
- Never bypass endstop safety
- Start with low speeds (5-10Â°/s)
- Verify soft limits in `config.h` match physical robot
- Add mechanical hard stops as backup

## Next Steps

After firmware is working:
1. âœ… Verify all 6 joints move correctly
2. âœ… Calibrate USTEPS_PER_DEG for each joint
3. âœ… Test homing sequence
4. âœ… Move to Phase 2: Web Application


