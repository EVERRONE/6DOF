# 6DOF Robot Arm - Teensy Firmware

## Overview
This firmware controls a 6-axis robot arm using a Teensy 4.1 microcontroller with TMC2209 stepper drivers.

## Hardware Requirements
- Teensy 4.1
- 6× TMC2209 stepper drivers (1/16 microstepping)
- 6× NEMA stepper motors
- 4× Omron D2F-L endstops (J2-J5)
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
- Enable Pin: 8 (Active LOW - connect to all driver EN pins)
- Endstops: J2(30), J3(31), J4(32), J5(33)

## Uploading Firmware

### Prerequisites
1. Install Arduino IDE 2.x or PlatformIO
2. Install Teensyduino from https://www.pjrc.com/teensy/td_download.html

### Steps
1. Open `firmware.ino` in Arduino IDE
2. Select **Tools → Board → Teensy 4.1**
3. Select **Tools → USB Type → Serial**
4. Select **Tools → CPU Speed → 600 MHz**
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
Expected: J1 moves to 10 degrees at 20°/s

### Test 4: Home a Joint
Send: `H 2`
Expected:
- J2 moves toward endstop
- Stops and backs off
- Re-approaches slowly
- Moves to post-home position (5°)
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
2. Command a 360° rotation: `J 360 0 0 0 0 0 20`
3. Measure actual rotation with a protractor
4. Adjust `USTEPS_PER_DEG` in `config.h` if error > 5°
5. Re-upload firmware

### Homing Sequence
The homing sequence is:
1. **Fast approach** - Move toward endstop at HOMING_SPEED (10°/s)
2. **Back off** - Move 2° away from endstop
3. **Slow approach** - Re-approach at 20% of HOMING_SPEED
4. **Set zero** - Define current position as 0°
5. **Move to post-home** - Move to safe position (defined in config.h)

## Troubleshooting

**Problem:** Motors don't move
- Check enable pin is LOW (8 → GND through Teensy)
- Verify driver Vref is set correctly (~0.6-1.0V depending on motor)
- Check 24V power supply

**Problem:** Position drift
- Verify USTEPS_PER_DEG calibration
- Check for mechanical binding
- Ensure step pulses are 5µs wide

**Problem:** Endstop not detected
- Verify wiring: C → Teensy pin, NO → GND
- Endstop should read LOW when triggered
- Check INPUT_PULLUP is enabled

**Problem:** Jerky motion
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

⚠️ **IMPORTANT:**
- Always test with motors disabled first
- Keep E-stop circuit functional
- Never bypass endstop safety
- Start with low speeds (5-10°/s)
- Verify soft limits in `config.h` match physical robot
- Add mechanical hard stops as backup

## Next Steps

After firmware is working:
1. ✅ Verify all 6 joints move correctly
2. ✅ Calibrate USTEPS_PER_DEG for each joint
3. ✅ Test homing sequence
4. ✅ Move to Phase 2: Web Application
