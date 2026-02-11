# 6DOF Robot Arm - Serial Protocol Specification

**Version:** 1.0
**Date:** February 2026
**Baud Rate:** 115200 8N1

---

## Overview

The robot arm uses an ASCII-based serial protocol for communication between the web application and Teensy firmware. Commands are sent from the host (browser) to the robot, and responses/updates are sent from the robot back to the host.

---

## Command Format

All commands are ASCII strings terminated with a newline character (`\n`).

### General Structure
```
<COMMAND_TYPE> [<ARGUMENTS>]\n
```

---

## Commands (Host → Robot)

### 1. Move to Joint Angles (J)

**Format:**
```
J <j1> <j2> <j3> <j4> <j5> <j6> <speed>
```

**Parameters:**
- `j1` to `j6` - Target joint angles in degrees (float)
- `speed` - Movement speed in degrees/second (float)

**Example:**
```
J 10.5 20.0 35.5 100.0 180.5 -45.0 30.0
```

**Response:**
```
OK Moving
```

**Notes:**
- Angles are automatically clamped to joint limits (defined in firmware config.h)
- All joints move simultaneously (coordinated motion)
- Movement completes when all joints reach target

---

### 2. Home Joints (H)

**Format:**
```
H <joints>
```

**Parameters:**
- `joints` - Joint numbers to home (1-6), or "ALL"

**Examples:**
```
H 2          # Home joint 2 only
H 2345       # Home joints 2, 3, 4, 5 in sequence
H ALL        # Home all joints with endstops
```

**Response (per joint):**
```
HOMED <joint_number>
```

**Final Response:**
```
OK All joints homed    # (if H ALL was used)
```

**Notes:**
- Only joints with endstops can be homed (J2, J3, J4, J5)
- Homing sequence: fast approach → back off → slow approach → set zero → move to post-home position
- Joints are homed sequentially, not simultaneously

**Error Conditions:**
- `ERROR No endstop on this joint` - Attempted to home J1 or J6
- `ERROR Endstop not found` - Endstop not triggered within maximum travel

---

### 3. Query Position (Q)

**Format:**
```
Q
```

**Response:**
Immediately sends current position (same format as periodic POS messages).

**Example:**
```
POS 10.50 20.00 35.50 100.00 180.50 -45.00
```

---

### 4. Enable/Disable Motors (E)

**Format:**
```
E <state>
```

**Parameters:**
- `state` - `1` to enable, `0` to disable

**Examples:**
```
E 1    # Enable motors
E 0    # Disable motors
```

**Response:**
```
OK Motors enabled     # or
OK Motors disabled
```

**Notes:**
- Motors are disabled on startup for safety
- Disabling motors immediately stops any ongoing motion

---

### 5. Emergency Stop (S)

**Format:**
```
S
```

**Response:**
```
OK Emergency stop
```

**Behavior:**
- Immediately halts all joint motion
- Sets target position to current position
- Does NOT disable motors (they remain holding position)
- Robot state becomes ESTOPPED

---

## Responses (Robot → Host)

### Position Update (POS)

**Format:**
```
POS <j1> <j2> <j3> <j4> <j5> <j6>
```

**Example:**
```
POS 0.00 5.00 55.00 129.00 220.00 0.00
```

**Update Rate:**
- Sent automatically every 100ms
- Also sent immediately in response to `Q` command

**Precision:**
- 2 decimal places

---

### Endstop State (ENDSTOP)

**Format:**
```
ENDSTOP <j1> <j2> <j3> <j4> <j5> <j6>
```

**Values:**
- `0` - Endstop not triggered
- `1` - Endstop triggered

**Example:**
```
ENDSTOP 0 1 0 0 0 0
```
(J2 endstop is currently triggered)

**Update Rate:**
- Sent automatically every 100ms

---

### Homing Complete (HOMED)

**Format:**
```
HOMED <joint_number>
```

**Example:**
```
HOMED 2
```

**Notes:**
- `joint_number` is 1-indexed (1 = J1, 2 = J2, etc.)
- Sent when a joint successfully completes its homing sequence

---

### Acknowledgment (OK)

**Format:**
```
OK <message>
```

**Examples:**
```
OK Robot arm ready
OK Motors enabled
OK Moving
OK Emergency stop
OK All joints homed
```

---

### Error (ERROR)

**Format:**
```
ERROR <message>
```

**Examples:**
```
ERROR Unknown command
ERROR Invalid move command format
ERROR No endstop on this joint
ERROR Endstop not found
```

---

## Message Flow Examples

### Example 1: Connect and Move Joint

**Host → Robot:**
```
E 1
```

**Robot → Host:**
```
OK Motors enabled
```

**Host → Robot:**
```
J 15 0 0 0 0 0 20
```

**Robot → Host:**
```
OK Moving
```

**Robot → Host (periodic updates while moving):**
```
POS 3.25 0.00 0.00 0.00 0.00 0.00
ENDSTOP 0 0 0 0 0 0
POS 6.50 0.00 0.00 0.00 0.00 0.00
ENDSTOP 0 0 0 0 0 0
...
POS 15.00 0.00 0.00 0.00 0.00 0.00
ENDSTOP 0 0 0 0 0 0
```

---

## Future Extensions

Potential future commands (not yet implemented):

- `C <x> <y> <z> <rx> <ry> <rz> <speed>` - Move to Cartesian coordinates (requires IK)
- `P <path_id>` - Execute predefined path
- `G <gcode>` - Execute G-code command
- `T <tool_state>` - Control end-effector tool

---

## Changelog

**v1.0 (2026-02-09)**
- Initial protocol specification
- Implemented: J, H, Q, E, S commands
- Implemented: POS, ENDSTOP, HOMED, OK, ERROR responses
