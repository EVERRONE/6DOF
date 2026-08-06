# 6DOF Robot Arm - Serial Protocol Specification

**Version:** 1.4  
**Date:** February 2026  
**Baud Rate:** 115200 8N1

---

## Overview

The robot arm uses an ASCII line protocol over serial.
Each command is newline-terminated (`\n`).

## Command Format

```text
<COMMAND> [ARGS]\n
```

## Commands (Host -> Robot)

### 1) Move Joints (`J`)

```text
J <j1> <j2> <j3> <j4> <j5> <j6> <speed>
```

- Angles are logical degrees.
- Speed is logical deg/s.
- Firmware clamps angles to configured limits.

Response:

```text
OK Moving
```

---

### 2) Home (`H`)

```text
H <joints>
```

Examples:

```text
H 2
H 2345
H ALL
```

Behavior:
- Only joints with endstops can be homed (`J2..J5`).
- Endstop homing is unchanged: move to switch -> set logical zero -> stay on switch.
- `H ALL` homes in sequence (`J2 -> J3 -> J4 -> J5`).
- If operational home pose is enabled, `H ALL` then executes post-home move to configured home-pose targets.

Responses:

```text
HOMED <joint_number>      # per joint
HOMEPOSE_REACHED          # only when post-home move succeeds
OK All joints homed       # final H ALL success
```

Errors:
- `ERROR No endstop on this joint`
- `ERROR Endstop not found`
- `ERROR HOMEPOSE <reason>`

---

### 3) Query Position (`Q`)

```text
Q
```

Response: immediate `POS ...`

---

### 4) Enable Motors (`E`)

```text
E <0|1>
```

Responses:

```text
OK Motors enabled
OK Motors disabled
```

---

### 5) Emergency Stop (`S`)

```text
S
```

Response:

```text
OK Emergency stop
```

---

### 6) Configuration (`CFG`)

Query:

```text
CFG?
```

Response:

```text
CFG {json}
```

Set (calibration arrays only):

```text
CFG <json>
```

Notes:
- Includes joint config, calibration, timing settings, and `homePose` object.
- Use `CAL SAVE` to persist calibration.

---

### 7) Calibration (`CAL`)

```text
CAL SET J<n> <scale> <offset>
CAL ZERO J<n> <logical_deg>
CAL SAVE
CAL LOAD
CAL RESET
```

Notes:
- `J<n>` is 1-indexed (`J1..J6`).
- `CAL ZERO` adjusts offset at current physical pose.

---

### 8) Relative Jog (`JR`)

```text
JR J<n> <delta_deg> <speed>
```

- Delta is relative logical degrees.
- Speed is logical deg/s.

---

### 9) Operational Home Pose (`HP`)

Use these commands to configure post-`H ALL` operational home positioning.

```text
HP?
HP EN <0|1>
HP SET J<n> <deg>         # J2..J5 only
HP SETALL <j2> <j3> <j4> <j5>
HP SPD <deg_s>            # clamped to 5..40
HP SAVE
HP LOAD
HP RESET
```

Notes:
- Endstop zero reference remains unchanged.
- Post-home operational move applies only to `J2..J5`.
- Trigger is `H ALL` only.
- UI uses `homePose.jointsDeg` as URDF-default anchor for `J2..J5`
  (with direction multipliers applied for mirrored joints such as J5).

---

### 10) Trajectory Queue (`TQ`)

Queue-based execution is used for high-quality Cartesian motion.

```text
TQ CLEAR
TQ PT <t_ms> <q1> <q2> <q3> <q4> <q5> <q6> <qd1> <qd2> <qd3> <qd4> <qd5> <qd6>
TQ RUN
TQ STOP
TQ?
```

Notes:
- `q*` are logical joint angles (deg).
- `qd*` are logical joint velocities (deg/s).
- `t_ms` must be strictly increasing.
- Queue points are validated against firmware joint limits.
- `TQ` is intended for Cartesian/trajectory execution; manual controls keep using `J`.
- `TQ?` returns compatibility status (`TQ READY`) and, when supported, extended status (`TQ STAT`).
- Industrial host flow should treat `TQ` as a transaction: `TQ CLEAR -> TQ PT* -> TQ? (count verify) -> TQ RUN -> TQ?`.

---

### 11) Motion Kernel Diagnostics (`MQ`)

Diagnostics for firmware queue execution timing.

```text
MQ?
MQ RESET
```

Notes:
- `MQ?` requests the latest diagnostic counters immediately.
- `MQ RESET` clears counters.

---

## Responses (Robot -> Host)

### Position (`POS`)

```text
POS <j1> <j2> <j3> <j4> <j5> <j6>
```

- Sent every 100 ms.
- Also sent in response to `Q`.

### Endstop (`ENDSTOP`)

```text
ENDSTOP <j1> <j2> <j3> <j4> <j5> <j6>
```

- `0` not triggered, `1` triggered.
- Sent every 100 ms.

### Configuration (`CFG`)

```text
CFG {
  "version": 1,
  "protocolVersion": 3,
  "capabilities": {
    "trajectoryQueue": true,
    "trajectoryMaxPoints": 256,
    "trajectoryPointFormat": "hermite_v1",
    "motionKernelV2": true,
    "motionKernelDiag": true,
    "commandAckV1": true,
    "trajectoryErrorCodesV1": true,
    "trajectoryStatusV2": true
  },
  "joints": [...],
  "pulseWidthUs": 5,
  "dirSetupUs": 2,
  "endstopDebounceMs": 5,
  "homePose": {
    "enabled": false,
    "speedDegS": 10,
    "applyAfterHAll": true,
    "jointsDeg": [0,0,0,0,0,0]
  }
}
```

- `homePose.jointsDeg` is used by UI as the URDF default anchor for `J2..J5`.
- `capabilities.trajectoryQueue` indicates support for `TQ*` queue commands.
- `capabilities.motionKernelV2` indicates timer-driven deterministic stepping is active.
- `capabilities.motionKernelDiag` indicates `MQ STAT` diagnostics are available.

### Trajectory Queue Status (`TQ READY`)

```text
TQ READY <count>
```

- Sent on `TQ?`, `TQ CLEAR`, and `TQ RUN`.
- `<count>` is the number of queued points.

### Extended Trajectory Queue Status (`TQ STAT`)

```text
TQ STAT <count> <running> <point_index> <elapsed_ms> <seq>
```

- Optional additive status line (capability-gated).
- `<running>` is `1` while queue execution is active, else `0`.
- `<seq>` is the current acknowledgement sequence snapshot.

### Trajectory Queue Typed Error (`TQ ERR`)

```text
TQ ERR <code> [detail]
```

- Optional additive typed queue error (capability-gated).
- Firmware still emits compatibility `ERROR TQ ...`.

### Trajectory Queue Progress (`TQ PROG`)

```text
TQ PROG <point_index> <elapsed_ms>
```

- Emitted periodically while queue execution is active.

### Trajectory Queue Done (`TQ DONE`)

```text
TQ DONE
```

- Emitted when queued trajectory execution finishes.

### Motion Kernel Status (`MQ STAT`)

```text
MQ STAT <tick_jitter_us> <queue_underrun> <step_overrun>
```

- Emitted on `MQ?` and periodically while connected.
- `tick_jitter_us` is the max observed scheduler jitter window.
- `queue_underrun` increments if queue segment resolution falls behind expected order.
- `step_overrun` increments when control tick spacing exceeds 2x configured period.

### Home Pose Config (`HP`)

```text
HP {"enabled":false,"speedDegS":10,"applyAfterHAll":true,"jointsDeg":[0,0,0,0,0,0]}
```

- Sent in response to `HP?`.

### Homed (`HOMED`)

```text
HOMED <joint_number>
```

### Home Pose Reached (`HOMEPOSE_REACHED`)

```text
HOMEPOSE_REACHED
```

- Sent after successful post-home operational move.

### OK / ERROR

```text
OK <message>
ERROR <message>
```

### Command Ack (`ACK`)

```text
ACK <seq> <scope> [detail]
```

- Optional additive acknowledgement (capability-gated) for transactional host handling.

---

## Message Flow Example (H ALL with Operational Home Enabled)

Host:

```text
E 1
H ALL
```

Robot:

```text
OK Motors enabled
HOMED 2
HOMED 3
HOMED 4
HOMED 5
HOMEPOSE_REACHED
OK All joints homed
```
