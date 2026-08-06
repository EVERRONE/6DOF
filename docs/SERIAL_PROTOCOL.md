# 6DOF Robot Arm - Serial Protocol Specification

**Version:** 2.0
**Transport:** USB CDC (Teensy 4.1). `Serial.begin(115200)` is nominal - the link
runs at USB speed, so throughput is not a constraint.

---

## Overview

An ASCII line protocol. The host sends commands, the firmware sends replies and
periodic reports. Every line is terminated with `\n`; a `\r` is tolerated and
ignored.

Two things drive the design and matter to any client:

**The firmware owns motion timing.** It holds a queue of moves and plans a
continuous velocity profile across them, so consecutive points are run as one
motion instead of a stop at each one. The host's job is to keep that queue fed,
not to pace points off its own clock.

**Move commands are acknowledged, and the acknowledgement carries queue space.**
That is the flow control: send, read the free-slot count, send more. When the
queue is full the firmware answers `BUSY` and the host resends.

---

## Commands (host to firmware)

### `J` - queue a joint move

```
J <j1> <j2> <j3> <j4> <j5> <j6> [speed]
```

| Field | Meaning |
|-------|---------|
| `j1`..`j6` | Absolute target angles in degrees. Clamped to `JOINT_MIN`/`JOINT_MAX`. |
| `speed` | Optional, deg/s. Applies to the joint with the largest angular displacement, and is clamped to the per-joint limits in `config.h`. Omitted means `DEFAULT_SPEED`. |

Replies:

```
OK J <freeSlots>      queued; freeSlots is the room left afterwards
BUSY <freeSlots>      queue full, nothing was queued - resend this move
ERROR <reason>        rejected (see below)
```

`BUSY` is normal back-pressure while streaming, not a fault. Resend the same
move after a short wait.

Rejected when the motors are disabled, an emergency stop is latched, homing is in
progress, or fewer than six angles were given.

```
J 10.5 20.0 35.5 100.0 180.5 -45.0 30.0
J 10.5 20.0 35.5 100.0 180.5 -45.0
```

> A move to the position already queued is accepted and discarded - it reports
> `OK J` but occupies no queue slot.

### `H` - home

```
H ALL          every joint that has an endstop, in order
H 2            one joint
H 2345         several joints, in the order given
```

Reply: `OK Homing`, immediately. **Homing is asynchronous**; it does not block
the controller, and position reports keep flowing throughout. Progress arrives as
`HOMED <n>` per joint and `OK Homed` at the end, or `ERROR <reason>` on failure.

Only joints with an endstop can be homed - J2 to J5 in the stock configuration.
Asking for J1 or J6 is rejected.

Each joint is homed in four acceleration-limited moves: fast seek onto the
switch, back-off, slow second approach for repeatability, then a move to the
resting pose in `POST_HOME_ANGLES`.

Failure reasons:

| Reason | Meaning |
|--------|---------|
| `Endstop not found in travel range, check HOME_TOWARD_MIN direction` | The seek covered the joint's whole range without the switch closing. Wiring, or the seek direction is wrong. |
| `Endstop still closed after back-off` | The switch did not release. Stuck or miswired; homing stops rather than drive into the hard stop. |
| `Endstop not found on fine approach` | The switch closed on the fast seek but not on the slow one. |
| `Homing phase timed out` | A phase exceeded its watchdog. |
| `Emergency stop during homing` | `S` arrived mid-run. |

### `Q` - query

```
Q
```

Sends `POS`, `ENDSTOP` and `STATUS` immediately.

### `E` - enable or disable the drivers

```
E 1     energise
E 0     de-energise
```

Replies `OK Motors enabled` / `OK Motors disabled`.

`E 1` also clears a latched emergency stop. `E 0` stops motion first, then cuts
the drivers - on a geared arm that means it can sag under its own weight.

### `S` - emergency stop

```
S
```

Reply: `OK Emergency stop`.

Cuts step generation immediately and drops the queue. The drivers stay
energised, because de-energising them would let the arm fall. Stopping from speed
without a ramp can lose steps, so this clears the position-trusted flag and the
arm should be re-homed. The stop stays latched until `E 1`.

### `A` - abort

```
A
```

Reply: `OK Aborted`.

Decelerates to a stop within the normal acceleration limit, then drops the queue.
Position stays trusted. This is the right stop for a user-requested cancel or
pause; `S` is for emergencies.

### `V` - motion limits, for tuning by ear

```
V                       report every axis
V <n> <speed> <accel>   set one axis, 1-based, deg/s and deg/s^2
V RESET                 back to the config.h values
```

Reply to a query, one line per axis:

```
LIMIT <n> <speed> <accel> <maxSpeed> <maxAccel>
```

`speed`/`accel` are what is in force; `maxSpeed`/`maxAccel` are `TUNING_MAX_*`
in `config.h`, the ceiling a request is held to. Setting replies with what was
actually taken, which is not what was asked for when the request went past the
ceiling:

```
OK V <n> <speed> <accel>
```

Acceleration decides whether the arm is quiet and can only be set by ear — run,
listen, adjust, run again. Held in `config.h` alone that costs a re-flash per
attempt.

**Nothing is persisted.** The firmware forgets on reboot, deliberately: a tuning
value that survives is one somebody forgets they left in, and then `config.h` no
longer describes the machine.

Refused while the arm is moving, which would alter the block the interrupt is
executing halfway through it.

> Past what an axis can hold, a stepper loses steps in silence — nothing detects
> it, and the position report keeps counting. Re-home after any run that ground.

---

## Reports (firmware to host)

Sent every `REPORT_INTERVAL_MS` (50 ms by default), and on demand after `Q`. If
the host is not draining the port, a round is skipped rather than blocking the
control loop.

### `POS`

```
POS <j1> <j2> <j3> <j4> <j5> <j6>
```

Current joint angles in degrees, two decimals. **Live during motion** - the
angles are read from the live step counters, so the host tracks the arm as it
moves.

### `ENDSTOP`

```
ENDSTOP <e1> <e2> <e3> <e4> <e5> <e6>
```

`1` = switch closed, `0` = open. Debounced. Joints without a switch always
report `0`.

### `STATUS`

```
STATUS <state> <queueFree> <moving> <positionTrusted> <homedMask> <enabled>
```

| Field | Meaning |
|-------|---------|
| `state` | `IDLE`, `MOVING`, `HOMING`, `ERROR` or `ESTOP` |
| `queueFree` | Free slots in the motion queue |
| `moving` | `1` while step generation is active |
| `positionTrusted` | `0` when a hard stop may have lost steps, or the arm has not been fully homed |
| `homedMask` | Bit *i* set means joint *i+1* has a datum. `30` = J2..J5 |
| `enabled` | `1` when the drivers are energised. Reported so the host need not assume its own `E` command succeeded |

`state` is `IDLE` only when the queue is empty **and** the arm has stopped, so it
is the correct thing to wait on for "motion finished". Note that a `STATUS` can
be in flight when a move is queued, so wait for a report timestamped after the
last command rather than acting on the first `IDLE` seen.

### Other lines

```
LIMIT <n> ...          one axis's motion limits, in reply to V
OK <text>              command accepted
ERROR <text>           command rejected, or an asynchronous fault
HOMED <n>              joint n finished homing (1-based)
OK Robot arm ready     sent once at start-up
```

`ERROR Command too long` means a line exceeded the receive buffer and was
dropped; the parser resynchronises at the next newline.

---

## Streaming a trajectory

```
1. E 1
2. H ALL, wait for "OK Homed"
3. For each point:
     send  J <angles> <speed>
     read  OK J <free>   -> continue
           BUSY <free>   -> wait ~20 ms, resend the same point
4. Wait for a STATUS with state=IDLE, timestamped after the last command.
```

Do not sleep between points to pace them. The firmware's planner produces the
timing, and a shallow queue is what forces it to decelerate to a stop at each
point. Keeping the queue full is the whole mechanism by which a streamed path
comes out smooth.

The queue holds `MOTION_QUEUE_LENGTH - 1` moves (23 by default). The firmware
also waits briefly - `START_QUEUE_DEPTH` moves or `START_DELAY_MS` - before
starting, so that a stream has look-ahead from the very first move. A single jog
command still starts within `START_DELAY_MS`.

### Corner behaviour

Junction speed scales with the cosine of the turn angle in joint space:
collinear moves run through at full speed, a reversal stops, and a right-angle
corner stops as well. That last one is deliberate - carrying speed through a
sharp corner would need a step change in a joint's velocity, which is the jolt
the planner exists to remove. A finely sampled path never hits it, because
consecutive points are nearly collinear.

---

## Changes from version 1.0

| Area | Was | Now |
|------|-----|-----|
| `J` reply | `OK Moving` | `OK J <free>` / `BUSY <free>`, so the host can apply back-pressure |
| Motion model | one target, recomputed per command | queue with look-ahead planning |
| `speed` argument | required | optional |
| Homing | blocking; froze the controller for the whole run | asynchronous state machine |
| `POS` during a move | stale; only refreshed when a move finished | live |
| `STATUS` | did not exist | queue space, homed flags, position trust, driver state |
| `A` (abort) | did not exist | graceful decelerate-and-clear |
| Position trust | not tracked | cleared by a hard stop, set by homing |

A version 1.0 client that sends `J ... <speed>` and reads `POS`/`ENDSTOP` still
works, except that it must tolerate `OK J <n>` where it expected `OK Moving`.
