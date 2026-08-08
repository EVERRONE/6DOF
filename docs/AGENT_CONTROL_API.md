# Agent Control API — Design Study

Status: **design proposal, not implemented**
Audited against code on 2026-08-08.

Question this document answers: *can we expose an API so an external AI agent
("Roberto") can drive the arm — so that "move right" produces a real
straight-line motion to the right?*

Short answer: **yes, and the hard parts are already built.** The work is not
kinematics; it is choosing where the API lives, defining what "right" means,
and adding the safety layer that a language model driving a physical machine
requires.

---

## 1. Where the intelligence actually lives

This is the constraint that determines everything else.

```
React app (browser)  ──Web Serial──>  Teensy 4.1
  FK / IK                              step generation
  Cartesian path planning              Hermite queue playback
  trajectory generation                joint limit clamps
  TQ upload
```

**The firmware has no concept of Cartesian space.** There is no `MOVE RIGHT`
in the serial protocol and one cannot be added cheaply: the Teensy has no IK,
no FK, and no path planner. It replays a queue of pre-computed joint knots
(`TQ PT`) with cubic Hermite interpolation. Everything that turns "a point in
space" into "joint angles over time" runs in the browser.

Therefore an agent API must sit at the level of `robotStore.moveToPosition()`
(`src/store/robotStore.ts:1468`), **not** at the serial protocol level. An API
that speaks `J`/`TQ` to the Teensy directly would have to reimplement the
entire kinematics stack.

### 1.1 Key finding: the brain is already portable

`kinematics/`, `motion/`, and `services/cartesian/` contain **zero** browser
API usage — no `window`, no `document`, no `localStorage`. The only
browser-specific line in the whole non-UI stack is:

```ts
// src/communication/SerialManager.ts:67
this.port = await navigator.serial.requestPort();
```

Everything else in `SerialManager` — the line parser, `sendAndAwait`, the TQ
transaction helpers — is portable JavaScript.

This matters enormously: running the planner headlessly in Node is a
**transport swap plus an orchestration extraction**, not a rewrite. The only
genuinely browser-fused piece is `moveToPosition` itself, ~700 lines of
orchestration currently living inside a Zustand store.

---

## 2. The straight-line requirement

There are two Cartesian entry points in the store and they behave very
differently. Picking the wrong one silently breaks the "op een lijn"
requirement.

| | `jogCartesian()` (`:2639`) | `moveToPosition()` (`:1468`) |
|---|---|---|
| Method | one endpoint IK solve → single `J` command | Stage1 + Stage2 Cartesian path sampling → N knots → `TQ` queue |
| TCP path | **joint-space interpolation → curved** | **sampled straight line in Cartesian space** |
| Orientation | solved at endpoint only | SLERP'd along the path, pose-locked |
| Validation | none — IK failure just `return`s | fail-closed joint-limit check, queue budget, strict residual gates |
| Error reporting | **none** (returns `void`, silent on every failure) | `planningState`, `ikStatus`, `planningNotes` |
| Latency | milliseconds | ~100 ms to 20 s (Stage2 budget, `:219-220`) |

`jogCartesian` looks like it moves in a line because the UI default step is
5 mm (`CartesianControlPanel.tsx:65`), where joint-space deviation is
negligible. At 50–100 mm the curvature is real. It also sends a fixed speed of
30 °/s and ignores the trajectory queue entirely (`:2729-2736`).

**Conclusion: agent line-moves must route through `moveToPosition`.** Wrapping
`jogCartesian` would appear to work and would quietly violate the requirement —
and would give the agent no way to tell success from failure, since it reports
nothing at all.

`moveToPosition(position, targetOrientation)` switches to `pose_lock` whenever
an orientation is passed (`:1483`). Passing the *current* tool orientation is
exactly the "move sideways without tilting the tool" semantic a person means.

---

## 3. What does "right" mean?

The runtime world frame (`docs/KINEMATICS.md:33-36`, `:431-435`):

- **X** = forward (front of robot)
- **Y** = left (right-hand rule)
- **Z** = up

So mechanically, **"right" = −Y**. But "right" is viewpoint-relative, and this
is the single most under-specified thing in the whole request. Three candidate
frames:

| Frame | "right" resolves to | When it is what a human means |
|---|---|---|
| `base` | always `−Y` | operator stands behind the arm, looking along `+X` |
| `tool` | perpendicular to current gripper axis | almost never, in casual speech |
| `view` | `−Y` rotated by a configured `viewYawDeg` | whenever the human is *not* standing behind the arm |

**Recommendation:** support all three, default to `view`, and let `view`
default to `viewYawDeg = 0` (identical to `base`) until configured. Add a
`set_view_frame({ yaw_deg })` tool so the human can teach the API where they
are sitting once.

**Every motion response must echo the resolved vector**, so both the agent and
the human can see what was understood:

```json
{
  "resolved": {
    "frame": "view",
    "viewYawDeg": 0,
    "worldDelta_mm": { "x": 0, "y": -50, "z": 0 }
  }
}
```

Without this echo, a frame misconfiguration is invisible until the arm moves
the wrong way.

### 3.1 The J1 sector limits how far "right" goes

`JOINT_MIN/MAX[0] = ±60°` (`firmware/config.h:36-37`) — the arm covers a 120°
sector in front of the base, not a full circle. Repeated "move right" commands
hit the J1 limit quickly. The planner already produces an actionable message
for this ("Target is outside the robot's reachable workspace (joint limit)…")
and the API should pass it through verbatim rather than mapping it to a code.

Workspace envelope for sanity checks (`reachabilityAtlas.generated.json`):
`x,y ∈ [−0.32, 0.32] m`, `z ∈ [0, 0.42] m`.

---

## 4. Topology: where does the API run?

A browser cannot listen on a socket, so there are three shapes.

### Option A — Headless Node executor

Node owns the serial port; the agent talks to Node.

- Swap `SerialManager`'s transport to the `serialport` package (~50 lines; the
  parser is already portable).
- Extract the `moveToPosition` orchestration out of the Zustand store into a
  framework-agnostic service. **This is the real work.**
- Node exposes HTTP + MCP.

**Pros:** works with no browser open; can run headless on a Pi; one robust daemon.
**Cons:** biggest refactor. Serial ports are exclusive — the web UI and the
daemon cannot both be connected, so the existing UI must be rewired through
the daemon or abandoned. The 3D viewer stops being live feedback.

### Option B — Browser stays master, local broker relays

A small Node broker exposes HTTP/MCP to the agent and a WebSocket that the
browser app dials *out* to. The browser executes commands by calling the same
store actions the UI buttons call.

**Pros:** zero duplication of kinematics; single source of truth; smallest
diff. The UI, the 3D viewer, and the E-stop button all stay live and show
exactly what the agent is doing — which is a genuine safety property, not just
convenience.
**Cons:** requires a browser tab open. Extra hop (negligible against
multi-second planning). Background tab throttling is worth verifying if the
user switches away.

### Option C — MCP server inside the browser

Not possible. A page cannot accept inbound connections.

### Recommendation

**Option B first, with the command contract designed so Option A drops in
later.** Define one JSON `RobotCommand` / `RobotEvent` contract. The broker
does not care whether the executor is a browser tab over WebSocket or an
in-process Node executor — so Option A later becomes "swap the executor",
not "rewrite the API".

```
Roberto ──MCP/HTTP──> broker (Node, localhost) ──WS──> browser app ──Web Serial──> Teensy
                                                └── later: in-process Node executor
```

---

## 5. Proposed API surface

Designed for a language model specifically: few tools, unambiguous names,
**millimetres and degrees** (not metres/radians — the UI already displays mm,
and LLMs reason about mm far more reliably), and every response echoing what
was actually done.

| Tool | Purpose |
|---|---|
| `get_status()` | connection, motors, homed J2–J5, TCP pose (mm), joint angles, busy, last error, frame config |
| `move_relative({direction \| dx,dy,dz, distance_mm, frame, keep_orientation, speed_mm_s, wait})` | straight-line relative move |
| `move_to({x_mm, y_mm, z_mm, keep_orientation, wait})` | straight-line absolute move |
| `move_joints({J1..J6, speed})` | point-to-point escape hatch for recovery |
| `home()` | `H ALL` |
| `enable_motors({enabled})` | |
| `stop()` | emergency stop — **privileged, never queued** |
| `set_view_frame({yaw_deg})` | teach the API where "right" is from the human's seat |

`direction` as an enum — `left | right | forward | backward | up | down` — is
what makes "naar rechts" trivial to map from natural language, with
`distance_mm` defaulting to a small value (50 mm suggested) and hard-capped.

Preconditions the agent will hit constantly (`isCartesianReady()`, `:1287`):
connected, motors enabled, **J2–J5 homed**, kinematics frame synced. These must
be surfaced by `get_status()` in plain language, because the agent cannot
recover from a precondition it cannot see.

### Error handling

The planner's existing failure messages are already written for humans:

> *"Pose lock infeasible along Cartesian path at current limits. Switch to
> Position Only and retry."*

These are ideal LLM feedback — **pass them through verbatim**. Do not collapse
them into error codes.

---

## 6. Safety model

A language model driving a physical machine will eventually emit a wrong
command. The guardrails must be structural, not dependent on the model
behaving well.

1. **Arming.** The API refuses all motion until a human clicks "Allow AI
   control" in the UI. Auto-disarm on idle timeout and on disconnect.
2. **Per-command caps.** Max distance per command (suggest 150 mm) and max
   speed, enforced server-side. Clamp or reject beyond it; never trust the
   requested value.
3. **One move at a time.** Reject a new motion while `moveInProgress` or the
   queue is running. Note: `moveToPosition` cancels the previous *planner* via
   `activePlannerRequestId` (`:1485`), but **a queue already running on the
   Teensy keeps running** — so the API layer must add its own busy guard rather
   than relying on planner cancellation.
4. **`stop()` is privileged** — bypasses the busy guard, maps to
   `emergencyStop()` → firmware `S`.
5. **Auth even on localhost.** A local HTTP server is reachable from *any* web
   page the user visits, via `fetch` to `127.0.0.1`. Require a bearer token,
   bind to loopback only, and check `Origin`. This is a standard local-daemon
   attack, not paranoia.
6. **Rate limit** motion commands.
7. **Audit log** of every agent command: timestamp, resolved world vector,
   outcome. When a physical machine does something unexpected you need the trace.
8. **Keep every existing guard**: fail-closed joint-limit validation
   (`TrajectoryExecutionService.validateTrajectoryPointsAgainstConstraints`),
   queue point budget, reachability atlas pre-check, firmware clamps.

### Out of scope for the agent

- **`CAL` and `HP` must stay read-only.** Home pose feeds
  `getEffectiveUrdfOffsets()` and therefore anchors the entire FK/IK/3D frame
  for J2–J5. An agent silently writing EEPROM calibration would corrupt the
  kinematic frame in a way that is very hard to diagnose.
- **No raw serial passthrough.** It bypasses every validation layer above.

The physical power cut remains the real safety layer. Software E-stop halts the
queue and the steppers, but does not defeat momentum.

---

## 7. Latency and completion

Planning is 100 ms – 20 s (`STAGE2_TIMEOUT_MIN_MS`/`MAX_MS` = 6000/20000, with
a strict-valid Stage1 fallback when Stage2 times out, `:2438-2566`). Short
jogs produce few points and converge fast; long moves can be slow.

For conversational control, **blocking with a timeout is the better default**
for LLM ergonomics — one call, one answer — with `wait: false` as an opt-out
returning a `move_id` to poll.

Completion signal: **`TQ_DONE`** is authoritative for queue moves (`:957`).
The `moveInProgress` flag also clears via a POS-stability heuristic (3
consecutive samples within 0.2°, `:797-827`), which is a fallback, not the
primary signal.

---

## 8. Drift on repeated relative moves

Each `move_relative` re-reads the *measured* pose via FK and adds the delta.
With a 2.5 mm tracking tolerance and a 1.5 mm final strict gate, ten
consecutive "right 50 mm" commands will **not** equal 500 mm, and will drift in
X and Z as well.

Recommendation: accept this and **return the actual resulting pose after every
move**, so the agent can observe and correct drift. The alternative — anchoring
a sequence to intended rather than measured poses — hides error accumulation
until it becomes large. Honesty is the better default here, but this should be
an explicit, documented choice.

---

## 9. Suggested phasing

1. **Contract + read-only.** Define `RobotCommand`/`RobotEvent`. Ship
   `get_status()` only. No motion. Validates the whole transport path safely.
2. **Broker + WS executor in the browser** (Option B), still read-only.
3. **Arming UI + `stop()`** — safety before motion.
4. **Motion**: `move_relative` / `move_to` via `moveToPosition`, with caps,
   busy guard, and resolved-vector echo.
5. **MCP adapter** over the HTTP API so Roberto gets typed tools.
6. *(Optional, later)* **Option A**: extract `moveToPosition` orchestration out
   of the store into a service; add a Node `serialport` transport; run headless.

Step 6 is also independently valuable as a refactor — 700 lines of planning
orchestration inside a Zustand store is the single biggest testability problem
in the web app today.

---

## 10. Open decisions

These change the implementation materially and are the user's call:

1. **Topology** — Option B (browser master, recommended) or Option A (headless
   Node) directly?
2. **Default frame for "right"** — `view` with a configurable yaw (recommended)
   or plain `base` (`−Y`)?
3. **Default step distance** when the agent says "right" without a number, and
   the hard per-command cap.
4. **Orientation policy** — keep tool orientation locked by default
   (`pose_lock`, harder to solve near limits) or allow position-only fallback
   automatically when pose-lock fails?
