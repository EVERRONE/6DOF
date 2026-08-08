# Agent Control API — Design Study

Status: **design proposal, not implemented**
Audited against code on 2026-08-08.

> **Revision note (settled).** Section 4 has been rewritten. The topology is now
> fixed by three requirements that arrived after the first draft: the agent is
> **Hermes Agent** (Nous Research) running on a separate Mac mini and reached
> over Telegram/WhatsApp; the arm should not depend on the user's own laptop
> being the only host; and — decisively — **this is an add-on that must not
> change existing behaviour.**
>
> That last constraint removes the "extract the planner out of the store"
> option as a first step. The browser keeps the serial port and keeps running
> `moveToPosition` exactly as it does today; the bridge calls it.
>
> Sections 1–3 and 5–9 (where the intelligence lives, the straight-line
> requirement, frame semantics, the tool surface, safety, latency and drift)
> remain valid as written.

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

A browser cannot listen on a socket, so the API cannot live inside the web app.
But the browser can dial *out*. That single fact settles the layout.

```
Telegram / WhatsApp
        │
     Hermes ──localhost──> broker            [mac mini — always on]
                              ▲
                              │ WebSocket, dialled OUT by the laptop
                              │
             Chrome + web app ──Web Serial──> Teensy ──> arm    [laptop, at the robot]
```

### Why the broker sits with the agent, not with the robot

The intuitive placement is "broker on the laptop, because that is where the
serial port is". That is backwards. Since the browser initiates the WebSocket,
the broker does not need to be where the port is — and putting it next to
Hermes buys three things:

- **Hermes → broker never leaves the Mac mini.** That hop is loopback, so it
  needs no network authentication and has no exposure.
- **The laptop needs no inbound port** — no forwarding, no firewall rule, no
  static address. It dials out and holds the connection open.
- **The stable machine hosts the stable endpoint.** A laptop sleeps, moves and
  changes IP; the always-on Mac mini does not. Pointing Hermes at a laptop
  would mean aiming at a moving target.

The only cross-machine hop is the outbound WebSocket, inside the user's own
LAN, bearer-token authenticated. Tailscale is the upgrade path if the laptop
should ever work from outside the house — a config change, not a redesign.

### Why not a headless Node executor (yet)

Owning the serial port from Node would need `SerialManager`'s transport swapped
to `serialport` **and** the ~700-line `moveToPosition` orchestration extracted
out of the Zustand store. Serial ports are exclusive, so the existing web UI
would have to be rewired through the daemon or abandoned, and the 3D viewer
would stop being live feedback.

That is a refactor of the core, which conflicts directly with the add-on
constraint. It remains the right long-term shape — and it is why the bridge
speaks a transport-neutral `RobotCommand` / `RobotEvent` contract. The broker
does not care whether the executor is a browser tab or an in-process Node
executor, so that migration later becomes "swap the executor", not "rewrite
the API".

Two findings keep that door open: `runStage2PlannerInline` already handles the
case where `Worker` is undefined, so a Node port gets Stage 2 for free; and the
firmware has **no link watchdog** (`SerialProtocol.cpp:763` is a homing timeout
only), which any future wireless-below-the-planner design would have to fix
first.

### Why not an ESP32 serial bridge

Tempting, since the hardware is on hand, but it puts the network boundary
*below* the planner: every one of the N `TQ PT` points would cross WiFi, and
the E-stop path would depend on the link — with no firmware watchdog to catch a
drop mid-queue. It also would not remove the need for a planner host. The
governing principle is **put the network boundary above the planner**: let the
network carry `"move 5 mm right"`, not the serial protocol.

### How Hermes attaches

Verified against the Hermes Agent docs: custom tools are **Python** (a plugin in
`~/.hermes/plugins/`, or a built-in), so wrapping a plain REST endpoint costs
code. MCP, by contrast, has been supported as a client since v0.2.0 over
stdio / HTTP / SSE and is **configuration only**:

```yaml
# ~/.hermes/config.yaml
mcp_servers:
  robotarm:
    url: "http://127.0.0.1:8765/mcp"
    headers:
      Authorization: "Bearer ***"
    tools:
      exclude: [move_relative, move_to]   # config-level kill switch for motion
```

So the broker exposes **both**: plain HTTP/JSON as the real API (for `curl`,
debugging, and any future client) with MCP as a thin adapter over it. The
`tools.exclude` filter is a genuine second lock, on a different layer than the
arming switch in the browser.

### 4.1 Completion and failure semantics — the easiest thing to get wrong

`moveToPosition` is `async`, but **awaiting it does not mean the arm arrived.**
The promise resolves once `TQ RUN` is acknowledged, while the motion is still
playing out. Worse, it **never throws**: every failure path is caught and
reported by setting `planningState: 'failed'` and `ikStatus.error`.

A bridge that does `await moveToPosition(...)` and reports success would
therefore be wrong twice over — reporting completion mid-motion, and reporting
success on failure.

The executor must instead subscribe to the store and settle on whichever comes
first:

| Signal | Meaning |
|---|---|
| `planningState === 'failed'` | failed — report `ikStatus.error` + `planningNotes` verbatim |
| `TQ_DONE` → `robotState` back to `IDLE` | arrived |
| timeout | unknown — report as such, do not claim success |

### 4.2 Arming is a window, not a click

Requiring a human to arm each command defeats the point when the operator is
texting from another room. Arming is therefore a **time-boxed window** (default
30 minutes) started at the laptop, with a visible countdown and automatic
expiry. A human still has to have been at the machine; they just do not have to
stay there.

### 4.3 A dry run belongs in phase 1

Frame semantics are the most likely thing to be wrong on first run, and the
worst way to discover it is by watching the arm move. `preview_move` resolves
the direction and reports the would-be target **without** calling
`moveToPosition` — pure bridge-side FK, no store interaction, no motion. It
ships before any motion command does.

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

## 9. Phasing

**Phase 1 + 2 — implemented.** Contract, broker, browser executor, arming
window, and the three commands that cannot move the arm:

- `get_status` — connection, motors, homed joints, TCP pose, busy, last error
- `preview_move` — dry run: resolves the direction and reports the would-be
  target without touching the store
- `stop` — privileged, bypasses arming and the busy guard

**Phase 3 — implemented, not yet run on hardware.** `move_relative` and
`move_to` through `moveToPosition`, with the settle state machine from §4.1,
arming enforced in the executor, a busy guard in both layers, a 150 mm
displacement ceiling in both layers, and the resolved vector echoed on every
response. `keep_orientation` defaults to true and passes the current tool
rotation explicitly, which forces `pose_lock` regardless of the app's Cartesian
mode.

Everything that can be checked without hardware is covered by tests: frame
resolution, parameter handling, clamping, and every refusal path. What remains
untested is the browser-to-broker link and the arm itself.

**Phase 4 — MCP adapter** over the HTTP API, so Hermes attaches by config
alone (§4).

**Phase 5 — optional, later.** Extract the `moveToPosition` orchestration out
of the store, add a Node `serialport` transport, run headless. Independently
valuable: 700 lines of planning orchestration inside a Zustand store is the
biggest testability problem in the web app today.

### The add-on constraint, concretely

Exactly one existing file is modified — `App.tsx`, one line to mount the panel.
Everything else is new: the `robot-agent-bridge/` package, `src/agent/*`, and
`AgentControlPanel.tsx`. Nothing in `kinematics/`, `motion/`, `services/`,
`communication/`, `store/` or the firmware changes.

The bridge connection is **opt-in and off by default**. With it off — or with
the broker simply not running — the app behaves exactly as it does today. That
is the property that makes this safe to merge.

---

## 10. Decisions

Settled:

1. **Topology** — broker beside the agent on the Mac mini; browser keeps the
   serial port (§4).
2. **Hermes attaches over MCP**, with plain HTTP/JSON underneath (§4).
3. **Arming is a time-boxed window**, not a per-command click (§4.2).
4. **Completion is observed from the store**, never from the `moveToPosition`
   promise (§4.1).

Still open, and only needed before phase 3:

5. **Default frame for "right"** — `view` with a configurable yaw
   (recommended) or plain `base` (`−Y`). `preview_move` exists so this can be
   answered by experiment rather than argument.
6. **Default step distance** when the agent gives no number, and the hard
   per-command cap. Suggested: 50 mm default, 150 mm cap.
7. **Orientation policy** — keep the tool orientation locked by default
   (`pose_lock`, harder to solve near limits) or fall back to position-only
   automatically when pose-lock fails.
