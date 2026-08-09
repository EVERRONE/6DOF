# robot-agent-bridge

A small local broker that lets an external AI agent (Hermes) drive the 6DOF arm
**through the existing web app**, without changing how that app works.

Design rationale lives in [`../docs/AGENT_CONTROL_API.md`](../docs/AGENT_CONTROL_API.md).

## Shape

```
Telegram / WhatsApp
        │
     Hermes ──localhost──> broker            [mac mini — always on]
                              ▲
                              │ WebSocket, dialled OUT by the laptop
                              │
             Chrome + web app ──Web Serial──> Teensy ──> arm    [laptop, at the robot]
```

The broker sits **with the agent, not with the robot**. Because the browser
dials out, the machine holding the serial port needs no inbound port, no port
forwarding and no stable address — while the always-on machine hosting the
agent provides the stable endpoint.

## Status: phase 1–3, untested on hardware

| Command | HTTP | Effect |
|---|---|---|
| `get_status` | `GET /api/status` | Connection, motors, homed joints, TCP pose, readiness in plain words |
| `preview_move` | `POST /api/preview_move` | **Dry run.** Resolves a direction and reports the would-be target. Sends nothing to the robot. |
| `move_relative` | `POST /api/move_relative` | Straight-line relative move via the queue-based Cartesian path. **Requires arming.** |
| `move_to` | `POST /api/move_to` | Straight-line move to an absolute point in mm. **Requires arming.** |
| `stop` | `POST /api/stop` | Emergency stop. Privileged — works armed or not, and disarms afterwards. |

Diagnostics: `GET /health` (no auth), `GET /api/telemetry`, `GET /api/audit`.

Motion parameters: `direction` (`right`/`left`/`forward`/`backward`/`up`/`down`),
`distance_mm` (default 50, clamped to 150), `frame` (`base` or `view`),
`keep_orientation` (default true — holds the tool angle through the move),
`wait` (default true — blocks until arrival rather than until the queue starts).

**The motion path has not yet run against real hardware.** The frame
resolution, the parameter handling and every refusal path are covered by tests;
the browser-to-broker link and the arm itself are what the first session on the
bench is for.

## Running it

```bash
cd robot-agent-bridge
npm install
BRIDGE_TOKEN="$(openssl rand -base64 24)" npm start
```

If `BRIDGE_TOKEN` is unset a token is generated and printed for that run.
Never run it without one: a local HTTP server is reachable from any web page
the user happens to visit.

| Variable | Default | Meaning |
|---|---|---|
| `BRIDGE_PORT` | `8765` | Listening port |
| `BRIDGE_HOST` | `0.0.0.0` | Loopback for the agent, LAN for the browser executor |
| `BRIDGE_TOKEN` | generated | Bearer token; also the WebSocket query token |
| `BRIDGE_COMMAND_TIMEOUT_MS` | `20000` | How long a command waits for the executor |

Then in the web app: **Agent Control** panel → tick *Connect to agent bridge*,
paste the same token, and press **Arm AI control**.

## Connecting Hermes

Hermes custom tools are Python, but MCP is configuration-only, so the intended
path is the MCP adapter (phase 4). Until then the plain HTTP API works directly:

```bash
AUTH=(-H "Authorization: Bearer $BRIDGE_TOKEN" -H 'content-type: application/json')

curl -s "${AUTH[@]}" http://127.0.0.1:8765/api/status

# Dry run first — always. This resolves the direction and shows the target
# without sending anything to the robot.
curl -s "${AUTH[@]}" -d '{"direction":"right","distance_mm":5}' \
     http://127.0.0.1:8765/api/preview_move

# Then, once armed in the browser panel:
curl -s "${AUTH[@]}" -d '{"direction":"right","distance_mm":5}' \
     http://127.0.0.1:8765/api/move_relative
```

## Safety model

- **Arming is a time-boxed window** set by a human at the machine, in the
  browser. The executor is authoritative; the broker only mirrors the state for
  faster, clearer refusals.
- **`stop` is privileged** — never queued behind anything, and it disarms.
- **Bearer token on every endpoint** except `/health`.
- **Any request carrying an `Origin` header is rejected.** Real clients send
  none; browsers always do. That closes the "any page can reach localhost" hole.
- **One executor at a time.** A second connection is refused rather than
  silently taking over.
- **One move at a time.** A second motion command is refused while the arm is
  busy. `moveToPosition` cancels the previous *planner*, but a queue already
  running on the Teensy keeps running, so this guard cannot be skipped.
- **Distances are clamped** to 150 mm per command — in the broker *and* again in
  the executor, which is the copy that matters — and the clamp is reported back
  rather than applied silently.
- **Outcomes are observed, never assumed.** `moveToPosition` resolves at
  `TQ RUN`, not on arrival, and never throws; the executor watches the store for
  `TQ_DONE`, `planningState: 'failed'` or an emergency stop. On timeout the
  answer is "unknown", never "done".
- **Planner errors pass through verbatim.** They are already written for a human
  and say what to do next, which beats inventing a code.
- **Every command is audited** with its resolved vector and outcome.

None of this replaces the physical power cut, which remains the real safety
layer. A software stop halts the queue and the steppers; it does not defeat
momentum.

## Tests

```bash
npm test
```

Covers parameter validation and clamping, token and Origin rejection, the
no-executor path, command round-trip, verbatim error pass-through, the timeout
path, and the audit log.

There is also an **integration test on the web side**
(`robot-arm-control/src/agent/__tests__/bridgeIntegration.test.ts`) that starts
this broker in a child process and drives it with the real browser client over a
real WebSocket:

```bash
cd ../robot-arm-control
npm test -- --watchAll=false --runInBand --testPathPattern bridgeIntegration
```

That is the only test forcing `src/protocol.js` here and `bridgeProtocol.ts`
there to agree — renaming a field on one side fails it. Run it after touching
either.
