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

## Status: phase 1 + 2

Nothing here can move the arm yet. Shipped commands:

| Command | HTTP | Effect |
|---|---|---|
| `get_status` | `GET /api/status` | Connection, motors, homed joints, TCP pose, readiness in plain words |
| `preview_move` | `POST /api/preview_move` | **Dry run.** Resolves a direction and reports the would-be target. Sends nothing to the robot. |
| `stop` | `POST /api/stop` | Emergency stop. Privileged — works armed or not, and disarms afterwards. |

`move_relative` / `move_to` arrive in phase 3, once this has been exercised on
real hardware.

Diagnostics: `GET /health` (no auth), `GET /api/telemetry`, `GET /api/audit`.

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
curl -s -H "Authorization: Bearer $BRIDGE_TOKEN" http://127.0.0.1:8765/api/status
curl -s -H "Authorization: Bearer $BRIDGE_TOKEN" -H 'content-type: application/json' \
     -d '{"direction":"right","distance_mm":5}' \
     http://127.0.0.1:8765/api/preview_move
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
- **Distances are clamped** server-side to 150 mm per command, and the clamp is
  reported back rather than applied silently.
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
