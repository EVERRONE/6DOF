# Starting The Control App

The web app can be started without touching a terminal. The launcher does the
whole sequence -- pull, install, serve, open browser -- in one step.

## One-Time Setup

1. Install [Node.js](https://nodejs.org/) LTS (18 or newer). This is the only
   prerequisite; everything else is handled by the launcher.
2. Install Chrome or Edge. Web Serial does not exist in Firefox or Safari, so
   the app cannot talk to the robot in those browsers.
3. Windows only, optional: double-click `create-desktop-shortcut.bat` once. That
   puts a **6DOF Robot Arm** icon on the desktop.

## Daily Use

| Platform | Double-click |
| --- | --- |
| Windows | `start-robot-arm.bat` (or the desktop shortcut) |
| macOS | `start-robot-arm.command` |
| Linux | `start-robot-arm.sh` |

From a terminal, `npm run launch` inside `robot-arm-control/` does the same thing.

Every start runs these four steps and prints what it did:

1. **Check for updates** -- fetches the current branch's upstream and
   fast-forwards. Nothing is pulled when the working tree has uncommitted
   changes, so local work is never overwritten.
2. **Check dependencies** -- reinstalls only when `package-lock.json` actually
   changed since the last successful install. Unchanged means this step is
   instant.
3. **Start the dev server** -- on port 3000, or the next free port when 3000 is
   taken. If the app is already running, the launcher just reopens the tab
   instead of starting a second server.
4. **Open the browser** -- Chrome, then Edge, then Chromium.

A console window stays open while the app runs. Closing it, or pressing
`Ctrl+C` in it, stops the app. Because this is the dev server, edits to the
source code reload in the browser automatically.

## Applying Updates From A Pull

Nothing extra to do: starting the launcher *is* the update. It pulls new commits
and installs new dependencies before serving, so a normal start always runs the
latest code.

Two things worth knowing:

- **Uncommitted local changes block the pull.** The launcher lists the files and
  keeps going with the local version rather than overwriting your work. Commit
  or stash them, then start again.
- **A diverged branch blocks the pull.** If the branch has local commits that
  are not on the remote, fast-forwarding is impossible. The launcher says so and
  starts anyway; resolve it with `git pull --rebase`.

To update without starting the app -- for instance to let a large dependency
install finish before going to the robot:

```bash
# from robot-arm-control/
npm run update
```

Or pass the flag to the launcher script: `start-robot-arm.bat --update-only`.

## Flags

| Flag | Effect |
| --- | --- |
| `--no-update` | Skip the fetch/fast-forward. Use when offline. |
| `--no-open` | Start the server but do not open a browser. |
| `--update-only` | Pull and install, then exit without serving. |
| `--port <n>` | Serve on a specific port instead of 3000. |

All of them work on every wrapper, e.g. `./start-robot-arm.sh --no-update`.

## Troubleshooting

**"Node.js was not found."**
Node is not installed, or not on `PATH`. Install the LTS build from
[nodejs.org](https://nodejs.org/) and start again. On macOS, a Node installed
through nvm is picked up automatically from `~/.nvm`.

**"No Chrome, Edge or Chromium found."**
The server is running fine; only the automatic browser launch failed. Open the
printed `http://localhost:3000` URL in Chrome or Edge yourself.

**Installing dependencies fails.**
Usually a network problem or a half-written `node_modules`. Delete
`robot-arm-control/node_modules` and start the launcher again -- it reinstalls
from scratch.

**"Ports 3000-3009 are all in use."**
Another program holds the whole range. Pass `--port 4000`, or close whatever is
using those ports.

**The browser opens but shows nothing.**
Compilation takes about half a minute on a cold start. Check the console window
for TypeScript errors; the launcher passes the dev server's output straight
through.

**The app cannot see the robot.**
The launcher only starts the app. Serial issues are separate -- verify the
Teensy is connected over USB, that no other program holds the port (Arduino IDE
Serial Monitor is the usual culprit), and see
[`SERIAL_PROTOCOL.md`](SERIAL_PROTOCOL.md).
