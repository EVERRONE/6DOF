#!/usr/bin/env bash
# ---------------------------------------------------------------------------
#  6DOF robot arm -- start the control app (macOS / Linux)
#
#  Run ./start-robot-arm.sh, or double-click start-robot-arm.command on macOS.
#  It pulls the latest commits, installs any changed dependencies, starts the
#  web app and opens it in Chrome or Edge.
#
#  Options:
#    --no-update     start without pulling from GitHub
#    --update-only   only pull + install, do not start the app
# ---------------------------------------------------------------------------

set -euo pipefail

cd "$(dirname "$0")"

# A Finder-launched shell gets a minimal PATH, so add the usual Node locations.
export PATH="$PATH:/usr/local/bin:/opt/homebrew/bin"

if ! command -v node >/dev/null 2>&1 && [ -s "$HOME/.nvm/nvm.sh" ]; then
  # shellcheck disable=SC1091
  . "$HOME/.nvm/nvm.sh" >/dev/null 2>&1 || true
fi

if ! command -v node >/dev/null 2>&1; then
  echo
  echo "  Node.js was not found."
  echo
  echo "  Install the LTS version from https://nodejs.org/ (or via brew/apt),"
  echo "  then start this script again."
  echo
  exit 1
fi

exec node robot-arm-control/scripts/launch.mjs "$@"
