#!/usr/bin/env bash
# Double-clickable macOS wrapper around start-robot-arm.sh.
# Finder only opens .command files, so this is the entry point on macOS.
# Keeps the Terminal window readable when the launcher stops with an error.

set -uo pipefail

cd "$(dirname "$0")"

./start-robot-arm.sh "$@"
status=$?

if [ "$status" -ne 0 ]; then
  echo
  echo "  The launcher stopped with error code $status."
  read -n 1 -s -r -p "  Press any key to close this window..."
  echo
fi

exit "$status"
