#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VENV_ACTIVATE="${REPO_ROOT}/.venv/bin/activate"

if [[ ! -f "${VENV_ACTIVATE}" ]]; then
  echo "Missing virtualenv activation script: ${VENV_ACTIVATE}"
  echo "Run setup first (example: tools/op.sh setup)."
  exit 1
fi

OPENPILOT_CMD="cd \"${REPO_ROOT}\"; source \"${VENV_ACTIVATE}\"; ./tools/sim/launch_openpilot.sh"
BRIDGE_READY_CMD="cd \"${REPO_ROOT}/tools/sim\"; source \"${VENV_ACTIVATE}\"; clear; echo \"Ready. Run: ./run_bridge.py\"; exec bash"

open_ubuntu_2404_terminal() {
  local cmd="$1"
  gnome-terminal -- bash -ic "${cmd}"
}

open_linux_terminal() {
  local cmd="$1"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -ic "${cmd}"
  elif command -v konsole >/dev/null 2>&1; then
    konsole -e bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --command="bash -ic '${cmd}'" >/dev/null 2>&1 &
  elif command -v mate-terminal >/dev/null 2>&1; then
    mate-terminal -- bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -e bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v alacritty >/dev/null 2>&1; then
    alacritty -e bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v kitty >/dev/null 2>&1; then
    kitty bash -ic "${cmd}" >/dev/null 2>&1 &
  elif command -v tilix >/dev/null 2>&1; then
    tilix --new-process bash -ic "${cmd}" >/dev/null 2>&1 &
  else
    echo "No supported Linux terminal found."
    echo "Install one of: gnome-terminal, konsole, xfce4-terminal, mate-terminal, xterm, alacritty, kitty, tilix."
    exit 1
  fi
}

OS="$(uname -s)"
if [[ "${OS}" == "Darwin" ]]; then
  if ! command -v osascript >/dev/null 2>&1; then
    echo "osascript not found. Install/enable AppleScript support."
    exit 1
  fi

  osascript <<EOF
tell application "Terminal"
  activate
  do script "${OPENPILOT_CMD}"
  do script "${BRIDGE_READY_CMD}"
end tell
EOF
elif [[ "${OS}" == "Linux" ]]; then
  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "No graphical session detected (DISPLAY/WAYLAND_DISPLAY missing)."
    echo "Run this script from a desktop session on Ubuntu 24.04."
    exit 1
  fi

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
  fi

  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
    if command -v gnome-terminal >/dev/null 2>&1; then
      open_ubuntu_2404_terminal "${OPENPILOT_CMD}"
      open_ubuntu_2404_terminal "${BRIDGE_READY_CMD}"
      exit 0
    fi
    echo "Ubuntu 24.04 detected, but gnome-terminal is not installed."
    echo "Install it with: sudo apt install gnome-terminal"
  fi

  open_linux_terminal "${OPENPILOT_CMD}"
  open_linux_terminal "${BRIDGE_READY_CMD}"
else
  echo "Unsupported OS: ${OS}"
  exit 1
fi
