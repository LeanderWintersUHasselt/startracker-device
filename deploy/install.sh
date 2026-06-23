#!/usr/bin/env bash
set -euo pipefail

# StarTracker install script.
# Run from the repository root (the directory containing deploy/).
# A build must exist first: cmake -B build && cmake --build build -- -j4
# Requires sudo / root privileges.

BACKEND_BIN="build/startracker"
UI_BIN="build/ui/startracker-ui"
AUTOSTART_TRACK_BIN="deploy/startracker-autostart-track"
KIOSK_PROFILE_TEMPLATE="deploy/bash_profile.kiosk"
DEPLOY_DIR="deploy"

# ── Preflight checks ─────────────────────────────────────────────────────────

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: install.sh must be run as root (use sudo)." >&2
    exit 1
fi

if [[ ! -f "$BACKEND_BIN" ]]; then
    echo "ERROR: Backend binary not found at $BACKEND_BIN" >&2
    echo "       Build first:  cmake -B build && cmake --build build -- -j4" >&2
    exit 1
fi

if [[ ! -f "$UI_BIN" ]]; then
    echo "ERROR: UI binary not found at $UI_BIN" >&2
    echo "       Build first:  cmake -B build && cmake --build build -- -j4" >&2
    exit 1
fi

echo "StarTracker install starting..."

KIOSK_USER="${KIOSK_USER:-${SUDO_USER:-pi}}"

# ── Install binaries ─────────────────────────────────────────────────────────

echo "  [1/7] Installing binaries to /usr/local/bin/"
install -m 0755 "$BACKEND_BIN"         /usr/local/bin/startracker
install -m 0755 "$UI_BIN"              /usr/local/bin/startracker-ui
install -m 0755 "$AUTOSTART_TRACK_BIN" /usr/local/bin/startracker-autostart-track
echo "        /usr/local/bin/startracker"
echo "        /usr/local/bin/startracker-ui"
echo "        /usr/local/bin/startracker-autostart-track"

# ── Install Weston config ─────────────────────────────────────────────────────
# /etc/startracker/ holds only weston.ini — the backend reads nothing from here.

echo "  [2/7] Installing Weston config to /etc/startracker/"
mkdir -p /etc/startracker
install -m 0644 "$DEPLOY_DIR/weston.ini" /etc/startracker/weston.ini
echo "        /etc/startracker/weston.ini"

# ── Install backend config, intrinsics, and star maps ─────────────────────────
# The backend binary resolves all data paths relative to its own directory
# (bin_dir = /usr/local/bin), so everything lives next to the binary.

echo "  [3/7] Installing backend config and data to /usr/local/bin/"

KIOSK_USER="${KIOSK_USER:-${SUDO_USER:-pi}}"

install -m 0644 "$DEPLOY_DIR/config.json"     /usr/local/bin/config.json
echo "        /usr/local/bin/config.json"

install -m 0644 "$DEPLOY_DIR/intrinsics.json" /usr/local/bin/intrinsics.json
echo "        /usr/local/bin/intrinsics.json"

mkdir -p /usr/local/bin/star_maps
cp -rp --no-clobber "$DEPLOY_DIR/star_maps/." /usr/local/bin/star_maps/
chmod -R 0644 /usr/local/bin/star_maps
find /usr/local/bin/star_maps -type d -exec chmod 0755 {} +
echo "        /usr/local/bin/star_maps/"

chown "${KIOSK_USER}:"    /usr/local/bin/config.json /usr/local/bin/intrinsics.json
chown -R "${KIOSK_USER}:" /usr/local/bin/star_maps

# ── Install systemd unit ──────────────────────────────────────────────────────

echo "  [4/7] Installing backend systemd service unit"
install -m 0644 "$DEPLOY_DIR/startracker.service" /etc/systemd/system/startracker.service
echo "        /etc/systemd/system/startracker.service"

# ── Reload systemd ────────────────────────────────────────────────────────────

echo "  [5/7] Reloading systemd daemon"
systemctl daemon-reload

# ── Enable backend service ────────────────────────────────────────────────────

echo "  [6/7] Enabling backend service"
systemctl enable startracker.service

# ── Install kiosk launcher ────────────────────────────────────────────────────

echo "  [7/7] Installing kiosk launcher to ~/.bash_profile"
KIOSK_HOME="/home/${KIOSK_USER}"
install -m 0644 "$KIOSK_PROFILE_TEMPLATE" "${KIOSK_HOME}/.bash_profile"
chown "${KIOSK_USER}:" "${KIOSK_HOME}/.bash_profile"
echo "        ${KIOSK_HOME}/.bash_profile"

echo ""
echo "Installation complete."
echo ""
echo "Next steps:"
echo "  1. Select boot mode: sudo ./deploy/set_boot_mode.sh kiosk"
echo "  2. Reboot:           sudo reboot"
echo "  3. Verify:           systemctl status startracker.service"
