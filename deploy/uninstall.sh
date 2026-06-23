#!/usr/bin/env bash
set -euo pipefail

# StarTracker uninstall script.
# Disables and removes all installed files, then optionally restores
# the Pi boot target and unmasks lightdm.

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: uninstall.sh must be run as root (use sudo)." >&2
    exit 1
fi

echo "StarTracker uninstall starting..."

KIOSK_USER="${KIOSK_USER:-${SUDO_USER:-pi}}"
USER_SERVICE_FILE="/home/${KIOSK_USER}/.config/systemd/user/startracker-ui.service"

# ── Stop and disable services ────────────────────────────────────────────────

echo "  [1/5] Stopping and disabling services"

# Stop and disable user service (run as kiosk user)
if sudo -u "${KIOSK_USER}" systemctl --user is-active --quiet startracker-ui.service 2>/dev/null; then
    echo "        Stopping startracker-ui.service"
    sudo -u "${KIOSK_USER}" systemctl --user stop startracker-ui.service
fi
if sudo -u "${KIOSK_USER}" systemctl --user is-enabled --quiet startracker-ui.service 2>/dev/null; then
    echo "        Disabling startracker-ui.service"
    sudo -u "${KIOSK_USER}" systemctl --user disable startracker-ui.service
fi

# Stop and disable system backend service
if systemctl is-active --quiet startracker.service 2>/dev/null; then
    echo "        Stopping startracker.service"
    systemctl stop startracker.service
fi
if systemctl is-enabled --quiet startracker.service 2>/dev/null; then
    echo "        Disabling startracker.service"
    systemctl disable startracker.service
fi

# ── Remove systemd units ─────────────────────────────────────────────────────

echo "  [2/5] Removing systemd service units"

if [[ -f /etc/systemd/system/startracker.service ]]; then
    rm -f /etc/systemd/system/startracker.service
    echo "        Removed /etc/systemd/system/startracker.service"
fi

if [[ -f "${USER_SERVICE_FILE}" ]]; then
    rm -f "${USER_SERVICE_FILE}"
    echo "        Removed ${USER_SERVICE_FILE}"
fi

systemctl daemon-reload
sudo -u "${KIOSK_USER}" systemctl --user daemon-reload

# ── Remove config directory ──────────────────────────────────────────────────

echo "  [3/5] Removing /etc/startracker/"
if [[ -d /etc/startracker ]]; then
    rm -rf /etc/startracker
    echo "        Removed /etc/startracker/"
fi

# ── Remove binaries ──────────────────────────────────────────────────────────

echo "  [4/5] Removing binaries and data from /usr/local/bin/"

for bin in /usr/local/bin/startracker \
           /usr/local/bin/startracker-ui \
           /usr/local/bin/startracker-autostart-track \
           /usr/local/bin/config.json \
           /usr/local/bin/intrinsics.json; do
    if [[ -f "$bin" ]]; then
        rm -f "$bin"
        echo "        Removed $bin"
    fi
done

if [[ -d /usr/local/bin/star_maps ]]; then
    rm -rf /usr/local/bin/star_maps
    echo "        Removed /usr/local/bin/star_maps/"
fi

# ── Restore Pi boot target and lightdm ───────────────────────────────────────

echo "  [5/5] Boot configuration restore"

# Detect whether raspi-config is available (Pi-only)
if command -v raspi-config &>/dev/null; then
    current_target="$(systemctl get-default 2>/dev/null || true)"
    if [[ "$current_target" == "multi-user.target" ]]; then
        read -r -p "        Restore desktop autologin boot target? [y/N] " yn
        if [[ "${yn,,}" == "y" ]]; then
            raspi-config nonint do_boot_behaviour B4
            echo "        Boot target restored to graphical autologin (B4)."
        else
            echo "        Skipped. Current target remains: multi-user.target"
            echo "        To restore manually: sudo raspi-config nonint do_boot_behaviour B4"
        fi
    else
        echo "        Boot target is already '$current_target' — no change needed."
    fi
else
    echo "        raspi-config not found — skipping boot target restore."
fi

# Unmask lightdm if it was masked
if systemctl is-enabled lightdm.service 2>/dev/null | grep -q "masked"; then
    read -r -p "        Unmask lightdm? [y/N] " yn
    if [[ "${yn,,}" == "y" ]]; then
        systemctl unmask lightdm
        echo "        lightdm unmasked."
    else
        echo "        Skipped. To unmask manually: sudo systemctl unmask lightdm"
    fi
elif ! systemctl list-unit-files lightdm.service &>/dev/null 2>&1; then
    echo "        lightdm not installed — nothing to unmask."
else
    echo "        lightdm is not masked — no change needed."
fi

echo ""
echo "Uninstall complete."
