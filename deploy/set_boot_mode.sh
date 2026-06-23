#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  sudo ./deploy/set_boot_mode.sh kiosk
  sudo ./deploy/set_boot_mode.sh desktop
  ./deploy/set_boot_mode.sh status

Modes:
  kiosk    Boot to console autologin on tty1 and launch the StarTracker frontend.
  desktop  Boot to the normal Raspberry Pi OS desktop session.
  status   Show the current boot-related configuration.
EOF
}

if [[ $# -ne 1 ]]; then
    usage >&2
    exit 1
fi

MODE="$1"

if [[ "$MODE" == "status" ]]; then
    default_target="$(systemctl get-default 2>/dev/null || echo unknown)"

    if systemctl list-unit-files lightdm.service >/dev/null 2>&1; then
        lightdm_state="$(systemctl is-enabled lightdm.service 2>/dev/null || true)"
    else
        lightdm_state="not installed"
    fi

    echo "Default target : ${default_target}"
    echo "lightdm        : ${lightdm_state}"

    if [[ "${default_target}" == "multi-user.target" ]]; then
        echo "Boot mode      : kiosk"
    elif [[ "${default_target}" == "graphical.target" ]]; then
        echo "Boot mode      : desktop"
    else
        echo "Boot mode      : custom/unknown"
    fi

    exit 0
fi

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: $0 must be run as root for mode '${MODE}'." >&2
    exit 1
fi

if ! command -v raspi-config >/dev/null 2>&1; then
    echo "ERROR: raspi-config not found. This script is intended for Raspberry Pi OS." >&2
    exit 1
fi

case "$MODE" in
    kiosk)
        echo "Switching boot mode to kiosk..."
        raspi-config nonint do_boot_behaviour B2

        if systemctl list-unit-files lightdm.service >/dev/null 2>&1; then
            systemctl mask lightdm >/dev/null 2>&1 || true
        fi

        systemctl enable startracker.service >/dev/null 2>&1 || true

        echo "Boot mode set to kiosk."
        echo "Reboot required: sudo reboot"
        ;;

    desktop)
        echo "Switching boot mode to Raspberry Pi OS desktop..."
        raspi-config nonint do_boot_behaviour B4

        if systemctl list-unit-files lightdm.service >/dev/null 2>&1; then
            systemctl unmask lightdm >/dev/null 2>&1 || true
            systemctl enable lightdm >/dev/null 2>&1 || true
        fi

        echo "Boot mode set to desktop."
        echo "Reboot required: sudo reboot"
        ;;

    *)
        usage >&2
        exit 1
        ;;
esac
