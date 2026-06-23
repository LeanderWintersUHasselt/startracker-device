# StarTracker — Deployment Guide

This directory contains all files needed to deploy StarTracker on a Raspberry Pi 5 in kiosk mode.

---

## 1. Prerequisites

- Raspberry Pi 5 running Raspberry Pi OS Bookworm (64-bit recommended)
- Plans C and D fully built (both binaries present in `build/`)
- Weston installed with kiosk-shell support:

```bash
sudo apt-get install -y weston
ls /usr/lib/*/weston/kiosk-shell.so 2>/dev/null || ls /usr/lib/weston/kiosk-shell.so
```

---

## 2. Build the binaries

From the repository root on the Pi:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j4
```

Verify both outputs exist:

```bash
ls -lh build/startracker build/ui/startracker-ui
```

---

## 3. Install

From the repository root:

```bash
sudo ./deploy/install.sh
```

The script will:
- Copy `build/startracker` → `/usr/local/bin/startracker`
- Copy `build/ui/startracker-ui` → `/usr/local/bin/startracker-ui`
- Create `/etc/startracker/` and copy `deploy/weston.ini` there
- Copy `startracker.service` to `/etc/systemd/system/`
- Install a tty1 kiosk launcher in the kiosk user's `~/.bash_profile`
- Enable the backend service

---

## 4. Choose boot mode

After installing, pick the mode you want on the next reboot.

### Boot into the StarTracker frontend

From the repository root:

```bash
sudo ./deploy/set_boot_mode.sh kiosk
```

This configures Raspberry Pi OS for console autologin on `tty1`. On login, the managed `~/.bash_profile` starts Weston and `startracker-ui`.

### Boot into normal Raspberry Pi OS desktop

From the repository root:

```bash
sudo ./deploy/set_boot_mode.sh desktop
```

This restores graphical desktop autologin (`B4`) so the Pi boots into the regular Raspberry Pi OS session instead of the kiosk frontend.

### Check current boot mode

```bash
./deploy/set_boot_mode.sh status
```

SSH access is unaffected — `sshd.service` is independent of the kiosk display stack.

---

## 5. Reboot and verify

```bash
sudo reboot
```

After reboot, the Pi should boot to a black screen briefly, then the StarTracker kiosk UI appears fullscreen.

**Verify via SSH after reboot:**

```bash
# Backend should be active (running)
systemctl status startracker.service --no-pager

# Check backend logs
journalctl -u startracker.service -n 30 --no-pager

# Check UI / Weston launcher logs
tail -n 30 /tmp/weston.log
tail -n 30 /tmp/startracker-ui.log
```

---

## 6. Uninstall

```bash
sudo ./deploy/uninstall.sh
```

This disables and stops both services, removes all installed files, and removes `/etc/startracker/`. It does NOT restore raspi-config boot settings — see the script's final output for manual restore commands.

---

## 7. Installed file locations

| File | Installed path |
|------|----------------|
| `build/startracker` | `/usr/local/bin/startracker` |
| `build/ui/startracker-ui` | `/usr/local/bin/startracker-ui` |
| `deploy/weston.ini` | `/etc/startracker/weston.ini` |
| `deploy/startracker.service` | `/etc/systemd/system/startracker.service` |
| `~/.bash_profile` | tty1 autostart launcher for Weston + `startracker-ui` |

Runtime files (created by systemd / backend at startup):

| Path | Created by |
|------|------------|
| `/run/startracker/` | systemd (`RuntimeDirectory=startracker`) |
| `/run/startracker/startracker.sock` | C++ backend `main()` |
| `/dev/shm/startracker` | C++ backend (shared memory) |

---

## 8. Troubleshooting

### Weston fails to start

```bash
tail -n 50 /tmp/weston.log
tail -n 50 /tmp/startracker-ui.log
```

**`kiosk-shell.so: cannot open shared object file`** — Weston kiosk-shell plugin missing:
```bash
sudo apt-get install -y weston
find /usr/lib -name "kiosk-shell.so" 2>/dev/null
```

**`failed to create display socket` or `XDG_RUNTIME_DIR not set`**:
```bash
echo '$XDG_RUNTIME_DIR'
loginctl session-status
```

**`no usable drm/kms found`** — pi user not in video/render group:
```bash
sudo usermod -aG video,render pi
sudo reboot
```

**Wrong resolution (e.g. 1280×720 instead of 1920×1080)** — edit `weston.ini` mode line, then:
```bash
sudo install -m 0644 deploy/weston.ini /etc/startracker/weston.ini
sudo pkill -f '/usr/bin/weston --config=/etc/startracker/weston.ini'
```

### Qt6 Wayland platform plugin missing

```
qt.qpa.plugin: Could not load the Qt platform plugin "wayland"
```

```bash
sudo apt-get install -y qt6-wayland
```

### Backend socket not appearing

```bash
# Check RuntimeDirectory was created
ls /run/startracker/ 2>/dev/null || echo "directory missing"

# Check backend logs for crash before socket creation
journalctl -u startracker.service -n 50 --no-pager
```

The `SocketClient` 1 s retry loop tolerates a startup delay of several seconds.

### Shared memory permission error

Both services default to running as root. If you add `User=` directives, both must use the same user. Emergency fix:
```bash
sudo chmod g+rw /dev/shm/startracker
```

### Crash restart loop (display flickers)

Stop the loop to debug:
```bash
sudo systemctl stop startracker.service
sudo pkill -f '/usr/local/bin/startracker-ui'
sudo pkill -f '/usr/bin/weston --config=/etc/startracker/weston.ini'
# Then run binaries manually to see error output
```
