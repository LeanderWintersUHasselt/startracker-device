# StarTracker — User & Operations Guide

> **System overview**
> The StarTracker consists of a Raspberry Pi 5 mounted on a camera arm. An upward-facing camera tracks IR light sources on the ceiling to compute the camera's 6-DOF position and orientation. The result is streamed in real time to Unreal Engine via the FreeD D1 UDP protocol, where it drives a virtual camera through the Live Link plugin.

---

## Table of Contents

1. [Physical Setup](#1-physical-setup)
2. [Powering On & Off](#2-powering-on--off)
3. [SSH Access & Useful Commands](#3-ssh-access--useful-commands)
4. [Using the Touchscreen UI](#4-using-the-touchscreen-ui)
   - 4.1 [Home Screen](#41-home-screen)
   - 4.2 [Calibration Workflow](#42-calibration-workflow)
   - 4.3 [Live Tracking Screen](#43-live-tracking-screen)
   - 4.4 [Settings Screen](#44-settings-screen)
   - 4.5 [Diagnostics Screen](#45-diagnostics-screen)
5. [Connecting to Unreal Engine](#5-connecting-to-unreal-engine)
   - 5.1 [Direct UDP Cable Connection](#51-direct-udp-cable-connection)
   - 5.2 [Setting Static IP Addresses](#52-setting-static-ip-addresses)
   - 5.3 [Allowing UDP Through the Windows Firewall](#53-allowing-udp-through-the-windows-firewall)
   - 5.4 [Live Link Plugin — FreeD Configuration](#54-live-link-plugin--freed-configuration)
   - 5.5 [CineCameraActor — Assigning the Tracking Role](#55-cinecameraactor--assigning-the-tracking-role)
   - 5.6 [Switchboard — Starting the World via the Listener](#56-switchboard--starting-the-world-via-the-listener)
6. [Typical Session Checklist](#6-typical-session-checklist)
7. [Troubleshooting](#7-troubleshooting)

---

## 1. Physical Setup

### Components

| Component | Notes |
|-----------|-------|
| Raspberry Pi 5 | Mounted on the camera arm via the bottom bracket |
| Touchscreen | Connected to the Pi via USB (touch input) and a dedicated power cable |
| IR LED ring | Powered separately; points upward toward the ceiling markers |
| Camera (Pi Camera Module) | Mounted facing upward, secured to the Pi carrier |

### Mounting order

1. **Attach the Pi to the arm.** Slide the Pi into the arm bracket and secure the mounting screws. The camera lens must face upward and be free of obstructions.

2. **Connect the touchscreen USB cable** to one of the USB-A ports on the Pi. This carries touch input only.

3. **Connect the touchscreen power cable** to its dedicated power input on the screen. The touchscreen has its own power supply — do not rely on USB bus power alone.

4. **Connect the Pi power supply** (USB-C, 5 V / 5 A recommended) to the Pi. Use the official Raspberry Pi 5 power adapter to avoid under-voltage throttling.

5. **Connect the IR LED ring power supply.** The LED ring runs on its own PSU. Ensure the connector is fully seated before powering on.

6. **Connect the Ethernet cable** from the Pi's RJ-45 port directly to the Ethernet port of the Unreal Engine PC. No switch or router is needed (see [Section 5](#5-connecting-to-unreal-engine)).

---

## 2. Powering On & Off

### Power on

Power on in this order:
1. Touchscreen PSU
2. IR LED PSU
3. Pi USB-C power supply

The Pi boots automatically into the StarTracker kiosk UI. The display shows the live camera preview within about 20–30 seconds.

### Power off

**From the UI:** On the Home screen, press the **Afsluiten** (Shutdown) button and confirm the dialog. The system performs a clean shutdown — wait for the HDMI signal to drop before removing power.

**From SSH:**
```bash
sudo poweroff
```

> Do not remove power without shutting down first. The star maps and settings are stored on the SD card and may be corrupted by a hard power cut.

---

## 3. SSH Access & Useful Commands

SSH is available regardless of whether the kiosk UI is running.

| | |
|-|-|
| **User** | `startracker` |
| **Password** | `stars` |
| **Hostname** | `raspberrypi5.local` (mDNS) or the device IP |

```bash
ssh startracker@raspberrypi5.local
```

### Frequently used commands

```bash
# Check whether the backend daemon is running
systemctl status startracker.service --no-pager

# Tail live backend logs (Ctrl+C to stop)
journalctl -u startracker.service -f

# Last 50 lines of backend log
journalctl -u startracker.service -n 50 --no-pager

# Restart the backend daemon (stops and restarts tracking)
sudo systemctl restart startracker.service

# Tail the UI / Weston launcher log
tail -n 30 /tmp/startracker-ui.log
tail -n 30 /tmp/weston.log

# List saved star maps
ls -lh /usr/local/bin/star_maps/

# List scale calibration files
ls -lh /usr/local/bin/scale_calibration/

# Check the currently saved runtime settings (FreeD IP/port, ESKF parameters)
cat ~/.config/startracker/settings.json

# Manually reboot
sudo reboot

# Manually power off
sudo poweroff
```

### Sending raw commands to the daemon

The daemon accepts JSON commands on its Unix socket. Useful for scripting or debugging without the UI:

```bash
# Check daemon status
echo '{"cmd":"get_status"}' | nc -U /run/startracker/startracker.sock

# Get current configuration
echo '{"cmd":"get_config"}' | nc -U /run/startracker/startracker.sock

# List available star maps
echo '{"cmd":"list_files","type":"star_maps"}' | nc -U /run/startracker/startracker.sock
```

> **Important:** The Qt UI keeps a persistent connection to the daemon socket. While the UI is running, raw `nc` commands may time out. Either use the UI for normal operation, or stop the UI first (`pkill -9 startracker-ui`) before running socket commands directly.

---

## 4. Using the Touchscreen UI

The UI starts automatically on boot. It consists of five screens accessible via the navigation buttons on each screen.

### 4.1 Home Screen

The Home screen shows the **live camera preview** from the Pi camera. This is the starting point for all operations.

| Button | Action |
|--------|--------|
| **Kalibratie** | Go to the Calibration screen |
| **Tracking** | Go to the Tracking screen (only enabled after a valid star map is loaded) |
| **Instellingen** | Go to the Settings screen |
| **Herstart** | Reboot the Pi (with confirmation dialog) |
| **Afsluiten** | Power off the Pi (with confirmation dialog) |

The status bar at the bottom shows whether the backend is connected and whether tracking is currently active.

---

### 4.2 Calibration Workflow

Calibration consists of two steps that must be completed in order before tracking is possible.

#### Step 1 — Scale calibration

Scale calibration tells the system the physical height of the camera above the floor (in metres). This is needed to convert pixel distances on the ceiling into real-world metres.

**Option A — New measurement**

1. Tap **Nieuwe meting**.
2. The camera pauses and a counting pattern appears on screen. Place the calibration object at the indicated distance.
3. When prompted, tap **Bevestig** to confirm the measurement.
4. The measured height is saved to `scale_calibration/<height>m.json` and used immediately.

**Option B — Load a previous measurement**

*(The scale will be corrected later so selecting a previous measurement is okay)*

1. Tap **Laad bestaand** and select a `.json` file from the list.
2. The saved height is applied instantly — no camera interruption.

#### Step 2 — Star map

The star map records the positions of the IR LEDs (the "stars") on the ceiling in camera-space coordinates.

**Option A — Scan new map**

1. Tap **Nieuwe scan**.
2. The system scans the ceiling for several seconds. A live progress indicator shows how many stars have been detected and confirmed.
3. When the scan finishes, the map is saved to `star_maps/star_map_<height>m_<timestamp>.csv`.
4. After saving, the system immediately enters localisation mode.

**Option B — Load an existing map**

1. Tap **Laad bestaand** and select a `.csv` file from the list.
2. The map is loaded and the system is ready to track.

#### Anchor scaling (optional, for metric accuracy)

After building or loading a map you can anchor the scale to two known markers:

1. Tap **Ankerpunten** → the UI shows a camera overlay with numbered markers.
2. Note the IDs of two markers whose physical distance you know.
3. Enter **ID A**, **ID B**, and the physical distance between them in centimetres.
4. Tap **Bevestig**. The entire map is rescaled and saved.

---

### 4.3 Live Tracking Screen

The tracking screen is reached from the Home screen once a star map is loaded.

On entry, the system automatically starts tracking with both **camera vision** and **IMU** enabled.

| Toggle | Effect |
|--------|--------|
| **Tracking** (camera) | Enables/disables the vision-based PnP pose pipeline |
| **IMU** | Enables/disables the IMU ESKF propagation between camera frames |

Both can be toggled independently. With only IMU active (**IMU-only mode**), the system outputs inertial-propagated pose without relying on the camera. This is useful when the camera view is temporarily blocked.

**Tracking loss:** If the tracker loses sight of enough stars (below the minimum match threshold), a warning banner appears. The system continues outputting the last valid pose. Move back into a well-lit area with visible ceiling markers to reacquire.

Press **Stop** to end tracking and return to the Home screen.

---

### 4.4 Settings Screen

The Settings screen is split into tabs.

#### FreeD / Output tab

| Field | Description | Default |
|-------|-------------|---------|
| **FreeD IP** | IP address of the Unreal Engine PC | `192.168.1.100` |
| **FreeD Port** | UDP port that Live Link listens on | `40000` |
| **FreeD ingeschakeld** | Toggle FreeD UDP output on/off | On |
| **IMU ingeschakeld** | Toggle IMU fusion globally | On |
| **Camera offset X/Y/Z (m)** | Physical offset between the camera optical centre and the tracked reference point | `0.0` |

After changing the FreeD IP/port, tap **Opslaan**. Changes take effect immediately — no restart required.

#### Advanced config tabs (Camera, Sterdetectie, Live detectie, Localisatie, Tracking)

These tabs expose the full backend configuration (camera resolution, detector thresholds, RANSAC parameters, etc.). Changes are saved to the config file and applied on the next pipeline restart. Only change these values if you know what you are doing — the defaults are tuned for the installed lens and ceiling setup.

---

### 4.5 Diagnostics Screen

The Diagnostics screen (reachable from Settings) shows:
- **Detector debug view**: overlay of raw detections on the camera preview, switchable between *off*, *normal* (full detector), and *light* (fast detector used during tracking).
- **Grid overlay**: toggles a reference grid on the preview.
- **IMU debug tools**: options to disable IMU prediction or inject a known angular perturbation for testing.

---

## 5. Connecting to Unreal Engine

### 5.1 Direct UDP Cable Connection

Connect an Ethernet cable **directly** from the Pi's Ethernet port to the Ethernet port of the Unreal Engine PC. Do not route through a switch or router — this avoids any DHCP conflict and keeps UDP latency minimal.

> **FreeD over WiFi:** Sending FreeD UDP over WiFi also works and is useful on a shared network. However, routing to a PC on a different subnet (e.g. the Pi on `192.168.1.x` and the PC on `192.168.50.x`) requires either a static route on the PC or a router with inter-subnet forwarding enabled. On the same subnet WiFi works out of the box — just set the FreeD IP to the PC's WiFi address in the Settings screen.

---

### 5.2 Setting Static IP Addresses

Because there is no DHCP server on a direct cable connection, both sides need a manual static IP.

#### On the Raspberry Pi

Edit the network configuration:

```bash
sudo nano /etc/dhcpcd.conf
```

Add the following at the end of the file:

```
interface eth0
static ip_address=192.168.10.1/24
```

Save and reboot, or apply immediately:

```bash
sudo ip addr add 192.168.10.1/24 dev eth0
sudo ip link set eth0 up
```

#### On the Unreal Engine PC (Windows)

1. Open **Control Panel → Network and Sharing Center → Change adapter settings**.
2. Right-click the Ethernet adapter connected to the Pi → **Properties**.
3. Select **Internet Protocol Version 4 (TCP/IPv4)** → **Properties**.
4. Select **Use the following IP address** and enter:
   - IP address: `192.168.10.2`
   - Subnet mask: `255.255.255.0`
   - Default gateway: *(leave empty)*
5. Click **OK** on all dialogs.

Verify connectivity:

```bash
# From the Pi
ping 192.168.10.2

# From Windows Command Prompt
ping 192.168.10.1
```

---

### 5.3 Allowing UDP Through the Windows Firewall

Windows Firewall blocks incoming UDP by default. Create an inbound rule for the FreeD port:

1. Open **Windows Defender Firewall with Advanced Security** (search in Start).
2. Click **Inbound Rules** → **New Rule…** (in the right panel).
3. Select **Port** → **Next**.
4. Select **UDP**, enter the port number `40000` → **Next**.
5. Select **Allow the connection** → **Next**.
6. Apply to all profiles (Domain, Private, Public) → **Next**.
7. Give the rule a name, e.g. `StarTracker FreeD UDP` → **Finish**.

---

### 5.4 Live Link Plugin — FreeD Configuration

1. In the Unreal Editor, open **Edit → Plugins** and ensure **Live Link** is enabled. Restart the editor if you just enabled it.
2. Open the **Live Link** panel (**Window → Live Link**).
3. Click **+ Source** → **FreeD Source**.
4. Configure the source:
   - **Mode**: `FreeD D1 (Camera)`
   - **UDP Port**: `40000` *(must match the FreeD port set on the Pi)*
5. Click **OK**. The source appears in the Live Link panel with a green indicator once the Pi is streaming data.

> Set the FreeD IP in the StarTracker UI (**Settings → FreeD IP**) to `192.168.10.2` (the PC's static IP) before starting tracking.

---

### 5.5 CineCameraActor — Assigning the Tracking Role

1. In the Unreal Editor, select the **CineCameraActor** that should be driven by the tracker.
2. In the **Details** panel, scroll to **Live Link**.
3. Set **Subject Representation** to the FreeD source subject you created in Section 5.4.
4. Under **Controller Offset**, verify the **Transform Role** is set to **Camera** (not Default).
5. Enable **Use Role** if the checkbox is present.

The CineCameraActor now mirrors the physical camera's position and orientation in real time.

---

### 5.6 Switchboard — Starting the World via the Listener

> **Important:** The Unreal Engine editor must be **closed** before starting the world via Switchboard. If the editor is open it binds to the same ports and the UDP FreeD data will not reach Live Link correctly.

#### Prerequisites

- Switchboard is installed (part of the Unreal Engine installation under `Engine/Extras/Switchboard`).
- The Switchboard Listener is running on the Unreal PC.
- An existing Switchboard configuration file (`.sblconfig`) is available for the project.

#### Starting the Listener

On the Unreal Engine PC, launch the **Switchboard Listener** before opening Switchboard:

```
<UE install path>\Engine\Binaries\Win64\SwitchboardListener.exe
```

The Listener runs in the system tray and waits for connections from Switchboard.

#### Opening Switchboard and loading the config

1. Launch **Switchboard** from `Engine/Extras/Switchboard/switchboard.bat` (or via the shortcut).
2. Click **File → Open Configuration** and select the existing `.sblconfig` file for this project.
3. The devices panel shows the Unreal Engine PC device. Verify its IP matches the machine running the Listener.

#### Launching the project

1. Ensure the **Unreal Engine editor is fully closed** on the target PC.
2. In Switchboard, select the device and click **Launch** (the play button).
3. Switchboard connects to the Listener, which starts the packaged (or cooked) Unreal build using the parameters defined in the config file.
4. The project launches in the background; the Live Link FreeD source connects automatically once the StarTracker begins streaming.

> **If the world was open in the editor:** Close the editor, wait for all processes to end (`UE4Editor.exe` / `UnrealEditor.exe` should not appear in Task Manager), then launch via Switchboard.

---

## 6. Typical Session Checklist

```
Physical
[ ] Pi mounted on arm, camera facing up
[ ] Touchscreen USB + power connected
[ ] IR LED PSU connected and on
[ ] Ethernet cable Pi ↔ Unreal PC

Network
[ ] Pi static IP: 192.168.10.1
[ ] PC static IP: 192.168.10.2
[ ] Ping works both ways

StarTracker UI
[ ] Pi powered on, kiosk UI visible
[ ] Home screen shows camera preview
[ ] Go to Settings → set FreeD IP to 192.168.10.2, port 40000 → Save
[ ] Go to Calibration → load or run scale calibration
[ ] Load or scan star map
[ ] "Tracking" button enabled on Home screen

Unreal Engine
[ ] Editor is CLOSED
[ ] Switchboard Listener running on Unreal PC
[ ] Windows Firewall rule allows UDP port 40000
[ ] Open Switchboard → load config → Launch
[ ] Project starts — Live Link FreeD source shows green

Tracking
[ ] On StarTracker UI → Home → Tracking
[ ] Tracking screen shows live pose data
[ ] CineCameraActor moves in the Unreal viewport
```

---

## 7. Troubleshooting

### UI shows "Geen signaal"

The camera preview is not available. Check the backend daemon:

```bash
systemctl status startracker.service --no-pager
journalctl -u startracker.service -n 30 --no-pager
```

A common cause is the camera being held by another process. Restart the daemon:

```bash
sudo systemctl restart startracker.service
```

### Tracking is lost immediately after starting

- Verify the IR LEDs are powered on and visible in the camera preview (bright spots).
- Check that a star map with enough points (≥ 10 recommended) is loaded.
- Go to **Diagnose** screen and set the detector to **Normal** debug mode to see what the detector sees.

### Live Link source is red / no data in Unreal

1. Check that tracking is active on the StarTracker UI.
2. Verify the FreeD IP in Settings matches the PC's static IP (`192.168.10.2`) and the port matches the Live Link source port (`40000`).
3. Confirm the Unreal Editor is closed and only the packaged build or nDisplay render node is running.
4. Check the Windows Firewall rule (Section 5.3).
5. Test UDP from the Pi:
   ```bash
   echo '{"cmd":"get_status"}' | nc -U /run/startracker/startracker.sock
   # Look for "freed_enabled":true and "tracking_active":true
   ```

### Switchboard cannot connect to Listener

- Verify the SwitchboardListener process is running on the target PC (check Task Manager).
- Confirm the IP address of the device in Switchboard matches the PC's IP.
- Check that the Switchboard port (default `2980`) is not blocked by the firewall.

### Pi does not get a static IP after reboot

Check that `/etc/dhcpcd.conf` has the `static ip_address` line for `eth0` (Section 5.2). Then:

```bash
sudo systemctl restart dhcpcd
ip addr show eth0
```

### Backend crash loop (display keeps flashing)

Stop all services and run the daemon manually to see the error:

```bash
sudo systemctl stop startracker.service
sudo pkill -9 startracker-ui
/usr/local/bin/startracker   # errors will print to the terminal
```
