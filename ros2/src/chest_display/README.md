# chest_display

OLAF's chest-display app (Epic 8) — a standalone **pygame / KMSDRM** portrait
dashboard on the **4.3" DSI** chest panel.

Default view (`DASHBOARD`) on a 480×800 portrait surface — **4 blocks**: a
full-width heart band (top 30%), a full-width middle band, then two quadrants:

```
┌───────────────────────────────┐
│          ❤ HEART  (8.2)        │  full-width band, top 30%  (480×240)
├───────────────────────────────┤
│          log_mid  (8.3)        │  full-width middle band    (480×280)
├───────────────┬───────────────┤
│   log_bl      │   log_br      │  two bottom quadrants (8.3)
│   (8.3)       │   (8.3)       │  each 240×280
└───────────────┴───────────────┘
```

A **view-manager** lets a single app take the full screen (`FULLSCREEN_TAKEOVER`)
and return to the dashboard when done (Story 8.5 fills this in).

## Architecture role

This app is the **renderer** in the heart's *delegating-surface* model: the
expression engine composes the heart state from `expression_map.yaml` and
forwards it here over local IPC (Story 8.4), exactly as the Head ESP32 owns eye
animation. Story 8.1 (this) is **ROS-free** — the app runs standalone with
placeholder cells; the heart widget (8.2) and log panels (8.3) drop into the
slots. See `docs/planning-artifacts/sprint-change-proposal-2026-06-03.md`.

## Run it

The panel is on DRM `card0`/`DSI-2`. User `kamal` is in the `video`+`render`
groups, so no sudo is needed to open the surface itself (only to free it from
the console login).

> **pygame must be the distro build, not pip/poetry.** The PyPI pygame wheels
> bundle their own SDL2 **without** the `kmsdrm` backend (`"kmsdrm not
> available"`). Use the apt package, which links the system SDL2 (2.30, has
> kmsdrm), and run under **system `python3`**:
> ```bash
> sudo apt install -y python3-pygame
> ```

```bash
# Reversible spike from SSH (frees the panel from the console login, runs the app):
sudo systemctl stop getty@tty1
SDL_VIDEODRIVER=kmsdrm python3 -m chest_display.app      # from this dir; system python3
sudo systemctl start getty@tty1   # restore the login prompt afterwards
```

**Confirmed on hardware (2026-06-03):** KMSDRM hands the app the **native
800×480 landscape** surface (the kernel `rotate=90` only rotates the console),
so the app rotates its 480×800 portrait canvas by `CHEST_ROTATE_DEG` (default
90; flip to 270 if upside down). The T/D/ESC demo keys need a keyboard on the
Pi — there isn't one — so use Ctrl-C to quit.

## Boot-to-app (permanent)

See `systemd/chest-display.service` (install steps in its header). In short:
disable `getty@tty1`, install + enable the unit; the app then owns the panel
from boot with `Restart=always` and JSON logs to journald.

## Tests

Pure-logic tests (layout geometry + view-manager) run headlessly — no panel, no
pygame display:

```bash
python3 -m pytest test/ -q
```

The KMSDRM surface, 480×800 portrait, rotation, ≥30 fps, and boot-to-app are
verified on the Pi hardware.
