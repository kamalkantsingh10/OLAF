# Story 8.1: Chest display foundation — boot-to-app, portrait 2×2 grid, view-manager

Status: ready-for-dev

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want the chest panel to boot straight into a fullscreen portrait dashboard instead of a Linux login prompt,
so that OLAF always presents a face on its chest, with room to host a full-screen app later.

> **Foundation story for Epic 8 (re-scoped 2026-06-03, SCP approved).** This stands up the standalone chest-display **app** (pygame/KMSDRM) and its layout/view-manager skeleton. The heart widget (8.2) and log panels (8.3) drop into the cells this story creates. No engine/ROS wiring yet — the app runs standalone; the engine↔chest link is Story 8.4.

## Acceptance Criteria

1. **Given** the Pi has finished booting, **When** the chest panel powers up, **Then** it shows the chest app — not the `getty@tty1` `login:` prompt — because `getty@tty1` is disabled/overridden and a `systemd` service launches the app on the panel.
2. **Given** the app starts, **When** it initializes the display, **Then** it opens a single fullscreen **KMSDRM** surface on `card0`/`DSI-2` at the rotated portrait resolution (**480×800**), with no X/Wayland/desktop session, acquired as user `kamal` **without sudo** (`video`/`render` group membership).
3. **Given** the running app, **When** it draws a frame, **Then** it renders a **2×2 grid of four 240×400 cells** with visible separators — top-left reserved for the heart, the other three reserved for log panels — placeholder cell content is acceptable in this story.
4. **Given** the app architecture, **When** a frame is composed, **Then** a **view-manager** selects the active view each frame; `DASHBOARD` is fully implemented, and a `FULLSCREEN_TAKEOVER` view (480×800, single app) can be **registered and switched to and back** without restructuring — proven this story by a stub takeover view.
5. **Given** the service, **When** the app process crashes or exits non-zero, **Then** `systemd` restarts it (`Restart=always`) and structured JSON logs are written to journald.
6. **Given** the other OLAF services (expression engine, drivers), **When** the chest app runs, **Then** it runs as a separate process that owns only the DSI panel and does not interfere with the engine, the I2C/servo buses, or the head ESP32.

## Tasks / Subtasks

- [ ] Task 1: Scaffold the chest-display package + dependency (AC: #2, #6)
  - [ ] Create a new package `ros2/src/chest_display/` (ament_python, matching the repo's other `ros2/src/*` packages); entry point `chest_display` runs the app `main()` [Saved Question #1 — package location/poetry env]
  - [ ] Add `pygame-ce` as a dependency and pin its version; install into the project's poetry env (`poetry add pygame-ce`) [Saved Question #1]
  - [ ] App is launchable standalone via the documented `poetry run python` pattern (no ROS dependency yet)
- [ ] Task 2: KMSDRM fullscreen surface (AC: #2)
  - [ ] Initialize pygame with `SDL_VIDEODRIVER=kmsdrm`; open a fullscreen surface on the DSI panel (`card0`/`DSI-2`)
  - [ ] Confirm the surface is **480×800 portrait** — resolve whether KMSDRM presents the kernel-rotated mode (480×800) directly or the native 800×480 needing an app-side rotate [Saved Question #2]
  - [ ] Verify it runs as `kamal` without sudo (video/render groups); `connect`/init is fatal with a clear journald error if the panel is unavailable
- [ ] Task 3: 2×2 grid layout (AC: #3)
  - [ ] Define a layout module: four 240×400 cells (TL/TR/BL/BR) with separators; TL = heart slot, TR/BL/BR = log slots
  - [ ] Render placeholder content in each cell (labels/borders) so the grid is visually verifiable before 8.2/8.3
- [ ] Task 4: View-manager (AC: #4)
  - [ ] `View` abstraction with `update(dt)` + `draw(surface)`; a `ViewManager` holds the active view and renders it each frame
  - [ ] Implement `DashboardView` (hosts the 2×2 grid); register a stub `FullscreenTakeoverView` (single 480×800 surface)
  - [ ] Provide a `switch_to(view)` / `return_to_dashboard()` path and prove a round-trip (dashboard → takeover → dashboard) without restructuring
- [ ] Task 5: Boot-to-app via systemd (AC: #1, #5)
  - [ ] Disable/override `getty@tty1` so the console login no longer owns the DSI panel [Saved Question #3 — exact DRM-master approach]
  - [ ] `systemd` unit launches the app on the panel at boot, `Restart=always`, structured JSON logs to journald
  - [ ] Document the install steps (and the reversible spike path: `sudo systemctl stop getty@tty1` → run from SSH) in the package README
- [ ] Task 6: Fixed-rate render loop + non-interference (AC: #3, #5, #6)
  - [ ] Render loop at a target frame rate (≥30 fps headroom for 8.2); clean shutdown on SIGTERM (so systemd restart is graceful)
  - [ ] Confirm the app touches only the DSI panel — no I2C/servo/ESP32 access (separate process from the engine)
- [ ] Task 7: Tests / verification (AC: #1–#4)
  - [ ] Headless-friendly unit tests for layout math (cell rects) and the view-manager switch/return logic (no display required)
  - [ ] On the Pi: cold-boot shows the app (no `login:`); grid renders portrait; takeover round-trip works; kill the process → systemd restarts it

## Dev Notes

### Verified hardware reality (on the robot, 2026-06-03)

Do **not** re-derive these — they were confirmed live over SSH:
- **Pi 5, Ubuntu 24.04 (aarch64)** — NOT Raspberry Pi OS.
- Panel = **LUCKFOX 4.3" DSI, 800×480, capacitive touch**, on DRM **`card0` / connector `DSI-2`** (connected). HDMI is a separate `card2` (disconnected).
- **Already kernel-rotated to portrait**: `/boot/firmware/cmdline.txt` has `video=DSI-2:800x480,rotate=90` → the app should target **480×800**. (`config.txt` also has legacy `display_rotate=3`, likely ignored under KMS.)
- **No display manager** (`display-manager` inactive/not-found). The "please login" on the panel today is just **`getty@tty1`**. There is no desktop/greeter to fight.
- User `kamal` is in the **`video` + `render`** groups → the app opens the `card0` DRM surface **without sudo** (`/dev/dri/card0` is `root:video`).
- `pygame` is **not yet installed**. Plan: `pygame-ce` (maintained fork, better KMSDRM support) on SDL2 KMSDRM.
[Source: memory project_epic8_chest_display; live SSH inspection 2026-06-03]

### This is a standalone app, not an engine adapter

Epic 8 (re-scoped) realizes the chest panel as a **separate process** with its own render loop — it is the *renderer* in the delegating-surface model (the engine's `heart_adapter` will forward state to it in 8.4, mirroring eye/ESP32). For 8.1–8.3 there is **no ROS / no engine link** — the app runs standalone with placeholder/resting content. Keep it decoupled so a heavy 30 fps loop never touches the engine's servo-tick thread. [Source: epics.md#Epic-8 Architecture Note; phase2-expression-engine.md#§2-Amendment; sprint-change-proposal-2026-06-03.md]

### Boot-to-app

The only thing showing "please login" is `getty@tty1` on the DSI console — there is no greeter. To own the panel: disable/override `getty@tty1` and launch the app via a `systemd` service. KMSDRM needs to become **DRM master**, which requires the VT/console not to hold it — the exact mechanism (run the service on tty1 with getty masked, vs a `seatd`/logind approach) needs to be settled on the hardware [Saved Question #3]. For a quick spike from SSH: `sudo systemctl stop getty@tty1`, then run the app with `SDL_VIDEODRIVER=kmsdrm` (reversible — restart getty to restore the login). Feeds the permanent unit in Story 8.8.

### View-manager (future-proofs fullscreen takeover)

Kamal's explicit forward requirement: a single app can later occupy the **full screen** depending on context, then return to the dashboard when done. Build the active-view indirection from day one so takeover is "register a view + switch," not a rewrite. The heart/log widgets live **inside** `DashboardView`; they keep running (the heart keeps beating) regardless of which view is active. [Source: epics.md#Story-8.1]

### Layout math

480×800 portrait → 2×2 grid of **240×400** cells. TL = heart (8.2), TR/BL/BR = `LogPanel`s (8.3). Keep cell rects in one layout module so 8.2/8.3 import them rather than hardcoding.

### Anti-patterns

- Do **not** install or start a desktop/X/Wayland session — KMSDRM draws directly to the framebuffer.
- Do **not** hardcode landscape (800×480) — the panel is rotated; target 480×800 and confirm the surface orientation [Saved Question #2].
- Do **not** require `sudo` at runtime — rely on `video`/`render` group membership.
- Do **not** add ROS subscriptions or talk to the engine/I2C here — that is Story 8.4.
- Do **not** block the render loop on slow I/O; keep SIGTERM shutdown clean for systemd restarts.

### Project Structure Notes

```
ros2/src/chest_display/                     # NEW package (ament_python)
  chest_display/
    app.py            # main(): init KMSDRM surface + run loop
    layout.py         # 2×2 grid cell rects (240×400)
    view_manager.py   # View base, ViewManager, switch/return
    views/
      dashboard.py    # DashboardView (hosts the 2×2 grid; heart+logs slot in 8.2/8.3)
      takeover.py     # FullscreenTakeoverView (stub for this story)
    test/             # layout + view-manager unit tests (headless)
  systemd/chest-display.service             # boot-to-app unit
  README.md                                 # install + reversible spike notes
```
Matches the repo's `ros2/src/*` ament_python convention. [Saved Question #1 if a non-ROS location is preferred.]

### References

- [Source: docs/planning-artifacts/epics.md#Epic-8 — Story 8.1 + Architecture Note]
- [Source: docs/planning-artifacts/prd/phase2-prd.md#Epic-8 — Story 8.1]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#§2-Amendment]
- [Source: docs/planning-artifacts/sprint-change-proposal-2026-06-03.md]
- [Source: memory project_epic8_chest_display — verified Pi hardware facts]

### Saved Questions

1. **Package location / poetry env** — new ROS2 package `ros2/src/chest_display/` (recommended, matches siblings) vs a non-ROS app folder; and which `pyproject.toml`/poetry env owns `pygame-ce` (its own vs the `expression_engine` env)?
2. **Rotation handling** — does the KMSDRM surface present the kernel-rotated **480×800** directly, or the native **800×480** that the app must rotate? (Resolve on hardware; affects all layout math.)
3. **DRM-master / boot mechanism** — run the systemd service on tty1 with `getty@tty1` masked, vs a logind/seatd approach? Settle the cleanest reliable path on the Pi 5.

## Dev Agent Record

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
