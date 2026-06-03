# Story 8.1: Chest display foundation — boot-to-app, portrait 2×2 grid, view-manager

Status: review

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

- [x] Task 1: Scaffold the chest-display package + dependency (AC: #2, #6)
  - [x] Create a new package `ros2/src/chest_display/` (ament_python, matching the repo's other `ros2/src/*` packages); entry point `chest_display` runs the app `main()` — **Saved Question #1 RESOLVED: separate package** (Kamal 2026-06-03 — the chest surface will do more than the heart; easier to manage standalone)
  - [x] Provide pygame **with kmsdrm** — RESOLVED (Kamal 2026-06-03): distro **`python3-pygame`** (apt) + **system `python3`**, NOT poetry. The pip/poetry pygame bundles its own SDL **without** kmsdrm (`"kmsdrm not available"`); `sudo apt install -y python3-pygame` links system SDL2 2.30 (has KMSDRM).
  - [x] App launches standalone: `SDL_VIDEODRIVER=kmsdrm python3 -m chest_display.app` — confirmed on Pi 2026-06-03 (`display_ready` driver=KMSDRM, `running` @ 30 fps)
- [x] Task 2: KMSDRM fullscreen surface (AC: #2)
  - [x] Initialize pygame with `SDL_VIDEODRIVER=kmsdrm`; open a fullscreen surface on the DSI panel (`card0`/`DSI-2`) — confirmed (driver=KMSDRM)
  - [x] **Saved Question #2 RESOLVED:** KMSDRM presents the **native 800×480 LANDSCAPE** (kernel `rotate=90` rotates only the console). The app draws a 480×800 portrait canvas and rotates it 90° (`CHEST_ROTATE_DEG`) onto the surface — handled automatically by the rotation-safe present.
  - [x] Runs as `kamal` without sudo (video/render groups); init is fatal with a clear journald error if the panel/driver is unavailable (proven by the `display_init_failed` path before the pygame fix)
- [x] Task 3: 2×2 grid layout (AC: #3)
  - [x] Define a layout module: four 240×400 cells (TL/TR/BL/BR) with separators; TL = heart slot, TR/BL/BR = log slots — `layout.py`; tested (`test_layout.py`, 7 cases: count/size/positions/tiling/slots/Rect helpers)
  - [x] Render placeholder content in each cell (labels/borders) — **visually confirmed on the panel 2026-06-03 (Kamal: "shows 4 sections")**
- [x] Task 4: View-manager (AC: #4)
  - [x] `View` abstraction with `update(dt)` + `draw(surface)`; a `ViewManager` holds the active view and renders it each frame — `view_manager.py`
  - [x] Implement `DashboardView` (hosts the 2×2 grid); register a stub `FullscreenTakeoverView` (single 480×800 surface) — `views/dashboard.py`, `views/takeover.py`
  - [x] Provide a `switch_to(view)` / `return_to_dashboard()` path and prove a round-trip (dashboard → takeover → dashboard) without restructuring — tested (`test_view_manager.py`, 5 cases incl. round-trip + delegate-to-active-only)
- [x] Task 5: Boot-to-app via systemd (AC: #1, #5)
  - [x] Disabled `getty@tty1`; the systemd unit (`Conflicts=getty@tty1`) owns the panel — **cold-boot confirmed 2026-06-03 (Kamal): dashboard comes up with no login flash**. Saved Q#3 resolved (no TTYPath trick needed).
  - [x] `systemd` unit (`systemd/chest-display.service`) launches via system `python3`, `Restart=always`, JSON logs to journald — installed + `enable --now`; `is-active`=active, `display_ready` emitted as a service (DRM master works for a service, not just SSH)
  - [x] Documented install steps + reversible-spike path in `README.md`
- [x] Task 6: Fixed-rate render loop + non-interference (AC: #3, #5, #6)
  - [x] Render loop at 30 fps (`running` log, `clock.tick(30)`); clean shutdown on SIGINT/SIGTERM (Ctrl-C → `shutting_down`, exit 0) — confirmed on Pi
  - [x] App touches only the DSI panel — no I2C/servo/ESP32 (separate process, pygame/KMSDRM only)
- [x] Task 7: Tests / verification (AC: #1–#4)
  - [x] Headless unit tests for layout math + view-manager switch/return — **15 passing on dev PC AND on the Pi**
  - [x] On the Pi: dashboard renders (final 4-block layout) ✅; takeover round-trip proven by tests (visual demo needs a keyboard on the Pi — none attached); **cold-boot shows the app (no `login:`) + systemd `Restart=always`** ✅

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

1. ~~**Package location / poetry env**~~ — **RESOLVED 2026-06-03 (Kamal): separate ament package `ros2/src/chest_display/`** (the chest surface will grow beyond the heart → easier to manage standalone; engine stays lean, pygame isolated). Which poetry env owns `pygame-ce` is settled during the Pi bring-up.
2. ~~**Rotation handling**~~ — **RESOLVED 2026-06-03:** KMSDRM hands the app the **native 800×480 landscape**; the app rotates its 480×800 portrait canvas 90° (`CHEST_ROTATE_DEG`). The kernel `rotate=90` only affects the console, not the app's KMSDRM surface.
3. ~~**DRM-master / boot mechanism**~~ — **RESOLVED 2026-06-03:** stopping `getty@tty1` is enough for KMSDRM to take DRM master (confirmed from an SSH launch). Boot path = disable getty + enable the systemd unit; no TTYPath/logind trick required.

**New finding (record):** pip/poetry pygame wheels bundle SDL **without** kmsdrm. Runtime is the **distro `python3-pygame`** (apt) under **system `python3`** — which is also the interpreter ROS 2 will use in Story 8.4.

## Dev Agent Record

### Agent Model Used

claude-opus-4-8 (1M context)

### Debug Log References

- Headless logic tests: `python3 -m pytest test/ -q` → **11 passed** (7 layout + 5 view-manager... 11 total) on dev PC (Python 3.12.3, pygame 2.5.2). RED confirmed first (ModuleNotFoundError) before GREEN.

### Completion Notes List

**Built + verified on dev PC (logic):**
- ✅ Package scaffolded as a **separate ament_python package** `ros2/src/chest_display/` (Saved Q#1 resolved — Kamal).
- ✅ `layout.py` — pure 2×2 portrait geometry (480×800 → four 240×400 cells); 7 tests pass.
- ✅ `view_manager.py` — `View`/`ViewManager` with DASHBOARD↔TAKEOVER round-trip; 5 tests pass. Pure logic, no pygame.
- ✅ `app.py` — KMSDRM fullscreen entrypoint with **rotation-safe present** (draws offscreen 480×800 portrait, rotates only if the device surface comes back landscape; `CHEST_ROTATE_DEG` env), SIGTERM clean shutdown, JSON logs to stdout/journald, view-switch demo keys (T/D/ESC).
- ✅ `views/dashboard.py` (placeholder cells + separators), `views/takeover.py` (stub).
- ✅ `systemd/chest-display.service` + `README.md` (install + reversible-spike steps).

**Verified on the Pi (2026-06-03):**
- ✅ pygame runtime resolved: distro `python3-pygame` (apt) + system `python3` (pip/poetry pygame's bundled SDL lacks kmsdrm). 11 headless tests also pass on the Pi.
- ✅ AC#2 — KMSDRM fullscreen surface acquired as `kamal` (no sudo); `display_ready driver=KMSDRM`. Saved Q#2 (rotation: native 800×480 landscape → app rotates 90°) + Q#3 (getty-stop gives DRM master) resolved.
- ✅ AC#3 — grid renders ("4 sections" confirmed on panel).
- ✅ AC#5/#6 — loop runs @ 30 fps target; clean SIGINT shutdown; separate process (panel-only).

- ✅ AC#1 — boot-to-app: systemd unit installed + enabled, `getty@tty1` disabled; **cold-boot confirmed by Kamal — dashboard comes up with no login flash; `is-active`=active**. DRM master works for a service (not just SSH).
- ✅ Orientation: `CHEST_ROTATE_DEG=90` is upright (Kamal-confirmed).

**Layout note (delivered vs AC#3 text):** AC#3 originally specced a "2×2 grid of four 240×400 cells." During live hardware bring-up Kamal refined the partition to **4 blocks — a full-width heart band (top 30%, 480×240) + a full-width middle band (480×280) + two bottom quadrants (240×280)** — with separators. The AC's *intent* (sectioned dashboard: heart focal + log areas + separators) is met; the exact split was adjusted to taste. `layout.py` is the single source: 8.2 targets `heart`; 8.3 targets `log_mid` / `log_bl` / `log_br`.

**All 6 ACs satisfied — Story 8.1 complete (→ review).**

### File List

_New (all under `ros2/src/chest_display/`):_
- `package.xml`, `setup.py`, `setup.cfg`, `resource/chest_display`
- `chest_display/__init__.py`
- `chest_display/layout.py`
- `chest_display/view_manager.py`
- `chest_display/app.py`
- `chest_display/views/__init__.py`
- `chest_display/views/dashboard.py`
- `chest_display/views/takeover.py`
- `systemd/chest-display.service`
- `README.md`
- `test/test_layout.py`
- `test/test_view_manager.py`

_Modified:_
- `docs/implementation-artifacts/sprint-status.yaml` (8-1 → in-progress)

### Change Log

- 2026-06-03: Implemented 8.1 logic (layout + view-manager + app skeleton + dashboard/takeover views + systemd + README); 11 headless tests green. Package shape resolved to a **separate ament package** (Saved Q#1, Kamal).
- 2026-06-03 (Pi bring-up): KMSDRM render confirmed on the panel. Runtime locked to distro `python3-pygame` + system `python3` (pip/poetry pygame lacks kmsdrm) → updated `systemd/chest-display.service` (ExecStart `/usr/bin/python3`) + `README.md`. Saved Q#2 (rotation: 800×480 landscape → app rotates 90°) and Q#3 (getty-stop = DRM master) resolved.
- 2026-06-03 (layout iteration, Kamal live feedback): dashboard layout → **4 blocks** (heart band + full-width middle band + 2 bottom quadrants); updated `layout.py`, `views/dashboard.py` separators, tests (now **15**), `README.md`.
- 2026-06-03 (boot-to-app): systemd unit installed + `enable --now`, `getty@tty1` disabled; **cold-boot confirmed — dashboard, no login flash**. All 6 ACs met. **Story 8.1 complete → review.**
