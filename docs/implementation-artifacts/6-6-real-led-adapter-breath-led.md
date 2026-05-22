# Story 6.6: Real LED adapter + breath-LED

Status: done

> **SUPERSEDED 2026-05-22 (Kamal — "we do not need 6.6").** This story
> assumed an ENGINE-OWNED WS2812 on the Pi (GPIO18), rendered frame-by-
> frame behind the `SurfaceAdapter` Protocol. In reality the strip lives
> on the **HEAD ESP32** (`modules/head/firmware/led_strip.cpp`) as a
> SMART PERIPHERAL: the engine drives it SEMANTICALLY over I2C via
> `system_status` (pattern + lit/dark) and `led_overlay` (mood tint),
> both wired in **Story 7.3**. So there is no engine-side `led_adapter` /
> `SurfaceFrame` LED rendering, and no Pi-side WS2812 lib/privilege
> question. "Breath-LED" is replaced by the 7.3/6.5 decision: idle =
> strip OFF; the lit patterns (listening/processing/speaking, symmetric,
> mood-tinted) are firmware-owned. The toml `[hardware]`
> `led_strip_count`/`led_strip_pin` are vestigial (no Pi-side strip).
> **Goal "mood + state visible on the strip" = MET via 7.3.** No code
> written for this story.

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the avatar,
I want real ambient LED behaviour,
so that mood and idle state are visible on the strip.

> Prerequisite: Stories 6.1–6.5 `done`. Story 6.5 already drives "breath-LED" through a placeholder LED path; this story makes it a real engine-owned WS2812 adapter behind the frozen `SurfaceAdapter` Protocol — the FSM must not change.

## Acceptance Criteria

1. **Given** the WS2812 strip, **When** the engine drives LEDs, **Then** an engine-owned adapter behind the `SurfaceAdapter` Protocol drives it, with no separate driver package (FR11, AR1).
2. **Given** the engine in an idle state, **When** the loop ticks, **Then** breath-LED renders (ties to Story 6.5).
3. **Given** a need to swap the LED hardware, **When** assessed, **Then** the change is a single new Protocol implementation only (NFR6).

## Tasks / Subtasks

- [ ] Task 1: `adapters/led_adapter.py` (AC: #1)
  - [ ] Implement the frozen `SurfaceAdapter` Protocol (`connect/close/render(frame)`); engine-owned, NO separate ROS driver package (FR11, AR1)
  - [ ] `render(frame)` drives the strip from `frame.led_color / intensity / pattern`
  - [ ] Hardware params from `expression_engine.toml [hardware]`: `led_strip_count=24`, `led_strip_pin=18` (architecture §8)
- [ ] Task 2: Wire into the render loop (AC: #2)
  - [ ] LED is engine-owned + continuous: ticked at `led_tick_hz` (default 30) for breathe; fire-on-change otherwise (AR1)
  - [ ] Breath-LED pattern renders during AMBIENT/DORMANT at `breath_period_seconds` (Story 6.5 already produces the intent; this renders it on real hardware)
- [ ] Task 3: SurfaceFrame definition (AC: #1)
  - [ ] Finalize the `SurfaceFrame` shape referenced by the frozen Protocol (`led_color`, `intensity`, `pattern`; heart fields reserved for Story 8.1) — coordinate with the 6.4 freeze; if it requires touching frozen surface, treat as a documented amendment
- [ ] Task 4: Hardware run (AC: #1, #2)
  - [ ] On the Pi: solid mood color, then idle breathe; visually correct, no flicker; `connect()` fatal if strip absent (NFR7)
- [ ] Task 5: Tests (AC: #1, #3)
  - [ ] Adapter satisfies `SurfaceAdapter` (`runtime_checkable` isinstance)
  - [ ] Breathe pattern: intensity is periodic over `breath_period_seconds`, smooth (no step)
  - [ ] Swap test: a second `SurfaceAdapter` impl substitutes with zero change to loop/map/FSM (proves NFR6)

## Dev Notes

### Engine-owned, not a driver package (FR11, AR1)

There is **no Phase 1 LED driver** and there must be **no separate LED driver package**. The WS2812 strip is an engine-owned surface rendered directly by `led_adapter.py` behind the `SurfaceAdapter` Protocol. This is the architectural opposite of neck/ears (which wrap Phase 1 driver classes). [Source: phase2-expression-engine.md#2, #4; phase2-prd.md#FR11]

### SurfaceAdapter Protocol (frozen in Story 6.4)

```python
@runtime_checkable
class SurfaceAdapter(Protocol):
    def connect(self) -> None: ...   # fatal on failure (NFR7)
    def close(self) -> None: ...
    def render(self, frame: "SurfaceFrame") -> None: ...   # per tick (LED) / low-rate (heart)
```
`led_adapter` maps `render(frame)` → drive 24 LEDs from `frame.led_color / intensity / pattern`. Same Protocol the heart adapter implements in Story 8.1 — keep `SurfaceFrame` general enough for both but do NOT implement heart here. [Source: phase2-expression-engine.md#4]

### WS2812 hardware specifics

- 24 LEDs, data pin GPIO18 (`expression_engine.toml [hardware] led_strip_count=24 led_strip_pin=18`). Confirm against actual wiring / `config/servo-ids.yaml` siblings before driving — do not assume.
- Pick the WS2812 library deliberately and pin its version in `setup.py`/`package.xml`. GPIO18 + PWM is the common `rpi_ws281x`/`adafruit-circuitpython-neopixel` path on a Pi 5 — verify Pi 5 support (Pi 5 changed the GPIO/PWM stack; some legacy `rpi_ws281x` builds don't work on Pi 5). Record the chosen lib + why in Completion Notes. [Saved question below]
- Driving WS2812 on a Pi 5 may need root/PWM privileges or a specific overlay — if so, document the runtime requirement (feeds Story 6.7 systemd unit).
- `connect()` must be fatal if the strip can't be initialized (NFR7, startup sequence step 5).

### Anti-patterns

- Do NOT add a ROS node/topic for LEDs (FR11 — engine-owned, no driver package, no intermediate topic).
- Do NOT recompute breathe timing here — Story 6.5's FSM/loop already produces the breathe intent; this adapter just renders frames. Keep timing in the loop (AR3, jitter-free).
- Do NOT block the render tick on LED I/O — budget the strip write within the `led_tick_hz` period.

### Previous Story Intelligence (6.1–6.5)

- 6.4 froze `SurfaceAdapter`; 6.5 produces breathe/ambient intent through a placeholder LED path. This story swaps the placeholder for real hardware **without touching the FSM or loop** (NFR6 in action — and itself the proof for AC #3).
- Reuse the established `connect()`-fatal / journald patterns for the strip-absent case.

### Project Structure Notes

```
ros2/src/expression_engine/expression_engine/
  adapters/led_adapter.py   # NEW
  adapters/base.py          # UPDATE only if SurfaceFrame needs finalizing (documented amendment if frozen)
  render_loop.py            # UPDATE — bind real led_adapter (replace placeholder)
  test/test_led_adapter.py  # NEW
```
Matches architecture §2/§4. No variance.

### References

- [Source: docs/planning-artifacts/epics.md#Story-6.6]
- [Source: docs/planning-artifacts/prd/phase2-prd.md — FR11, NFR6, NFR7]
- [Source: docs/planning-artifacts/architecture/phase2-expression-engine.md#2, #4, #8]

### Saved Questions

1. Confirm the WS2812 library choice for Pi 5 (GPIO18) — `rpi_ws281x` vs `adafruit-circuitpython-neopixel` vs Pi 5-specific path — and whether it needs elevated privileges (impacts the Story 6.7 systemd unit).

## Dev Agent Record

### Agent Model Used

### Debug Log References

### Completion Notes List

### File List
