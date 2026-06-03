# Story 8.3: Log panels (three placeholder panels)

Status: review

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As the maintainer,
I want the three non-heart quarters to be reusable log panels showing dummy data,
so that the dashboard layout and legibility can be evaluated now and real feeds wired in later with no rework.

> Depends on Story 8.1 (`DashboardView` + the three 240×400 cells TR/BL/BR). Builds a reusable `LogPanel` widget; **dummy data only** — real feeds are wired later (deferred). Completes the dashboard MVP alongside the heart (8.2).

## Acceptance Criteria

1. **Given** the three non-heart cells, **When** the app runs, **Then** each renders a reusable **`LogPanel`** widget: a titled, monospace, dim/cool-styled panel showing scrolling text lines within its 240×400 bounds.
2. **Given** no real data feed is wired yet, **When** the panels render, **Then** each shows **placeholder/dummy lines** under a placeholder title (e.g. `SYSTEM` / `SPEECH` / `SENSORS`) so layout and readability can be judged.
3. **Given** a `LogPanel`, **When** lines accumulate, **Then** it scrolls (newest-first), caps retained lines to a bounded buffer, and wraps/truncates long lines to the cell width without overflowing into adjacent cells.
4. **Given** real data becomes available later, **When** a panel is wired, **Then** feeding it is a single `push(line)` / `set_lines(...)` call per panel — no layout or styling rework required.
5. **Given** all four cells render together, **When** the dashboard is viewed, **Then** the **heart remains the focal point** (warm glow) and the log panels stay visually subordinate (cool, dim) so the chest reads as "a heart with status around it," not "three text boxes and a heart."

## Tasks / Subtasks

- [x] Task 1: `LogPanel` widget (AC: #1, #3)
  - [x] Titled panel: title bar + rule + monospace body, cool/dim palette; draws within any given rect — `widgets/log_panel.py`
  - [x] Bounded ring buffer (`LogBuffer`, deque maxlen); **terminal-tail** scroll (newest at the bottom); long lines **truncated to width with `…`** (no overflow; dashboard also clips each cell)
- [x] Task 2: Feed API (AC: #4)
  - [x] `push(line)` (evicts oldest past cap) + `set_lines(lines)` (replaces, keeps newest within cap); coerces to str
  - [x] No data source baked in — the panel only displays lines it is given (real feeds in 8.4+ call these)
- [x] Task 3: Dummy content (AC: #2)
  - [x] Three panels in **`log_mid` (`SYSTEM`) / `log_bl` (`SPEECH`) / `log_br` (`SENSORS`)**; a `_DummyFeeder` pushes a timestamped line ~every 1.3s so scrolling/legibility are visible. **Saved Q#1 (real feed identities) still open** — titles are placeholders.
- [x] Task 4: Visual hierarchy (AC: #5)
  - [x] Cool/dim panel palette; heart stays the focal point — verified on the panel together (Kamal: "looks perfect")
- [x] Task 5: Tests / verification (AC: #3, #4)
  - [x] Unit: ring-buffer cap/eviction, `push`/`set_lines`, ordering, str-coercion — **5 tests** (`test_log_panel.py`); 32 total in the package
  - [x] On the Pi: three panels scroll dummy lines, text stays inside cells, heart reads as focal — confirmed

## Dev Notes

### One reusable widget, three instances

Build a single `LogPanel` used three times — not three bespoke panels. The only per-instance differences are title, position (cell rect), and (later) data source. This is what makes "wire real data later" a one-line change. [Source: epics.md#Story-8.3]

### Display-only, data-source-agnostic

The panel must not know where lines come from. For this story the lines are dummy; later (deferred) a real feed calls `push()`/`set_lines()`. Keep all formatting/scroll/wrap logic inside the widget so the future wiring is trivial. The three real feeds are intentionally undecided — placeholder titles `SYSTEM`/`SPEECH`/`SENSORS` are illustrative, not a commitment [Saved Question #1]. [Source: epics.md#Story-8.3, prompt 2026-06-03 "for now just put dummy values — we add later"]

### Hierarchy: heart is the star

Kamal's framing: heart top-left is the focal point; the three logs are subordinate status around it. Keep panels cool/dim (low-contrast monospace) so they never out-shout the heart's warm glow. Evaluate the full 2×2 together, not panels in isolation. [Source: epics.md#Story-8.2/8.3]

### Portrait legibility

Cells are 240 wide × 400 tall on an 800-tall portrait panel viewed across a room — pick a monospace size that fits a useful number of readable lines without crowding [Saved Question #2]. Truncate/wrap so nothing bleeds into the heart cell or between panels.

### Anti-patterns

- Do **not** hardcode a specific data source or schema — display-only, fed via `push`/`set_lines`.
- Do **not** let text overflow the cell — wrap/truncate to width, cap the buffer.
- Do **not** make panels visually compete with the heart — cool/dim, subordinate.
- Do **not** duplicate the panel three times — one widget, three instances.

### Project Structure Notes

```
ros2/src/chest_display/chest_display/
  widgets/log_panel.py    # NEW — LogPanel: draw(surface, rect), push(line), set_lines(lines)
  views/dashboard.py      # UPDATE — instantiate 3 LogPanels in TR/BL/BR with dummy data
  test/test_log_panel.py  # NEW — ring buffer + wrap/truncate
```
Builds on the 8.1 skeleton and sits beside the 8.2 heart widget. [Source: Story 8.1/8.2 Project Structure Notes]

### References

- [Source: docs/planning-artifacts/epics.md#Epic-8 — Story 8.3]
- [Source: docs/planning-artifacts/prd/phase2-prd.md#Epic-8 — Story 8.3]
- [Source: user prompt 2026-06-03 — "divide the screen into 4 quarters … other three will have logs (dummy values, add later)"]

### Saved Questions

1. **Real feed identities** — what do the three panels actually show later (e.g., system/journald, speech/emotion events, sensor telemetry)? Placeholder titles `SYSTEM`/`SPEECH`/`SENSORS` are stand-ins.
2. **Font + size** — preferred monospace and point size for 240×400 cells on the portrait panel at viewing distance.

## Dev Agent Record

### Agent Model Used

claude-opus-4-8 (1M context)

### Debug Log References

- `python3 -m pytest test/ -o addopts=""` → **32 passed** (dev PC + Pi). Headless smoke (SDL `dummy`): dummy feeder accumulates lines and the panels render without spill.

### Completion Notes List

- **One reusable `LogPanel` (+ pure `LogBuffer`) used three times**, in the **`log_mid` / `log_bl` / `log_br`** areas (the 8.1 4-block layout, not the original "TR/BL/BR 240×400 quarters" AC text). Titles `SYSTEM` / `SPEECH` / `SENSORS` are placeholders (Saved Q#1 open).
- **Scroll is terminal-tail** (newest at the bottom) — read of AC#3's "newest-first"; easy to flip if preferred.
- Long lines **truncate to width with `…`**; the dashboard also **clips each cell**, so nothing spills (AC#3).
- **Data-source-agnostic:** real feeds in 8.4+ just call `panel.push(...)` / `set_lines(...)`. The `_DummyFeeder` (in `dashboard.py`) is placeholder-only and gets removed/replaced when real data lands.
- **Hierarchy (AC#5):** cool/dim palette keeps the heart focal — Kamal confirmed "looks perfect" with all four blocks live. (Note AC#5 says "warm glow" — the heart has no glow now; it's the 3D image, per 8.2.)
- ACs #1–#5 met (with the layout/scroll/glow wording reconciled above).

### File List

_New:_
- `ros2/src/chest_display/chest_display/widgets/log_panel.py` — `LogBuffer` (pure) + `LogPanel` (pygame)
- `ros2/src/chest_display/test/test_log_panel.py`

_Modified:_
- `ros2/src/chest_display/chest_display/views/dashboard.py` — instantiate 3 log panels + `_DummyFeeder`
- `docs/implementation-artifacts/sprint-status.yaml` (8-3 → review)

### Change Log

- 2026-06-03: Implemented 8.3 — reusable `LogPanel`/`LogBuffer`, three placeholder panels (SYSTEM/SPEECH/SENSORS) with a dummy feeder; 5 buffer tests (32 total); verified scrolling + hierarchy on the panel. **Story 8.3 → review.** Dashboard MVP (heart + logs) complete; real feeds + emotion wiring = 8.4.
