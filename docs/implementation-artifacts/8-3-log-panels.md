# Story 8.3: Log panels (three placeholder panels)

Status: ready-for-dev

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

- [ ] Task 1: `LogPanel` widget (AC: #1, #3)
  - [ ] Titled panel: small title bar + monospace body, dim/cool palette; draws within a given 240×400 rect
  - [ ] Bounded ring buffer of lines; newest-first scroll; wrap or truncate lines to the cell width (no overflow into neighbors) [Saved Question #2 — font/size for portrait legibility]
- [ ] Task 2: Feed API (AC: #4)
  - [ ] `push(line: str)` appends one line (evicts oldest past the cap); `set_lines(lines: list[str])` replaces the buffer
  - [ ] No data source baked in — the panel only knows how to display lines it is given
- [ ] Task 3: Dummy content (AC: #2)
  - [ ] Instantiate three panels in TR/BL/BR with placeholder titles `SYSTEM` / `SPEECH` / `SENSORS` and rotating dummy lines (timestamps + sample messages) so scrolling/legibility are visible [Saved Question #1 — real feed identities]
- [ ] Task 4: Visual hierarchy (AC: #5)
  - [ ] Tune panel colors dim/cool so the heart's warm glow stays the focal point; verify the four-cell composition together
- [ ] Task 5: Tests / verification (AC: #3, #4)
  - [ ] Unit: ring buffer caps correctly; `push`/`set_lines` behave; long-line wrap/truncate stays within width (no headless display needed)
  - [ ] On the Pi: three panels scroll dummy lines, text stays inside cells, heart still reads as the focal element

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

### Debug Log References

### Completion Notes List

### File List
