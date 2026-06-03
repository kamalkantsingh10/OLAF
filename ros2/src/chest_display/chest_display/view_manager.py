"""View-manager: the active-view indirection that makes full-screen takeover
a "register a view + switch" operation rather than a rewrite (AC #4).

Pure logic — NO pygame import. `View.draw(surface)` is a no-op in the base so
the manager's switching behaviour is unit-testable headlessly; concrete views
(dashboard, takeover) override `draw` with real pygame rendering.

Forward-looking (Kamal's requirement): a single app can take the full 480x800
screen, then the manager returns to the dashboard when that work is done. The
dashboard's widgets (heart, logs) keep running regardless of the active view.
"""

from __future__ import annotations


class View:
    """Base view. Subclasses override `update`/`draw`."""

    def __init__(self, name: str):
        self.name = name

    def update(self, dt: float) -> None:  # noqa: D401 - simple hook
        """Advance animation by `dt` seconds. No-op in the base."""

    def draw(self, surface) -> None:
        """Render onto `surface`. No-op in the base (overridden by concrete views)."""


class ViewManager:
    """Holds registered views and renders exactly one (the active) per frame."""

    def __init__(self) -> None:
        self._views: dict[str, View] = {}
        self._active: str | None = None
        self._default: str | None = None

    def register(self, view: View, *, default: bool = False, activate: bool = False) -> View:
        """Register `view`. The first registered view becomes default+active unless
        a later call overrides with `default=True` / `activate=True`."""
        self._views[view.name] = view
        if default or self._default is None:
            self._default = view.name
        if activate or self._active is None:
            self._active = view.name
        return view

    @property
    def active(self) -> View | None:
        return self._views.get(self._active) if self._active is not None else None

    @property
    def active_name(self) -> str | None:
        return self._active

    def switch_to(self, name: str) -> View:
        """Make `name` the active view. Raises KeyError if unknown."""
        if name not in self._views:
            raise KeyError(f"unknown view: {name!r}")
        self._active = name
        return self._views[name]

    def return_to_dashboard(self) -> View:
        """Return to the default (dashboard) view — used when a takeover finishes."""
        assert self._default is not None, "no default view registered"
        return self.switch_to(self._default)

    def update(self, dt: float) -> None:
        if self.active is not None:
            self.active.update(dt)

    def draw(self, surface) -> None:
        if self.active is not None:
            self.active.draw(surface)
