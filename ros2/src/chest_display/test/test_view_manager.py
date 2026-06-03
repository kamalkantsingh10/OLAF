"""Headless tests for the view-manager (no pygame needed).

Proves the DASHBOARD <-> FULLSCREEN_TAKEOVER round-trip (AC #4) at the
logic level; the actual on-panel rendering is verified on hardware.
"""

import pytest

from chest_display.view_manager import View, ViewManager


class FakeView(View):
    def __init__(self, name):
        super().__init__(name)
        self.updates = 0
        self.draws = 0
        self.last_dt = None

    def update(self, dt):
        self.updates += 1
        self.last_dt = dt

    def draw(self, surface):
        self.draws += 1


def make_vm():
    vm = ViewManager()
    dash = FakeView("dashboard")
    take = FakeView("takeover")
    vm.register(dash, default=True, activate=True)
    vm.register(take)
    return vm, dash, take


def test_default_active_is_dashboard():
    vm, dash, _take = make_vm()
    assert vm.active_name == "dashboard"
    assert vm.active is dash


def test_switch_and_return_round_trip():
    vm, dash, take = make_vm()
    assert vm.switch_to("takeover") is take
    assert vm.active_name == "takeover"
    assert vm.return_to_dashboard() is dash
    assert vm.active_name == "dashboard"


def test_switch_unknown_raises():
    vm, _, _ = make_vm()
    with pytest.raises(KeyError):
        vm.switch_to("nope")


def test_update_and_draw_delegate_to_active_only():
    vm, dash, take = make_vm()
    vm.update(0.016)
    vm.draw(surface=None)
    assert dash.updates == 1 and dash.draws == 1
    assert take.updates == 0 and take.draws == 0

    vm.switch_to("takeover")
    vm.update(0.033)
    vm.draw(surface=None)
    assert take.updates == 1 and take.draws == 1
    assert take.last_dt == 0.033


def test_register_first_view_becomes_default_and_active():
    vm = ViewManager()
    only = FakeView("solo")
    vm.register(only)
    assert vm.active is only
    assert vm.return_to_dashboard() is only  # default falls back to the first registered
