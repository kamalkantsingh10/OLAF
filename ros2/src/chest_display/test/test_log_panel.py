"""Headless tests for the LogPanel's line buffer (no pygame needed).

The buffer is the data-source-agnostic core (AC#3 bounded buffer, AC#4 push/
set_lines API); the on-panel rendering (wrap/truncate/scroll) is verified visually.
"""

from chest_display.widgets.log_panel import LogBuffer


def test_push_appends_in_order():
    b = LogBuffer(capacity=10)
    for s in ["a", "b", "c"]:
        b.push(s)
    assert b.lines() == ["a", "b", "c"]


def test_capacity_evicts_oldest():
    b = LogBuffer(capacity=3)
    for s in ["a", "b", "c", "d", "e"]:
        b.push(s)
    assert b.lines() == ["c", "d", "e"]   # newest kept, oldest dropped
    assert len(b) == 3


def test_set_lines_replaces_buffer():
    b = LogBuffer(capacity=5)
    b.push("old")
    b.set_lines(["x", "y", "z"])
    assert b.lines() == ["x", "y", "z"]


def test_set_lines_respects_capacity_keeping_newest():
    b = LogBuffer(capacity=2)
    b.set_lines(["1", "2", "3", "4"])
    assert b.lines() == ["3", "4"]


def test_push_coerces_to_str():
    b = LogBuffer(capacity=5)
    b.push(42)
    assert b.lines() == ["42"]
