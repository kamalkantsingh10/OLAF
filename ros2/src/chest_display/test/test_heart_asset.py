"""The default heart sprite asset must ship with the package."""

import os


def test_heart_asset_present():
    from chest_display.widgets.heart import DEFAULT_IMAGE

    assert os.path.exists(DEFAULT_IMAGE), f"missing heart sprite: {DEFAULT_IMAGE}"
