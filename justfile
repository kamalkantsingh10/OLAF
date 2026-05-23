# OLAF Build System - just task runner (migrated from Makefile)
#
# Convenience wrapper for ROS2 and firmware build commands, abstracting
# away the details of working across multiple subsystems.
#
# Usage:   just <command>      (bare `just` shows this help)
# Example: just ros-build

# Show available commands (default)
help:
    @echo "OLAF Build System"
    @echo "================="
    @echo ""
    @echo "ROS2 Orchestration Layer:"
    @echo "  just ros-build        Build all ROS2 packages"
    @echo "  just ros-launch       Launch full OLAF system"
    @echo "  just ros-test         Run ROS2 tests"
    @echo "  just ros-clean        Clean ROS2 build artifacts"
    @echo ""
    @echo "Firmware Layer (ESP32):"
    @echo "  just firmware-head    Build and upload head module firmware"
    @echo "  just firmware-all     Build all module firmware"
    @echo ""
    @echo "Testing:"
    @echo "  just test             Run all tests (ROS2 + firmware)"
    @echo "  just exp-test         expression_engine host tests (poetry)"
    @echo ""
    @echo "Robot (run ON the Pi — drives real hardware):"
    @echo "  just body-up          Boot the body — bring the engine node up & ready"
    @echo "  just monitor          Live-watch the 4 interface topics (read-only)"
    @echo "  just bodylog          Watch the body's own logs on /rosout (remote)"
    @echo "  just topics           List the /olaf/* topics on the DDS domain"
    @echo "  just activity-walk    Story 7.3 — walk every ActivityState"
    @echo "  just voc-run          Story 7.2 — fire each vocalization"
    @echo "  just idle-run         Story 6.5 — watch the head drift to sleep"
    @echo "  just exp-run <script> Run any expression_engine script"
    @echo ""
    @echo "Cleanup:"
    @echo "  just clean            Clean all build artifacts"

# ==============================================================================
# ROS2 Commands
# ==============================================================================

# Build all ROS2 packages
ros-build:
    @echo "Building ROS2 packages..."
    cd ros2 && colcon build --symlink-install

# Launch the full OLAF system
ros-launch:
    #!/usr/bin/env bash
    echo "Launching OLAF full system..."
    cd ros2 && source install/setup.bash && \
        ros2 launch orchestrator olaf_full.launch.py

# Run ROS2 tests
ros-test:
    @echo "Running ROS2 tests..."
    cd ros2 && colcon test

# Clean ROS2 build artifacts
ros-clean:
    @echo "Cleaning ROS2 build artifacts..."
    rm -rf ros2/build ros2/install ros2/log

# ==============================================================================
# Firmware Commands
# ==============================================================================

# Build and upload head module firmware
firmware-head:
    @echo "Building and uploading head module firmware..."
    cd firmware/head && pio run -t upload

# Build all module firmware
firmware-all:
    #!/usr/bin/env bash
    echo "Building all module firmware..."
    for module in head ears-neck body base; do
        if [ -f firmware/$module/platformio.ini ]; then
            echo "Building $module..."
            (cd firmware/$module && pio run)
        else
            echo "Skipping $module (no platformio.ini found)"
        fi
    done

# ==============================================================================
# Testing Commands
# ==============================================================================

# Run all tests (ROS2 + firmware)
test: ros-test
    @echo "✅ All tests complete"
    @echo "Note: Firmware tests not yet implemented"

# expression_engine host tests (poetry env + ROS PYTHONPATH). Extra
# pytest args pass through, e.g. `just exp-test -k activity`.
exp-test *args:
    PYTHONPATH="ros2/src/expression_engine:${PYTHONPATH}" \
        poetry run python -m pytest ros2/src/expression_engine/test/ {{args}}

# ==============================================================================
# Robot Commands — RUN ON THE Pi (olaf.local). These drive the real
# I2C head + servos, so they only work where the hardware is attached.
# Each sources ROS (jazzy) for rclpy and PREPENDS the package paths to
# PYTHONPATH (never clobbers the ROS overlay — that drops rclpy).
# `poetry` + `just` resolve via ~/.local/bin on the Pi's login PATH.
# ==============================================================================

# Package paths prepended to PYTHONPATH (engine + drivers + libs).
exp_pp := "ros2/src/expression_engine:ros2/src/olaf_drivers/neck_driver:ros2/src/olaf_drivers/head_ears_driver:libs"

# Thin wrapper over scripts/start-body.sh — the production entrypoint
# that runs the body to RECEIVE the producer's 4 topics and render them
# (the §9 startup: load config+map, connect adapters — wakes the head
# ESP32 — subscribe, render). Fatal-fast; foreground; Ctrl-C to stop.
# Run ON the Pi. Args pass through as ROS args, e.g. a namespaced 2nd
# body (Story 7.6): just body-up --ros-args -r __ns:=/olaf_2

# Boot the body (engine node) & leave it ready to render — Pi only.
body-up *args:
    ./scripts/start-body.sh {{args}}

# Joins the engine's configured DDS domain + per-topic QoS, so it sees
# exactly what the body sees — latched mood/activity replay their
# current value on connect (a plain `ros2 topic echo` would miss them).
# Read-only. Ctrl-C to stop. Run alongside `just body-up` or against a
# producer.

# Live-watch the 4 interface topics (read-only) — same domain as body.
monitor:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH="{{exp_pp}}:${PYTHONPATH:-}" \
        poetry run python ros2/src/expression_engine/tools/monitor.py

# Watch the body's OWN logs (engine events, crashes, received + the
# conversation narrative) which it publishes to the standard /rosout
# topic — so you can follow it from ANY machine on domain 42, no SSH.
# Shows the msg field per entry. GUI alternative: rqt_console.

# Tail the body's /rosout logs (domain 42) — remote, no SSH.
bodylog:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    ROS_DOMAIN_ID=42 ros2 topic echo /rosout --field msg

# Quick discovery on domain 42. `-v` adds QoS + pub/sub counts:
#   just topics -v

# List the /olaf/* topics on the DDS domain (discovery check).
topics *args:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    ROS_DOMAIN_ID=42 ros2 topic list {{args}} | grep -E '/olaf/' \
        || echo "(no /olaf/* topics on domain 42 — is the body or a producer up?)"

# Run any expression_engine script on the robot (path relative to the
# package). Usage: just exp-run test/e2e_activity_run.py [args...]
exp-run script *args:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH="{{exp_pp}}:${PYTHONPATH:-}" \
        poetry run python ros2/src/expression_engine/{{script}} {{args}}

# Story 7.3 hardware walk — step every ActivityState (posture + eyes +
# LEDs). Optional leaf labels for a subset:
#   just activity-walk listening speaking
activity-walk *args:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH="{{exp_pp}}:${PYTHONPATH:-}" \
        poetry run python ros2/src/expression_engine/test/e2e_activity_run.py {{args}}

# Story 7.2 — fire each vocalization on the robot. Optional tag subset.
voc-run *args:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH="{{exp_pp}}:${PYTHONPATH:-}" \
        poetry run python ros2/src/expression_engine/test/e2e_vocalization_run.py {{args}}

# Story 6.5 — watch the head drift to sleep (idle decay) on the robot.
# Fast demo timings by default; see the script header for flags.
idle-run *args:
    #!/usr/bin/env bash
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH="{{exp_pp}}:${PYTHONPATH:-}" \
        poetry run python ros2/src/expression_engine/test/e2e_idle_run.py {{args}}

# ==============================================================================
# Cleanup Commands
# ==============================================================================

# Clean all build artifacts (ROS2 + firmware)
clean: ros-clean
    #!/usr/bin/env bash
    echo "Cleaning firmware build artifacts..."
    for module in head ears-neck body base; do
        if [ -d firmware/$module/.pio ]; then
            echo "Cleaning $module/.pio..."
            rm -rf firmware/$module/.pio
        fi
    done
    echo "✅ All build artifacts cleaned"
