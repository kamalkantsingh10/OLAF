#!/usr/bin/env bash
#
# OLAF body — production entrypoint.
#
# This is how the body runs FOR REAL: it brings up the expression engine
# so it RECEIVES the producer's four canonical topics (mood / activity /
# speech_emotion / vocalization) on the configured DDS domain and renders
# them on the hardware. The producer (olaf_companion, or any publisher on
# the same domain — see ros2/src/expression_engine/contract/INTERFACE.md)
# pushes intent; this process subscribes and drives eyes / ears / neck /
# LEDs. It does NOT publish.
#
# Under the hood it runs expression_engine/node.py:main — the §9 startup:
#   load config + expression map  ->  connect adapters (wakes the head
#   ESP32)  ->  subscribe to the 4 topics  ->  start the render loop  ->
#   spin. Fatal-fast (NFR7): a bad config/map or an adapter that won't
#   connect exits non-zero.
#
# Usage (on the Pi, where the hardware is attached):
#   scripts/start-body.sh                       # run it
#   scripts/start-body.sh --ros-args -r __ns:=/olaf_2   # namespaced body
# Stop with Ctrl-C (SIGINT) or SIGTERM (systemd) — both exit cleanly.
#
# DDS domain comes from config/expression_engine.toml ([dds].domain_id);
# the producer MUST be on the same domain.
set -euo pipefail

# Resolve the repo root from this script's location, so it works no
# matter where it's invoked from (manual, systemd, cron, boot).
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ROS 2 (rclpy) overlay. setup.bash is NOT nounset-clean — it reads
# unset vars (AMENT_TRACE_SETUP_FILES, ...), which `set -u` treats as a
# fatal "unbound variable". Relax -u just for the source, then restore.
set +u
source /opt/ros/jazzy/setup.bash
set -u

# PREPEND the package paths — never clobber the ROS overlay (that would
# drop rclpy).
PKG_PATHS="ros2/src/expression_engine:ros2/src/olaf_drivers/neck_driver:ros2/src/olaf_drivers/head_ears_driver:libs"
export PYTHONPATH="${PKG_PATHS}:${PYTHONPATH:-}"

# `exec` so signals (SIGTERM from systemd) reach the node directly and it
# shuts the adapters down cleanly. `poetry` resolves via the login PATH
# on the Pi (~/.local/bin); a systemd unit should use the full path.
exec poetry run python -m expression_engine.node "$@"
