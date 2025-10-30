# OLAF Build System - Convenience wrapper for ROS2 and firmware build commands
#
# This Makefile provides simple commands for common development tasks,
# abstracting away the details of working across multiple subsystems.
#
# Usage: make <command>
# Example: make ros-build

.PHONY: help ros-build ros-launch ros-test firmware-head firmware-all test clean

help:
	@echo "OLAF Build System"
	@echo "================="
	@echo ""
	@echo "ROS2 Orchestration Layer:"
	@echo "  make ros-build        Build all ROS2 packages"
	@echo "  make ros-launch       Launch full OLAF system"
	@echo "  make ros-test         Run ROS2 tests"
	@echo "  make ros-clean        Clean ROS2 build artifacts"
	@echo ""
	@echo "Firmware Layer (ESP32):"
	@echo "  make firmware-head    Build and upload head module firmware"
	@echo "  make firmware-all     Build all module firmware"
	@echo ""
	@echo "Testing:"
	@echo "  make test             Run all tests (ROS2 + firmware)"
	@echo ""
	@echo "Cleanup:"
	@echo "  make clean            Clean all build artifacts"

# ==============================================================================
# ROS2 Commands
# ==============================================================================

ros-build:
	@echo "Building ROS2 packages..."
	cd ros2 && colcon build --symlink-install

ros-launch:
	@echo "Launching OLAF full system..."
	cd ros2 && source install/setup.bash && \
		ros2 launch orchestrator olaf_full.launch.py

ros-test:
	@echo "Running ROS2 tests..."
	cd ros2 && colcon test

ros-clean:
	@echo "Cleaning ROS2 build artifacts..."
	rm -rf ros2/build ros2/install ros2/log

# ==============================================================================
# Firmware Commands
# ==============================================================================

firmware-head:
	@echo "Building and uploading head module firmware..."
	cd firmware/head && pio run -t upload

firmware-all:
	@echo "Building all module firmware..."
	@for module in head ears-neck body base; do \
		if [ -f firmware/$$module/platformio.ini ]; then \
			echo "Building $$module..."; \
			cd firmware/$$module && pio run && cd ../..; \
		else \
			echo "Skipping $$module (no platformio.ini found)"; \
		fi \
	done

# ==============================================================================
# Testing Commands
# ==============================================================================

test: ros-test
	@echo "✅ All tests complete"
	@echo "Note: Firmware tests not yet implemented"

# ==============================================================================
# Cleanup Commands
# ==============================================================================

clean: ros-clean
	@echo "Cleaning firmware build artifacts..."
	@for module in head ears-neck body base; do \
		if [ -d firmware/$$module/.pio ]; then \
			echo "Cleaning $$module/.pio..."; \
			rm -rf firmware/$$module/.pio; \
		fi \
	done
	@echo "✅ All build artifacts cleaned"
