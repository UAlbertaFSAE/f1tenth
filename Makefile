.PHONY: help deps build build_all clean rebuild test package run run_auto run_sim

.DEFAULT_GOAL := help

PACKAGES_IGNORE ?= livox_ros_driver2 livox_sdk2 zed_wrapper zed_components

# Resolve the workspace root (parent of this file's directory) from this
# Makefile's own location, not the caller's cwd -- so `make` works the same
# whether invoked from the workspace root, from src/ itself, or via
# `make -C src` / `make -f src/Makefile` from anywhere else. Every recipe
# below cds into WS_ROOT first, so build/install/log always land in the
# workspace root and never inside this (git-tracked) src/ directory.
WS_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)

VENV_ACTIVATE := src/venv/bin/activate

help:
	@echo "Usage:"
	@echo "  make deps                         Run full environment setup (scripts/setup.sh)"
	@echo "  make build                        Build packages excluding ignored packages"
	@echo "  make build_all                    Build all packages"
	@echo "  make package PKG1 PKG2 ...        Build specific package(s)"
	@echo "  make clean                        Remove build artifacts"
	@echo "  make rebuild                      Clean and rebuild"
	@echo "  make test                         Run tests"
	@echo "  make run_auto                     Run autonomous launch"
	@echo "  make run_sim                      Run simulation launch"

deps:
	cd $(WS_ROOT) && bash src/scripts/setup.sh

build:
	cd $(WS_ROOT) && colcon build --packages-ignore $(PACKAGES_IGNORE)

build_all:
	cd $(WS_ROOT) && colcon build

package:
	@if [ "$(words $(MAKECMDGOALS))" -lt 2 ]; then \
		echo "Error: No package specified."; \
		echo "Usage: make package <package_name> [package_name ...]"; \
		exit 1; \
	fi
	cd $(WS_ROOT) && colcon build --packages-select $(filter-out package,$(MAKECMDGOALS))

clean:
	cd $(WS_ROOT) && rm -rf build install log

rebuild: clean build

test:
	cd $(WS_ROOT) && colcon test
	cd $(WS_ROOT) && colcon test-result --verbose

run_auto:
	cd $(WS_ROOT) && bash -c "source $(VENV_ACTIVATE) && source install/setup.bash && ros2 launch launch_pkg fsae.launch.py config:=config.yaml"

run_sim:
	cd $(WS_ROOT) && bash -c "source $(VENV_ACTIVATE) && source install/setup.bash && ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml"

# Allow package names to be passed as command-line arguments
%:
	@:
