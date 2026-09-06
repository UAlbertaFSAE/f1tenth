# Single entry point for building, testing, linting and running this workspace.
#
# Everything used to be spread across scripts/formula_bashrc.sh, simulator_setup.sh,
# install_extensions.sh and whatever each person typed by hand. Those disagreed with
# each other, which is why "works on my machine" kept coming up.
#
# WORKSPACE ROOT. The repo is cloned *as* the colcon workspace's src/ directory:
#
#   f1tenth_ws/          <- workspace root: build/, install/, log/, venv/ live here
#   └── src/             <- THIS REPO (where this Makefile is)
#       └── src/         <- the ROS packages
#
# The root is therefore resolved from this file's own location, never from the
# caller's cwd, so `make`, `make -C src` and `make -f src/Makefile` all put build
# artifacts in the same place. (The old scripts resolved it from the caller and
# dropped build/, install/ and log/ *inside* the git tree.)

MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
REPO_DIR      := $(patsubst %/,%,$(dir $(MAKEFILE_PATH)))
WS_ROOT       := $(abspath $(REPO_DIR)/..)
SRC_DIR       := $(REPO_DIR)/src
VENV_DIR      := $(WS_ROOT)/venv

SHELL := /bin/bash
.SHELLFLAGS := -o pipefail -c

ROS_DISTRO ?= humble
ROS_SETUP  := /opt/ros/$(ROS_DISTRO)/setup.bash

# Sourced by every recipe, so a missing `source` can never be the cause of a
# failure again. ROS setup scripts reference unset variables, hence `set +u`.
SOURCE_ENV = set +u; \
	source $(ROS_SETUP); \
	if [ -f "$(VENV_DIR)/bin/activate" ]; then source "$(VENV_DIR)/bin/activate"; fi

CMAKE_BUILD_TYPE ?= RelWithDebInfo
PARALLEL_WORKERS ?= $(shell python3 -c "import math, os; print(math.ceil(os.cpu_count() ** 0.5) + 1)")

COLCON_BUILD_ARGS = \
	--base-paths $(SRC_DIR) \
	--executor parallel \
	--parallel-workers $(PARALLEL_WORKERS) \
	--continue-on-error \
	--symlink-install \
	--cmake-args -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) -DCMAKE_EXPORT_COMPILE_COMMANDS=On

# Packages skipped by a plain `make build`. These are the vendored LiDAR and ZED
# packages: they are slow, they need SDKs that are not on every machine, and
# nothing in day-to-day development touches them. `make build_all` builds them,
# and `make package zed_wrapper` builds one on demand.
PACKAGES_IGNORE ?= \
	livox_SDK2 \
	livox_ros_driver2 \
	zed_components \
	zed_ros2 \
	zed_wrapper

# Vendored third-party trees. Single source of truth for what `make lint` skips.
VENDORED_PATHS := \
	$(SRC_DIR)/hardware/f1tenth_system \
	$(SRC_DIR)/perception/livox_SDK2 \
	$(SRC_DIR)/perception/livox_ros_driver2 \
	$(SRC_DIR)/perception/zed_wrapper \
	$(SRC_DIR)/simulation

# find(1) prune expression built from VENDORED_PATHS, plus build output dirs.
FIND_PRUNE := $(foreach p,$(VENDORED_PATHS),-path '$(p)' -o ) \
	-name build -o -name install -o -name log -o -name .git

.PHONY: help deps build build_all package clean rebuild test lint lint-python lint-cpp \
        run_auto run_sim

help: ## Show this help
	@echo "Workspace root: $(WS_ROOT)"
	@echo "Sources:        $(SRC_DIR)"
	@echo
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'

deps: ## Full environment setup: ROS/system/rosdep deps plus the Python venv
	@$(REPO_DIR)/scripts/setup.sh

build: ## colcon build, minus PACKAGES_IGNORE (livox, zed)
	@$(SOURCE_ENV); \
	cd $(WS_ROOT) && colcon build $(COLCON_BUILD_ARGS) --packages-ignore $(PACKAGES_IGNORE)

build_all: ## colcon build, everything including the vendored LiDAR and ZED packages
	@$(SOURCE_ENV); \
	cd $(WS_ROOT) && colcon build $(COLCON_BUILD_ARGS)

# `make package pure_pursuit path_planning` -- the package names arrive as extra
# goals, so they are filtered out of MAKECMDGOALS and absorbed by the catch-all
# below. The catch-all is defined only when `package` is actually being run, so
# it cannot silently swallow a mistyped target in any other invocation.
PACKAGE_ARGS = $(filter-out package,$(MAKECMDGOALS))

package: ## Build only the named packages: make package <pkg> [<pkg>..]
	@if [ -z "$(PACKAGE_ARGS)" ]; then \
		echo "usage: make package <pkg> [<pkg>..]"; exit 2; \
	fi
	@$(SOURCE_ENV); \
	cd $(WS_ROOT) && colcon build $(COLCON_BUILD_ARGS) --packages-select $(PACKAGE_ARGS)

ifneq (,$(filter package,$(MAKECMDGOALS)))
$(PACKAGE_ARGS):
	@:
endif

clean: ## Remove build/, install/ and log/ from the workspace root
	@echo "Removing $(WS_ROOT)/{build,install,log}"
	@rm -rf $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/log

rebuild: clean build ## clean followed by build

test: ## Run colcon test and print the full results
	@$(SOURCE_ENV); \
	cd $(WS_ROOT) && colcon test --base-paths $(SRC_DIR) --packages-ignore $(PACKAGES_IGNORE) \
		&& colcon test-result --verbose

lint: lint-python lint-cpp ## Run every linter CI runs, over all of src/

lint-python: ## ruff check, ruff format --check and mypy over all first-party Python
	@$(SOURCE_ENV); \
	cd $(REPO_DIR) && \
	ruff check --config pyproject.toml src && \
	ruff format --check --config pyproject.toml src && \
	mypy --config-file pyproject.toml src

lint-cpp: ## clang-format and clang-tidy over all first-party C++
	@if [ ! -f "$(WS_ROOT)/build/compile_commands.json" ]; then \
		echo "No compile_commands.json in $(WS_ROOT)/build -- run 'make build' first."; \
		exit 2; \
	fi
	@$(SOURCE_ENV); \
	files=$$(find $(SRC_DIR) \( $(FIND_PRUNE) \) -prune -o \
		-type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' \) -print); \
	if [ -z "$$files" ]; then echo "No first-party C++ sources found."; exit 0; fi; \
	echo "$$files" | xargs clang-format --dry-run -Werror -style=file && \
	echo "$$files" | xargs clang-tidy -p $(WS_ROOT)/build --quiet

run_auto: ## Launch the autonomous stack on the car
	@$(SOURCE_ENV); \
	source $(WS_ROOT)/install/setup.bash; \
	ros2 launch launch_pkg fsae.launch.py config:=config.yaml

# CSV=<path> rewrites the track-dependent waypoint path in every config that
# carries one, so a track change is one argument rather than several hand edits
# that drift apart.
run_sim: ## Launch the simulator stack: make run_sim [CSV=<path to waypoints csv>]
	@$(SOURCE_ENV); \
	source $(WS_ROOT)/install/setup.bash; \
	if [ -n "$(CSV)" ]; then \
		csv=$$(cd "$$(dirname "$(CSV)")" && pwd -P)/$$(basename "$(CSV)"); \
		if [ ! -f "$$csv" ]; then echo "No such CSV: $$csv"; exit 2; fi; \
		echo "Pointing track-dependent configs at $$csv"; \
		grep -rl 'waypoints_path:' $(SRC_DIR) --include='*.yaml' \
			| xargs sed -i -E 's|(waypoints_path:).*|\1 "'"$$csv"'"|'; \
	fi; \
	ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml
