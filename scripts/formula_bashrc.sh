#!/usr/bin/env bash

export FORMULA_HOME=$(
	cd "$(dirname "${BASH_SOURCE[0]}")/.."
	pwd -P
)
alias rc_home="cd $FORMULA_HOME"

# unlimited bash history size
export HISTFILESIZE=
export HISTSIZE=

export AMENT_PREFIX_PATH=""
export CMAKE_PREFIX_PATH=""

rc_clean() {
	cd $FORMULA_HOME && rm -rf build install log
}

rc_source() {
	source $FORMULA_HOME/install/setup.bash
}

export CMAKE_BUILD_TYPE=RelWithDebInfo
rc_build() {
	local args=(
		--executor parallel
		--parallel-workers $(python3 -c "import math; print(math.ceil($(nproc) ** 0.5) + 1)")
		--continue-on-error
		--symlink-install
		--cmake-args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
		-Wall -Wextra -Wpedantic
	)
	cd $FORMULA_HOME && colcon build "${args[@]}" "$@"
}
alias rc_all='rc_clean && rc_build && rc_source'

source /opt/ros/humble/setup.bash