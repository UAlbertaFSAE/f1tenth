#!/usr/bin/env bash

export FORMULA_HOME=$(
	cd "$(dirname "${BASH_SOURCE[0]}")/../.."
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
	# Make colcon builds reproducible regardless of user's ~/.colcon defaults.yaml
	# or user-site Python packages (~/.local) overriding system ROS tooling.
	export COLCON_DEFAULTS_FILE="$FORMULA_HOME/src/scripts/colcon_defaults.yaml"
	export PYTHONNOUSERSITE=1

	# ROS 2 Humble is built against the system Python (3.10). Some tools (e.g.
	# rosidl_generator_py) are invoked via '#!/usr/bin/env python3', so having
	# conda's python3 first in PATH will break message generation.
	if [[ -n "${CONDA_PREFIX:-}" ]]; then
		if declare -F conda >/dev/null 2>&1; then
			# Deactivate all active conda envs (best-effort).
			while [[ -n "${CONDA_PREFIX:-}" ]]; do
				conda deactivate || break
			done
		fi
		unset CONDA_PREFIX
		unset CONDA_DEFAULT_ENV
	fi

	# Avoid inheriting a user's PYTHONPATH (often points at stale install overlays
	# and can cause confusing imports during builds).
	unset PYTHONPATH
	unset PYTHONHOME

	# Ensure '/usr/bin/python3' wins for env-based shebangs.
	export PATH="/usr/bin:/bin:${PATH}"
	hash -r 2>/dev/null || true

	# Restore ROS environment variables (including PYTHONPATH) after sanitizing.
	source_ros_humble

	# Build with system Python (ROS tooling expects /usr/bin/python3 on Ubuntu).
	# If a venv is active, it can break ament/rosidl tooling (missing catkin_pkg, em, etc.).
	if [[ -n "${VIRTUAL_ENV:-}" ]]; then
		if declare -F deactivate >/dev/null 2>&1; then
			deactivate
		fi
	fi

	local sys_py="/usr/bin/python3"
	if [[ ! -x "$sys_py" ]]; then
		sys_py="python3"
	fi

	local args=(
		--executor parallel
		--parallel-workers $($sys_py -c "import math; print(math.ceil($(nproc) ** 0.5) + 1)")
		--continue-on-error
		--cmake-args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DPython3_EXECUTABLE=$sys_py"
		-Wall -Wextra -Wpedantic
	)

	# important: do not add commas between any new packages added here
	local packages_to_skip=(
		"safety_node"
		"zed_components"
		"zed_ros2"
		"zed_wrapper"
		"camera_detection" 
		"livox_ros_driver2" 
		"livox_sdk2"
	)

	# dont want to build zed ros tools if not on jetson
	if [ "$IS_JETSON" == "TRUE" ]; then
		cd $FORMULA_HOME && colcon build "${args[@]}" "$@"
	else
		cd $FORMULA_HOME && colcon build --packages-skip "${packages_to_skip[@]}" "${args[@]}" "$@"
	fi
}

# ----------------- all aliases go here ---------------------------------------
alias rc_all='rc_clean && rc_build && rc_source'
alias rc_run_auto='rc_source && ros2 launch launch_pkg fsae.launch.py config:=src/src/common/launch_pkg/config/config.yaml'
alias rc_run_sim='rc_source && ros2 launch launch_pkg fsae.launch.py config:=src/src/common/launch_pkg/config/sim_config.yaml'
alias launch_zed_wrapper='ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i ros_params_override_path:=src/perception/config/wrapper_params_override.yaml'

source_ros_humble() {
	if [[ ! -f /opt/ros/humble/setup.bash ]]; then
		return 0
	fi

	# ROS setup scripts may reference unset variables; with `set -u` this errors.
	local had_nounset=0
	case $- in
		*u*) had_nounset=1 ;;
	esac

	set +u
	# shellcheck disable=SC1091
	source /opt/ros/humble/setup.bash
	if [[ $had_nounset -eq 1 ]]; then
		set -u
	fi
}

source_ros_humble
