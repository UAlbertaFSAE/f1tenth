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

	# important: do not add commas between any new packages added here
	local packages_to_skip=(
		"safety_node"
		"zed_components"
		"zed_ros2"
		"zed_wrapper"
		"zed_interfaces"
		"rviz_plugin_zed_od"
		"zed_topic_benchmark"
		"zed_topic_benchmark_component"
		"zed_topic_benchmark_interfaces"
		"zed_display_rviz2"
		"zed_tutorial_video"
		"zed_robot_integration"
		"zed_rgb_convert"
		"zed_tutorial_pos_tracking"
		"zed_multi_camera"
		"zed_tutorial_depth"
		"zed_depth_to_laserscan"
		"zed_aruco_localization"
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
alias launch_zed_wrapper='ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i ros_params_override_path:=src/perception/config/wrapper_params_override.yaml'

source /opt/ros/humble/setup.bash
