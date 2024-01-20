#!/usr/bin/env bash

echo -e "\nsource /opt/ros/humble/setup.bash" >>~/.bashrc
# echo "source /f1tenth/dev_ws/install/local_setup.bash" >> ~/.bashrc

export FORMULA_HOME=$(
	cd "$(dirname "${BASH_SOURCE[0]}")/.."
	pwd -P
)
alias rc_home="cd $FORMULA_HOME"

# unlimited bash history size
export HISTFILESIZE=
export HISTSIZE=

rc_clean() {
	cd $FORMULA_HOME/dev_ws && rm -rf build install log
}

pre-commit install
