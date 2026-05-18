source /opt/ros/humble/setup.bash

start_sim() {
    cd /sim_ws/
    source install/local_setup.bash

    # set up a tmux session that spins up simulator and attach to it
    tmux new-session -d -s sim_session

    # default it to bash shell and enable mouse scrolling
    tmux set-option -t sim_session default-shell /bin/bash
    tmux set-option -t sim_session mouse on

    # spin up simulator
    tmux send-keys -t sim_session:0 "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" C-m

    # attach to the session in current terminal
    tmux attach-session -t sim_session
}

run_sim() {
    export DISPLAY=${DISPLAY:-:0}

    source /opt/ros/humble/setup.bash
    source $FORMULA_HOME/install/setup.bash

    # set up a tmux session with two windows: sim launch + rviz
    tmux new-session -d -s rc_sim_session

    tmux set-option -t rc_sim_session default-shell /bin/bash
    tmux set-option -t rc_sim_session mouse on

    # window 0: run the sim launch file
    tmux send-keys -t rc_sim_session:0 \
        "bash -c 'source /opt/ros/humble/setup.bash && source $FORMULA_HOME/install/setup.bash && ros2 launch launch_pkg fsae.launch.py config:=sim_config.yaml'" C-m

    # window 1: run rviz2 once ros is up
    tmux new-window -t rc_sim_session
    tmux send-keys -t rc_sim_session:1 \
        "bash -c 'source /opt/ros/humble/setup.bash && sleep 5 && rviz2'" C-m

    # attach to window 0
    tmux select-window -t rc_sim_session:0
    tmux attach-session -t rc_sim_session
}
