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
