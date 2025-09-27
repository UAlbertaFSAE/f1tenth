# Simulator Info

We use the [F1Tenth ROS-Gym bridge](https://github.com/f1tenth/f1tenth_gym_ros) simulation environment to test some algorithms for our RC car. This file contains information for using the simulator.

### Simulator Start Up

1. open a new terminal (outside of the devcontainer, so in WSL if on windows or a regular new terminal on linux/mac)
2. run `docker exec -it docker-sim-1 /bin/bash` to enter into the simulator container
   - you'll know your in if it now says your the root user and your in the `/sim_ws` folder
3. run `start_sim` to start a tmux session that will spin up the simulator ROS node
4. go to http://localhost:8080/vnc.html in your browser to see the NoVNC client,press connect

you should see the rviz window with the little rc car in it. If you don't, let a lead know that something is up.

Additionally to vizualise cones and waypoint positions make sure to spin up the cones_viz node.
Once the node is spun up you can click "add" -> "add by topic" in the simulator and select the /visible_cones_viz and /published_waypoint markers.

**Note**: to operate the simulated vehicle from the terminal, open up a new tmux pane with `ctrl+b c` and run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`. The terminal will provide instructions on how to move the vehicle, and you must have the terminal as the active window in order for the keyboard inputs to be read.
