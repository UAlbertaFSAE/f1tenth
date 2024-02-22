## Sim Workspace
This is solely to house the forked F1Tenth ROS-Gym bridge repository. BIG thanks to F1Tenth for creating such a resource!
New members will not have to develop inside this workspace, it's just for separating out concerns.

The f1tenth github can be found [here](https://github.com/f1tenth)

**Note:** To run keyboard operation of the simulator, you need to open a new terminal (outside of vscode) and run `docker exec -it docker-sim-1 /bin/bash` to enter into the simulator container. Once in (you'll know your in if it now says your the root user) you can run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to operate the simulated vehicle from the terminal

**Note:** If you are on windows and want the simulator to pop open, you will need to run `export DISPLAY=host.docker.internal:0.0"` when inside of the sim container (i.e. after running the `docker exec ...` command mentioned in the above note). Then you need to follow the commands below to rerun the simulator. (TODO: This is just a temporary fix until we get display mapping working properly)

If you ever need to close and reopen the sim, you will have to run the following commands in the sim docker container:
```
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
